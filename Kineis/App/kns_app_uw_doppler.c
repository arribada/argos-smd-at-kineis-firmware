/**
 * @file    kns_app_uw_doppler.c
 * @brief   UW_DOPPLER application - Autonomous underwater/surface tracker
 *
 * @details
 * Main application for wildlife tracking tags. Monitors water conductivity via
 * a Salt Water Switch (SWS) and transmits satellite messages when the device
 * surfaces after a dive.
 *
 * State machine:
 *   BOOT -> BOOT_DEPLOY_LED -> INIT_MAC -> WAIT_MAC_READY -> MONITORING
 *                                                               |
 *                            SURFACE_TX -> WAIT_TX_DONE --------+
 *                               |
 *                       SHUTDOWN_BLINK (reed hold >10s)
 *
 * TX scheduling:
 *   - First TX is immediate upon surface detection
 *   - Subsequent TXs: T(n) = T_initial * (1 + growth/100)^n
 *   - Interval capped at T_max, count limited to max_count
 *   - Scheduling resets when device goes back underwater
 *
 * MAC profile compatibility:
 *   - BLIND (retx_nb=0): app handles scheduling, MAC sends once per request
 *   - BASIC: immediate single TX per request, same app-level scheduling
 *
 * Robustness:
 *   - IWDG watchdog (16s timeout) refreshed every loop iteration
 *   - State timeouts on all waiting states (configurable per state)
 *   - Error tracking via TAMP backup registers (survives resets)
 *   - Event logging in SRAM2 retention RAM (256 entries, survives resets)
 *   - NVM config persistence with CRC32 integrity
 *   - Reed switch shutdown cancellable by removing magnet during blink
 *
 * TX payload: 9 bytes [state(1), adc(2), tx_count(1), vbat_mV(2), reserved(3)]
 *
 * @see kns_app_uw_doppler.h
 * @see mgr_sws.h
 * @see mgr_nvm.h
 */

/**
 * @addtogroup KNS_APP_UW_DOPPLER
 * @{
 */

#include <stdbool.h>
#include "kns_app_uw_doppler.h"
#include "main.h"
#include "mgr_sws.h"
#include "mgr_nvm.h"
#include "mgr_wdg.h"
#include "mgr_err.h"
#include "mgr_evtlog.h"
#include "stm32wlxx_hal.h"
#include "kns_q.h"
#include "kns_mac.h"
#include "kns_cfg.h"
#include "kineis_sw_conf.h"
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"
#include "mgr_lpm.h"
#if defined(USE_UART_DRIVER)
#include "mgr_at_cmd.h"
#endif

#if defined(BSP_HAS_LED_RGB)
#include "mgr_led.h"
#endif
#if defined(BSP_HAS_REED_SWITCH)
#include "mgr_reed.h"
#endif
#if defined(BSP_HAS_VBAT_ADC)
#include "mgr_bat.h"
#endif

/* ---- State Machine ---- */

typedef enum {
	UW_DOPPLER_BOOT,
	UW_DOPPLER_BOOT_DEPLOY_LED,
	UW_DOPPLER_INIT_MAC,
	UW_DOPPLER_WAIT_MAC_READY,
	UW_DOPPLER_MONITORING,
	UW_DOPPLER_SURFACE_TX,
	UW_DOPPLER_WAIT_TX_DONE,
	UW_DOPPLER_SHUTDOWN_BLINK,
} UwDopplerState_t;

/* ---- Private variables ---- */

static __attribute__((__section__(".retentionRamData")))
UwDopplerState_t uw_doppler_state;

/* Exported for fault handlers (MGR_ERR_LOG_FAULT macro in stm32wlxx_it.c) */
volatile uint32_t g_uw_doppler_state_for_err = 0;

static KNS_APP_UwDopplerTxCfg_t tx_cfg = {
	.tx_initial_interval_s = 10,
	.tx_growth_percent     = 10,
	.tx_max_interval_s     = 600,
	.tx_max_count          = 0,  /* unlimited */
};

/* Deploy mode: 1 = deployed (TX enabled), 0 = not deployed (SWS runs but no TX) */
static uint8_t deploy_mode = 1;


/* TX scheduling state */
static uint32_t tx_count = 0;              /**< Number of TXs sent in current surface event */
static uint32_t last_tx_tick = 0;          /**< Tick of last TX */
static uint32_t current_interval_ms = 0;   /**< Current interval between TXs in ms */
static bool     surface_tx_pending = false; /**< Immediate TX needed on surface detection */

/* State timeout tracking */
static uint32_t state_enter_tick = 0;      /**< Tick when current state was entered */
static uint8_t  mac_init_retries = 0;      /**< MAC init retry counter */

#define TIMEOUT_BOOT_MS          10000  /**< Boot blink timeout */
#define TIMEOUT_BOOT_DEPLOY_MS   5000   /**< Deploy LED timeout */
#define TIMEOUT_MAC_READY_MS     30000  /**< Wait for MAC ready */
#define TIMEOUT_TX_DONE_MS       60000  /**< Wait for TX done */
#define TIMEOUT_SHUTDOWN_BLINK_MS 10000 /**< Shutdown blink timeout */
#define MAX_MAC_INIT_RETRIES     3      /**< Max MAC init retries before reset */

/* Reed switch shutdown tracking */
#if defined(BSP_HAS_REED_SWITCH)
#define REED_SHUTDOWN_HOLD_MS  10000  /**< Hold magnet 10s to trigger shutdown */
static uint32_t magnet_on_tick = 0;        /**< Tick when magnet was detected */
static bool     shutdown_triggered = false; /**< Shutdown sequence started */
#endif

/* Battery monitoring */
#if defined(BSP_HAS_VBAT_ADC)
static uint16_t last_vbat_mV = 0;          /**< Last battery voltage reading */
#endif

/* LPM client: prevent SHUTDOWN during UW_DOPPLER operation */
static enum MgrLpm_LPM_t uw_doppler_lpmReq(void)
{
	return LOW_POWER_MODE_STOP;
}

static bool uw_doppler_lpmNotifEnter(__attribute__((unused)) enum MgrLpm_LPM_t lpm)
{
	return true;
}

static bool uw_doppler_lpmNotifExit(__attribute__((unused)) enum MgrLpm_LPM_t lpm)
{
	return true;
}

static struct MgrLpmClientCb_t uw_doppler_lpm_client = {
	.fpMGR_LPM_LpmReqCb        = uw_doppler_lpmReq,
	.fpMGR_LPM_LpmNotifEnterCb = uw_doppler_lpmNotifEnter,
	.fpMGR_LPM_LpmNotifExitCb  = uw_doppler_lpmNotifExit,
};

#ifdef USE_MAC_PRFL_BLIND
static struct KNS_MAC_BLIND_usrCfg_t prflBlindUserCfg = {
	.retx_nb = 0,
	.retx_period_s = 60,
	.nb_parrallel_msg = 1
};
#endif

/* ---- State transition helper ---- */

static void transition_to(UwDopplerState_t new_state)
{
	uw_doppler_state = new_state;
	state_enter_tick = HAL_GetTick();
	g_uw_doppler_state_for_err = (uint32_t)new_state;
	MGR_EVTLOG_log(EVT_STATE_CHANGE, (uint16_t)new_state);
}

static uint32_t state_elapsed_ms(void)
{
	return HAL_GetTick() - state_enter_tick;
}

/* ---- Shutdown ---- */

#if defined(BSP_HAS_REED_SWITCH)
/**
 * @brief Enter SHUTDOWN mode with PWR_LATCH pulled LOW
 *
 * Saves config to NVM, releases power latch, configures internal
 * pull-down on PB7 to overcome external pull-up, then enters SHUTDOWN.
 * Board loses power entirely - next boot via hardware reed switch circuit.
 */
static void enter_shutdown(void)
{
	MGR_LOG_DEBUG("[UW_DPL] Entering SHUTDOWN...\r\n");
	MGR_EVTLOG_log(EVT_SHUTDOWN, 0);
	MGR_WDG_refresh();  /* Refresh before NVM save (flash write takes time) */
	MGR_NVM_save();

	/* Release power latch (drive LOW) */
	MGR_REED_releasePower();

	/* Configure internal pull-down on PWR_LATCH (PB7) to maintain LOW in SHUTDOWN */
	HAL_PWREx_EnablePullUpPullDownConfig();
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_7);

	/* Enter SHUTDOWN - never returns, board loses power */
	HAL_PWREx_EnterSHUTDOWNMode();
}
#endif

/* ---- Helpers ---- */

static uint32_t compute_next_interval_ms(uint32_t n)
{
	/* T_n = T_initial * (1 + growth/100)^n
	 * Computed iteratively to avoid floating point.
	 * Each step: interval = interval * (100 + growth) / 100
	 */
	uint32_t interval_ms = (uint32_t)tx_cfg.tx_initial_interval_s * 1000;
	uint32_t max_ms = (uint32_t)tx_cfg.tx_max_interval_s * 1000;
	for (uint32_t i = 0; i < n; i++) {
		uint64_t next = (uint64_t)interval_ms * (100 + tx_cfg.tx_growth_percent) / 100;
		if (next >= max_ms) {
			interval_ms = max_ms;
			break;
		}
		interval_ms = (uint32_t)next;
	}
	return interval_ms;
}

static void reset_tx_scheduling(void)
{
	tx_count = 0;
	last_tx_tick = 0;
	current_interval_ms = 0;
	surface_tx_pending = false;
}

static void build_tx_payload(struct KNS_MAC_appEvt_t *appEvt)
{
	struct KNS_CFG_radio_t device_radio_cfg;
	kns_assert(KNS_CFG_getRadioInfo(&device_radio_cfg) == KNS_STATUS_OK);

	appEvt->id = KNS_MAC_SEND_DATA;
	appEvt->data_ctxt.sf = KNS_SF_NO_SERVICE;

	/* Build payload: [state(1B)][adc(2B)][tx_count(1B)][vbat_mV(2B)] + padding */
	MGR_SWS_State_t state = MGR_SWS_getState();
	uint16_t adc = MGR_SWS_getLastADC();

	appEvt->data_ctxt.usrdata[0] = (uint8_t)state;
	appEvt->data_ctxt.usrdata[1] = (uint8_t)(adc >> 8);
	appEvt->data_ctxt.usrdata[2] = (uint8_t)(adc & 0xFF);
	appEvt->data_ctxt.usrdata[3] = (uint8_t)(tx_count & 0xFF);
#if defined(BSP_HAS_VBAT_ADC)
	appEvt->data_ctxt.usrdata[4] = (uint8_t)(last_vbat_mV >> 8);
	appEvt->data_ctxt.usrdata[5] = (uint8_t)(last_vbat_mV & 0xFF);
#else
	appEvt->data_ctxt.usrdata[4] = 0;
	appEvt->data_ctxt.usrdata[5] = 0;
#endif

	/* Fill rest with zeros */
	for (uint16_t i = 6; i < KNS_MAC_USRDATA_MAXLEN; i++)
		appEvt->data_ctxt.usrdata[i] = 0;

	/* Set bitlen based on modulation */
	switch (device_radio_cfg.modulation) {
	case KNS_TX_MOD_LDA2:
		appEvt->data_ctxt.usrdata_bitlen = 192;
		break;
	case KNS_TX_MOD_LDA2L:
		appEvt->data_ctxt.usrdata_bitlen = 196;
		break;
	case KNS_TX_MOD_VLDA4:
		appEvt->data_ctxt.usrdata_bitlen = 24;
		break;
	case KNS_TX_MOD_LDK:
		appEvt->data_ctxt.usrdata_bitlen = 128;
		break;
	default:
		kns_assert(0);
		break;
	}
}

static void start_mac_profile(void)
{
	enum KNS_status_t status;
	struct KNS_MAC_appEvt_t appEvt;

	appEvt.id = KNS_MAC_INIT;
	appEvt.init_prfl_ctxt.id = KNS_MAC_PRFL_NONE;
#ifdef USE_MAC_PRFL_BASIC
	appEvt.init_prfl_ctxt.id = KNS_MAC_PRFL_BASIC;
	MGR_LOG_DEBUG("[UW_DPL] MAC profile: BASIC\r\n");
#endif
#ifdef USE_MAC_PRFL_BLIND
	appEvt.init_prfl_ctxt.id = KNS_MAC_PRFL_BLIND;
	appEvt.init_prfl_ctxt.blindCfg = prflBlindUserCfg;
	MGR_LOG_DEBUG("[UW_DPL] MAC profile: BLIND\r\n");
#endif

	status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
	if (status != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[UW_DPL] MAC init push failed: 0x%x\r\n", status);
		/* Don't assert — let timeout handle retry */
	}
}

static bool process_mac_events(void)
{
	enum KNS_status_t status;
	struct KNS_MAC_srvcEvt_t srvcEvt;
	bool got_event = false;

	for (status = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&srvcEvt);
	     status != KNS_STATUS_QEMPTY;
	     status = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&srvcEvt)) {
		if (status != KNS_STATUS_OK)
			continue;

		got_event = true;

		switch (srvcEvt.id) {
		case KNS_MAC_OK:
			if (uw_doppler_state == UW_DOPPLER_WAIT_MAC_READY) {
				MGR_LOG_DEBUG("[UW_DPL] MAC ready\r\n");
				MGR_EVTLOG_log(EVT_MAC_READY, 0);
				mac_init_retries = 0;
				transition_to(UW_DOPPLER_MONITORING);
			} else if (srvcEvt.app_evt == KNS_MAC_SEND_DATA) {
				MGR_LOG_DEBUG("[UW_DPL] TX accepted\r\n");
			}
			break;

		case KNS_MAC_TX_DONE:
			MGR_LOG_DEBUG("[UW_DPL] TX done (#%lu)\r\n", tx_count);
			MGR_EVTLOG_log(EVT_TX_DONE, 0);
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE)
				transition_to(UW_DOPPLER_MONITORING);
			break;

		case KNS_MAC_TX_TIMEOUT:
			MGR_LOG_DEBUG("[UW_DPL] TX timeout\r\n");
			MGR_EVTLOG_log(EVT_TX_TIMEOUT, 0);
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE)
				transition_to(UW_DOPPLER_MONITORING);
			break;

		case KNS_MAC_ERROR:
			MGR_LOG_DEBUG("[UW_DPL] MAC error: %d\r\n", srvcEvt.status);
			MGR_EVTLOG_log(EVT_MAC_ERROR, (uint16_t)srvcEvt.status);
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE)
				transition_to(UW_DOPPLER_MONITORING);
			else if (uw_doppler_state == UW_DOPPLER_WAIT_MAC_READY)
				transition_to(UW_DOPPLER_INIT_MAC); /* retry */
			break;

		default:
			break;
		}
	}

	return got_event;
}

/* ---- Public API ---- */

void KNS_APP_uw_doppler_init(void)
{
	reset_tx_scheduling();
	mac_init_retries = 0;
	state_enter_tick = HAL_GetTick();

	/* Init event log (SRAM2 retention - survives resets) */
	MGR_EVTLOG_init();
	MGR_EVTLOG_log(EVT_BOOT, 0);

	/* Load saved config from flash (if valid) */
	MGR_NVM_load();

	MGR_LOG_DEBUG("[UW_DPL] Init: interval=%us growth=%u%% max=%us deploy=%u\r\n",
		tx_cfg.tx_initial_interval_s, tx_cfg.tx_growth_percent,
		tx_cfg.tx_max_interval_s, deploy_mode);

	/* Register LPM client to prevent SHUTDOWN during operation */
	MGR_LPM_registerClient(uw_doppler_lpm_client);

#if defined(BSP_HAS_REED_SWITCH)
	MGR_REED_latchPower();
	MGR_LOG_DEBUG("[REED] Init: magnet=%u\r\n", (unsigned)MGR_REED_isMagnetPresent());
#endif
#if defined(BSP_HAS_VBAT_ADC)
	MGR_BAT_init();
	last_vbat_mV = MGR_BAT_readVoltage_mV();
	MGR_LOG_DEBUG("[BAT] Init: %umV\r\n", last_vbat_mV);
#endif

	/* Start IWDG watchdog (16s timeout) */
	MGR_WDG_init();

#if defined(BSP_HAS_LED_RGB)
	/* Boot sequence: blink 10x */
	MGR_LED_blink(MGR_LED_BLUE, 10, 200, 200);
	transition_to(UW_DOPPLER_BOOT);
#else
	transition_to(UW_DOPPLER_INIT_MAC);
#endif
}

void KNS_APP_uw_doppler_loop(void)
{
	/* Refresh watchdog every loop iteration */
	MGR_WDG_refresh();

	/* Process pending AT commands */
#if defined(USE_UART_DRIVER)
	{
		uint8_t *pu8_atcmd = MGR_AT_CMD_popNextAt();
		if (pu8_atcmd != NULL)
			MGR_AT_CMD_decodeAt(pu8_atcmd);
	}
#endif

#if defined(BSP_HAS_LED_RGB)
	MGR_LED_task();
#endif

	/* Reed switch events (EXTI interrupt driven) */
#if defined(BSP_HAS_REED_SWITCH)
	{
		MGR_REED_Event_t evt = MGR_REED_getEvent();
		if (evt == MGR_REED_EVT_MAGNET_ON) {
			MGR_LOG_DEBUG("[REED] Magnet DETECTED\r\n");
			MGR_EVTLOG_log(EVT_REED_ON, 0);
			magnet_on_tick = HAL_GetTick();
			shutdown_triggered = false;
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_set(MGR_LED_WHITE);
#endif
		} else if (evt == MGR_REED_EVT_MAGNET_OFF) {
			uint32_t hold_ms = MGR_REED_getLastHoldDuration_ms();
			MGR_LOG_DEBUG("[REED] Magnet REMOVED (hold=%lums)\r\n",
				(unsigned long)hold_ms);
			MGR_EVTLOG_log(EVT_REED_OFF, (uint16_t)(hold_ms / 100));
			magnet_on_tick = 0;
			shutdown_triggered = false;
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
		}

		/* Check for long hold → shutdown */
		if (MGR_REED_isMagnetPresent() && !shutdown_triggered && magnet_on_tick > 0) {
			uint32_t hold_ms = HAL_GetTick() - magnet_on_tick;
			if (hold_ms >= REED_SHUTDOWN_HOLD_MS) {
				MGR_LOG_DEBUG("[REED] Shutdown hold detected (%lums)\r\n",
					(unsigned long)hold_ms);
				shutdown_triggered = true;
#if defined(BSP_HAS_LED_RGB)
				MGR_LED_blink(MGR_LED_WHITE, 10, 200, 200);
#endif
				transition_to(UW_DOPPLER_SHUTDOWN_BLINK);
				return;
			}
		}
	}
#endif

	/* Run SWS measurement after boot */
	if (uw_doppler_state >= UW_DOPPLER_MONITORING)
		MGR_SWS_task();

	switch (uw_doppler_state) {
#if defined(BSP_HAS_LED_RGB)
	case UW_DOPPLER_BOOT:
		/* Wait for boot blink to finish (with timeout) */
		if (MGR_LED_isBlinkDone() || state_elapsed_ms() > TIMEOUT_BOOT_MS) {
			if (state_elapsed_ms() > TIMEOUT_BOOT_MS)
				MGR_LOG_DEBUG("[UW_DPL] Boot blink timeout, skipping\r\n");
			/* Show deploy status: green=deployed, blue=not deployed */
			if (deploy_mode)
				MGR_LED_set(MGR_LED_GREEN);
			else
				MGR_LED_set(MGR_LED_BLUE);
			transition_to(UW_DOPPLER_BOOT_DEPLOY_LED);
		}
		return;

	case UW_DOPPLER_BOOT_DEPLOY_LED:
		/* Show deploy color for 2s then proceed (with timeout) */
		if (state_elapsed_ms() >= 2000 || state_elapsed_ms() > TIMEOUT_BOOT_DEPLOY_MS) {
			MGR_LED_off();
			transition_to(UW_DOPPLER_INIT_MAC);
		}
		return;
#endif

	case UW_DOPPLER_INIT_MAC:
		start_mac_profile();
		transition_to(UW_DOPPLER_WAIT_MAC_READY);
		return;

	case UW_DOPPLER_WAIT_MAC_READY:
		process_mac_events();
		/* Timeout protection */
		if (uw_doppler_state == UW_DOPPLER_WAIT_MAC_READY &&
		    state_elapsed_ms() > TIMEOUT_MAC_READY_MS) {
			MGR_LOG_DEBUG("[UW_DPL] MAC ready timeout (retry %u/%u)\r\n",
				mac_init_retries + 1, MAX_MAC_INIT_RETRIES);
			MGR_EVTLOG_log(EVT_TIMEOUT, (uint16_t)UW_DOPPLER_WAIT_MAC_READY);
			MGR_ERR_log(ERR_MAC_TIMEOUT);
			if (++mac_init_retries >= MAX_MAC_INIT_RETRIES) {
				MGR_LOG_DEBUG("[UW_DPL] MAC init failed after %u retries, resetting\r\n",
					MAX_MAC_INIT_RETRIES);
				MGR_EVTLOG_log(EVT_ERROR, (uint16_t)ERR_MAC_INIT_FAIL);
				MGR_ERR_logAndReset(ERR_MAC_INIT_FAIL);
				/* Never reaches here */
			}
			transition_to(UW_DOPPLER_INIT_MAC);
		}
		return;

	case UW_DOPPLER_MONITORING:
	{
		/* Process any pending MAC events */
		process_mac_events();

		MGR_SWS_State_t sws_state = MGR_SWS_getState();

		/* Check for surface detection */
		if (MGR_SWS_stateChanged()) {
			if (sws_state == MGR_SWS_STATE_SURFACE) {
				/* Surface detected! Schedule immediate TX */
				MGR_LOG_DEBUG("[UW_DPL] Surface detected, starting TX\r\n");
				MGR_EVTLOG_log(EVT_SWS_SURFACE, MGR_SWS_getLastADC());
				reset_tx_scheduling();
				surface_tx_pending = true;
#if defined(BSP_HAS_LED_RGB)
				MGR_LED_blink(MGR_LED_CYAN, 3, 200, 200);
#endif
			} else {
				/* Went underwater, stop TX scheduling */
				MGR_LOG_DEBUG("[UW_DPL] Underwater, stopping TX\r\n");
				MGR_EVTLOG_log(EVT_SWS_UNDERWATER, MGR_SWS_getLastADC());
				reset_tx_scheduling();
#if defined(BSP_HAS_LED_RGB)
				MGR_LED_blink(MGR_LED_YELLOW, 3, 200, 200);
#endif
			}
		}

		/* TX scheduling logic - only TX if deployed */
		if (deploy_mode && sws_state == MGR_SWS_STATE_SURFACE) {
			bool should_tx = false;

			if (surface_tx_pending) {
				/* Immediate first TX */
				should_tx = true;
				surface_tx_pending = false;
			} else if (tx_count > 0 && current_interval_ms > 0) {
				/* Check if interval has elapsed */
				uint32_t elapsed = HAL_GetTick() - last_tx_tick;
				if (elapsed >= current_interval_ms)
					should_tx = true;
			}

			/* Check max TX count */
			if (should_tx && tx_cfg.tx_max_count > 0 && tx_count >= tx_cfg.tx_max_count)
				should_tx = false;

			if (should_tx) {
#if defined(BSP_HAS_VBAT_ADC)
				last_vbat_mV = MGR_BAT_readVoltage_mV();
				MGR_EVTLOG_log(EVT_BAT, last_vbat_mV);
#endif
				MGR_EVTLOG_log(EVT_TX_START, (uint16_t)tx_count);
				transition_to(UW_DOPPLER_SURFACE_TX);
				/* Fall through to SURFACE_TX */
			} else {
				return;
			}
		} else {
			return;
		}
	}
	/* Fall through */

	case UW_DOPPLER_SURFACE_TX:
	{
#if defined(BSP_HAS_LED_RGB)
		MGR_LED_set(MGR_LED_VIOLET);
#endif
		struct KNS_MAC_appEvt_t appEvt;
		build_tx_payload(&appEvt);

		enum KNS_status_t status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
		if (status == KNS_STATUS_OK) {
			MGR_LOG_DEBUG("[UW_DPL] TX #%lu sent (interval=%lums)\r\n",
				tx_count, current_interval_ms);

			last_tx_tick = HAL_GetTick();
			current_interval_ms = compute_next_interval_ms(tx_count);
			tx_count++;

			transition_to(UW_DOPPLER_WAIT_TX_DONE);
		} else {
			MGR_LOG_DEBUG("[UW_DPL] TX push failed: 0x%x\r\n", status);
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			transition_to(UW_DOPPLER_MONITORING);
		}
		return;
	}

	case UW_DOPPLER_WAIT_TX_DONE:
		process_mac_events();
		if (uw_doppler_state == UW_DOPPLER_MONITORING) {
			/* TX complete, turn off TX LED */
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
		}
		/* Timeout protection */
		if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE &&
		    state_elapsed_ms() > TIMEOUT_TX_DONE_MS) {
			MGR_LOG_DEBUG("[UW_DPL] TX done timeout\r\n");
			MGR_EVTLOG_log(EVT_TIMEOUT, (uint16_t)UW_DOPPLER_WAIT_TX_DONE);
			MGR_ERR_log(ERR_TX_TIMEOUT);
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			transition_to(UW_DOPPLER_MONITORING);
		}
		return;

#if defined(BSP_HAS_REED_SWITCH)
	case UW_DOPPLER_SHUTDOWN_BLINK:
		/* Cancel shutdown if magnet removed during blink */
		if (!MGR_REED_isMagnetPresent()) {
			MGR_LOG_DEBUG("[UW_DPL] Shutdown cancelled (magnet removed)\r\n");
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			shutdown_triggered = false;
			transition_to(UW_DOPPLER_MONITORING);
			return;
		}
		/* Wait for blink to finish or timeout */
#if defined(BSP_HAS_LED_RGB)
		if (MGR_LED_isBlinkDone() || state_elapsed_ms() > TIMEOUT_SHUTDOWN_BLINK_MS)
#else
		if (state_elapsed_ms() > TIMEOUT_SHUTDOWN_BLINK_MS)
#endif
		{
			enter_shutdown();
			/* Never reaches here - board loses power */
		}
		return;
#endif

	default:
		MGR_LOG_DEBUG("[UW_DPL] Invalid state %u, resetting to INIT_MAC\r\n",
			uw_doppler_state);
		transition_to(UW_DOPPLER_INIT_MAC);
		break;
	}
}

KNS_APP_UwDopplerTxCfg_t KNS_APP_uw_doppler_getTxCfg(void)
{
	return tx_cfg;
}

void KNS_APP_uw_doppler_setTxCfg(const KNS_APP_UwDopplerTxCfg_t *cfg)
{
	if (cfg) {
		tx_cfg = *cfg;
		MGR_LOG_DEBUG("[UW_DPL] TX config: interval=%us growth=%u%% max=%us count=%u\r\n",
			tx_cfg.tx_initial_interval_s, tx_cfg.tx_growth_percent,
			tx_cfg.tx_max_interval_s, tx_cfg.tx_max_count);
	}
}

uint8_t KNS_APP_uw_doppler_getDeployMode(void)
{
	return deploy_mode;
}

void KNS_APP_uw_doppler_setDeployMode(uint8_t mode)
{
	deploy_mode = mode ? 1 : 0;
	MGR_LOG_DEBUG("[UW_DPL] Deploy mode: %u\r\n", deploy_mode);
}

/**
 * @}
 */
