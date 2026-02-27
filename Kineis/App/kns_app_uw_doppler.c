/**
 * @file    kns_app_uw_doppler.c
 * @brief   UW_DOPPLER application - SWS monitoring with BLIND-DOPPLER TX scheduling
 *
 * State machine:
 *   BOOT → INIT_MAC → WAIT_MAC_READY → MONITORING → SURFACE_TX → WAIT_TX_DONE → MONITORING
 *
 * TX scheduling (BLIND-DOPPLER):
 *   On surface detection, first TX is immediate.
 *   Subsequent TXs use incremental intervals: T_n = T_initial * (1 + growth/100)^n
 *   Resets when going back underwater.
 */

#include <stdbool.h>
#include <stdlib.h>
#include "kns_app_uw_doppler.h"
#include "main.h"
#include "mgr_sws.h"
#include "stm32wlxx_hal.h"
#include "kns_q.h"
#include "kns_mac.h"
#include "kns_cfg.h"
#include "kineis_sw_conf.h"
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"

#if defined(BSP_HAS_LED_RGB)
#include "mgr_led.h"
#endif
#if defined(BSP_HAS_REED_SWITCH)
#include "mgr_reed.h"
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
} UwDopplerState_t;

/* ---- Private variables ---- */

static __attribute__((__section__(".retentionRamData")))
UwDopplerState_t uw_doppler_state;

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
#if defined(BSP_HAS_LED_RGB)
static uint32_t boot_tick = 0;             /**< Tick for boot LED sequence timing */
#endif

#ifdef USE_MAC_PRFL_BLIND
static struct KNS_MAC_BLIND_usrCfg_t prflBlindUserCfg = {
	.retx_nb = 0,
	.retx_period_s = 60,
	.nb_parrallel_msg = 1
};
#endif

/* ---- Helpers ---- */

static uint32_t compute_next_interval_ms(uint32_t n)
{
	/* T_n = T_initial * (1 + growth/100)^n
	 * Computed iteratively to avoid floating point.
	 * Each step: interval = interval * (100 + growth) / 100
	 */
	uint32_t interval_ms = (uint32_t)tx_cfg.tx_initial_interval_s * 1000;
	for (uint32_t i = 0; i < n; i++) {
		interval_ms = interval_ms * (100 + tx_cfg.tx_growth_percent) / 100;
		/* Cap to max */
		if (interval_ms > (uint32_t)tx_cfg.tx_max_interval_s * 1000) {
			interval_ms = (uint32_t)tx_cfg.tx_max_interval_s * 1000;
			break;
		}
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

	/* Build payload: [state(1B)][adc(2B)][tx_count(1B)] + padding */
	MGR_SWS_State_t state = MGR_SWS_getState();
	uint16_t adc = MGR_SWS_getLastADC();

	appEvt->data_ctxt.usrdata[0] = (uint8_t)state;
	appEvt->data_ctxt.usrdata[1] = (uint8_t)(adc >> 8);
	appEvt->data_ctxt.usrdata[2] = (uint8_t)(adc & 0xFF);
	appEvt->data_ctxt.usrdata[3] = (uint8_t)(tx_count & 0xFF);

	/* Fill rest with zeros */
	for (uint16_t i = 4; i < KNS_MAC_USRDATA_MAXLEN; i++)
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
		MGR_LOG_DEBUG("[UW_DPL] MAC init failed: 0x%x\r\n", status);
		kns_assert(0);
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
				uw_doppler_state = UW_DOPPLER_MONITORING;
			} else if (srvcEvt.app_evt == KNS_MAC_SEND_DATA) {
				MGR_LOG_DEBUG("[UW_DPL] TX accepted\r\n");
			}
			break;

		case KNS_MAC_TX_DONE:
			MGR_LOG_DEBUG("[UW_DPL] TX done (#%lu)\r\n", tx_count);
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE)
				uw_doppler_state = UW_DOPPLER_MONITORING;
			break;

		case KNS_MAC_TX_TIMEOUT:
			MGR_LOG_DEBUG("[UW_DPL] TX timeout\r\n");
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE)
				uw_doppler_state = UW_DOPPLER_MONITORING;
			break;

		case KNS_MAC_ERROR:
			MGR_LOG_DEBUG("[UW_DPL] MAC error: %d\r\n", srvcEvt.status);
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE)
				uw_doppler_state = UW_DOPPLER_MONITORING;
			else if (uw_doppler_state == UW_DOPPLER_WAIT_MAC_READY)
				uw_doppler_state = UW_DOPPLER_INIT_MAC; /* retry */
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
	MGR_LOG_DEBUG("[UW_DPL] Init: interval=%us growth=%u%% max=%us deploy=%u\r\n",
		tx_cfg.tx_initial_interval_s, tx_cfg.tx_growth_percent,
		tx_cfg.tx_max_interval_s, deploy_mode);

#if defined(BSP_HAS_REED_SWITCH)
	MGR_REED_latchPower();
	MGR_LOG_DEBUG("[REED] Init: magnet=%u\r\n", (unsigned)MGR_REED_isMagnetPresent());
#endif
#if defined(BSP_HAS_LED_RGB)
	/* Diagnostic: test each LED pin individually */
	MGR_LED_bootTest();
	/* Boot sequence: white blink 10x */
	MGR_LED_blink(MGR_LED_WHITE, 10, 200, 200);
	uw_doppler_state = UW_DOPPLER_BOOT;
#else
	uw_doppler_state = UW_DOPPLER_INIT_MAC;
#endif
}

void KNS_APP_uw_doppler_loop(void)
{
#if defined(BSP_HAS_LED_RGB)
	MGR_LED_task();
#endif

	/* Reed switch events (EXTI interrupt driven) */
#if defined(BSP_HAS_REED_SWITCH)
	{
		MGR_REED_Event_t evt = MGR_REED_getEvent();
		if (evt == MGR_REED_EVT_MAGNET_ON) {
			MGR_LOG_DEBUG("[REED] Magnet DETECTED\r\n");
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_set(MGR_LED_WHITE);
#endif
		} else if (evt == MGR_REED_EVT_MAGNET_OFF) {
			uint32_t hold_ms = MGR_REED_getLastHoldDuration_ms();
			MGR_LOG_DEBUG("[REED] Magnet REMOVED (hold=%lums)\r\n",
				(unsigned long)hold_ms);
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			/* TODO: action mapping based on hold duration
			 * e.g. < 2s = toggle deploy, > 5s = power off, etc.
			 */
		}
	}
#endif

	/* Run SWS measurement after boot */
	if (uw_doppler_state >= UW_DOPPLER_MONITORING)
		MGR_SWS_task();

	switch (uw_doppler_state) {
#if defined(BSP_HAS_LED_RGB)
	case UW_DOPPLER_BOOT:
		/* Wait for boot blink to finish */
		if (MGR_LED_isBlinkDone()) {
			/* Show deploy status: green=deployed, blue=not deployed */
			if (deploy_mode)
				MGR_LED_set(MGR_LED_GREEN);
			else
				MGR_LED_set(MGR_LED_BLUE);
			boot_tick = HAL_GetTick();
			uw_doppler_state = UW_DOPPLER_BOOT_DEPLOY_LED;
		}
		return;

	case UW_DOPPLER_BOOT_DEPLOY_LED:
		/* Show deploy color for 2s then proceed */
		if ((HAL_GetTick() - boot_tick) >= 2000) {
			MGR_LED_off();
			uw_doppler_state = UW_DOPPLER_INIT_MAC;
		}
		return;
#endif

	case UW_DOPPLER_INIT_MAC:
		start_mac_profile();
		uw_doppler_state = UW_DOPPLER_WAIT_MAC_READY;
		return;

	case UW_DOPPLER_WAIT_MAC_READY:
		process_mac_events();
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
				reset_tx_scheduling();
				surface_tx_pending = true;
#if defined(BSP_HAS_LED_RGB)
				MGR_LED_blink(MGR_LED_CYAN, 3, 200, 200);
#endif
			} else {
				/* Went underwater, stop TX scheduling */
				MGR_LOG_DEBUG("[UW_DPL] Underwater, stopping TX\r\n");
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
				uw_doppler_state = UW_DOPPLER_SURFACE_TX;
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

			uw_doppler_state = UW_DOPPLER_WAIT_TX_DONE;
		} else {
			MGR_LOG_DEBUG("[UW_DPL] TX push failed: 0x%x\r\n", status);
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			uw_doppler_state = UW_DOPPLER_MONITORING;
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
		return;

	default:
		uw_doppler_state = UW_DOPPLER_INIT_MAC;
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
