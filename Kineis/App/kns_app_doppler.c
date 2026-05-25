/**
 * @file    kns_app_doppler.c
 * @brief   DOPPLER application - Periodic satellite message transmitter
 *
 * @details
 * Simple periodic transmitter for satellite messages. No ADC/SWS.
 *
 * State machine:
 *   BOOT -> CHECK_SCHEDULE -> INIT_MAC -> WAIT_MAC_READY -> SEND_MSG
 *                |                                            |
 *           (skip: MCU_DONE pulse)              WAIT_TX_DONE -+-> WAIT_INTERVAL
 *                                                                    |
 *                                               (msg_count done) -> SEQUENCE_DONE
 *
 * Two operating modes:
 *   - MCU_DONE defined: TPL5111 power-cycles board, modulo scheduling via flash wku counter
 *   - MCU_DONE not defined: RTC wakeup timer between sequences, SHUTDOWN sleep
 *
 * MAC profile behavior:
 *   - BASIC: app sends each message individually with timer between each TX
 *   - BLIND: MAC handles retransmissions (retx_nb=msg_count, retx_period=msg_interval)
 *
 * @see kns_app_doppler.h
 */

/**
 * @addtogroup KNS_APP_DOPPLER
 * @{
 */

#include <stdbool.h>
#include <string.h>
#include "kns_app_doppler.h"
#include "main.h"
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
#include "lpm.h"
#include "mcu_flash.h"
#include "rtc.h"
#include "usart.h"
#if defined(USE_UART_DRIVER)
#include "mgr_at_cmd.h"
#endif

#if defined(BSP_HAS_LED_RGB)
#include "mgr_led.h"
#endif
#if defined(BSP_HAS_VBAT_ADC)
#include "mgr_bat.h"
#endif

/* Compile-time safety: DOPPLER without TPL5111 uses SHUTDOWN via RTC wakeup.
 * The LPM framework must have SHUTDOWN support enabled. */
#if !defined(MCU_DONE_Pin) && !defined(LPM_SHUTDOWN_ENABLED)
#error "DOPPLER without MCU_DONE requires LPM=SHUTDOWN (add -DLPM_SHUTDOWN_ENABLED)"
#endif

/* ---- State Machine ---- */

typedef enum {
	DOPPLER_BOOT,
	DOPPLER_CHECK_SCHEDULE,
	DOPPLER_INIT_MAC,
	DOPPLER_WAIT_MAC_READY,
	DOPPLER_SEND_MSG,
	DOPPLER_WAIT_TX_DONE,
	DOPPLER_WAIT_INTERVAL,
	DOPPLER_SEQUENCE_DONE,
} DopplerState_t;

/* ---- Private variables ---- */

static DopplerState_t doppler_state;

/* Exported for fault handlers (mgr_err.c and mgr_evtlog.c reference this name) */
volatile uint32_t g_uw_doppler_state_for_err = 0;

static KNS_APP_DopplerCfg_t doppler_cfg = {
	.msg_count           = 3,
	.msg_interval_s      = 60,
	.sequence_interval_s = 14400,  /* 4 hours */
	.tpl_interval_s      = 0,     /* 0 = no TPL5111 */
};

/* TX state within current sequence */
static uint8_t  tx_index = 0;       /**< Current message index in sequence */
static uint32_t last_tx_tick = 0;   /**< Tick of last TX */

/* State timeout tracking */
static uint32_t state_enter_tick = 0;
static uint8_t  mac_init_retries = 0;

#define TIMEOUT_BOOT_MS          3000
#define TIMEOUT_MAC_READY_MS     30000
#define TIMEOUT_TX_DONE_MS       60000
#define MAX_MAC_INIT_RETRIES     3

/* Max msg_interval_s before uint32 overflow on *1000 conversion */
#define MSG_INTERVAL_MAX_S       (UINT32_MAX / 1000UL)

/* Max RTC wakeup timer period in seconds (17-bit mode) */
#define RTC_WAKEUP_MAX_S         0x20000UL  /* 131072s ≈ 36h */

/* WDG refresh interval for long AT log dumps */
#define LOG_WDG_REFRESH_ENTRIES  32

/* Battery monitoring */
#if defined(BSP_HAS_VBAT_ADC)
static uint16_t last_vbat_mV = 0;
#endif

/* LPM mode tracking: STOP during sequence, SHUTDOWN after */
static enum MgrLpm_LPM_t doppler_lpm_mode = LOW_POWER_MODE_STOP;

/* Deploy mode: 1 = deployed (UART off after boot window), 0 = config mode */
static uint8_t deploy_mode = 0;

/* Boot window: AT command received during boot window keeps UART active */
static bool boot_window_at_received = false;

#define BOOT_WINDOW_MS  5000  /**< UART listen window at boot (5s) */

/* NVM dirty flag: only write flash when config has actually changed */
static bool nvm_dirty = false;

/* ---- NVM config (simple flash storage, after WKU counter region) ---- */

#define DOPPLER_NVM_MAGIC   0x44504C52UL  /* "DPLR" */
#define DOPPLER_NVM_VERSION 3

typedef struct {
	uint32_t magic;
	uint8_t  version;
	uint8_t  msg_count;
	uint8_t  led_mode;     /**< MGR_LED_Mode_t: 0=off, 1=on, 2=24h */
	uint8_t  deployed;     /**< 1=deployed (UART off after boot window), 0=config mode */
	uint32_t msg_interval_s;
	uint32_t sequence_interval_s;
	uint32_t tpl_interval_s;
	uint32_t crc32;
} DopplerNvmConfig_t;

/**
 * @brief CRC-32/MPEG-2 (same algorithm as mgr_nvm.c and STM32 HW CRC unit)
 *
 * Polynomial: 0x04C11DB7 (MSB-first, no reflection)
 * Initial value: 0xFFFFFFFF, Final XOR: none
 *
 * Note: Identical to nvm_crc32() in mgr_nvm.c. Duplicated here because
 * only one APP is compiled at a time (mutual exclusion in Makefile).
 */
static uint32_t nvm_crc32(const void *data, size_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	uint32_t crc = 0xFFFFFFFFUL;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint32_t)p[i] << 24;
		for (int bit = 0; bit < 8; bit++) {
			if (crc & 0x80000000UL)
				crc = (crc << 1) ^ 0x04C11DB7UL;
			else
				crc <<= 1;
		}
	}
	return crc;
}

static bool nvm_load(void)
{
	DopplerNvmConfig_t cfg;

	if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK)
		return false;

	if (cfg.magic != DOPPLER_NVM_MAGIC)
		return false;

	/* Accept v2 (no deploy field) or v3 (with deploy field) */
	if (cfg.version != DOPPLER_NVM_VERSION && cfg.version != 2)
		return false;

	uint32_t computed = nvm_crc32(&cfg, offsetof(DopplerNvmConfig_t, crc32));
	if (computed != cfg.crc32) {
		MGR_LOG_DEBUG("[DPL] NVM CRC mismatch\r\n");
		return false;
	}

	/* Validate loaded values before applying */
	if (cfg.msg_count == 0 || cfg.msg_interval_s == 0 || cfg.sequence_interval_s == 0) {
		MGR_LOG_DEBUG("[DPL] NVM invalid params (count=%u int=%lu seq=%lu)\r\n",
			cfg.msg_count, (unsigned long)cfg.msg_interval_s,
			(unsigned long)cfg.sequence_interval_s);
		return false;
	}
	if (cfg.msg_interval_s > MSG_INTERVAL_MAX_S) {
		MGR_LOG_DEBUG("[DPL] NVM msg_interval too large (%lu)\r\n",
			(unsigned long)cfg.msg_interval_s);
		return false;
	}
#if !defined(MCU_DONE_Pin)
	if (cfg.sequence_interval_s > RTC_WAKEUP_MAX_S) {
		MGR_LOG_DEBUG("[DPL] NVM seq_interval too large (%lu), capped\r\n",
			(unsigned long)cfg.sequence_interval_s);
		cfg.sequence_interval_s = RTC_WAKEUP_MAX_S;
	}
#endif

	doppler_cfg.msg_count           = cfg.msg_count;
	doppler_cfg.msg_interval_s      = cfg.msg_interval_s;
	doppler_cfg.sequence_interval_s = cfg.sequence_interval_s;
	doppler_cfg.tpl_interval_s      = cfg.tpl_interval_s;

	/* Deploy mode: v2 had _pad0=0 at this offset → defaults to not deployed */
	deploy_mode = (cfg.deployed <= 1) ? cfg.deployed : 0;

#if defined(BSP_HAS_LED_RGB)
	if (cfg.led_mode <= 2)
		MGR_LED_setMode((MGR_LED_Mode_t)cfg.led_mode);
#endif

	/* Mark dirty if migrating from v2 so next save writes v3 format */
	nvm_dirty = (cfg.version < DOPPLER_NVM_VERSION);

	MGR_LOG_DEBUG("[DPL] NVM loaded: count=%u interval=%lus seq=%lus tpl=%lus\r\n",
		doppler_cfg.msg_count,
		(unsigned long)doppler_cfg.msg_interval_s,
		(unsigned long)doppler_cfg.sequence_interval_s,
		(unsigned long)doppler_cfg.tpl_interval_s);
	return true;
}

static bool nvm_save(void)
{
	if (!nvm_dirty) {
		MGR_LOG_DEBUG("[DPL] NVM unchanged, skip write\r\n");
		return true;
	}

	DopplerNvmConfig_t cfg;
	memset(&cfg, 0, sizeof(cfg));

	cfg.magic   = DOPPLER_NVM_MAGIC;
	cfg.version = DOPPLER_NVM_VERSION;
	cfg.msg_count           = doppler_cfg.msg_count;
	cfg.msg_interval_s      = doppler_cfg.msg_interval_s;
	cfg.sequence_interval_s = doppler_cfg.sequence_interval_s;
	cfg.tpl_interval_s      = doppler_cfg.tpl_interval_s;
	cfg.deployed            = deploy_mode;
#if defined(BSP_HAS_LED_RGB)
	cfg.led_mode            = (uint8_t)MGR_LED_getMode();
#endif

	cfg.crc32 = nvm_crc32(&cfg, offsetof(DopplerNvmConfig_t, crc32));

	/* Refresh WDG before flash erase+write (can take time) */
	MGR_WDG_refresh();

	if (MCU_FLASH_write(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[DPL] NVM save error\r\n");
		return false;
	}

	nvm_dirty = false;
	MGR_LOG_DEBUG("[DPL] NVM saved (CRC=0x%08lx)\r\n", cfg.crc32);
	return true;
}

/* ---- Helpers ---- */

static void transition_to(DopplerState_t new_state)
{
	doppler_state = new_state;
	g_uw_doppler_state_for_err = (uint32_t)new_state;
	state_enter_tick = HAL_GetTick();
	MGR_EVTLOG_log(EVT_STATE_CHANGE, (uint16_t)new_state);
}

static uint32_t state_elapsed_ms(void)
{
	return HAL_GetTick() - state_enter_tick;
}

/* ---- MCU_DONE (TPL5111) ---- */

#if defined(MCU_DONE_Pin)
static void pulse_mcu_done(void)
{
	GPIO_InitTypeDef gpio = {0};

	/* Enable the GPIO port clock for MCU_DONE */
	if (MCU_DONE_GPIO_Port == GPIOA)
		__HAL_RCC_GPIOA_CLK_ENABLE();
	else if (MCU_DONE_GPIO_Port == GPIOB)
		__HAL_RCC_GPIOB_CLK_ENABLE();
	else if (MCU_DONE_GPIO_Port == GPIOC)
		__HAL_RCC_GPIOC_CLK_ENABLE();

	gpio.Pin = MCU_DONE_Pin;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(MCU_DONE_GPIO_Port, &gpio);

	HAL_GPIO_WritePin(MCU_DONE_GPIO_Port, MCU_DONE_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(MCU_DONE_GPIO_Port, MCU_DONE_Pin, GPIO_PIN_RESET);
	/* TPL5111 cuts power after this pulse */
	HAL_Delay(200);
}

/**
 * @brief Check if this boot should send a sequence (TPL5111 modulo logic)
 *
 * Reads the wku counter from flash, computes modulo, and decides.
 * Both send and skip paths increment the counter for next cycle.
 * If not time to send: pulses MCU_DONE (TPL5111 cuts power, never returns).
 *
 * @return true if sequence should be sent, false otherwise (but false path
 *         pulses MCU_DONE so typically does not return)
 */
static bool check_tpl_schedule(void)
{
	if (doppler_cfg.tpl_interval_s == 0)
		return true;  /* TPL not configured, always send */

	uint32_t modulo = (doppler_cfg.sequence_interval_s + doppler_cfg.tpl_interval_s - 1)
	                  / doppler_cfg.tpl_interval_s;
	if (modulo == 0)
		modulo = 1;

	uint64_t boot_count = MCU_FLASH_read_wku_counter();
	MGR_LOG_DEBUG("[DPL] WKU boot_count=%lu modulo=%lu\r\n",
		(unsigned long)boot_count, (unsigned long)modulo);

	/* Always increment counter for next cycle */
	MCU_FLASH_increment_wku_counter();

	if ((boot_count % (uint64_t)modulo) == 0) {
		return true;
	}

	/* Not time yet — power off */
	MGR_LOG_DEBUG("[DPL] Skip boot, pulse MCU_DONE\r\n");
	MGR_EVTLOG_log(EVT_BOOT, (uint16_t)(boot_count & 0xFFFF));
	pulse_mcu_done();
	/* Should not reach here -- TPL5111 cuts power */
	return false;
}
#endif /* MCU_DONE_Pin */

#if !defined(MCU_DONE_Pin)
/**
 * @brief Enter deep sleep with RTC wakeup (mode without MCU_DONE)
 *
 * Configures RTC wakeup timer and enters SHUTDOWN mode.
 * Minimum sleep is 1 second. Maximum is ~36 hours (17-bit counter at 1Hz).
 * Does not return.
 */
static void enter_deep_sleep(uint32_t wakeup_seconds)
{
	/* Clamp to valid range: minimum 1s, maximum 131072s (~36h).
	 * 16-bit mode: period = WUT+1, range [1, 65536]s
	 * 17-bit mode: period = WUT+1+0x10000, range [65537, 131072]s */
	if (wakeup_seconds == 0) {
		MGR_LOG_DEBUG("[DPL] WARNING: wakeup_seconds=0, clamped to 1s\r\n");
		wakeup_seconds = 1;
	}
	if (wakeup_seconds > RTC_WAKEUP_MAX_S) {
		MGR_LOG_DEBUG("[DPL] WARNING: wakeup_seconds=%lu clamped to %lu\r\n",
			(unsigned long)wakeup_seconds, (unsigned long)RTC_WAKEUP_MAX_S);
		wakeup_seconds = RTC_WAKEUP_MAX_S;
	}

	MGR_LOG_DEBUG("[DPL] Entering SHUTDOWN for %lus\r\n", (unsigned long)wakeup_seconds);

	/* Save NVM before sleep (only writes if dirty) */
	if (!nvm_save()) {
		MGR_LOG_DEBUG("[DPL] WARNING: NVM save failed before SHUTDOWN\r\n");
		MGR_EVTLOG_log(EVT_ERROR, (uint16_t)ERR_NONE);
	}

#if defined(BSP_HAS_LED_RGB)
	MGR_LED_off();
#endif

	/* Deactivate any pending wakeup timer */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

	/* Select clock source based on counter value.
	 * 16-bit: counter = wakeup_seconds-1, max 65535 → period up to 65536s
	 * 17-bit: counter = wakeup_seconds-1-0x10000, period 65537..131072s */
	uint32_t clk_src = (wakeup_seconds > 0x10000) ?
		RTC_WAKEUPCLOCK_CK_SPRE_17BITS : RTC_WAKEUPCLOCK_CK_SPRE_16BITS;
	uint16_t counter = (uint16_t)((wakeup_seconds - 1) & 0xFFFF);

	HAL_StatusTypeDef rtc_status =
		HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, counter, clk_src, 0);
	if (rtc_status != HAL_OK) {
		MGR_LOG_DEBUG("[DPL] CRITICAL: RTC wakeup setup failed (%d), resetting\r\n",
			rtc_status);
		MGR_ERR_logAndReset(ERR_ASSERT);
		/* Never reaches here */
	}

	/* Disable UART before SHUTDOWN to prevent spurious activity */
	HAL_NVIC_DisableIRQ(LPUART1_IRQn);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);  /* LPUART1_RX */

	/* Set all GPIOs to analog to minimize current leakage */
	GPIO_DisableAllToAnalogInput();

	/* Enable pull-up/pull-down configuration for SHUTDOWN */
	HAL_PWREx_EnablePullUpPullDownConfig();
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_3);  /* EXT_WKUP */
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PA_PSU_EN_Pin);   /* PA off */
#if !defined(SMD_STDALONE)
	/* STDALONE: PC1 = TPS63901 SEL — pull held externally by R11 (10M to VBAT) */
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_C, PA_PSU_SEL_Pin);    /* VSEL high */
#endif

#if defined(BSP_HAS_PWR_LATCH)
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_7);  /* PWR_LATCH off */
#endif

	/* Enable internal wakeup line (RTC → PWR controller).
	 * Without this the RTC wakeup event does not reach the PWR
	 * controller and the device never wakes from SHUTDOWN. */
	HAL_PWREx_DisableInternalWakeUpLine();
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);
	HAL_PWREx_EnableInternalWakeUpLine();

	/* Configure external wakeup pins */
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN3);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3_HIGH);

	__HAL_RCC_CLEAR_RESET_FLAGS();

	/* Enter SHUTDOWN -- never returns */
	HAL_PWREx_EnterSHUTDOWNMode();
}
#endif /* !MCU_DONE_Pin */

/* ---- MAC profile init ---- */

#ifdef USE_MAC_PRFL_BLIND
static struct KNS_MAC_BLIND_usrCfg_t prflBlindUserCfg;
#endif

static void start_mac_profile(void)
{
	struct KNS_MAC_appEvt_t appEvt;
	appEvt.id = KNS_MAC_INIT;
	appEvt.init_prfl_ctxt.id = KNS_MAC_PRFL_NONE;

#ifdef USE_MAC_PRFL_BASIC
	appEvt.init_prfl_ctxt.id = KNS_MAC_PRFL_BASIC;
#endif
#ifdef USE_MAC_PRFL_BLIND
	/* BLIND: MAC handles retransmissions.
	 * retx_period_s is uint16_t, clamped at setCfg validation. */
	prflBlindUserCfg.retx_nb = doppler_cfg.msg_count;
	prflBlindUserCfg.retx_period_s = (uint16_t)doppler_cfg.msg_interval_s;
	prflBlindUserCfg.nb_parrallel_msg = 1;
	appEvt.init_prfl_ctxt.id = KNS_MAC_PRFL_BLIND;
	appEvt.init_prfl_ctxt.blindCfg = prflBlindUserCfg;
#endif

	enum KNS_status_t status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
	if (status != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[DPL] MAC init push failed: 0x%x\r\n", status);
	}
}

/* ---- TX payload builder ---- */

static bool build_tx_payload(struct KNS_MAC_appEvt_t *appEvt)
{
	struct KNS_CFG_radio_t device_radio_cfg;
	if (KNS_CFG_getRadioInfo(&device_radio_cfg) != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[DPL] Radio config read failed\r\n");
		return false;
	}

	appEvt->id = KNS_MAC_SEND_DATA;
	appEvt->data_ctxt.sf = KNS_SF_NO_SERVICE;

	/* Payload: [wku_count(2B)][vbat_mV(2B)][tx_index(1B)][msg_count(1B)][pad] */
	memset(appEvt->data_ctxt.usrdata, 0, KNS_MAC_USRDATA_MAXLEN);

	uint16_t wku = 0;
#if defined(MCU_DONE_Pin)
	wku = (uint16_t)(MCU_FLASH_read_wku_counter() & 0xFFFF);
#endif
	appEvt->data_ctxt.usrdata[0] = (uint8_t)(wku >> 8);
	appEvt->data_ctxt.usrdata[1] = (uint8_t)(wku & 0xFF);

	uint16_t vbat = 0;
#if defined(BSP_HAS_VBAT_ADC)
	vbat = last_vbat_mV;
#endif
	appEvt->data_ctxt.usrdata[2] = (uint8_t)(vbat >> 8);
	appEvt->data_ctxt.usrdata[3] = (uint8_t)(vbat & 0xFF);
	appEvt->data_ctxt.usrdata[4] = tx_index;
	appEvt->data_ctxt.usrdata[5] = doppler_cfg.msg_count;

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
		MGR_LOG_DEBUG("[DPL] Unknown modulation %d\r\n", device_radio_cfg.modulation);
		return false;
	}

	return true;
}

/* ---- MAC event processing ---- */

static bool process_mac_events(void)
{
	bool got_event = false;
	enum KNS_status_t status;
	struct KNS_MAC_srvcEvt_t srvcEvt;

	for (status = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&srvcEvt);
	     status != KNS_STATUS_QEMPTY;
	     status = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&srvcEvt)) {
		if (status != KNS_STATUS_OK)
			continue;

		got_event = true;
		switch (srvcEvt.id) {
		case KNS_MAC_OK:
			if (srvcEvt.app_evt == KNS_MAC_INIT) {
				MGR_LOG_DEBUG("[DPL] MAC init OK\r\n");
				MGR_EVTLOG_log(EVT_MAC_READY, 0);
				if (doppler_state == DOPPLER_WAIT_MAC_READY)
					transition_to(DOPPLER_SEND_MSG);
			}
			break;

		case KNS_MAC_TX_DONE:
			MGR_LOG_DEBUG("[DPL] TX done (#%u)\r\n", tx_index);
			MGR_EVTLOG_log(EVT_TX_DONE, (uint16_t)tx_index);
			if (doppler_state == DOPPLER_WAIT_TX_DONE) {
#ifdef USE_MAC_PRFL_BLIND
				/* BLIND: MAC handled all retx, sequence is done */
				tx_index = doppler_cfg.msg_count;
				transition_to(DOPPLER_SEQUENCE_DONE);
#else
				/* BASIC: check if more messages to send */
				tx_index++;
				if (tx_index >= doppler_cfg.msg_count)
					transition_to(DOPPLER_SEQUENCE_DONE);
				else
					transition_to(DOPPLER_WAIT_INTERVAL);
#endif
			}
			break;

		case KNS_MAC_TX_TIMEOUT:
			MGR_LOG_DEBUG("[DPL] TX timeout\r\n");
			MGR_ERR_log(ERR_TX_TIMEOUT);
			MGR_EVTLOG_log(EVT_TX_TIMEOUT, (uint16_t)tx_index);
			if (doppler_state == DOPPLER_WAIT_TX_DONE) {
#ifdef USE_MAC_PRFL_BLIND
				tx_index = doppler_cfg.msg_count;
				transition_to(DOPPLER_SEQUENCE_DONE);
#else
				tx_index++;
				if (tx_index >= doppler_cfg.msg_count)
					transition_to(DOPPLER_SEQUENCE_DONE);
				else
					transition_to(DOPPLER_WAIT_INTERVAL);
#endif
			}
			break;

		case KNS_MAC_ERROR:
			MGR_LOG_DEBUG("[DPL] MAC error: %d\r\n", srvcEvt.status);
			MGR_EVTLOG_log(EVT_MAC_ERROR, (uint16_t)srvcEvt.status);
			if (doppler_state == DOPPLER_WAIT_TX_DONE)
				transition_to(DOPPLER_SEQUENCE_DONE);
			else if (doppler_state == DOPPLER_WAIT_MAC_READY)
				transition_to(DOPPLER_INIT_MAC);
			break;

		default:
			break;
		}
	}

	return got_event;
}

/* ---- LPM client ---- */

static enum MgrLpm_LPM_t doppler_lpmReq(void)
{
	return doppler_lpm_mode;
}

static bool doppler_lpmNotifEnter(__attribute__((unused)) enum MgrLpm_LPM_t lpm)
{
#if defined(BSP_HAS_LED_RGB)
	MGR_LED_off();
#endif
	return true;
}

static bool doppler_lpmNotifExit(__attribute__((unused)) enum MgrLpm_LPM_t lpm)
{
	return true;
}

static struct MgrLpmClientCb_t doppler_lpm_client = {
	.fpMGR_LPM_LpmReqCb        = doppler_lpmReq,
	.fpMGR_LPM_LpmNotifEnterCb = doppler_lpmNotifEnter,
	.fpMGR_LPM_LpmNotifExitCb  = doppler_lpmNotifExit,
};

/* ---- Sequence completion ---- */

static void finish_sequence(void)
{
	MGR_LOG_DEBUG("[DPL] Sequence done (%u msgs)\r\n", tx_index);
	MGR_EVTLOG_log(EVT_SHUTDOWN, (uint16_t)tx_index);

#if defined(MCU_DONE_Pin)
	/* Mode TPL5111: save config (if dirty) then pulse MCU_DONE to cut power.
	 * No point calling stop_mac_profile: TPL5111 cuts power immediately. */
	nvm_save();
	MGR_LOG_DEBUG("[DPL] Pulse MCU_DONE\r\n");
	pulse_mcu_done();
	/* Should not reach here */
#else
	/* Mode RTC: enter deep sleep for sequence_interval_s.
	 * No point calling stop_mac_profile: SHUTDOWN kills everything. */
	enter_deep_sleep(doppler_cfg.sequence_interval_s);
	/* Should not reach here */
#endif
}

/* ---- Public API ---- */

void KNS_APP_doppler_init(void)
{
	tx_index = 0;
	last_tx_tick = 0;
	mac_init_retries = 0;
	state_enter_tick = HAL_GetTick();
	doppler_lpm_mode = LOW_POWER_MODE_STOP;
	nvm_dirty = false;

	/* Start IWDG watchdog early (16s timeout) - before any slow init */
	MGR_WDG_init();

	/* Init event log */
	MGR_EVTLOG_init();
	MGR_EVTLOG_log(EVT_BOOT, 0);

	/* Load saved config from flash */
	nvm_load();

#if defined(BSP_HAS_VBAT_ADC)
	MGR_BAT_init();
	last_vbat_mV = MGR_BAT_readVoltage_mV();
	MGR_LOG_DEBUG("[DPL] BAT: %umV\r\n", last_vbat_mV);
#endif

	/* Register LPM client */
	MGR_LPM_registerClient(doppler_lpm_client);

	MGR_LOG_DEBUG("[DPL] Init: count=%u interval=%lus seq=%lus tpl=%lus\r\n",
		doppler_cfg.msg_count,
		(unsigned long)doppler_cfg.msg_interval_s,
		(unsigned long)doppler_cfg.sequence_interval_s,
		(unsigned long)doppler_cfg.tpl_interval_s);

#if defined(BSP_HAS_LED_RGB)
	/* Default to 24h auto-off for deployment battery savings.
	 * nvm_load() above may override this if a saved mode exists. */
	if (MGR_LED_getMode() == MGR_LED_MODE_ON)
		MGR_LED_setMode(MGR_LED_MODE_24H);
	MGR_LED_blink(MGR_LED_BLUE, 3, 200, 200);
#endif

	transition_to(DOPPLER_BOOT);
}

void KNS_APP_doppler_loop(void)
{
	/* Refresh watchdog every loop */
	MGR_WDG_refresh();

#if defined(BSP_HAS_LED_RGB)
	/* Update LED blink animations */
	MGR_LED_task();
#endif

	/* Process AT commands if available */
#if defined(USE_UART_DRIVER)
	{
		uint8_t *pu8_atcmd = MGR_AT_CMD_popNextAt();
		if (pu8_atcmd != NULL) {
			MGR_AT_CMD_decodeAt(pu8_atcmd);
			/* Track AT activity during boot window */
			if (doppler_state == DOPPLER_BOOT)
				boot_window_at_received = true;
		}
		MGR_AT_CMD_macEvtProcess();
	}
#endif

	switch (doppler_state) {

	case DOPPLER_BOOT:
	{
		/* Boot window: UART accepts AT commands for BOOT_WINDOW_MS.
		 * If deployed and no AT received → UART is de-init for power savings.
		 * If AT received during window → UART stays active (config session). */
		bool boot_done = false;
#if defined(BSP_HAS_LED_RGB)
		boot_done = (MGR_LED_isBlinkDone() || state_elapsed_ms() > TIMEOUT_BOOT_MS);
#else
		boot_done = (state_elapsed_ms() > BOOT_WINDOW_MS);
#endif
		if (boot_done) {
#if defined(USE_UART_DRIVER)
			if (deploy_mode && !boot_window_at_received) {
				MGR_LOG_DEBUG("[DPL] Deploy mode: UART disabled\r\n");
				HAL_NVIC_DisableIRQ(LPUART1_IRQn);
				HAL_UART_DeInit(&hlpuart1);
				HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
			} else if (deploy_mode) {
				MGR_LOG_DEBUG("[DPL] Deploy mode: UART kept (AT received)\r\n");
			}
#endif
			transition_to(DOPPLER_CHECK_SCHEDULE);
		}
		return;
	}

	case DOPPLER_CHECK_SCHEDULE:
	{
#if defined(MCU_DONE_Pin)
		if (!check_tpl_schedule()) {
			/* check_tpl_schedule pulsed MCU_DONE and should not return.
			 * If it does (TPL5111 didn't cut power), just idle. */
			return;
		}
#endif
		/* Proceed to send sequence */
#if defined(BSP_HAS_LED_RGB)
		MGR_LED_blink(MGR_LED_GREEN, 2, 200, 200);
#endif
		transition_to(DOPPLER_INIT_MAC);
		return;
	}

	case DOPPLER_INIT_MAC:
		start_mac_profile();
		transition_to(DOPPLER_WAIT_MAC_READY);
		return;

	case DOPPLER_WAIT_MAC_READY:
		process_mac_events();
		if (doppler_state != DOPPLER_WAIT_MAC_READY)
			return;  /* Transitioned by event */

		if (state_elapsed_ms() > TIMEOUT_MAC_READY_MS) {
			MGR_LOG_DEBUG("[DPL] MAC ready timeout (retry %u/%u)\r\n",
				mac_init_retries + 1, MAX_MAC_INIT_RETRIES);
			MGR_ERR_log(ERR_MAC_TIMEOUT);
			MGR_EVTLOG_log(EVT_TIMEOUT, (uint16_t)DOPPLER_WAIT_MAC_READY);
			if (++mac_init_retries >= MAX_MAC_INIT_RETRIES) {
				MGR_LOG_DEBUG("[DPL] MAC init failed, resetting\r\n");
				MGR_ERR_logAndReset(ERR_MAC_INIT_FAIL);
			}
			transition_to(DOPPLER_INIT_MAC);
		}
		return;

	case DOPPLER_SEND_MSG:
	{
#if defined(BSP_HAS_LED_RGB)
		MGR_LED_set(MGR_LED_VIOLET);
#endif

#if defined(BSP_HAS_VBAT_ADC)
		last_vbat_mV = MGR_BAT_readVoltage_mV();
		MGR_EVTLOG_log(EVT_BAT, last_vbat_mV);
		if (!MGR_BAT_isTxAllowed()) {
			MGR_LOG_DEBUG("[DPL] Battery low (%umV), TX inhibited\r\n", last_vbat_mV);
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_blink(MGR_LED_RED, 5, 100, 100);
#endif
			transition_to(DOPPLER_SEQUENCE_DONE);
			return;
		}
#endif

		struct KNS_MAC_appEvt_t appEvt;
		if (!build_tx_payload(&appEvt)) {
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			transition_to(DOPPLER_SEQUENCE_DONE);
			return;
		}

		enum KNS_status_t status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
		if (status == KNS_STATUS_OK) {
			MGR_LOG_DEBUG("[DPL] TX #%u sent\r\n", tx_index);
			last_tx_tick = HAL_GetTick();
			transition_to(DOPPLER_WAIT_TX_DONE);
		} else {
			MGR_LOG_DEBUG("[DPL] TX push failed: 0x%x\r\n", status);
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			transition_to(DOPPLER_SEQUENCE_DONE);
		}
		return;
	}

	case DOPPLER_WAIT_TX_DONE:
		process_mac_events();
		if (doppler_state != DOPPLER_WAIT_TX_DONE) {
			/* Transitioned by event */
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			return;
		}

		/* Timeout protection */
		if (state_elapsed_ms() > TIMEOUT_TX_DONE_MS) {
			MGR_LOG_DEBUG("[DPL] TX done timeout\r\n");
			MGR_ERR_log(ERR_TX_TIMEOUT);
			MGR_EVTLOG_log(EVT_TIMEOUT, (uint16_t)DOPPLER_WAIT_TX_DONE);
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
#ifdef USE_MAC_PRFL_BLIND
			tx_index = doppler_cfg.msg_count;
			transition_to(DOPPLER_SEQUENCE_DONE);
#else
			tx_index++;
			if (tx_index >= doppler_cfg.msg_count)
				transition_to(DOPPLER_SEQUENCE_DONE);
			else
				transition_to(DOPPLER_WAIT_INTERVAL);
#endif
		}
		return;

	case DOPPLER_WAIT_INTERVAL:
	{
		/* BASIC mode only: wait msg_interval_s between messages.
		 * Uses unsigned subtraction for safe HAL_GetTick() wrap-around.
		 * msg_interval_s is validated at setCfg() to not overflow *1000. */
		uint32_t elapsed_ms = HAL_GetTick() - last_tx_tick;
		uint32_t interval_ms = doppler_cfg.msg_interval_s * 1000UL;
		if (elapsed_ms >= interval_ms) {
			MGR_LOG_DEBUG("[DPL] Interval elapsed, next msg #%u\r\n", tx_index);
			transition_to(DOPPLER_SEND_MSG);
		}
		return;
	}

	case DOPPLER_SEQUENCE_DONE:
		finish_sequence();
		/* Should not reach here (power off or SHUTDOWN) */
		return;

	default:
		kns_assert(0);
		return;
	}
}

KNS_APP_DopplerCfg_t KNS_APP_doppler_getCfg(void)
{
	return doppler_cfg;
}

bool KNS_APP_doppler_setCfg(const KNS_APP_DopplerCfg_t *cfg)
{
	if (cfg->msg_count == 0 || cfg->msg_interval_s == 0 || cfg->sequence_interval_s == 0)
		return false;

	if (cfg->msg_interval_s > MSG_INTERVAL_MAX_S)
		return false;

#if !defined(MCU_DONE_Pin)
	/* RTC wakeup max is 131072s (~36h). Reject values that would be silently clamped. */
	if (cfg->sequence_interval_s > RTC_WAKEUP_MAX_S)
		return false;
#endif

#ifdef USE_MAC_PRFL_BLIND
	/* BLIND retx_period_s is uint16_t */
	if (cfg->msg_interval_s > UINT16_MAX)
		return false;
#endif

	doppler_cfg = *cfg;
	nvm_dirty = true;
	return true;
}

bool KNS_APP_doppler_nvmSave(void)
{
	return nvm_save();
}

void KNS_APP_doppler_markDirty(void)
{
	nvm_dirty = true;
}

uint8_t KNS_APP_doppler_getDeployMode(void)
{
	return deploy_mode;
}

void KNS_APP_doppler_setDeployMode(uint8_t mode)
{
	deploy_mode = mode ? 1 : 0;
	nvm_dirty = true;
	MGR_LOG_DEBUG("[DPL] Deploy mode: %u\r\n", deploy_mode);
}

/**
 * @}
 */
