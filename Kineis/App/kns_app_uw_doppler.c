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
 * TX payload (UW Doppler format, MSB-first bit-packed):
 *   [0..2]    header = 0b011 (UW Doppler type indicator)
 *   [3..10]   last_known_pos = 0 (8 bits, reserved for future GPS hint)
 *   [11..17]  battery_encoded (mV - 2700) / 20, capped 0..127
 *   [18]      is_low_battery flag
 *   [19..]    zero pad up to modulation frame size
 *   LDA2/LDA2L: CRC8 (linkit-style poly 0x8380) at byte 23
 *   VLDA4(24)/LDK(128): modem-handled CRC
 *
 * Header value 0b011 is unused by linkit-v4 and reserved here for "UW Doppler"
 * (battery-only telemetry, no GPS). Self-describing across all modulations.
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
#include <string.h>
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
#include "mcu_misc.h"
#if defined(USE_UART_DRIVER)
#include "mgr_at_cmd.h"
#include "usart.h"
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

/* SWS baselines retained across warm resets (SRAM2 survives IWDG/SW reset) */
#define SWS_RETAINED_MAGIC 0x53575342UL /* "SWSB" */
static __attribute__((__section__(".retentionRamData")))
struct {
	uint32_t magic;
	uint16_t air_baseline;
	uint16_t water_baseline;
	uint16_t observed_peak_adc;  /**< Highest ADC ever seen (dynamic cap) */
} sws_retained;

/* Exported for fault handlers (MGR_ERR_LOG_FAULT macro in stm32wlxx_it.c) */
volatile uint32_t g_uw_doppler_state_for_err = 0;

static KNS_APP_UwDopplerTxCfg_t tx_cfg = {
	.tx_initial_interval_s = 10,
	.tx_growth_percent     = 10,
	.tx_max_interval_s     = 180,  /**< 3min cap: better Argos pass coverage for Doppler */
	.tx_max_count          = 0,    /**< unlimited */
	.tx_jitter_percent     = 10,   /**< +/-10% randomization to avoid TX collisions */
};

/* Deploy mode: 1 = deployed (TX enabled), 0 = not deployed (SWS runs but no TX) */
static uint8_t deploy_mode = 1;

/* Boot window: AT command received during boot window keeps UART active */
#if defined(USE_UART_DRIVER)
static bool boot_window_at_received = false;
#endif

#define BOOT_WINDOW_MS  5000  /**< UART listen window at boot (5s) */

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
#define BOOT_DEPLOY_LED_MS       2000   /**< Deploy color display duration */
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

/* TCXO pre-warmup: started on UW->SURFACE so 1st TX skips warmup wait */
static bool     tcxo_first_tx_skip = false;   /**< Apply 0ms warmup for next TX */
static uint32_t tcxo_warmup_saved_ms = 0;     /**< Original warmup, restored after 1st TX */

/* Periodic SWS calibration save to flash (debounced) */
#define NVM_CALIB_SAVE_MIN_INTERVAL_S  300  /**< Min 5 min between flash writes */

/* Simple LCG for jitter (no need for cryptographic randomness) */
static uint32_t prng_state = 0xA5A5A5A5UL;
static uint32_t prng_next(void)
{
	/* Numerical Recipes LCG */
	prng_state = prng_state * 1664525UL + 1013904223UL;
	return prng_state;
}

/* LPM client: prevent SHUTDOWN during UW_DOPPLER operation */
static enum MgrLpm_LPM_t uw_doppler_lpmReq(void)
{
	return LOW_POWER_MODE_STOP;
}

static bool uw_doppler_lpmNotifEnter(__attribute__((unused)) enum MgrLpm_LPM_t lpm)
{
#if defined(BSP_HAS_LED_RGB)
	/* Turn off LEDs before STOP to avoid current draw during sleep */
	MGR_LED_off();
#endif
	/* Reconfigure SWS power pin as analog to eliminate GPIO leakage in STOP */
	MGR_SWS_enterLowPower();
	return true;
}

static bool uw_doppler_lpmNotifExit(__attribute__((unused)) enum MgrLpm_LPM_t lpm)
{
	/* Restore SWS power pin as output after STOP wakeup */
	MGR_SWS_exitLowPower();
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

	/* Apply jitter: random +/- jitter_percent of base interval.
	 * Avoids RF collisions when multiple tags surface together.
	 */
	if (tx_cfg.tx_jitter_percent > 0 && interval_ms > 0) {
		uint32_t span_ms = (interval_ms * tx_cfg.tx_jitter_percent) / 100;
		if (span_ms > 0) {
			/* Map prng to [-span_ms, +span_ms] */
			uint32_t r = prng_next();
			int32_t delta = (int32_t)(r % (2 * span_ms + 1)) - (int32_t)span_ms;
			int64_t jittered = (int64_t)interval_ms + delta;
			if (jittered < 1000) jittered = 1000;  /* min 1s safety */
			interval_ms = (uint32_t)jittered;
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

/* ---- Linkit-v4 compatible Argos packet helpers ---- */

/* Battery encoding (Argos/linkit-v4): (mV - 2700) / 20, clamped 0..127 (7 bits).
 * Reference: linkit-v4-core/core/services/argos_packet_builder.cpp:38
 */
#define ARGOS_BATT_REF_MV    2700U
#define ARGOS_BATT_MV_PER_UNIT  20U
#define ARGOS_BATT_MAX_VAL    127U

/* Low-battery margin above hard inhibit threshold. The flag warns receivers
 * that the device is approaching cutoff so they can deprioritize the fix.
 */
#define LOW_BATT_MARGIN_MV    200U

/* UW Doppler packet header — 3-bit type identifier so receivers can demux
 * this packet from linkit's Short(000)/Sensor(001)/Fastloc(010)/RSPB(100-110)/
 * CloudLocate(111) types. Value 0b011 is unused by linkit and reserved here
 * for "UW Doppler" (battery-only, no GPS).
 */
#define UW_DOPPLER_PACKET_HEADER  0b011U

/* LDA2 frame: data ends at bit 184, CRC8 at byte 23 (bits 184..191).
 * Reference: linkit-v4-core/core/services/argos_packet_builder.hpp
 */
#define LDA2_FRAME_BYTES   24U
#define LDA2_DATA_BITS    184U

static uint8_t argos_convert_battery_voltage(uint16_t mV)
{
	if (mV <= ARGOS_BATT_REF_MV)
		return 0;
	uint32_t v = ((uint32_t)mV - ARGOS_BATT_REF_MV) / ARGOS_BATT_MV_PER_UNIT;
	if (v > ARGOS_BATT_MAX_VAL) v = ARGOS_BATT_MAX_VAL;
	return (uint8_t)v;
}

/* Pack `nbits` bits of `value` into `buf` at bit offset `*bit_pos`,
 * MSB-first big-endian (same semantics as linkit's PACK_BITS macro).
 * Updates *bit_pos by nbits.
 */
static void argos_pack_bits(uint8_t *buf, uint32_t *bit_pos,
                            uint32_t value, uint8_t nbits)
{
	for (int i = (int)nbits - 1; i >= 0; i--) {
		uint8_t bit = (uint8_t)((value >> i) & 1U);
		uint32_t byte_idx = *bit_pos >> 3;
		uint8_t  bit_in_byte = 7U - (uint8_t)(*bit_pos & 7U);
		buf[byte_idx] = (uint8_t)((buf[byte_idx] & ~(1U << bit_in_byte)) |
		                          (bit << bit_in_byte));
		(*bit_pos)++;
	}
}

/* CRC8 used by Argos LDA2 frames (linkit-v4 core/util/crc8.hpp).
 * Polynomial 0x8380 in 16-bit accumulator (NOT the standard 0x07 CRC8-CCITT
 * used by the SPI protocol layer). Final CRC = (acc >> 8) & 0xFF.
 *
 * Computes CRC over `nbits` bits of `data`, prepending zero pad to byte-align.
 * Result is the CRC8 byte to be stored at byte 23 of the LDA2 frame.
 */
static uint8_t argos_crc8(const uint8_t *data, uint32_t nbits)
{
	/* Linkit's algorithm zero-pads at the START so total bit count becomes a
	 * multiple of 8. We replicate by computing remainder up front, then
	 * iterating bytes from the padded start.
	 */
	uint16_t crc = 0;
	uint32_t remainder = (8U - (nbits % 8U)) % 8U;
	uint32_t total_bits = nbits + remainder;
	uint32_t total_bytes = total_bits / 8U;

	/* Re-pack data with leading zero-pad of `remainder` bits */
	uint8_t buffer[LDA2_FRAME_BYTES] = {0};
	uint32_t op_pos = remainder;
	uint32_t ip_pos = 0;
	uint32_t left = nbits;
	while (left) {
		uint32_t bits = (left > 8U) ? 8U : left;
		uint8_t byte = 0;
		/* Extract `bits` from data starting at ip_pos (MSB-first) */
		for (uint32_t k = 0; k < bits; k++) {
			uint32_t b_idx = (ip_pos + k) >> 3;
			uint8_t  b_off = 7U - (uint8_t)((ip_pos + k) & 7U);
			byte = (uint8_t)((byte << 1) | ((data[b_idx] >> b_off) & 1U));
		}
		/* Pack `bits` into buffer at op_pos (MSB-first) */
		for (int k = (int)bits - 1; k >= 0; k--) {
			uint32_t b_idx = op_pos >> 3;
			uint8_t  b_off = 7U - (uint8_t)(op_pos & 7U);
			uint8_t bit = (uint8_t)((byte >> k) & 1U);
			buffer[b_idx] = (uint8_t)((buffer[b_idx] & ~(1U << b_off)) | (bit << b_off));
			op_pos++;
		}
		ip_pos += bits;
		left -= bits;
	}

	for (uint32_t idx = 0; idx < total_bytes; idx++) {
		crc ^= (uint16_t)((uint16_t)buffer[idx] << 8);
		for (int i = 0; i < 8; i++) {
			if (crc & 0x8000U)
				crc ^= 0x8380U;
			crc = (uint16_t)(crc << 1);
		}
	}
	return (uint8_t)(crc >> 8);
}

/* Build the UW Doppler payload — battery telemetry only, with explicit
 * 3-bit type header so receivers can identify it on any modulation.
 *
 * Bit layout (MSB-first):
 *   [0..2]    header = 0b011  (UW Doppler type)
 *   [3..10]   last_known_pos = 0  (8 bits, reserved for future GPS hint)
 *   [11..17]  battery_encoded     (7 bits, (mV-2700)/20 capped to 127)
 *   [18]      is_low_battery flag (1 bit)
 *   [19..]    zero pad
 *   For LDA2/LDA2L: CRC8 at byte 23 (computed over first 184 bits).
 *
 * Frame sizes per modulation:
 *   VLDA4 → 24 bits  (3 bytes), modem handles CRC
 *   LDK   → 128 bits (16 bytes), modem handles CRC
 *   LDA2  → 192 bits (24 bytes), firmware embeds CRC8 at byte 23
 *   LDA2L → 196 bits (24.5 bytes), firmware embeds CRC8 at byte 23
 */
static bool build_tx_payload(struct KNS_MAC_appEvt_t *appEvt)
{
	struct KNS_CFG_radio_t device_radio_cfg;
	if (KNS_CFG_getRadioInfo(&device_radio_cfg) != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[UW_DPL] Radio config read failed, skipping TX\r\n");
		return false;
	}

	appEvt->id = KNS_MAC_SEND_DATA;
	appEvt->data_ctxt.sf = KNS_SF_NO_SERVICE;

	/* Zero the entire payload buffer (zero pad is the linkit convention) */
	memset(appEvt->data_ctxt.usrdata, 0, KNS_MAC_USRDATA_MAXLEN);

	/* Resolve battery + low-battery flag */
	uint16_t batt_mV = 0;
	bool is_low_battery = false;
#if defined(BSP_HAS_VBAT_ADC)
	batt_mV = last_vbat_mV;
	uint16_t min_tx_mV = MGR_BAT_getMinTxVoltage_mV();
	if (min_tx_mV > 0)
		is_low_battery = (batt_mV < (uint16_t)(min_tx_mV + LOW_BATT_MARGIN_MV));
#endif

	uint8_t batt_encoded = argos_convert_battery_voltage(batt_mV);

	/* Pack UW Doppler layout (19 useful bits, MSB-first):
	 *   header(3) + last_known_pos(8) + batt(7) + low_batt(1) = 19 bits
	 */
	uint32_t bit_pos = 0;
	const uint32_t last_known_pos = 0;  /* placeholder, no GPS */
	argos_pack_bits(appEvt->data_ctxt.usrdata, &bit_pos, UW_DOPPLER_PACKET_HEADER, 3);
	argos_pack_bits(appEvt->data_ctxt.usrdata, &bit_pos, last_known_pos, 8);
	argos_pack_bits(appEvt->data_ctxt.usrdata, &bit_pos, batt_encoded, 7);
	argos_pack_bits(appEvt->data_ctxt.usrdata, &bit_pos, is_low_battery ? 1U : 0U, 1);

	/* Set bitlen based on modulation. LDA2/LDA2L need a CRC8 byte at the end
	 * because the modem doesn't add one for LDA2; LDK/VLDA4 modem handles CRC.
	 */
	bool needs_lda2_crc = false;
	switch (device_radio_cfg.modulation) {
	case KNS_TX_MOD_LDA2:
		appEvt->data_ctxt.usrdata_bitlen = 192;
		needs_lda2_crc = true;
		break;
	case KNS_TX_MOD_LDA2L:
		appEvt->data_ctxt.usrdata_bitlen = 196;
		needs_lda2_crc = true;
		break;
	case KNS_TX_MOD_VLDA4:
		/* 24-bit frame: 19 useful + 5 zero pad. Modem handles CRC. */
		appEvt->data_ctxt.usrdata_bitlen = 24;
		break;
	case KNS_TX_MOD_LDK:
		/* 128-bit frame: 19 useful + 109 zero pad. Modem handles CRC. */
		appEvt->data_ctxt.usrdata_bitlen = 128;
		break;
	default:
		MGR_LOG_DEBUG("[UW_DPL] Unknown modulation %d, fallback to LDA2\r\n",
			device_radio_cfg.modulation);
		appEvt->data_ctxt.usrdata_bitlen = 192;
		needs_lda2_crc = true;
		break;
	}

	/* LDA2/LDA2L: compute CRC8 over the first 184 bits, store at byte 23 */
	if (needs_lda2_crc) {
		uint8_t crc = argos_crc8(appEvt->data_ctxt.usrdata, LDA2_DATA_BITS);
		appEvt->data_ctxt.usrdata[LDA2_FRAME_BYTES - 1] = crc;
	}

	MGR_LOG_DEBUG("[UW_DPL] TX payload: hdr=0x%X batt=%umV(enc=%u) low=%u "
		"mod=%d bits=%u data=%02X%02X%02X...%02X\r\n",
		UW_DOPPLER_PACKET_HEADER, batt_mV, batt_encoded,
		is_low_battery ? 1 : 0, device_radio_cfg.modulation,
		appEvt->data_ctxt.usrdata_bitlen,
		appEvt->data_ctxt.usrdata[0], appEvt->data_ctxt.usrdata[1],
		appEvt->data_ctxt.usrdata[2],
		needs_lda2_crc ? appEvt->data_ctxt.usrdata[LDA2_FRAME_BYTES - 1] : 0);

	return true;
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
			/* Restore TCXO warmup after first TX completes */
			if (tcxo_first_tx_skip) {
				MCU_MISC_TCXO_set_warmup(tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = false;
				MGR_LOG_DEBUG("[UW_DPL] TCXO warmup restored to %lums\r\n",
					(unsigned long)tcxo_warmup_saved_ms);
			}
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE)
				transition_to(UW_DOPPLER_MONITORING);
			break;

		case KNS_MAC_TX_TIMEOUT:
			MGR_LOG_DEBUG("[UW_DPL] TX timeout\r\n");
			MGR_EVTLOG_log(EVT_TX_TIMEOUT, 0);
			if (tcxo_first_tx_skip) {
				MCU_MISC_TCXO_set_warmup(tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = false;
			}
			if (uw_doppler_state == UW_DOPPLER_WAIT_TX_DONE)
				transition_to(UW_DOPPLER_MONITORING);
			break;

		case KNS_MAC_ERROR:
			MGR_LOG_DEBUG("[UW_DPL] MAC error: %d\r\n", srvcEvt.status);
			MGR_EVTLOG_log(EVT_MAC_ERROR, (uint16_t)srvcEvt.status);
			if (tcxo_first_tx_skip) {
				MCU_MISC_TCXO_set_warmup(tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = false;
			}
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

	/* Seed PRNG with a runtime-varying value so each device/boot picks a
	 * different jitter sequence — important to avoid clustered TX patterns
	 * across a multi-tag deployment.
	 */
	prng_state ^= HAL_GetTick();
	prng_state ^= (uint32_t)(uintptr_t)&prng_state;
	(void)prng_next();  /* discard first value */

	/* Start IWDG watchdog early (16s timeout) - before any slow init */
	MGR_WDG_init();

	/* Init event log (SRAM2 retention - survives resets) */
	MGR_EVTLOG_init();
	MGR_EVTLOG_log(EVT_BOOT, 0);

	/* Load saved config from flash (if valid) */
	MGR_NVM_load();

	MGR_LOG_DEBUG("[UW_DPL] Init: interval=%us growth=%u%% max=%us deploy=%u\r\n",
		tx_cfg.tx_initial_interval_s, tx_cfg.tx_growth_percent,
		tx_cfg.tx_max_interval_s, deploy_mode);

	/* SWS baselines will be restored after MGR_SWS_init() via
	 * KNS_APP_uw_doppler_restoreSwsBaselines() called from main.c */

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

#if defined(BSP_HAS_LED_RGB)
	/* Boot sequence: blink 10x */
	MGR_LED_blink(MGR_LED_BLUE, 10, 200, 200);
#endif
	/* Always go through BOOT state for boot window (UART listen period) */
	transition_to(UW_DOPPLER_BOOT);
}

void KNS_APP_uw_doppler_loop(void)
{
	/* Refresh watchdog every loop iteration */
	MGR_WDG_refresh();

	/* Process pending AT commands */
#if defined(USE_UART_DRIVER)
	{
		uint8_t *pu8_atcmd = MGR_AT_CMD_popNextAt();
		if (pu8_atcmd != NULL) {
			MGR_AT_CMD_decodeAt(pu8_atcmd);
			/* Track AT activity during boot window */
			if (uw_doppler_state <= UW_DOPPLER_BOOT_DEPLOY_LED)
				boot_window_at_received = true;
		}
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
	if (uw_doppler_state >= UW_DOPPLER_MONITORING) {
		MGR_SWS_task();
		/* Save adapted baselines to retention RAM for warm reset recovery */
		sws_retained.magic = SWS_RETAINED_MAGIC;
		sws_retained.air_baseline = MGR_SWS_getAirBaseline();
		sws_retained.water_baseline = MGR_SWS_getWaterBaseline();
		sws_retained.observed_peak_adc = MGR_SWS_getObservedPeak();
	}

	switch (uw_doppler_state) {
	case UW_DOPPLER_BOOT:
	{
		/* Boot window: UART listens for AT commands.
		 * With LED: wait for blink to finish (with timeout).
		 * Without LED: wait BOOT_WINDOW_MS for AT commands. */
		bool boot_done = false;
#if defined(BSP_HAS_LED_RGB)
		boot_done = (MGR_LED_isBlinkDone() || state_elapsed_ms() > TIMEOUT_BOOT_MS);
#else
		boot_done = (state_elapsed_ms() > BOOT_WINDOW_MS);
#endif
		if (boot_done) {
#if defined(BSP_HAS_LED_RGB)
			/* Show deploy status: green=deployed, blue=not deployed */
			if (deploy_mode)
				MGR_LED_set(MGR_LED_GREEN);
			else
				MGR_LED_set(MGR_LED_BLUE);
#endif
			transition_to(UW_DOPPLER_BOOT_DEPLOY_LED);
		}
		return;
	}

	case UW_DOPPLER_BOOT_DEPLOY_LED:
	{
		/* Show deploy color for 2s then proceed (with timeout) */
		bool deploy_led_done = false;
#if defined(BSP_HAS_LED_RGB)
		deploy_led_done = (state_elapsed_ms() >= BOOT_DEPLOY_LED_MS ||
		                   state_elapsed_ms() > TIMEOUT_BOOT_DEPLOY_MS);
#else
		deploy_led_done = true;  /* No LED: skip immediately */
#endif
		if (deploy_led_done) {
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
#if defined(USE_UART_DRIVER)
			/* If deployed and no AT command received during boot window,
			 * disable UART to save power for multi-year deployment */
			if (deploy_mode && !boot_window_at_received) {
				MGR_LOG_DEBUG("[UW_DPL] Deploy mode: UART disabled\r\n");
				HAL_NVIC_DisableIRQ(LPUART1_IRQn);
				HAL_UART_DeInit(&hlpuart1);
				HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
			} else if (deploy_mode) {
				MGR_LOG_DEBUG("[UW_DPL] Deploy mode: UART kept (AT received)\r\n");
			}
#endif
			transition_to(UW_DOPPLER_INIT_MAC);
		}
		return;
	}

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

				/* Pre-warmup TCXO: start it now so it's stable by the time
				 * the first TX fires. Save current warmup setting so we can
				 * temporarily zero it for the first TX (avoid double wait).
				 * Re-entrance guard: don't overwrite saved value if a previous
				 * skip is still pending (would store the already-zeroed value).
				 */
				if (deploy_mode && !tcxo_first_tx_skip) {
					MCU_MISC_TCXO_get_warmup(&tcxo_warmup_saved_ms);
					MCU_MISC_TCXO_Force_State(true);
					tcxo_first_tx_skip = true;
				}
#if defined(BSP_HAS_LED_RGB)
				MGR_LED_blink(MGR_LED_CYAN, 3, 200, 200);
#endif
			} else {
				/* Went underwater, stop TX scheduling */
				MGR_LOG_DEBUG("[UW_DPL] Underwater, stopping TX\r\n");
				MGR_EVTLOG_log(EVT_SWS_UNDERWATER, MGR_SWS_getLastADC());
				reset_tx_scheduling();
				/* If a TCXO pre-warmup was pending, release it */
				if (tcxo_first_tx_skip) {
					MCU_MISC_TCXO_set_warmup(tcxo_warmup_saved_ms);
					MCU_MISC_TCXO_Force_State(false);
					tcxo_first_tx_skip = false;
				}
#if defined(BSP_HAS_LED_RGB)
				MGR_LED_blink(MGR_LED_YELLOW, 3, 200, 200);
#endif
			}

			/* Persist updated baselines/peak to flash (debounced).
			 * State changes are the natural moment to save: calibration just
			 * finished applying for the previous state.
			 */
			(void)MGR_NVM_saveCalibDebounced(NVM_CALIB_SAVE_MIN_INTERVAL_S);
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
				if (!MGR_BAT_isTxAllowedAt(last_vbat_mV)) {
					MGR_LOG_DEBUG("[UW_DPL] Battery low (%umV < %umV), TX inhibited\r\n",
						last_vbat_mV, MGR_BAT_getMinTxVoltage_mV());
					should_tx = false;
				}
#endif
			}
			if (should_tx) {
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
		if (!build_tx_payload(&appEvt)) {
			/* Radio config error - skip TX, go back to monitoring */
#if defined(BSP_HAS_LED_RGB)
			MGR_LED_off();
#endif
			transition_to(UW_DOPPLER_MONITORING);
			return;
		}

		/* For the very first TX after surface detection, the TCXO has been
		 * pre-warmed since UW->SURFACE; tell the MAC to skip its warmup wait
		 * by setting it to 0 just before the push. Restore on TX done.
		 */
		if (tcxo_first_tx_skip) {
			MCU_MISC_TCXO_set_warmup(0);
			MGR_LOG_DEBUG("[UW_DPL] First TX: TCXO warmup bypassed (was prewarmed)\r\n");
		}

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
			/* Restore TCXO warmup since the MAC never picked up the request */
			if (tcxo_first_tx_skip) {
				MCU_MISC_TCXO_set_warmup(tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = false;
			}
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
			/* Restore TCXO warmup: TX_DONE never fired, MAC may be stuck */
			if (tcxo_first_tx_skip) {
				MCU_MISC_TCXO_set_warmup(tcxo_warmup_saved_ms);
				tcxo_first_tx_skip = false;
			}
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

void KNS_APP_uw_doppler_restoreSwsBaselines(void)
{
	if (sws_retained.magic == SWS_RETAINED_MAGIC &&
	    sws_retained.air_baseline > 0 &&
	    sws_retained.water_baseline > sws_retained.air_baseline) {
		MGR_SWS_restoreBaselines(sws_retained.air_baseline,
		                         sws_retained.water_baseline);
		if (sws_retained.observed_peak_adc > 0)
			MGR_SWS_restoreObservedPeak(sws_retained.observed_peak_adc);
		MGR_LOG_DEBUG("[UW_DPL] Restored SWS: air=%u water=%u peak=%u\r\n",
			sws_retained.air_baseline, sws_retained.water_baseline,
			sws_retained.observed_peak_adc);
	}
}

/**
 * @}
 */
