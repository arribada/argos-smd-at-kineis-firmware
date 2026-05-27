/**
 * @file    mgr_nvm.h
 * @brief   NVM (Non-Volatile Memory) manager for persistent config storage
 *
 * Stores UW_DOPPLER configuration parameters in flash so they survive
 * power cycles. Uses the last free page in the FLASH_USER region.
 *
 * Version history:
 *   v1: TX, SWS, deploy, LED
 *   v2: + bat_min_tx_mV
 *   v3: + jitter_percent (TX), split surface/underwater intervals (SWS),
 *        adaptive sample delay bounds (SWS),
 *        adapted air/water baselines + observed peak (SWS calibration runtime)
 *   v4: + rate_window_s + rate_max_tx (persistent TX rate limiter config)
 *   v5: + lb_enter_mV/lb_exit_mV/lb_tx_interval_s/lb_tx_max_s/lb_tx_max_count
 *       (low-battery mode config — hysteretic switch to slower TX cadence)
 */

#ifndef MGR_NVM_H
#define MGR_NVM_H

#include <stdint.h>
#include <stdbool.h>
#include "kns_app_uw_doppler.h"
#include "mgr_sws.h"
#include "mgr_led.h"
#include "mgr_bat.h"

#define NVM_MAGIC   0x434F4E46UL  /* "CONF" */
#define NVM_VERSION 5

/**
 * @brief NVM config structure stored in flash (v3)
 *
 * Layout is designed for 64-bit aligned flash writes.
 * Total size must stay within one flash page (2KB).
 */
typedef struct {
	uint32_t magic;
	uint8_t  version;
	uint8_t  deploy_mode;
	uint8_t  led_mode;
	uint8_t  _pad0;
	/* TX config (12 bytes) */
	uint16_t tx_initial_interval_s;
	uint8_t  tx_growth_percent;
	uint8_t  tx_max_count;
	uint16_t tx_max_interval_s;
	uint8_t  tx_jitter_percent;       /**< v3: random +/- % on TX intervals (0=off) */
	uint8_t  _pad1;
	uint16_t tx_cooldown_s;           /**< v4: global post-TX quiet time (s, 0=off) */
	uint8_t  _pad1b[2];
	/* SWS config (28 bytes) */
	uint16_t sws_threshold_min;
	uint16_t sws_threshold_max;
	uint16_t sws_initial_air_baseline;
	uint16_t sws_initial_water_baseline;
	uint32_t sws_test_interval_surface_ms;     /**< v3: was test_interval_ms (surface state) */
	uint32_t sws_test_interval_underwater_ms;  /**< v3: new (underwater state) */
	uint32_t sws_max_dive_time_s;
	uint32_t sws_min_surface_time_s;
	uint16_t sws_sample_delay_min_us;          /**< v3 */
	uint16_t sws_sample_delay_max_us;          /**< v3 */
	uint16_t sws_sample_delay_default_us;      /**< v3 */
	uint8_t  sws_enabled;
	uint8_t  _pad2;
	/* SWS runtime calibration persistence (v3, 6 bytes + 2 pad) */
	uint16_t sws_run_air_baseline;             /**< Adapted air baseline */
	uint16_t sws_run_water_baseline;           /**< Adapted water baseline */
	uint16_t sws_run_observed_peak;            /**< Observed peak ADC */
	uint8_t  _pad3[2];
	/* Battery config */
	uint16_t bat_min_tx_mV;
	uint8_t  _pad4[2];
	/* v4: Rate limiter persistent config (8 bytes) */
	uint32_t rate_window_s;
	uint16_t rate_max_tx;
	uint8_t  _pad5[2];
	/* v5: Low-battery mode config (12 bytes) */
	uint16_t lb_enter_mV;
	uint16_t lb_exit_mV;
	uint16_t lb_tx_interval_s;
	uint16_t lb_tx_max_s;
	uint8_t  lb_tx_max_count;
	uint8_t  _pad6[3];
	uint32_t crc32;    /**< CRC32 of all bytes before this field (CRC-32/MPEG-2) */
} NVM_Config_t;

/** @brief Load config from flash. If invalid, keeps compile-time defaults. */
bool MGR_NVM_load(void);

/** @brief Save current config to flash.
 *  @return true on success
 */
bool MGR_NVM_save(void);

/** @brief Save only the SWS runtime calibration (baselines, peak) to flash,
 *         debounced to prevent flash wear. Called from app loop on state change.
 *  @param min_interval_s  Minimum interval between flash writes (e.g. 60 seconds).
 *  @return true if saved, false if skipped (too soon) or failed.
 */
bool MGR_NVM_saveCalibDebounced(uint32_t min_interval_s);

/** @brief Reset NVM to factory defaults (erases flash config) */
bool MGR_NVM_reset(void);

#endif /* MGR_NVM_H */
