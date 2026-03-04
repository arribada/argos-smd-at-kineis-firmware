/**
 * @file    mgr_nvm.h
 * @brief   NVM (Non-Volatile Memory) manager for persistent config storage
 *
 * Stores UW_DOPPLER configuration parameters in flash so they survive
 * power cycles. Uses the last free page in the FLASH_USER region.
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
#define NVM_VERSION 2

/**
 * @brief NVM config structure stored in flash
 *
 * Layout is designed for 64-bit aligned flash writes.
 * Total size must stay within one flash page (2KB).
 *
 * @note Version 2 added: bat_min_tx_mV field.
 *       Version 1 configs are auto-migrated on load (bat_min_tx_mV = default).
 */
typedef struct {
	uint32_t magic;
	uint8_t  version;
	uint8_t  deploy_mode;
	uint8_t  led_mode;
	uint8_t  _pad0;
	/* TX config (6 bytes + 2 pad) */
	uint16_t tx_initial_interval_s;
	uint8_t  tx_growth_percent;
	uint8_t  tx_max_count;
	uint16_t tx_max_interval_s;
	uint8_t  _pad1[2];
	/* SWS config */
	uint16_t sws_threshold_min;
	uint16_t sws_threshold_max;
	uint16_t sws_initial_air_baseline;
	uint16_t sws_initial_water_baseline;
	uint32_t sws_test_interval_ms;
	uint32_t sws_max_dive_time_s;
	uint32_t sws_min_surface_time_s;
	uint8_t  sws_enabled;
	uint8_t  _pad2[1];
	/* Battery config (v2) */
	uint16_t bat_min_tx_mV;        /**< Min battery voltage for TX (0 = disabled) */
	uint32_t crc32;    /**< CRC32 of all bytes before this field (CRC-32/MPEG-2) */
} NVM_Config_t;

/** @brief Load config from flash. If invalid, keeps compile-time defaults. */
bool MGR_NVM_load(void);

/** @brief Save current config to flash.
 *  @return true on success
 */
bool MGR_NVM_save(void);

/** @brief Reset NVM to factory defaults (erases flash config) */
bool MGR_NVM_reset(void);

#endif /* MGR_NVM_H */
