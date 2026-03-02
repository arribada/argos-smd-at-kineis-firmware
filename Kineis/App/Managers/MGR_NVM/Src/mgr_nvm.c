/**
 * @file    mgr_nvm.c
 * @brief   NVM manager - persistent config storage in flash
 *
 * Stores/loads UW_DOPPLER configuration (TX config, SWS config,
 * deploy mode, LED mode) from the last free flash page (0x0803F800).
 * Uses CRC32 (MPEG-2, polynomial 0x04C11DB7) for integrity verification.
 *
 * Flash write sequence: erase page, write 64-bit doublewords.
 * On load: validate magic + version + CRC32 before applying config.
 * On failure: compile-time defaults are kept (no partial load).
 */

/**
 * @addtogroup MGR_NVM
 * @{
 */

#include "mgr_nvm.h"
#include "mcu_flash.h"
#include "mgr_log.h"
#include <string.h>
#include <stddef.h>

/* ---- CRC32 (MPEG-2, polynomial 0x04C11DB7) ---- */

/**
 * @brief Compute CRC-32/MPEG-2 (same as STM32 hardware CRC unit)
 *
 * Polynomial: 0x04C11DB7 (MSB-first, no reflection)
 * Initial value: 0xFFFFFFFF
 * Final XOR: none
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

bool MGR_NVM_load(void)
{
	NVM_Config_t cfg;

	if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[NVM] Flash read error\r\n");
		return false;
	}

	if (cfg.magic != NVM_MAGIC || cfg.version != NVM_VERSION) {
		MGR_LOG_DEBUG("[NVM] No valid config (magic=0x%08lx ver=%u), using defaults\r\n",
			cfg.magic, cfg.version);
		return false;
	}

	/* Verify CRC32 integrity */
	uint32_t computed_crc = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	if (computed_crc != cfg.crc32) {
		MGR_LOG_DEBUG("[NVM] CRC mismatch (stored=0x%08lx computed=0x%08lx), using defaults\r\n",
			cfg.crc32, computed_crc);
		return false;
	}

	/* Restore TX config */
	KNS_APP_UwDopplerTxCfg_t tx_cfg;
	tx_cfg.tx_initial_interval_s = cfg.tx_initial_interval_s;
	tx_cfg.tx_growth_percent     = cfg.tx_growth_percent;
	tx_cfg.tx_max_interval_s     = cfg.tx_max_interval_s;
	tx_cfg.tx_max_count          = cfg.tx_max_count;
	KNS_APP_uw_doppler_setTxCfg(&tx_cfg);

	/* Restore SWS config */
	MGR_SWS_Config_t sws_cfg;
	sws_cfg.threshold_min          = cfg.sws_threshold_min;
	sws_cfg.threshold_max          = cfg.sws_threshold_max;
	sws_cfg.initial_air_baseline   = cfg.sws_initial_air_baseline;
	sws_cfg.initial_water_baseline = cfg.sws_initial_water_baseline;
	sws_cfg.test_interval_ms       = cfg.sws_test_interval_ms;
	sws_cfg.max_dive_time_s        = cfg.sws_max_dive_time_s;
	sws_cfg.min_surface_time_s     = cfg.sws_min_surface_time_s;
	sws_cfg.enabled                = cfg.sws_enabled;
	MGR_SWS_setConfig(&sws_cfg);

	/* Restore app config */
	KNS_APP_uw_doppler_setDeployMode(cfg.deploy_mode);

#if defined(BSP_HAS_LED_RGB)
	MGR_LED_setMode((MGR_LED_Mode_t)cfg.led_mode);
#endif

	MGR_LOG_DEBUG("[NVM] Config loaded (CRC OK): deploy=%u interval=%us growth=%u%%\r\n",
		cfg.deploy_mode, cfg.tx_initial_interval_s, cfg.tx_growth_percent);

	return true;
}

bool MGR_NVM_save(void)
{
	NVM_Config_t cfg;
	memset(&cfg, 0, sizeof(cfg));

	cfg.magic   = NVM_MAGIC;
	cfg.version = NVM_VERSION;

	/* TX config */
	KNS_APP_UwDopplerTxCfg_t tx_cfg = KNS_APP_uw_doppler_getTxCfg();
	cfg.tx_initial_interval_s = tx_cfg.tx_initial_interval_s;
	cfg.tx_growth_percent     = tx_cfg.tx_growth_percent;
	cfg.tx_max_interval_s     = tx_cfg.tx_max_interval_s;
	cfg.tx_max_count          = tx_cfg.tx_max_count;

	/* SWS config */
	MGR_SWS_Config_t sws_cfg = MGR_SWS_getConfig();
	cfg.sws_threshold_min          = sws_cfg.threshold_min;
	cfg.sws_threshold_max          = sws_cfg.threshold_max;
	cfg.sws_initial_air_baseline   = sws_cfg.initial_air_baseline;
	cfg.sws_initial_water_baseline = sws_cfg.initial_water_baseline;
	cfg.sws_test_interval_ms       = sws_cfg.test_interval_ms;
	cfg.sws_max_dive_time_s        = sws_cfg.max_dive_time_s;
	cfg.sws_min_surface_time_s     = sws_cfg.min_surface_time_s;
	cfg.sws_enabled                = sws_cfg.enabled;

	/* App config */
	cfg.deploy_mode = KNS_APP_uw_doppler_getDeployMode();

#if defined(BSP_HAS_LED_RGB)
	cfg.led_mode = (uint8_t)MGR_LED_getMode();
#endif

	/* Compute CRC32 over all fields before crc32 */
	cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));

	if (MCU_FLASH_write(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[NVM] Flash write error\r\n");
		return false;
	}

	MGR_LOG_DEBUG("[NVM] Config saved (CRC=0x%08lx)\r\n", cfg.crc32);
	return true;
}

bool MGR_NVM_reset(void)
{
	/* Write all 0xFF to invalidate the magic */
	NVM_Config_t cfg;
	memset(&cfg, 0xFF, sizeof(cfg));

	if (MCU_FLASH_write(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[NVM] Flash reset error\r\n");
		return false;
	}

	MGR_LOG_DEBUG("[NVM] Config reset to defaults\r\n");
	return true;
}

/**
 * @}
 */
