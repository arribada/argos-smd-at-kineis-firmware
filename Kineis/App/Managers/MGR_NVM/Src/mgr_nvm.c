/**
 * @file    mgr_nvm.c
 * @brief   NVM manager - persistent config storage in flash
 *
 * Stores/loads UW_DOPPLER configuration (TX config, SWS config,
 * deploy mode, LED mode, battery threshold) from the last free flash
 * page (0x0803F800).
 * Uses CRC32 (MPEG-2, polynomial 0x04C11DB7) for integrity verification.
 *
 * Flash write sequence: erase page, write 64-bit doublewords.
 * On load: validate magic + version + CRC32 before applying config.
 * On failure: compile-time defaults are kept (no partial load).
 *
 * Version history:
 *   v1: TX, SWS, deploy, LED
 *   v2: Added bat_min_tx_mV (auto-migrated from v1)
 */

/**
 * @addtogroup MGR_NVM
 * @{
 */

#include "mgr_nvm.h"
#include "mcu_flash.h"
#include "mgr_wdg.h"
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

/* ---- V1 layout for migration ---- */

typedef struct {
	uint32_t magic;
	uint8_t  version;
	uint8_t  deploy_mode;
	uint8_t  led_mode;
	uint8_t  _pad0;
	uint16_t tx_initial_interval_s;
	uint8_t  tx_growth_percent;
	uint8_t  tx_max_count;
	uint16_t tx_max_interval_s;
	uint8_t  _pad1[2];
	uint16_t sws_threshold_min;
	uint16_t sws_threshold_max;
	uint16_t sws_initial_air_baseline;
	uint16_t sws_initial_water_baseline;
	uint32_t sws_test_interval_ms;
	uint32_t sws_max_dive_time_s;
	uint32_t sws_min_surface_time_s;
	uint8_t  sws_enabled;
	uint8_t  _pad2[3];
	uint32_t crc32;
} NVM_Config_v1_t;

/**
 * @brief Apply a loaded config to all modules
 */
static bool validate_config(const NVM_Config_t *cfg)
{
	/* TX: intervals must be non-zero, initial <= max */
	if (cfg->tx_initial_interval_s == 0 || cfg->tx_max_interval_s == 0)
		return false;
	if (cfg->tx_initial_interval_s > cfg->tx_max_interval_s)
		return false;
	if (cfg->tx_max_count == 0)
		return false;

	/* SWS: thresholds ordered, interval non-zero */
	if (cfg->sws_threshold_min >= cfg->sws_threshold_max)
		return false;
	if (cfg->sws_initial_water_baseline <= cfg->sws_initial_air_baseline)
		return false;
	if (cfg->sws_test_interval_ms == 0)
		return false;

	return true;
}

static void apply_config(const NVM_Config_t *cfg)
{
	/* Restore TX config */
	KNS_APP_UwDopplerTxCfg_t tx_cfg;
	tx_cfg.tx_initial_interval_s = cfg->tx_initial_interval_s;
	tx_cfg.tx_growth_percent     = cfg->tx_growth_percent;
	tx_cfg.tx_max_interval_s     = cfg->tx_max_interval_s;
	tx_cfg.tx_max_count          = cfg->tx_max_count;
	KNS_APP_uw_doppler_setTxCfg(&tx_cfg);

	/* Restore SWS config */
	MGR_SWS_Config_t sws_cfg;
	sws_cfg.threshold_min          = cfg->sws_threshold_min;
	sws_cfg.threshold_max          = cfg->sws_threshold_max;
	sws_cfg.initial_air_baseline   = cfg->sws_initial_air_baseline;
	sws_cfg.initial_water_baseline = cfg->sws_initial_water_baseline;
	sws_cfg.test_interval_ms       = cfg->sws_test_interval_ms;
	sws_cfg.max_dive_time_s        = cfg->sws_max_dive_time_s;
	sws_cfg.min_surface_time_s     = cfg->sws_min_surface_time_s;
	sws_cfg.enabled                = cfg->sws_enabled;
	MGR_SWS_setConfig(&sws_cfg);

	/* Restore app config */
	KNS_APP_uw_doppler_setDeployMode(cfg->deploy_mode);

#if defined(BSP_HAS_LED_RGB)
	MGR_LED_setMode((MGR_LED_Mode_t)cfg->led_mode);
#endif

#if defined(BSP_HAS_VBAT_ADC)
	MGR_BAT_setMinTxVoltage_mV(cfg->bat_min_tx_mV);
#endif
}

bool MGR_NVM_load(void)
{
	NVM_Config_t cfg;

	if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[NVM] Flash read error\r\n");
		return false;
	}

	if (cfg.magic != NVM_MAGIC) {
		MGR_LOG_DEBUG("[NVM] No valid config (magic=0x%08lx), using defaults\r\n",
			cfg.magic);
		return false;
	}

	/* Handle version migration */
	if (cfg.version == 1) {
		/* Read as v1 layout and migrate */
		NVM_Config_v1_t v1;
		if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &v1, sizeof(v1)) != KNS_STATUS_OK)
			return false;

		uint32_t computed_crc = nvm_crc32(&v1, offsetof(NVM_Config_v1_t, crc32));
		if (computed_crc != v1.crc32) {
			MGR_LOG_DEBUG("[NVM] v1 CRC mismatch, using defaults\r\n");
			return false;
		}

		/* Migrate: copy common fields, add v2 defaults */
		memset(&cfg, 0, sizeof(cfg));
		cfg.magic   = NVM_MAGIC;
		cfg.version = NVM_VERSION;
		cfg.deploy_mode              = v1.deploy_mode;
		cfg.led_mode                 = v1.led_mode;
		cfg.tx_initial_interval_s    = v1.tx_initial_interval_s;
		cfg.tx_growth_percent        = v1.tx_growth_percent;
		cfg.tx_max_count             = v1.tx_max_count;
		cfg.tx_max_interval_s        = v1.tx_max_interval_s;
		cfg.sws_threshold_min        = v1.sws_threshold_min;
		cfg.sws_threshold_max        = v1.sws_threshold_max;
		cfg.sws_initial_air_baseline = v1.sws_initial_air_baseline;
		cfg.sws_initial_water_baseline = v1.sws_initial_water_baseline;
		cfg.sws_test_interval_ms     = v1.sws_test_interval_ms;
		cfg.sws_max_dive_time_s      = v1.sws_max_dive_time_s;
		cfg.sws_min_surface_time_s   = v1.sws_min_surface_time_s;
		cfg.sws_enabled              = v1.sws_enabled;
		cfg.bat_min_tx_mV            = MGR_BAT_DEFAULT_MIN_TX_MV;

		MGR_LOG_DEBUG("[NVM] Migrated v1 -> v2 (bat_min=%umV)\r\n",
			cfg.bat_min_tx_mV);

		if (!validate_config(&cfg)) {
			MGR_LOG_DEBUG("[NVM] v1 migrated config invalid, using defaults\r\n");
			return false;
		}
		apply_config(&cfg);
		return true;
	}

	if (cfg.version != NVM_VERSION) {
		MGR_LOG_DEBUG("[NVM] Unknown version %u, using defaults\r\n", cfg.version);
		return false;
	}

	/* Verify CRC32 integrity */
	uint32_t computed_crc = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	if (computed_crc != cfg.crc32) {
		MGR_LOG_DEBUG("[NVM] CRC mismatch (stored=0x%08lx computed=0x%08lx), using defaults\r\n",
			cfg.crc32, computed_crc);
		return false;
	}

	if (!validate_config(&cfg)) {
		MGR_LOG_DEBUG("[NVM] Config values invalid, using defaults\r\n");
		return false;
	}

	apply_config(&cfg);

	MGR_LOG_DEBUG("[NVM] Config loaded (v%u CRC OK): deploy=%u interval=%us bat_min=%umV\r\n",
		cfg.version, cfg.deploy_mode, cfg.tx_initial_interval_s, cfg.bat_min_tx_mV);

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

#if defined(BSP_HAS_VBAT_ADC)
	cfg.bat_min_tx_mV = MGR_BAT_getMinTxVoltage_mV();
#endif

	/* Compute CRC32 over all fields before crc32 */
	cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));

	/* Refresh watchdog before flash erase+write (can take ~20ms per page) */
	MGR_WDG_refresh();

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
