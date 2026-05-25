/**
 * @file    mgr_nvm.c
 * @brief   NVM manager - persistent config storage in flash
 *
 * Stores/loads UW_DOPPLER configuration (TX config, SWS config + runtime
 * baselines, deploy mode, LED mode, battery threshold) from the last free
 * flash page (0x0803F800).
 * Uses CRC32 (MPEG-2, polynomial 0x04C11DB7) for integrity verification.
 *
 * Version history:
 *   v1: TX, SWS, deploy, LED
 *   v2: + bat_min_tx_mV
 *   v3: + tx_jitter_percent, split surface/underwater intervals,
 *        sample delay bounds, runtime calibration baselines+peak
 */

/**
 * @addtogroup MGR_NVM
 * @{
 */

#include "mgr_nvm.h"
#include "mcu_flash.h"
#include "mgr_wdg.h"
#include "mgr_log.h"
#include "stm32wlxx_hal.h"
#include <string.h>
#include <stddef.h>

/* ---- CRC32 (MPEG-2, polynomial 0x04C11DB7) ---- */

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

/* ---- Legacy layouts for migration ---- */

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
	uint16_t bat_min_tx_mV;
	uint8_t  _pad3[2];
	uint32_t crc32;
} NVM_Config_v2_t;

/* ---- Last save timestamp (for debouncing) ---- */

static uint32_t last_save_tick = 0;

/* ---- Validation & apply ---- */

static bool validate_config(const NVM_Config_t *cfg)
{
	if (cfg->tx_initial_interval_s == 0 || cfg->tx_max_interval_s == 0)
		return false;
	if (cfg->tx_initial_interval_s > cfg->tx_max_interval_s)
		return false;
	if (cfg->tx_max_count == 0)
		return false;
	if (cfg->tx_jitter_percent > 50)  /* sanity: jitter capped at 50% */
		return false;

	if (cfg->sws_threshold_min >= cfg->sws_threshold_max)
		return false;
	if (cfg->sws_initial_water_baseline <= cfg->sws_initial_air_baseline)
		return false;
	if (cfg->sws_test_interval_surface_ms == 0 || cfg->sws_test_interval_underwater_ms == 0)
		return false;
	if (cfg->sws_sample_delay_min_us == 0 ||
	    cfg->sws_sample_delay_max_us < cfg->sws_sample_delay_min_us)
		return false;
	if (cfg->sws_sample_delay_default_us < cfg->sws_sample_delay_min_us ||
	    cfg->sws_sample_delay_default_us > cfg->sws_sample_delay_max_us)
		return false;

	return true;
}

static void apply_config(const NVM_Config_t *cfg)
{
	/* TX config */
	KNS_APP_UwDopplerTxCfg_t tx_cfg;
	tx_cfg.tx_initial_interval_s = cfg->tx_initial_interval_s;
	tx_cfg.tx_growth_percent     = cfg->tx_growth_percent;
	tx_cfg.tx_max_interval_s     = cfg->tx_max_interval_s;
	tx_cfg.tx_max_count          = cfg->tx_max_count;
	tx_cfg.tx_jitter_percent     = cfg->tx_jitter_percent;
	KNS_APP_uw_doppler_setTxCfg(&tx_cfg);

	/* SWS config */
	MGR_SWS_Config_t sws_cfg;
	sws_cfg.threshold_min                 = cfg->sws_threshold_min;
	sws_cfg.threshold_max                 = cfg->sws_threshold_max;
	sws_cfg.initial_air_baseline          = cfg->sws_initial_air_baseline;
	sws_cfg.initial_water_baseline        = cfg->sws_initial_water_baseline;
	sws_cfg.test_interval_surface_ms      = cfg->sws_test_interval_surface_ms;
	sws_cfg.test_interval_underwater_ms   = cfg->sws_test_interval_underwater_ms;
	sws_cfg.max_dive_time_s               = cfg->sws_max_dive_time_s;
	sws_cfg.min_surface_time_s            = cfg->sws_min_surface_time_s;
	sws_cfg.sample_delay_min_us           = cfg->sws_sample_delay_min_us;
	sws_cfg.sample_delay_max_us           = cfg->sws_sample_delay_max_us;
	sws_cfg.sample_delay_default_us       = cfg->sws_sample_delay_default_us;
	sws_cfg.enabled                       = cfg->sws_enabled;
	MGR_SWS_setConfig(&sws_cfg);

	/* SWS runtime calibration */
	if (cfg->sws_run_air_baseline > 0 &&
	    cfg->sws_run_water_baseline > cfg->sws_run_air_baseline) {
		MGR_SWS_restoreBaselines(cfg->sws_run_air_baseline,
		                         cfg->sws_run_water_baseline);
	}
	if (cfg->sws_run_observed_peak > 0)
		MGR_SWS_restoreObservedPeak(cfg->sws_run_observed_peak);

	/* App config */
	KNS_APP_uw_doppler_setDeployMode(cfg->deploy_mode);

#if defined(BSP_HAS_LED_RGB)
	MGR_LED_setMode((MGR_LED_Mode_t)cfg->led_mode);
#endif

#if defined(BSP_HAS_VBAT_ADC)
	MGR_BAT_setMinTxVoltage_mV(cfg->bat_min_tx_mV);
#endif
}

/* ---- Build current config snapshot from all modules ---- */

static void gather_config(NVM_Config_t *cfg)
{
	memset(cfg, 0, sizeof(*cfg));
	cfg->magic   = NVM_MAGIC;
	cfg->version = NVM_VERSION;

	/* TX */
	KNS_APP_UwDopplerTxCfg_t tx_cfg = KNS_APP_uw_doppler_getTxCfg();
	cfg->tx_initial_interval_s = tx_cfg.tx_initial_interval_s;
	cfg->tx_growth_percent     = tx_cfg.tx_growth_percent;
	cfg->tx_max_interval_s     = tx_cfg.tx_max_interval_s;
	cfg->tx_max_count          = tx_cfg.tx_max_count;
	cfg->tx_jitter_percent     = tx_cfg.tx_jitter_percent;

	/* SWS */
	MGR_SWS_Config_t sws_cfg = MGR_SWS_getConfig();
	cfg->sws_threshold_min                = sws_cfg.threshold_min;
	cfg->sws_threshold_max                = sws_cfg.threshold_max;
	cfg->sws_initial_air_baseline         = sws_cfg.initial_air_baseline;
	cfg->sws_initial_water_baseline       = sws_cfg.initial_water_baseline;
	cfg->sws_test_interval_surface_ms     = sws_cfg.test_interval_surface_ms;
	cfg->sws_test_interval_underwater_ms  = sws_cfg.test_interval_underwater_ms;
	cfg->sws_max_dive_time_s              = sws_cfg.max_dive_time_s;
	cfg->sws_min_surface_time_s           = sws_cfg.min_surface_time_s;
	cfg->sws_sample_delay_min_us          = sws_cfg.sample_delay_min_us;
	cfg->sws_sample_delay_max_us          = sws_cfg.sample_delay_max_us;
	cfg->sws_sample_delay_default_us      = sws_cfg.sample_delay_default_us;
	cfg->sws_enabled                      = sws_cfg.enabled;

	/* SWS runtime calibration */
	cfg->sws_run_air_baseline   = MGR_SWS_getAirBaseline();
	cfg->sws_run_water_baseline = MGR_SWS_getWaterBaseline();
	cfg->sws_run_observed_peak  = MGR_SWS_getObservedPeak();

	/* App */
	cfg->deploy_mode = KNS_APP_uw_doppler_getDeployMode();

#if defined(BSP_HAS_LED_RGB)
	cfg->led_mode = (uint8_t)MGR_LED_getMode();
#endif

#if defined(BSP_HAS_VBAT_ADC)
	cfg->bat_min_tx_mV = MGR_BAT_getMinTxVoltage_mV();
#endif
}

/* ---- Migration helpers ---- */

static void migrate_v1_to_v3(const NVM_Config_v1_t *v1, NVM_Config_t *out)
{
	memset(out, 0, sizeof(*out));
	out->magic   = NVM_MAGIC;
	out->version = NVM_VERSION;
	out->deploy_mode             = v1->deploy_mode;
	out->led_mode                = v1->led_mode;
	out->tx_initial_interval_s   = v1->tx_initial_interval_s;
	out->tx_growth_percent       = v1->tx_growth_percent;
	out->tx_max_count            = v1->tx_max_count;
	out->tx_max_interval_s       = v1->tx_max_interval_s;
	out->tx_jitter_percent       = 0;  /* v3 default */
	out->sws_threshold_min       = v1->sws_threshold_min;
	out->sws_threshold_max       = v1->sws_threshold_max;
	out->sws_initial_air_baseline   = v1->sws_initial_air_baseline;
	out->sws_initial_water_baseline = v1->sws_initial_water_baseline;
	out->sws_test_interval_surface_ms    = v1->sws_test_interval_ms * 5;  /* surface = 5x slower default */
	out->sws_test_interval_underwater_ms = v1->sws_test_interval_ms;       /* underwater = original cadence */
	out->sws_max_dive_time_s     = v1->sws_max_dive_time_s;
	out->sws_min_surface_time_s  = v1->sws_min_surface_time_s;
	out->sws_sample_delay_min_us     = 200;
	out->sws_sample_delay_max_us     = 1000;
	out->sws_sample_delay_default_us = 500;
	out->sws_enabled             = v1->sws_enabled;
	out->bat_min_tx_mV           = MGR_BAT_DEFAULT_MIN_TX_MV;
	/* Runtime calibration left at 0: not stored in v1 */
}

static void migrate_v2_to_v3(const NVM_Config_v2_t *v2, NVM_Config_t *out)
{
	memset(out, 0, sizeof(*out));
	out->magic   = NVM_MAGIC;
	out->version = NVM_VERSION;
	out->deploy_mode             = v2->deploy_mode;
	out->led_mode                = v2->led_mode;
	out->tx_initial_interval_s   = v2->tx_initial_interval_s;
	out->tx_growth_percent       = v2->tx_growth_percent;
	out->tx_max_count            = v2->tx_max_count;
	out->tx_max_interval_s       = v2->tx_max_interval_s;
	out->tx_jitter_percent       = 0;
	out->sws_threshold_min       = v2->sws_threshold_min;
	out->sws_threshold_max       = v2->sws_threshold_max;
	out->sws_initial_air_baseline   = v2->sws_initial_air_baseline;
	out->sws_initial_water_baseline = v2->sws_initial_water_baseline;
	out->sws_test_interval_surface_ms    = v2->sws_test_interval_ms * 5;
	out->sws_test_interval_underwater_ms = v2->sws_test_interval_ms;
	out->sws_max_dive_time_s     = v2->sws_max_dive_time_s;
	out->sws_min_surface_time_s  = v2->sws_min_surface_time_s;
	out->sws_sample_delay_min_us     = 200;
	out->sws_sample_delay_max_us     = 1000;
	out->sws_sample_delay_default_us = 500;
	out->sws_enabled             = v2->sws_enabled;
	out->bat_min_tx_mV           = v2->bat_min_tx_mV;
}

/* ---- Public API ---- */

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

	/* Migration paths */
	if (cfg.version == 1) {
		NVM_Config_v1_t v1;
		if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &v1, sizeof(v1)) != KNS_STATUS_OK)
			return false;
		uint32_t computed = nvm_crc32(&v1, offsetof(NVM_Config_v1_t, crc32));
		if (computed != v1.crc32) {
			MGR_LOG_DEBUG("[NVM] v1 CRC mismatch\r\n");
			return false;
		}
		migrate_v1_to_v3(&v1, &cfg);
		MGR_LOG_DEBUG("[NVM] Migrated v1 -> v3\r\n");
		if (!validate_config(&cfg)) return false;
		apply_config(&cfg);
		return true;
	}

	if (cfg.version == 2) {
		NVM_Config_v2_t v2;
		if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &v2, sizeof(v2)) != KNS_STATUS_OK)
			return false;
		uint32_t computed = nvm_crc32(&v2, offsetof(NVM_Config_v2_t, crc32));
		if (computed != v2.crc32) {
			MGR_LOG_DEBUG("[NVM] v2 CRC mismatch\r\n");
			return false;
		}
		migrate_v2_to_v3(&v2, &cfg);
		MGR_LOG_DEBUG("[NVM] Migrated v2 -> v3\r\n");
		if (!validate_config(&cfg)) return false;
		apply_config(&cfg);
		return true;
	}

	if (cfg.version != NVM_VERSION) {
		MGR_LOG_DEBUG("[NVM] Unknown version %u, using defaults\r\n", cfg.version);
		return false;
	}

	/* v3: verify CRC32 */
	uint32_t computed_crc = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	if (computed_crc != cfg.crc32) {
		MGR_LOG_DEBUG("[NVM] v3 CRC mismatch (stored=0x%08lx computed=0x%08lx)\r\n",
			cfg.crc32, computed_crc);
		return false;
	}

	if (!validate_config(&cfg)) {
		MGR_LOG_DEBUG("[NVM] Config values invalid, using defaults\r\n");
		return false;
	}

	apply_config(&cfg);

	MGR_LOG_DEBUG("[NVM] v3 loaded: deploy=%u tx=%us jit=%u%% sws_surf=%lums sws_uw=%lums "
		"run_air=%u run_water=%u peak=%u\r\n",
		cfg.deploy_mode, cfg.tx_initial_interval_s, cfg.tx_jitter_percent,
		(unsigned long)cfg.sws_test_interval_surface_ms,
		(unsigned long)cfg.sws_test_interval_underwater_ms,
		cfg.sws_run_air_baseline, cfg.sws_run_water_baseline,
		cfg.sws_run_observed_peak);

	return true;
}

bool MGR_NVM_save(void)
{
	NVM_Config_t cfg;
	gather_config(&cfg);

	cfg.crc32 = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));

	MGR_WDG_refresh();

	if (MCU_FLASH_write(FLASH_NVM_CONFIG_ADDR, &cfg, sizeof(cfg)) != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("[NVM] Flash write error\r\n");
		return false;
	}

	last_save_tick = HAL_GetTick();
	MGR_SWS_clearCalibDirty();
	MGR_LOG_DEBUG("[NVM] Saved (CRC=0x%08lx)\r\n", cfg.crc32);
	return true;
}

bool MGR_NVM_saveCalibDebounced(uint32_t min_interval_s)
{
	if (!MGR_SWS_calibDirty())
		return false;

	uint32_t now = HAL_GetTick();
	uint32_t since_last_ms = now - last_save_tick;
	if (last_save_tick != 0 && since_last_ms < min_interval_s * 1000)
		return false;  /* too soon */

	return MGR_NVM_save();
}

bool MGR_NVM_reset(void)
{
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
