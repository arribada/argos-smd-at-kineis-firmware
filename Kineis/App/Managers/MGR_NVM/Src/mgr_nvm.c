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
#include "mgr_rate.h"
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

/* v4 layout (was current in Sprint 1; lacks v5 LB-mode fields). Used for
 * migration only — production code always reads/writes the latest version. */
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
	uint8_t  tx_jitter_percent;
	uint8_t  _pad1;
	uint16_t tx_cooldown_s;
	uint8_t  _pad1b[2];
	uint16_t sws_threshold_min;
	uint16_t sws_threshold_max;
	uint16_t sws_initial_air_baseline;
	uint16_t sws_initial_water_baseline;
	uint32_t sws_test_interval_surface_ms;
	uint32_t sws_test_interval_underwater_ms;
	uint32_t sws_max_dive_time_s;
	uint32_t sws_min_surface_time_s;
	uint16_t sws_sample_delay_min_us;
	uint16_t sws_sample_delay_max_us;
	uint16_t sws_sample_delay_default_us;
	uint8_t  sws_enabled;
	uint8_t  _pad2;
	uint16_t sws_run_air_baseline;
	uint16_t sws_run_water_baseline;
	uint16_t sws_run_observed_peak;
	uint8_t  _pad3[2];
	uint16_t bat_min_tx_mV;
	uint8_t  _pad4[2];
	uint32_t rate_window_s;
	uint16_t rate_max_tx;
	uint8_t  _pad5[2];
	uint32_t crc32;
} NVM_Config_v4_t;

/* v3 layout (current layout WITHOUT the v4 rate-limiter fields). Used for
 * migration only — production code always reads/writes the latest version. */
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
	uint8_t  tx_jitter_percent;
	uint8_t  _pad1;
	uint16_t sws_threshold_min;
	uint16_t sws_threshold_max;
	uint16_t sws_initial_air_baseline;
	uint16_t sws_initial_water_baseline;
	uint32_t sws_test_interval_surface_ms;
	uint32_t sws_test_interval_underwater_ms;
	uint32_t sws_max_dive_time_s;
	uint32_t sws_min_surface_time_s;
	uint16_t sws_sample_delay_min_us;
	uint16_t sws_sample_delay_max_us;
	uint16_t sws_sample_delay_default_us;
	uint8_t  sws_enabled;
	uint8_t  _pad2;
	uint16_t sws_run_air_baseline;
	uint16_t sws_run_water_baseline;
	uint16_t sws_run_observed_peak;
	uint8_t  _pad3[2];
	uint16_t bat_min_tx_mV;
	uint8_t  _pad4[2];
	uint32_t crc32;
} NVM_Config_v3_t;

/* ---- Last save timestamp (for debouncing) ---- */

static uint32_t last_save_tick = 0;

/* ---- Validation & apply ---- */

static bool validate_config(const NVM_Config_t *cfg)
{
	/* TX scheduling */
	if (cfg->tx_initial_interval_s == 0 || cfg->tx_max_interval_s == 0)
		return false;
	if (cfg->tx_initial_interval_s > cfg->tx_max_interval_s)
		return false;
	/* tx_max_count==0 means "unlimited" — explicitly allowed (bug fix v5). */
	if (cfg->tx_jitter_percent > 50)
		return false;
	/* cooldown is allowed to be 0 (disabled). Upper bound = u16 max. */

	/* SWS — leave existing checks intact. */
	if (cfg->sws_threshold_min >= cfg->sws_threshold_max)
		return false;
	if (cfg->sws_initial_water_baseline <= cfg->sws_initial_air_baseline)
		return false;
	if (cfg->sws_test_interval_surface_ms == 0 ||
	    cfg->sws_test_interval_underwater_ms == 0)
		return false;
	if (cfg->sws_sample_delay_min_us == 0 ||
	    cfg->sws_sample_delay_max_us < cfg->sws_sample_delay_min_us)
		return false;
	if (cfg->sws_sample_delay_default_us < cfg->sws_sample_delay_min_us ||
	    cfg->sws_sample_delay_default_us > cfg->sws_sample_delay_max_us)
		return false;
	/* Sanity bounds on SWS intervals — pathological values brick the loop. */
	if (cfg->sws_max_dive_time_s > 7u * 86400u)        /* > 7 days makes no sense */
		return false;
	if (cfg->sws_min_surface_time_s > 3600u)           /* > 1h makes no sense */
		return false;

	/* Battery */
	if (cfg->bat_min_tx_mV > 5000u)                    /* > 5 V → bad sample */
		return false;

	/* Rate limiter (0/0 = "use compile-time defaults") */
	if (cfg->rate_window_s != 0 || cfg->rate_max_tx != 0) {
		if (cfg->rate_window_s < 60u ||
		    cfg->rate_window_s > 7u * 86400u)
			return false;
		if (cfg->rate_max_tx == 0 || cfg->rate_max_tx > 256u)
			return false;
	}

	/* LB mode (enter_mV==0 = LB disabled, exit_mV ignored in that case) */
	if (cfg->lb_enter_mV != 0) {
		if (cfg->lb_enter_mV < 1500u || cfg->lb_enter_mV > 5000u)
			return false;
		if (cfg->lb_exit_mV <= cfg->lb_enter_mV ||
		    cfg->lb_exit_mV > 5000u)
			return false;
	}
	/* When at least one LB timing field is set, all must be coherent. */
	if (cfg->lb_tx_interval_s != 0 || cfg->lb_tx_max_s != 0) {
		if (cfg->lb_tx_interval_s == 0 ||
		    cfg->lb_tx_max_s == 0 ||
		    cfg->lb_tx_interval_s > cfg->lb_tx_max_s)
			return false;
	}

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
	tx_cfg.tx_cooldown_s         = cfg->tx_cooldown_s;
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

	/* Rate limiter config. 0/0 means "no value persisted yet" (e.g. fresh
	 * boot or migrated from v3) — keep MGR_RATE on its compile-time defaults
	 * rather than clamping to a tiny window. */
	if (cfg->rate_window_s != 0 && cfg->rate_max_tx != 0)
		MGR_RATE_setConfig(cfg->rate_window_s, cfg->rate_max_tx);

	/* LB mode config (v5). Skip if migrated/uninitialised (all zeros): keep
	 * compile-time defaults rather than disabling the feature accidentally. */
	if (cfg->lb_enter_mV != 0 || cfg->lb_exit_mV != 0 ||
	    cfg->lb_tx_interval_s != 0) {
		KNS_APP_UwDopplerLbCfg_t lbc = {
			.lb_enter_mV       = cfg->lb_enter_mV,
			.lb_exit_mV        = cfg->lb_exit_mV,
			.lb_tx_interval_s  = cfg->lb_tx_interval_s,
			.lb_tx_max_s       = cfg->lb_tx_max_s,
			.lb_tx_max_count   = cfg->lb_tx_max_count,
		};
		KNS_APP_uw_doppler_setLbCfg(&lbc);
	}

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
	cfg->tx_cooldown_s         = tx_cfg.tx_cooldown_s;

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

	/* Rate limiter snapshot */
	{
		uint32_t w = 0;
		uint16_t m = 0;
		MGR_RATE_getConfig(&w, &m, NULL);
		cfg->rate_window_s = w;
		cfg->rate_max_tx   = m;
	}

	/* LB mode snapshot (v5) */
	{
		KNS_APP_UwDopplerLbCfg_t lbc = KNS_APP_uw_doppler_getLbCfg();
		cfg->lb_enter_mV       = lbc.lb_enter_mV;
		cfg->lb_exit_mV        = lbc.lb_exit_mV;
		cfg->lb_tx_interval_s  = lbc.lb_tx_interval_s;
		cfg->lb_tx_max_s       = lbc.lb_tx_max_s;
		cfg->lb_tx_max_count   = lbc.lb_tx_max_count;
	}
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

static void migrate_v4_to_v5(const NVM_Config_v4_t *v4, NVM_Config_t *out)
{
	/* v4 is a strict prefix of v5: same fields, plus the new LB-mode block.
	 * Leave the LB fields at 0 — apply_config() treats that as "keep
	 * KNS_APP_uw_doppler_setLbCfg's compile-time defaults" so LB stays
	 * enabled with sensible thresholds on first boot after upgrade. */
	memset(out, 0, sizeof(*out));
	out->magic                          = NVM_MAGIC;
	out->version                        = NVM_VERSION;
	out->deploy_mode                    = v4->deploy_mode;
	out->led_mode                       = v4->led_mode;
	out->tx_initial_interval_s          = v4->tx_initial_interval_s;
	out->tx_growth_percent              = v4->tx_growth_percent;
	out->tx_max_count                   = v4->tx_max_count;
	out->tx_max_interval_s              = v4->tx_max_interval_s;
	out->tx_jitter_percent              = v4->tx_jitter_percent;
	out->tx_cooldown_s                  = v4->tx_cooldown_s;
	out->sws_threshold_min              = v4->sws_threshold_min;
	out->sws_threshold_max              = v4->sws_threshold_max;
	out->sws_initial_air_baseline       = v4->sws_initial_air_baseline;
	out->sws_initial_water_baseline     = v4->sws_initial_water_baseline;
	out->sws_test_interval_surface_ms   = v4->sws_test_interval_surface_ms;
	out->sws_test_interval_underwater_ms= v4->sws_test_interval_underwater_ms;
	out->sws_max_dive_time_s            = v4->sws_max_dive_time_s;
	out->sws_min_surface_time_s         = v4->sws_min_surface_time_s;
	out->sws_sample_delay_min_us        = v4->sws_sample_delay_min_us;
	out->sws_sample_delay_max_us        = v4->sws_sample_delay_max_us;
	out->sws_sample_delay_default_us    = v4->sws_sample_delay_default_us;
	out->sws_enabled                    = v4->sws_enabled;
	out->sws_run_air_baseline           = v4->sws_run_air_baseline;
	out->sws_run_water_baseline         = v4->sws_run_water_baseline;
	out->sws_run_observed_peak          = v4->sws_run_observed_peak;
	out->bat_min_tx_mV                  = v4->bat_min_tx_mV;
	out->rate_window_s                  = v4->rate_window_s;
	out->rate_max_tx                    = v4->rate_max_tx;
}

static void migrate_v3_to_v4(const NVM_Config_v3_t *v3, NVM_Config_t *out)
{
	/* v3 fields are a strict prefix of v4 (minus rate_*). Copy them over and
	 * leave rate_window_s/rate_max_tx at 0 — apply_config() interprets 0/0 as
	 * "keep MGR_RATE compile-time defaults". */
	memset(out, 0, sizeof(*out));
	out->magic                          = NVM_MAGIC;
	out->version                        = NVM_VERSION;
	out->deploy_mode                    = v3->deploy_mode;
	out->led_mode                       = v3->led_mode;
	out->tx_initial_interval_s          = v3->tx_initial_interval_s;
	out->tx_growth_percent              = v3->tx_growth_percent;
	out->tx_max_count                   = v3->tx_max_count;
	out->tx_max_interval_s              = v3->tx_max_interval_s;
	out->tx_jitter_percent              = v3->tx_jitter_percent;
	out->sws_threshold_min              = v3->sws_threshold_min;
	out->sws_threshold_max              = v3->sws_threshold_max;
	out->sws_initial_air_baseline       = v3->sws_initial_air_baseline;
	out->sws_initial_water_baseline     = v3->sws_initial_water_baseline;
	out->sws_test_interval_surface_ms   = v3->sws_test_interval_surface_ms;
	out->sws_test_interval_underwater_ms= v3->sws_test_interval_underwater_ms;
	out->sws_max_dive_time_s            = v3->sws_max_dive_time_s;
	out->sws_min_surface_time_s         = v3->sws_min_surface_time_s;
	out->sws_sample_delay_min_us        = v3->sws_sample_delay_min_us;
	out->sws_sample_delay_max_us        = v3->sws_sample_delay_max_us;
	out->sws_sample_delay_default_us    = v3->sws_sample_delay_default_us;
	out->sws_enabled                    = v3->sws_enabled;
	out->sws_run_air_baseline           = v3->sws_run_air_baseline;
	out->sws_run_water_baseline         = v3->sws_run_water_baseline;
	out->sws_run_observed_peak          = v3->sws_run_observed_peak;
	out->bat_min_tx_mV                  = v3->bat_min_tx_mV;
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
		MGR_LOG_DEBUG("[NVM] Migrated v1 -> v5\r\n");
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
		MGR_LOG_DEBUG("[NVM] Migrated v2 -> v5\r\n");
		if (!validate_config(&cfg)) return false;
		apply_config(&cfg);
		return true;
	}

	if (cfg.version == 3) {
		NVM_Config_v3_t v3;
		if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &v3, sizeof(v3)) != KNS_STATUS_OK)
			return false;
		uint32_t computed = nvm_crc32(&v3, offsetof(NVM_Config_v3_t, crc32));
		if (computed != v3.crc32) {
			MGR_LOG_DEBUG("[NVM] v3 CRC mismatch\r\n");
			return false;
		}
		migrate_v3_to_v4(&v3, &cfg);
		MGR_LOG_DEBUG("[NVM] Migrated v3 -> v5\r\n");
		if (!validate_config(&cfg)) return false;
		apply_config(&cfg);
		return true;
	}

	if (cfg.version == 4) {
		NVM_Config_v4_t v4;
		if (MCU_FLASH_read(FLASH_NVM_CONFIG_ADDR, &v4, sizeof(v4)) != KNS_STATUS_OK)
			return false;
		uint32_t computed = nvm_crc32(&v4, offsetof(NVM_Config_v4_t, crc32));
		if (computed != v4.crc32) {
			MGR_LOG_DEBUG("[NVM] v4 CRC mismatch\r\n");
			return false;
		}
		migrate_v4_to_v5(&v4, &cfg);
		MGR_LOG_DEBUG("[NVM] Migrated v4 -> v5\r\n");
		if (!validate_config(&cfg)) return false;
		apply_config(&cfg);
		return true;
	}

	if (cfg.version != NVM_VERSION) {
		MGR_LOG_DEBUG("[NVM] Unknown version %u, using defaults\r\n", cfg.version);
		return false;
	}

	/* v5: verify CRC32 */
	uint32_t computed_crc = nvm_crc32(&cfg, offsetof(NVM_Config_t, crc32));
	if (computed_crc != cfg.crc32) {
		MGR_LOG_DEBUG("[NVM] v5 CRC mismatch (stored=0x%08lx computed=0x%08lx)\r\n",
			cfg.crc32, computed_crc);
		return false;
	}

	if (!validate_config(&cfg)) {
		MGR_LOG_DEBUG("[NVM] Config values invalid, using defaults\r\n");
		return false;
	}

	apply_config(&cfg);

	MGR_LOG_DEBUG("[NVM] v5 loaded: deploy=%u tx=%us jit=%u%% sws_surf=%lums sws_uw=%lums "
		"run_air=%u run_water=%u peak=%u rate=%u/%lus lb=%u/%u\r\n",
		cfg.deploy_mode, cfg.tx_initial_interval_s, cfg.tx_jitter_percent,
		(unsigned long)cfg.sws_test_interval_surface_ms,
		(unsigned long)cfg.sws_test_interval_underwater_ms,
		cfg.sws_run_air_baseline, cfg.sws_run_water_baseline,
		cfg.sws_run_observed_peak,
		cfg.rate_max_tx, (unsigned long)cfg.rate_window_s,
		cfg.lb_enter_mV, cfg.lb_exit_mV);

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
