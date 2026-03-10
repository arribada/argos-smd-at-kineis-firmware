/**
 * @file    mgr_sws.c
 * @brief   Salt Water Switch manager - 5-level adaptive underwater/surface detection
 *
 * @details
 * Port of the Linkit v4 sws_analog_service 5-level algorithm for STM32WL55.
 * All absolute ADC values scaled from 14-bit (0-16383) to 12-bit (0-4095) by /4.
 *
 * **Hardware:**
 *   - PA12 = SWS_OUT (sensor power control, GPIO output push-pull)
 *   - PA11 = SWS_IN  (ADC_IN7, analog input)
 *   - Stabilization delay: 1ms (RC time constant discrimination)
 *     Water (R~10k): tau=1ms, at 1ms ~63% charged -> high ADC
 *     Wet film (R~50-100k): tau=5-10ms, at 1ms ~10-18% -> low ADC
 *     Air (R=inf): 0% -> ~0 ADC
 *
 * **5-Level Surface Detection (all require UW >= 1s + proximity guard):**
 *   - L1: Drop > 5% from recent peak (decaying) -> immediate surface
 *   - L2: 2 consecutive raw drops, cumulative > 3% -> surface
 *   - L3: MA2 decreasing 3+ times, total drop > 5% -> surface
 *   - L4: filtered < water_baseline * 85% -> surface
 *   - L5: Drop from dive peak > 15%, >10s gate -> safety fallback
 *
 * **Proximity guard:**
 *   Overrides blocked if filtered > 95% of max(water_baseline, dive_peak).
 *   Prevents false surface from normal underwater ADC drift.
 *
 * **Dynamic threshold:**
 *   Contrast-adaptive ratio: clean(>=8x)=35%, moderate(>=4x)=50%, low(<4x)=40%
 *   Hysteresis: 4% (reduced from 14% for faster response)
 *
 * **Adaptive calibration:**
 *   - Air: upward (10% EMA if avg > 1.3x baseline), downward (20% if avg < 0.7x)
 *   - Water: EMA alpha=19%, capped at observed peak ADC
 *   - First-sample coherence check on boot
 *   - MIN_WATER_AIR_RATIO = 3 (reduced from 5)
 *
 * @see mgr_sws.h for the public API and configuration structure
 */

/**
 * @addtogroup MGR_SWS
 * @{
 */

#include <string.h>
#include "mgr_sws.h"
#include "adc.h"
#include "main.h"
#include "mgr_log.h"
#include "stm32wlxx_hal.h"

/* ---- Pin Definitions ---- */

#define SWS_POWER_PIN        GPIO_PIN_12
#define SWS_POWER_PORT       GPIOA
#define SWS_STABILIZATION_MS 1   /**< 1ms RC discrimination (was 10ms) */

/* ---- Algorithm Constants (12-bit scaled from 14-bit spec /4) ---- */

/* Filter */
#define ADC_HISTORY_SIZE                    2

/* Detection defaults */
#define DEFAULT_THRESHOLD_RATIO_PERCENT    35
#define DEFAULT_HYSTERESIS_PERCENT          4   /**< Reduced from 14% */
#define DEFAULT_ALPHA_PERCENT              19   /**< EMA water baseline speed */

/* Water baseline protection */
#define ABSOLUTE_MIN_WATER_ADC            500   /**< 2000/4 */
#define MIN_WATER_AIR_RATIO                 3   /**< Reduced from 5 */

/* Surface adaptation */
#define SURFACE_ADAPT_THRESHOLD_X10        13   /**< 1.3x stored as x10 */
#define MIN_SURFACE_TIME_FOR_ADAPT_S       10
#define SURFACE_READINGS_SIZE              10
#define CALIB_INTERVAL_S                 3600   /**< Periodic full recalib (1h) */

/* ---- 5-Level Surface Detection Constants ---- */

/* Level 1: Instant drop from recent peak */
#define L1_DROP_PERCENT                     5

/* Level 2: Consecutive 2-sample raw drops */
#define L2_DROP_PERCENT                     3
#define L2_MIN_CONSECUTIVE                  2

/* Level 3: MA trend (window = ADC_HISTORY_SIZE) */
#define L3_MIN_CONSECUTIVE                  3
#define L3_DROP_PERCENT                     5
#define TREND_MA_SIZE                       3

/* Level 4: Absolute water baseline drop */
#define L4_DROP_PERCENT                    15

/* Level 5: Dive peak cumulative drop */
#define L5_DROP_PERCENT                    15
#define L5_MIN_TIME_SEC                    10

/* Safety */
#define OVERRIDE_MIN_TIME_SEC               1   /**< Min UW time before any L-override */
#define SURFACE_LOCKOUT_S                  30

/* Proximity guard */
#define PROXIMITY_GUARD_PERCENT            95

/* First-sample coherence thresholds */
#define WATER_DETECT_HEURISTIC            625   /**< 2500/4 - above this = likely in water */

/* ---- Internal State ---- */

static MGR_SWS_Config_t sws_config = {
	.threshold_min          = 0,
	.threshold_max          = 2000,  /* 8000/4 */
	.initial_air_baseline   = 50,    /* 200/4 */
	.initial_water_baseline = 750,   /* 3000/4 */
	.test_interval_ms       = 1000,
	.max_dive_time_s        = 7200,
	.min_surface_time_s     = 10,
	.enabled                = true,
};

static MGR_SWS_State_t sws_state = MGR_SWS_STATE_UNKNOWN;
static bool state_changed_flag = false;
static bool force_measurement = false;
static uint16_t last_raw_adc = 0;

/* Timing */
static uint32_t last_measurement_tick = 0;
static uint32_t state_enter_tick = 0;

/* ADC filter */
static uint16_t adc_history[ADC_HISTORY_SIZE];
static uint8_t  adc_history_idx = 0;

/* Baselines */
static uint16_t air_baseline;
static uint16_t water_baseline;
static uint16_t threshold_current;
static uint16_t hysteresis_value;

/* Observed peak ADC (dynamic cap for water baseline) */
static uint16_t observed_peak_adc = 0;

/* Surface adaptation buffer */
static uint16_t surface_readings[SURFACE_READINGS_SIZE];
static uint8_t  surface_readings_idx = 0;
static uint8_t  surface_readings_count = 0;

/* Level 1 & 2: Fast raw drop detection */
static uint16_t prev_raw = 0;
static uint16_t recent_peak = 0;           /**< Decaying peak for L1 */
static uint16_t drop_reference = 0;        /**< Raw value when consecutive drops started */
static uint8_t  consecutive_raw_drops = 0;

/* Level 3: MA trend detection (TREND_MA_SIZE=3 window) */
static uint16_t trend_buffer[TREND_MA_SIZE];
static uint8_t  trend_buffer_idx = 0;
static uint8_t  trend_buffer_count = 0;
static uint16_t prev_ma3 = 0;
static uint16_t ma3_trend_start = 0;
static uint8_t  ma3_trend_count = 0;

/* Level 4 & 5: Dive tracking */
static uint16_t peak_adc_since_underwater = 0;
static uint16_t min_adc_during_dive = 0xFFFF;

/* Safety */
static uint32_t surface_lockout_until = 0;  /**< HAL tick when lockout expires (0 = no lockout) */

/* Periodic recalibration timer */
static uint32_t last_calib_tick = 0;

/* First-sample coherence */
static bool first_sample_done = false;

/* ---- Helper Functions ---- */

static uint32_t elapsed_s(uint32_t from_tick)
{
	return (HAL_GetTick() - from_tick) / 1000;
}

static void update_dynamic_threshold(void)
{
	if (water_baseline <= air_baseline) {
		threshold_current = air_baseline + 1;
		hysteresis_value = 1;
		return;
	}

	/* Dynamic ratio based on contrast */
	uint16_t range = water_baseline - air_baseline;
	uint16_t contrast_x10 = (air_baseline > 0) ?
		(uint16_t)((uint32_t)water_baseline * 10 / air_baseline) : 100;

	uint8_t ratio;
	if (contrast_x10 >= 80) {
		ratio = DEFAULT_THRESHOLD_RATIO_PERCENT; /* Clean: 35% */
	} else if (contrast_x10 >= 40) {
		ratio = 50; /* Moderate biofouling: midpoint */
	} else {
		ratio = 40; /* Low contrast: closer to air */
	}

	threshold_current = air_baseline + (range * ratio) / 100;

	hysteresis_value = (threshold_current * DEFAULT_HYSTERESIS_PERCENT) / 100;
	if (hysteresis_value < 3) hysteresis_value = 3;

	/* Cap threshold+hysteresis at observed peak so we never exceed actual readings */
	if (observed_peak_adc > 0) {
		uint16_t max_thresh_high = observed_peak_adc;
		if (threshold_current + hysteresis_value > max_thresh_high) {
			if (threshold_current >= max_thresh_high) {
				threshold_current = (max_thresh_high * 90) / 100;
				hysteresis_value = (max_thresh_high * 5) / 100;
			} else {
				hysteresis_value = max_thresh_high - threshold_current;
			}
			if (hysteresis_value < 3) hysteresis_value = 3;
		}
	}
}

static uint16_t moving_average(uint16_t new_val)
{
	adc_history[adc_history_idx] = new_val;
	adc_history_idx = (adc_history_idx + 1) % ADC_HISTORY_SIZE;

	uint32_t sum = 0;
	uint8_t count = 0;
	for (uint8_t i = 0; i < ADC_HISTORY_SIZE; i++) {
		if (adc_history[i] != 0) {
			sum += adc_history[i];
			count++;
		}
	}
	return (count > 0) ? (uint16_t)(sum / count) : new_val;
}

static void calibrate_water_baseline(uint16_t value)
{
	uint16_t threshold_with_margin = threshold_current + hysteresis_value;
	uint16_t min_expected_water = (water_baseline * 85) / 100;

	/* Air ratio guard: skip if air baseline is unreasonably high (>250 = 1000/4) */
	bool air_ratio_ok = (air_baseline >= 250) ||
		(value >= air_baseline * MIN_WATER_AIR_RATIO);

	bool ok = (value > threshold_with_margin) &&
		(value >= ABSOLUTE_MIN_WATER_ADC) &&
		air_ratio_ok &&
		(value >= min_expected_water || value > water_baseline);

	if (ok) {
		uint16_t new_water = (uint16_t)((DEFAULT_ALPHA_PERCENT * (uint32_t)value +
			(100 - DEFAULT_ALPHA_PERCENT) * (uint32_t)water_baseline) / 100);
		/* Cap at observed peak */
		if (observed_peak_adc > 0 && new_water > observed_peak_adc)
			new_water = observed_peak_adc;
		water_baseline = new_water;
		update_dynamic_threshold();
	}
}

static void sws_power_on(void)
{
	HAL_GPIO_WritePin(SWS_POWER_PORT, SWS_POWER_PIN, GPIO_PIN_SET);
}

static void sws_power_off(void)
{
	HAL_GPIO_WritePin(SWS_POWER_PORT, SWS_POWER_PIN, GPIO_PIN_RESET);
}

static uint16_t sws_read_adc(void)
{
	sws_power_on();

	/* 1ms RC discrimination delay - key to discriminating wet vs submerged */
	volatile uint32_t count = SWS_STABILIZATION_MS * 8000; /* ~1ms at 32MHz/4cyc */
	while (count--) __NOP();

	uint32_t raw = ADC_ReadValue();
	sws_power_off();

	return (uint16_t)(raw & 0xFFF);
}

/* ---- Core Detection (5-level) ---- */

static bool detector_state(void)
{
	uint16_t raw_value = sws_read_adc();
	last_raw_adc = raw_value;

	/* 1. Validate ADC */
	if (raw_value > sws_config.threshold_max)
		return (sws_state == MGR_SWS_STATE_UNDERWATER);

	/* 1b. First-sample coherence check */
	if (!first_sample_done) {
		first_sample_done = true;
		bool calib_incoherent = false;

		/* Reading far above water baseline -> calibration stale */
		if (raw_value > threshold_current + hysteresis_value * 3 &&
		    raw_value > (water_baseline * 13) / 10) {
			MGR_LOG_DEBUG("[SWS] Coherence fail: raw=%u >> water=%u\r\n",
				raw_value, water_baseline);
			calib_incoherent = true;
		}
		/* Reading far below air baseline -> air was calibrated in water
		 * Guard: air_baseline > 500 (=2000/4, matching reference 14-bit threshold) */
		else if (raw_value < air_baseline / 2 && air_baseline > 500) {
			MGR_LOG_DEBUG("[SWS] Coherence fail: raw=%u << air=%u\r\n",
				raw_value, air_baseline);
			calib_incoherent = true;
		}

		if (calib_incoherent) {
			if (raw_value > WATER_DETECT_HEURISTIC) {
				water_baseline = raw_value;
				air_baseline = raw_value / 3;
				if (air_baseline < sws_config.threshold_min)
					air_baseline = sws_config.threshold_min;
			} else {
				air_baseline = raw_value;
				water_baseline = raw_value * 3;
				if (observed_peak_adc > 0 && water_baseline > observed_peak_adc)
					water_baseline = observed_peak_adc;
				if (water_baseline > sws_config.threshold_max)
					water_baseline = sws_config.threshold_max;
			}
			update_dynamic_threshold();
			MGR_LOG_DEBUG("[SWS] Recalib coherence: air=%u water=%u th=%u\r\n",
				air_baseline, water_baseline, threshold_current);
		}
	}

	/* 1c. Update observed peak ADC (dynamic cap for baselines) */
	if (raw_value > observed_peak_adc)
		observed_peak_adc = raw_value;

	/* Save prev_raw BEFORE filtering */
	uint16_t saved_prev_raw = prev_raw;
	prev_raw = raw_value;

	/* 2. Filter */
	uint16_t filtered = moving_average(raw_value);

	/* Track min during dive */
	if (sws_state == MGR_SWS_STATE_UNDERWATER && filtered < min_adc_during_dive)
		min_adc_during_dive = filtered;

	uint32_t time_in_state = elapsed_s(state_enter_tick);

	/* 3. 5-LEVEL SURFACE DETECTION */
	uint8_t surface_level = 0;

	/* Proximity guard: block overrides if value still very close to water/peak */
	uint16_t proximity_ref = water_baseline;
	if (peak_adc_since_underwater > proximity_ref)
		proximity_ref = peak_adc_since_underwater;
	bool proximity_ok = (proximity_ref == 0) ||
		(filtered < (uint16_t)((uint32_t)proximity_ref * PROXIMITY_GUARD_PERCENT / 100));

	bool is_underwater = (sws_state == MGR_SWS_STATE_UNDERWATER);

	if (is_underwater && saved_prev_raw > 0 &&
	    time_in_state >= OVERRIDE_MIN_TIME_SEC && proximity_ok) {

		/* LEVEL 1: Drop from recent peak (decaying) */
		if (recent_peak > 0 && raw_value < recent_peak) {
			uint16_t peak_drop_pct = (uint16_t)(((uint32_t)(recent_peak - raw_value) * 100) / recent_peak);
			if (peak_drop_pct >= L1_DROP_PERCENT) {
				surface_level = 1;
			}
		}

		/* LEVEL 2: Two consecutive raw drops with cumulative threshold */
		if (surface_level == 0) {
			if (raw_value < saved_prev_raw) {
				if (consecutive_raw_drops == 0)
					drop_reference = saved_prev_raw;
				consecutive_raw_drops++;

				if (drop_reference > 0) {
					uint16_t cumul_pct = (uint16_t)(((uint32_t)(drop_reference - raw_value) * 100) / drop_reference);
					if (consecutive_raw_drops >= L2_MIN_CONSECUTIVE && cumul_pct >= L2_DROP_PERCENT)
						surface_level = 2;
				}
			} else {
				consecutive_raw_drops = 0;
			}
		}
	} else if (!is_underwater) {
		consecutive_raw_drops = 0;
	}

	/* LEVEL 3: MA trend detection */
	trend_buffer[trend_buffer_idx] = filtered;
	trend_buffer_idx = (trend_buffer_idx + 1) % TREND_MA_SIZE;
	if (trend_buffer_count < TREND_MA_SIZE)
		trend_buffer_count++;

	uint16_t current_ma3 = filtered;
	if (trend_buffer_count >= TREND_MA_SIZE) {
		uint32_t ma_sum = 0;
		for (uint8_t i = 0; i < TREND_MA_SIZE; i++)
			ma_sum += trend_buffer[i];
		current_ma3 = (uint16_t)(ma_sum / TREND_MA_SIZE);
	}

	if (surface_level == 0 && is_underwater && prev_ma3 > 0 &&
	    trend_buffer_count >= TREND_MA_SIZE &&
	    time_in_state >= OVERRIDE_MIN_TIME_SEC && proximity_ok) {

		if (current_ma3 < prev_ma3) {
			if (ma3_trend_count == 0)
				ma3_trend_start = prev_ma3;
			ma3_trend_count++;
		} else {
			/* Allow 1 flat/increase without full reset (noise tolerance) */
			if (ma3_trend_count > 0)
				ma3_trend_count--;
		}

		if (ma3_trend_count >= L3_MIN_CONSECUTIVE && ma3_trend_start > 0) {
			uint16_t ma3_drop = (uint16_t)(((uint32_t)(ma3_trend_start - current_ma3) * 100) / ma3_trend_start);
			if (ma3_drop >= L3_DROP_PERCENT)
				surface_level = 3;
		}
	} else if (!is_underwater) {
		ma3_trend_count = 0;
		ma3_trend_start = 0;
	}

	prev_ma3 = current_ma3;

	/* LEVEL 4 & 5: Track dive peak and recent peak */
	if (is_underwater) {
		if (filtered > peak_adc_since_underwater)
			peak_adc_since_underwater = filtered;

		/* Recent peak: decays 5% per sample toward current reading */
		if (raw_value > recent_peak || recent_peak == 0) {
			recent_peak = raw_value;
		} else {
			recent_peak = (uint16_t)(((uint32_t)recent_peak * 95 + (uint32_t)raw_value * 5) / 100);
		}
	}

	if (surface_level == 0 && is_underwater &&
	    time_in_state >= OVERRIDE_MIN_TIME_SEC && proximity_ok) {

		/* LEVEL 4: Drop relative to water baseline */
		if (water_baseline > 0) {
			uint16_t water_thresh = (uint16_t)((uint32_t)water_baseline * (100 - L4_DROP_PERCENT) / 100);
			if (filtered < water_thresh)
				surface_level = 4;
		}

		/* LEVEL 5: Cumulative drop from dive peak (with time gate) */
		if (surface_level == 0 && peak_adc_since_underwater > 0 &&
		    time_in_state > L5_MIN_TIME_SEC) {
			uint16_t drop_pct = (uint16_t)(((uint32_t)(peak_adc_since_underwater - filtered) * 100) /
				peak_adc_since_underwater);
			if (drop_pct >= L5_DROP_PERCENT)
				surface_level = 5;
		}
	}

	/* 4. SURFACE BASELINE TRACKING */
	if (!is_underwater && time_in_state > MIN_SURFACE_TIME_FOR_ADAPT_S) {
		/* Only accept readings below threshold (guard against wrong state) */
		if (filtered < threshold_current) {
			surface_readings[surface_readings_idx] = filtered;
			surface_readings_idx = (surface_readings_idx + 1) % SURFACE_READINGS_SIZE;
			if (surface_readings_count < SURFACE_READINGS_SIZE)
				surface_readings_count++;
		}

		if (surface_readings_count >= SURFACE_READINGS_SIZE / 2) {
			uint32_t sum = 0;
			for (uint8_t i = 0; i < surface_readings_count; i++)
				sum += surface_readings[i];
			uint16_t avg = (uint16_t)(sum / surface_readings_count);

			/* Periodic full recalibration (every CALIB_INTERVAL_S) */
			if (CALIB_INTERVAL_S > 0 &&
			    elapsed_s(last_calib_tick) >= CALIB_INTERVAL_S) {
				MGR_LOG_DEBUG("[SWS] Air recalib %u -> %u\r\n",
					air_baseline, avg);
				air_baseline = avg;
				/* Slow decay of observed peak (1% per recalib cycle) */
				if (observed_peak_adc > 0)
					observed_peak_adc = (uint16_t)((uint32_t)observed_peak_adc * 99 / 100);
				update_dynamic_threshold();
				last_calib_tick = HAL_GetTick();
				surface_readings_count = 0;
				surface_readings_idx = 0;
			} else if (avg * 10 > (uint32_t)air_baseline * SURFACE_ADAPT_THRESHOLD_X10 &&
			    avg < threshold_current) {
				/* Upward adaptation: biofouling raising air level (slow 10%) */
				air_baseline = (uint16_t)((90 * (uint32_t)air_baseline + 10 * (uint32_t)avg) / 100);
				update_dynamic_threshold();
			} else if (avg < (air_baseline * 70) / 100) {
				/* Downward adaptation: air was too high, adapt faster (20%) */
				air_baseline = (uint16_t)((80 * (uint32_t)air_baseline + 20 * (uint32_t)avg) / 100);
				update_dynamic_threshold();
				surface_readings_count = 0;
				surface_readings_idx = 0;
				MGR_LOG_DEBUG("[SWS] Air adapt DOWN: %u\r\n", air_baseline);
			}
		}
	} else if (is_underwater) {
		surface_readings_count = 0;
		surface_readings_idx = 0;
	}

	/* 5. WATER BASELINE EMA (when clearly underwater, not during lockout) */
	if (is_underwater && (surface_lockout_until == 0 || HAL_GetTick() >= surface_lockout_until))
		calibrate_water_baseline(filtered);

	/* 6. STATE DETERMINATION */
	bool new_is_underwater;
	uint16_t threshold_high = threshold_current + hysteresis_value;
	uint16_t threshold_low = (threshold_current > hysteresis_value) ?
		(threshold_current - hysteresis_value) : 1;

	if (filtered > threshold_high) {
		new_is_underwater = true;
	} else if (filtered < threshold_low) {
		new_is_underwater = false;
	} else {
		/* Hysteresis zone - maintain previous state */
		new_is_underwater = is_underwater;
	}

	/* Apply multi-level surface override */
	if (surface_level > 0 && is_underwater && new_is_underwater) {
		new_is_underwater = false;

		/* Recalibrate air baseline to current reading if it won't destroy contrast */
		uint16_t max_air_for_contrast = (water_baseline * 80) / 100;
		if (filtered > air_baseline * 2 && filtered < max_air_for_contrast) {
			air_baseline = filtered;
			update_dynamic_threshold();
		}

		/* Enforce surface lockout */
		if (sws_config.min_surface_time_s > 0)
			surface_lockout_until = HAL_GetTick() + sws_config.min_surface_time_s * 1000;

		MGR_LOG_DEBUG("[SWS] SURFACE L%u raw=%u filt=%u air=%u water=%u\r\n",
			surface_level, raw_value, filtered, air_baseline, water_baseline);
	}

	/* 7. MAX DIVE TIMEOUT */
	if (is_underwater && sws_config.max_dive_time_s > 0 &&
	    time_in_state >= sws_config.max_dive_time_s) {
		new_is_underwater = false;
		surface_lockout_until = HAL_GetTick() + SURFACE_LOCKOUT_S * 1000;
		MGR_LOG_DEBUG("[SWS] MAX_DIVE timeout, forcing surface\r\n");
	}

	/* 8. SURFACE LOCKOUT (time-based) */
	if (surface_lockout_until > 0 && HAL_GetTick() < surface_lockout_until) {
		if (new_is_underwater)
			new_is_underwater = false;
	} else {
		surface_lockout_until = 0;
	}

	return new_is_underwater;
}

/* ---- Public API ---- */

void MGR_SWS_init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Configure PA12 as output for sensor power control */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = SWS_POWER_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SWS_POWER_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(SWS_POWER_PORT, SWS_POWER_PIN, GPIO_PIN_RESET);

	/* Initialize baselines */
	air_baseline = sws_config.initial_air_baseline;
	water_baseline = sws_config.initial_water_baseline;
	observed_peak_adc = 0;
	update_dynamic_threshold();

	/* Clear buffers */
	memset(adc_history, 0, sizeof(adc_history));
	memset(trend_buffer, 0, sizeof(trend_buffer));
	memset(surface_readings, 0, sizeof(surface_readings));
	adc_history_idx = 0;
	trend_buffer_idx = 0;
	trend_buffer_count = 0;
	prev_ma3 = 0;
	ma3_trend_start = 0;
	ma3_trend_count = 0;
	prev_raw = 0;
	recent_peak = 0;
	drop_reference = 0;
	consecutive_raw_drops = 0;
	peak_adc_since_underwater = 0;
	min_adc_during_dive = 0xFFFF;
	surface_readings_idx = 0;
	surface_readings_count = 0;
	surface_lockout_until = 0;
	last_calib_tick = HAL_GetTick();
	first_sample_done = false;

	sws_state = MGR_SWS_STATE_UNKNOWN;
	state_enter_tick = HAL_GetTick();
	last_measurement_tick = 0;

	/* Initial calibration read */
	uint16_t initial_read = sws_read_adc();
	if (initial_read <= sws_config.threshold_max) {
		if (initial_read > WATER_DETECT_HEURISTIC) {
			/* Started in water */
			sws_state = MGR_SWS_STATE_UNDERWATER;
			water_baseline = initial_read;
			air_baseline = initial_read / 3;
			if (air_baseline < sws_config.threshold_min)
				air_baseline = sws_config.threshold_min;
		} else if (initial_read < threshold_current) {
			sws_state = MGR_SWS_STATE_SURFACE;
			air_baseline = initial_read;
		} else {
			sws_state = MGR_SWS_STATE_UNDERWATER;
		}
		observed_peak_adc = initial_read;
		update_dynamic_threshold();
		/* Pre-fill filter */
		for (uint8_t i = 0; i < ADC_HISTORY_SIZE; i++)
			adc_history[i] = initial_read;
		prev_raw = initial_read;
	}

	MGR_LOG_DEBUG("[SWS] Init: state=%d air=%u water=%u thresh=%u hyst=%u\r\n",
		sws_state, air_baseline, water_baseline, threshold_current, hysteresis_value);
}

void MGR_SWS_task(void)
{
	if (!sws_config.enabled)
		return;

	uint32_t now = HAL_GetTick();
	uint32_t elapsed_ms = now - last_measurement_tick;

	if (!force_measurement && elapsed_ms < sws_config.test_interval_ms && last_measurement_tick != 0)
		return;

	force_measurement = false;
	last_measurement_tick = now;

	/* Run detection */
	bool is_underwater = detector_state();

	/* Update state */
	MGR_SWS_State_t new_state = is_underwater ? MGR_SWS_STATE_UNDERWATER : MGR_SWS_STATE_SURFACE;

	if (new_state != sws_state) {
		MGR_LOG_DEBUG("[SWS] %s -> %s (adc=%u air=%u water=%u th=%u)\r\n",
			sws_state == MGR_SWS_STATE_UNDERWATER ? "UW" : "SURF",
			new_state == MGR_SWS_STATE_UNDERWATER ? "UW" : "SURF",
			last_raw_adc, air_baseline, water_baseline, threshold_current);

		sws_state = new_state;
		state_enter_tick = HAL_GetTick();
		state_changed_flag = true;

		/* Reset tracking on state change */
		if (new_state == MGR_SWS_STATE_UNDERWATER) {
			min_adc_during_dive = 0xFFFF;
			peak_adc_since_underwater = 0;
			recent_peak = 0;
			consecutive_raw_drops = 0;
			drop_reference = 0;
			trend_buffer_count = 0;
			trend_buffer_idx = 0;
			ma3_trend_count = 0;
			ma3_trend_start = 0;
			prev_ma3 = 0;
		} else {
			surface_readings_idx = 0;
			surface_readings_count = 0;
		}
	}
}

MGR_SWS_State_t MGR_SWS_getState(void)
{
	return sws_state;
}

uint16_t MGR_SWS_getLastADC(void)
{
	return last_raw_adc;
}

MGR_SWS_Config_t MGR_SWS_getConfig(void)
{
	return sws_config;
}

void MGR_SWS_setConfig(const MGR_SWS_Config_t *config)
{
	if (!config)
		return;

	if (config->threshold_min >= config->threshold_max)
		return;
	if (config->threshold_min > 4095 || config->threshold_max > 4095)
		return;
	if (config->initial_water_baseline <= config->initial_air_baseline)
		return;
	if (config->test_interval_ms == 0)
		return;

	sws_config = *config;
	air_baseline = config->initial_air_baseline;
	water_baseline = config->initial_water_baseline;
	update_dynamic_threshold();
}

void MGR_SWS_forceMeasurement(void)
{
	force_measurement = true;
}

bool MGR_SWS_stateChanged(void)
{
	bool changed = state_changed_flag;
	state_changed_flag = false;
	return changed;
}

uint16_t MGR_SWS_getAirBaseline(void)
{
	return air_baseline;
}

uint16_t MGR_SWS_getWaterBaseline(void)
{
	return water_baseline;
}

void MGR_SWS_restoreBaselines(uint16_t air, uint16_t water)
{
	if (air > 0 && water > air) {
		air_baseline = air;
		water_baseline = water;
		update_dynamic_threshold();
		MGR_LOG_DEBUG("[SWS] Baselines restored: air=%u water=%u th=%u hyst=%u\r\n",
			air, water, threshold_current, hysteresis_value);
	}
}

uint16_t MGR_SWS_getObservedPeak(void)
{
	return observed_peak_adc;
}

void MGR_SWS_restoreObservedPeak(uint16_t peak)
{
	if (peak > 0) {
		observed_peak_adc = peak;
		MGR_LOG_DEBUG("[SWS] Observed peak restored: %u\r\n", peak);
	}
}

void MGR_SWS_enterLowPower(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Reconfigure PA12 (SWS_OUT) as analog to eliminate leakage in STOP */
	GPIO_InitStruct.Pin = SWS_POWER_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SWS_POWER_PORT, &GPIO_InitStruct);
}

void MGR_SWS_exitLowPower(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Restore PA12 (SWS_OUT) as output push-pull for sensor power control */
	GPIO_InitStruct.Pin = SWS_POWER_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SWS_POWER_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(SWS_POWER_PORT, SWS_POWER_PIN, GPIO_PIN_RESET);
}

/**
 * @}
 */
