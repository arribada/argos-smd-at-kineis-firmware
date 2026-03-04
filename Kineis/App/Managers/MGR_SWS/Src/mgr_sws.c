/**
 * @file    mgr_sws.c
 * @brief   Salt Water Switch manager - Adaptive underwater/surface detection
 *
 * @details
 * Biofouling-adaptive detection algorithm based on sws_analog_implementation.md spec.
 * Adapted for STM32WL55 12-bit ADC (0-4095). All absolute ADC thresholds are
 * scaled from the 14-bit spec by dividing by 4.
 *
 * **Hardware:**
 *   - PA12 = SWS_OUT (sensor power control, GPIO output push-pull)
 *   - PA11 = SWS_IN  (ADC_IN7, analog input)
 *   - Stabilization delay: 10ms after powering sensor before ADC read
 *
 * **Detection pipeline (per measurement cycle):**
 *   1. Power sensor ON, wait 10ms, read ADC, power OFF
 *   2. Validate reading within [threshold_min, threshold_max]
 *   3. Apply moving average filter (2-sample history)
 *   4. Compute trend buffer statistics (8-sample sliding window):
 *      - Consecutive decrease counter
 *      - Total drop from peak
 *      - Variance (uint32_t to avoid truncation for high-spread values)
 *   5. Check 3-tier rapid transition detection (biofouling override):
 *      - Tier 1: >25% instant drop AND >75 ADC absolute (immediate surface)
 *      - Tier 2: >12% drop AND >38 absolute AND 1+ trend decrease AND below water baseline
 *      - Tier 3: >8% drop AND >25 absolute AND 2+ consecutive decreases
 *   6. Check trend + variance override (catches slow drift):
 *      - 3+ consecutive decreases with >10% total drop
 *      - High variance (>625) with >15% cumulative drop from peak
 *   7. Apply standard threshold with hysteresis:
 *      - Below threshold_low -> SURFACE
 *      - Above threshold_high -> UNDERWATER
 *      - Between: maintain previous state (hysteresis band)
 *
 * **Adaptive calibration:**
 *   - Air baseline: recalibrated from recent readings after stable surface time (>10s)
 *   - Water baseline: EMA (alpha=19%) during dives, absolute minimum tracking
 *   - Extended dive: recalibrate water baseline every 30s after 60s underwater
 *   - Safety guard: water baseline cannot drop below ABSOLUTE_MIN_WATER_ADC (500)
 *   - Ratio guard: water must be >= 5x air baseline
 *
 * **State transitions:**
 *   - UNKNOWN -> SURFACE or UNDERWATER (initial calibration read)
 *   - SURFACE -> UNDERWATER (filtered value crosses threshold_high)
 *   - UNDERWATER -> SURFACE (filtered below threshold_low OR biofouling override)
 *   - Max dive timeout forces SURFACE after configurable duration (default 7200s)
 *   - Surface lockout prevents quick SURFACE->UNDERWATER->SURFACE oscillation (30s)
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
#define SWS_STABILIZATION_MS 10   /**< Delay after powering sensor before ADC read */

/* ---- Algorithm Constants (12-bit scaled from 14-bit spec ÷4) ---- */

/* Detection */
#define ADC_HISTORY_SIZE                    2
#define DEFAULT_THRESHOLD_RATIO_PERCENT    35
#define DEFAULT_HYSTERESIS_PERCENT         14
#define DEFAULT_ALPHA_PERCENT              19   /**< EMA water baseline speed */

/* Rapid Transition Tiers */
#define RAPID_DROP_TIER1_PERCENT           25
#define RAPID_DROP_TIER1_ABSOLUTE          75   /**< 300/4 */
#define RAPID_DROP_TIER2_PERCENT           12
#define RAPID_DROP_TIER2_ABSOLUTE          38   /**< 150/4 */
#define RAPID_DROP_TIER3_PERCENT            8
#define RAPID_DROP_TIER3_ABSOLUTE          25   /**< 100/4 */
#define BIOFOULING_OVERRIDE_MIN_TIME_S      3

/* Biofouling Adaptation */
#define ABSOLUTE_MIN_WATER_ADC            500   /**< 2000/4 */
#define MIN_WATER_AIR_RATIO                 5
#define SURFACE_ADAPT_THRESHOLD_X10        13   /**< 1.3x stored as x10 */
#define MIN_SURFACE_TIME_FOR_ADAPT_S       10
#define EXTENDED_DIVE_RECALIB_START_S      60
#define EXTENDED_DIVE_RECALIB_INTERVAL_S   30

/* Trend and Variance */
#define TREND_BUFFER_SIZE                   8
#define TREND_DECREASE_THRESHOLD_PERCENT    2
#define TREND_DECREASE_ABSOLUTE_MIN         2   /**< 8/4 */
#define TREND_CONSECUTIVE_DECREASE_MIN      3
#define TREND_TOTAL_DROP_PERCENT           10
#define CUMULATIVE_DROP_PERCENT            15
#define VARIANCE_HIGH_THRESHOLD           625   /**< 10000/16 (squared values scale by 16) */

/* Surface adaptation */
#define SURFACE_READINGS_SIZE              10
#define SURFACE_LOCKOUT_S                  30

/* Hysteresis stuck recalibration */
#define HYSTERESIS_STUCK_TIMEOUT_S         30
#define HYSTERESIS_STUCK_RECALIB_INTERVAL_S 15

/* ---- Internal State ---- */

static MGR_SWS_Config_t sws_config = {
	.threshold_min          = 13,    /* 50/4 */
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
static uint32_t state_enter_tick = 0;       /**< Tick when current state was entered */
static uint32_t last_recalib_tick = 0;      /**< Extended dive recalibration timer */
static uint32_t hysteresis_enter_tick = 0;  /**< When ADC entered hysteresis zone */
static uint32_t last_hyst_recalib_tick = 0;

/* ADC filter */
static uint16_t adc_history[ADC_HISTORY_SIZE];
static uint8_t  adc_history_idx = 0;
static bool     adc_history_full = false;

/* Trend detection */
static uint16_t trend_buffer[TREND_BUFFER_SIZE];
static uint8_t  trend_buffer_idx = 0;
static bool     trend_buffer_full = false;
static uint8_t  trend_consecutive_decrease = 0;
static uint16_t trend_peak_value = 0;       /**< Peak ADC since last underwater entry */

/* Baselines */
static uint16_t air_baseline;
static uint16_t water_baseline;
static uint16_t threshold_current;
static uint16_t threshold_high;
static uint16_t threshold_low;

/* Surface adaptation buffer */
static uint16_t surface_readings[SURFACE_READINGS_SIZE];
static uint8_t  surface_readings_idx = 0;
static uint8_t  surface_readings_count = 0;

/* Rapid detection */
static bool rapid_surface_confirmed = false;
static uint16_t prev_trend_value = 0;

/* Dive tracking */
static uint16_t min_adc_during_dive = 0xFFFF;
static bool     in_hysteresis_zone = false;

/* ---- Helper Functions ---- */

static uint32_t elapsed_s(uint32_t from_tick)
{
	return (HAL_GetTick() - from_tick) / 1000;
}

static void update_thresholds(void)
{
	uint16_t range = (water_baseline > air_baseline) ? (water_baseline - air_baseline) : 1;
	threshold_current = air_baseline + (range * DEFAULT_THRESHOLD_RATIO_PERCENT) / 100;
	uint16_t hyst = (threshold_current * DEFAULT_HYSTERESIS_PERCENT) / 100;
	if (hyst < 1) hyst = 1;
	threshold_high = threshold_current + hyst;
	threshold_low  = (threshold_current > hyst) ? (threshold_current - hyst) : 1;
}

static uint16_t moving_average(uint16_t new_val)
{
	adc_history[adc_history_idx] = new_val;
	adc_history_idx = (adc_history_idx + 1) % ADC_HISTORY_SIZE;
	if (!adc_history_full && adc_history_idx == 0)
		adc_history_full = true;

	uint32_t sum = 0;
	uint8_t count = adc_history_full ? ADC_HISTORY_SIZE : adc_history_idx;
	if (count == 0) return new_val;
	for (uint8_t i = 0; i < count; i++)
		sum += adc_history[i];
	return (uint16_t)(sum / count);
}

static void reset_adc_history(uint16_t value)
{
	for (uint8_t i = 0; i < ADC_HISTORY_SIZE; i++)
		adc_history[i] = value;
	adc_history_idx = 0;
	adc_history_full = true;
}

static void update_trend(uint16_t filtered)
{
	uint16_t prev = trend_buffer_full ?
		trend_buffer[(trend_buffer_idx + TREND_BUFFER_SIZE - 1) % TREND_BUFFER_SIZE] :
		(trend_buffer_idx > 0 ? trend_buffer[trend_buffer_idx - 1] : filtered);

	trend_buffer[trend_buffer_idx] = filtered;
	trend_buffer_idx = (trend_buffer_idx + 1) % TREND_BUFFER_SIZE;
	if (!trend_buffer_full && trend_buffer_idx == 0)
		trend_buffer_full = true;

	/* Count consecutive decreases */
	if (prev > 0) {
		uint16_t drop_abs = (prev > filtered) ? (prev - filtered) : 0;
		uint16_t drop_pct = (prev > 0) ? (drop_abs * 100 / prev) : 0;
		if (drop_pct >= TREND_DECREASE_THRESHOLD_PERCENT && drop_abs >= TREND_DECREASE_ABSOLUTE_MIN)
			trend_consecutive_decrease++;
		else
			trend_consecutive_decrease = 0;
	}

	/* Track peak underwater value */
	if (filtered > trend_peak_value)
		trend_peak_value = filtered;
}

static void compute_variance(uint32_t *var_out)
{
	uint8_t count = trend_buffer_full ? TREND_BUFFER_SIZE :
		(trend_buffer_idx > 0 ? trend_buffer_idx : 1);
	uint32_t sum = 0;
	for (uint8_t i = 0; i < count; i++)
		sum += trend_buffer[i];
	uint16_t mean = (uint16_t)(sum / count);

	uint32_t sq_sum = 0;
	for (uint8_t i = 0; i < count; i++) {
		int32_t diff = (int32_t)trend_buffer[i] - (int32_t)mean;
		sq_sum += (uint32_t)(diff * diff);
	}
	*var_out = sq_sum / count;
}

static uint16_t trend_total_drop(void)
{
	if (!trend_buffer_full && trend_buffer_idx < 2) return 0;
	uint8_t count = trend_buffer_full ? TREND_BUFFER_SIZE : trend_buffer_idx;
	uint16_t max_v = 0, min_v = 0xFFFF;
	for (uint8_t i = 0; i < count; i++) {
		if (trend_buffer[i] > max_v) max_v = trend_buffer[i];
		if (trend_buffer[i] < min_v) min_v = trend_buffer[i];
	}
	return (max_v > min_v) ? (max_v - min_v) : 0;
}

static void update_water_baseline_ema(uint16_t value)
{
	/* Strict conditions to avoid corruption */
	if (value < threshold_current + (threshold_current * DEFAULT_HYSTERESIS_PERCENT / 100))
		return;
	if (value < ABSOLUTE_MIN_WATER_ADC)
		return;
	if (air_baseline > 0 && value < air_baseline * MIN_WATER_AIR_RATIO)
		return;
	/* Within +-15% of current baseline */
	if (water_baseline > 0) {
		uint16_t low  = water_baseline - water_baseline * 15 / 100;
		uint16_t high = water_baseline + water_baseline * 15 / 100;
		if (value < low || value > high)
			return;
	}
	/* EMA update: alpha=19% */
	water_baseline = (uint16_t)((DEFAULT_ALPHA_PERCENT * (uint32_t)value +
		(100 - DEFAULT_ALPHA_PERCENT) * (uint32_t)water_baseline) / 100);
	update_thresholds();
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

	/* Wait for sensor stabilization - use busy-wait to not depend on SysTick */
	volatile uint32_t count = SWS_STABILIZATION_MS * 8000; /* ~10ms at 32MHz/4cyc */
	while (count--) __NOP();

	uint32_t raw = ADC_ReadValue();
	sws_power_off();

	return (uint16_t)(raw & 0xFFF);
}

/* ---- Core Detection ---- */

static bool detector_state(void)
{
	uint16_t raw_value = sws_read_adc();
	last_raw_adc = raw_value;

	/* 1. Validate ADC */
	if (raw_value < sws_config.threshold_min || raw_value > sws_config.threshold_max)
		return (sws_state == MGR_SWS_STATE_UNDERWATER);

	/* 2. Filter */
	uint16_t filtered = moving_average(raw_value);

	/* 3. Trend + variance */
	update_trend(filtered);
	uint32_t variance = 0;
	compute_variance(&variance);
	uint16_t total_drop = trend_total_drop();

	/* Track minimum during dive */
	if (sws_state == MGR_SWS_STATE_UNDERWATER && raw_value < min_adc_during_dive)
		min_adc_during_dive = raw_value;

	/* 4. Biofouling surface override */
	bool biofouling_surface_override = false;
	bool rapid_detected = false;

	if (sws_state == MGR_SWS_STATE_UNDERWATER && prev_trend_value > 0 &&
	    raw_value < prev_trend_value &&
	    elapsed_s(state_enter_tick) >= BIOFOULING_OVERRIDE_MIN_TIME_S) {

		uint16_t drop_abs = prev_trend_value - raw_value;
		uint16_t drop_pct = (drop_abs * 100) / prev_trend_value;

		/* Tier 1: Large instant drop */
		if (drop_pct >= RAPID_DROP_TIER1_PERCENT && drop_abs >= RAPID_DROP_TIER1_ABSOLUTE) {
			rapid_detected = true;
		}
		/* Tier 2: Medium drop + trend + below water baseline */
		else if (drop_pct >= RAPID_DROP_TIER2_PERCENT && drop_abs >= RAPID_DROP_TIER2_ABSOLUTE &&
		         trend_consecutive_decrease >= 1 && raw_value < water_baseline) {
			rapid_detected = true;
		}
		/* Tier 3: Small drop + strong trend */
		else if (drop_pct >= RAPID_DROP_TIER3_PERCENT && drop_abs >= RAPID_DROP_TIER3_ABSOLUTE &&
		         trend_consecutive_decrease >= 2) {
			rapid_detected = true;
		}

		if (rapid_detected) {
			biofouling_surface_override = true;
			/* Reset ADC buffer to prevent filtered value from staying high */
			reset_adc_history(raw_value);
			/* Recalibrate air baseline if raw is above current baseline */
			if (raw_value > air_baseline) {
				air_baseline = (raw_value * 80) / 100;
				update_thresholds();
			}
			MGR_LOG_DEBUG("[SWS] RAPID T%d: drop=%u%% abs=%u\r\n",
				rapid_detected ? (drop_pct >= RAPID_DROP_TIER1_PERCENT ? 1 :
				(drop_pct >= RAPID_DROP_TIER2_PERCENT ? 2 : 3)) : 0,
				drop_pct, drop_abs);
		}
		/* Trend + variance override (slower but catches small changes) */
		else if (trend_consecutive_decrease >= TREND_CONSECUTIVE_DECREASE_MIN &&
		         total_drop > 0 && (total_drop * 100 / prev_trend_value) >= TREND_TOTAL_DROP_PERCENT) {
			biofouling_surface_override = true;
		}
		else if (variance >= VARIANCE_HIGH_THRESHOLD &&
		         trend_peak_value > 0 &&
		         (trend_peak_value - raw_value) * 100 / trend_peak_value >= CUMULATIVE_DROP_PERCENT) {
			biofouling_surface_override = true;
		}
	}

	prev_trend_value = raw_value;

	/* 5. Adaptive air recalibration (at surface > MIN_SURFACE_TIME_FOR_ADAPT_S) */
	if (sws_state == MGR_SWS_STATE_SURFACE &&
	    elapsed_s(state_enter_tick) >= MIN_SURFACE_TIME_FOR_ADAPT_S) {
		surface_readings[surface_readings_idx] = raw_value;
		surface_readings_idx = (surface_readings_idx + 1) % SURFACE_READINGS_SIZE;
		if (surface_readings_count < SURFACE_READINGS_SIZE)
			surface_readings_count++;

		if (surface_readings_count >= SURFACE_READINGS_SIZE) {
			uint32_t sum = 0;
			for (uint8_t i = 0; i < SURFACE_READINGS_SIZE; i++)
				sum += surface_readings[i];
			uint16_t avg = (uint16_t)(sum / SURFACE_READINGS_SIZE);
			/* If avg > 1.3x air_baseline → biofouling, adapt gradually */
			if (avg * 10 > air_baseline * SURFACE_ADAPT_THRESHOLD_X10) {
				air_baseline = (uint16_t)((90 * (uint32_t)air_baseline + 10 * (uint32_t)avg) / 100);
				update_thresholds();
			}
		}
	}

	/* 6. Extended dive recalibration */
	if (sws_state == MGR_SWS_STATE_UNDERWATER &&
	    elapsed_s(state_enter_tick) >= EXTENDED_DIVE_RECALIB_START_S &&
	    elapsed_s(last_recalib_tick) >= EXTENDED_DIVE_RECALIB_INTERVAL_S) {
		last_recalib_tick = HAL_GetTick();
		if (min_adc_during_dive > air_baseline * 3 &&
		    min_adc_during_dive < (water_baseline * 70) / 100) {
			uint16_t shift = min_adc_during_dive - air_baseline;
			if (air_baseline > 0 && (shift * 100 / air_baseline) > 50) {
				air_baseline = (min_adc_during_dive * 85) / 100;
				update_thresholds();
			}
		}
	}

	/* 7. Water baseline EMA update when clearly underwater */
	if (sws_state == MGR_SWS_STATE_UNDERWATER)
		update_water_baseline_ema(filtered);

	/* 8. Hysteresis decision */
	bool new_is_underwater;

	if (biofouling_surface_override) {
		new_is_underwater = false;
		rapid_surface_confirmed = true;
	} else if (rapid_surface_confirmed && filtered < threshold_high) {
		new_is_underwater = false;
	} else if (filtered > threshold_high) {
		rapid_surface_confirmed = false;
		new_is_underwater = true;
		in_hysteresis_zone = false;
	} else if (filtered < threshold_low) {
		rapid_surface_confirmed = false;
		new_is_underwater = false;
		in_hysteresis_zone = false;
	} else {
		/* In hysteresis zone - maintain previous state */
		new_is_underwater = (sws_state == MGR_SWS_STATE_UNDERWATER);
		if (!in_hysteresis_zone) {
			in_hysteresis_zone = true;
			hysteresis_enter_tick = HAL_GetTick();
			last_hyst_recalib_tick = HAL_GetTick();
		}
		/* Stuck in hysteresis > 30s → recalibrate */
		if (elapsed_s(hysteresis_enter_tick) >= HYSTERESIS_STUCK_TIMEOUT_S &&
		    elapsed_s(last_hyst_recalib_tick) >= HYSTERESIS_STUCK_RECALIB_INTERVAL_S) {
			last_hyst_recalib_tick = HAL_GetTick();
			uint16_t candidate = (filtered * 75) / 100;
			if (candidate > air_baseline * 12 / 10) {
				air_baseline = candidate;
				update_thresholds();
			}
		}
	}

	/* 9. Safeties */

	/* Max dive timeout */
	if (sws_state == MGR_SWS_STATE_UNDERWATER && sws_config.max_dive_time_s > 0 &&
	    elapsed_s(state_enter_tick) >= sws_config.max_dive_time_s) {
		new_is_underwater = false;
		/* Recalibrate air baseline (biofouling suspected) */
		if (min_adc_during_dive > air_baseline * 2) {
			air_baseline = (min_adc_during_dive * 80) / 100;
			update_thresholds();
		}
		MGR_LOG_DEBUG("[SWS] MAX_DIVE timeout, forcing surface\r\n");
	}

	/* Min surface time - prevent re-submersion too quickly */
	if (sws_state == MGR_SWS_STATE_SURFACE && new_is_underwater &&
	    elapsed_s(state_enter_tick) < sws_config.min_surface_time_s) {
		new_is_underwater = false;
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
	update_thresholds();

	/* Clear buffers */
	memset(adc_history, 0, sizeof(adc_history));
	memset(trend_buffer, 0, sizeof(trend_buffer));
	memset(surface_readings, 0, sizeof(surface_readings));
	adc_history_idx = 0;
	adc_history_full = false;
	trend_buffer_idx = 0;
	trend_buffer_full = false;
	trend_consecutive_decrease = 0;
	trend_peak_value = 0;
	prev_trend_value = 0;
	rapid_surface_confirmed = false;
	min_adc_during_dive = 0xFFFF;
	surface_readings_idx = 0;
	surface_readings_count = 0;
	in_hysteresis_zone = false;

	sws_state = MGR_SWS_STATE_UNKNOWN;
	state_enter_tick = HAL_GetTick();
	last_measurement_tick = 0;
	last_recalib_tick = HAL_GetTick();

	/* Perform initial calibration read */
	uint16_t initial_read = sws_read_adc();
	if (initial_read >= sws_config.threshold_min && initial_read <= sws_config.threshold_max) {
		if (initial_read < threshold_current) {
			sws_state = MGR_SWS_STATE_SURFACE;
			air_baseline = initial_read;
		} else {
			sws_state = MGR_SWS_STATE_UNDERWATER;
		}
		update_thresholds();
		reset_adc_history(initial_read);
		prev_trend_value = initial_read;
	}

	MGR_LOG_DEBUG("[SWS] Init: state=%d air=%u water=%u thresh=%u/%u/%u\r\n",
		sws_state, air_baseline, water_baseline, threshold_low, threshold_current, threshold_high);
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

		/* Reset dive tracking on state change */
		if (new_state == MGR_SWS_STATE_UNDERWATER) {
			min_adc_during_dive = 0xFFFF;
			trend_peak_value = 0;
			rapid_surface_confirmed = false;
		} else {
			surface_readings_idx = 0;
			surface_readings_count = 0;
		}
		in_hysteresis_zone = false;
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

	/* Validate: thresholds within ADC range, min < max, water > air */
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
	update_thresholds();
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
		update_thresholds();
		MGR_LOG_DEBUG("[SWS] Baselines restored: air=%u water=%u th=%u/%u/%u\r\n",
			air, water, threshold_low, threshold_current, threshold_high);
	}
}

/**
 * @}
 */
