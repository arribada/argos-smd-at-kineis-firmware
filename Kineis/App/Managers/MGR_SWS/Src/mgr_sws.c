/**
 * @file    mgr_sws.c
 * @brief   Salt Water Switch manager - 5-level adaptive underwater/surface detection
 *
 * @details
 * Hardened port of the Linkit v4 sws_analog_service algorithm for STM32WL55.
 * All absolute ADC values scaled from 14-bit (0-16383) to 12-bit (0-4095) by /4.
 *
 * **Hardware:**
 *   - PA12 = SWS_OUT (sensor power control, GPIO output push-pull)
 *   - PA11 = SWS_IN  (ADC_IN7, analog input)
 *
 * **5-Level Surface Detection (all require UW >= 1s + proximity guard):**
 *   - L1: Drop > 4% from previous raw -> immediate surface
 *   - L2: 2 consecutive raw drops, cumulative > 3%, each step >=2% -> surface
 *   - L3: MA3 decreasing 3+ times, total drop > 4% -> surface
 *   - L4: filtered < water_baseline * 92% -> surface
 *   - L5: Drop from dive peak > 10%, >10s gate -> safety fallback
 *
 * **Hardening fixes ported from linkit v4 commits Apr-2026:**
 *   - AIR_BASELINE_FLOOR: prevents air-baseline collapse (death spiral)
 *   - Stuck-state recovery: forces fresh calib if air collapses 5+ samples
 *   - Anti-spike observed_peak: rejects raw > peak*1.2 unless plausible
 *   - Continuous coherence check (3 samples) detects stale calibration
 *   - Adaptive sample delay (200us-5ms) for biofouling vs clean electrode
 *   - AIR_RECALIB_MAX_RATIO 70%: hard cap on air during L-override recalib
 *   - L2_MIN_STEP_PERCENT 2%: filters drift below threshold
 *   - PROXIMITY_GUARD adaptive (95% / 99% biofouling)
 *   - Dive timeout escalation: recalib N times, then force-surface + lockout
 *   - THRESHOLD_MIN_ABOVE_AIR: minimum gap to prevent false UW from noise
 *   - Fast convergence alpha=50% for first 5 underwater samples
 *   - Two distinct measurement intervals (surface vs underwater state)
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
#include "mgr_evtlog.h"
#include "stm32wlxx_hal.h"

/* ---- Pin Definitions ---- */

#define SWS_POWER_PIN        GPIO_PIN_12
#define SWS_POWER_PORT       GPIOA

/* ---- Algorithm Constants (12-bit scaled from 14-bit spec /4) ---- */

/* Filter */
#define ADC_HISTORY_SIZE                    2

/* Detection defaults */
#define DEFAULT_THRESHOLD_RATIO_PERCENT    35
#define DEFAULT_HYSTERESIS_PERCENT          4
#define DEFAULT_ALPHA_PERCENT              19   /**< EMA water baseline speed (normal) */
#define FAST_CONVERGENCE_ALPHA_PERCENT     50   /**< EMA during first dive (estimated water) */
#define FAST_CONVERGENCE_MAX_SAMPLES        5

/* Water baseline protection */
#define ABSOLUTE_MIN_WATER_ADC            500   /**< 2000/4 */
#define MIN_WATER_AIR_RATIO                 3

/* Air baseline floor (anti-collapse / death-spiral) */
#define AIR_BASELINE_FLOOR                 12   /**< 50/4 - dry electrode min */
#define AIR_BASELINE_RECOVER               38   /**< 150/4 - below this force delay UP */
#define THRESHOLD_MIN_ABOVE_AIR             5   /**< 20/4 - min gap thr - air */

/* Surface adaptation */
#define SURFACE_ADAPT_THRESHOLD_X10        13   /**< 1.3x stored as x10 */
#define MIN_SURFACE_TIME_FOR_ADAPT_S       10
#define SURFACE_READINGS_SIZE              10
#define CALIB_INTERVAL_S                 3600   /**< Periodic full recalib (1h) */

/* Air recalibration on L-override (gentle EMA + hard cap) */
#define AIR_RECALIB_EMA_WEIGHT_PCT         15
#define AIR_RECALIB_MAX_RATIO_PCT          70   /**< air <= 70% water */

/* ---- 5-Level Surface Detection Constants ---- */

/* Level 1: Instant drop from previous raw */
#define L1_DROP_PERCENT                     4

/* Level 2: Consecutive 2-sample raw drops */
#define L2_DROP_PERCENT                     3
#define L2_MIN_CONSECUTIVE                  2
#define L2_MIN_STEP_PERCENT                 2   /**< Each step must clear this (anti-drift) */

/* Level 3: MA trend (window = TREND_MA_SIZE) */
#define L3_MIN_CONSECUTIVE                  3
#define L3_DROP_PERCENT                     4
#define TREND_MA_SIZE                       3

/* Level 4: Absolute water baseline drop — aligned to linkit-v4 main (was 8). */
#define L4_DROP_PERCENT                    15

/* Level 5: Dive peak cumulative drop */
#define L5_DROP_PERCENT                    10
#define L5_MIN_TIME_SEC                    10

/* Safety */
#define OVERRIDE_MIN_TIME_SEC               1   /**< Min UW time before any L-override */
#define SURFACE_LOCKOUT_S                  30
#define MAX_CONSECUTIVE_DIVE_TIMEOUTS       3   /**< Force surface after N timeouts */

/* Proximity guard (adaptive) */
#define PROXIMITY_GUARD_PERCENT            95
#define PROXIMITY_GUARD_BIOFOULING         99   /**< Relaxed: when contrast < 5x */

/* Stuck-state recovery */
#define AIR_COLLAPSE_RECOVERY_SAMPLES       5

/* Continuous coherence (water << raw) */
#define COHERENCE_HIGH_REQUIRED             3   /**< Consecutive samples > water*2 */

/* Anti-spike observed_peak */
#define PEAK_SPIKE_NUMERATOR                6   /**< accept if raw <= peak * 6/5 */
#define PEAK_SPIKE_DENOMINATOR              5
#define PEAK_STUCK_REJECTS                 10   /**< force reset peak after this */

/* Adaptive sample delay */
#define CONTRAST_LOW_THRESHOLD             50   /**< contrast_x10 < 5.0 -> reduce delay */
#define CONTRAST_HIGH_THRESHOLD           100   /**< contrast_x10 > 10.0 -> increase delay */

/* First-sample coherence thresholds */
#define WATER_DETECT_HEURISTIC            625   /**< 2500/4 - above this = likely in water */

/* ---- Internal State ---- */

static MGR_SWS_Config_t sws_config = {
	.threshold_min                = 0,
	.threshold_max                = 4095,  /* 12-bit ADC full scale — accept saturated readings (bench short, real seawater can hit ~4000). Lower to reject only truly invalid readings. */
	.initial_air_baseline         = 50,    /* 200/4 */
	.initial_water_baseline       = 750,   /* 3000/4 */
	.test_interval_surface_ms     = 1000,  /**< Poll every 1s at surface */
	.test_interval_underwater_ms  = 500,   /**< Poll every 0.5s underwater (fast surface detection) */
	.max_dive_time_s              = 7200,
	.min_surface_time_s           = 10,
	.sample_delay_min_us          = 200,   /**< RC charge floor (pulse-ON before ADC read) */
	/* 10 ms ceiling per the linkit-v4 reference (SAMPLE_DELAY_MAX_US 10000):
	 * biofouled electrodes have tau > 10 ms — the old 1 ms cap kept the RC
	 * undercharged, water read as air, and dives went undetected late in a
	 * deployment. Salt water still converges to the 200 us floor. */
	.sample_delay_max_us          = 10000, /**< RC charge ceiling */
	.sample_delay_default_us      = 1000,  /**< RC charge starting point (linkit default) */
	.enabled                      = true,
};

static MGR_SWS_State_t sws_state = MGR_SWS_STATE_UNKNOWN;
static bool state_changed_flag = false;
static bool force_measurement = false;
static uint16_t last_raw_adc = 0;

/* Calibration dirty flag (set on baseline/peak update, cleared after flash save) */
static bool calib_dirty = false;

/* Timing */
static uint32_t last_measurement_tick = 0;
static uint32_t state_enter_tick = 0;

/* ADC filter */
static uint16_t adc_history[ADC_HISTORY_SIZE];
static uint8_t  adc_history_idx = 0;
static uint8_t  adc_history_count = 0;

/* Baselines */
static uint16_t air_baseline;
static uint16_t water_baseline;
static uint16_t threshold_current;
static uint16_t hysteresis_value;
static uint16_t contrast_x10 = 100;     /**< water/air * 10 */

/* Observed peak ADC (dynamic cap for water baseline) */
static uint16_t observed_peak_adc = 0;
static uint8_t  consecutive_spike_rejects = 0;

/* Surface adaptation buffer */
static uint16_t surface_readings[SURFACE_READINGS_SIZE];
static uint8_t  surface_readings_idx = 0;
static uint8_t  surface_readings_count = 0;

/* Level 1 & 2: Fast raw drop detection */
static uint16_t prev_raw = 0;
static uint16_t drop_reference = 0;
static uint8_t  consecutive_raw_drops = 0;

/* Level 3: MA trend detection */
static uint16_t trend_buffer[TREND_MA_SIZE];
static uint8_t  trend_buffer_idx = 0;
static uint8_t  trend_buffer_count = 0;
static uint16_t prev_ma3 = 0;
static uint16_t ma3_trend_start = 0;
static uint8_t  ma3_trend_count = 0;

/* Level 4 & 5: Dive tracking */
static uint16_t peak_adc_since_underwater = 0;
static uint16_t recent_peak = 0;
static uint16_t min_adc_during_dive = 0xFFFF;

/* Safety / lockout */
static uint32_t surface_lockout_until = 0;
static uint8_t  consecutive_dive_timeouts = 0;

/* Periodic recalibration timer */
static uint32_t last_calib_tick = 0;

/* First-sample coherence */
static bool first_sample_done = false;

/* Continuous coherence (water << raw) */
static uint8_t coherence_high_count = 0;

/* Stuck-state recovery */
static uint8_t air_collapse_count = 0;

/* Fast convergence (alpha=50% for first dive) */
static uint8_t fast_convergence_count = 0;

/* Adaptive sample delay */
static uint16_t sample_delay_us = 1000;

/* ---- Helper Functions ---- */

static uint32_t elapsed_s(uint32_t from_tick)
{
	return (HAL_GetTick() - from_tick) / 1000;
}

static void mark_calib_dirty(void)
{
	calib_dirty = true;
}

static uint32_t current_test_interval_ms(void)
{
	if (sws_state == MGR_SWS_STATE_SURFACE)
		return sws_config.test_interval_surface_ms;
	/* UNKNOWN or UNDERWATER -> use underwater (fast) cadence */
	return sws_config.test_interval_underwater_ms;
}

/* Public accessors used by the event-driven LPM scheduler to compute the
 * next wake delay. The LPM client treats the SWS interval as the natural
 * idle-period source of truth (see MGR_LPM_UW_idleTick). */
uint32_t MGR_SWS_getSurfIntervalMs(void)
{
	return sws_config.test_interval_surface_ms;
}

uint32_t MGR_SWS_getUWIntervalMs(void)
{
	return sws_config.test_interval_underwater_ms;
}

uint32_t MGR_SWS_msUntilNextSample(void)
{
	/* SWS disabled: no sampling deadline — return "infinite" so it never
	 * dominates the scheduler's min(); other deadlines (or the STOP2
	 * clamp) bound the sleep. */
	if (!sws_config.enabled)
		return 0xFFFFFFFFu;
	if (last_measurement_tick == 0)
		return 0;
	uint32_t interval = current_test_interval_ms();
	uint32_t elapsed = HAL_GetTick() - last_measurement_tick;
	return (elapsed >= interval) ? 0u : (interval - elapsed);
}

static void update_dynamic_threshold(void)
{
	if (water_baseline <= air_baseline) {
		threshold_current = air_baseline + THRESHOLD_MIN_ABOVE_AIR;
		hysteresis_value = 3;
		contrast_x10 = 10;
		return;
	}

	/* Contrast (water/air * 10) */
	contrast_x10 = (air_baseline > 0) ?
		(uint16_t)((uint32_t)water_baseline * 10 / air_baseline) : 100;

	uint8_t ratio;
	if (contrast_x10 >= 80) {
		ratio = DEFAULT_THRESHOLD_RATIO_PERCENT;
	} else if (contrast_x10 >= 40) {
		ratio = 50;
	} else {
		ratio = 40;
	}

	uint16_t range = water_baseline - air_baseline;
	threshold_current = air_baseline + (range * ratio) / 100;

	/* Enforce minimum gap above air (anti false-UW from noise) */
	uint16_t min_thresh = air_baseline + THRESHOLD_MIN_ABOVE_AIR;
	if (threshold_current < min_thresh)
		threshold_current = min_thresh;

	hysteresis_value = (threshold_current * DEFAULT_HYSTERESIS_PERCENT) / 100;
	if (hysteresis_value < 3) hysteresis_value = 3;

	/* Cap threshold+hysteresis at observed peak only when peak is plausibly water
	 * (peak >= water/2). A stale peak that decayed below water would otherwise
	 * pin threshold_high too low and block dive detection.
	 */
	if (observed_peak_adc > 0 && water_baseline > 0 &&
	    observed_peak_adc >= water_baseline / 2) {
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
	if (adc_history_count < ADC_HISTORY_SIZE)
		adc_history_count++;

	uint32_t sum = 0;
	for (uint8_t i = 0; i < adc_history_count; i++)
		sum += adc_history[i];
	return (uint16_t)(sum / adc_history_count);
}

static void adjust_sample_delay(void)
{
	if (air_baseline == 0)
		return;

	uint16_t old_delay = sample_delay_us;

	/* GUARD: if air baseline is near-zero, RC circuit isn't charging enough.
	 * Force delay UP to allow proper charging - breaks the death spiral.
	 */
	if (air_baseline < AIR_BASELINE_RECOVER) {
		if (sample_delay_us < sws_config.sample_delay_max_us) {
			sample_delay_us = (uint16_t)((uint32_t)sample_delay_us * 11 / 10);
			if (sample_delay_us > sws_config.sample_delay_max_us)
				sample_delay_us = sws_config.sample_delay_max_us;
		}
	}
	/* Normal adaptive: low contrast (biofouling) -> shorter delay,
	 * high contrast (clean) -> longer delay
	 */
	else if (contrast_x10 < CONTRAST_LOW_THRESHOLD &&
	         sample_delay_us > sws_config.sample_delay_min_us) {
		sample_delay_us = (uint16_t)((uint32_t)sample_delay_us * 3 / 4);
		if (sample_delay_us < sws_config.sample_delay_min_us)
			sample_delay_us = sws_config.sample_delay_min_us;
	} else if (contrast_x10 > CONTRAST_HIGH_THRESHOLD &&
	           sample_delay_us < sws_config.sample_delay_max_us) {
		sample_delay_us = (uint16_t)((uint32_t)sample_delay_us * 11 / 10);
		if (sample_delay_us > sws_config.sample_delay_max_us)
			sample_delay_us = sws_config.sample_delay_max_us;
	}

	if (old_delay != sample_delay_us) {
		MGR_LOG_DEBUG("[SWS] Adaptive delay %uus -> %uus (contrast_x10=%u)\r\n",
			old_delay, sample_delay_us, contrast_x10);
	}
}

static void calibrate_water_baseline(uint16_t value)
{
	bool water_is_estimated = (observed_peak_adc == 0);

	/* Aggressive alpha during first deployment for fast convergence */
	uint16_t alpha;
	if (water_is_estimated && fast_convergence_count < FAST_CONVERGENCE_MAX_SAMPLES) {
		alpha = FAST_CONVERGENCE_ALPHA_PERCENT;
	} else {
		alpha = DEFAULT_ALPHA_PERCENT;
	}

	uint16_t threshold_with_margin = threshold_current + hysteresis_value;
	uint16_t min_expected_water = water_is_estimated ?
		(uint16_t)(air_baseline * MIN_WATER_AIR_RATIO) :
		(uint16_t)((water_baseline * 85) / 100);

	/* Air ratio guard: skip when air is unreasonably high (>750 = 3000/4) */
	bool air_ratio_ok = (air_baseline >= 750) ||
		(value >= air_baseline * MIN_WATER_AIR_RATIO);

	bool ok = (value > threshold_with_margin) &&
		(value >= ABSOLUTE_MIN_WATER_ADC) &&
		air_ratio_ok &&
		(value >= min_expected_water || value > water_baseline);

	if (ok) {
		uint16_t new_water = (uint16_t)(((uint32_t)alpha * value +
			(100 - alpha) * (uint32_t)water_baseline) / 100);
		/* Cap at observed peak only when peak is plausibly water (>= value/2) */
		if (observed_peak_adc >= value / 2 && new_water > observed_peak_adc)
			new_water = observed_peak_adc;
		if (new_water != water_baseline) {
			water_baseline = new_water;
			update_dynamic_threshold();
			mark_calib_dirty();
		}

		if (water_is_estimated && fast_convergence_count < FAST_CONVERGENCE_MAX_SAMPLES) {
			fast_convergence_count++;
			MGR_LOG_DEBUG("[SWS] Fast convergence %u/%u water=%u\r\n",
				fast_convergence_count, FAST_CONVERGENCE_MAX_SAMPLES, new_water);
		}
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

	/* Adaptive RC discrimination delay
	 * Salt water (~1k): tau~1ms -> short delay enough
	 * Tap/biofouled: tau>5ms -> need longer delay
	 * ~8 cycles per __NOP at 32MHz -> count = us * 8 / 1
	 * Using 8 NOP per us as approximation (conservative).
	 */
	uint32_t count = (uint32_t)sample_delay_us * 8;
	while (count--) __NOP();

	uint32_t raw = ADC_ReadValue();
	sws_power_off();

	return (uint16_t)(raw & 0xFFF);
}

/* ---- Core Detection (5-level + hardening) ---- */

/* ---- Sensor fault detection (Sprint 3) ---------------------------------- */

#define SWS_FAULT_STUCK_SAMPLES   20u  /* same ADC for N consecutive reads → stuck */
#define SWS_FAULT_OOR_SAMPLES     10u  /* rail-low/high for N reads → broken sensor */
#define SWS_FAULT_OOR_LOW         5u
#define SWS_FAULT_OOR_HIGH        4090u
#define SWS_FAULT_VAR_WINDOW      30u  /* rolling window for variance check */
#define SWS_FAULT_VAR_MIN         3u   /* dead if variance < 3 LSB² */

static uint8_t  sws_current_fault    = MGR_SWS_FAULT_NONE;
static uint16_t sws_fault_stuck_last = 0xFFFFu;
static uint16_t sws_fault_stuck_cnt  = 0;
static uint16_t sws_fault_oor_cnt    = 0;
static uint16_t sws_fault_var_buf[SWS_FAULT_VAR_WINDOW];
static uint16_t sws_fault_var_idx    = 0;
static uint16_t sws_fault_var_filled = 0;

static void sws_fault_update(uint16_t adc)
{
	uint8_t new_fault = MGR_SWS_FAULT_NONE;

	/* STUCK: identical ADC for N reads. Counter saturates at u16 max. */
	if (adc == sws_fault_stuck_last) {
		if (sws_fault_stuck_cnt < 0xFFFFu)
			sws_fault_stuck_cnt++;
	} else {
		sws_fault_stuck_cnt = 0;
		sws_fault_stuck_last = adc;
	}
	if (sws_fault_stuck_cnt >= SWS_FAULT_STUCK_SAMPLES)
		new_fault |= MGR_SWS_FAULT_STUCK;

	/* OUT-OF-RANGE: rail-low or rail-high for N reads. */
	if (adc <= SWS_FAULT_OOR_LOW || adc >= SWS_FAULT_OOR_HIGH) {
		if (sws_fault_oor_cnt < 0xFFFFu)
			sws_fault_oor_cnt++;
	} else {
		sws_fault_oor_cnt = 0;
	}
	if (sws_fault_oor_cnt >= SWS_FAULT_OOR_SAMPLES)
		new_fault |= MGR_SWS_FAULT_OUT_OF_RANGE;

	/* NO_VARIANCE: rolling window, dead-sensor detection. */
	sws_fault_var_buf[sws_fault_var_idx] = adc;
	sws_fault_var_idx = (uint16_t)((sws_fault_var_idx + 1u) % SWS_FAULT_VAR_WINDOW);
	if (sws_fault_var_filled < SWS_FAULT_VAR_WINDOW)
		sws_fault_var_filled++;
	if (sws_fault_var_filled >= SWS_FAULT_VAR_WINDOW) {
		uint32_t sum = 0;
		for (uint16_t i = 0; i < SWS_FAULT_VAR_WINDOW; i++)
			sum += sws_fault_var_buf[i];
		uint16_t mean = (uint16_t)(sum / SWS_FAULT_VAR_WINDOW);
		uint32_t ssq = 0;
		for (uint16_t i = 0; i < SWS_FAULT_VAR_WINDOW; i++) {
			int32_t d = (int32_t)sws_fault_var_buf[i] - (int32_t)mean;
			ssq += (uint32_t)(d * d);
		}
		uint32_t variance = ssq / SWS_FAULT_VAR_WINDOW;
		if (variance < SWS_FAULT_VAR_MIN)
			new_fault |= MGR_SWS_FAULT_NO_VARIANCE;
	}

	/* Log only on rising edge (a fault bit that wasn't set before).
	 * Avoids spamming the log every single measurement once a sensor is broken. */
	uint8_t rising = (uint8_t)(new_fault & ~sws_current_fault);
	if (rising) {
		MGR_EVTLOG_log(EVT_SWS_FAULT, (uint16_t)new_fault);
	}
	sws_current_fault = new_fault;
}

uint8_t MGR_SWS_getFault(void)
{
	return sws_current_fault;
}

static bool detector_state(void)
{
	uint16_t raw_value = sws_read_adc();
	last_raw_adc = raw_value;
	sws_fault_update(raw_value);

	/* 1. Validate ADC */
	if (raw_value > sws_config.threshold_max)
		return (sws_state == MGR_SWS_STATE_UNDERWATER);

	bool is_underwater = (sws_state == MGR_SWS_STATE_UNDERWATER);

	/* 1b. First-sample coherence + continuous coherence check */
	{
		bool calib_incoherent = false;
		bool incoherent_in_water = false;

		if (!first_sample_done) {
			first_sample_done = true;

			/* Case 1: stored air low, reading way above water -> wrong medium */
			if (raw_value > threshold_current + hysteresis_value * 3 &&
			    water_baseline > 0 &&
			    raw_value > (uint16_t)((uint32_t)water_baseline * 13 / 10)) {
				MGR_LOG_DEBUG("[SWS] Coherence fail: raw=%u >> water=%u\r\n",
					raw_value, water_baseline);
				calib_incoherent = true;
				incoherent_in_water = true;
			}
			/* Case 2: stored air high (calibrated in water), reading low -> in air
			 * Guard: air > 1250 (= 5000/4 in 12-bit) */
			else if (raw_value < air_baseline / 2 && air_baseline > 1250) {
				MGR_LOG_DEBUG("[SWS] Coherence fail: raw=%u << air=%u\r\n",
					raw_value, air_baseline);
				calib_incoherent = true;
			}
		}

		/* Continuous coherence: 3 consecutive samples > water*2 -> adapt water.
		 * Catches deployment moves or salinity ramps without waiting for slow EMA.
		 */
		if (!calib_incoherent && first_sample_done &&
		    water_baseline > 0 &&
		    raw_value > (uint32_t)water_baseline * 2) {
			coherence_high_count++;
			if (coherence_high_count >= COHERENCE_HIGH_REQUIRED) {
				MGR_LOG_DEBUG("[SWS] Continuous coherence: raw=%u >> water=%u, adapting\r\n",
					raw_value, water_baseline);
				uint16_t new_water = raw_value;
				if (observed_peak_adc >= raw_value / 2 &&
				    new_water > observed_peak_adc) {
					new_water = observed_peak_adc;
				}
				water_baseline = new_water;
				update_dynamic_threshold();
				coherence_high_count = 0;
				mark_calib_dirty();
			}
		} else {
			coherence_high_count = 0;
		}

		if (calib_incoherent) {
			if (incoherent_in_water || raw_value > WATER_DETECT_HEURISTIC) {
				water_baseline = raw_value;
				air_baseline = raw_value / 3;
			} else {
				air_baseline = raw_value;
				water_baseline = raw_value * 3;
				if (observed_peak_adc > 0 && water_baseline > observed_peak_adc)
					water_baseline = observed_peak_adc;
				if (water_baseline > sws_config.threshold_max)
					water_baseline = sws_config.threshold_max;
			}
			if (air_baseline < AIR_BASELINE_FLOOR)
				air_baseline = AIR_BASELINE_FLOOR;
			if (water_baseline <= (uint32_t)air_baseline * MIN_WATER_AIR_RATIO)
				water_baseline = air_baseline * MIN_WATER_AIR_RATIO;
			update_dynamic_threshold();
			mark_calib_dirty();
			MGR_LOG_DEBUG("[SWS] Recalib coherence: air=%u water=%u th=%u\r\n",
				air_baseline, water_baseline, threshold_current);
		}
	}

	/* Save prev_raw BEFORE filtering */
	uint16_t saved_prev_raw = prev_raw;
	prev_raw = raw_value;

	/* 2. Filter */
	uint16_t filtered = moving_average(raw_value);

	/* Track min during dive */
	if (is_underwater && filtered < min_adc_during_dive)
		min_adc_during_dive = filtered;

	uint32_t time_in_state = elapsed_s(state_enter_tick);

	/* 3. 5-LEVEL SURFACE DETECTION */
	uint8_t surface_level = 0;

	/* Proximity guard adaptive: relaxes at low contrast (biofouling) */
	uint16_t proximity_ref = water_baseline;
	if (peak_adc_since_underwater > proximity_ref)
		proximity_ref = peak_adc_since_underwater;
	uint8_t guard_pct = (contrast_x10 < CONTRAST_LOW_THRESHOLD) ?
		PROXIMITY_GUARD_BIOFOULING : PROXIMITY_GUARD_PERCENT;
	bool proximity_ok = (proximity_ref == 0) ||
		(filtered < (uint16_t)((uint32_t)proximity_ref * guard_pct / 100));

	if (is_underwater && saved_prev_raw > 0 &&
	    time_in_state >= OVERRIDE_MIN_TIME_SEC && proximity_ok) {

		/* LEVEL 1: Sudden single-sample drop from previous raw */
		if (raw_value < saved_prev_raw) {
			uint16_t single_drop_pct = (uint16_t)(((uint32_t)(saved_prev_raw - raw_value) * 100) /
				saved_prev_raw);
			if (single_drop_pct >= L1_DROP_PERCENT)
				surface_level = 1;
		}

		/* LEVEL 2: Two consecutive raw drops, each >= L2_MIN_STEP_PERCENT,
		 * cumulative >= L2_DROP_PERCENT */
		if (surface_level == 0) {
			if (raw_value < saved_prev_raw) {
				uint16_t step_pct = (uint16_t)(((uint32_t)(saved_prev_raw - raw_value) * 100) /
					saved_prev_raw);
				if (step_pct >= L2_MIN_STEP_PERCENT) {
					if (consecutive_raw_drops == 0)
						drop_reference = saved_prev_raw;
					consecutive_raw_drops++;

					if (drop_reference > 0) {
						uint16_t cumul_pct = (uint16_t)(((uint32_t)(drop_reference - raw_value) * 100) /
							drop_reference);
						if (consecutive_raw_drops >= L2_MIN_CONSECUTIVE &&
						    cumul_pct >= L2_DROP_PERCENT)
							surface_level = 2;
					}
				} else {
					consecutive_raw_drops = 0;  /* drift, not a real drop */
				}
			} else {
				consecutive_raw_drops = 0;
			}
		}
	} else if (!is_underwater) {
		consecutive_raw_drops = 0;
	}

	/* LEVEL 3: MA trend detection (consecutive MA3 decreases) */
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
			if (ma3_trend_count > 0)
				ma3_trend_count--;  /* tolerate 1 flat/up */
		}

		if (ma3_trend_count >= L3_MIN_CONSECUTIVE && ma3_trend_start > 0) {
			uint16_t ma3_drop = (uint16_t)(((uint32_t)(ma3_trend_start - current_ma3) * 100) /
				ma3_trend_start);
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

		/* Recent peak: decays 2% per sample toward current reading */
		if (raw_value > recent_peak || recent_peak == 0)
			recent_peak = raw_value;
		else
			recent_peak = (uint16_t)(((uint32_t)recent_peak * 98 + (uint32_t)raw_value * 2) / 100);
	}

	if (surface_level == 0 && is_underwater &&
	    time_in_state >= OVERRIDE_MIN_TIME_SEC && proximity_ok) {

		/* LEVEL 4: Drop relative to water baseline */
		if (water_baseline > 0) {
			uint16_t water_thresh = (uint16_t)((uint32_t)water_baseline * (100 - L4_DROP_PERCENT) / 100);
			if (filtered < water_thresh)
				surface_level = 4;
		}

		/* LEVEL 5: Cumulative drop from dive peak (>10s gate, underflow guard) */
		if (surface_level == 0 && peak_adc_since_underwater > 0 &&
		    filtered < peak_adc_since_underwater &&
		    time_in_state > L5_MIN_TIME_SEC) {
			uint16_t drop_pct = (uint16_t)(((uint32_t)(peak_adc_since_underwater - filtered) * 100) /
				peak_adc_since_underwater);
			if (drop_pct >= L5_DROP_PERCENT)
				surface_level = 5;
		}
	}

	/* 4. SURFACE BASELINE TRACKING (blocked during lockout) */
	if (!is_underwater && time_in_state > MIN_SURFACE_TIME_FOR_ADAPT_S &&
	    (surface_lockout_until == 0 || HAL_GetTick() >= surface_lockout_until)) {
		/* Reject sub-floor readings (likely uncharged RC / disconnected electrode) */
		if (filtered < threshold_current && filtered >= AIR_BASELINE_FLOOR) {
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

			if (CALIB_INTERVAL_S > 0 &&
			    elapsed_s(last_calib_tick) >= CALIB_INTERVAL_S) {
				/* Periodic full air recalibration with FLOOR clamp */
				uint16_t old __attribute__((unused)) = air_baseline;
				uint16_t new_air = avg;
				if (new_air < AIR_BASELINE_FLOOR)
					new_air = AIR_BASELINE_FLOOR;
				air_baseline = new_air;
				/* Slow decay of observed peak (1% per cycle) */
				if (observed_peak_adc > 0)
					observed_peak_adc = (uint16_t)((uint32_t)observed_peak_adc * 99 / 100);
				update_dynamic_threshold();
				adjust_sample_delay();
				mark_calib_dirty();
				last_calib_tick = HAL_GetTick();
				surface_readings_count = 0;
				surface_readings_idx = 0;
				MGR_LOG_DEBUG("[SWS] Air recalib %u -> %u%s\r\n", old, air_baseline,
					(avg < AIR_BASELINE_FLOOR) ? " (floored)" : "");
			} else if (avg * 10 > (uint32_t)air_baseline * SURFACE_ADAPT_THRESHOLD_X10 &&
			           avg < threshold_current) {
				/* Upward adaptation: biofouling raising air level (slow 10%) */
				uint16_t new_air;
				if (air_baseline < AIR_BASELINE_FLOOR) {
					new_air = AIR_BASELINE_FLOOR;
				} else {
					new_air = (uint16_t)((90 * (uint32_t)air_baseline + 10 * (uint32_t)avg) / 100);
					if (new_air < AIR_BASELINE_FLOOR)
						new_air = AIR_BASELINE_FLOOR;
				}
				if (new_air != air_baseline) {
					air_baseline = new_air;
					update_dynamic_threshold();
					adjust_sample_delay();
					mark_calib_dirty();
				}
			} else if (avg < (uint16_t)((uint32_t)air_baseline * 70 / 100)) {
				/* Downward adaptation: air was too high. Guard against runaway. */
				bool avg_too_low = (avg < AIR_BASELINE_FLOOR * 2);
				bool air_already_low = (air_baseline < avg * 2);
				if (!avg_too_low && !air_already_low) {
					uint16_t new_air = (uint16_t)((80 * (uint32_t)air_baseline + 20 * (uint32_t)avg) / 100);
					if (new_air < AIR_BASELINE_FLOOR)
						new_air = AIR_BASELINE_FLOOR;
					air_baseline = new_air;
					update_dynamic_threshold();
					adjust_sample_delay();
					mark_calib_dirty();
					surface_readings_count = 0;
					surface_readings_idx = 0;
					MGR_LOG_DEBUG("[SWS] Air adapt DOWN: %u\r\n", air_baseline);
				}
			}
		}
	} else if (is_underwater) {
		surface_readings_count = 0;
		surface_readings_idx = 0;
	}

	/* 4b. STUCK-STATE RECOVERY (death-spiral fix) */
	if (!is_underwater && air_baseline < AIR_BASELINE_FLOOR) {
		air_collapse_count++;
		if (air_collapse_count >= AIR_COLLAPSE_RECOVERY_SAMPLES) {
			uint16_t old_air __attribute__((unused)) = air_baseline;
			uint16_t old_water __attribute__((unused)) = water_baseline;

			uint16_t recovered_air = (filtered >= AIR_BASELINE_FLOOR) ?
				filtered : AIR_BASELINE_FLOOR;
			air_baseline = recovered_air;
			if (water_baseline <= (uint32_t)recovered_air * MIN_WATER_AIR_RATIO) {
				water_baseline = recovered_air * MIN_WATER_AIR_RATIO;
				if (water_baseline > sws_config.threshold_max)
					water_baseline = sws_config.threshold_max;
			}
			observed_peak_adc = 0;
			consecutive_spike_rejects = 0;

			update_dynamic_threshold();
			adjust_sample_delay();
			mark_calib_dirty();
			surface_readings_count = 0;
			surface_readings_idx = 0;
			air_collapse_count = 0;

			MGR_LOG_DEBUG("[SWS] Stuck recovery: air %u->%u water %u->%u peak->0\r\n",
				old_air, air_baseline, old_water, water_baseline);
		}
	} else {
		air_collapse_count = 0;
	}

	/* 5. WATER BASELINE EMA (when underwater, not during lockout) */
	if (is_underwater && (surface_lockout_until == 0 || HAL_GetTick() >= surface_lockout_until))
		calibrate_water_baseline(filtered);

	/* 6. STATE DETERMINATION */
	bool new_is_underwater = is_underwater;
	uint16_t threshold_high = threshold_current + hysteresis_value;
	uint16_t threshold_low = (threshold_current > hysteresis_value) ?
		(threshold_current - hysteresis_value) : 1;
	if (threshold_low <= air_baseline && threshold_current > air_baseline)
		threshold_low = air_baseline + 1;

	if (filtered > threshold_high) {
		new_is_underwater = true;
	} else if (filtered < threshold_low) {
		new_is_underwater = false;
	}
	/* else: hysteresis zone, keep state */

	/* 6b. Update observed peak ADC with anti-spike + slow decay + coherence guard */
	{
		bool peak_updated = false;

		if (raw_value > observed_peak_adc) {
			/* First water contact: peak below half water baseline = stale.
			 * Accept new readings unconditionally to re-converge.
			 */
			uint16_t stale_ref = (water_baseline > 0) ? water_baseline / 2 : threshold_current;
			bool first_water_contact = (observed_peak_adc < stale_ref);
			if (observed_peak_adc == 0 || first_water_contact ||
			    raw_value <= ((uint32_t)observed_peak_adc * PEAK_SPIKE_NUMERATOR) / PEAK_SPIKE_DENOMINATOR) {
				observed_peak_adc = raw_value;
				peak_updated = true;
				consecutive_spike_rejects = 0;
			} else {
				consecutive_spike_rejects++;
				if (consecutive_spike_rejects >= PEAK_STUCK_REJECTS) {
					MGR_LOG_DEBUG("[SWS] Peak stuck: raw=%u >> peak=%u, resetting\r\n",
						raw_value, observed_peak_adc);
					observed_peak_adc = raw_value;
					water_baseline = raw_value;
					update_dynamic_threshold();
					peak_updated = true;
					consecutive_spike_rejects = 0;
				}
			}
		} else if (observed_peak_adc > 0) {
			/* Slow decay (0.1% per sample) only when raw is plausibly water,
			 * floored at water_baseline so peak can't drop into surface noise.
			 */
			bool raw_is_water = (water_baseline > 0 && raw_value > water_baseline / 2);
			if (raw_is_water) {
				uint16_t decayed = (uint16_t)(((uint32_t)observed_peak_adc * 999 +
					(uint32_t)raw_value) / 1000);
				if (decayed < water_baseline)
					decayed = water_baseline;
				if (decayed != observed_peak_adc) {
					observed_peak_adc = decayed;
					peak_updated = true;
				}
			}
		}

		/* Coherence guard: peak <= water * 5 */
		if (observed_peak_adc > 0 && water_baseline > 0 &&
		    observed_peak_adc > (uint32_t)water_baseline * 5) {
			MGR_LOG_DEBUG("[SWS] Peak incoherent: peak=%u > water*5, resetting\r\n",
				observed_peak_adc);
			observed_peak_adc = water_baseline;
			peak_updated = true;
		}

		if (peak_updated)
			mark_calib_dirty();
	}

	/* Apply multi-level surface override */
	if (surface_level > 0 && is_underwater && new_is_underwater) {
		new_is_underwater = false;

		/* Air recalib: gentle EMA toward raw with hard cap at AIR_RECALIB_MAX_RATIO_PCT * water */
		uint16_t old_air __attribute__((unused)) = air_baseline;
		uint16_t new_air = (uint16_t)(((uint32_t)air_baseline * (100 - AIR_RECALIB_EMA_WEIGHT_PCT) +
			(uint32_t)raw_value * AIR_RECALIB_EMA_WEIGHT_PCT) / 100);
		uint16_t hard_cap = (uint16_t)((uint32_t)water_baseline * AIR_RECALIB_MAX_RATIO_PCT / 100);
		if (new_air > hard_cap) new_air = hard_cap;
		if (new_air < AIR_BASELINE_FLOOR) new_air = AIR_BASELINE_FLOOR;
		if (new_air != air_baseline && new_air < water_baseline) {
			air_baseline = new_air;
			update_dynamic_threshold();
			adjust_sample_delay();
			mark_calib_dirty();
		}

		/* Enforce surface lockout */
		if (sws_config.min_surface_time_s > 0)
			surface_lockout_until = HAL_GetTick() + sws_config.min_surface_time_s * 1000;

		MGR_LOG_INFO("[SWS] SURFACE L%u raw=%u filt=%u air=%u->%u water=%u\r\n",
			surface_level, raw_value, filtered, old_air, air_baseline, water_baseline);
	}

	/* 7. MAX DIVE TIMEOUT - escalating response */
	if (is_underwater && sws_config.max_dive_time_s > 0 &&
	    time_in_state >= sws_config.max_dive_time_s) {
		consecutive_dive_timeouts++;
		uint16_t old_water __attribute__((unused)) = water_baseline;

		/* Always recalibrate water baseline from current reading */
		if (filtered > air_baseline * 2) {
			water_baseline = filtered;
			update_dynamic_threshold();
			mark_calib_dirty();
		}

		if (consecutive_dive_timeouts >= MAX_CONSECUTIVE_DIVE_TIMEOUTS) {
			new_is_underwater = false;
			surface_lockout_until = HAL_GetTick() + SURFACE_LOCKOUT_S * 1000;
			consecutive_dive_timeouts = 0;
			observed_peak_adc = water_baseline;
			consecutive_spike_rejects = 0;
			surface_readings_count = 0;
			surface_readings_idx = 0;
			update_dynamic_threshold();
			mark_calib_dirty();
			MGR_LOG_WARN("[SWS] Dive timeout escalation -> force surface (water %u->%u)\r\n",
				old_water, water_baseline);
		} else {
			/* Reset state timer for next escalation interval */
			state_enter_tick = HAL_GetTick();
			MGR_LOG_WARN("[SWS] Dive timeout %u/%u (water %u->%u)\r\n",
				consecutive_dive_timeouts, MAX_CONSECUTIVE_DIVE_TIMEOUTS,
				old_water, water_baseline);
		}
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

	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStruct.Pin = SWS_POWER_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SWS_POWER_PORT, &GPIO_InitStruct);
	HAL_GPIO_WritePin(SWS_POWER_PORT, SWS_POWER_PIN, GPIO_PIN_RESET);

	air_baseline = sws_config.initial_air_baseline;
	if (air_baseline < AIR_BASELINE_FLOOR)
		air_baseline = AIR_BASELINE_FLOOR;
	water_baseline = sws_config.initial_water_baseline;
	observed_peak_adc = 0;
	contrast_x10 = 100;
	update_dynamic_threshold();

	memset(adc_history, 0, sizeof(adc_history));
	memset(trend_buffer, 0, sizeof(trend_buffer));
	memset(surface_readings, 0, sizeof(surface_readings));
	adc_history_idx = 0;
	adc_history_count = 0;
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
	consecutive_dive_timeouts = 0;
	consecutive_spike_rejects = 0;
	coherence_high_count = 0;
	air_collapse_count = 0;
	fast_convergence_count = 0;
	last_calib_tick = HAL_GetTick();
	first_sample_done = false;
	calib_dirty = false;

	/* Initialize adaptive sample delay */
	sample_delay_us = sws_config.sample_delay_default_us;
	if (sample_delay_us < sws_config.sample_delay_min_us)
		sample_delay_us = sws_config.sample_delay_min_us;
	if (sample_delay_us > sws_config.sample_delay_max_us)
		sample_delay_us = sws_config.sample_delay_max_us;

	sws_state = MGR_SWS_STATE_UNKNOWN;
	state_enter_tick = HAL_GetTick();
	last_measurement_tick = 0;

	/* Initial calibration read */
	uint16_t initial_read = sws_read_adc();
	if (initial_read <= sws_config.threshold_max) {
		if (initial_read > WATER_DETECT_HEURISTIC) {
			sws_state = MGR_SWS_STATE_UNDERWATER;
			water_baseline = initial_read;
			air_baseline = initial_read / 3;
			if (air_baseline < AIR_BASELINE_FLOOR)
				air_baseline = AIR_BASELINE_FLOOR;
		} else if (initial_read < threshold_current) {
			sws_state = MGR_SWS_STATE_SURFACE;
			air_baseline = initial_read;
			if (air_baseline < AIR_BASELINE_FLOOR)
				air_baseline = AIR_BASELINE_FLOOR;
		} else {
			sws_state = MGR_SWS_STATE_UNDERWATER;
		}
		observed_peak_adc = initial_read;
		update_dynamic_threshold();
		for (uint8_t i = 0; i < ADC_HISTORY_SIZE; i++)
			adc_history[i] = initial_read;
		adc_history_count = ADC_HISTORY_SIZE;
		prev_raw = initial_read;
	}

	MGR_LOG_INFO("[SWS] Init: state=%d air=%u water=%u thresh=%u hyst=%u delay=%uus\r\n",
		sws_state, air_baseline, water_baseline, threshold_current, hysteresis_value,
		sample_delay_us);
}

void MGR_SWS_task(void)
{
	if (!sws_config.enabled)
		return;

	uint32_t now = HAL_GetTick();
	uint32_t elapsed_ms = now - last_measurement_tick;
	uint32_t interval_ms = current_test_interval_ms();

	if (!force_measurement && elapsed_ms < interval_ms && last_measurement_tick != 0)
		return;

	force_measurement = false;
	last_measurement_tick = now;

	bool is_underwater = detector_state();

	MGR_SWS_State_t new_state = is_underwater ? MGR_SWS_STATE_UNDERWATER : MGR_SWS_STATE_SURFACE;

	/* Per-measurement ADC log so the dev can validate PA11 readings live.
	 * Shows raw ADC, current baselines and threshold. Fires every measurement
	 * (1s in surface, 500ms underwater per default config). */
	MGR_LOG_DEBUG("[SWS] adc=%u %s air=%u water=%u th=%u\r\n",
		last_raw_adc,
		(new_state == MGR_SWS_STATE_UNDERWATER) ? "UW" : "SURF",
		air_baseline, water_baseline, threshold_current);

	if (new_state != sws_state) {
		MGR_LOG_INFO("[SWS] %s -> %s (adc=%u air=%u water=%u th=%u peak=%u)\r\n",
			sws_state == MGR_SWS_STATE_UNDERWATER ? "UW" : "SURF",
			new_state == MGR_SWS_STATE_UNDERWATER ? "UW" : "SURF",
			last_raw_adc, air_baseline, water_baseline, threshold_current,
			observed_peak_adc);

		sws_state = new_state;
		state_enter_tick = HAL_GetTick();
		state_changed_flag = true;

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
			consecutive_dive_timeouts = 0;
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
	if (config->test_interval_surface_ms == 0 || config->test_interval_underwater_ms == 0)
		return;
	if (config->sample_delay_min_us == 0 ||
	    config->sample_delay_max_us < config->sample_delay_min_us ||
	    config->sample_delay_default_us < config->sample_delay_min_us ||
	    config->sample_delay_default_us > config->sample_delay_max_us)
		return;

	sws_config = *config;
	air_baseline = config->initial_air_baseline;
	if (air_baseline < AIR_BASELINE_FLOOR)
		air_baseline = AIR_BASELINE_FLOOR;
	water_baseline = config->initial_water_baseline;
	update_dynamic_threshold();

	/* Re-clamp adaptive delay to new bounds */
	if (sample_delay_us < config->sample_delay_min_us)
		sample_delay_us = config->sample_delay_min_us;
	if (sample_delay_us > config->sample_delay_max_us)
		sample_delay_us = config->sample_delay_max_us;
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

uint16_t MGR_SWS_getAirBaseline(void)   { return air_baseline; }
uint16_t MGR_SWS_getWaterBaseline(void) { return water_baseline; }
uint16_t MGR_SWS_getObservedPeak(void)  { return observed_peak_adc; }
uint16_t MGR_SWS_getSampleDelayUs(void) { return sample_delay_us; }
uint16_t MGR_SWS_getThreshold(void)     { return threshold_current; }

void MGR_SWS_restoreBaselines(uint16_t air, uint16_t water)
{
	if (air > 0 && water > air) {
		if (air < AIR_BASELINE_FLOOR)
			air = AIR_BASELINE_FLOOR;
		air_baseline = air;
		water_baseline = water;
		update_dynamic_threshold();
		MGR_LOG_INFO("[SWS] Baselines restored: air=%u water=%u th=%u hyst=%u\r\n",
			air, water, threshold_current, hysteresis_value);
	}
}

void MGR_SWS_restoreObservedPeak(uint16_t peak)
{
	if (peak > 0) {
		observed_peak_adc = peak;
		MGR_LOG_DEBUG("[SWS] Observed peak restored: %u\r\n", peak);
	}
}

bool MGR_SWS_calibDirty(void)
{
	return calib_dirty;
}

void MGR_SWS_clearCalibDirty(void)
{
	calib_dirty = false;
}

void MGR_SWS_enterLowPower(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = SWS_POWER_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SWS_POWER_PORT, &GPIO_InitStruct);
}

void MGR_SWS_exitLowPower(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

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
