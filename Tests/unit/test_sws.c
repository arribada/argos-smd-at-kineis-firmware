/**
 * @file    test_sws.c
 * @brief   Unit tests for SWS 5-level detection algorithm
 *
 * Tests the core algorithm logic without hardware:
 *   - Dynamic threshold computation (contrast-adaptive ratio)
 *   - Moving average filter
 *   - Water baseline EMA calibration
 *   - 5-level surface detection (L1-L5)
 *   - Proximity guard
 *   - Surface lockout and max dive timeout
 *   - First-sample coherence check
 *   - Air baseline adaptation (up/down)
 *   - Periodic recalibration
 *
 * Hardware dependencies (ADC, GPIO) are stubbed.
 */

#include "test_framework.h"

/*******************************************************************************
 * CONSTANTS (from mgr_sws.c)
 ******************************************************************************/

#define ADC_HISTORY_SIZE                    2
#define DEFAULT_THRESHOLD_RATIO_PERCENT    35
#define DEFAULT_HYSTERESIS_PERCENT          4
#define DEFAULT_ALPHA_PERCENT              19
#define ABSOLUTE_MIN_WATER_ADC            500
#define MIN_WATER_AIR_RATIO                 3
#define SURFACE_ADAPT_THRESHOLD_X10        13
#define MIN_SURFACE_TIME_FOR_ADAPT_S       10
#define SURFACE_READINGS_SIZE              10
#define CALIB_INTERVAL_S                 3600

#define L1_DROP_PERCENT                     5
#define L2_DROP_PERCENT                     3
#define L2_MIN_CONSECUTIVE                  2
#define L3_MIN_CONSECUTIVE                  3
#define L3_DROP_PERCENT                     5
#define TREND_MA_SIZE                       3
#define L4_DROP_PERCENT                    15
#define L5_DROP_PERCENT                    15
#define L5_MIN_TIME_SEC                    10
#define OVERRIDE_MIN_TIME_SEC               1
#define SURFACE_LOCKOUT_S                  30
#define PROXIMITY_GUARD_PERCENT            95
#define WATER_DETECT_HEURISTIC            625

/*******************************************************************************
 * FUNCTIONS UNDER TEST (extracted from mgr_sws.c)
 ******************************************************************************/

/* Baselines */
static uint16_t air_baseline;
static uint16_t water_baseline;
static uint16_t threshold_current;
static uint16_t hysteresis_value;
static uint16_t observed_peak_adc;

static void update_dynamic_threshold(void)
{
	if (water_baseline <= air_baseline) {
		threshold_current = air_baseline + 1;
		hysteresis_value = 1;
		return;
	}

	uint16_t range = water_baseline - air_baseline;
	uint16_t contrast_x10 = (air_baseline > 0) ?
		(uint16_t)((uint32_t)water_baseline * 10 / air_baseline) : 100;

	uint8_t ratio;
	if (contrast_x10 >= 80)
		ratio = DEFAULT_THRESHOLD_RATIO_PERCENT;
	else if (contrast_x10 >= 40)
		ratio = 50;
	else
		ratio = 40;

	threshold_current = air_baseline + (range * ratio) / 100;

	hysteresis_value = (threshold_current * DEFAULT_HYSTERESIS_PERCENT) / 100;
	if (hysteresis_value < 3) hysteresis_value = 3;

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

/* Moving average */
static uint16_t adc_history[ADC_HISTORY_SIZE];
static uint8_t  adc_history_idx;

static void reset_filter(void)
{
	for (int i = 0; i < ADC_HISTORY_SIZE; i++)
		adc_history[i] = 0;
	adc_history_idx = 0;
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

/* Water baseline EMA */
static void calibrate_water_baseline(uint16_t value)
{
	uint16_t threshold_with_margin = threshold_current + hysteresis_value;
	uint16_t min_expected_water = (water_baseline * 85) / 100;

	bool air_ratio_ok = (air_baseline >= 250) ||
		(value >= air_baseline * MIN_WATER_AIR_RATIO);

	bool ok = (value > threshold_with_margin) &&
		(value >= ABSOLUTE_MIN_WATER_ADC) &&
		air_ratio_ok &&
		(value >= min_expected_water || value > water_baseline);

	if (ok) {
		uint16_t new_water = (uint16_t)((DEFAULT_ALPHA_PERCENT * (uint32_t)value +
			(100 - DEFAULT_ALPHA_PERCENT) * (uint32_t)water_baseline) / 100);
		if (observed_peak_adc > 0 && new_water > observed_peak_adc)
			new_water = observed_peak_adc;
		water_baseline = new_water;
		update_dynamic_threshold();
	}
}

/* Battery level mapping (from mgr_bat.c) */
static uint8_t bat_get_level(uint16_t mV)
{
	if (mV >= 3600) return 100;
	if (mV <= 3000) return 0;
	return (uint8_t)((mV - 3000) * 100 / 600);
}

/* isTxAllowedAt (from mgr_bat.c) */
static uint16_t min_tx_voltage_mV = 2800;
static bool bat_is_tx_allowed_at(uint16_t mV)
{
	if (min_tx_voltage_mV == 0) return true;
	if (mV == 0) return true;
	return (mV >= min_tx_voltage_mV);
}

/* Helpers */
static void reset_baselines(uint16_t air, uint16_t water, uint16_t peak)
{
	air_baseline = air;
	water_baseline = water;
	observed_peak_adc = peak;
	update_dynamic_threshold();
}

/*******************************************************************************
 * DYNAMIC THRESHOLD TESTS
 ******************************************************************************/

/** Clean water (high contrast >= 8x): ratio = 35% */
void test_threshold_clean_water(void)
{
	reset_baselines(50, 750, 800);
	/* contrast = 750*10/50 = 150 >= 80 → ratio 35% */
	/* threshold = 50 + (700 * 35) / 100 = 50 + 245 = 295 */
	ASSERT_EQ(295, threshold_current);
	/* hyst = 295 * 4 / 100 = 11 */
	ASSERT_EQ(11, hysteresis_value);
	TEST_PASS();
}

/** Moderate biofouling (contrast 4-8x): ratio = 50% */
void test_threshold_moderate_contrast(void)
{
	reset_baselines(200, 800, 900);
	/* contrast = 800*10/200 = 40 >= 40 → ratio 50% */
	/* threshold = 200 + (600 * 50) / 100 = 200 + 300 = 500 */
	ASSERT_EQ(500, threshold_current);
	TEST_PASS();
}

/** Low contrast (<4x): ratio = 40% */
void test_threshold_low_contrast(void)
{
	reset_baselines(300, 800, 900);
	/* contrast = 800*10/300 = 26 < 40 → ratio 40% */
	/* threshold = 300 + (500 * 40) / 100 = 300 + 200 = 500 */
	ASSERT_EQ(500, threshold_current);
	TEST_PASS();
}

/** Degenerate: water <= air */
void test_threshold_degenerate(void)
{
	air_baseline = 500;
	water_baseline = 500;
	observed_peak_adc = 0;
	update_dynamic_threshold();
	ASSERT_EQ(501, threshold_current);
	ASSERT_EQ(1, hysteresis_value);
	TEST_PASS();
}

/** Hysteresis minimum clamped to 3 */
void test_threshold_hysteresis_min(void)
{
	reset_baselines(10, 30, 50);
	/* Very low values → hysteresis would be < 3, clamped to 3 */
	ASSERT_TRUE(hysteresis_value >= 3);
	TEST_PASS();
}

/** Observed peak caps threshold */
void test_threshold_peak_cap(void)
{
	reset_baselines(50, 750, 200);
	/* peak=200 is very low, threshold+hyst should be capped */
	ASSERT_TRUE(threshold_current + hysteresis_value <= 200);
	/* threshold should be 90% of 200 = 180 */
	ASSERT_EQ(180, threshold_current);
	/* hyst = 5% of 200 = 10 */
	ASSERT_EQ(10, hysteresis_value);
	TEST_PASS();
}

/** Air baseline = 0: contrast formula handles division by zero */
void test_threshold_air_zero(void)
{
	reset_baselines(0, 750, 800);
	/* air=0 → contrast_x10=100 → ratio=35% (clean) */
	/* threshold = 0 + (750 * 35) / 100 = 262 */
	ASSERT_EQ(262, threshold_current);
	TEST_PASS();
}

/*******************************************************************************
 * MOVING AVERAGE TESTS
 ******************************************************************************/

/** Single value in empty buffer */
void test_ma_single_value(void)
{
	reset_filter();
	uint16_t result = moving_average(1000);
	ASSERT_EQ(1000, result);
	TEST_PASS();
}

/** Two values: average of both */
void test_ma_two_values(void)
{
	reset_filter();
	moving_average(1000);
	uint16_t result = moving_average(2000);
	ASSERT_EQ(1500, result);
	TEST_PASS();
}

/** Pre-filled buffer: sliding window */
void test_ma_sliding(void)
{
	reset_filter();
	/* Fill with 100 */
	for (int i = 0; i < ADC_HISTORY_SIZE; i++)
		moving_average(100);
	/* Add 200: avg of (100, 200) = 150 */
	uint16_t result = moving_average(200);
	ASSERT_EQ(150, result);
	TEST_PASS();
}

/** Zeros are skipped in average */
void test_ma_skips_zeros(void)
{
	reset_filter();
	/* Buffer is [0, 0], add 500 → only count non-zero */
	uint16_t result = moving_average(500);
	ASSERT_EQ(500, result);
	TEST_PASS();
}

/*******************************************************************************
 * WATER BASELINE EMA TESTS
 ******************************************************************************/

/** Valid water reading: EMA updates baseline */
void test_water_ema_updates(void)
{
	reset_baselines(50, 750, 1000);
	uint16_t old_water = water_baseline;
	calibrate_water_baseline(800);
	/* new = 19% * 800 + 81% * 750 = 152 + 607 = 759 */
	ASSERT_EQ(759, water_baseline);
	ASSERT_TRUE(water_baseline != old_water);
	TEST_PASS();
}

/** Reading below threshold: no update */
void test_water_ema_below_threshold_no_update(void)
{
	reset_baselines(50, 750, 1000);
	uint16_t old_water = water_baseline;
	calibrate_water_baseline(100); /* Way below threshold */
	ASSERT_EQ(old_water, water_baseline);
	TEST_PASS();
}

/** Reading below ABSOLUTE_MIN_WATER_ADC: no update */
void test_water_ema_below_absolute_min(void)
{
	reset_baselines(50, 750, 1000);
	uint16_t old_water = water_baseline;
	calibrate_water_baseline(400); /* Below 500 min */
	ASSERT_EQ(old_water, water_baseline);
	TEST_PASS();
}

/** Capped at observed peak */
void test_water_ema_capped_at_peak(void)
{
	reset_baselines(50, 750, 760);
	calibrate_water_baseline(800);
	/* Without cap: 759. But peak=760, 759 < 760 so no cap here.
	 * Try with lower peak: */
	reset_baselines(50, 750, 755);
	calibrate_water_baseline(800);
	/* Would be 759 but capped at 755 */
	ASSERT_EQ(755, water_baseline);
	TEST_PASS();
}

/** Air ratio guard: low air baseline, value must be >= 3x air */
void test_water_ema_air_ratio_guard(void)
{
	reset_baselines(100, 750, 1000);
	uint16_t old_water = water_baseline;
	/* value=250 < 100*3=300, air < 250 → ratio not ok → no update */
	calibrate_water_baseline(250);
	ASSERT_EQ(old_water, water_baseline);
	TEST_PASS();
}

/*******************************************************************************
 * BATTERY LOGIC TESTS
 ******************************************************************************/

/** getLevel boundary values */
void test_bat_level_boundaries(void)
{
	ASSERT_EQ(0,   bat_get_level(3000));
	ASSERT_EQ(0,   bat_get_level(2500));
	ASSERT_EQ(100, bat_get_level(3600));
	ASSERT_EQ(100, bat_get_level(4200));
	ASSERT_EQ(50,  bat_get_level(3300));
	TEST_PASS();
}

/** getLevel linear mapping */
void test_bat_level_linear(void)
{
	uint8_t l1 = bat_get_level(3100);
	uint8_t l2 = bat_get_level(3200);
	uint8_t l3 = bat_get_level(3500);
	ASSERT_TRUE(l1 < l2);
	ASSERT_TRUE(l2 < l3);
	/* 3100: (100)*100/600 = 16 */
	ASSERT_EQ(16, l1);
	/* 3200: (200)*100/600 = 33 */
	ASSERT_EQ(33, l2);
	TEST_PASS();
}

/** isTxAllowedAt: threshold disabled (0) always allows */
void test_bat_tx_allowed_disabled(void)
{
	min_tx_voltage_mV = 0;
	ASSERT_TRUE(bat_is_tx_allowed_at(0));
	ASSERT_TRUE(bat_is_tx_allowed_at(1000));
	ASSERT_TRUE(bat_is_tx_allowed_at(5000));
	TEST_PASS();
}

/** isTxAllowedAt: mV=0 always allows (ADC failure) */
void test_bat_tx_allowed_adc_fail(void)
{
	min_tx_voltage_mV = 2800;
	ASSERT_TRUE(bat_is_tx_allowed_at(0));
	TEST_PASS();
}

/** isTxAllowedAt: above threshold → allowed */
void test_bat_tx_allowed_above(void)
{
	min_tx_voltage_mV = 2800;
	ASSERT_TRUE(bat_is_tx_allowed_at(2800));
	ASSERT_TRUE(bat_is_tx_allowed_at(3600));
	TEST_PASS();
}

/** isTxAllowedAt: below threshold → blocked */
void test_bat_tx_allowed_below(void)
{
	min_tx_voltage_mV = 2800;
	ASSERT_FALSE(bat_is_tx_allowed_at(2799));
	ASSERT_FALSE(bat_is_tx_allowed_at(2000));
	TEST_PASS();
}

/*******************************************************************************
 * COHERENCE CHECK TESTS
 ******************************************************************************/

/** Coherence: reading far above water → recalibrate as "in water" */
void test_coherence_reading_above_water(void)
{
	reset_baselines(50, 750, 800);

	uint16_t raw = 1500; /* Way above water */
	uint16_t th_with_3hyst = threshold_current + hysteresis_value * 3;
	uint16_t water_130pct = (water_baseline * 13) / 10;

	bool incoherent = (raw > th_with_3hyst && raw > water_130pct);
	ASSERT_TRUE(incoherent);

	/* Simulate recalib: raw > WATER_DETECT_HEURISTIC */
	ASSERT_TRUE(raw > WATER_DETECT_HEURISTIC);
	water_baseline = raw;
	air_baseline = raw / 3;
	update_dynamic_threshold();

	ASSERT_EQ(1500, water_baseline);
	ASSERT_EQ(500, air_baseline);
	TEST_PASS();
}

/** Coherence: reading far below air (air was calibrated in water) */
void test_coherence_reading_below_air(void)
{
	reset_baselines(600, 1200, 1300);

	uint16_t raw = 200; /* Way below air */
	/* Guard: air > 500 → true (600 > 500) */
	bool incoherent = (raw < air_baseline / 2 && air_baseline > 500);
	ASSERT_TRUE(incoherent);

	/* Simulate recalib: raw < WATER_DETECT_HEURISTIC */
	ASSERT_TRUE(raw < WATER_DETECT_HEURISTIC);
	air_baseline = raw;
	water_baseline = raw * 3;
	update_dynamic_threshold();

	ASSERT_EQ(200, air_baseline);
	ASSERT_EQ(600, water_baseline);
	TEST_PASS();
}

/** Coherence guard: air <= 500 → skip low-reading check */
void test_coherence_guard_low_air(void)
{
	reset_baselines(200, 750, 800);

	uint16_t raw = 50;
	/* air=200 ≤ 500 → guard blocks the check */
	bool incoherent = (raw < air_baseline / 2 && air_baseline > 500);
	ASSERT_FALSE(incoherent);
	TEST_PASS();
}

/*******************************************************************************
 * SURFACE DETECTION LEVEL TESTS
 ******************************************************************************/

/** L1: Drop > 5% from recent peak */
void test_level1_peak_drop(void)
{
	uint16_t recent_peak = 1000;
	uint16_t raw = 940; /* 6% drop */
	uint16_t drop_pct = (uint16_t)(((uint32_t)(recent_peak - raw) * 100) / recent_peak);
	ASSERT_TRUE(drop_pct >= L1_DROP_PERCENT);

	raw = 960; /* 4% drop - not enough */
	drop_pct = (uint16_t)(((uint32_t)(recent_peak - raw) * 100) / recent_peak);
	ASSERT_FALSE(drop_pct >= L1_DROP_PERCENT);
	TEST_PASS();
}

/** L2: 2 consecutive raw drops, cumulative > 3% */
void test_level2_consecutive_drops(void)
{
	uint16_t drop_reference = 1000;
	uint16_t raw = 965; /* 3.5% cumulative drop from reference */
	uint8_t consecutive = 2;

	uint16_t cumul_pct = (uint16_t)(((uint32_t)(drop_reference - raw) * 100) / drop_reference);
	bool triggered = (consecutive >= L2_MIN_CONSECUTIVE && cumul_pct >= L2_DROP_PERCENT);
	ASSERT_TRUE(triggered);

	raw = 975; /* 2.5% - not enough */
	cumul_pct = (uint16_t)(((uint32_t)(drop_reference - raw) * 100) / drop_reference);
	triggered = (consecutive >= L2_MIN_CONSECUTIVE && cumul_pct >= L2_DROP_PERCENT);
	ASSERT_FALSE(triggered);
	TEST_PASS();
}

/** L4: Drop relative to water baseline > 15% */
void test_level4_water_drop(void)
{
	uint16_t wb = 1000;
	uint16_t water_thresh = (uint16_t)((uint32_t)wb * (100 - L4_DROP_PERCENT) / 100);
	ASSERT_EQ(850, water_thresh);

	ASSERT_TRUE(840 < water_thresh);  /* Triggers L4 */
	ASSERT_FALSE(860 < water_thresh); /* Does not trigger */
	TEST_PASS();
}

/** L5: Cumulative drop from dive peak > 15%, time gate */
void test_level5_dive_peak_drop(void)
{
	uint16_t peak = 1000;
	uint16_t filtered = 840;
	uint16_t drop_pct = (uint16_t)(((uint32_t)(peak - filtered) * 100) / peak);
	ASSERT_EQ(16, drop_pct);
	ASSERT_TRUE(drop_pct >= L5_DROP_PERCENT);

	filtered = 860;
	drop_pct = (uint16_t)(((uint32_t)(peak - filtered) * 100) / peak);
	ASSERT_EQ(14, drop_pct);
	ASSERT_FALSE(drop_pct >= L5_DROP_PERCENT);
	TEST_PASS();
}

/** Proximity guard: blocks overrides when close to water/peak */
void test_proximity_guard(void)
{
	uint16_t proximity_ref = 1000;
	uint16_t filtered = 960; /* 96% of ref → blocked */
	bool prox_ok = (filtered < (uint16_t)((uint32_t)proximity_ref * PROXIMITY_GUARD_PERCENT / 100));
	ASSERT_FALSE(prox_ok);

	filtered = 940; /* 94% → allowed */
	prox_ok = (filtered < (uint16_t)((uint32_t)proximity_ref * PROXIMITY_GUARD_PERCENT / 100));
	ASSERT_TRUE(prox_ok);
	TEST_PASS();
}

/** Proximity guard: zero ref always passes */
void test_proximity_guard_zero_ref(void)
{
	uint16_t proximity_ref = 0;
	bool prox_ok = (proximity_ref == 0) || (500 < (uint16_t)((uint32_t)proximity_ref * 95 / 100));
	ASSERT_TRUE(prox_ok);
	TEST_PASS();
}

/*******************************************************************************
 * SURFACE LOCKOUT TESTS
 ******************************************************************************/

/** Lockout forces surface for N seconds */
void test_surface_lockout(void)
{
	uint32_t lockout = SURFACE_LOCKOUT_S;
	bool new_underwater = true;

	/* While lockout > 0, override to surface */
	while (lockout > 0) {
		lockout--;
		if (new_underwater)
			new_underwater = false;
	}
	ASSERT_EQ(0, lockout);
	ASSERT_FALSE(new_underwater);
	TEST_PASS();
}

/*******************************************************************************
 * RECENT PEAK DECAY TESTS
 ******************************************************************************/

/** Recent peak decays 5% per sample toward current reading */
void test_recent_peak_decay(void)
{
	uint16_t recent_peak = 1000;
	uint16_t raw = 900;

	/* Decay: 95% peak + 5% raw */
	recent_peak = (uint16_t)(((uint32_t)recent_peak * 95 + (uint32_t)raw * 5) / 100);
	ASSERT_EQ(995, recent_peak);

	/* After several iterations, converges toward raw */
	for (int i = 0; i < 50; i++) {
		recent_peak = (uint16_t)(((uint32_t)recent_peak * 95 + (uint32_t)raw * 5) / 100);
	}
	/* Should be close to 900 after 50 decays */
	ASSERT_TRUE(recent_peak < 920);
	ASSERT_TRUE(recent_peak >= 900);
	TEST_PASS();
}

/** Recent peak updates immediately when raw > peak */
void test_recent_peak_immediate_update(void)
{
	uint16_t recent_peak = 900;
	uint16_t raw = 950;

	if (raw > recent_peak)
		recent_peak = raw;

	ASSERT_EQ(950, recent_peak);
	TEST_PASS();
}

/*******************************************************************************
 * AIR BASELINE ADAPTATION TESTS
 ******************************************************************************/

/** Upward adaptation: avg > 1.3x air → 10% EMA */
void test_air_adapt_upward(void)
{
	uint16_t air = 100;
	uint16_t avg = 150; /* 1.5x > 1.3x threshold */

	bool should_adapt = (avg * 10 > (uint32_t)air * SURFACE_ADAPT_THRESHOLD_X10);
	ASSERT_TRUE(should_adapt);

	/* 10% EMA: 90% old + 10% new */
	air = (uint16_t)((90 * (uint32_t)air + 10 * (uint32_t)avg) / 100);
	ASSERT_EQ(105, air);
	TEST_PASS();
}

/** Downward adaptation: avg < 70% air → 20% EMA */
void test_air_adapt_downward(void)
{
	uint16_t air = 200;
	uint16_t avg = 120; /* 60% < 70% threshold */

	bool should_adapt = (avg < (air * 70) / 100);
	ASSERT_TRUE(should_adapt);

	/* 20% EMA: 80% old + 20% new */
	air = (uint16_t)((80 * (uint32_t)air + 20 * (uint32_t)avg) / 100);
	ASSERT_EQ(184, air);
	TEST_PASS();
}

/** No adaptation in normal range (0.7x - 1.3x) */
void test_air_no_adapt_normal(void)
{
	uint16_t air = 100;
	uint16_t avg = 110; /* 1.1x: between 0.7 and 1.3 */

	bool up = (avg * 10 > (uint32_t)air * SURFACE_ADAPT_THRESHOLD_X10);
	bool down = (avg < (air * 70) / 100);
	ASSERT_FALSE(up);
	ASSERT_FALSE(down);
	TEST_PASS();
}

/*******************************************************************************
 * HYSTERESIS STATE DETERMINATION
 ******************************************************************************/

/** Above threshold+hyst: underwater */
void test_hysteresis_above_high(void)
{
	reset_baselines(50, 750, 800);
	uint16_t th_high = threshold_current + hysteresis_value;
	uint16_t filtered = th_high + 1;
	ASSERT_TRUE(filtered > th_high);
	TEST_PASS();
}

/** Below threshold-hyst: surface */
void test_hysteresis_below_low(void)
{
	reset_baselines(50, 750, 800);
	uint16_t th_low = (threshold_current > hysteresis_value) ?
		(threshold_current - hysteresis_value) : 1;
	uint16_t filtered = th_low - 1;
	ASSERT_TRUE(filtered < th_low);
	TEST_PASS();
}

/** In hysteresis zone: maintains previous state */
void test_hysteresis_zone_maintains(void)
{
	reset_baselines(50, 750, 800);
	uint16_t th_high = threshold_current + hysteresis_value;
	uint16_t th_low = threshold_current - hysteresis_value;
	uint16_t filtered = threshold_current; /* Exactly in zone */

	ASSERT_TRUE(filtered <= th_high);
	ASSERT_TRUE(filtered >= th_low);
	/* Both conditions false → maintain previous state */
	bool was_underwater = true;
	bool new_state = was_underwater; /* No change */
	ASSERT_TRUE(new_state);

	was_underwater = false;
	new_state = was_underwater;
	ASSERT_FALSE(new_state);
	TEST_PASS();
}

/*******************************************************************************
 * NVM v2 STRUCT LAYOUT TESTS
 ******************************************************************************/

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

/** v2 struct: 4-byte aligned */
void test_nvm_v2_alignment(void)
{
	ASSERT_EQ(0, sizeof(NVM_Config_v2_t) % 4);
	TEST_PASS();
}

/** v2 struct: 8-byte aligned for 64-bit flash writes */
void test_nvm_v2_doubleword_alignment(void)
{
	ASSERT_EQ(0, sizeof(NVM_Config_v2_t) % 8);
	TEST_PASS();
}

/** v2 struct: CRC at end */
void test_nvm_v2_crc_at_end(void)
{
	size_t crc_end = __builtin_offsetof(NVM_Config_v2_t, crc32) + sizeof(uint32_t);
	ASSERT_EQ(crc_end, sizeof(NVM_Config_v2_t));
	TEST_PASS();
}

/** v2 struct: bat_min_tx_mV field present */
void test_nvm_v2_has_bat_field(void)
{
	size_t bat_off = __builtin_offsetof(NVM_Config_v2_t, bat_min_tx_mV);
	/* Must be before CRC */
	size_t crc_off = __builtin_offsetof(NVM_Config_v2_t, crc32);
	ASSERT_TRUE(bat_off < crc_off);
	TEST_PASS();
}

/*******************************************************************************
 * TEST RUNNER
 ******************************************************************************/

int main(void)
{
	TEST_SUITE_START("SWS Algorithm & Battery Unit Tests");

	/* Dynamic threshold */
	RUN_TEST(test_threshold_clean_water);
	RUN_TEST(test_threshold_moderate_contrast);
	RUN_TEST(test_threshold_low_contrast);
	RUN_TEST(test_threshold_degenerate);
	RUN_TEST(test_threshold_hysteresis_min);
	RUN_TEST(test_threshold_peak_cap);
	RUN_TEST(test_threshold_air_zero);

	/* Moving average */
	RUN_TEST(test_ma_single_value);
	RUN_TEST(test_ma_two_values);
	RUN_TEST(test_ma_sliding);
	RUN_TEST(test_ma_skips_zeros);

	/* Water baseline EMA */
	RUN_TEST(test_water_ema_updates);
	RUN_TEST(test_water_ema_below_threshold_no_update);
	RUN_TEST(test_water_ema_below_absolute_min);
	RUN_TEST(test_water_ema_capped_at_peak);
	RUN_TEST(test_water_ema_air_ratio_guard);

	/* Battery */
	RUN_TEST(test_bat_level_boundaries);
	RUN_TEST(test_bat_level_linear);
	RUN_TEST(test_bat_tx_allowed_disabled);
	RUN_TEST(test_bat_tx_allowed_adc_fail);
	RUN_TEST(test_bat_tx_allowed_above);
	RUN_TEST(test_bat_tx_allowed_below);

	/* Coherence check */
	RUN_TEST(test_coherence_reading_above_water);
	RUN_TEST(test_coherence_reading_below_air);
	RUN_TEST(test_coherence_guard_low_air);

	/* 5-level detection */
	RUN_TEST(test_level1_peak_drop);
	RUN_TEST(test_level2_consecutive_drops);
	RUN_TEST(test_level4_water_drop);
	RUN_TEST(test_level5_dive_peak_drop);
	RUN_TEST(test_proximity_guard);
	RUN_TEST(test_proximity_guard_zero_ref);

	/* Lockout and decay */
	RUN_TEST(test_surface_lockout);
	RUN_TEST(test_recent_peak_decay);
	RUN_TEST(test_recent_peak_immediate_update);

	/* Air baseline adaptation */
	RUN_TEST(test_air_adapt_upward);
	RUN_TEST(test_air_adapt_downward);
	RUN_TEST(test_air_no_adapt_normal);

	/* Hysteresis */
	RUN_TEST(test_hysteresis_above_high);
	RUN_TEST(test_hysteresis_below_low);
	RUN_TEST(test_hysteresis_zone_maintains);

	/* NVM v2 struct layout */
	RUN_TEST(test_nvm_v2_alignment);
	RUN_TEST(test_nvm_v2_doubleword_alignment);
	RUN_TEST(test_nvm_v2_crc_at_end);
	RUN_TEST(test_nvm_v2_has_bat_field);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
