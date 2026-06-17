/**
 * @file    test_sws_l4_guards.c
 * @brief   Regression spec for the L4/L5 linkit-v4 hardening guards.
 *
 * Mirrors the L4/L5 surface-detection block in mgr_sws.c after the conformity
 * fix: L4 now requires (a) raw in the air zone (raw < threshold_high) AND
 * (b) two consecutive below-threshold samples (debounce); L5 carries the same
 * raw guard. Locks the contract that a single noisy MA2 dip, or a dip while
 * raw is still water-level, does NOT declare surface.
 */

#include "test_framework.h"

#define L4_DROP_PERCENT 15
#define L5_DROP_PERCENT 10
#define L5_MIN_TIME_SEC 10

static uint8_t l4_consecutive_below;

/* Returns the surface_level (0/4/5) for one sample, mirroring mgr_sws.c. */
static int l45_eval(uint16_t filtered, uint16_t raw_value, uint16_t water_baseline,
                    uint16_t peak, uint16_t threshold_current, uint16_t hysteresis,
                    uint32_t time_in_state, int is_underwater, int proximity_ok)
{
	int surface_level = 0;
	if (is_underwater && time_in_state >= 1u && proximity_ok) {
		uint16_t th_high = threshold_current + hysteresis;

		if (water_baseline > 0) {
			uint16_t water_thresh = (uint16_t)((uint32_t)water_baseline * (100 - L4_DROP_PERCENT) / 100);
			if (filtered < water_thresh && raw_value < th_high) {
				if (l4_consecutive_below < 0xFFu) l4_consecutive_below++;
				if (l4_consecutive_below >= 2u) surface_level = 4;
			} else {
				l4_consecutive_below = 0;
			}
		} else {
			l4_consecutive_below = 0;
		}

		if (surface_level == 0 && peak > 0 && filtered < peak &&
		    raw_value < th_high && time_in_state > L5_MIN_TIME_SEC) {
			uint16_t drop = (uint16_t)(((uint32_t)(peak - filtered) * 100) / peak);
			if (drop >= L5_DROP_PERCENT) surface_level = 5;
		}
	} else {
		l4_consecutive_below = 0;
	}
	return surface_level;
}

/* water=1000 -> L4 threshold = 850; threshold_current=300, hyst=12 -> th_high=312 */
static void test_l4_single_dip_does_not_trigger(void)
{
	l4_consecutive_below = 0;
	/* one sample below 850 with raw in air zone (200 < 312) → only 1 consecutive */
	ASSERT_EQ(0, l45_eval(800, 200, 1000, 0, 300, 12, 5, 1, 1));
	TEST_PASS();
}

static void test_l4_two_consecutive_triggers(void)
{
	l4_consecutive_below = 0;
	ASSERT_EQ(0, l45_eval(800, 200, 1000, 0, 300, 12, 5, 1, 1));  /* 1st */
	ASSERT_EQ(4, l45_eval(800, 200, 1000, 0, 300, 12, 5, 1, 1));  /* 2nd → L4 */
	TEST_PASS();
}

static void test_l4_blocked_when_raw_still_water(void)
{
	l4_consecutive_below = 0;
	/* filtered dips below 850 (stale-baseline artefact) but raw=900 is still
	 * water-level (>= th_high 312) → never counts, never triggers */
	ASSERT_EQ(0, l45_eval(800, 900, 1000, 0, 300, 12, 5, 1, 1));
	ASSERT_EQ(0, l45_eval(800, 900, 1000, 0, 300, 12, 5, 1, 1));
	ASSERT_EQ(0, l4_consecutive_below);  /* reset, never incremented */
	TEST_PASS();
}

static void test_l4_debounce_resets_on_bounce(void)
{
	l4_consecutive_below = 0;
	ASSERT_EQ(0, l45_eval(800, 200, 1000, 0, 300, 12, 5, 1, 1));  /* below: 1 */
	ASSERT_EQ(0, l45_eval(900, 200, 1000, 0, 300, 12, 5, 1, 1));  /* above 850: reset */
	ASSERT_EQ(0, l45_eval(800, 200, 1000, 0, 300, 12, 5, 1, 1));  /* below: 1 again */
	TEST_PASS();
}

static void test_l5_blocked_when_raw_still_water(void)
{
	l4_consecutive_below = 0;
	/* peak=1000, filtered=850 = 15% drop >= L5 10%, time>10s, but raw=900 still
	 * water-level → L5 must NOT fire (the new raw guard) */
	ASSERT_EQ(0, l45_eval(850, 900, 0 /*no water baseline*/, 1000, 300, 12, 15, 1, 1));
	TEST_PASS();
}

static void test_l5_fires_when_raw_in_air_zone(void)
{
	l4_consecutive_below = 0;
	/* same drop but raw=200 in air zone, time>10s → L5 fires */
	ASSERT_EQ(5, l45_eval(850, 200, 0, 1000, 300, 12, 15, 1, 1));
	TEST_PASS();
}

static void test_window_exit_resets_debounce(void)
{
	l4_consecutive_below = 1;
	(void)l45_eval(800, 200, 1000, 0, 300, 12, 5, 0 /*not underwater*/, 1);
	ASSERT_EQ(0, l4_consecutive_below);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("SWS L4/L5 hardening guards (linkit-v4 parity)");

	RUN_TEST(test_l4_single_dip_does_not_trigger);
	RUN_TEST(test_l4_two_consecutive_triggers);
	RUN_TEST(test_l4_blocked_when_raw_still_water);
	RUN_TEST(test_l4_debounce_resets_on_bounce);
	RUN_TEST(test_l5_blocked_when_raw_still_water);
	RUN_TEST(test_l5_fires_when_raw_in_air_zone);
	RUN_TEST(test_window_exit_resets_debounce);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
