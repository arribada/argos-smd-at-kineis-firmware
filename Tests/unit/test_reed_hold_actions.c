/**
 * @file    test_reed_hold_actions.c
 * @brief   Unit tests for Phase C: reed switch hold-duration action mapping
 *
 *   < REED_SHORT_HOLD_MIN_MS                 -> NONE (tap/glitch)
 *   REED_SHORT_HOLD_MIN_MS  .. _MAX_MS       -> SHORT (status indication)
 *   REED_MEDIUM_HOLD_MIN_MS .. _MAX_MS       -> MEDIUM (toggle deploy)
 *   >= REED_SHUTDOWN_HOLD_MS                 -> LONG (shutdown — fires during
 *                                                hold, not on release; not
 *                                                covered here)
 */

#include "test_framework.h"

#define REED_SHORT_HOLD_MIN_MS   1000u
#define REED_SHORT_HOLD_MAX_MS   3000u
#define REED_MEDIUM_HOLD_MIN_MS  3000u
#define REED_MEDIUM_HOLD_MAX_MS  10000u
#define REED_SHUTDOWN_HOLD_MS    10000u

typedef enum {
	ACTION_NONE = 0,
	ACTION_SHORT,
	ACTION_MEDIUM,
	ACTION_LONG_SHUTDOWN,
} ReedAction_t;

/* Mirror of dispatch logic from kns_app_uw_doppler.c
 * (the MAGNET_OFF event path, which classifies hold duration). */
static ReedAction_t classify_hold(uint32_t hold_ms)
{
	if (hold_ms >= REED_SHUTDOWN_HOLD_MS)
		return ACTION_LONG_SHUTDOWN;
	if (hold_ms >= REED_MEDIUM_HOLD_MIN_MS && hold_ms < REED_MEDIUM_HOLD_MAX_MS)
		return ACTION_MEDIUM;
	if (hold_ms >= REED_SHORT_HOLD_MIN_MS && hold_ms < REED_SHORT_HOLD_MAX_MS)
		return ACTION_SHORT;
	return ACTION_NONE;
}

void test_zero_hold_is_none(void)
{
	TEST_START("hold=0ms -> NONE");
	ASSERT_EQ(ACTION_NONE, classify_hold(0));
	TEST_PASS();
}

void test_tap_below_short_is_none(void)
{
	TEST_START("hold=500ms -> NONE (below SHORT threshold)");
	ASSERT_EQ(ACTION_NONE, classify_hold(500));
	TEST_PASS();
}

void test_short_at_min_threshold(void)
{
	TEST_START("hold=1000ms -> SHORT (at min)");
	ASSERT_EQ(ACTION_SHORT, classify_hold(1000));
	TEST_PASS();
}

void test_short_mid_range(void)
{
	TEST_START("hold=2000ms -> SHORT (mid range)");
	ASSERT_EQ(ACTION_SHORT, classify_hold(2000));
	TEST_PASS();
}

void test_short_just_below_max(void)
{
	TEST_START("hold=2999ms -> SHORT (just below MAX)");
	ASSERT_EQ(ACTION_SHORT, classify_hold(2999));
	TEST_PASS();
}

void test_boundary_3000_is_medium(void)
{
	TEST_START("hold=3000ms -> MEDIUM (exact boundary)");
	ASSERT_EQ(ACTION_MEDIUM, classify_hold(3000));
	TEST_PASS();
}

void test_medium_mid_range(void)
{
	TEST_START("hold=5000ms -> MEDIUM (mid range)");
	ASSERT_EQ(ACTION_MEDIUM, classify_hold(5000));
	TEST_PASS();
}

void test_medium_just_below_max(void)
{
	TEST_START("hold=9999ms -> MEDIUM (just below 10s)");
	ASSERT_EQ(ACTION_MEDIUM, classify_hold(9999));
	TEST_PASS();
}

void test_boundary_10000_is_shutdown(void)
{
	TEST_START("hold=10000ms -> LONG_SHUTDOWN (exact boundary)");
	ASSERT_EQ(ACTION_LONG_SHUTDOWN, classify_hold(10000));
	TEST_PASS();
}

void test_very_long_hold_is_shutdown(void)
{
	TEST_START("hold=60000ms -> LONG_SHUTDOWN");
	ASSERT_EQ(ACTION_LONG_SHUTDOWN, classify_hold(60000));
	TEST_PASS();
}

/* Boundary safety: ensure no overlapping ranges produce ambiguous classification */
void test_no_overlap_at_3s_boundary(void)
{
	TEST_START("Boundary 3s: 2999ms=SHORT, 3000ms=MEDIUM");
	ASSERT_TRUE(classify_hold(2999) == ACTION_SHORT);
	ASSERT_TRUE(classify_hold(3000) == ACTION_MEDIUM);
	TEST_PASS();
}

void test_no_overlap_at_10s_boundary(void)
{
	TEST_START("Boundary 10s: 9999ms=MEDIUM, 10000ms=SHUTDOWN");
	ASSERT_TRUE(classify_hold(9999) == ACTION_MEDIUM);
	ASSERT_TRUE(classify_hold(10000) == ACTION_LONG_SHUTDOWN);
	TEST_PASS();
}

int main(void)
{
	printf("=== Reed hold-duration action mapping tests (Phase C) ===\n");

	test_zero_hold_is_none();
	test_tap_below_short_is_none();
	test_short_at_min_threshold();
	test_short_mid_range();
	test_short_just_below_max();
	test_boundary_3000_is_medium();
	test_medium_mid_range();
	test_medium_just_below_max();
	test_boundary_10000_is_shutdown();
	test_very_long_hold_is_shutdown();
	test_no_overlap_at_3s_boundary();
	test_no_overlap_at_10s_boundary();

	printf("\n%d/%d tests passed (%d failed)\n",
	       tests_passed, tests_run, tests_failed);
	return tests_failed ? 1 : 0;
}
