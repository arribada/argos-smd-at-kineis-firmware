/**
 * @file    test_sws_degraded.c
 * @brief   Regression spec for the SWS degraded beacon mode.
 *
 * Mirrors mgr_sws.c sections 7 (dive-timeout escalation) + 9 (degraded
 * override): after 3x max_dive_time_s stuck underwater (healthy sensor) — or 1x
 * for a CONFIRMED stuck/no-variance fault — hold SURFACE for a full
 * max_dive_time_s window (beacon), then drop back underwater to re-arm
 * escalation. Exit early on a frank air reading (raw < threshold_low).
 */

#include "test_framework.h"

#define MAX_CONSECUTIVE_DIVE_TIMEOUTS 3

static uint32_t fk_tick;
static uint32_t HAL_GetTick(void) { return fk_tick; }

static uint8_t  consecutive_dive_timeouts;
static int      degraded_mode;
static uint32_t degraded_start_tick;
static uint32_t state_enter_tick;
static int      sensor_faulted_flag;   /* mirror of (STUCK|NO_VARIANCE) latched */

/* mirror of section 7 escalation; call when a dive-timeout fires.
 * returns new_is_underwater (false = forced surface at the 3rd). */
static int escalate(uint32_t max_dive_s)
{
	int new_is_underwater = 1;
	consecutive_dive_timeouts++;
	/* Healthy sensor: 3x backstop. CONFIRMED stuck/no-variance fault: 1x so a
	 * dead electrode beacons in ~2h not ~6h (mgr_sws.c section 7). */
	if (consecutive_dive_timeouts >= MAX_CONSECUTIVE_DIVE_TIMEOUTS ||
	    (sensor_faulted_flag && consecutive_dive_timeouts >= 1)) {
		new_is_underwater = 0;
		degraded_mode = 1;
		degraded_start_tick = HAL_GetTick();
		consecutive_dive_timeouts = 0;
		(void)max_dive_s;
	} else {
		state_enter_tick = HAL_GetTick();
	}
	return new_is_underwater;
}

/* mirror of section 9 override; in_underwater = result of the normal state
 * machine for this sample. */
static int degraded_override(int in_underwater, uint16_t raw, uint16_t threshold_low,
                             uint32_t max_dive_s)
{
	if (!degraded_mode)
		return in_underwater;
	uint32_t window_ms = max_dive_s * 1000u;
	if (raw < threshold_low) {
		degraded_mode = 0;
		return 0;                  /* real surface recovered */
	} else if ((uint32_t)(HAL_GetTick() - degraded_start_tick) >= window_ms) {
		degraded_mode = 0;
		return 1;                  /* window done -> underwater */
	}
	return 0;                       /* hold beacon surface */
}

static void reset(void)
{
	fk_tick = 100000; consecutive_dive_timeouts = 0;
	degraded_mode = 0; degraded_start_tick = 0; state_enter_tick = 0;
	sensor_faulted_flag = 0;
}

/* max_dive=7200s -> window 7200000 ms; threshold_low=250; stuck raw=900 */

static void test_enters_after_3_timeouts(void)
{
	reset();
	ASSERT_EQ(1, escalate(7200));   /* timeout 1/3: still UW */
	ASSERT_EQ(0, degraded_mode);
	ASSERT_EQ(1, escalate(7200));   /* 2/3: still UW */
	ASSERT_EQ(0, degraded_mode);
	ASSERT_EQ(0, escalate(7200));   /* 3/3: forced surface */
	ASSERT_EQ(1, degraded_mode);
	ASSERT_EQ(0, consecutive_dive_timeouts);  /* counter reset */
	TEST_PASS();
}

static void test_faulted_enters_after_1_timeout(void)
{
	reset();
	sensor_faulted_flag = 1;        /* confirmed STUCK/NO_VARIANCE electrode */
	ASSERT_EQ(0, escalate(7200));   /* 1st dive-timeout already forces surface */
	ASSERT_EQ(1, degraded_mode);
	ASSERT_EQ(0, consecutive_dive_timeouts);
	TEST_PASS();
}

static void test_healthy_sensor_keeps_3x(void)
{
	reset();
	sensor_faulted_flag = 0;        /* healthy: no early escalation */
	ASSERT_EQ(1, escalate(7200));
	ASSERT_EQ(0, degraded_mode);    /* not on the 1st */
	ASSERT_EQ(1, escalate(7200));
	ASSERT_EQ(0, degraded_mode);    /* not on the 2nd either */
	TEST_PASS();
}

static void test_holds_surface_during_window(void)
{
	reset();
	escalate(7200); escalate(7200); escalate(7200);   /* enter degraded */
	/* 1h into the 2h window, sensor still stuck-high -> hold surface */
	fk_tick += 3600u * 1000u;
	ASSERT_EQ(0, degraded_override(1 /*normal says UW*/, 900, 250, 7200));
	ASSERT_EQ(1, degraded_mode);    /* still degraded */
	TEST_PASS();
}

static void test_window_ends_to_underwater(void)
{
	reset();
	escalate(7200); escalate(7200); escalate(7200);
	/* exactly max_dive_time_s later -> window done -> back underwater */
	fk_tick += 7200u * 1000u;
	ASSERT_EQ(1, degraded_override(1, 900, 250, 7200));
	ASSERT_EQ(0, degraded_mode);    /* exited */
	TEST_PASS();
}

static void test_exits_on_real_surface(void)
{
	reset();
	escalate(7200); escalate(7200); escalate(7200);
	/* 30min in, sensor genuinely reads air (raw 100 < threshold_low 250) */
	fk_tick += 1800u * 1000u;
	ASSERT_EQ(0, degraded_override(1, 100, 250, 7200));  /* surface, exit */
	ASSERT_EQ(0, degraded_mode);    /* exited early */
	TEST_PASS();
}

static void test_re_arms_after_window(void)
{
	reset();
	escalate(7200); escalate(7200); escalate(7200);     /* degraded */
	fk_tick += 7200u * 1000u;
	degraded_override(1, 900, 250, 7200);               /* window done -> UW */
	ASSERT_EQ(0, degraded_mode);
	/* now underwater again: 3 fresh timeouts re-enter degraded */
	ASSERT_EQ(1, escalate(7200));
	ASSERT_EQ(1, escalate(7200));
	ASSERT_EQ(0, escalate(7200));
	ASSERT_EQ(1, degraded_mode);    /* re-armed */
	TEST_PASS();
}

static void test_no_override_when_not_degraded(void)
{
	reset();
	/* degraded_mode false -> override is a pass-through */
	ASSERT_EQ(1, degraded_override(1, 900, 250, 7200));
	ASSERT_EQ(0, degraded_override(0, 100, 250, 7200));
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("SWS degraded beacon mode");

	RUN_TEST(test_enters_after_3_timeouts);
	RUN_TEST(test_faulted_enters_after_1_timeout);
	RUN_TEST(test_healthy_sensor_keeps_3x);
	RUN_TEST(test_holds_surface_during_window);
	RUN_TEST(test_window_ends_to_underwater);
	RUN_TEST(test_exits_on_real_surface);
	RUN_TEST(test_re_arms_after_window);
	RUN_TEST(test_no_override_when_not_degraded);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
