/**
 * @file    test_bat_auto_shutdown.c
 * @brief   Unit tests for Phase F: battery low auto-shutdown
 *
 * Verifies the N-sample debounce streak counter:
 *   - VBAT below threshold: increments streak, triggers shutdown at N
 *   - VBAT above threshold: resets streak
 *   - VBAT==0 (read failure): does NOT count as critical (don't shutdown
 *     on a sensor glitch returning 0)
 */

#include "test_framework.h"

#define BAT_AUTO_SHUTDOWN_MV      2500u
#define BAT_AUTO_SHUTDOWN_HITS    3u

/* Mirror of the state machine logic from kns_app_uw_doppler.c */
typedef struct {
	uint8_t critical_hits;
	bool    shutdown_triggered; /* in real code, never returns from shutdown() */
} BatState_t;

static void bat_check(BatState_t *st, uint16_t vbat_mv)
{
	if (vbat_mv > 0 && vbat_mv < BAT_AUTO_SHUTDOWN_MV) {
		st->critical_hits++;
		if (st->critical_hits >= BAT_AUTO_SHUTDOWN_HITS) {
			st->shutdown_triggered = true;
		}
	} else {
		/* VBAT recovered (or read failure) — reset streak */
		st->critical_hits = 0;
	}
}

void test_single_low_does_not_shutdown(void)
{
	TEST_START("Single low reading does NOT shutdown");
	BatState_t st = {0};
	bat_check(&st, 2400);
	ASSERT_EQ(1, st.critical_hits);
	ASSERT_FALSE(st.shutdown_triggered);
	TEST_PASS();
}

void test_three_consecutive_low_shutdown(void)
{
	TEST_START("3 consecutive lows -> shutdown");
	BatState_t st = {0};
	bat_check(&st, 2400);
	bat_check(&st, 2300);
	bat_check(&st, 2200);
	ASSERT_TRUE(st.shutdown_triggered);
	TEST_PASS();
}

void test_recovery_resets_streak(void)
{
	TEST_START("Recovery resets streak");
	BatState_t st = {0};
	bat_check(&st, 2400);  /* 1 */
	bat_check(&st, 2400);  /* 2 */
	bat_check(&st, 3500);  /* recovered, streak=0 */
	ASSERT_EQ(0, st.critical_hits);
	ASSERT_FALSE(st.shutdown_triggered);
	bat_check(&st, 2400);  /* streak restarts at 1 */
	ASSERT_EQ(1, st.critical_hits);
	ASSERT_FALSE(st.shutdown_triggered);
	TEST_PASS();
}

void test_exactly_threshold_not_critical(void)
{
	TEST_START("VBAT exactly at threshold (2500mV) is NOT critical");
	BatState_t st = {0};
	bat_check(&st, 2500);
	bat_check(&st, 2500);
	bat_check(&st, 2500);
	ASSERT_EQ(0, st.critical_hits);
	ASSERT_FALSE(st.shutdown_triggered);
	TEST_PASS();
}

void test_vbat_zero_does_not_count(void)
{
	TEST_START("VBAT=0 (read failure) does NOT count as critical");
	BatState_t st = {0};
	bat_check(&st, 0);
	bat_check(&st, 0);
	bat_check(&st, 0);
	ASSERT_EQ(0, st.critical_hits);
	ASSERT_FALSE(st.shutdown_triggered);
	TEST_PASS();
}

void test_just_below_threshold_counts(void)
{
	TEST_START("VBAT just below threshold (2499mV) counts");
	BatState_t st = {0};
	bat_check(&st, 2499);
	ASSERT_EQ(1, st.critical_hits);
	TEST_PASS();
}

void test_alternating_does_not_shutdown(void)
{
	TEST_START("Alternating low/high does NOT accumulate");
	BatState_t st = {0};
	for (int i = 0; i < 10; i++) {
		bat_check(&st, 2400);  /* low */
		bat_check(&st, 3000);  /* recovered */
	}
	ASSERT_FALSE(st.shutdown_triggered);
	TEST_PASS();
}

int main(void)
{
	printf("=== Battery auto-shutdown tests (Phase F) ===\n");

	test_single_low_does_not_shutdown();
	test_three_consecutive_low_shutdown();
	test_recovery_resets_streak();
	test_exactly_threshold_not_critical();
	test_vbat_zero_does_not_count();
	test_just_below_threshold_counts();
	test_alternating_does_not_shutdown();

	printf("\n%d/%d tests passed (%d failed)\n",
	       tests_passed, tests_run, tests_failed);
	return tests_failed ? 1 : 0;
}
