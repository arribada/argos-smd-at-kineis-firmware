/**
 * @file    test_lpm_uw.c
 * @brief   Unit tests for MGR_LPM_UW logic (UW_DOPPLER duty-cycle policy).
 *
 * The real module depends on STM32 HAL + retention sections we can't link
 * here, so we mirror its decision functions and verify the algorithmic
 * core: SWS-state transition detection, threshold-based STANDBY vs
 * SHUTDOWN selection, and config defaults.
 *
 * Bugs guarded:
 *
 *  1. SWS enum mismatch. MGR_SWS_State_t is UNKNOWN=0/SURFACE=1/UNDERWATER=2,
 *     not UNDERWATER=0/SURFACE=1. An earlier refactor encoded the wrong
 *     mapping and would have made the duty cycle pick the WRONG sleep
 *     duration on every wake (surface_s while underwater = far too short).
 *
 *  2. UNKNOWN sample on cold-boot overwriting the persisted UW/SURF
 *     baseline. The SWS task isn't immediately ready post-cold-boot;
 *     the comparison must skip UNKNOWN samples or it loses the prev-cycle
 *     state and never fires the cold-boot UW->SURF instant TX.
 *
 *  3. shutdown_threshold_s = 0 must disable the SHUTDOWN selection
 *     (always STANDBY). A non-zero threshold ≥ short-sleep cutoff picks
 *     SHUTDOWN+RTC for sleep_s ≥ threshold.
 */

#include "test_framework.h"

/* Mirror of MGR_SWS_State_t (mgr_sws.h). */
enum sws_state_t {
	SWS_UNKNOWN    = 0,
	SWS_SURFACE    = 1,
	SWS_UNDERWATER = 2,
};

/* Mirror of UwLpmDutyCfg_t — only the fields exercised here. */
typedef struct {
	uint16_t uw_sleep_s;
	uint16_t surf_sleep_s;
	uint8_t  enabled;
	uint8_t  last_sws_state;
	uint8_t  wake_should_tx;
	uint16_t shutdown_threshold_s;
} cfg_t;

#define SHORT_SLEEP_THRESHOLD_S  5u

/* ---- Mirrors of the production decision functions ---- */

static bool detect_surface_wake(cfg_t *c, int current)
{
	const uint8_t prev = c->last_sws_state;
	const uint8_t curr = (uint8_t)current;
	bool fire = false;

	if (prev == SWS_UNDERWATER && curr == SWS_SURFACE) {
		fire = true;
		c->wake_should_tx = 1u;
	}
	if (curr == SWS_SURFACE || curr == SWS_UNDERWATER) {
		c->last_sws_state = curr;
	}
	return fire;
}

/* Returns: 0=stay awake, 1=STANDBY, 2=SHUTDOWN. */
static int auto_cycle_decision(const cfg_t *c, int sws_state)
{
	if (!c->enabled) return 0;

	uint32_t sleep_s = (sws_state == SWS_UNDERWATER)
	                   ? c->uw_sleep_s
	                   : c->surf_sleep_s;
	if (sleep_s < SHORT_SLEEP_THRESHOLD_S) return 0;

	if (c->shutdown_threshold_s >= SHORT_SLEEP_THRESHOLD_S &&
	    sleep_s >= c->shutdown_threshold_s) {
		return 2;
	}
	return 1;
}

/* ---- Tests ---- */

void test_detect_uw_to_surf_fires(void)
{
	TEST_START("detect: UW(2) -> SURF(1) sets wake_should_tx");
	cfg_t c = { .last_sws_state = SWS_UNDERWATER, .wake_should_tx = 0 };
	ASSERT_TRUE(detect_surface_wake(&c, SWS_SURFACE));
	ASSERT_EQ(1, c.wake_should_tx);
	ASSERT_EQ(SWS_SURFACE, c.last_sws_state);
	TEST_PASS();
}

void test_detect_surf_to_surf_no_fire(void)
{
	TEST_START("detect: SURF -> SURF no fire");
	cfg_t c = { .last_sws_state = SWS_SURFACE, .wake_should_tx = 0 };
	ASSERT_FALSE(detect_surface_wake(&c, SWS_SURFACE));
	ASSERT_EQ(0, c.wake_should_tx);
	TEST_PASS();
}

void test_detect_uw_to_uw_no_fire(void)
{
	TEST_START("detect: UW -> UW no fire");
	cfg_t c = { .last_sws_state = SWS_UNDERWATER, .wake_should_tx = 0 };
	ASSERT_FALSE(detect_surface_wake(&c, SWS_UNDERWATER));
	ASSERT_EQ(0, c.wake_should_tx);
	TEST_PASS();
}

void test_detect_surf_to_uw_no_fire(void)
{
	TEST_START("detect: SURF -> UW no fire (only UW->SURF triggers)");
	cfg_t c = { .last_sws_state = SWS_SURFACE, .wake_should_tx = 0 };
	ASSERT_FALSE(detect_surface_wake(&c, SWS_UNDERWATER));
	ASSERT_EQ(0, c.wake_should_tx);
	TEST_PASS();
}

void test_detect_unknown_preserves_state(void)
{
	TEST_START("detect: UNKNOWN sample doesn't overwrite persisted state");
	cfg_t c = { .last_sws_state = SWS_UNDERWATER, .wake_should_tx = 0 };
	ASSERT_FALSE(detect_surface_wake(&c, SWS_UNKNOWN));
	ASSERT_EQ(SWS_UNDERWATER, c.last_sws_state);
	/* Subsequent SURFACE sample must STILL detect UW->SURF transition */
	ASSERT_TRUE(detect_surface_wake(&c, SWS_SURFACE));
	TEST_PASS();
}

void test_detect_first_boot_no_fire(void)
{
	TEST_START("detect: first-ever boot (last=0xFF) no false fire");
	cfg_t c = { .last_sws_state = 0xFFu, .wake_should_tx = 0 };
	ASSERT_FALSE(detect_surface_wake(&c, SWS_SURFACE));
	ASSERT_EQ(0, c.wake_should_tx);
	ASSERT_EQ(SWS_SURFACE, c.last_sws_state);
	TEST_PASS();
}

void test_cycle_disabled_stays_awake(void)
{
	TEST_START("auto-cycle: enabled=0 stays awake");
	cfg_t c = { .enabled = 0, .uw_sleep_s = 1800, .surf_sleep_s = 60,
	            .shutdown_threshold_s = 300 };
	ASSERT_EQ(0, auto_cycle_decision(&c, SWS_UNDERWATER));
	TEST_PASS();
}

void test_cycle_short_sleep_skipped(void)
{
	TEST_START("auto-cycle: sleep_s < 5s stays awake (cold-boot overhead)");
	cfg_t c = { .enabled = 1, .uw_sleep_s = 2, .surf_sleep_s = 60,
	            .shutdown_threshold_s = 300 };
	ASSERT_EQ(0, auto_cycle_decision(&c, SWS_UNDERWATER));
	TEST_PASS();
}

void test_cycle_uw_long_picks_shutdown(void)
{
	TEST_START("auto-cycle: UW with 30min sleep picks SHUTDOWN (≥thr)");
	cfg_t c = { .enabled = 1, .uw_sleep_s = 1800, .surf_sleep_s = 60,
	            .shutdown_threshold_s = 300 };
	ASSERT_EQ(2, auto_cycle_decision(&c, SWS_UNDERWATER));
	TEST_PASS();
}

void test_cycle_surf_short_picks_standby(void)
{
	TEST_START("auto-cycle: SURF with 60s sleep picks STANDBY (<thr)");
	cfg_t c = { .enabled = 1, .uw_sleep_s = 1800, .surf_sleep_s = 60,
	            .shutdown_threshold_s = 300 };
	ASSERT_EQ(1, auto_cycle_decision(&c, SWS_SURFACE));
	TEST_PASS();
}

void test_cycle_threshold_zero_always_standby(void)
{
	TEST_START("auto-cycle: threshold=0 always STANDBY (never SHUTDOWN)");
	cfg_t c = { .enabled = 1, .uw_sleep_s = 1800, .surf_sleep_s = 60,
	            .shutdown_threshold_s = 0 };
	ASSERT_EQ(1, auto_cycle_decision(&c, SWS_UNDERWATER));
	TEST_PASS();
}

void test_cycle_threshold_picks_at_boundary(void)
{
	TEST_START("auto-cycle: sleep == threshold picks SHUTDOWN");
	cfg_t c = { .enabled = 1, .uw_sleep_s = 300, .surf_sleep_s = 60,
	            .shutdown_threshold_s = 300 };
	ASSERT_EQ(2, auto_cycle_decision(&c, SWS_UNDERWATER));
	TEST_PASS();
}

void test_cycle_unknown_sample_treated_as_surface(void)
{
	TEST_START("auto-cycle: UNKNOWN sample uses surf_sleep_s (safe default)");
	cfg_t c = { .enabled = 1, .uw_sleep_s = 1800, .surf_sleep_s = 60,
	            .shutdown_threshold_s = 300 };
	/* sleep_s = surf_sleep_s = 60 < 300 -> STANDBY */
	ASSERT_EQ(1, auto_cycle_decision(&c, SWS_UNKNOWN));
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("MGR_LPM_UW auto-cycle + detectSurfaceWake");
	test_detect_uw_to_surf_fires();
	test_detect_surf_to_surf_no_fire();
	test_detect_uw_to_uw_no_fire();
	test_detect_surf_to_uw_no_fire();
	test_detect_unknown_preserves_state();
	test_detect_first_boot_no_fire();
	test_cycle_disabled_stays_awake();
	test_cycle_short_sleep_skipped();
	test_cycle_uw_long_picks_shutdown();
	test_cycle_surf_short_picks_standby();
	test_cycle_threshold_zero_always_standby();
	test_cycle_threshold_picks_at_boundary();
	test_cycle_unknown_sample_treated_as_surface();
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
