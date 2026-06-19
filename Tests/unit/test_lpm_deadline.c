/**
 * @file    test_lpm_deadline.c
 * @brief   Unit tests for the deadline-based LPM scheduler.
 *
 * Mirrors:
 *  - uw_ms_until_next_action() (kns_app_uw_doppler.c): min of next SWS
 *    sample / next TX of the sequence / seq-restart deadline.
 *  - MGR_SWS_msUntilNextSample() (mgr_sws.c).
 *  - MGR_LPM_UW_enterStop2TimedMs() RTC arming math (mgr_lpm_uw.c):
 *    RTCCLK/16 ticks below 29 s (never oversleeps), floored CK_SPRE
 *    seconds above.
 */

#include "test_framework.h"

/*******************************************************************************
 * MOCKED STATE
 ******************************************************************************/

static uint32_t hal_tick;
static uint32_t HAL_GetTick(void) { return hal_tick; }

/* SWS side */
static int      sws_enabled;
static uint32_t sws_last_measurement_tick;
static uint32_t sws_interval_ms;

static uint32_t MGR_SWS_msUntilNextSample(void)
{
	if (!sws_enabled)
		return 0xFFFFFFFFu;
	if (sws_last_measurement_tick == 0)
		return 0;
	uint32_t elapsed = HAL_GetTick() - sws_last_measurement_tick;
	return (elapsed >= sws_interval_ms) ? 0u : (sws_interval_ms - elapsed);
}

/* App side */
static uint32_t tx_count;
static uint32_t last_tx_tick;
static uint32_t current_interval_ms;
static uint8_t  tx_max_count;
static uint16_t tx_seq_restart_s;

static uint32_t uw_ms_until_next_action(void)
{
	uint32_t delta = MGR_SWS_msUntilNextSample();
	const uint8_t cap = tx_max_count;
	const uint32_t since = HAL_GetTick() - last_tx_tick;

	if (tx_count > 0 && current_interval_ms > 0 &&
	    (cap == 0u || tx_count < cap)) {
		uint32_t r = (since >= current_interval_ms)
			? 0u : (current_interval_ms - since);
		if (r < delta)
			delta = r;
	}
	if (tx_seq_restart_s > 0u && cap > 0u && tx_count >= cap &&
	    last_tx_tick > 0u) {
		const uint32_t rst_ms = (uint32_t)tx_seq_restart_s * 1000u;
		uint32_t r = (since >= rst_ms) ? 0u : (rst_ms - since);
		if (r < delta)
			delta = r;
	}
	return delta;
}

/* STOP2 arming math (enterStop2TimedMs) */
static uint32_t stop2_wut_counter;   /* value loaded into the WUT */
static int      stop2_used_div16;    /* 1 = RTCCLK/16, 0 = CK_SPRE */

static void stop2_arm(uint32_t ms)
{
	if (ms > 65535000u) ms = 65535000u;
	if (ms < 29000u) {
		uint32_t ticks = (ms * 2048u) / 1000u;
		if (ticks == 0u) ticks = 1u;
		stop2_wut_counter = ticks - 1u;
		stop2_used_div16 = 1;
	} else {
		uint32_t seconds = ms / 1000u;
		stop2_wut_counter = seconds - 1u;
		stop2_used_div16 = 0;
	}
}

static void reset_state(void)
{
	hal_tick = 100000;
	sws_enabled = 1;
	sws_last_measurement_tick = 99000;   /* sampled 1s ago */
	sws_interval_ms = 5000;              /* surface cadence */
	tx_count = 0;
	last_tx_tick = 0;
	current_interval_ms = 0;
	tx_max_count = 3;
	tx_seq_restart_s = 45;
}

/*******************************************************************************
 * TEST CASES — deadline aggregation
 ******************************************************************************/

/** Idle, no sequence: delta = time to next SWS sample. */
void test_idle_uses_sws_deadline(void)
{
	reset_state();
	ASSERT_EQ(4000, uw_ms_until_next_action());
	TEST_PASS();
}

/** Mid-sequence TX sooner than SWS sample: TX deadline wins. */
void test_tx_deadline_wins(void)
{
	reset_state();
	tx_count = 1;
	last_tx_tick = 98000;            /* TX was 2s ago */
	current_interval_ms = 4500;      /* next TX in 2.5s < SWS 4s */
	ASSERT_EQ(2500, uw_ms_until_next_action());
	TEST_PASS();
}

/** Mid-sequence TX later than SWS sample: SWS wins (dive detection). */
void test_sws_deadline_wins(void)
{
	reset_state();
	tx_count = 1;
	last_tx_tick = 98000;
	current_interval_ms = 10000;     /* next TX in 8s > SWS 4s */
	ASSERT_EQ(4000, uw_ms_until_next_action());
	TEST_PASS();
}

/** TX overdue: returns 0 (action due now, caller must not sleep). */
void test_tx_overdue_returns_zero(void)
{
	reset_state();
	tx_count = 1;
	last_tx_tick = 80000;
	current_interval_ms = 10000;     /* due 10s ago */
	ASSERT_EQ(0, uw_ms_until_next_action());
	TEST_PASS();
}

/** Cap reached: TX deadline drops out, seq-restart deadline takes over. */
void test_cap_reached_uses_restart_deadline(void)
{
	reset_state();
	tx_count = 3;                    /* == cap */
	last_tx_tick = 60000;            /* 40s ago; restart at 45s -> 5s left */
	current_interval_ms = 10000;
	sws_last_measurement_tick = hal_tick;  /* SWS just sampled: 5s away */
	/* restart in 5000, SWS in 5000 -> 5000 */
	ASSERT_EQ(5000, uw_ms_until_next_action());
	hal_tick += 2000;                /* restart in 3000 < SWS refresh */
	sws_last_measurement_tick = hal_tick;
	ASSERT_EQ(3000, uw_ms_until_next_action());
	TEST_PASS();
}

/** Unlimited cap (0): TX deadline applies, restart timer never. */
void test_unlimited_cap(void)
{
	reset_state();
	tx_max_count = 0;
	tx_count = 50;
	last_tx_tick = 99500;            /* 0.5s ago */
	current_interval_ms = 2000;      /* next TX in 1.5s */
	ASSERT_EQ(1500, uw_ms_until_next_action());
	TEST_PASS();
}

/** SWS disabled: no SWS deadline; falls back to TX deadline. */
void test_sws_disabled_infinite(void)
{
	reset_state();
	sws_enabled = 0;
	ASSERT_EQ(0xFFFFFFFFu, uw_ms_until_next_action());
	tx_count = 1;
	last_tx_tick = 99000;
	current_interval_ms = 10000;
	ASSERT_EQ(9000, uw_ms_until_next_action());
	TEST_PASS();
}

/** SWS never sampled yet: sample due now. */
void test_sws_never_sampled_due_now(void)
{
	reset_state();
	sws_last_measurement_tick = 0;
	ASSERT_EQ(0, uw_ms_until_next_action());
	TEST_PASS();
}

/*******************************************************************************
 * TEST CASES — STOP2 arming math
 ******************************************************************************/

/** Sub-29s uses RTCCLK/16 with ~0.49ms resolution. */
void test_stop2_div16_exact(void)
{
	stop2_arm(1000);                 /* 1s -> 2048 ticks */
	ASSERT_EQ(1, stop2_used_div16);
	ASSERT_EQ(2047, stop2_wut_counter);

	stop2_arm(2500);                 /* 2.5s -> 5120 ticks */
	ASSERT_EQ(5119, stop2_wut_counter);
	TEST_PASS();
}

/** Smallest request never computes a zero-tick (HW would never fire). */
void test_stop2_min_one_tick(void)
{
	stop2_arm(1);                    /* 2 ticks: (1*2048)/1000 = 2 */
	ASSERT_EQ(1, stop2_used_div16);
	ASSERT_EQ(1, stop2_wut_counter); /* 2 ticks - 1 */
	TEST_PASS();
}

/** 28999ms stays in DIV16 and fits the 16-bit counter. */
void test_stop2_div16_upper_bound(void)
{
	stop2_arm(28999);                /* 59389 ticks <= 0xFFFF */
	ASSERT_EQ(1, stop2_used_div16);
	ASSERT_TRUE(stop2_wut_counter <= 0xFFFFu);
	ASSERT_EQ(59389 - 1, stop2_wut_counter);
	TEST_PASS();
}

/** >=29s uses CK_SPRE with FLOOR — never oversleeps the deadline. */
void test_stop2_spre_floor(void)
{
	stop2_arm(45000);
	ASSERT_EQ(0, stop2_used_div16);
	ASSERT_EQ(44, stop2_wut_counter);  /* 45s - 1 */

	stop2_arm(44400);                  /* floor -> 44s, remainder next pass */
	ASSERT_EQ(43, stop2_wut_counter);
	TEST_PASS();
}

/** Clamp at the 16-bit CK_SPRE ceiling. */
void test_stop2_clamp(void)
{
	stop2_arm(0xFFFFFFFFu);
	ASSERT_EQ(0, stop2_used_div16);
	ASSERT_EQ(65534, stop2_wut_counter); /* 65535s - 1 */
	TEST_PASS();
}

/*******************************************************************************
 * MAIN
 ******************************************************************************/

int main(void)
{
	TEST_SUITE_START("Deadline-based LPM scheduler");

	RUN_TEST(test_idle_uses_sws_deadline);
	RUN_TEST(test_tx_deadline_wins);
	RUN_TEST(test_sws_deadline_wins);
	RUN_TEST(test_tx_overdue_returns_zero);
	RUN_TEST(test_cap_reached_uses_restart_deadline);
	RUN_TEST(test_unlimited_cap);
	RUN_TEST(test_sws_disabled_infinite);
	RUN_TEST(test_sws_never_sampled_due_now);
	RUN_TEST(test_stop2_div16_exact);
	RUN_TEST(test_stop2_min_one_tick);
	RUN_TEST(test_stop2_div16_upper_bound);
	RUN_TEST(test_stop2_spre_floor);
	RUN_TEST(test_stop2_clamp);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
