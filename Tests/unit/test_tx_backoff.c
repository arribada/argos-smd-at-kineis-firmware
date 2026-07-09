/**
 * @file    test_tx_backoff.c
 * @brief   Regression spec for the UW_DOPPLER device-error TX backoff, after
 *          the 2026-07-08 "recovery leniency" change (①).
 *
 * The production logic lives in kns_app_uw_doppler.c (tx_backoff_arm /
 * tx_backoff_blocked / tx_backoff_reset) and compiles against the STM32 HAL +
 * MGR_EVTLOG/MGR_LOG, so — per this repo's per-file standalone runner — it
 * cannot be linked directly. These tests MIRROR the exact arithmetic and lock
 * the new contract:
 *   - The 1st CONSECUTIVE TX error is FREE (no silence) so a single transient
 *     failure never swallows a whole brief surface window.
 *   - Backoff engages from the 2nd error: 60 s, doubling, capped at 3 min.
 *   - One TX_DONE (reset) clears the streak so "first error free" re-applies.
 *   - blocked() is wrap-safe across the 49.7-day HAL_GetTick rollover.
 */

#include "test_framework.h"

/* ---- Mirror of the backoff constants (kns_app_uw_doppler.c) ---- */
#define TX_BACKOFF_BASE_MS       60000u
#define TX_BACKOFF_MAX_MS       180000u
#define TX_BACKOFF_MAX_SHIFT         4

/* ---- Mirror of the backoff state + a mockable tick ---- */
static uint8_t  consecutive_tx_errors;
static uint32_t tx_backoff_until_tick;
static uint32_t mock_tick;

static uint32_t HAL_GetTick(void) { return mock_tick; }

/* Mirror of tx_backoff_arm() with the ① first-error-free change. */
static void tx_backoff_arm(void)
{
	if (consecutive_tx_errors < 2) {
		tx_backoff_until_tick = 0;      /* no silence on the first error */
		return;
	}
	uint8_t shift = (uint8_t)(consecutive_tx_errors - 2); /* 2nd err -> shift 0 */
	if (shift > TX_BACKOFF_MAX_SHIFT)
		shift = TX_BACKOFF_MAX_SHIFT;
	uint32_t backoff_ms = TX_BACKOFF_BASE_MS << shift;
	if (backoff_ms > TX_BACKOFF_MAX_MS)
		backoff_ms = TX_BACKOFF_MAX_MS;
	tx_backoff_until_tick = HAL_GetTick() + backoff_ms;
}

static void tx_backoff_reset(void)
{
	consecutive_tx_errors = 0;
	tx_backoff_until_tick = 0;
}

static int tx_backoff_blocked(uint32_t *retry_in_s)
{
	if (tx_backoff_until_tick == 0)
		return 0;
	uint32_t now = HAL_GetTick();
	if ((int32_t)(tx_backoff_until_tick - now) <= 0) {
		tx_backoff_until_tick = 0;
		return 0;
	}
	if (retry_in_s)
		*retry_in_s = (tx_backoff_until_tick - now) / 1000u;
	return 1;
}

/* Mirror of the caller: consecutive_tx_errors is incremented BEFORE arm. */
static void tx_error(void)
{
	if (consecutive_tx_errors < 0xFFu)
		consecutive_tx_errors++;
	tx_backoff_arm();
}

static void reset_state(void)
{
	consecutive_tx_errors = 0;
	tx_backoff_until_tick = 0;
	mock_tick = 0;
}

/* ---- Tests ---- */

/** The very first error must NOT arm any backoff (free retry). */
void test_first_error_is_free(void)
{
	reset_state();
	tx_error();                                 /* 1st error */
	ASSERT_EQ(0, (long)tx_backoff_until_tick);
	ASSERT_FALSE(tx_backoff_blocked(NULL));
	TEST_PASS();
}

/** The second consecutive error arms the 60 s base backoff. */
void test_second_error_60s(void)
{
	reset_state();
	tx_error(); tx_error();                     /* 1st free, 2nd arms */
	uint32_t retry = 0;
	ASSERT_TRUE(tx_backoff_blocked(&retry));
	ASSERT_EQ(60, (long)retry);
	TEST_PASS();
}

/** The third error doubles to 120 s. */
void test_third_error_120s(void)
{
	reset_state();
	tx_error(); tx_error(); tx_error();
	uint32_t retry = 0;
	ASSERT_TRUE(tx_backoff_blocked(&retry));
	ASSERT_EQ(120, (long)retry);
	TEST_PASS();
}

/** The fourth error would be 240 s but is clamped to the 180 s (3 min) cap. */
void test_fourth_error_capped_180s(void)
{
	reset_state();
	tx_error(); tx_error(); tx_error(); tx_error();
	uint32_t retry = 0;
	ASSERT_TRUE(tx_backoff_blocked(&retry));
	ASSERT_EQ(180, (long)retry);
	TEST_PASS();
}

/** Many consecutive errors never exceed the 180 s cap. */
void test_cap_never_exceeded(void)
{
	reset_state();
	for (int i = 0; i < 12; i++)
		tx_error();
	uint32_t retry = 0;
	ASSERT_TRUE(tx_backoff_blocked(&retry));
	ASSERT_EQ(180, (long)retry);
	TEST_PASS();
}

/** A single TX_DONE (reset) between errors restores "first error is free". */
void test_success_between_errors_reclaims_free_first(void)
{
	reset_state();
	tx_error();                                 /* 1st error (free) */
	tx_backoff_reset();                         /* TX_DONE clears the streak */
	tx_error();                                 /* 1st again -> must be free */
	ASSERT_EQ(0, (long)tx_backoff_until_tick);
	ASSERT_FALSE(tx_backoff_blocked(NULL));
	TEST_PASS();
}

/** reset() clears an armed backoff and the counter. */
void test_reset_clears_armed_backoff(void)
{
	reset_state();
	tx_error(); tx_error();                     /* armed 60 s */
	ASSERT_TRUE(tx_backoff_blocked(NULL));
	tx_backoff_reset();
	ASSERT_EQ(0, (long)consecutive_tx_errors);
	ASSERT_FALSE(tx_backoff_blocked(NULL));
	TEST_PASS();
}

/** Backoff self-expires once the deadline tick is reached. */
void test_backoff_expires(void)
{
	reset_state();
	tx_error(); tx_error();                     /* armed until 60000 */
	mock_tick = 30000u;
	uint32_t retry = 0;
	ASSERT_TRUE(tx_backoff_blocked(&retry));
	ASSERT_EQ(30, (long)retry);                 /* 30 s left */
	mock_tick = 60000u;                          /* deadline reached */
	ASSERT_FALSE(tx_backoff_blocked(NULL));
	TEST_PASS();
}

/** blocked() stays correct across the uint32 HAL_GetTick wrap. */
void test_backoff_blocked_wrapsafe(void)
{
	reset_state();
	mock_tick = 0xFFFFFF00u;                     /* near the 49.7-day wrap */
	tx_error(); tx_error();                      /* until = 0xFFFFFF00 + 60000 (wraps) */
	uint32_t retry = 0;
	ASSERT_TRUE(tx_backoff_blocked(&retry));     /* still blocked despite wrap */
	ASSERT_EQ(60, (long)retry);
	mock_tick = 0x0000E960u;                      /* wrapped deadline */
	ASSERT_FALSE(tx_backoff_blocked(NULL));       /* expired */
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("UW_DOPPLER TX backoff (first-error-free, 3 min cap)");

	RUN_TEST(test_first_error_is_free);
	RUN_TEST(test_second_error_60s);
	RUN_TEST(test_third_error_120s);
	RUN_TEST(test_fourth_error_capped_180s);
	RUN_TEST(test_cap_never_exceeded);
	RUN_TEST(test_success_between_errors_reclaims_free_first);
	RUN_TEST(test_reset_clears_armed_backoff);
	RUN_TEST(test_backoff_expires);
	RUN_TEST(test_backoff_blocked_wrapsafe);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
