/**
 * @file    test_tx_cooldown.c
 * @brief   Regression spec for the first-TX cooldown gate (tx_cooldown_s).
 *
 * Mirrors the surface_tx_pending gate in kns_app_uw_doppler.c: the first TX of
 * a surface event fires only when BOTH the anti-collision random offset AND the
 * cooldown quiet-floor (measured from last_actual_tx_tick) have elapsed.
 * Contract: cooldown 0 = disabled; first-ever TX (last_actual_tx_tick==0)
 * bypasses; cooldown survives dive/surface cycles via the wall-clock tick.
 */

#include "test_framework.h"

static uint32_t fk_tick;
static uint32_t HAL_GetTick(void) { return fk_tick; }

/* Mirror of the two-gate first-TX decision. Returns 1 if the first TX fires. */
static int first_tx_fires(uint32_t since_last_tx, uint32_t random_offset_ms,
                          uint16_t cooldown_s, uint32_t last_actual_tx_tick)
{
	const uint32_t cooldown_ms = (uint32_t)cooldown_s * 1000u;
	int cooldown_ok = (cooldown_ms == 0u) ||
		(last_actual_tx_tick == 0u) ||
		((HAL_GetTick() - last_actual_tx_tick) >= cooldown_ms);
	return (since_last_tx >= random_offset_ms) && cooldown_ok;
}

static void test_first_ever_tx_bypasses_cooldown(void)
{
	fk_tick = 5000;
	/* last_actual_tx_tick == 0 → no prior TX → cooldown bypassed */
	ASSERT_TRUE(first_tx_fires(0xFFFFFFFFu, 0, 10, 0));
	TEST_PASS();
}

static void test_cooldown_zero_disabled(void)
{
	fk_tick = 1000;
	/* cooldown 0 → fires immediately even right after a TX */
	ASSERT_TRUE(first_tx_fires(0xFFFFFFFFu, 0, 0, 990));
	TEST_PASS();
}

static void test_cooldown_blocks_quick_resurface(void)
{
	fk_tick = 100000;
	/* last TX 3s ago, cooldown 10s → blocked */
	ASSERT_FALSE(first_tx_fires(0xFFFFFFFFu, 0, 10, 100000 - 3000));
	/* 11s ago → allowed */
	ASSERT_TRUE(first_tx_fires(0xFFFFFFFFu, 0, 10, 100000 - 11000));
	TEST_PASS();
}

static void test_cooldown_boundary(void)
{
	fk_tick = 50000;
	/* exactly 10s ago → allowed (>=) */
	ASSERT_TRUE(first_tx_fires(0xFFFFFFFFu, 0, 10, 50000 - 10000));
	/* 1ms short → blocked */
	ASSERT_FALSE(first_tx_fires(0xFFFFFFFFu, 0, 10, 50000 - 9999));
	TEST_PASS();
}

static void test_random_offset_still_gates(void)
{
	fk_tick = 100000;
	/* cooldown satisfied, but random offset not yet elapsed → no TX */
	ASSERT_FALSE(first_tx_fires(300, 500, 10, 100000 - 20000));
	/* offset elapsed too → TX */
	ASSERT_TRUE(first_tx_fires(600, 500, 10, 100000 - 20000));
	TEST_PASS();
}

static void test_cooldown_survives_across_wrap(void)
{
	/* tick wraps between the TX and the resurface check; unsigned delta
	 * stays correct. last TX 12s before wrap, now 2s after wrap = 14s. */
	fk_tick = 2000;                       /* 2s past wrap */
	uint32_t last = 0xFFFFFFFFu - 12000u + 1u;  /* 12s before wrap */
	ASSERT_TRUE(first_tx_fires(0xFFFFFFFFu, 0, 10, last));   /* 14s >= 10s */
	ASSERT_FALSE(first_tx_fires(0xFFFFFFFFu, 0, 20, last));  /* 14s < 20s */
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("UW_DOPPLER first-TX cooldown gate");

	RUN_TEST(test_first_ever_tx_bypasses_cooldown);
	RUN_TEST(test_cooldown_zero_disabled);
	RUN_TEST(test_cooldown_blocks_quick_resurface);
	RUN_TEST(test_cooldown_boundary);
	RUN_TEST(test_random_offset_still_gates);
	RUN_TEST(test_cooldown_survives_across_wrap);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
