/**
 * @file    test_wrap_overflow.c
 * @brief   Long-deployment overflow protection: HAL tick wrap (49.7 days)
 *          must not break the rate limiter clock nor the SWS surface lockout.
 *
 * Mirrors:
 *   - mgr_rate.c monotonic-seconds clock (MGR_RATE_nowS) + trim_window,
 *     including the soft-reset resume from ring.mono_base_s.
 *   - mgr_sws.c surface_lockout_active() (start + duration, delta compare).
 */

#include "test_framework.h"

/*******************************************************************************
 * FAKE HAL TICK
 ******************************************************************************/

static uint32_t fake_tick;
static uint32_t HAL_GetTick(void) { return fake_tick; }

/*******************************************************************************
 * MIRROR: mgr_rate.c monotonic clock + window trim
 ******************************************************************************/

#define RATE_CAP 100u

static struct {
	uint32_t timestamps[RATE_CAP];
	uint16_t head;
	uint16_t count;
	uint32_t mono_base_s;
} ring;

static uint32_t mono_s, mono_last_ms, mono_rem_ms;

static uint32_t rate_nowS(void)
{
	const uint32_t now_ms = HAL_GetTick();
	const uint32_t delta  = now_ms - mono_last_ms;

	mono_last_ms = now_ms;
	mono_rem_ms += delta % 1000u;
	mono_s      += delta / 1000u + mono_rem_ms / 1000u;
	mono_rem_ms %= 1000u;
	return mono_s;
}

static void rate_commit(void) { ring.mono_base_s = mono_s; }

static void rate_init_cold(void)
{
	memset(&ring, 0, sizeof(ring));
	mono_s = 0; mono_rem_ms = 0;
	mono_last_ms = HAL_GetTick();
	rate_commit();
}

/* Soft reset: ring survives (retention), runtime clock resumes from base. */
static void rate_init_warm(void)
{
	mono_last_ms = HAL_GetTick();
	mono_rem_ms = 0;
	mono_s = ring.mono_base_s;
}

static void rate_trim(uint32_t now_s, uint32_t window_s)
{
	while (ring.count > 0) {
		uint16_t oldest_idx = (uint16_t)((ring.head + RATE_CAP -
			ring.count) % RATE_CAP);
		uint32_t oldest = ring.timestamps[oldest_idx];
		if (now_s >= oldest && (now_s - oldest) >= window_s)
			ring.count--;
		else
			break;
	}
}

static void rate_record(uint32_t window_s)
{
	uint32_t now_s = rate_nowS();
	rate_trim(now_s, window_s);
	ring.timestamps[ring.head] = now_s;
	ring.head = (uint16_t)((ring.head + 1u) % RATE_CAP);
	if (ring.count < RATE_CAP)
		ring.count++;
	rate_commit();
}

static int rate_blocked(uint32_t window_s, uint16_t max_tx)
{
	uint32_t now_s = rate_nowS();
	rate_trim(now_s, window_s);
	return ring.count >= max_tx;
}

/*******************************************************************************
 * MIRROR: mgr_sws.c surface lockout (start + duration)
 ******************************************************************************/

static uint32_t lockout_start_tick, lockout_ms;

static int lockout_active(void)
{
	if (lockout_ms == 0u)
		return 0;
	if ((uint32_t)(HAL_GetTick() - lockout_start_tick) >= lockout_ms) {
		lockout_ms = 0u;
		return 0;
	}
	return 1;
}

static void lockout_arm(uint32_t dur_ms)
{
	lockout_start_tick = HAL_GetTick();
	lockout_ms = dur_ms;
}

/*******************************************************************************
 * TESTS — rate limiter
 ******************************************************************************/

#define WINDOW_S 86400u
#define MAX_TX   100u

static void test_rate_mono_clock_survives_tick_wrap(void)
{
	fake_tick = 0xFFFFF000u;        /* 4096 ms before the wrap */
	rate_init_cold();
	uint32_t before = rate_nowS();

	fake_tick += 10000u;            /* crosses 0xFFFFFFFF -> 0x00001718 */
	uint32_t after = rate_nowS();

	ASSERT_EQ(10u, after - before); /* exactly 10 s elapsed, no jump */
	TEST_PASS();
}

static void test_rate_quota_releases_across_wrap(void)
{
	fake_tick = 0xFFFFFFFFu - 3600u * 1000u;  /* 1 h before the wrap */
	rate_init_cold();

	/* Fill the quota just before the wrap. */
	for (int i = 0; i < (int)MAX_TX; i++)
		rate_record(WINDOW_S);
	ASSERT_TRUE(rate_blocked(WINDOW_S, MAX_TX));

	/* 12 h later (tick has wrapped): still inside the 24 h window. */
	fake_tick += 12u * 3600u * 1000u;
	ASSERT_TRUE(rate_blocked(WINDOW_S, MAX_TX));

	/* 25 h after the burst: window elapsed, quota must free up —
	 * the old code kept it blocked for ~49 days here. */
	fake_tick += 13u * 3600u * 1000u;
	ASSERT_FALSE(rate_blocked(WINDOW_S, MAX_TX));
	ASSERT_EQ(0, ring.count);
	TEST_PASS();
}

static void test_rate_quota_survives_warm_reset(void)
{
	fake_tick = 50000u;
	rate_init_cold();

	for (int i = 0; i < (int)MAX_TX; i++)
		rate_record(WINDOW_S);
	ASSERT_TRUE(rate_blocked(WINDOW_S, MAX_TX));

	/* Soft reset 1 h later: HAL tick restarts at ~0, ring survives. */
	fake_tick = 1000u;
	rate_init_warm();

	/* Still blocked — entries must NOT look future-dated/stuck. */
	ASSERT_TRUE(rate_blocked(WINDOW_S, MAX_TX));

	/* 25 h of post-reset runtime: window elapsed, quota frees up. */
	fake_tick += 25u * 3600u * 1000u;
	ASSERT_FALSE(rate_blocked(WINDOW_S, MAX_TX));
	TEST_PASS();
}

static void test_rate_mono_accumulates_subsecond_deltas(void)
{
	fake_tick = 0u;
	rate_init_cold();
	/* 1000 calls of 999 ms: naive /1000-per-call would lose 999 s. */
	uint32_t start = rate_nowS();
	for (int i = 0; i < 1000; i++) {
		fake_tick += 999u;
		(void)rate_nowS();
	}
	uint32_t end = rate_nowS();
	ASSERT_EQ(999u, end - start);   /* 999000 ms = 999 s exactly */
	TEST_PASS();
}

/*******************************************************************************
 * TESTS — SWS surface lockout
 ******************************************************************************/

static void test_lockout_blocks_then_expires(void)
{
	fake_tick = 1000000u;
	lockout_arm(10000u);
	ASSERT_TRUE(lockout_active());
	fake_tick += 9999u;
	ASSERT_TRUE(lockout_active());
	fake_tick += 2u;
	ASSERT_FALSE(lockout_active());
	ASSERT_EQ(0, lockout_ms);       /* self-disarms */
	TEST_PASS();
}

static void test_lockout_expires_across_tick_wrap(void)
{
	/* Armed 3 s before the wrap: the old absolute-deadline code read
	 * "until" as a huge value and kept dive detection blocked ~49 days. */
	fake_tick = 0xFFFFFFFFu - 3000u;
	lockout_arm(10000u);
	ASSERT_TRUE(lockout_active());

	fake_tick += 5000u;             /* wrapped: now = 0x7CF */
	ASSERT_TRUE(lockout_active());  /* 5 s in, still locked */

	fake_tick += 6000u;             /* 11 s in: expired */
	ASSERT_FALSE(lockout_active());
	TEST_PASS();
}

static void test_lockout_zero_means_inactive(void)
{
	fake_tick = 123456u;
	lockout_start_tick = 0; lockout_ms = 0;
	ASSERT_FALSE(lockout_active());
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("Long-deployment overflow protection (tick wrap)");

	RUN_TEST(test_rate_mono_clock_survives_tick_wrap);
	RUN_TEST(test_rate_quota_releases_across_wrap);
	RUN_TEST(test_rate_quota_survives_warm_reset);
	RUN_TEST(test_rate_mono_accumulates_subsecond_deltas);
	RUN_TEST(test_lockout_blocks_then_expires);
	RUN_TEST(test_lockout_expires_across_tick_wrap);
	RUN_TEST(test_lockout_zero_means_inactive);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
