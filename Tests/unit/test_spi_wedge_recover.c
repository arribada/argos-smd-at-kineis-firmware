/**
 * @file    test_spi_wedge_recover.c
 * @brief   Unit tests for the SPI slave wedge-recovery watchdog (SPICMD_IDLE)
 *
 * Mirrors the watchdog added to mgr_spi_cmd.c: while the slave waits in IDLE,
 * if no clean transaction completes for SPI_WEDGE_RECOVER_MS AND error_count
 * keeps climbing during that dry spell (host clocking into a stuck RX after an
 * OVR desync), force a driver reset. Gated on errors so a genuinely idle bus —
 * and the normal happy path, where every completed transaction clears the
 * timer — is NEVER reset. Keep SPI_WEDGE_RECOVER_MS in sync with mgr_spi_cmd.c.
 */

#include "test_framework.h"

#define SPI_WEDGE_RECOVER_MS 750u

/* Mirror of the two file-statics in mgr_spi_cmd.c. */
static uint32_t watch_tick;
static uint32_t err_snapshot;

static void wd_reset(void)
{
	watch_tick = 0;
	err_snapshot = 0;
}

/* Mirror of one SPICMD_IDLE iteration. Returns true if the watchdog would force
 * SPICMD_ERROR (a driver reset) this iteration. */
static bool wd_step(uint32_t now, uint32_t err_count, bool transaction_completed)
{
	if (transaction_completed) {
		watch_tick = 0;                 /* clean transaction -> end dry spell */
		return false;
	}
	if (watch_tick == 0) {
		watch_tick = now;
		err_snapshot = err_count;
		return false;
	}
	if ((now - watch_tick) > SPI_WEDGE_RECOVER_MS && err_count != err_snapshot) {
		watch_tick = 0;                 /* trigger -> reset, fresh measurement */
		return true;
	}
	return false;
}

static void test_idle_bus_never_resets(void)
{
	/* No host activity: error_count never moves. The slave must wait forever
	 * without a reset, no matter how long. */
	wd_reset();
	ASSERT_FALSE(wd_step(1000u, 7u, false));     /* dry spell starts, snapshot=7 */
	ASSERT_FALSE(wd_step(2000u, 7u, false));     /* +1s, errors static */
	ASSERT_FALSE(wd_step(100000u, 7u, false));   /* +99s, still static */
	TEST_PASS();
}

static void test_wedge_triggers(void)
{
	/* Host clocking into a stuck RX: errors climb, no transaction completes. */
	wd_reset();
	ASSERT_FALSE(wd_step(1000u, 10u, false));               /* snapshot=10 */
	ASSERT_FALSE(wd_step(1000u + 700u, 14u, false));        /* errs climb, <750ms */
	ASSERT_TRUE(wd_step(1000u + 751u, 18u, false));         /* >750ms + errs moved */
	TEST_PASS();
}

static void test_within_window_no_trigger(void)
{
	/* Errors climbing but the dry spell is still inside the window. */
	wd_reset();
	ASSERT_FALSE(wd_step(5000u, 0u, false));                /* snapshot=0 */
	ASSERT_FALSE(wd_step(5000u + SPI_WEDGE_RECOVER_MS, 9u, false)); /* == window edge */
	TEST_PASS();
}

static void test_clean_transaction_clears_timer(void)
{
	/* A completed transaction mid-dry-spell resets the timer, so a later short
	 * wait cannot trigger. */
	wd_reset();
	ASSERT_FALSE(wd_step(2000u, 3u, false));                /* dry spell building */
	ASSERT_FALSE(wd_step(2500u, 5u, true));                 /* transaction OK -> clear */
	ASSERT_FALSE(wd_step(2600u, 6u, false));               /* new spell, snapshot=6 */
	ASSERT_FALSE(wd_step(3000u, 9u, false));               /* only 400ms in, no trigger */
	TEST_PASS();
}

static void test_single_glitch_then_quiet_no_trigger(void)
{
	/* One CS-release-early OVR is counted BEFORE the dry spell snapshot, then
	 * the bus goes quiet (no new errors). Must NOT reset — this is the normal
	 * occasional-glitch case, not a wedge. */
	wd_reset();
	ASSERT_FALSE(wd_step(1000u, 42u, false));               /* snapshot=42 */
	ASSERT_FALSE(wd_step(1000u + 760u, 42u, false));        /* past window, errs static */
	ASSERT_FALSE(wd_step(1000u + 5000u, 42u, false));       /* still static */
	TEST_PASS();
}

static void test_fresh_after_trigger(void)
{
	/* After a trigger (reset), the timer is fresh and a new wedge must take a
	 * full window again. */
	wd_reset();
	ASSERT_FALSE(wd_step(1000u, 1u, false));
	ASSERT_TRUE(wd_step(1000u + 800u, 5u, false));          /* trigger, watch_tick=0 */
	ASSERT_FALSE(wd_step(2000u, 5u, false));                /* new spell, snapshot=5 */
	ASSERT_FALSE(wd_step(2000u + 100u, 6u, false));         /* only 100ms, no trigger */
	ASSERT_TRUE(wd_step(2000u + 760u, 7u, false));          /* full new window -> trigger */
	TEST_PASS();
}

static void test_wrap_safe(void)
{
	/* Dry spell that straddles the 32-bit tick wrap still measures correctly. */
	wd_reset();
	uint32_t base = 0xFFFFFFFFu - 100u;
	ASSERT_FALSE(wd_step(base, 0u, false));                 /* snapshot, watch_tick=base */
	ASSERT_FALSE(wd_step(base + 700u, 3u, false));          /* 700ms (wrapped), <window */
	ASSERT_TRUE(wd_step(base + 800u, 4u, false));           /* 800ms past wrap -> trigger */
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("SPI wedge-recovery watchdog");
	RUN_TEST(test_idle_bus_never_resets);
	RUN_TEST(test_wedge_triggers);
	RUN_TEST(test_within_window_no_trigger);
	RUN_TEST(test_clean_transaction_clears_timer);
	RUN_TEST(test_single_glitch_then_quiet_no_trigger);
	RUN_TEST(test_fresh_after_trigger);
	RUN_TEST(test_wrap_safe);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
