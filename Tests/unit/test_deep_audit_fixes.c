/**
 * @file    test_deep_audit_fixes.c
 * @brief   Regression specs for the deep-audit pass-2 fixes (2026-07).
 *
 * Mirrors the fixed logic of:
 *  1. Tick-wrap clear-on-expiry deadlines (console-RX holdoff in
 *     mgr_lpm_uw.c, SWS TX-blank in mgr_sws.c, SPI STOP grace in lpm.c):
 *     a stale deadline must NEVER re-engage 2^31 ms after expiry.
 *  2. MGR_RATE commit-on-blocked: awake-time progress must accumulate
 *     across resets so a full quota cannot deadlock under a crash loop.
 *  3. MGR_PMLOG_get hole-skipping: torn slots are skipped, newest entries
 *     stay reachable, garbage slots are never returned.
 *  4. Gesture non-idle watchdog: reed activity re-arms the budget; only a
 *     truly wedged (edge-less) FSM is force-aborted.
 *  5. Bootloader WRITE parser: a full 248-byte chunk parses without
 *     truncation (4-byte addr + BL_CHUNK_SIZE data).
 */

#include "test_framework.h"

static uint32_t fk_tick;

/* =====================================================================
 * 1. Clear-on-expiry deadline (shared pattern of the three fixes)
 * ===================================================================== */

/* Mirror: mgr_lpm_uw.c console holdoff / mgr_sws.c blank / lpm.c grace. */
static uint32_t deadline_tick;   /* 0 = disarmed */

static int deadline_blocks_fixed(void)
{
	if (deadline_tick == 0u)
		return 0;
	if ((int32_t)(deadline_tick - fk_tick) > 0)
		return 1;
	deadline_tick = 0u;    /* clear-on-expiry */
	return 0;
}

/* The PRE-FIX form (kept to document the defect the fix removes). */
static int deadline_blocks_old(void)
{
	return deadline_tick != 0u &&
	       (fk_tick - deadline_tick) > 0x80000000u;
}

static void test_deadline_blocks_inside_window(void)
{
	fk_tick = 1000000;
	deadline_tick = fk_tick + 2500;
	ASSERT_TRUE(deadline_blocks_fixed());
	fk_tick += 2499;
	ASSERT_TRUE(deadline_blocks_fixed());
	fk_tick += 1;
	ASSERT_FALSE(deadline_blocks_fixed());   /* expired */
	ASSERT_EQ(0, deadline_tick);             /* and disarmed */
	TEST_PASS();
}

static void test_deadline_no_reengage_at_half_wrap(void)
{
	/* Arm at T, let it expire, then advance 24.85 days: the OLD compare
	 * re-engaged; the FIXED one must stay clear because the deadline was
	 * disarmed at expiry. */
	fk_tick = 5000;
	deadline_tick = fk_tick + 2500;
	fk_tick += 3000;
	ASSERT_FALSE(deadline_blocks_fixed());   /* consumes + clears */

	fk_tick += 0x80000000u;                  /* +24.85 days */
	ASSERT_FALSE(deadline_blocks_fixed());   /* fixed: still clear */

	/* Prove the old form was broken under the same timeline. */
	fk_tick = 5000;
	deadline_tick = fk_tick + 2500;          /* stale, never cleared */
	fk_tick += 3000 + 0x80000000u;
	ASSERT_TRUE(deadline_blocks_old());      /* the 24.8-day sleep veto */
	TEST_PASS();
}

static void test_deadline_never_armed_is_never_blocking(void)
{
	/* SWS-blank never-armed case: with deadline 0 the OLD signed compare
	 * (int32_t)(0 - now) > 0 blocked for now in (2^31, 2^32). The fixed
	 * form treats 0 as disarmed at any uptime. */
	deadline_tick = 0u;
	fk_tick = 0x80000001u;                   /* uptime 24.85 d + 1 ms */
	ASSERT_FALSE(deadline_blocks_fixed());
	fk_tick = 0xFFFFFFFFu;                   /* uptime 49.7 d */
	ASSERT_FALSE(deadline_blocks_fixed());
	TEST_PASS();
}

static void test_deadline_arm_across_wrap_still_works(void)
{
	/* Deadline lands after the 32-bit wrap: must block until it, then
	 * clear. (The arm sites nudge an exact-0 result to 1.) */
	fk_tick = 0xFFFFFFF0u;
	uint32_t until = fk_tick + 2500u;        /* wraps to 0x9C4 - 16 */
	deadline_tick = (until == 0u) ? 1u : until;
	ASSERT_TRUE(deadline_blocks_fixed());
	fk_tick = deadline_tick - 1u;
	ASSERT_TRUE(deadline_blocks_fixed());
	fk_tick = deadline_tick;
	ASSERT_FALSE(deadline_blocks_fixed());
	ASSERT_EQ(0, deadline_tick);
	TEST_PASS();
}

/* =====================================================================
 * 2. MGR_RATE commit-on-blocked (crash-loop quota deadlock)
 * ===================================================================== */

/* Mirror of the limiter's persistent snapshot + awake clock. */
typedef struct {
	uint32_t oldest_ts;     /* single-entry stand-in for the ring */
	uint32_t count;
	uint32_t mono_base_s;   /* committed clock snapshot */
} RateRing;

static uint32_t rate_mono_s;

static void rate_boot(RateRing *r)
{
	/* init resumes the clock from the last committed snapshot, +1 s
	 * round-up (fix 2026-07): the sub-second remainder is not retained,
	 * so without the round-up a crash loop with per-boot awake < 1 s
	 * never advanced the clock at all (sim-proven permanent block). */
	rate_mono_s = r->mono_base_s + 1u;
}

static int rate_is_blocked(RateRing *r, uint32_t window_s, uint32_t max_tx,
                           int commit_on_blocked)
{
	/* trim */
	if (r->count > 0 && rate_mono_s >= r->oldest_ts &&
	    (rate_mono_s - r->oldest_ts) >= window_s)
		r->count = 0;
	/* fix: persist clock progress even while blocked */
	if (commit_on_blocked && rate_mono_s != r->mono_base_s)
		r->mono_base_s = rate_mono_s;
	return r->count >= max_tx;
}

static void test_rate_crash_loop_deadlock_fixed(void)
{
	/* Quota full at mono=1000 s, window 3600 s. Device crash-loops every
	 * 1800 s of awake time. WITHOUT the fix: every boot resumes at 1000,
	 * never reaches 4600 -> blocked forever. WITH the fix: progress
	 * accumulates and the quota unblocks after ~2 reboots. */
	RateRing r = { .oldest_ts = 1000, .count = 20, .mono_base_s = 1000 };

	/* --- old behaviour: 10 reboots, still blocked --- */
	for (int boot = 0; boot < 10; boot++) {
		rate_boot(&r);
		for (int s = 0; s < 1800; s += 600) {
			rate_mono_s += 600;
			ASSERT_TRUE(rate_is_blocked(&r, 3600, 20, 0));
		}
	}
	ASSERT_TRUE(rate_is_blocked(&r, 3600, 20, 0));   /* deadlocked */

	/* --- fixed behaviour: unblocks once TOTAL awake time >= window --- */
	RateRing f = { .oldest_ts = 1000, .count = 20, .mono_base_s = 1000 };
	int boots_to_unblock = 0;
	int blocked = 1;
	for (int boot = 0; boot < 10 && blocked; boot++) {
		boots_to_unblock++;
		rate_boot(&f);
		for (int s = 0; s < 1800 && blocked; s += 600) {
			rate_mono_s += 600;
			blocked = rate_is_blocked(&f, 3600, 20, 1);
		}
	}
	ASSERT_FALSE(blocked);
	ASSERT_EQ(2, boots_to_unblock);   /* 2 x 1800 s awake = 3600 s window */
	TEST_PASS();
}

static void test_rate_healthy_path_unchanged(void)
{
	/* No reset: quota expires after window_s (the boot round-up makes
	 * entries age at most 1 s early per reboot — conservative for
	 * battery, negligible against the 3600 s window). */
	RateRing r = { .oldest_ts = 50, .count = 20, .mono_base_s = 50 };
	rate_boot(&r);                       /* clock at 51 (+1 round-up) */
	rate_mono_s += 3598;                 /* 3599 s since oldest */
	ASSERT_TRUE(rate_is_blocked(&r, 3600, 20, 1));
	rate_mono_s += 1;                    /* 3600 s — window drained */
	ASSERT_FALSE(rate_is_blocked(&r, 3600, 20, 1));
	TEST_PASS();
}

/** The sim-found sub-second crash-loop regime: with the +1 round-up the
 *  window drains after at most window_s reboots instead of never. */
static void test_rate_subsecond_crash_loop_bounded(void)
{
	RateRing r = { .oldest_ts = 1000, .count = 20, .mono_base_s = 1000 };
	int boots = 0;
	while (rate_is_blocked(&r, 3600, 20, 1) && boots < 10000) {
		boots++;
		rate_boot(&r);   /* awake < 1 s: only the round-up advances */
	}
	ASSERT_TRUE(boots <= 3601);   /* bounded — pre-fix: infinite */
	ASSERT_FALSE(rate_is_blocked(&r, 3600, 20, 1));
	TEST_PASS();
}

/* =====================================================================
 * 3. MGR_PMLOG_get hole-skipping
 * ===================================================================== */

#define PM_SLOTS 8
#define PM_VALID 0xB5u
#define PM_EMPTY 0xFFu   /* torn slot: magic erased, dw1 programmed */

static uint8_t pm_magic[PM_SLOTS];

static uint16_t pm_count(void)
{
	uint16_t n = 0;
	for (int i = 0; i < PM_SLOTS; i++)
		if (pm_magic[i] == PM_VALID)
			n++;
	return n;
}

/* Mirror of the FIXED get(): logical index over VALID slots only. */
static int pm_get_fixed(uint16_t index)
{
	if (index >= pm_count())
		return -1;
	uint16_t seen = 0;
	for (int i = 0; i < PM_SLOTS; i++) {
		if (pm_magic[i] != PM_VALID)
			continue;
		if (seen == index)
			return i;
		seen++;
	}
	return -1;
}

static void test_pmlog_holes_skipped(void)
{
	/* Slots 0-2 valid, slot 3 TORN (hole), slots 4-5 valid. */
	memset(pm_magic, 0xEE, sizeof(pm_magic));  /* 0xEE = untouched */
	pm_magic[0] = pm_magic[1] = pm_magic[2] = PM_VALID;
	pm_magic[3] = PM_EMPTY;                    /* torn */
	pm_magic[4] = pm_magic[5] = PM_VALID;

	ASSERT_EQ(5, pm_count());
	ASSERT_EQ(0, pm_get_fixed(0));
	ASSERT_EQ(2, pm_get_fixed(2));
	ASSERT_EQ(4, pm_get_fixed(3));   /* hole skipped */
	ASSERT_EQ(5, pm_get_fixed(4));   /* newest entry reachable */
	ASSERT_EQ(-1, pm_get_fixed(5));  /* out of range */
	/* The OLD direct mapping would have returned slot 3 (garbage) for
	 * index 3 and hidden slot 5 entirely. */
	TEST_PASS();
}

static void test_pmlog_no_holes_identity(void)
{
	memset(pm_magic, PM_EMPTY, sizeof(pm_magic));
	pm_magic[0] = pm_magic[1] = PM_VALID;
	ASSERT_EQ(0, pm_get_fixed(0));
	ASSERT_EQ(1, pm_get_fixed(1));
	ASSERT_EQ(-1, pm_get_fixed(2));
	TEST_PASS();
}

/* =====================================================================
 * 4. Gesture non-idle watchdog: activity re-arms the budget
 * ===================================================================== */

#define G_BUDGET_MS 30000u

static uint32_t g_nonidle_first;
static int g_fsm_idle;

/* Mirror: reed edge re-arms; watchdog aborts only past budget w/o edges. */
static void g_task(int reed_edge)
{
	if (reed_edge)
		g_nonidle_first = 0u;
}

static int g_is_interacting(void)
{
	if (g_fsm_idle) {
		g_nonidle_first = 0u;
		return 0;
	}
	if (g_nonidle_first == 0u)
		g_nonidle_first = (fk_tick == 0u) ? 1u : fk_tick;
	if ((fk_tick - g_nonidle_first) > G_BUDGET_MS) {
		g_fsm_idle = 1;   /* force-abort */
		g_nonidle_first = 0u;
		return 0;
	}
	return 1;
}

static void test_gesture_slow_operator_not_aborted(void)
{
	/* Wake blink 2.5 s, hold 6 s, ponder 8 s, release edge, confirm touch:
	 * edges at each step keep re-arming — never aborted. */
	fk_tick = 10000;
	g_fsm_idle = 0;
	g_nonidle_first = 0;

	ASSERT_TRUE(g_is_interacting());     /* WAKE_BLINK running */
	fk_tick += 2500;
	g_task(1);                           /* magnet ON edge */
	ASSERT_TRUE(g_is_interacting());
	fk_tick += 6000;                     /* holding */
	ASSERT_TRUE(g_is_interacting());
	fk_tick += 8000;                     /* pondering, magnet still ON */
	ASSERT_TRUE(g_is_interacting());
	g_task(1);                           /* release edge */
	fk_tick += 1500;
	ASSERT_TRUE(g_is_interacting());
	g_task(1);                           /* confirm touch edge */
	fk_tick += 1500;
	ASSERT_TRUE(g_is_interacting());     /* confirm blink completes */
	ASSERT_FALSE(g_fsm_idle);            /* shutdown NOT silently dropped */
	TEST_PASS();
}

static void test_gesture_wedged_fsm_still_aborted(void)
{
	/* Stuck-HIGH Hall: no edges ever — budget must still fire. */
	fk_tick = 50000;
	g_fsm_idle = 0;
	g_nonidle_first = 0;

	ASSERT_TRUE(g_is_interacting());
	fk_tick += G_BUDGET_MS + 1u;         /* 30 s with zero edges */
	ASSERT_FALSE(g_is_interacting());    /* force-aborted */
	ASSERT_TRUE(g_fsm_idle);
	TEST_PASS();
}

/* =====================================================================
 * 5. Bootloader WRITE parser: full 248-byte chunk (4+248 buffer)
 * ===================================================================== */

#define CHUNK 248

static int bl_parse_write_len(int data_bytes, int buf_total)
{
	/* Mirror: payload_len starts at 4 (address), hex pairs consumed while
	 * payload_len < buf_total. Returns parsed DATA byte count. */
	int payload_len = 4;
	int consumed = 0;
	while (consumed < data_bytes && payload_len < buf_total) {
		payload_len++;
		consumed++;
	}
	return payload_len - 4;
}

static void test_bl_write_parses_full_chunk(void)
{
	/* OLD: buffer = CHUNK -> 244 of 248 parsed (silent truncation, VERIFY
	 * always fails). FIXED: buffer = 4 + CHUNK -> all 248 parsed. */
	ASSERT_EQ(244, bl_parse_write_len(248, CHUNK));      /* old defect */
	ASSERT_EQ(248, bl_parse_write_len(248, 4 + CHUNK));  /* fixed */
	ASSERT_EQ(100, bl_parse_write_len(100, 4 + CHUNK));  /* short chunk OK */
	TEST_PASS();
}

/* =====================================================================
 * 6. Pass-3 fixes: park-wake grace, mid-sequence veto holdoff,
 *    SWS stale-high water recovery
 * ===================================================================== */

/* Mirror of the boot-loop park-wake grace (kns_app boot_loop_handle). */
static void test_park_wake_grants_one_attempt(void)
{
	unsigned failures = 16, armed = 1, parked = 0;
	/* wake from park: grace path fires BEFORE counting */
	if (armed) { armed = 0; failures = 16 - 1; }
	ASSERT_EQ(15, (long)failures);
	/* the granted attempt dies pre-MONITORING -> counter back at the rung */
	failures++;
	if (failures >= 16) { parked = 1; armed = 1; }
	ASSERT_TRUE(parked);          /* re-parks after ONE attempt, not zero */
	/* next park wake: grace again -> mission retried every 24 h */
	if (armed) { armed = 0; failures = 15; }
	ASSERT_EQ(15, (long)failures);
	TEST_PASS();
}

/* Mirror of the mid-sequence veto holdoff (kns_app periodic TX branch +
 * uw_ms_until_next_action). */
static void test_mid_sequence_veto_parks_and_sleeps(void)
{
	uint32_t retry_tick = 0, last_tx = 100000, interval = 10000;
	int tx_count = 1, bat_ok = 0, dispatched = 0, attempts = 0;

	for (fk_tick = 110000; fk_tick < 200000 && !dispatched;
	     fk_tick += 1000) {
		int retry_ok = (retry_tick == 0u) ||
			((int32_t)(fk_tick - retry_tick) >= 0);
		int should = (tx_count > 0) &&
			((fk_tick - last_tx) >= interval) && retry_ok;
		if (should) {
			attempts++;
			if (!bat_ok) {
				retry_tick = fk_tick + 60000u;   /* 60 s holdoff */
				bat_ok = 1;   /* pack recovers during the holdoff */
			} else {
				dispatched = 1;
			}
		}
		/* scheduler: with a parked holdoff, next-action is deferred */
		uint32_t r = ((fk_tick - last_tx) >= interval)
			? 0u : interval - (fk_tick - last_tx);
		if (retry_tick != 0u) {
			int32_t hold = (int32_t)(retry_tick - fk_tick);
			if (hold > 0 && (uint32_t)hold > r)
				r = (uint32_t)hold;
		}
		if (attempts > 0 && !dispatched)
			ASSERT_TRUE(r > 0);   /* pre-fix: r==0 -> awake spin */
	}
	ASSERT_TRUE(dispatched);
	ASSERT_EQ(2, (long)attempts);   /* one veto + one success — not 60 */
	TEST_PASS();
}

/* Mirror of SWS section 4c: stale-high water recovery. */
#define MIN_WATER_AIR_RATIO 3
static void test_sws_stale_water_recovery(void)
{
	unsigned air = 50, water = 3000, count = 0;
	unsigned threshold = air + ((water - air) * 35u) / 100u;  /* ~1082 */
	int recovered = 0;

	/* Submerged in fresh water: raw ~300 -> classified SURFACE (latched). */
	for (int i = 0; i < 60; i++) {
		unsigned raw = 300;
		int is_underwater = (raw > threshold);   /* false: 300 < 1082 */
		ASSERT_FALSE(is_underwater);
		if (!is_underwater && raw >= air * MIN_WATER_AIR_RATIO) {
			if (++count >= 60) {
				unsigned nw = raw * 115u / 100u;      /* 345 */
				if (nw < air * MIN_WATER_AIR_RATIO)
					nw = air * MIN_WATER_AIR_RATIO;
				if (nw < water) {
					water = nw;
					threshold = air + ((water - air) * 35u) / 100u;
					recovered = 1;
				}
				count = 0;
			}
		} else {
			count = 0;
		}
	}
	ASSERT_TRUE(recovered);
	ASSERT_EQ(345, (long)water);
	/* thresholds re-converged: the same submerged reading now reads UW */
	ASSERT_TRUE(300 > (long)threshold);            /* ~153 */
	/* and a genuine surfacing (raw ~air) still reads SURFACE */
	ASSERT_FALSE(50 > (long)threshold);
	/* a splashing SURFACE electrode (alternating air/water readings)
	 * never accumulates the 60-sample run */
	count = 0;
	int false_trigger = 0;
	for (int i = 0; i < 600; i++) {
		unsigned raw = (i % 2) ? 40 : 400;
		if (raw >= 50 * MIN_WATER_AIR_RATIO) {
			if (++count >= 60)
				false_trigger = 1;
		} else {
			count = 0;
		}
	}
	ASSERT_FALSE(false_trigger);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("Deep-audit pass-2 fixes (tick-wrap, RATE, PMLOG, gesture, BL)");

	RUN_TEST(test_deadline_blocks_inside_window);
	RUN_TEST(test_deadline_no_reengage_at_half_wrap);
	RUN_TEST(test_deadline_never_armed_is_never_blocking);
	RUN_TEST(test_deadline_arm_across_wrap_still_works);
	RUN_TEST(test_rate_crash_loop_deadlock_fixed);
	RUN_TEST(test_rate_healthy_path_unchanged);
	RUN_TEST(test_rate_subsecond_crash_loop_bounded);
	RUN_TEST(test_pmlog_holes_skipped);
	RUN_TEST(test_pmlog_no_holes_identity);
	RUN_TEST(test_gesture_slow_operator_not_aborted);
	RUN_TEST(test_gesture_wedged_fsm_still_aborted);
	RUN_TEST(test_bl_write_parses_full_chunk);
	RUN_TEST(test_park_wake_grants_one_attempt);
	RUN_TEST(test_mid_sequence_veto_parks_and_sleeps);
	RUN_TEST(test_sws_stale_water_recovery);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
