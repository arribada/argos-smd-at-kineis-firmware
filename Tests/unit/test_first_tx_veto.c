/**
 * @file    test_first_tx_veto.c
 * @brief   Regression spec for the first-TX veto fix (2026-07).
 *
 * Mirrors the fixed first-TX arbitration in kns_app_uw_doppler.c MONITORING:
 *  - surface_tx_pending is consumed at DISPATCH, not at gate-1, so a veto by a
 *    later gate (battery / rate / backoff / gesture / hard-min floor) leaves
 *    the first TX of the surface window armed instead of silencing the window
 *    (permanently, for an animal floating at the surface).
 *  - Gates with a known time horizon park a retry deadline in
 *    first_tx_retry_tick so the loop can deep-sleep through the wait.
 *  - Zero-TX safety net: a deployed tag continuously at SURFACE never goes
 *    more than tx_seq_restart_s without ATTEMPTING a TX (tx_count==0 and no
 *    pending flag => re-arm), whatever path dropped the first TX.
 */

#include "test_framework.h"

static uint32_t fk_tick;

#define TX_HARD_MIN_INTERVAL_MS   2000u
#define FIRST_TX_BAT_RETRY_MS    60000u
#define SWS_NEXT_SAMPLE_MS        1000u  /* surface cadence stand-in */

/* ---- Mirrored state ---- */
typedef struct {
	/* persistent scheduling state */
	int      pending;              /* surface_tx_pending */
	uint32_t retry_tick;           /* first_tx_retry_tick (0 = none) */
	uint32_t last_tx_tick;
	uint32_t last_actual_tx_tick;  /* never zeroed by surface paths */
	uint32_t tx_count;
	uint32_t surface_since;        /* surface_since_tick (0 = not at surface) */
	/* config */
	uint32_t offset_ms;            /* first_tx_random_offset_ms */
	uint16_t cooldown_s;           /* tx_cfg.tx_cooldown_s */
	uint16_t seq_restart_s;        /* tx_cfg.tx_seq_restart_s */
	/* per-pass gate inputs */
	int      bat_ok;
	int      rate_blocked;   uint32_t rate_retry_s;
	int      backoff_blocked; uint32_t backoff_retry_s;
	int      gesture;
	/* per-pass outputs */
	int      dispatched;
	int      rearmed;              /* zero-TX net fired this pass */
	int      slept;
	uint32_t slept_delta;
} Sm;

static uint32_t retry_at(uint32_t delay_ms)
{
	uint32_t t = fk_tick + delay_ms;
	return (t == 0u) ? 1u : t;
}

/* One MONITORING pass over the fixed arbitration (deployed, at SURFACE). */
static void step(Sm *s)
{
	s->dispatched = 0;
	s->rearmed = 0;
	s->slept = 0;
	s->slept_delta = 0;

	int should_tx = 0, first_attempt = 0;
	uint32_t since_last = fk_tick - s->last_tx_tick;
	uint32_t cooldown_ms = (uint32_t)s->cooldown_s * 1000u;

	/* gate 1: offset + cooldown + veto-retry holdoff */
	if (s->pending) {
		int cooldown_ok = (cooldown_ms == 0u) ||
			(s->last_actual_tx_tick == 0u) ||
			((fk_tick - s->last_actual_tx_tick) >= cooldown_ms);
		int retry_ok = (s->retry_tick == 0u) ||
			((int32_t)(fk_tick - s->retry_tick) >= 0);
		if (since_last >= s->offset_ms && cooldown_ok && retry_ok) {
			should_tx = 1;
			first_attempt = 1;
			/* pending consumed at dispatch, NOT here */
		} else if (!cooldown_ok && s->retry_tick == 0u) {
			/* park the cooldown wait so the loop can sleep through it */
			uint32_t since_actual = fk_tick - s->last_actual_tx_tick;
			s->retry_tick = retry_at(cooldown_ms - since_actual);
		}
	}

	/* zero-TX safety net (runs after the seq-restart block in prod) */
	if (s->seq_restart_s > 0u && !s->pending && s->tx_count == 0u &&
	    s->surface_since != 0u) {
		uint32_t rst_ms = (uint32_t)s->seq_restart_s * 1000u;
		uint32_t anchor = s->surface_since;
		if (s->last_actual_tx_tick != 0u &&
		    (int32_t)(s->last_actual_tx_tick - anchor) > 0)
			anchor = s->last_actual_tx_tick;
		if ((fk_tick - anchor) >= rst_ms) {
			s->pending = 1;
			s->retry_tick = 0;
			s->last_tx_tick = 0;
			s->rearmed = 1;
		}
	}

	/* veto gates — each only clears should_tx, never the pending flag */
	if (should_tx && !s->bat_ok) {
		should_tx = 0;
		if (first_attempt)
			s->retry_tick = retry_at(FIRST_TX_BAT_RETRY_MS);
	}
	if (should_tx && s->rate_blocked) {
		should_tx = 0;
		if (first_attempt)
			s->retry_tick = retry_at(
				((s->rate_retry_s != 0u) ? s->rate_retry_s : 1u) * 1000u);
	}
	if (should_tx && s->backoff_blocked) {
		should_tx = 0;
		if (first_attempt)
			s->retry_tick = retry_at(
				((s->backoff_retry_s != 0u) ? s->backoff_retry_s : 1u) * 1000u);
	}
	if (should_tx && s->gesture)
		should_tx = 0;   /* true defer now: pending stays armed, no holdoff */
	if (should_tx && s->last_actual_tx_tick != 0u &&
	    (fk_tick - s->last_actual_tx_tick) < TX_HARD_MIN_INTERVAL_MS) {
		should_tx = 0;
		if (first_attempt)
			s->retry_tick = retry_at(TX_HARD_MIN_INTERVAL_MS -
				(fk_tick - s->last_actual_tx_tick));
	}

	if (should_tx) {
		/* dispatch: the ONLY place the arm is consumed */
		s->pending = 0;
		s->retry_tick = 0;
		s->dispatched = 1;
		s->last_tx_tick = fk_tick;
		s->last_actual_tx_tick = (fk_tick == 0u) ? 1u : fk_tick;
		s->tx_count++;
	} else {
		/* surface-idle sleep gate: pending blocks sleep UNLESS parked on a
		 * retry holdoff still in the future */
		int pending_hold = s->pending && s->retry_tick != 0u &&
			(int32_t)(s->retry_tick - fk_tick) > 0;
		if (!s->pending || pending_hold) {
			uint32_t delta = SWS_NEXT_SAMPLE_MS;
			if (pending_hold) {
				uint32_t r = s->retry_tick - fk_tick;
				if (r < delta)
					delta = r;
			}
			s->slept = 1;
			s->slept_delta = delta;
		}
	}
}

/* Surface-detect arm (mirror of the UW->SURF / cold-boot / boot-at-surface
 * paths + reset_tx_scheduling). */
static void arm_surface(Sm *s, uint32_t offset_ms)
{
	s->tx_count = 0;
	s->pending = 1;
	s->retry_tick = 0;      /* reset_tx_scheduling clears the holdoff */
	s->last_tx_tick = 0;
	s->offset_ms = offset_ms;
	if (s->surface_since == 0u)
		s->surface_since = (fk_tick == 0u) ? 1u : fk_tick;
}

static void go_underwater(Sm *s)
{
	s->tx_count = 0;
	s->pending = 0;
	s->retry_tick = 0;
	s->surface_since = 0;
}

static Sm mk(void)
{
	Sm s;
	memset(&s, 0, sizeof(s));
	s.bat_ok = 1;
	s.cooldown_s = 10;
	s.seq_restart_s = 1200;
	return s;
}

/* ---- Regression: happy path unchanged ---- */
static void test_happy_path_first_pass_dispatch(void)
{
	Sm s = mk();
	fk_tick = 100000;
	arm_surface(&s, 0);
	step(&s);
	ASSERT_TRUE(s.dispatched);
	ASSERT_FALSE(s.pending);
	ASSERT_EQ(0, s.retry_tick);
	ASSERT_EQ(1, s.tx_count);
	TEST_PASS();
}

/* ---- The core bug: a battery veto no longer kills the window ---- */
static void test_battery_veto_keeps_window_armed(void)
{
	Sm s = mk();
	fk_tick = 100000;
	arm_surface(&s, 0);
	s.bat_ok = 0;             /* transient false-low read */
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.pending);   /* pre-fix: pending was already consumed */
	ASSERT_NE(0, s.retry_tick);

	/* holdoff blocks attempts before 60 s, but the loop can SLEEP */
	s.bat_ok = 1;
	fk_tick += 30000;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.slept);     /* pending_hold allows STOP2 */

	/* retry after 60 s: battery recovered -> TX fires */
	fk_tick += 30001;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

static void test_genuinely_low_battery_stays_inhibited(void)
{
	Sm s = mk();
	fk_tick = 500000;
	arm_surface(&s, 0);
	s.bat_ok = 0;
	for (int i = 0; i < 5; i++) {
		step(&s);
		ASSERT_FALSE(s.dispatched);   /* hard floor keeps protecting */
		fk_tick += 61000;
	}
	ASSERT_TRUE(s.pending);           /* window still armed for recovery */
	TEST_PASS();
}

/* ---- Backoff / rate vetoes retry at their own horizon ---- */
static void test_backoff_veto_retries_at_horizon(void)
{
	Sm s = mk();
	fk_tick = 200000;
	arm_surface(&s, 0);
	s.backoff_blocked = 1;
	s.backoff_retry_s = 120;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.pending);

	/* before horizon: gated + sleepable */
	s.backoff_blocked = 0;
	fk_tick += 60000;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.slept);

	/* at horizon: fires */
	fk_tick += 60001;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

static void test_rate_veto_retries_at_horizon(void)
{
	Sm s = mk();
	fk_tick = 300000;
	arm_surface(&s, 0);
	s.rate_blocked = 1;
	s.rate_retry_s = 300;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.pending);
	s.rate_blocked = 0;
	fk_tick += 300001;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

/* ---- Gesture veto = true defer (retry next pass, no holdoff) ---- */
static void test_gesture_veto_defers_not_drops(void)
{
	Sm s = mk();
	fk_tick = 400000;
	arm_surface(&s, 0);
	s.gesture = 1;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.pending);
	ASSERT_EQ(0, s.retry_tick);   /* no holdoff for a seconds-scale wait */
	s.gesture = 0;
	fk_tick += 100;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

/* ---- Hard-min floor: retry exactly when the floor clears ---- */
static void test_hard_min_floor_retries(void)
{
	Sm s = mk();
	fk_tick = 500000;
	s.cooldown_s = 0;                     /* operator opt-out */
	arm_surface(&s, 0);
	s.last_actual_tx_tick = fk_tick - 500;  /* actual TX 0.5 s ago */
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.pending);
	ASSERT_EQ(fk_tick + 1500, s.retry_tick);
	fk_tick += 1500;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

/* ---- Cooldown wait now parks a holdoff => the loop sleeps through it ---- */
static void test_cooldown_wait_sleeps_instead_of_spinning(void)
{
	Sm s = mk();
	fk_tick = 600000;
	s.cooldown_s = 10;
	arm_surface(&s, 0);
	s.last_actual_tx_tick = fk_tick - 3000;  /* TX 3 s ago, 7 s to wait */
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.pending);
	ASSERT_EQ(fk_tick + 7000, s.retry_tick);
	ASSERT_TRUE(s.slept);                    /* pre-fix: span awake 7 s */
	ASSERT_EQ(SWS_NEXT_SAMPLE_MS, s.slept_delta); /* min(1 s SWS, 7 s) */
	fk_tick += 7000;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

/* ---- Zero-TX safety net ---- */
static void test_zero_tx_net_rearms_after_restart_window(void)
{
	Sm s = mk();
	fk_tick = 700000;
	/* pathological: at surface, no pending, no TX ever (dropped by an
	 * unknown path) — pre-fix this was silent forever */
	s.surface_since = fk_tick;
	fk_tick += (uint32_t)s.seq_restart_s * 1000u - 1u;
	step(&s);
	ASSERT_FALSE(s.rearmed);          /* not before the window */
	fk_tick += 1u;
	step(&s);
	ASSERT_TRUE(s.rearmed);
	ASSERT_TRUE(s.pending);
	/* next pass: all gates pass -> beacon resumes */
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

static void test_zero_tx_net_anchored_to_last_actual_tx(void)
{
	Sm s = mk();
	fk_tick = 800000;
	s.surface_since = fk_tick - 2000000;       /* at surface for a while */
	s.last_actual_tx_tick = fk_tick - 600000;  /* TX 10 min ago */
	step(&s);
	ASSERT_FALSE(s.rearmed);   /* 10 min < 20 min from the LATER anchor */
	fk_tick += 600000;
	step(&s);
	ASSERT_TRUE(s.rearmed);    /* 20 min since last actual TX */
	TEST_PASS();
}

static void test_zero_tx_net_respects_gates_and_disable(void)
{
	/* disabled by tx_seq_restart_s == 0 */
	Sm s = mk();
	fk_tick = 900000;
	s.seq_restart_s = 0;
	s.surface_since = fk_tick - 10000000;
	step(&s);
	ASSERT_FALSE(s.rearmed);

	/* never fires while a sequence ran (tx_count > 0) */
	Sm t = mk();
	fk_tick = 950000;
	t.surface_since = fk_tick - 10000000;
	t.tx_count = 3;
	step(&t);
	ASSERT_FALSE(t.rearmed);

	/* never fires while pending is already armed */
	Sm u = mk();
	fk_tick = 980000;
	u.surface_since = fk_tick - 10000000;
	u.pending = 1;
	u.retry_tick = retry_at(30000);   /* parked on a battery retry */
	step(&u);
	ASSERT_FALSE(u.rearmed);

	/* re-armed attempt still obeys the gates (battery still low) */
	Sm v = mk();
	fk_tick = 990000;
	v.surface_since = fk_tick - 10000000;
	v.bat_ok = 0;
	step(&v);
	ASSERT_TRUE(v.rearmed);
	step(&v);                          /* attempt pass */
	ASSERT_FALSE(v.dispatched);        /* vetoed by battery as it must be */
	ASSERT_TRUE(v.pending);
	TEST_PASS();
}

/* ---- Wrap safety of the holdoff compare ---- */
static void test_retry_holdoff_wrap_safe(void)
{
	Sm s = mk();
	/* retry deadline lands AFTER the 32-bit tick wrap */
	fk_tick = 0xFFFFFFF0u;
	arm_surface(&s, 0);
	s.bat_ok = 0;
	step(&s);                          /* holdoff = 0xFFFFFFF0 + 60000 (wraps) */
	ASSERT_FALSE(s.dispatched);
	uint32_t expected = 0xFFFFFFF0u + 60000u;   /* wrapped value */
	ASSERT_EQ_HEX(expected, s.retry_tick);

	/* pre-wrap: still held */
	s.bat_ok = 1;
	fk_tick = 0xFFFFFFFAu;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.slept);

	/* post-wrap, past the deadline: fires */
	fk_tick = expected + 1u;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

/* ---- Dive/resurface resets everything cleanly ---- */
static void test_underwater_resets_holdoff_and_net(void)
{
	Sm s = mk();
	fk_tick = 1000000;
	arm_surface(&s, 0);
	s.bat_ok = 0;
	step(&s);                 /* battery veto arms a 60 s holdoff */
	ASSERT_NE(0, s.retry_tick);

	go_underwater(&s);        /* dive: reset_tx_scheduling + anchor cleared */
	ASSERT_EQ(0, s.retry_tick);
	ASSERT_EQ(0u, s.surface_since);

	fk_tick += 5000;
	s.bat_ok = 1;
	arm_surface(&s, 0);       /* resurface */
	/* cooldown vs last_actual: no prior actual TX here -> fires at once */
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("UW_DOPPLER first-TX veto fix (pending consumed at dispatch)");

	RUN_TEST(test_happy_path_first_pass_dispatch);
	RUN_TEST(test_battery_veto_keeps_window_armed);
	RUN_TEST(test_genuinely_low_battery_stays_inhibited);
	RUN_TEST(test_backoff_veto_retries_at_horizon);
	RUN_TEST(test_rate_veto_retries_at_horizon);
	RUN_TEST(test_gesture_veto_defers_not_drops);
	RUN_TEST(test_hard_min_floor_retries);
	RUN_TEST(test_cooldown_wait_sleeps_instead_of_spinning);
	RUN_TEST(test_zero_tx_net_rearms_after_restart_window);
	RUN_TEST(test_zero_tx_net_anchored_to_last_actual_tx);
	RUN_TEST(test_zero_tx_net_respects_gates_and_disable);
	RUN_TEST(test_retry_holdoff_wrap_safe);
	RUN_TEST(test_underwater_resets_holdoff_and_net);

	TEST_SUITE_END();
	return (tests_failed == 0) ? 0 : 1;
}
