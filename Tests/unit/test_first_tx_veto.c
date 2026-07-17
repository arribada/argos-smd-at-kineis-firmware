/**
 * @file    test_first_tx_veto.c
 * @brief   A vetoed TX at the surface is retried, never dropped
 *          (kns_app_uw_doppler.c, MONITORING scheduler).
 *
 * Mirrors the TX arbitration:
 *  - surface_tx_pending is consumed at dispatch only. A veto by a later gate
 *    (battery / rate / backoff / gesture / inter-TX floor) keeps it armed;
 *    before, it was cleared at gate 1 with tx_count == 0 and nothing re-armed
 *    it until the next dive.
 *  - Gates with a known horizon park first_tx_retry_tick (first TX and
 *    mid-sequence alike, cooldown wait included); the surface idle path may
 *    then sleep until min(next action, retry). The random offset parks
 *    nothing.
 *  - Zero-TX safety net: at SURFACE with tx_count == 0 and nothing pending
 *    for tx_seq_restart_s, the first TX is re-armed.
 *  - A dive clears everything (reset_tx_scheduling).
 */

#include "test_framework.h"

static uint32_t fk_tick;

#define TX_HARD_MIN_INTERVAL_MS   2000u
#define FIRST_TX_BAT_RETRY_MS    60000u
#define FIRST_TX_MAX_RETRY_MS  3600000u
#define SWS_NEXT_SAMPLE_MS        1000u  /* surface cadence stand-in */

typedef struct {
	int      pending;              /* surface_tx_pending */
	uint32_t retry_tick;           /* first_tx_retry_tick (0 = none) */
	uint32_t last_tx_tick;
	uint32_t last_actual_tx_tick;
	uint32_t tx_count;
	uint32_t interval_ms;          /* current_interval_ms (0 = no schedule) */
	uint32_t surface_since;        /* surface_since_tick (0 = not at surface) */
	uint32_t offset_ms;            /* first_tx_random_offset_ms */
	uint16_t cooldown_s;
	uint16_t seq_restart_s;
	/* per-pass gate inputs */
	int      bat_ok;
	int      rate_blocked;    uint32_t rate_retry_s;
	int      backoff_blocked; uint32_t backoff_retry_s;
	int      gesture;
	/* per-pass outputs */
	int      dispatched;
	int      rearmed;
	int      slept;
	uint32_t slept_delta;
} Sm;

static uint32_t retry_at(uint32_t delay_ms)
{
	uint32_t t;

	if (delay_ms > FIRST_TX_MAX_RETRY_MS)
		delay_ms = FIRST_TX_MAX_RETRY_MS;
	t = fk_tick + delay_ms;
	return (t == 0u) ? 1u : t;
}

static uint32_t retry_at_s(uint32_t s)
{
	if (s == 0u)
		s = 1u;
	if (s > FIRST_TX_MAX_RETRY_MS / 1000u)
		s = FIRST_TX_MAX_RETRY_MS / 1000u;
	return retry_at(s * 1000u);
}

static int retry_holding(const Sm *s)
{
	return s->retry_tick != 0u && (int32_t)(s->retry_tick - fk_tick) > 0;
}

static int first_holding(const Sm *s)
{
	return s->pending && retry_holding(s);
}

/* uw_ms_until_next_action (tx cap not modelled). */
static uint32_t next_action_ms(const Sm *s)
{
	uint32_t delta = SWS_NEXT_SAMPLE_MS;
	uint32_t since = fk_tick - s->last_tx_tick;

	if (s->tx_count > 0u && s->interval_ms > 0u) {
		uint32_t r = (since >= s->interval_ms) ? 0u : (s->interval_ms - since);

		if (retry_holding(s) && (s->retry_tick - fk_tick) > r)
			r = s->retry_tick - fk_tick;
		if (r < delta)
			delta = r;
	}
	if (first_holding(s) && (s->retry_tick - fk_tick) < delta)
		delta = s->retry_tick - fk_tick;
	return delta;
}

/* One MONITORING pass, deployed, at SURFACE. */
static void step(Sm *s)
{
	int should_tx = 0;
	uint32_t since_last = fk_tick - s->last_tx_tick;
	const int retry_ok = !retry_holding(s);

	s->dispatched = 0;
	s->rearmed = 0;
	s->slept = 0;
	s->slept_delta = 0;

	if (s->pending) {
		uint32_t cooldown_ms = (uint32_t)s->cooldown_s * 1000u;
		int cooldown_ok = (cooldown_ms == 0u) ||
			(s->last_actual_tx_tick == 0u) ||
			((fk_tick - s->last_actual_tx_tick) >= cooldown_ms);

		if (since_last >= s->offset_ms && cooldown_ok && retry_ok) {
			should_tx = 1;
		} else if (!cooldown_ok && !retry_holding(s)) {
			s->retry_tick = retry_at(cooldown_ms -
				(fk_tick - s->last_actual_tx_tick));
		}
	} else if (s->tx_count > 0u && s->interval_ms > 0u) {
		if (since_last >= s->interval_ms && retry_ok)
			should_tx = 1;
	}

	/* zero-TX safety net */
	if (s->seq_restart_s > 0u && !s->pending && s->tx_count == 0u &&
	    s->surface_since != 0u) {
		uint32_t anchor = s->surface_since;

		if (s->last_actual_tx_tick != 0u &&
		    (int32_t)(s->last_actual_tx_tick - anchor) > 0)
			anchor = s->last_actual_tx_tick;
		if ((fk_tick - anchor) >= (uint32_t)s->seq_restart_s * 1000u) {
			s->pending = 1;
			s->retry_tick = 0;
			s->last_tx_tick = 0;
			s->rearmed = 1;
		}
	}

	if (should_tx && !s->bat_ok) {
		should_tx = 0;
		s->retry_tick = retry_at(FIRST_TX_BAT_RETRY_MS);
	}
	if (should_tx && s->rate_blocked) {
		should_tx = 0;
		s->retry_tick = retry_at_s(s->rate_retry_s);
	}
	if (should_tx && s->backoff_blocked) {
		should_tx = 0;
		s->retry_tick = retry_at_s(s->backoff_retry_s);
	}
	if (should_tx && s->gesture)
		should_tx = 0;
	if (should_tx && s->last_actual_tx_tick != 0u) {
		uint32_t floor_elapsed = fk_tick - s->last_actual_tx_tick;

		if (floor_elapsed < TX_HARD_MIN_INTERVAL_MS) {
			should_tx = 0;
			s->retry_tick = retry_at(TX_HARD_MIN_INTERVAL_MS - floor_elapsed);
		}
	}

	if (should_tx) {
		s->pending = 0;
		s->retry_tick = 0;
		s->dispatched = 1;
		s->last_tx_tick = (fk_tick == 0u) ? 1u : fk_tick;
		s->last_actual_tx_tick = s->last_tx_tick;
		s->tx_count++;
	} else if (!s->pending || first_holding(s)) {
		uint32_t delta = next_action_ms(s);

		if (delta > 0u) {
			s->slept = 1;
			s->slept_delta = delta;
		}
	}
}

/* UW->SURF detect + reset_tx_scheduling. */
static void arm_surface(Sm *s, uint32_t offset_ms)
{
	s->tx_count = 0;
	s->interval_ms = 0;
	s->pending = 1;
	s->retry_tick = 0;
	s->last_tx_tick = 0;
	s->offset_ms = offset_ms;
	if (s->surface_since == 0u)
		s->surface_since = (fk_tick == 0u) ? 1u : fk_tick;
}

/* SURF->UW: reset_tx_scheduling. */
static void go_underwater(Sm *s)
{
	s->tx_count = 0;
	s->interval_ms = 0;
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

static void test_happy_path_first_pass_dispatch(void)
{
	Sm s = mk();

	TEST_START("No veto: first pass dispatches, pending consumed");
	fk_tick = 100000;
	arm_surface(&s, 0);
	step(&s);
	ASSERT_TRUE(s.dispatched);
	ASSERT_FALSE(s.pending);
	ASSERT_EQ(0, s.retry_tick);
	ASSERT_EQ(1, s.tx_count);
	TEST_PASS();
}

static void test_battery_veto_keeps_window_and_retries(void)
{
	Sm s = mk();

	TEST_START("Battery veto: pending kept, retry after 60 s, sleeps meanwhile");
	fk_tick = 200000;
	arm_surface(&s, 0);
	s.bat_ok = 0;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.pending);
	ASSERT_EQ(fk_tick + FIRST_TX_BAT_RETRY_MS, s.retry_tick);
	ASSERT_TRUE(s.slept);
	ASSERT_EQ(SWS_NEXT_SAMPLE_MS, s.slept_delta);

	/* recovered, but holdoff not expired: no TX, battery not re-read */
	s.bat_ok = 1;
	fk_tick += FIRST_TX_BAT_RETRY_MS - 1u;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_EQ(1, s.slept_delta);
	fk_tick += 1u;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	ASSERT_FALSE(s.pending);
	TEST_PASS();
}

static void test_floating_animal_not_silenced(void)
{
	Sm s = mk();
	int i;

	TEST_START("Veto at surface: TX resumes without a dive (was silent)");
	fk_tick = 300000;
	arm_surface(&s, 0);
	s.bat_ok = 0;
	for (i = 0; i < 10; i++) {          /* 10 min of low readings */
		step(&s);
		ASSERT_FALSE(s.dispatched);
		fk_tick += FIRST_TX_BAT_RETRY_MS;
	}
	s.bat_ok = 1;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

static void test_rate_and_backoff_retry_at_horizon(void)
{
	Sm s = mk();
	Sm b = mk();

	TEST_START("Rate / backoff veto: retry exactly at the reported horizon");
	fk_tick = 400000;
	arm_surface(&s, 0);
	s.rate_blocked = 1;
	s.rate_retry_s = 300;
	step(&s);
	ASSERT_TRUE(s.pending);
	ASSERT_EQ(fk_tick + 300000u, s.retry_tick);
	s.rate_blocked = 0;
	fk_tick += 299999u;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	fk_tick += 1u;
	step(&s);
	ASSERT_TRUE(s.dispatched);

	fk_tick = 500000;
	arm_surface(&b, 0);
	b.backoff_blocked = 1;
	b.backoff_retry_s = 0;              /* 0 s reported: retry after 1 s */
	step(&b);
	ASSERT_EQ(fk_tick + 1000u, b.retry_tick);
	TEST_PASS();
}

static void test_long_horizon_clamped_to_one_hour(void)
{
	Sm s = mk();

	TEST_START("Rate horizon > 1 h (or *1000 overflow) clamped to 1 h");
	fk_tick = 600000;
	arm_surface(&s, 0);
	s.rate_blocked = 1;
	s.rate_retry_s = 5000000u;          /* * 1000 would overflow 32 bits */
	step(&s);
	ASSERT_EQ(fk_tick + FIRST_TX_MAX_RETRY_MS, s.retry_tick);
	TEST_PASS();
}

static void test_gesture_defers_without_holdoff(void)
{
	Sm s = mk();

	TEST_START("Gesture veto: true defer, no holdoff, no sleep while pending");
	fk_tick = 700000;
	arm_surface(&s, 0);
	s.gesture = 1;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.pending);
	ASSERT_EQ(0, s.retry_tick);
	ASSERT_FALSE(s.slept);
	s.gesture = 0;
	fk_tick += 100;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

static void test_hard_floor_retries_when_clear(void)
{
	Sm s = mk();

	TEST_START("Inter-TX floor: retry exactly when the 2 s floor clears");
	fk_tick = 800000;
	s.cooldown_s = 0;
	arm_surface(&s, 0);
	s.last_actual_tx_tick = fk_tick - 500u;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_EQ(fk_tick + 1500u, s.retry_tick);
	fk_tick += 1500u;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

static void test_cooldown_wait_parked(void)
{
	Sm s = mk();

	TEST_START("Cooldown wait: parked as a retry, the loop sleeps through it");
	fk_tick = 900000;
	arm_surface(&s, 0);
	s.last_actual_tx_tick = fk_tick - 3000u;   /* 7 s of cooldown left */
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.pending);
	ASSERT_EQ(fk_tick + 7000u, s.retry_tick);
	ASSERT_TRUE(s.slept);
	ASSERT_EQ(SWS_NEXT_SAMPLE_MS, s.slept_delta);
	fk_tick += 7000u;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

static void test_offset_wait_not_parked(void)
{
	Sm s = mk();

	TEST_START("Random-offset wait: no holdoff, no sleep while pending");
	fk_tick = 950000;
	arm_surface(&s, 300);
	s.last_tx_tick = fk_tick;               /* offset counted from now */
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_EQ(0, s.retry_tick);
	ASSERT_FALSE(s.slept);
	fk_tick += 300u;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

static void test_dive_clears_pending_and_holdoff(void)
{
	Sm s = mk();

	TEST_START("Dive while parked: pending + holdoff cleared, fresh window fires");
	fk_tick = 1000000;
	arm_surface(&s, 0);
	s.bat_ok = 0;
	step(&s);
	ASSERT_TRUE(s.retry_tick != 0u);
	go_underwater(&s);
	ASSERT_FALSE(s.pending);
	ASSERT_EQ(0, s.retry_tick);
	ASSERT_EQ(0, s.surface_since);
	s.bat_ok = 1;
	fk_tick += 5000u;
	arm_surface(&s, 0);
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

static void test_mid_sequence_veto_parks_retry(void)
{
	Sm s = mk();

	TEST_START("Mid-sequence veto: retry parked, periodic TX gated, loop sleeps");
	fk_tick = 1100000;
	arm_surface(&s, 0);
	step(&s);
	ASSERT_TRUE(s.dispatched);
	s.interval_ms = 30000u;

	/* periodic TX due, battery vetoes it */
	s.bat_ok = 0;
	fk_tick += 30000u;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_EQ(fk_tick + FIRST_TX_BAT_RETRY_MS, s.retry_tick);
	ASSERT_TRUE(s.slept);                   /* overdue TX no longer spins */
	ASSERT_EQ(SWS_NEXT_SAMPLE_MS, s.slept_delta);

	/* recovered but still parked */
	s.bat_ok = 1;
	fk_tick += 30000u;
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.slept);
	fk_tick += 30000u;
	step(&s);
	ASSERT_TRUE(s.dispatched);
	ASSERT_EQ(2, s.tx_count);
	ASSERT_EQ(0, s.retry_tick);
	TEST_PASS();
}

static void test_zero_tx_net_rearms(void)
{
	Sm s = mk();

	TEST_START("Zero-TX net: first TX re-armed after tx_seq_restart_s at surface");
	fk_tick = 1200000;
	s.surface_since = fk_tick;              /* first TX lost by some path */
	fk_tick += (uint32_t)s.seq_restart_s * 1000u - 1u;
	step(&s);
	ASSERT_FALSE(s.rearmed);
	fk_tick += 1u;
	step(&s);
	ASSERT_TRUE(s.rearmed);
	ASSERT_TRUE(s.pending);
	step(&s);
	ASSERT_TRUE(s.dispatched);
	TEST_PASS();
}

static void test_zero_tx_net_anchor_and_guards(void)
{
	Sm s = mk();
	Sm t = mk();
	Sm u = mk();
	Sm v = mk();

	TEST_START("Zero-TX net: later anchor wins, disabled/tx_count/pending guards");
	fk_tick = 3000000;
	s.surface_since = fk_tick - 2000000u;
	s.last_actual_tx_tick = fk_tick - 600000u;  /* last TX 10 min ago */
	step(&s);
	ASSERT_FALSE(s.rearmed);
	fk_tick += 600000u;
	step(&s);
	ASSERT_TRUE(s.rearmed);

	fk_tick = 20000000;
	t.seq_restart_s = 0;                    /* net disabled */
	t.surface_since = fk_tick - 10000000u;
	step(&t);
	ASSERT_FALSE(t.rearmed);

	u.surface_since = fk_tick - 10000000u;  /* a sequence ran */
	u.tx_count = 3;
	step(&u);
	ASSERT_FALSE(u.rearmed);

	v.surface_since = fk_tick - 10000000u;  /* re-armed attempt obeys backoff */
	v.backoff_blocked = 1;
	v.backoff_retry_s = 120;
	step(&v);
	ASSERT_TRUE(v.rearmed);
	step(&v);
	ASSERT_FALSE(v.dispatched);
	ASSERT_TRUE(v.pending);
	ASSERT_EQ(fk_tick + 120000u, v.retry_tick);
	TEST_PASS();
}

static void test_holdoff_wrap_safe(void)
{
	Sm s = mk();

	TEST_START("Holdoff across the 32-bit tick wrap, never the 0 sentinel");
	fk_tick = 0xFFFFFFF0u;
	arm_surface(&s, 0);
	s.backoff_blocked = 1;
	s.backoff_retry_s = 60;
	step(&s);
	ASSERT_TRUE(s.pending);
	ASSERT_TRUE(s.retry_tick != 0u);
	s.backoff_blocked = 0;
	fk_tick += 30000u;                  /* wrapped, still inside holdoff */
	step(&s);
	ASSERT_FALSE(s.dispatched);
	ASSERT_TRUE(s.slept);
	fk_tick += 30000u;
	step(&s);
	ASSERT_TRUE(s.dispatched);

	fk_tick = 0u - 2000u;               /* now + 2000 == 0 -> nudged to 1 */
	ASSERT_EQ(1, retry_at(2000u));
	TEST_PASS();
}

int main(void)
{
	printf("\n=== Vetoed TX is retried, never dropped ===\n");
	test_happy_path_first_pass_dispatch();
	test_battery_veto_keeps_window_and_retries();
	test_floating_animal_not_silenced();
	test_rate_and_backoff_retry_at_horizon();
	test_long_horizon_clamped_to_one_hour();
	test_gesture_defers_without_holdoff();
	test_hard_floor_retries_when_clear();
	test_cooldown_wait_parked();
	test_offset_wait_not_parked();
	test_dive_clears_pending_and_holdoff();
	test_mid_sequence_veto_parks_retry();
	test_zero_tx_net_rearms();
	test_zero_tx_net_anchor_and_guards();
	test_holdoff_wrap_safe();

	printf("\nResults: %d/%d passed\n", tests_passed, tests_run);
	return tests_failed ? 1 : 0;
}
