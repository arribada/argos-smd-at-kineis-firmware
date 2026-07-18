/**
 * @file    test_tx_serialization.c
 * @brief   TX serialization: the UW_DOPPLER FSM must NEVER dispatch TX#2 until
 *          TX#1's TX_DONE (or the 10s deadman) returns it to MONITORING. This
 *          is the structural guard against the historical "TX#2 starts before
 *          TX#1 finished when the inter-TX timer is too short" bug.
 *
 * The only code that dispatches a TX (KNS_Q_push) lives in case MONITORING.
 * SURFACE_TX -> WAIT_TX_DONE, and WAIT_TX_DONE runs ONLY process_mac_events +
 * the timeout escape — it never re-enters the scheduler. This test mirrors the
 * TOP-LEVEL switch(state) (not a single MONITORING pass) and uses an
 * anti-tautology counter to prove the scheduler body does not even RUN while a
 * TX is in flight. It also checks the interval countdown is anchored at the
 * KNS_Q_push tick (last_tx_tick stamped at dispatch), not at TX_DONE.
 */

#include "test_framework.h"

#define TX_HARD_MIN_INTERVAL_MS   2000u
#define TIMEOUT_TX_DONE_MS       10000u

typedef enum { MONITORING, SURFACE_TX, WAIT_TX_DONE } St;

static uint32_t fk_tick;
static St       state;
static uint32_t state_enter_tick;
static uint32_t tx_count;
static uint32_t last_tx_tick;
static uint32_t last_actual_tx_tick;
static uint32_t current_interval_ms;
static bool     mac_tx_done;           /* injectable MAC event */
static int      sched_gate_evals;      /* anti-tautology: ++ at top of MONITORING body */

static void reset(uint32_t interval_ms)
{
	fk_tick = 100000;
	state = MONITORING;
	state_enter_tick = fk_tick;
	tx_count = 0;
	last_tx_tick = 0;
	last_actual_tx_tick = 0;
	current_interval_ms = interval_ms;
	mac_tx_done = false;
	sched_gate_evals = 0;
}

static void go_state(St s) { state = s; state_enter_tick = fk_tick; }

/* Mirror of the top-level switch(state). MONITORING is the ONLY branch that
 * evaluates the schedule or dispatches; WAIT_TX_DONE only escapes on the MAC
 * event or the 10s deadman. Returns 1 if a TX was dispatched this pass. */
static int run_pass(void)
{
	switch (state) {
	case MONITORING: {
		sched_gate_evals++;            /* the scheduler body actually ran */
		uint32_t since_last = fk_tick - last_tx_tick;
		int first_tx = (last_actual_tx_tick == 0u);
		int interval_ok = (last_tx_tick == 0u) || (since_last >= current_interval_ms);
		int hard_min_ok = first_tx ||
			((fk_tick - last_actual_tx_tick) >= TX_HARD_MIN_INTERVAL_MS);
		if (interval_ok && hard_min_ok) {
			/* dispatch (mirror KNS_Q_push site): stamp the anchor at PUSH time */
			last_tx_tick = (fk_tick == 0u) ? 1u : fk_tick;
			last_actual_tx_tick = last_tx_tick;
			tx_count++;
			go_state(WAIT_TX_DONE);
			return 1;
		}
		return 0;
	}
	case SURFACE_TX:
		go_state(WAIT_TX_DONE);
		return 0;
	case WAIT_TX_DONE:
		/* ONLY the MAC event or the 10s deadman — never the scheduler. */
		if (mac_tx_done) {
			mac_tx_done = false;
			go_state(MONITORING);
		} else if ((fk_tick - state_enter_tick) >= TIMEOUT_TX_DONE_MS) {
			go_state(MONITORING);      /* deadman recovery */
		}
		return 0;
	}
	return 0;
}

/* ---- Tests ---- */

/** TX#2 is not even considered until TX#1's TX_DONE returns to MONITORING;
 *  once it does, TX#2 is spaced by at least the interval AND the hard floor. */
void test_no_tx2_until_tx1_done(void)
{
	reset(10000);                      /* 10 s configured interval */
	ASSERT_EQ(1, run_pass());          /* TX#1 dispatches */
	ASSERT_EQ((int)WAIT_TX_DONE, (int)state);
	ASSERT_EQ(1, (long)tx_count);
	uint32_t push1 = last_tx_tick;
	int gate0 = sched_gate_evals;

	/* 9 s of passes with NO mac_tx_done (past interval+hard-min, under 10s). */
	for (int i = 0; i < 60; i++) {
		fk_tick += 150;
		ASSERT_EQ(0, run_pass());      /* nothing dispatches */
	}
	ASSERT_EQ((int)WAIT_TX_DONE, (int)state);   /* still in flight */
	ASSERT_EQ(1, (long)tx_count);               /* no TX#2 */
	ASSERT_EQ(gate0, sched_gate_evals);         /* scheduler body NEVER ran */

	/* TX_DONE arrives -> MONITORING. */
	mac_tx_done = true;
	fk_tick += 150;
	ASSERT_EQ(0, run_pass());
	ASSERT_EQ((int)MONITORING, (int)state);

	/* Advance until the interval elapses -> TX#2, spaced >= hard floor. */
	int fired = 0;
	for (int i = 0; i < 100 && !fired; i++) {
		fk_tick += 200;
		fired = run_pass();
	}
	ASSERT_TRUE(fired);
	ASSERT_EQ(2, (long)tx_count);
	ASSERT_TRUE((last_tx_tick - push1) >= TX_HARD_MIN_INTERVAL_MS);
	TEST_PASS();
}

/** The interval countdown is anchored at the PUSH tick, not at TX_DONE: a
 *  TX_DONE that arrives late must not reset the inter-TX clock. */
void test_interval_anchored_at_push_not_tx_done(void)
{
	reset(10000);
	ASSERT_EQ(1, run_pass());          /* TX#1 push at fk=100000 */
	uint32_t push1 = last_tx_tick;
	ASSERT_EQ(100000, (long)push1);

	/* TX_DONE arrives 5 s LATER than the push. */
	fk_tick += 5000;
	mac_tx_done = true;
	ASSERT_EQ(0, run_pass());
	ASSERT_EQ((int)MONITORING, (int)state);
	ASSERT_EQ((long)push1, (long)last_tx_tick);   /* anchor unchanged by TX_DONE */

	/* Interval measured from PUSH (100000): due at 110000, not 115000. */
	fk_tick = 109999; ASSERT_EQ(0, run_pass());
	fk_tick = 110000; ASSERT_EQ(1, run_pass());   /* fires exactly at push+interval */
	TEST_PASS();
}

/** A silent/ACK-less MAC (TX_DONE never arrives) is escaped by the 10s deadman,
 *  and only then can the next TX be considered — never during the wait. */
void test_silent_mac_escaped_by_deadman(void)
{
	reset(1000);                       /* short interval — irrelevant while gated */
	ASSERT_EQ(1, run_pass());          /* TX#1 */
	int gate0 = sched_gate_evals;

	/* 9.9 s, no mac event: still WAIT_TX_DONE, scheduler never runs. */
	for (int i = 0; i < 66; i++) {
		fk_tick += 150;
		ASSERT_EQ(0, run_pass());
	}
	ASSERT_EQ((int)WAIT_TX_DONE, (int)state);
	ASSERT_EQ(gate0, sched_gate_evals);

	/* Past 10s -> deadman returns to MONITORING. */
	fk_tick += 300;                    /* total > 10000 since enter */
	ASSERT_EQ(0, run_pass());
	ASSERT_EQ((int)MONITORING, (int)state);
	TEST_PASS();
}

/** Wrap-safe: the whole serialization holds across the 49.7-day tick wrap. */
void test_serialization_wrap_safe(void)
{
	reset(10000);
	fk_tick = 0xFFFFFFF0u;
	state_enter_tick = fk_tick;
	last_tx_tick = 0; last_actual_tx_tick = 0;
	ASSERT_EQ(1, run_pass());          /* TX#1 across the wrap boundary */
	uint32_t push1 = last_tx_tick;
	int gate0 = sched_gate_evals;
	for (int i = 0; i < 40; i++) {     /* 8 s of passes over the wrap */
		fk_tick += 200;                /* wraps through 0 */
		ASSERT_EQ(0, run_pass());
	}
	ASSERT_EQ((int)WAIT_TX_DONE, (int)state);
	ASSERT_EQ(gate0, sched_gate_evals);
	mac_tx_done = true; fk_tick += 200; run_pass();
	int fired = 0;
	for (int i = 0; i < 100 && !fired; i++) { fk_tick += 300; fired = run_pass(); }
	ASSERT_TRUE(fired);
	ASSERT_TRUE((uint32_t)(last_tx_tick - push1) >= TX_HARD_MIN_INTERVAL_MS);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("UW_DOPPLER TX serialization (no TX#2 before TX#1 done)");

	RUN_TEST(test_no_tx2_until_tx1_done);
	RUN_TEST(test_interval_anchored_at_push_not_tx_done);
	RUN_TEST(test_silent_mac_escaped_by_deadman);
	RUN_TEST(test_serialization_wrap_safe);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
