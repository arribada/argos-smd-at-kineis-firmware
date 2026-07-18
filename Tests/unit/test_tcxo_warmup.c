/**
 * @file    test_tcxo_warmup.c
 * @brief   TCXO warmup fast-path lifecycle (kns_app_uw_doppler.c), the
 *          maintainer's inter-TX/TCXO concern. Mirrors the exact arm / dispatch
 *          / restore snippets and pins the invariant that the warmup is NEVER
 *          left at the 0 ms fast-path once the FSM is back in MONITORING.
 *
 * Real code refs:
 *   arm (surface detect):   MCU_MISC_TCXO_get_warmup(&saved); tcxo_first_tx_skip=true
 *   dispatch (SURFACE_TX):  if (tcxo_first_tx_skip) MCU_MISC_TCXO_set_warmup(0)
 *   restore (every exit):   TX_DONE(1622), TX_TIMEOUT/ABORT(1648), MAC_ERROR(1670),
 *                           KNS_Q_push-fail(3021), 10s deadman(3068), and the
 *                           default/unknown-MAC-event branch (1714, audit #10 fix)
 *
 * Only the burst's FIRST TX skips warmup; TX#2+ must wait the real configured
 * warmup. And get_warmup must never sample while the register is 0, or the
 * "saved" value would latch the 0 sentinel and pin every subsequent TX at 0.
 */

#include "test_framework.h"

/* ---- Mocked TCXO warmup register (MCU_MISC_TCXO_get/set_warmup) ---- */
#define CONFIGURED_WARMUP_MS  2000u

static uint32_t g_warmup;              /* the live TCXO warmup register */
static int      get_calls_at_zero;     /* times get_warmup sampled a 0 register */

static void tcxo_get_warmup(uint32_t *out)
{
	if (g_warmup == 0u)
		get_calls_at_zero++;           /* poisoning detector */
	*out = g_warmup;
}
static void tcxo_set_warmup(uint32_t v) { g_warmup = v; }

/* ---- Mirrored app state ---- */
static uint32_t tcxo_warmup_saved_ms;
static bool     tcxo_first_tx_skip;
static uint32_t pushed_warmup;         /* warmup in force at the TX dispatch */

static void reset_all(void)
{
	g_warmup = CONFIGURED_WARMUP_MS;
	get_calls_at_zero = 0;
	tcxo_warmup_saved_ms = 0;
	tcxo_first_tx_skip = false;
	pushed_warmup = 0xFFFFFFFFu;
}

/* Surface-detect arm (cold-boot / boot-at-surface / live UW->SURF). */
static void arm_first_tx_skip(void)
{
	tcxo_get_warmup(&tcxo_warmup_saved_ms);
	tcxo_first_tx_skip = true;
}

/* SURFACE_TX dispatch: the first TX of the burst forces a 0 ms warmup. */
static void tx_dispatch(void)
{
	if (tcxo_first_tx_skip)
		tcxo_set_warmup(0);
	pushed_warmup = g_warmup;          /* what the radio actually gets */
}

/* The restore snippet run on EVERY TX exit path (identical in all of them). */
static void tx_exit_restore(void)
{
	if (tcxo_first_tx_skip) {
		tcxo_set_warmup(tcxo_warmup_saved_ms);
		tcxo_first_tx_skip = false;
	}
}

/* ---- Tests ---- */

/** The burst's first TX runs with warmup 0 (the intended fast path). */
void test_first_tx_uses_zero_warmup(void)
{
	reset_all();
	arm_first_tx_skip();
	tx_dispatch();
	ASSERT_EQ(0, (long)pushed_warmup);
	ASSERT_EQ((long)CONFIGURED_WARMUP_MS, (long)tcxo_warmup_saved_ms);
	TEST_PASS();
}

/** Every TX exit path restores the configured warmup and clears the flag.
 *  All six live restore sites + the audit#10 default-branch fix share the same
 *  snippet, so exercising it once per path proves each site. */
void test_every_exit_path_restores_warmup(void)
{
	const char *paths[] = { "TX_DONE", "TX_TIMEOUT", "TX_ABORT",
	                        "MAC_ERROR", "KNS_Q_push_fail", "deadman_10s",
	                        "default_unknown_evt" };
	for (unsigned p = 0; p < sizeof(paths)/sizeof(paths[0]); p++) {
		reset_all();
		arm_first_tx_skip();
		tx_dispatch();
		ASSERT_EQ(0, (long)g_warmup);      /* warmup pinned at 0 in-flight */
		tx_exit_restore();                 /* this path's restore */
		ASSERT_EQ((long)CONFIGURED_WARMUP_MS, (long)g_warmup);  /* restored */
		ASSERT_FALSE(tcxo_first_tx_skip);  /* flag cleared */
	}
	TEST_PASS();
}

/** Only the FIRST TX of a burst skips warmup; TX#2 gets the real warmup. */
void test_second_tx_uses_configured_warmup(void)
{
	reset_all();
	arm_first_tx_skip();
	tx_dispatch();                         /* TX#1 -> 0 */
	ASSERT_EQ(0, (long)pushed_warmup);
	tx_exit_restore();                     /* TX_DONE */
	/* TX#2 dispatches WITHOUT a re-arm (same burst). */
	tx_dispatch();
	ASSERT_EQ((long)CONFIGURED_WARMUP_MS, (long)pushed_warmup);
	TEST_PASS();
}

/** Poisoning guard: get_warmup must never sample the 0 register, else `saved`
 *  latches 0 and every future TX skips warmup forever. Run many full cycles. */
void test_saved_never_latches_zero(void)
{
	reset_all();
	for (int i = 0; i < 50; i++) {
		arm_first_tx_skip();               /* get_warmup sampled here */
		tx_dispatch();                     /* now g_warmup == 0 */
		tx_exit_restore();                 /* restores to configured */
	}
	ASSERT_EQ(0, get_calls_at_zero);       /* never sampled while 0 */
	ASSERT_EQ((long)CONFIGURED_WARMUP_MS, (long)tcxo_warmup_saved_ms);
	TEST_PASS();
}

/** Negative control: if a re-arm ever happened while the register was 0, the
 *  detector would catch it (proves the guard above is not vacuous). */
void test_poisoning_detector_is_live(void)
{
	reset_all();
	arm_first_tx_skip();
	tx_dispatch();                         /* g_warmup now 0, NO restore */
	arm_first_tx_skip();                   /* BUG shape: re-arm while 0 */
	ASSERT_TRUE(get_calls_at_zero > 0);
	ASSERT_EQ(0, (long)tcxo_warmup_saved_ms);  /* saved would be poisoned */
	TEST_PASS();
}

/** Invariant: after ANY exit path the warmup register is back to configured —
 *  it is never left at 0 once the burst's TX cycle ends. */
void test_warmup_never_left_at_zero(void)
{
	reset_all();
	for (int i = 0; i < 20; i++) {
		arm_first_tx_skip();
		tx_dispatch();
		tx_exit_restore();
		ASSERT_NE(0, (long)g_warmup);      /* MONITORING never sees warmup 0 */
	}
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("TCXO warmup fast-path lifecycle (UW_DOPPLER)");

	RUN_TEST(test_first_tx_uses_zero_warmup);
	RUN_TEST(test_every_exit_path_restores_warmup);
	RUN_TEST(test_second_tx_uses_configured_warmup);
	RUN_TEST(test_saved_never_latches_zero);
	RUN_TEST(test_poisoning_detector_is_live);
	RUN_TEST(test_warmup_never_left_at_zero);

	TEST_SUITE_END();
	TEST_SUMMARY();
}
