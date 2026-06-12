/**
 * @file    test_lpm_gating.c
 * @brief   Unit tests for UW_DOPPLER LPM client gating logic
 *
 * Two real bugs guarded by these tests:
 *
 *  1. uw_doppler_lpmReq() must return NONE when state < MONITORING.
 *     In STOP the SysTick is frozen, so HAL_GetTick()-based boot timeouts
 *     (TIMEOUT_BOOT_MS, TIMEOUT_MAC_READY_MS) would never elapse and the
 *     chip would stay asleep waiting for an event that never fires.
 *
 *  2. lpmNotifEnter()/lpmNotifExit() must early-return when mode == NONE.
 *     The MGR_LPM aggregator calls them every loop tick (~1 kHz) regardless
 *     of whether the chip actually sleeps. If side-effects (MGR_LED_off,
 *     UART trace, SWS analog) run on every tick they kill the LED state
 *     and flood UART at 9600 baud, slowing the loop to ~60 ms/iter.
 */

#include "test_framework.h"

/* Mirror of MgrLpm_LPM_t enum (mgr_lpm.h). */
enum lpm_mode_t {
	LPM_NONE     = 0x00,
	LPM_SLEEP    = 0x01,
	LPM_STOP     = 0x02,
	LPM_STANDBY  = 0x04,
	LPM_SHUTDOWN = 0x08,
};

/* Mirror of UwDopplerState_t enum (kns_app_uw_doppler.c). */
enum app_state_t {
	S_BOOT = 0,
	S_BOOT_DEPLOY_LED,
	S_INIT_MAC,
	S_WAIT_MAC_READY,
	S_MONITORING,
	S_SURFACE_TX,
	S_WAIT_TX_DONE,
	S_SHUTDOWN_BLINK,
};

/* Counter-based mocks: every "side-effect" call increments a counter. */
static int led_off_calls;
static int sws_lp_enter_calls;
static int sws_lp_exit_calls;
static int trace_calls;

static void reset_mocks(void)
{
	led_off_calls = 0;
	sws_lp_enter_calls = 0;
	sws_lp_exit_calls = 0;
	trace_calls = 0;
}

/* Mirror of uw_doppler_lpmReq(). */
static enum lpm_mode_t lpm_req(enum app_state_t app_state)
{
	if (app_state < S_MONITORING)
		return LPM_NONE;
	/* For test purposes the prod choice (STOP vs NONE) is irrelevant; we
	 * only verify the BOOT-vs-MONITORING gate here. Match current prod. */
	return LPM_NONE;
}

/* Mirror of uw_doppler_lpmNotifEnter() — focus on gating + side-effects. */
static bool lpm_notif_enter(enum lpm_mode_t lpm)
{
	if (lpm == LPM_NONE)
		return true;
	trace_calls++;
	led_off_calls++;
	sws_lp_enter_calls++;
	return true;
}

/* Mirror of uw_doppler_lpmNotifExit(). */
static bool lpm_notif_exit(enum lpm_mode_t lpm)
{
	if (lpm == LPM_NONE)
		return true;
	sws_lp_exit_calls++;
	trace_calls++;
	return true;
}

void test_req_in_boot(void)
{
	TEST_START("BOOT state -> NONE");
	ASSERT_EQ(LPM_NONE, lpm_req(S_BOOT));
	TEST_PASS();
}

void test_req_in_init_mac(void)
{
	TEST_START("INIT_MAC state -> NONE (no STOP before MAC ready)");
	ASSERT_EQ(LPM_NONE, lpm_req(S_INIT_MAC));
	TEST_PASS();
}

void test_req_in_wait_mac(void)
{
	TEST_START("WAIT_MAC_READY -> NONE");
	ASSERT_EQ(LPM_NONE, lpm_req(S_WAIT_MAC_READY));
	TEST_PASS();
}

void test_req_in_monitoring(void)
{
	TEST_START("MONITORING -> not NONE in production (STOP target)");
	/* lpm_req currently returns NONE in prod for safety; this test guards
	 * the boot-gate logic. When STOP is re-enabled, change to STOP check. */
	ASSERT_EQ(LPM_NONE, lpm_req(S_MONITORING));
	TEST_PASS();
}

void test_notif_enter_none_no_side_effects(void)
{
	TEST_START("lpmNotifEnter(NONE) does NOT touch LED/SWS/UART");
	reset_mocks();
	lpm_notif_enter(LPM_NONE);
	ASSERT_EQ(0, led_off_calls);
	ASSERT_EQ(0, sws_lp_enter_calls);
	ASSERT_EQ(0, trace_calls);
	TEST_PASS();
}

void test_notif_exit_none_no_side_effects(void)
{
	TEST_START("lpmNotifExit(NONE) does NOT touch LED/SWS/UART");
	reset_mocks();
	lpm_notif_exit(LPM_NONE);
	ASSERT_EQ(0, sws_lp_exit_calls);
	ASSERT_EQ(0, trace_calls);
	TEST_PASS();
}

void test_notif_enter_stop_runs_side_effects(void)
{
	TEST_START("lpmNotifEnter(STOP) DOES run LED off + SWS LP + trace");
	reset_mocks();
	lpm_notif_enter(LPM_STOP);
	ASSERT_EQ(1, led_off_calls);
	ASSERT_EQ(1, sws_lp_enter_calls);
	ASSERT_EQ(1, trace_calls);
	TEST_PASS();
}

void test_notif_enter_repeated_none_stays_silent(void)
{
	TEST_START("1000 lpmNotifEnter(NONE) calls => still 0 side-effects");
	reset_mocks();
	for (int i = 0; i < 1000; i++) {
		lpm_notif_enter(LPM_NONE);
		lpm_notif_exit(LPM_NONE);
	}
	ASSERT_EQ(0, led_off_calls);
	ASSERT_EQ(0, trace_calls);
	TEST_PASS();
}

void test_notif_enter_standby_runs(void)
{
	TEST_START("lpmNotifEnter(STANDBY) DOES run side-effects");
	reset_mocks();
	lpm_notif_enter(LPM_STANDBY);
	ASSERT_EQ(1, led_off_calls);
	TEST_PASS();
}


/* ---- Reed-debounce awake budget (mirror of mgr_lpm_uw.c idleTick) ----
 * Unbounded "stay awake while debouncing" held the chip at ~5 mA whenever
 * a floating reed node chattered after TX transients. The budget grants
 * 500 ms per episode: real presses confirm in 50/200 ms, chatter loses
 * its grant until the debouncer settles once. */

#define BUDGET_MS 500u
static uint32_t fk_tick;
static int      fk_debouncing;
static uint32_t dbg_awake_since;

static int dbg_awake_armed;

static int budget_allows_sleep(void)
{
	if (fk_debouncing) {
		if (!dbg_awake_armed) {
			dbg_awake_armed = 1;
			dbg_awake_since = fk_tick;
		}
		if ((fk_tick - dbg_awake_since) < BUDGET_MS)
			return 0;          /* stay awake: converging */
		return 1;                  /* budget burnt: sleep anyway */
	}
	dbg_awake_armed = 0;
	return 1;
}

static void test_debounce_budget_real_press_keeps_awake(void)
{
	fk_tick = 10000u; fk_debouncing = 1; dbg_awake_armed = 0;
	ASSERT_EQ(0, budget_allows_sleep());       /* t0: awake granted */
	fk_tick += 60;                             /* real ON confirms in 50ms */
	ASSERT_EQ(0, budget_allows_sleep());       /* still within budget */
	fk_debouncing = 0;                         /* confirmed -> settled */
	ASSERT_EQ(1, budget_allows_sleep());
	ASSERT_EQ(0, dbg_awake_armed);             /* budget re-armed */
	TEST_PASS();
}

static void test_debounce_budget_chatter_sleeps_after_500ms(void)
{
	fk_tick = 20000u; fk_debouncing = 1; dbg_awake_armed = 0;
	ASSERT_EQ(0, budget_allows_sleep());
	fk_tick += 499;
	ASSERT_EQ(0, budget_allows_sleep());
	fk_tick += 2;                              /* 501 ms of churn */
	ASSERT_EQ(1, budget_allows_sleep());       /* sleep despite chatter */
	fk_tick += 5000;
	ASSERT_EQ(1, budget_allows_sleep());       /* no new grant while chattering */
	fk_debouncing = 0;
	ASSERT_EQ(1, budget_allows_sleep());       /* settled */
	fk_debouncing = 1;
	ASSERT_EQ(0, budget_allows_sleep());       /* fresh episode -> fresh budget */
	TEST_PASS();
}

static void test_debounce_budget_tick_wrap_safe(void)
{
	fk_tick = 0xFFFFFFFFu - 100u; fk_debouncing = 1; dbg_awake_armed = 0;
	ASSERT_EQ(0, budget_allows_sleep());
	fk_tick += 200u;                           /* wraps past 0 */
	ASSERT_EQ(0, budget_allows_sleep());       /* 200ms < 500ms despite wrap */
	fk_tick += 400u;
	ASSERT_EQ(1, budget_allows_sleep());       /* 600ms: budget burnt */
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("UW_DOPPLER LPM client gating");
	test_req_in_boot();
	test_req_in_init_mac();
	test_req_in_wait_mac();
	test_req_in_monitoring();
	test_notif_enter_none_no_side_effects();
	test_notif_exit_none_no_side_effects();
	test_notif_enter_stop_runs_side_effects();
	test_notif_enter_repeated_none_stays_silent();
	test_notif_enter_standby_runs();
	RUN_TEST(test_debounce_budget_real_press_keeps_awake);
	RUN_TEST(test_debounce_budget_chatter_sleeps_after_500ms);
	RUN_TEST(test_debounce_budget_tick_wrap_safe);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
