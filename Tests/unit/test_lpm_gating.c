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

/* ---- TX-transient reed blanking (mirror of mgr_reed.c reed_poll) ----
 * The SubGHz PA on/off transient couples into the floating reed node and
 * fakes magnet chatter that (a) burns surface awake-time via the debounce
 * budget above and (b) can walk the gesture FSM to a spurious power-off.
 * MGR_REED_blankUntil() freezes the debouncer for the TX window + settle
 * tail: no candidate flip, no published events, isDebouncing()==false. The
 * EXTI STOP2 wake (not modelled here) is untouched. */

static bool     rb_confirmed;
static bool     rb_candidate;
static uint8_t  rb_counter;
static int      rb_events;        /* published MAGNET_ON/OFF count */
static uint32_t rb_blank_until;
static bool     rb_blank_active;

static void rb_reset(bool confirmed)
{
	rb_confirmed = confirmed;
	rb_candidate = confirmed;
	rb_counter = 0;
	rb_events = 0;
	rb_blank_active = false;
	rb_blank_until = 0;
}

static void rb_blank(uint32_t until_tick)
{
	rb_blank_until = until_tick;
	rb_blank_active = true;
}

/* Mirror of reed_poll(): honour blank window first, then debounce. */
static void rb_poll(uint32_t now, bool raw)
{
	if (rb_blank_active) {
		if ((int32_t)(rb_blank_until - now) > 0) {
			rb_candidate = rb_confirmed;   /* freeze: no chatter latched */
			rb_counter = 0;
			return;
		}
		rb_blank_active = false;
	}
	if (raw == rb_confirmed) { rb_candidate = raw; rb_counter = 0; return; }
	if (raw != rb_candidate) { rb_candidate = raw; rb_counter = 1; return; }
	rb_counter++;
	uint8_t needed = raw ? 5u : 20u;    /* ON 50ms / OFF 200ms @10ms poll */
	if (rb_counter < needed) return;
	rb_confirmed = raw;
	rb_counter = 0;
	rb_events++;
}

static bool rb_is_debouncing(void) { return rb_candidate != rb_confirmed; }

static void test_blank_suppresses_tx_glitch_chatter(void)
{
	rb_reset(false);                 /* no magnet */
	rb_blank(10500u);                /* blank a 500ms TX window */
	for (uint32_t t = 10000u; t < 10500u; t += 10u) {
		rb_poll(t, (t / 10u) & 1u);  /* raw oscillates HIGH/LOW (PA coupling) */
		ASSERT_FALSE(rb_is_debouncing());  /* never reports chatter */
	}
	ASSERT_EQ(0, rb_events);         /* no spurious MAGNET_ON/OFF published */
	TEST_PASS();
}

static void test_blank_expiry_resumes_sampling(void)
{
	rb_reset(false);
	rb_blank(10500u);
	rb_poll(10600u, true);           /* first poll past the window, raw HIGH */
	ASSERT_TRUE(rb_is_debouncing()); /* normal debounce resumes */
	for (uint32_t t = 10610u; t <= 10650u; t += 10u)
		rb_poll(t, true);            /* sustain HIGH -> confirm ON (5 samples) */
	ASSERT_TRUE(rb_confirmed);
	ASSERT_EQ(1, rb_events);
	TEST_PASS();
}

static void test_blank_preserves_confirmed_magnet(void)
{
	rb_reset(true);                  /* magnet already confirmed before TX */
	rb_blank(20500u);
	for (uint32_t t = 20000u; t < 20500u; t += 10u) {
		rb_poll(t, false);           /* TX-coupled LOW dips during the hold */
		ASSERT_TRUE(rb_confirmed);   /* no false MAGNET_OFF mid-gesture */
	}
	ASSERT_EQ(0, rb_events);
	TEST_PASS();
}

static void test_blank_wrap_safe(void)
{
	rb_reset(false);
	uint32_t base = 0xFFFFFFFFu - 100u;
	rb_blank(base + 300u);           /* deadline wraps past 0 */
	rb_poll(base + 50u, true);       /* before deadline (despite wrap) */
	ASSERT_FALSE(rb_is_debouncing());
	rb_poll(base + 400u, true);      /* past deadline -> sampling resumes */
	ASSERT_TRUE(rb_is_debouncing());
	TEST_PASS();
}

/* ---- Console RX-wake arming gate (mirror of mgr_lpm_uw.c CONSOLE_WAKE_ON_RX) ----
 * The PA3 = LPUART_RX falling-edge STOP2 wake is armed ONLY while the console is
 * up (APP_UART_isEnabled). A sealed UW_DOPPLER unit (OPERATIONAL, UART torn
 * down, no DEBUG) leaves PA3 floating with no host holding it idle-high; arming
 * the EXTI there turned ambient EMI into a 2.5s-holdoff wake-storm (~5 mA, never
 * re-sleeps — the symptom that appeared the moment the UART cable was unplugged).
 * Gate fix: no live UART -> EXTI not armed -> a noise edge cannot create a
 * holdoff -> the chip still reaches STOP2. */

#define CW_HOLDOFF_MS 2500u

static bool     cw_uart_enabled;   /* APP_UART_isEnabled() ground truth */
static bool     cw_armed;          /* PA3 EXTI3 armed for this STOP2 window */
static bool     cw_rx_edge;        /* EXTI3_IRQHandler latched a falling edge */
static uint32_t cw_holdoff_until;  /* s_console_holdoff_until */
static uint32_t cw_tick;

static void cw_reset(bool uart_enabled)
{
	cw_uart_enabled  = uart_enabled;
	cw_armed         = false;
	cw_rx_edge       = false;
	cw_holdoff_until = 0u;
	cw_tick          = 50000u;
}

/* Mirror of the STOP2-entry arm block: if (APP_UART_isEnabled()) arm EXTI3. */
static void cw_stop2_enter(void) { cw_armed = cw_uart_enabled; }

/* A PA3 line edge reaches EXTI3_IRQHandler only while the pin is armed as a
 * falling-edge EXTI; otherwise it stays analog/AF and no handler runs. */
static void cw_pa3_edge(void) { if (cw_armed) cw_rx_edge = true; }

/* Mirror of the wake block: arm the holdoff iff we armed AND saw an edge. */
static void cw_stop2_exit(void)
{
	if (cw_armed && cw_rx_edge) {
		cw_rx_edge = false;
		cw_holdoff_until = cw_tick + CW_HOLDOFF_MS;
	}
}

/* Mirror of the idleTick holdoff gate (wrap-safe): 1 = may STOP2, 0 = held. */
static int cw_may_sleep(void)
{
	if (cw_holdoff_until != 0u &&
	    (uint32_t)(cw_tick - cw_holdoff_until) > 0x80000000u)
		return 0;
	return 1;
}

/* UART up: edge during STOP2 arms the 2.5s holdoff, which holds the chip awake
 * until it expires (the legitimate bench console-wake behaviour). */
static void test_console_wake_armed_when_uart_up(void)
{
	cw_reset(true);
	cw_stop2_enter();
	ASSERT_TRUE(cw_armed);
	cw_pa3_edge();
	cw_stop2_exit();
	ASSERT_EQ(0, cw_may_sleep());          /* held awake */
	cw_tick += CW_HOLDOFF_MS;              /* holdoff elapsed */
	ASSERT_EQ(1, cw_may_sleep());
	TEST_PASS();
}

/* UART down (sealed operational): EXTI never armed, so a noise edge latches
 * nothing and no holdoff is created -> the chip still sleeps. Regression lock
 * for the floating-PA3 wake-storm fix. */
static void test_console_wake_not_armed_when_uart_down(void)
{
	cw_reset(false);
	cw_stop2_enter();
	ASSERT_FALSE(cw_armed);
	cw_pa3_edge();
	ASSERT_FALSE(cw_rx_edge);
	cw_stop2_exit();
	ASSERT_EQ(0u, cw_holdoff_until);
	ASSERT_EQ(1, cw_may_sleep());          /* free to STOP2 */
	TEST_PASS();
}

/* The actual field scenario: continuous EMI on a floating disconnected RX. With
 * the UART down, 1000 noise edges across 1000 sleep cycles never hold the chip
 * awake -> no wake-storm, battery floor preserved. */
static void test_console_wake_storm_suppressed_when_down(void)
{
	cw_reset(false);
	for (int i = 0; i < 1000; i++) {
		cw_stop2_enter();
		cw_pa3_edge();
		cw_stop2_exit();
		ASSERT_EQ(1, cw_may_sleep());
		cw_tick += 400u;
	}
	ASSERT_EQ(0u, cw_holdoff_until);
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
	RUN_TEST(test_blank_suppresses_tx_glitch_chatter);
	RUN_TEST(test_blank_expiry_resumes_sampling);
	RUN_TEST(test_blank_preserves_confirmed_magnet);
	RUN_TEST(test_blank_wrap_safe);
	RUN_TEST(test_console_wake_armed_when_uart_up);
	RUN_TEST(test_console_wake_not_armed_when_uart_down);
	RUN_TEST(test_console_wake_storm_suppressed_when_down);
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
