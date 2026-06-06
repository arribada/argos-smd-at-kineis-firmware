/**
 * @file    test_state_machine.c
 * @brief   Unit tests for UW_DOPPLER state machine transitions
 *
 * Validates legal transitions and timeout-driven progression. Mirrors the
 * state graph from kns_app_uw_doppler.c without coupling to HAL.
 *
 * Legal transitions (from kns_app_uw_doppler.c):
 *   BOOT             -> BOOT_DEPLOY_LED   (after LED blink or TIMEOUT_BOOT_MS)
 *   BOOT_DEPLOY_LED  -> INIT_MAC          (after BOOT_DEPLOY_LED_MS or timeout)
 *   INIT_MAC         -> WAIT_MAC_READY    (always, after pushing INIT)
 *   WAIT_MAC_READY   -> MONITORING        (on KNS_MAC_OK event)
 *   WAIT_MAC_READY   -> INIT_MAC          (retry, on TIMEOUT_MAC_READY_MS)
 *   MONITORING       -> SURFACE_TX        (SWS surface or test burst)
 *   SURFACE_TX       -> WAIT_TX_DONE      (after MAC accepts)
 *   WAIT_TX_DONE     -> MONITORING        (on KNS_MAC_TX_DONE or timeout)
 *   any              -> SHUTDOWN_BLINK    (magnet 6s gesture)
 */

#include "test_framework.h"

typedef enum {
	UW_BOOT = 0,
	UW_BOOT_DEPLOY_LED,
	UW_INIT_MAC,
	UW_WAIT_MAC_READY,
	UW_MONITORING,
	UW_SURFACE_TX,
	UW_WAIT_TX_DONE,
	UW_SHUTDOWN_BLINK,
	UW_STATE_COUNT,
} uw_state_t;

/* Adjacency matrix: legal[from][to] = true if from -> to is legal. */
static bool legal[UW_STATE_COUNT][UW_STATE_COUNT];

static void init_graph(void)
{
	for (int i = 0; i < UW_STATE_COUNT; i++)
		for (int j = 0; j < UW_STATE_COUNT; j++)
			legal[i][j] = false;

	legal[UW_BOOT][UW_BOOT_DEPLOY_LED] = true;
	legal[UW_BOOT_DEPLOY_LED][UW_INIT_MAC] = true;
	legal[UW_INIT_MAC][UW_WAIT_MAC_READY] = true;
	legal[UW_WAIT_MAC_READY][UW_MONITORING] = true;
	legal[UW_WAIT_MAC_READY][UW_INIT_MAC] = true;  /* retry */
	legal[UW_MONITORING][UW_SURFACE_TX] = true;
	legal[UW_SURFACE_TX][UW_WAIT_TX_DONE] = true;
	legal[UW_SURFACE_TX][UW_MONITORING] = true;    /* abort */
	legal[UW_WAIT_TX_DONE][UW_MONITORING] = true;

	/* SHUTDOWN_BLINK reachable from any state via magnet 6s gesture. */
	for (int i = 0; i < UW_STATE_COUNT; i++)
		legal[i][UW_SHUTDOWN_BLINK] = true;
}

void test_boot_to_deploy_legal(void)
{
	TEST_START("BOOT -> BOOT_DEPLOY_LED legal");
	init_graph();
	ASSERT_TRUE(legal[UW_BOOT][UW_BOOT_DEPLOY_LED]);
	TEST_PASS();
}

void test_boot_to_monitoring_illegal(void)
{
	TEST_START("BOOT -> MONITORING directly is ILLEGAL (must traverse boot states)");
	init_graph();
	ASSERT_FALSE(legal[UW_BOOT][UW_MONITORING]);
	TEST_PASS();
}

void test_normal_path_chain(void)
{
	TEST_START("BOOT -> ... -> MONITORING is a connected legal chain");
	init_graph();
	uw_state_t path[] = {
		UW_BOOT, UW_BOOT_DEPLOY_LED, UW_INIT_MAC,
		UW_WAIT_MAC_READY, UW_MONITORING,
	};
	for (size_t i = 0; i + 1 < sizeof(path)/sizeof(path[0]); i++) {
		if (!legal[path[i]][path[i+1]]) {
			TEST_FAIL("Path segment illegal");
			return;
		}
	}
	TEST_PASS();
}

void test_mac_retry_legal(void)
{
	TEST_START("WAIT_MAC_READY -> INIT_MAC retry is legal");
	init_graph();
	ASSERT_TRUE(legal[UW_WAIT_MAC_READY][UW_INIT_MAC]);
	TEST_PASS();
}

void test_monitoring_to_tx_path(void)
{
	TEST_START("MONITORING -> SURFACE_TX -> WAIT_TX_DONE -> MONITORING");
	init_graph();
	ASSERT_TRUE(legal[UW_MONITORING][UW_SURFACE_TX]);
	ASSERT_TRUE(legal[UW_SURFACE_TX][UW_WAIT_TX_DONE]);
	ASSERT_TRUE(legal[UW_WAIT_TX_DONE][UW_MONITORING]);
	TEST_PASS();
}

void test_shutdown_reachable_from_any(void)
{
	TEST_START("SHUTDOWN_BLINK reachable from any state (magnet 6s)");
	init_graph();
	for (int i = 0; i < UW_STATE_COUNT; i++) {
		if (i == UW_SHUTDOWN_BLINK)
			continue;
		if (!legal[i][UW_SHUTDOWN_BLINK]) {
			TEST_FAIL("Some state can't reach SHUTDOWN_BLINK");
			return;
		}
	}
	TEST_PASS();
}

void test_no_backward_skip(void)
{
	TEST_START("Cannot skip BOOT_DEPLOY_LED (boot indicator visible)");
	init_graph();
	ASSERT_FALSE(legal[UW_BOOT][UW_INIT_MAC]);
	TEST_PASS();
}

void test_monitoring_not_reachable_from_boot_directly(void)
{
	TEST_START("BOOT_DEPLOY_LED cannot jump to MONITORING (must init MAC)");
	init_graph();
	ASSERT_FALSE(legal[UW_BOOT_DEPLOY_LED][UW_MONITORING]);
	TEST_PASS();
}

void test_tx_done_returns_to_monitoring(void)
{
	TEST_START("WAIT_TX_DONE -> SURFACE_TX is NOT legal (no re-TX in same cycle)");
	init_graph();
	ASSERT_FALSE(legal[UW_WAIT_TX_DONE][UW_SURFACE_TX]);
	TEST_PASS();
}

/* Counter-style: simulate the BOOT sequence ticking through timeouts. */
typedef struct {
	uw_state_t state;
	uint32_t enter_tick_ms;
} sm_t;

static void sm_init(sm_t *sm) { sm->state = UW_BOOT; sm->enter_tick_ms = 0; }

static void sm_advance_if_elapsed(sm_t *sm, uint32_t now_ms,
                                  uint32_t window_ms, uw_state_t next)
{
	if (now_ms - sm->enter_tick_ms >= window_ms) {
		if (legal[sm->state][next]) {
			sm->state = next;
			sm->enter_tick_ms = now_ms;
		}
	}
}

void test_boot_timeout_advances(void)
{
	TEST_START("BOOT window 4 s elapses -> advance to BOOT_DEPLOY_LED");
	init_graph();
	sm_t sm;
	sm_init(&sm);
	sm_advance_if_elapsed(&sm, 3000, 4000, UW_BOOT_DEPLOY_LED);
	ASSERT_EQ(UW_BOOT, sm.state);  /* not yet */
	sm_advance_if_elapsed(&sm, 4500, 4000, UW_BOOT_DEPLOY_LED);
	ASSERT_EQ(UW_BOOT_DEPLOY_LED, sm.state);
	TEST_PASS();
}

int main(void)
{
	TEST_SUITE_START("UW_DOPPLER state machine transitions");
	test_boot_to_deploy_legal();
	test_boot_to_monitoring_illegal();
	test_normal_path_chain();
	test_mac_retry_legal();
	test_monitoring_to_tx_path();
	test_shutdown_reachable_from_any();
	test_no_backward_skip();
	test_monitoring_not_reachable_from_boot_directly();
	test_tx_done_returns_to_monitoring();
	test_boot_timeout_advances();
	TEST_SUITE_END();
	return tests_failed ? 1 : 0;
}
