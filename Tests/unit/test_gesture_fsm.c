/**
 * @file    test_gesture_fsm.c
 * @brief   Unit tests for MGR_GESTURE 2-gesture magnet FSM
 *
 * Tests the protocol decisions WITHOUT pulling in HAL/MGR_REED/MGR_LED so the
 * tests stay pure logic. We mirror the FSM transition rules:
 *   - long hold (3s)  → propose mode switch
 *   - shutdown hold (6s) → propose SHUTDOWN
 *   - 2s confirmation window: needs OFF→ON toggle
 *   - no toggle in window → revert (no event)
 *   - toggle in window → CONFIRMED event
 */

#include "test_framework.h"

#define LONG_HOLD_MS        3000u
#define SHUTDOWN_HOLD_MS    6000u
#define CONFIRM_WINDOW_MS   2000u

typedef enum { MODE_OP = 1, MODE_CFG = 2, MODE_OFF = 0 } Mode_t;
typedef enum {
	EVT_NONE = 0,
	EVT_ENTER_OPERATIONAL,
	EVT_ENTER_CONFIG,
	EVT_REQUEST_SHUTDOWN,
} Event_t;
typedef enum {
	FSM_IDLE = 0,
	FSM_HOLDING,
	FSM_ASKING_MODE_SWITCH,
	FSM_ASKING_SHUTDOWN,
} Fsm_t;

typedef struct {
	Mode_t mode;
	Fsm_t  fsm;
	uint32_t press_tick;
	uint32_t window_start;
	bool saw_off;
	Event_t pending;
} G_t;

static void g_init(G_t *g, Mode_t initial)
{
	g->mode = initial;
	g->fsm = FSM_IDLE;
	g->press_tick = 0;
	g->window_start = 0;
	g->saw_off = false;
	g->pending = EVT_NONE;
}

/* One tick step. Caller supplies the current time, the magnet state, and
 * the optional edge event (raw from MGR_REED). */
typedef enum { ED_NONE, ED_ON, ED_OFF } Edge_t;

static void g_tick(G_t *g, uint32_t now, bool magnet_present, Edge_t edge)
{
	if (g->fsm == FSM_IDLE) {
		if (edge == ED_ON) {
			g->fsm = FSM_HOLDING;
			g->press_tick = now;
		}
		return;
	}
	if (g->fsm == FSM_HOLDING) {
		if (!magnet_present) {
			g->fsm = FSM_IDLE;
			return;
		}
		uint32_t elapsed = now - g->press_tick;
		if (elapsed >= SHUTDOWN_HOLD_MS) {
			g->fsm = FSM_ASKING_SHUTDOWN;
			g->window_start = now;
			g->saw_off = false;
			return;
		}
		if (elapsed >= LONG_HOLD_MS) {
			g->fsm = FSM_ASKING_MODE_SWITCH;
			g->window_start = now;
			g->saw_off = false;
			return;
		}
		return;
	}
	/* ASKING_* states */
	if (edge == ED_OFF) g->saw_off = true;
	if (edge == ED_ON && g->saw_off) {
		if (g->fsm == FSM_ASKING_MODE_SWITCH) {
			Mode_t target = (g->mode == MODE_OP) ? MODE_CFG : MODE_OP;
			g->mode = target;
			g->pending = (target == MODE_CFG) ? EVT_ENTER_CONFIG : EVT_ENTER_OPERATIONAL;
		} else {
			g->pending = EVT_REQUEST_SHUTDOWN;
		}
		g->fsm = FSM_IDLE;
		g->saw_off = false;
		return;
	}
	if ((now - g->window_start) >= CONFIRM_WINDOW_MS) {
		g->fsm = FSM_IDLE;
		g->saw_off = false;
	}
}

/* ---- Tests ---- */

void test_short_press_no_action(void)
{
	TEST_START("Short press (500ms) does nothing");
	G_t g; g_init(&g, MODE_OP);
	g_tick(&g, 0, true, ED_ON);
	g_tick(&g, 500, true, ED_NONE);
	g_tick(&g, 600, false, ED_OFF);
	ASSERT_EQ(EVT_NONE, g.pending);
	ASSERT_EQ(MODE_OP, g.mode);
	TEST_PASS();
}

void test_op_to_config_with_confirm(void)
{
	TEST_START("OPERATIONAL: 3s hold + OFF→ON within 2s → CONFIG");
	G_t g; g_init(&g, MODE_OP);
	g_tick(&g, 0,    true, ED_ON);
	g_tick(&g, 3000, true, ED_NONE);  /* enters ASKING_MODE_SWITCH */
	ASSERT_EQ(FSM_ASKING_MODE_SWITCH, g.fsm);
	g_tick(&g, 3500, false, ED_OFF);
	g_tick(&g, 4000, true,  ED_ON);   /* confirm */
	ASSERT_EQ(EVT_ENTER_CONFIG, g.pending);
	ASSERT_EQ(MODE_CFG, g.mode);
	TEST_PASS();
}

void test_op_to_config_no_confirm_reverts(void)
{
	TEST_START("OPERATIONAL: 3s hold but no toggle in 2s → revert");
	G_t g; g_init(&g, MODE_OP);
	g_tick(&g, 0,    true, ED_ON);
	g_tick(&g, 3000, true, ED_NONE);
	ASSERT_EQ(FSM_ASKING_MODE_SWITCH, g.fsm);
	g_tick(&g, 5001, true, ED_NONE);  /* window expired */
	ASSERT_EQ(FSM_IDLE, g.fsm);
	ASSERT_EQ(MODE_OP, g.mode);
	ASSERT_EQ(EVT_NONE, g.pending);
	TEST_PASS();
}

void test_config_to_op(void)
{
	TEST_START("CONFIG: 3s hold + confirm → back to OPERATIONAL");
	G_t g; g_init(&g, MODE_CFG);
	g_tick(&g, 0,    true, ED_ON);
	g_tick(&g, 3000, true, ED_NONE);
	g_tick(&g, 3500, false, ED_OFF);
	g_tick(&g, 4000, true,  ED_ON);
	ASSERT_EQ(EVT_ENTER_OPERATIONAL, g.pending);
	ASSERT_EQ(MODE_OP, g.mode);
	TEST_PASS();
}

void test_shutdown_from_op(void)
{
	TEST_START("OPERATIONAL: 6s hold + confirm → SHUTDOWN request");
	G_t g; g_init(&g, MODE_OP);
	g_tick(&g, 0,    true, ED_ON);
	g_tick(&g, 6000, true, ED_NONE);
	ASSERT_EQ(FSM_ASKING_SHUTDOWN, g.fsm);
	g_tick(&g, 6500, false, ED_OFF);
	g_tick(&g, 7000, true,  ED_ON);
	ASSERT_EQ(EVT_REQUEST_SHUTDOWN, g.pending);
	TEST_PASS();
}

void test_shutdown_from_config(void)
{
	TEST_START("CONFIG: 6s hold + confirm → SHUTDOWN request");
	G_t g; g_init(&g, MODE_CFG);
	g_tick(&g, 0,    true, ED_ON);
	g_tick(&g, 6000, true, ED_NONE);
	g_tick(&g, 6500, false, ED_OFF);
	g_tick(&g, 7000, true,  ED_ON);
	ASSERT_EQ(EVT_REQUEST_SHUTDOWN, g.pending);
	TEST_PASS();
}

void test_shutdown_no_confirm_reverts(void)
{
	TEST_START("6s hold but no toggle → revert (no shutdown)");
	G_t g; g_init(&g, MODE_OP);
	g_tick(&g, 0,    true, ED_ON);
	g_tick(&g, 6000, true, ED_NONE);
	g_tick(&g, 8001, true, ED_NONE);  /* window expired */
	ASSERT_EQ(EVT_NONE, g.pending);
	ASSERT_EQ(FSM_IDLE, g.fsm);
	TEST_PASS();
}

void test_release_during_holding_no_action(void)
{
	TEST_START("Release before 3s threshold → no action");
	G_t g; g_init(&g, MODE_OP);
	g_tick(&g, 0,    true, ED_ON);
	g_tick(&g, 2999, false, ED_OFF);
	ASSERT_EQ(FSM_IDLE, g.fsm);
	ASSERT_EQ(EVT_NONE, g.pending);
	TEST_PASS();
}

void test_shutdown_window_strict(void)
{
	TEST_START("Confirm at edge of 2s window still confirms");
	G_t g; g_init(&g, MODE_OP);
	g_tick(&g, 0,    true, ED_ON);
	g_tick(&g, 6000, true, ED_NONE);
	g_tick(&g, 6500, false, ED_OFF);
	g_tick(&g, 7999, true,  ED_ON);  /* 1999 ms after window_start = still in */
	ASSERT_EQ(EVT_REQUEST_SHUTDOWN, g.pending);
	TEST_PASS();
}

int main(void)
{
	printf("=== MGR_GESTURE 2-gesture magnet FSM tests ===\n");

	test_short_press_no_action();
	test_op_to_config_with_confirm();
	test_op_to_config_no_confirm_reverts();
	test_config_to_op();
	test_shutdown_from_op();
	test_shutdown_from_config();
	test_shutdown_no_confirm_reverts();
	test_release_during_holding_no_action();
	test_shutdown_window_strict();

	printf("\n%d/%d tests passed (%d failed)\n",
	       tests_passed, tests_run, tests_failed);
	return tests_failed ? 1 : 0;
}
