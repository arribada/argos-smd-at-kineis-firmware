/**
 * @file    mgr_gesture.c
 * @brief   Magnet gesture FSM — "decide on release" protocol.
 *
 * Redesigned 2026-06-06 to fix two UX bugs in the previous "commit at 3s"
 * design:
 *   - User couldn't escalate from "switch mode" (3 s) to "shutdown" (6 s)
 *     because the FSM left the HOLDING branch as soon as 3 s elapsed.
 *   - Confirmation flow was unclear; LED feedback was discontinuous.
 *
 * New FSM (states, with magnet-OFF / magnet-ON transitions and timing):
 *
 *   IDLE                                            LED: OFF
 *      └─ magnet ON ───────────────────────────────→ HOLDING
 *
 *   HOLDING  (continuous LED update by elapsed time since press)
 *      ┊  0 .. 3 s    : soft WHITE   ("magnet detected, keep holding")
 *      ┊  3 .. 6 s    : BLUE  blink  ("release now to switch mode")
 *      ┊  ≥ 6 s       : RED   blink  ("release now to SHUTDOWN")
 *      └─ magnet OFF  ─ classify by max phase reached ──→ WAIT_CONFIRM
 *                       phase 0 → no action (back to IDLE, too short)
 *                       phase 1 → pending = SWITCH MODE
 *                       phase 2 → pending = SHUTDOWN
 *
 *   WAIT_CONFIRM  (magnet just released after a valid hold)            LED: OFF
 *      ┊  window CONFIRM_WINDOW_MS                    (2 s)
 *      ┊  magnet ON inside window → CONFIRMED
 *      └─ window expires             → IDLE (silently)
 *
 *   CONFIRMED                                       LED: slow blink in target colour
 *      ┊  GREEN slow = OPERATIONAL
 *      ┊  BLUE  slow = CONFIG
 *      ┊  RED   slow = SHUTDOWN
 *      └─ blink done → fire event → IDLE
 *
 *   WAKE_BLINK (one-shot at first boot after POWER_OFF wake)
 *      ┊  5 × GREEN slow
 *      └─ blink done → IDLE
 *
 * Mode is persisted in TAMP BKP10R (magic 0x47535400). Survives all resets
 * except VBAT loss; defaults to OPERATIONAL on first power-up.
 */

#include "mgr_gesture.h"
#include "mgr_reed.h"
#include "mgr_led.h"
#include "mgr_log.h"
#include "stm32wlxx_hal.h"
#include "usart.h"
#include <stdio.h>

/* Set by main.c at startup from the RCC->CSR snapshot taken before HAL
 * clears the flags. Lets us tell a true cold boot (BOR/PIN/OBL) from a
 * soft reset (IWDG/SFT) without re-reading RCC->CSR ourselves. */
extern uint32_t g_boot_rcc_csr_raw;

/* Direct UART trace helper. Compiled out in DEBUG=0 production builds so
 * an OPERATIONAL tag emits nothing on the wire (AT responses excepted).
 *
 * In DEBUG=1 builds it bypasses MGR_LOG_DEBUG so the user can see gesture
 * events without enabling the verbose MGR_LOG ring. Gated on hlpuart1.gState
 * (no-op when UART has been de-init'd). Stack buffer (not static) so the
 * function is reentrant. TX timeout 20 ms (cap at 115200 baud). */
/* gst_trace is ALWAYS on (production + debug) so mode transitions are
 * visible in the field for validation. Stack buffer (not static) so
 * the function is reentrant. TX timeout 20 ms (cap at 115200 baud). */
static void gst_trace(const char *fmt, uint32_t a)
{
	extern UART_HandleTypeDef hlpuart1;
	if (hlpuart1.gState == HAL_UART_STATE_RESET)
		return;
	char buf[80];
	int n = snprintf(buf, sizeof(buf), fmt, (unsigned long)a);
	if (n > 0 && n < (int)sizeof(buf))
		(void)HAL_UART_Transmit(&hlpuart1, (uint8_t *)buf,
		                        (uint16_t)n, 20);
}

/* ---- Mode persistence (TAMP BKP10R) --------------------------------------- */
#define GESTURE_BKP_REG          (TAMP->BKP10R)
#define GESTURE_BKP_MAGIC        0x47535400u  /* "GST\0" upper 24 bits */
#define GESTURE_BKP_MAGIC_MASK   0xFFFFFF00u
#define GESTURE_BKP_MODE_MASK    0x0000000Fu

/* ---- FSM internal state --------------------------------------------------- */

typedef enum {
	GFSM_IDLE = 0,
	GFSM_HOLDING,
	GFSM_ACK_BRIEF,      /**< brief tap < LONG_HOLD: hold LED visible for ACK */
	GFSM_WAIT_CONFIRM,
	GFSM_CONFIRMED,
	GFSM_WAKE_BLINK,
} GestureFsmState_t;

/* Minimum LED visibility window when a tap is too brief to trigger a phase.
 * Without this the LED would light for ~1 main-loop iteration (5-50 ms) and
 * be invisible to the eye. 300 ms is long enough to register as a flash. */
#define MGR_GESTURE_ACK_BRIEF_MS  300u

typedef enum {
	HOLD_PHASE_NONE = 0,    /**< < LONG_HOLD_MS, too short */
	HOLD_PHASE_SWITCH,      /**< [LONG_HOLD_MS, SHUTDOWN_HOLD_MS) */
	HOLD_PHASE_SHUTDOWN,    /**< ≥ SHUTDOWN_HOLD_MS */
} HoldPhase_t;

typedef enum {
	PENDING_NONE = 0,
	PENDING_SWITCH,
	PENDING_SHUTDOWN,
} PendingAction_t;

static MGR_GESTURE_Mode_t s_mode = MGR_GESTURE_MODE_OPERATIONAL;
static GestureFsmState_t  s_fsm  = GFSM_IDLE;
static uint32_t           s_press_tick = 0;
static HoldPhase_t        s_max_phase = HOLD_PHASE_NONE;
static uint32_t           s_window_start_tick = 0;
static PendingAction_t    s_pending_action = PENDING_NONE;
static MGR_GESTURE_Mode_t s_pending_mode = MGR_GESTURE_MODE_OPERATIONAL;
static MGR_GESTURE_Event_t s_pending_event = MGR_GESTURE_EVT_NONE;
static bool               s_first_init = true;

/* ---- Mode persistence ---- */

static void persist_mode(MGR_GESTURE_Mode_t mode)
{
	GESTURE_BKP_REG = GESTURE_BKP_MAGIC | ((uint32_t)mode & GESTURE_BKP_MODE_MASK);
}

static MGR_GESTURE_Mode_t load_mode_default_operational(void)
{
	uint32_t raw = GESTURE_BKP_REG;
	if ((raw & GESTURE_BKP_MAGIC_MASK) != GESTURE_BKP_MAGIC)
		return MGR_GESTURE_MODE_OPERATIONAL;
	MGR_GESTURE_Mode_t m = (MGR_GESTURE_Mode_t)(raw & GESTURE_BKP_MODE_MASK);
	if (m >= MGR_GESTURE_MODE_COUNT)
		return MGR_GESTURE_MODE_OPERATIONAL;
	return m;
}

/* ---- LED helpers ---- */

static MGR_LED_Color_t target_color(MGR_GESTURE_Mode_t target, bool is_shutdown)
{
	if (is_shutdown)
		return MGR_LED_RED;
	if (target == MGR_GESTURE_MODE_CONFIG)
		return MGR_LED_BLUE;
	return MGR_LED_GREEN;
}

/* Return the colour that the SWITCH phase (3-6 s hold) would commit to
 * if released now: target = the OTHER mode. */
static MGR_LED_Color_t switch_target_color(void)
{
	/* If currently OPERATIONAL → switching to CONFIG → BLUE.
	 * If currently CONFIG      → switching to OPERATIONAL → GREEN. */
	const MGR_GESTURE_Mode_t target =
	    (s_mode == MGR_GESTURE_MODE_OPERATIONAL)
	    ? MGR_GESTURE_MODE_CONFIG
	    : MGR_GESTURE_MODE_OPERATIONAL;
	return target_color(target, false);
}

static void led_show_phase(HoldPhase_t phase)
{
	/* LED feedback during the hold.
	 *   DETECT (0-3 s)   : WHITE solid    "magnet seen, keep holding"
	 *   SWITCH (3-6 s)   : target FAST    "release+reapply to <target>"
	 *   SHUTDOWN (≥6 s)  : RED FAST       "release+reapply to power off"
	 *
	 * The hold-phase blink IS the call to action — "release now, then
	 * re-apply to confirm". It continues for as long as the user holds
	 * past the threshold (so they have all the time they need to decide),
	 * and is replaced by SLOW blink upon successful confirm (see
	 * led_show_confirm). */
	switch (phase) {
	case HOLD_PHASE_NONE:
		MGR_LED_set(MGR_LED_WHITE);
		gst_trace("[GST] phase=DETECT (white solid)\r\n", 0);
		break;
	case HOLD_PHASE_SWITCH: {
		const MGR_LED_Color_t c = switch_target_color();
		MGR_LED_blink(c, 0,
		              MGR_GESTURE_BLINK_FAST_ON_MS,
		              MGR_GESTURE_BLINK_FAST_OFF_MS);
		gst_trace("[GST] phase=SWITCH fast target=%lu\r\n",
		          (uint32_t)c);
		break;
	}
	case HOLD_PHASE_SHUTDOWN:
		MGR_LED_blink(MGR_LED_RED, 0,
		              MGR_GESTURE_BLINK_FAST_ON_MS,
		              MGR_GESTURE_BLINK_FAST_OFF_MS);
		gst_trace("[GST] phase=SHUTDOWN (red fast)\r\n", 0);
		break;
	}
}

/** CONFIRMED ack: SLOW blink in target colour for ~1.5 s, then fire the
 *  event. Slow blink is the visual signal "✓ state change committed",
 *  distinguishing the just-validated transition from the FAST blink the
 *  user was looking at moments earlier (hold-phase or wait-for-confirm).
 *
 *  3 cycles × (250 ms on + 250 ms off) = 1.5 s total. */
static void led_show_confirm(MGR_GESTURE_Mode_t target, bool is_shutdown)
{
	const MGR_LED_Color_t c = target_color(target, is_shutdown);
	MGR_LED_blink(c, 3u,
	              MGR_GESTURE_BLINK_SLOW_ON_MS,
	              MGR_GESTURE_BLINK_SLOW_OFF_MS);
}

/** WAIT_CONFIRM indicator: FAST blink (5 Hz) in the colour that will be
 *  applied if the user re-applies the magnet inside the 2 s confirm
 *  window. Blinking == "action required NOW" (release+reapply); the
 *  preceding hold phases use solid colours so the user can tell them
 *  apart at a glance. */
static void led_show_wait_confirm(MGR_GESTURE_Mode_t target, bool is_shutdown)
{
	const MGR_LED_Color_t c = target_color(target, is_shutdown);
	MGR_LED_blink(c, 0u,  /* infinite — WAIT_CONFIRM state will MGR_LED_off
	                       * on expiry / completion */
	              MGR_GESTURE_BLINK_FAST_ON_MS,
	              MGR_GESTURE_BLINK_FAST_OFF_MS);
}

/* ---- Public API ---- */

void MGR_GESTURE_init(void)
{
	/* Always boot in OPERATIONAL — sealed-deployment safety.
	 *
	 * If anything goes wrong in the capsule (IWDG fires, HardFault,
	 * brownout, watchdog cycle from an MAC stack hang) the tag MUST come
	 * back up tracking. Inheriting a stale CONFIG mode from a previous
	 * session would strand the device in "waiting for AT commands"
	 * forever — but in the sealed capsule there's no UART access, so
	 * that state is a brick.
	 *
	 * CONFIG remains reachable at runtime via the explicit 3 s magnet
	 * gesture; it is never inherited. TAMP BKP10R is still written by
	 * MGR_GESTURE_setMode (observable / debuggable) but is overridden
	 * on every boot here — equivalent to "no persistence". */
	(void)load_mode_default_operational();  /* legacy: keep symbol live */
	s_mode = MGR_GESTURE_MODE_OPERATIONAL;
	persist_mode(s_mode);
	gst_trace("[MODE] boot in OPERATIONAL t=%lu ms\r\n", HAL_GetTick());

	s_fsm = GFSM_IDLE;
	s_pending_event = MGR_GESTURE_EVT_NONE;
	s_pending_action = PENDING_NONE;
	/* Default pending_mode to the just-loaded persisted mode so a
	 * confirm-after-reset is a safe no-op rather than a forced OPERATIONAL
	 * switch (s_pending_mode lives in .bss, not retention). */
	s_pending_mode = s_mode;
	s_max_phase = HOLD_PHASE_NONE;
	s_press_tick = 0;

	/* WAKE_BLINK is the "I just woke up from POWER_OFF / SHUTDOWN" indicator.
	 * Fire it only on a true cold boot (BOR, NRST, OBL launch). On software
	 * resets (IWDG, SFT reset triggered by AT+RESET or HardFault) the user
	 * didn't physically wake the device — replaying 5 green blinks would be
	 * a false signal that survives the reboot loop. */
	if (s_first_init) {
		s_first_init = false;
		const uint32_t cold_mask =
		    RCC_CSR_BORRSTF | RCC_CSR_PINRSTF | RCC_CSR_OBLRSTF;
		if (g_boot_rcc_csr_raw & cold_mask) {
			MGR_LED_blink(MGR_LED_GREEN, MGR_GESTURE_WAKE_BLINK_COUNT,
			              MGR_GESTURE_BLINK_SLOW_ON_MS,
			              MGR_GESTURE_BLINK_SLOW_OFF_MS);
			s_fsm = GFSM_WAKE_BLINK;
		}
	}
}

MGR_GESTURE_Mode_t MGR_GESTURE_getMode(void)
{
	return s_mode;
}

void MGR_GESTURE_setMode(MGR_GESTURE_Mode_t mode)
{
	if (mode >= MGR_GESTURE_MODE_COUNT)
		return;
	s_mode = mode;
	persist_mode(mode);
}

MGR_GESTURE_Event_t MGR_GESTURE_getEvent(void)
{
	MGR_GESTURE_Event_t e = s_pending_event;
	s_pending_event = MGR_GESTURE_EVT_NONE;
	return e;
}

/* Watchdog for FSM wedged in non-IDLE. Bounds the "stuck FSM blocks
 * STOP2 forever" scenario flagged in the gesture audit (RISK A): if
 * isInteracting() returned true permanently (SEU on s_fsm, glued
 * reed, task() never re-entered) the LPM client would skip STOP2 on
 * every tick and burn the 19 Ah battery in ~4 days. Any non-IDLE
 * window longer than this returns false so STOP2 can run; the FSM
 * stays where it is until the next reed event re-arms it. */
#define GESTURE_NONIDLE_MAX_MS  15000u  /* 15 s. Longest legit gesture is
                                         * 6 s hold + 2 s confirm + 1.5 s
                                         * confirm blink ≈ 10 s, so 15 s
                                         * leaves a 50 % margin while
                                         * still recovering quickly enough
                                         * that the operator notices the
                                         * gesture failed. */
static uint32_t s_nonidle_first_tick = 0u;

bool MGR_GESTURE_isInteracting(void)
{
	/* IDLE means: no magnet activity in flight. Anything else means the FSM
	 * has the LED busy (hold feedback, wait-for-confirm urgent blink,
	 * confirmed solid pulse, or wake-blink boot sequence). */
	if (s_fsm == GFSM_IDLE) {
		s_nonidle_first_tick = 0u;
		return false;
	}
	const uint32_t now = HAL_GetTick();
	if (s_nonidle_first_tick == 0u)
		s_nonidle_first_tick = (now == 0u) ? 1u : now;
	/* Hard reset after 60 s stuck in non-IDLE. Originally we just told
	 * the LPM client "not interacting" so STOP2 could resume; the FSM
	 * itself stayed in its wedged state and the LED kept blinking
	 * forever (observed on the bench: Hall sensor hysteresis holds the
	 * reed-input HIGH after the operator believes they released, so
	 * HOLDING never transitions to WAIT_CONFIRM, and the RED blink
	 * looks indistinguishable from a stuck WAIT_CONFIRM). Force the
	 * FSM to IDLE and turn the LED off so the operator gets a clear
	 * "abandoned" signal. */
	if ((now - s_nonidle_first_tick) > GESTURE_NONIDLE_MAX_MS) {
		gst_trace("[MODE] gesture watchdog kicked, forcing IDLE t=%lu\r\n",
		          now);
		MGR_LED_off();
		s_fsm = GFSM_IDLE;
		s_pending_action = PENDING_NONE;
		s_max_phase = HOLD_PHASE_NONE;
		s_nonidle_first_tick = 0u;
		return false;
	}
	return true;
}

void MGR_GESTURE_task(void)
{
	MGR_REED_Event_t reed_evt = MGR_REED_getEvent();
	const uint32_t now = HAL_GetTick();
	const bool magnet_present = MGR_REED_isMagnetPresent();

	/* ----- WAKE_BLINK: just wait for the LED sequence to finish ----- */
	if (s_fsm == GFSM_WAKE_BLINK) {
		if (MGR_LED_isBlinkDone()) {
			s_fsm = GFSM_IDLE;
			/* Re-emit the persisted mode so the app's gesture-event
			 * handler can replay its side effects (SWS power, UART
			 * gating) — previously hardcoded to OPERATIONAL which
			 * left a tag booting in CONFIG in a half-configured
			 * state. */
			s_pending_event =
			    (s_mode == MGR_GESTURE_MODE_CONFIG)
			    ? MGR_GESTURE_EVT_ENTER_CONFIG
			    : MGR_GESTURE_EVT_ENTER_OPERATIONAL;
		}
		return;
	}

	/* ----- IDLE: fresh magnet ON kicks off hold tracking. Use the explicit
	 *             EXTI event when available, fall back to polled state so the
	 *             FSM still works if the boot-time edge was missed (chip
	 *             booted with magnet already present) or debounce ate it. ---- */
	if (s_fsm == GFSM_IDLE) {
		if (reed_evt == MGR_REED_EVT_MAGNET_ON || magnet_present) {
			s_fsm = GFSM_HOLDING;
			s_press_tick = now;
			s_max_phase = HOLD_PHASE_NONE;
			MGR_LED_setMode(MGR_LED_MODE_ON);
			led_show_phase(HOLD_PHASE_NONE);
			gst_trace("[GST] ON t=%lu\r\n", now);
		}
		return;
	}

	/* ----- HOLDING: track elapsed, update LED on phase boundary,
	 *               classify on release.
	 * Tick arithmetic is uint32_t; subtraction wraps cleanly at the
	 * 49.7-day uptime limit so no monotonic-check guard is needed. ----- */
	if (s_fsm == GFSM_HOLDING) {
		if (!magnet_present || reed_evt == MGR_REED_EVT_MAGNET_OFF) {
			/* Release — classify by max phase reached. */
			gst_trace("[GST] OFF max_phase=%lu\r\n",
			          (uint32_t)s_max_phase);
			switch (s_max_phase) {
			case HOLD_PHASE_NONE:
				/* Brief tap — keep the WHITE detect-LED visible
				 * for MGR_GESTURE_ACK_BRIEF_MS so the user sees
				 * the press was registered (same colour as the
				 * DETECT hold phase for visual continuity), then
				 * return to IDLE. */
				s_fsm = GFSM_ACK_BRIEF;
				s_window_start_tick = now;
				return;
			case HOLD_PHASE_SWITCH:
				s_pending_action = PENDING_SWITCH;
				s_pending_mode =
				    (s_mode == MGR_GESTURE_MODE_OPERATIONAL)
				    ? MGR_GESTURE_MODE_CONFIG
				    : MGR_GESTURE_MODE_OPERATIONAL;
				break;
			case HOLD_PHASE_SHUTDOWN:
				s_pending_action = PENDING_SHUTDOWN;
				s_pending_mode = s_mode;
				break;
			default:
				break;
			}
			s_fsm = GFSM_WAIT_CONFIRM;
			s_window_start_tick = now;
			/* Very-fast blink in the pending target colour so the user
			 * knows the first hold was registered and we're waiting for
			 * the second tap. Replaces the prior "LED OFF" silence. */
			led_show_wait_confirm(s_pending_mode,
			                      s_pending_action == PENDING_SHUTDOWN);
			gst_trace("[GST] WAIT_CONFIRM 2s window pending=%lu\r\n",
			          (uint32_t)s_pending_action);
			return;
		}

		/* Still holding — check whether elapsed crossed a phase boundary. */
		uint32_t elapsed = now - s_press_tick;
		HoldPhase_t new_phase = HOLD_PHASE_NONE;
		if (elapsed >= MGR_GESTURE_SHUTDOWN_HOLD_MS)
			new_phase = HOLD_PHASE_SHUTDOWN;
		else if (elapsed >= MGR_GESTURE_LONG_HOLD_MS)
			new_phase = HOLD_PHASE_SWITCH;

		if (new_phase != s_max_phase) {
			s_max_phase = new_phase;
			led_show_phase(new_phase);
		}
		return;
	}

	/* ----- ACK_BRIEF: tap was too short to trigger a phase, but show RED
	 *                 for MGR_GESTURE_ACK_BRIEF_MS so the user sees that the
	 *                 magnet was registered. ----- */
	if (s_fsm == GFSM_ACK_BRIEF) {
		if ((now - s_window_start_tick) >= MGR_GESTURE_ACK_BRIEF_MS) {
			MGR_LED_off();
			s_fsm = GFSM_IDLE;
		}
		return;
	}

	/* ----- WAIT_CONFIRM: user must re-apply within CONFIRM_WINDOW_MS ----- */
	if (s_fsm == GFSM_WAIT_CONFIRM) {
		uint32_t win_elapsed = now - s_window_start_tick;

		if (reed_evt == MGR_REED_EVT_MAGNET_ON) {
			/* Confirmed! */
			bool is_shutdown = (s_pending_action == PENDING_SHUTDOWN);
			if (is_shutdown) {
				s_pending_event = MGR_GESTURE_EVT_REQUEST_SHUTDOWN;
				gst_trace("[MODE] SHUTDOWN requested by gesture t=%lu ms\r\n",
				          now);
			} else {
				MGR_GESTURE_setMode(s_pending_mode);
				s_pending_event =
				    (s_pending_mode == MGR_GESTURE_MODE_CONFIG)
				    ? MGR_GESTURE_EVT_ENTER_CONFIG
				    : MGR_GESTURE_EVT_ENTER_OPERATIONAL;
				if (s_pending_mode == MGR_GESTURE_MODE_CONFIG) {
					gst_trace("[MODE] CONFIG entered (TX disabled) t=%lu\r\n",
					          now);
				} else {
					gst_trace("[MODE] OPERATIONAL entered (TX enabled) t=%lu\r\n",
					          now);
				}
			}
			led_show_confirm(s_pending_mode, is_shutdown);
			s_fsm = GFSM_CONFIRMED;
			return;
		}

		if (win_elapsed >= MGR_GESTURE_CONFIRM_WINDOW_MS) {
			gst_trace("[GST] EXPIRED t=%lu\r\n", now);
			MGR_LED_off();
			s_fsm = GFSM_IDLE;
			s_pending_action = PENDING_NONE;
			return;
		}
		return;
	}

	/* ----- CONFIRMED: wait for confirm blink to finish, then idle ----- */
	if (s_fsm == GFSM_CONFIRMED) {
		if (MGR_LED_isBlinkDone()) {
			s_fsm = GFSM_IDLE;
			s_pending_action = PENDING_NONE;
		}
		return;
	}

	/* Fallback: state enum corrupted (SEU, stack smash, future enum value
	 * added without handler). Recover to IDLE rather than silently freezing. */
	gst_trace("[GST] FSM recover from invalid state=%lu\r\n",
	          (uint32_t)s_fsm);
	MGR_LED_off();
	s_fsm = GFSM_IDLE;
	s_pending_action = PENDING_NONE;
}
