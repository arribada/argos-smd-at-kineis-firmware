/**
 * @file    mgr_gesture.h
 * @brief   Magnet gesture FSM — 2-gesture confirmation protocol over MGR_REED
 *
 * Layer on top of `MGR_REED` (raw ON/OFF + hold duration) that implements the
 * deploy-tag UX:
 *
 *   POWER_OFF (SHUTDOWN, only reed wakes)
 *     ┊ Magnet ON
 *     └→ LED white solid → 5 × slow GREEN blink → OPERATIONAL
 *
 *   OPERATIONAL   (normal: SWS polling, TX on surface)
 *     ┊ Magnet held 3 s
 *     ├→ fast BLUE blink (question: enter CONFIG?)
 *     │    ┊ user OFF→ON inside 2 s
 *     │    ├→ slow BLUE confirm → CONFIG
 *     │    └→ no toggle in 2 s → revert to OPERATIONAL
 *     │
 *     ┊ Magnet held 6 s+
 *     └→ fast RED blink (question: SHUTDOWN?)
 *          ┊ user OFF→ON inside 2 s
 *          ├→ LED off → SHUTDOWN (PWR_LATCH LOW)
 *          └→ no toggle → revert to previous mode
 *
 *   CONFIG  (UART AT command session)
 *     ┊ Magnet held 3 s
 *     ├→ fast GREEN blink (question: back to OPERATIONAL?)
 *     │    ┊ user OFF→ON inside 2 s
 *     │    ├→ slow GREEN confirm → OPERATIONAL
 *     │    └→ no toggle → revert to CONFIG
 *     │
 *     ┊ Magnet held 6 s+
 *     └→ fast RED blink → confirm → SHUTDOWN
 *
 * Confirmation window: 2 s.
 * Cadences: SLOW = 250 ms on / 250 ms off, FAST = 100 ms on / 100 ms off.
 *
 * Mode is persisted in TAMP backup so OPERATIONAL/CONFIG survive resets and
 * the magnet wake from SHUTDOWN always lands in OPERATIONAL.
 */

#ifndef MGR_GESTURE_H
#define MGR_GESTURE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Public types ---- */

typedef enum {
	MGR_GESTURE_MODE_POWER_OFF = 0,  /**< Board is in SHUTDOWN, no firmware running */
	MGR_GESTURE_MODE_OPERATIONAL,    /**< Normal field mode: SWS + TX */
	MGR_GESTURE_MODE_CONFIG,         /**< UART AT command session, GUI configurable */
	MGR_GESTURE_MODE_COUNT,
} MGR_GESTURE_Mode_t;

typedef enum {
	MGR_GESTURE_EVT_NONE = 0,
	MGR_GESTURE_EVT_ENTER_OPERATIONAL,  /**< Switched to OPERATIONAL */
	MGR_GESTURE_EVT_ENTER_CONFIG,       /**< Switched to CONFIG */
	MGR_GESTURE_EVT_REQUEST_SHUTDOWN,   /**< User confirmed SHUTDOWN (caller must do enter_shutdown) */
} MGR_GESTURE_Event_t;

/* ---- Tunables (mirror linkit-v4 cadences) ---- */

#define MGR_GESTURE_LONG_HOLD_MS        3000u  /**< magnet hold to ask "switch mode?" */
#define MGR_GESTURE_SHUTDOWN_HOLD_MS    6000u  /**< magnet hold to ask "shutdown?" */
#define MGR_GESTURE_CONFIRM_WINDOW_MS   2000u  /**< user must OFF→ON inside this window */
#define MGR_GESTURE_WAKE_BLINK_COUNT       5u  /**< green blinks on POWER_OFF→OPERATIONAL */

#define MGR_GESTURE_BLINK_SLOW_ON_MS     250u
#define MGR_GESTURE_BLINK_SLOW_OFF_MS    250u
#define MGR_GESTURE_BLINK_FAST_ON_MS     100u
#define MGR_GESTURE_BLINK_FAST_OFF_MS    100u

/** Very-fast urgent blink for the WAIT_CONFIRM window — 10 Hz, more attention-
 *  grabbing than the 5 Hz fast blink used during the hold. */
#define MGR_GESTURE_BLINK_VFAST_ON_MS    50u
#define MGR_GESTURE_BLINK_VFAST_OFF_MS   50u

/** Duration of the SOLID-colour CONFIRMED acknowledgement after the second
 *  magnet apply. Short enough to feel "instant ack", long enough to be
 *  unmissable. */
#define MGR_GESTURE_CONFIRM_SOLID_MS    600u

/* ---- API ---- */

/** Initialize the gesture FSM. Call after MGR_REED_init + MGR_LED_init.
 *  Restores the persisted mode from TAMP backup; defaults to OPERATIONAL on
 *  first power-up. Should be called near the end of app init. */
void MGR_GESTURE_init(void);

/** Run one FSM iteration. Call from the main app loop (idempotent). */
void MGR_GESTURE_task(void);

/** Pop the next high-level event since the last call. Returns NONE if
 *  nothing happened. The caller (app) reacts: switch state machine into
 *  CONFIG / OPERATIONAL, or trigger shutdown. */
MGR_GESTURE_Event_t MGR_GESTURE_getEvent(void);

/** Current persisted mode. */
MGR_GESTURE_Mode_t MGR_GESTURE_getMode(void);

/** Force a mode (e.g. boot path applying restored mode). Persisted. */
void MGR_GESTURE_setMode(MGR_GESTURE_Mode_t mode);

/** @brief True if the gesture FSM is currently driving the LED.
 *
 * Used by the app to gate competing LED writers (e.g. the CONFIG-mode
 * BLUE indicator must yield while a magnet hold is in progress so the
 * user sees the gesture feedback, not the steady-state colour). */
bool MGR_GESTURE_isInteracting(void);

#ifdef __cplusplus
}
#endif

#endif /* MGR_GESTURE_H */
