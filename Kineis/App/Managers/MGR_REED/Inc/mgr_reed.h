/**
 * @file    mgr_reed.h
 * @brief   Reed switch (EXTI interrupt) and power latch driver for SMD_STDALONE board
 *
 * Reed switch: PB6, active HIGH (HIGH = magnet present)
 * Uses EXTI interrupt on both edges to detect magnet presence/removal
 * and measure hold duration for action mapping.
 *
 * Power latch: PB7, HIGH = keep board powered
 */

#ifndef MGR_REED_H
#define MGR_REED_H

#include <stdint.h>
#include <stdbool.h>

/** @brief Reed switch events */
typedef enum {
	MGR_REED_EVT_NONE = 0,
	MGR_REED_EVT_MAGNET_ON,   /**< Rising edge: magnet just detected */
	MGR_REED_EVT_MAGNET_OFF,  /**< Falling edge: magnet just removed */
} MGR_REED_Event_t;

/** @brief Initialize reed switch (PB6 EXTI) and power latch (PB7 output HIGH) */
void MGR_REED_init(void);

/** @brief Check if magnet is currently present (PB6 = HIGH) */
bool MGR_REED_isMagnetPresent(void);

/** @brief True iff a magnet-state transition is currently being polled but
 *  has not yet accumulated enough consecutive samples to confirm. The LPM
 *  client checks this before entering STOP2 — otherwise the chip can
 *  re-enter sleep mid-debounce and the magnet edge never gets recognised
 *  (debouncer accumulates ~1 sample per wake → never reaches the N-sample
 *  confirmation window). */
bool MGR_REED_isDebouncing(void);

/** @brief Get and clear pending reed event (call from main loop)
 *  @return Event type, or MGR_REED_EVT_NONE if no pending event
 */
MGR_REED_Event_t MGR_REED_getEvent(void);

/** @brief Get duration of last completed magnet hold in ms
 *  Valid after MGR_REED_EVT_MAGNET_OFF event.
 */
uint32_t MGR_REED_getLastHoldDuration_ms(void);

/** @brief Latch power on (PB7 = HIGH) */
void MGR_REED_latchPower(void);

/** @brief Release power latch (PB7 = LOW) */
void MGR_REED_releasePower(void);

#endif /* MGR_REED_H */
