/**
 * @file    mgr_txstats.h
 * @brief   Persistent TX statistics for radio health monitoring
 *
 * Counts every TX attempt and its outcome (DONE / TIMEOUT / MAC_ERROR), plus
 * derived metrics (consecutive failures, total PA-on time). Counters live in
 * SRAM2 retention so they survive software / IWDG / brown-out resets, but are
 * lost on full SHUTDOWN (PWR_LATCH down or VBAT removed). The operator should
 * snapshot via AT+TXSTATS=? before any deliberate shutdown.
 *
 * Magic + CRC32 detect first-power-on or corruption; in either case the
 * counters reset to zero.
 *
 * Hook points (called from the UW_DOPPLER state machine):
 *   - MGR_TXSTATS_recordAttempt() when KNS_Q_push for TX succeeds
 *   - MGR_TXSTATS_recordDone()    on KNS_MAC_TX_DONE
 *   - MGR_TXSTATS_recordTimeout() on KNS_MAC_TX_TIMEOUT
 *   - MGR_TXSTATS_recordError()   on KNS_MAC_ERROR (with SEND_DATA evt)
 */

#ifndef MGR_TXSTATS_H
#define MGR_TXSTATS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint32_t attempts;          /**< TX requests pushed to MAC */
	uint32_t done;              /**< TX_DONE acknowledgments */
	uint32_t timeouts;          /**< TX_TIMEOUT events */
	uint32_t errors;            /**< KNS_MAC_ERROR on SEND_DATA */
	uint32_t consecutive_fail;  /**< Streak length (reset on each DONE) */
	uint32_t worst_consecutive; /**< Highest consecutive_fail ever observed */
} MGR_TXSTATS_t;

/** @brief Validate retention buffer and reset to zeros if first-boot/corrupt. */
void MGR_TXSTATS_init(void);

void MGR_TXSTATS_recordAttempt(void);
void MGR_TXSTATS_recordDone(void);
void MGR_TXSTATS_recordTimeout(void);
void MGR_TXSTATS_recordError(void);

/** @brief Copy a consistent snapshot of all counters. */
void MGR_TXSTATS_get(MGR_TXSTATS_t *out);

/** @brief Reset all counters (operator action). */
void MGR_TXSTATS_clear(void);

#endif /* MGR_TXSTATS_H */
