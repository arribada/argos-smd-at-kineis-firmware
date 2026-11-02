/**
 * @file    lpm_spi_gate.h
 * @brief   STOP-over-SPI gating: pure logic shared by lpm.c and its unit test.
 *
 * STOP de-initialises SPI1 (lpm.c LPM_stop_enter): a response armed for the
 * host is lost and the frame that wakes the slave is dropped. The slave must
 * therefore stay out of STOP while the host is talking to it.
 */

#ifndef LPM_SPI_GATE_H
#define LPM_SPI_GATE_H

#include <stdbool.h>
#include <stdint.h>

/** Hold after a STOP wake or an SPI transaction. Covers the deployed hosts'
 * response poll (30 ms + 10 x 20 ms) and the gap to their next command. */
#define LPM_SPI_STOP_GRACE_MS 500u

/** SLEEP keeps the 1 ms tick for the SPI slave: a frame shorter than the DMA
 * half-transfer raises no IRQ, and the end-of-transaction detector runs on
 * HAL_GetTick, so a suspended tick leaves the frame unprocessed. */
#define LPM_SPI_SLEEP_KEEPS_TICK 1

/** Deadline of a hold window starting now; 0 is the "disarmed" sentinel. */
static inline uint32_t LPM_spiGraceDeadline(uint32_t now, uint32_t hold_ms)
{
	uint32_t deadline = now + hold_ms;

	return (deadline == 0u) ? 1u : deadline;
}

/** Wrap-safe; an expired deadline is cleared so it cannot re-engage 2^31 ms later. */
static inline bool LPM_spiGraceActive(volatile uint32_t *deadline, uint32_t now)
{
	if (*deadline == 0u)
		return false;
	if ((int32_t)(*deadline - now) > 0)
		return true;
	*deadline = 0u;
	return false;
}

/**
 * STOP is refused while the host is active: inside the hold window that follows
 * every transaction (a response may be armed for its next poll), while NSS is
 * low (frame in flight), while received bytes are not processed yet, or while a
 * command is being processed.
 */
static inline bool LPM_spiStopGate(bool grace_active, bool nss_low,
				   bool rx_pending, bool cmd_pending)
{
	return grace_active || nss_low || rx_pending || cmd_pending;
}

#endif /* LPM_SPI_GATE_H */
