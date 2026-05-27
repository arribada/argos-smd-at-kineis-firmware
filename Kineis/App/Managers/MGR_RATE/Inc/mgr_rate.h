/**
 * @file    mgr_rate.h
 * @brief   Persistent sliding-window TX rate limiter
 *
 * Last-resort defence for a deployed tracker: caps the number of TX bursts
 * inside a configurable rolling window. If a scheduling bug, an oscillating
 * SWS or a runaway state machine tries to keep transmitting, the limiter
 * blocks before the battery is drained.
 *
 * Storage:
 *   - Ring buffer of unix-style timestamps (seconds since boot, monotonic)
 *     held in SRAM2 retention RAM (.retentionRamBss), survives every reset
 *     class except VBAT removal.
 *   - Magic + CRC32 detect first-power-on / accidental corruption.
 *
 * Config is held in RAM (gettable/settable via API) and is persisted by the
 * NVM layer so that AT+RATECFG values survive a power cycle.
 *
 * Time base:
 *   The limiter uses a "seconds since first call to MGR_RATE_init()" clock
 *   derived from HAL_GetTick(). This monotonic clock survives sleep/wake and
 *   does NOT need a synced RTC. It rolls over only after ~136 years.
 */

#ifndef MGR_RATE_H
#define MGR_RATE_H

#include <stdint.h>
#include <stdbool.h>

/** Sane defaults — 100 TX per 24h. Configurable at runtime via AT+RATECFG. */
#define MGR_RATE_DEFAULT_WINDOW_S   86400u
#define MGR_RATE_DEFAULT_MAX_TX     100u

/** Max ring capacity (compile-time). Window may be configured larger than
 *  this many TX, but past MAX_CAP we just block. Sized to ~1 KB at 4B each. */
#define MGR_RATE_MAX_CAP            256u

/** Hard bounds for AT+RATECFG sanity-checks. */
#define MGR_RATE_MIN_WINDOW_S       60u           /* 1 minute */
#define MGR_RATE_MAX_WINDOW_S       (7u * 86400u) /* 7 days   */
#define MGR_RATE_MIN_MAX_TX         1u
#define MGR_RATE_MAX_MAX_TX         MGR_RATE_MAX_CAP

/** Initialise (idempotent). Validates retention buffer, starts the clock. */
void MGR_RATE_init(void);

/** Record a TX that's just been accepted. Trims old entries past the window.
 *  Does NOT enforce the limit — call MGR_RATE_isBlocked() before TX. */
void MGR_RATE_recordTx(void);

/**
 * @brief Check whether a new TX would exceed the budget.
 *
 * @param[out] retry_in_s  If blocked, time (s) until the oldest entry exits
 *                         the window. May be NULL.
 * @return true if budget exhausted, false if a TX is allowed.
 */
bool MGR_RATE_isBlocked(uint32_t *retry_in_s);

/** Configure the limiter. Bounds-checked; out-of-range values are clamped.
 *  Does NOT persist — caller (MGR_NVM_save) handles flash. */
void MGR_RATE_setConfig(uint32_t window_s, uint16_t max_tx);

/** Read current config + live counter. Any out-pointer may be NULL. */
void MGR_RATE_getConfig(uint32_t *window_s, uint16_t *max_tx,
                        uint16_t *current_count);

/** Wipe the ring buffer (e.g. after RTC resync or AT+RATECLEAR). Config
 *  unchanged. */
void MGR_RATE_clear(void);

/** Current "limiter clock" in seconds, for logging / AT+RATE. */
uint32_t MGR_RATE_nowS(void);

#endif /* MGR_RATE_H */
