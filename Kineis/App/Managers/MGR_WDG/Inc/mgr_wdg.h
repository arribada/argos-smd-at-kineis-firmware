/**
 * @file    mgr_wdg.h
 * @brief   IWDG watchdog manager (direct register access)
 *
 * Configures the STM32WL Independent Watchdog with ~16s timeout.
 * Uses LSI oscillator (32kHz), runs independently of system clock.
 *
 * The IWDG_STOP option byte should be set (default on STM32WL) so the
 * watchdog is frozen during STOP mode and doesn't need to be refreshed
 * before entering low-power modes.
 *
 * Once started, the IWDG cannot be stopped — only a reset clears it.
 */

#ifndef MGR_WDG_H
#define MGR_WDG_H

/**
 * @brief Start the IWDG with ~16s timeout
 *
 * Configuration: prescaler /256, reload 2000
 * Timeout = 2000 * 256 / 32000 = 16s
 *
 * The watchdog starts immediately and cannot be stopped.
 */
void MGR_WDG_init(void);

/**
 * @brief Refresh (kick) the watchdog
 *
 * Must be called at least every 16s to prevent reset.
 * Call from the main application loop.
 */
void MGR_WDG_refresh(void);

#endif /* MGR_WDG_H */
