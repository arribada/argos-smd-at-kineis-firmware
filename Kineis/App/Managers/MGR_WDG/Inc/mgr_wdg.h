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

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Ensure the IWDG_STOP option byte is set (IWDG frozen during STOP).
 *
 * Reads FLASH_OPTR bit 17. If already set, returns false (no action). If
 * NOT set, unlocks FLASH + option-byte registers, programs the bit, then
 * calls HAL_FLASH_OB_Launch() which **triggers a system reset** to reload
 * the option byte. The function does NOT return on the success-program
 * path; the next boot sees the bit set and proceeds normally.
 *
 * Must be called early in init, BEFORE MGR_WDG_init() and BEFORE any LPM
 * client is active.
 *
 * Without the IWDG_STOP bit, IWDG keeps counting in STOP mode. The MAC
 * stack may keep the chip in STOP > 16 s between events, which would fire
 * the watchdog and silently reset the chip. With IWDG_STOP=1 the watchdog
 * freezes during STOP and only counts CPU-active time.
 *
 * @return false if the option byte was already correct OR if the program
 *         sequence failed (logged via MGR_LOG_DEBUG). The function does
 *         not return on the success-program path.
 */
bool MGR_WDG_ensureIwdgStopOptionByte(void);

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

/**
 * @brief Blocking delay that periodically kicks the watchdog.
 *
 * Use this in place of HAL_Delay() when a blocking wait can exceed (or come
 * close to) the IWDG timeout. The watchdog is refreshed every ~1 s, so the
 * total wait can run up to UINT32_MAX ms without nuisance reset.
 *
 * Granularity is 50 ms (the inner HAL_Delay step), so callers asking for
 * shorter waits should still use HAL_Delay directly to avoid rounding.
 *
 * @param total_ms Total wait time in milliseconds.
 */
void MGR_WDG_delayWithKick(uint32_t total_ms);

#endif /* MGR_WDG_H */
