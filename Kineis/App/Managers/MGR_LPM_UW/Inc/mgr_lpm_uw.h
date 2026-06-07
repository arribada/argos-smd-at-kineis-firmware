/**
 * @file    mgr_lpm_uw.h
 * @brief   UW_DOPPLER-dedicated LPM manager.
 *
 * App-driven low-power-mode strategy for the UW_DOPPLER sealed-deployment
 * profile. Independent of the Kineis MGR_LPM aggregator: that one is driven
 * by the MAC stack's resource status, which on STM32WL55 + STDALONE asks
 * for STANDBY/SHUTDOWN when idle in a way that doesn't match our sealed-
 * capsule duty-cycle needs.
 *
 * Two entry points are exposed:
 *   - STANDBY_TIMED       : RTC-armed wake after N seconds. Board stays
 *                           powered (PWR_LATCH held HIGH via PWR controller
 *                           pull-up). Cold-boot on wake; SRAM2 retention
 *                           preserves boot-loop counter, SWS calibration,
 *                           crash forensics, and this module's config.
 *                           ~2 µA in STANDBY + cold-boot active time per
 *                           cycle. Use case: scheduled SWS check + opt TX.
 *
 *   - SHUTDOWN_REED       : Board powers off (PWR_LATCH pulled LOW).
 *                           Only the HW reed-switch circuit can re-energise
 *                           the regulator. <1 µA between magnet events.
 *                           Use case: end-of-mission, between user-driven
 *                           events, or anywhere a magnet operator owns the
 *                           wake-up.
 *
 * The auto-cycle policy (`MGR_LPM_UW_tryAutoCycle`) inspects SWS state,
 * gesture FSM, deploy mode and the persisted duty configuration to decide
 * whether to drop to STANDBY at every MONITORING tick.
 *
 * Persistence: `duty_cfg` lives in `.retentionRamNoload` (NOLOAD section
 * in SRAM2). It survives every software-class reset including each
 * STANDBY cold-boot. Only VBAT loss / battery removal wipes it.
 */

#ifndef MGR_LPM_UW_H
#define MGR_LPM_UW_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Public API ---- */

/** @brief Validate the retention-NOLOAD duty config, applying defaults
 *  on first power-on (magic mismatch). Call once during app init. */
void MGR_LPM_UW_init(void);

/** @brief Enter STANDBY mode for `seconds`, then cold-boot.
 *
 * Saves NVM, arms RTC wake, holds PWR_LATCH + VSEL HIGH via the PWR
 * controller, enables SRAM2 retention + internal wake line, clears
 * sticky PWR flags, then calls HAL_PWR_EnterSTANDBYMode.
 *
 * Does not return. Cold-boots via NVIC reset → main() once the RTC
 * timer fires. The retention NOLOAD region preserves boot counter,
 * SWS calibration, crash forensics, and this module's `duty_cfg`.
 *
 * @param seconds  Wake interval (1..65535 s on the 16-bit CK_SPRE
 *                 prescaler — covers up to 18.2 hours per cycle).
 */
__attribute__((noreturn))
void MGR_LPM_UW_enterStandbyTimed(uint32_t seconds);

/** @brief Enter SHUTDOWN mode. Board powers off via PWR_LATCH LOW.
 *
 * Only the HW reed-switch circuit can re-energise the regulator on
 * SMD_STDALONE (the magnet re-applies VBAT to the regulator enable
 * via the external reed/PWR_LATCH OR-gate path). RTC wake does NOT
 * work from this mode because the chip itself is power-gated.
 *
 * Does not return.
 */
__attribute__((noreturn))
void MGR_LPM_UW_enterShutdownReed(void);

/** @brief Enter SHUTDOWN mode with optional auto-wake. If
 *  `wakeup_seconds > 0` the RTC alarm fires after the interval to
 *  cold-boot the chip even without a magnet event (intended for the
 *  boot-loop guard's "24 h retry from PERMANENT_OFF" path — only
 *  meaningful on boards where the PWR_LATCH path keeps the RTC
 *  domain alive; on STDALONE this is best-effort and may require
 *  a magnet to actually re-power).
 */
__attribute__((noreturn))
void MGR_LPM_UW_enterShutdownAutoWake(uint32_t wakeup_seconds);

/* ---- Auto-cycle policy ---- */

/** @brief Auto-cycle decision. Called once per MONITORING tick from
 *  the app's main loop when the state machine has no urgent work to
 *  do. Drops to STANDBY (via MGR_LPM_UW_enterStandbyTimed) when:
 *    - duty_cfg.enabled is true
 *    - `sws_state` ∈ { SURFACE_IDLE, UNDERWATER }
 *    - no gesture activity (`gesture_busy` == false)
 *    - not in CONFIG mode (`config_mode` == false)
 *    - the boot stabilization window has expired
 *
 *  Sleep duration is `surf_sleep_s` when surface-idle and
 *  `uw_sleep_s` when underwater (per `duty_cfg`).
 *
 *  Threshold-based mode selection (when the computed sleep ≥ 5 s and
 *  `shutdown_threshold_s` ≥ 5 s in the config): drops to SHUTDOWN+RTC
 *  instead of STANDBY for the deepest power saving (< 1 µA vs ~2 µA).
 *  SHUTDOWN loses SRAM2 retention so anything needed at next wake
 *  MUST already be in NVM or the retention NOLOAD section here.
 *
 *  Does not return on success (chip enters STANDBY or SHUTDOWN).
 */
void MGR_LPM_UW_tryAutoCycle(int sws_state, bool gesture_busy, bool config_mode);

/** @brief Mark the first MONITORING entry of a boot — starts the
 *  stabilization timer that gates `tryAutoCycle`. Subsequent
 *  re-entries are no-ops. */
void MGR_LPM_UW_markMonitoringEntered(void);

/** @brief Get/set the persisted duty configuration. */
void MGR_LPM_UW_setDutyCfg(uint16_t uw_sleep_s, uint16_t surf_sleep_s,
                          uint8_t enabled);
void MGR_LPM_UW_getDutyCfg(uint16_t *uw_sleep_s, uint16_t *surf_sleep_s,
                          uint8_t *enabled);

#ifdef __cplusplus
}
#endif

#endif /* MGR_LPM_UW_H */
