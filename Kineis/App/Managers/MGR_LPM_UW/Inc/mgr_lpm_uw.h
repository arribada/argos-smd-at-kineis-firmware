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
 * KEPT FOR REFERENCE / AT+STANDBYTEST ONLY. The auto-cycle path uses
 * MGR_LPM_UW_enterStop2Timed instead — STOP2 gives the same ~2 µA
 * floor without the 6 s MAC re-init penalty on every wake, AND
 * supports EXTI wake on the reed switch (PB6) which is not a WKUP
 * pin and therefore cannot wake STANDBY on this PCB.
 *
 * @param seconds  Wake interval (1..65535 s on the 16-bit CK_SPRE
 *                 prescaler — covers up to 18.2 hours per cycle).
 */
__attribute__((noreturn))
void MGR_LPM_UW_enterStandbyTimed(uint32_t seconds);

/** @brief Enter STOP2 mode for up to `seconds` seconds, then RETURN.
 *
 * Unlike STANDBY this does NOT cold-boot the chip on wake. Execution
 * resumes from inside this function after the wake event, with all
 * RAM (including the MAC stack in .knsCtxtData), peripheral state and
 * the state machine intact. This is the production duty-cycle path
 * on SMD_STDALONE.
 *
 * Wake sources, in priority order:
 *   - RTC wake-up timer at the configured `seconds`
 *   - EXTI on the reed switch (PB6) — magnet detection
 *   - Any other EXTI line left armed (LPUART RX, gesture FSM, etc.)
 *
 * ~2 µA in STOP2 + a few µs of awake overhead per wake. Compared to
 * STANDBY this saves the ~6 s of cold-boot + MAC re-init energy per
 * cycle, which dominates the deployment-year budget.
 *
 * @param seconds  Maximum wake interval (1..65535 s). Pass 0 to enter
 *                 STOP2 with no RTC alarm (only EXTI wakes).
 */
void MGR_LPM_UW_enterStop2Timed(uint32_t seconds);

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

/** @brief Sleep duration ≥ this value picks SHUTDOWN+RTC over STANDBY in
 *  the auto-cycle. Set to 0 to always use STANDBY. Default 300 s. */
void     MGR_LPM_UW_setShutdownThreshold(uint16_t seconds);
uint16_t MGR_LPM_UW_getShutdownThreshold(void);

/** @brief Compare current SWS state with the persisted last-state from
 *  the previous STANDBY entry. Returns true if a UW→SURFACE transition
 *  is detected (previous was UNDERWATER=0, current is SURFACE=1) — the
 *  caller should fire an immediate TX on this wake cycle.
 *
 *  Also updates the persisted last_sws_state so the next cycle's check
 *  sees this state as the baseline. */
bool MGR_LPM_UW_detectSurfaceWake(int current_sws_state);

/** @brief True if the last duty cycle decided we should TX on this wake.
 *  Set by detectSurfaceWake; consumed by the app's wake-init branch. */
bool MGR_LPM_UW_isWakeShouldTx(void);

/** @brief Acknowledge / clear the wake_should_tx flag after consumption. */
void MGR_LPM_UW_clearWakeShouldTx(void);

/** @brief True if this boot is a cold-boot from STANDBY (PWR_SR1.SBF was
 *  set on boot). False on POR / IWDG / SW / PIN / SHUTDOWN-wake.
 *
 *  Reads the snapshot captured by main() before HAL clears the flags
 *  (`g_boot_pwr_sr1_raw`). Stable across the whole boot lifetime.
 *
 *  Use case: app can branch on this to skip work that's only meaningful
 *  on a true cold boot (e.g. some MAC re-init steps, NVM re-load) when
 *  the previous cycle's SRAM2 is still valid. */
bool MGR_LPM_UW_isWakeFromStandby(void);

#ifdef __cplusplus
}
#endif

#endif /* MGR_LPM_UW_H */
