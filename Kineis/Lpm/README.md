# Kineis/Lpm — Low-Power Management Layer

## Purpose
Two-layer low-power-mode (LPM) management for the STM32WL55. A generic, portable
*aggregator* ([mgr_lpm.c](Kineis/Lpm/Src/mgr_lpm.c)) collects each registered
client's deepest acceptable mode, picks the shallowest, notifies clients, and
enters it. An STM32WL55-specific *overlay* ([lpm.c](Kineis/Lpm/Src/lpm.c))
provides the actual SLEEP/STOP2/STANDBY/SHUTDOWN enter/exit sequences, wake
configuration, RAM retention wiring, RTC tick compensation, and the
STOP-over-SPI grace window. [lpm_cli_kstk.c](Kineis/Lpm/Src/lpm_cli_kstk.c) is
the Kineis-stack client that derives the allowed mode from MAC resource status.

## Files
| File | Role |
| --- | --- |
| [Inc/mgr_lpm.h](Kineis/Lpm/Inc/mgr_lpm.h) | Generic API: `enum MgrLpm_LPM_t` (NONE/SLEEP/STOP/STANDBY/SHUTDOWN as a bitmap), `MgrLpm_EnvConfig_t` (allowed-mode bitmap + enter/exit callbacks), `MgrLpmClientCb_t` (per-client req/notify callbacks), `MgrLpm_ctxt_t`. |
| [Src/mgr_lpm.c](Kineis/Lpm/Src/mgr_lpm.c) | Aggregator. `MGR_LPM_init`, `MGR_LPM_registerClient`/`unregisterClient`, `MGR_LPM_enter`. Computes deepest client mode, masks it against the allowed bitmap, dispatches to the per-mode `vMGR_LPM_enter*` helpers, which call the platform callbacks and the HAL enter functions. |
| [Inc/lpm.h](Kineis/Lpm/Inc/lpm.h) | STM32WL55 overlay API: `LPM_init`, `LPM_enter`, `LPM_forceMode`/`LPM_getMode`, `LPM_setForcedMode`/`LPM_getForcedMode`, `LPM_shutdownNow`, `LPM_shutdownWithAutoWake`, `LPM_saveRtcTime`/`LPM_compensateTick`, `LPM_spiStopGraceActive` (SPI only). |
| [Src/lpm.c](Kineis/Lpm/Src/lpm.c) | Platform implementation: `lpm_config` (the env config), the `LPM_*_enter`/`LPM_*_exit` callbacks, clock/GPIO/wake setup, retention sections, sealed-deploy SHUTDOWN paths. |
| [Inc/lpm_cli_kstk.h](Kineis/Lpm/Inc/lpm_cli_kstk.h) | Kineis-stack client API + `mgrLpmCliKstk` callback struct. |
| [Src/lpm_cli_kstk.c](Kineis/Lpm/Src/lpm_cli_kstk.c) | `KSTK_lpmReq` maps `KNS_MAC_getRsrcStatus()` → allowed LPM; `KSTK_lpmNotifEnter`/`Exit` (no-op stubs returning true). |

## Key flows / data structures

### Mode bitmap and selection
`enum MgrLpm_LPM_t` is a **monotonic bitmap** sorted shallow→deep:
`NONE=0x00, SLEEP=0x01, STOP=0x02, STANDBY=0x04, SHUTDOWN=0x08`. Numerical
ordering is load-bearing — `eMGR_LPM_clientRequest()` takes the numeric **min**
across clients (deepest allowed by all), and `MGR_LPM_enter` then walks the
chosen bit **right** (`>>1`) until it lands on a bit set in
`env_config.allowedLPMbitmap`, i.e. it degrades to the deepest *enabled* mode no
deeper than requested.

### Aggregator entry (`MGR_LPM_enter`, mgr_lpm.c)
1. Reject if `allowedLPMbitmap == 0`.
2. `deepest_lpm = eMGR_LPM_clientRequest()` (min over `fpMGR_LPM_LpmReqCb`).
3. Mask down to an allowed bit.
4. `vMGR_LPM_clientNotifyEnter` (forward order, clients `0..MAX`).
5. `switch` → `vMGR_LPM_enterSleep/Stop/StandBy/Shutdown`, each calling the
   matching `fp_*_enter` callback then the HAL enter (`HAL_PWR_EnterSLEEPMode`,
   `HAL_PWREx_EnterSTOP2Mode`, `HAL_PWR_EnterSTANDBYMode`,
   `HAL_PWREx_EnterSHUTDOWNMode`).
6. On wake (SLEEP/STOP only — STANDBY/SHUTDOWN cold-boot) `fp_*_exit` runs, then
   `vMGR_LPM_clientNotifyExit` (reverse order).

Note: on STM32WL55 "STOP" is `HAL_PWREx_EnterSTOP2Mode` (STOP1 is commented
out). Each deep-mode entry forces `PWR->C2CR1.LPMS = SHUTDOWN` so the CPU2
LPMS floor (sticky across all but POR, observed polluted to Stop0 on the bench)
cannot silently degrade the system mode.

### Kineis-stack client (`KSTK_lpmReq`, lpm_cli_kstk.c)
- If a forced mode is set (`LPM_setForcedMode`) and no radio op is in flight,
  return it; if `modemOn`/`txTimeout`/`rxTimeout`/`satDetTimeout` is set,
  downgrade to SLEEP so timers/SRAM survive.
- Otherwise: `rsrc.raw == 0` → SHUTDOWN; `txTimeout` → SLEEP (TX-timeout timer
  TIM16 only wakes from SLEEP, not STOP); else STANDBY.

### Forced-mode override
`LPM_setForcedMode` sets a file-static consulted by `KSTK_lpmReq`. Needed to
reach SHUTDOWN/STANDBY in practice because the MAC keeps `keepKnsCtxt=1` once
initialized. The forced mode is still masked by `allowedLPMbitmap` in
`MGR_LPM_enter`, so it must also be compiled-in to actually be entered.
`LPM_forceMode` is different — it directly overwrites the stored `lpm_ctxt`
mode (used to backup/restore mode across LPM), not the client request.

### RTC tick compensation
SysTick is dead in STOP. `LPM_stop_enter` calls `LPM_saveRtcTime()`; `LPM_stop_exit`
calls `LPM_compensateTick()`, which reads the RTC delta and adds it to `uwTick`
so all `HAL_GetTick`-based software timers keep wall-clock time across sleep.
The pair is exported because MGR_LPM_UW reuses it. `LPM_compensateTick` guards
against bogus deltas (>24 h → ignored).

### Sealed-deployment SHUTDOWN (`LPM_shutdownWithAutoWake`)
Single-place teardown for app code entering SHUTDOWN outside the aggregator
(magnet gesture, boot-loop PERMANENT_OFF, low-battery). It runs `LPM_shutdown_enter`,
disarms any leftover RTC wake-up timer, then either kills every wake source
(`wakeup_seconds == 0`, true power-off) or arms the RTC wake-up timer
(1..131072 s, 16-bit or 17-bit CK_SPRE) and holds `PWR_LATCH` (PB7) HIGH so the
board stays powered until the timer cold-boots it. It also gates on RTC liveness:
if `HAL_RTC_WaitForSynchro` fails (LSE dead) it arms the LSI fallback and resets
rather than entering an unwakeable SHUTDOWN. `LPM_shutdownNow()` ==
`LPM_shutdownWithAutoWake(0)`.

## Integration
- **Init/use:** `LPM_init()` calls `MGR_LPM_init(lpm_config)` and registers
  `mgrLpmCliKstk`. The main idle loop calls `LPM_enter()` →
  `MGR_LPM_enter(lpm_config, &lpm_ctxt)`.
- **`LPM=` build flag** ([Makefile](Makefile)): `NONE|SLEEP|STOP|STANDBY|SHUTDOWN`,
  default `SHUTDOWN`. The Makefile passes `-DLPM_$(LPM)_ENABLED`. In
  [lpm.c](Kineis/Lpm/Src/lpm.c) this **cumulatively** builds
  `allowedLPMbitmap` (e.g. `LPM=STANDBY` enables SLEEP|STOP|STANDBY). The whole
  ladder ships by default; runtime can still cap via `AT+LPM=<bitmap>`.
  `LPM_SHUTDOWN_ENABLED` also gates compilation of `LPM_shutdown_enter` and the
  real `LPM_shutdownWithAutoWake` body (else it falls back to `NVIC_SystemReset`).
- **`APP`:** `USE_UW_DOPPLER_APP` adds ADC deinit/reinit around STOP/STANDBY/SHUTDOWN,
  SubGHz radio teardown (`HAL_SUBGHZ_DeInit`, ~500 µA), VBAT divider gating, and
  extra SLEEP-mode peripheral clock enables (TIM16, SUBGHZSPI, GPIO, DMA, FLASH/SRAM).
  **UW_DOPPLER also runs its own duty-cycle path** —
  [MGR_LPM_UW](Kineis/App/Managers/MGR_LPM_UW/Src/mgr_lpm_uw.c) — independent of
  this aggregator (STANDBY_TIMED / SHUTDOWN_REED); it reuses `LPM_saveRtcTime`/
  `LPM_compensateTick` and the `.retentionRamNoload` section.
- **`COMM`/`USE_SPI_DRIVER`:** adds NSS(PA15)-EXTI wake from STOP, SPI re-arm in
  `LPM_stop_exit`, and the `LPM_spiStopGraceActive()` grace window. DOPPLER/
  UW_DOPPLER are UART-only and reject `COMM=SPI` (Makefile guard).
- **`BOARD`:** `SMD_STDALONE` (and `BSP_HAS_PWR_LATCH`/`REED_SWITCH`/`LED_RGB`/
  `VBAT_ADC`) changes GPIO analog masks, PWR-controller pulls, and the PB7
  power-latch behaviour. See [bsp](Kineis/Bsp) wiring notes.
- **`DEBUG`:** only affects `MGR_LOG_DEBUG` traces (e.g. STANDBY/SHUTDOWN markers).

## Retention model
Three distinct persistence tiers (see [STM32WL55XX_FLASH_APP.ld](STM32WL55XX_FLASH_APP.ld)):

| Section | Where | Lifetime | Used by |
| --- | --- | --- | --- |
| `.retentionRamData` | `.data2` in SRAM2, **copy-init from ROM** by `Sram2_Init` | re-initialised on every cold/factory boot | `lpm_config` |
| `.retentionRamNoload` | `.retention_noload` (NOLOAD) in SRAM2 | survives every software-class reset (IWDG/SFT/OBL/BOR/PIN); cleared only by VBAT loss; **no C init** (self-validate via magic+CRC) | MGR_LPM_UW `duty_cfg`, boot counter, SWS cal, crash forensics |
| `.lpmSection` | `.rtc_bkpr` (RTC backup registers, BKP0R) | survives STANDBY/SHUTDOWN/resets while VBAT present | `lpm_ctxt` (current LPM mode; low 4 bits must match `MgrLpm_ctxt_t`) |

## Gotchas / constraints
- **lpm.c is shared by ALL apps.** The RTC-liveness gate resets via plain
  `NVIC_SystemReset` (no MGR_ERR dependency, which is UW_DOPPLER-only).
- **STOP-over-SPI is fragile.** STOP wakes on NSS-EXTI only — the triggering SPI
  frame is *always lost*; the host must retry. `LPM_stop_exit` re-arms SPI before
  the UART-recovery `HAL_Delay(100)` and opens a 500 ms grace window
  (`LPM_spiStopGraceActive`) during which the idle loop must NOT re-enter STOP, or
  STOP is unrecoverable over SPI. STANDBY/SHUTDOWN are unaffected (they cold-boot).
- **STANDBY wake = WKUP3 (PB3), rising edge.** Enabling EWUP3 auto-disables PB3's
  internal pull (RM0453 §5.4) → PB3 floats unless an external pull-down or active
  master hold exists; noise can cause false/missed wakes.
- **Auto-wake SHUTDOWN must hold PB7 HIGH.** `LPM_shutdown_enter` defaults PB7 to
  pull-DOWN (true power-off); `LPM_shutdownWithAutoWake(>0)` overrides to pull-UP,
  otherwise the latch opens, VDD drops, the RTC backup domain dies, and the
  wake-up timer can never fire → permanent brick of a sealed unit.
- **Leftover RTC WUT bricks SHUTDOWN.** The MAC may arm the WUT; `LPM_shutdownWithAutoWake`
  always `HAL_RTCEx_DeactivateWakeUpTimer` first (observed ~0.2 s self-wake otherwise).
- **STOP "works once" trap.** `vMGR_LPM_enterStop` clears `PWR_FLAG_WU`/`WUFI`
  before entry so a stale sticky doesn't make WFI return immediately on the
  second entry (STOP is currently disabled in the production MONITORING profile —
  this is dormant hardening).
- **SLEEP needs explicit peripheral sleep-clocks.** `LPM_SystemClockConfig` zeroes
  all `APBxSMENR`/`AHBxSMENR`, then re-enables only the bits the active build needs
  (LPUART/RTC always; SPI/DMA/GPIO for SPI; TIM16/SUBGHZSPI/FLASH/SRAM for
  UW_DOPPLER). Missing one stalls the MAC and trips IWDG — root cause of the
  rolled-back 2026-06-06 SLEEP regression.
- **`PA_PSU_SEL`/VSEL must stay HIGH** through STANDBY/SHUTDOWN so the TPS63901
  regulator stays in 3V3 mode for the next wake; on STDALONE the external R11 pull
  is too weak alone, so the PWR controller pull-up is enabled.
