# Changelog

All notable changes for the `v2` release line (merge `v2` → `main`).
Format: Keep-a-Changelog-ish, grouped by area. Dates are ISO. Firmware AT+VERSION reports `v0.8.1`.

---

## [v2] — sealed-deployment release (merge of the `v2-fix-lpm` robustness line)

`v2` is a **clean backward-compatible superset of `main`**: same credential/flash layout, the
AT command set is a strict superset (no command removed), the radio/MAC behaviour is unchanged
(same closed libkineis), and the SPI driver is newer than `main`'s. Existing `main` integrators
can adopt `v2` without host-side changes. Update path: full SWD reflash (credentials preserved;
`main` has no in-app bootloader).

### Added — new application
- **`UW_DOPPLER`**: underwater Argos Doppler tracker for sealed ocean deployment (board
  `SMD_STDALONE`, `COMM=UART`, MAC profile `BASIC`). Surface/dive detection drives a duty-cycled
  TX schedule; epoxy-sealed, magnet-only field input. Locked to `BASIC` at compile + runtime.
  Compile-time restricted to `SMD_STDALONE` + `COMM=UART` (`#error` otherwise).

### Added — managers (12 new)
- `MGR_SWS` (salt-water switch dive/surface), `MGR_REED` (reed/Hall + power latch, debounce +
  TX blanking), `MGR_GESTURE` (magnet gestures: OPERATIONAL / CONFIG / SHUTDOWN), `MGR_LPM_UW`
  (UW duty-cycle low-power), `MGR_NVM` (config persistence, NVM v8 with migration + heal),
  `MGR_BAT` (battery + VREFINT true-VDDA), `MGR_ERR` (reset-cause, crash-loop backstop, HardFault
  forensics), `MGR_WDG` (IWDG + option-byte guard), `MGR_EVTLOG`, `MGR_PMLOG` (post-mortem flash
  log), `MGR_RATE` (TX rate limiter), `MGR_LED` (RGB status + soft-PWM, 24 h auto-off),
  `MGR_CRED` (credential mirror + atomic restore).

### Added — power management
- Event-driven LPM scheduler with `AT+LPMTHR` (spin / STOP1+LPTIM / STOP2+RTC tiers).
- Production path: **STOP2 + RTC** timed wake; magnet power-off via **STOP2 soft-off + reed wake**
  / RTC auto-wake. Deadline-based wall-clock scheduling across STOP modes.
- CPU2 LPMS floor forced open before every deep-sleep entry (was silently degrading SHUTDOWN).
- 49.7-day HAL tick-wrap survival (rate limiter + SWS lockout).

### Added — bootloader / OTA
- In-app DFU bootloader (UART + SPI) at `0x08033000`, app shrunk to 200 KB; `FLASH_USER`
  credentials preserved at `0x0803B000`. Optional `BL_LED`, configurable DFU baud.

### Added — TX payload & telemetry
- Minimal F.6 Doppler payload + episode stats (`AT+PAYCFG`, `AT+STATS`, `AT+TXSTATS`, `AT+LPMSTAT`).

### Added — AT commands (~40, superset of `main`)
- `AT+BATCFG AT+BOOT AT+DEPLOY AT+DIAG AT+DL AT+DPLCFG AT+DPLWKU AT+DUTYCFG AT+GNSS AT+KCFG
  AT+KEVT AT+LB AT+LBCFG AT+LED AT+LOG AT+LOGLVL AT+LPMSTAT AT+LPMTHR AT+MODE AT+PAYCFG AT+PMLOG
  AT+PREPASS_EN AT+RATE AT+RATECFG AT+RATECLEAR AT+RCONFRAW AT+RESET AT+SAVE AT+SHUTDOWN
  AT+STANDBYTEST AT+STATS AT+STATUS AT+STOPTEST AT+SWS AT+SWSCFG AT+SWSFORCE AT+TEST AT+TXCFG
  AT+TXSTATS AT+UARTLOG` (all 17 `main` commands remain).

### Hardening — sealed-deploy brick fixes
- **#1** reed TX-chatter brick (driver blanking during TX) + power-off path redesign.
- **#2** LSE→LSI fallback so a dead 32 kHz crystal can't brick boot.
- **#3** STOP2 wake-arm check + auto-wake latch.
- **#4** credential durability: page-0 mirror @`0x08032800` + atomic `MGR_CRED_syncAndRestore`.
- MC enforced to 9-bit protocol range (clamp + `AT+MC` modulo) — fixes >511 frame corruption.
- Bootloader flash-safety guards (v3.7.0): `BL_STATE` persistence compiled out (was aliasing the
  credentials page), DFU app-erase clamped to `APP_FLASH_SIZE=0x32000` (spares the FLASH_PMLOG /
  CRED mirror), DFU write end-bound. Bootloader can no longer erase the credentials.

### Fixed — 2026-06-20/21 total audit (3 CRITICAL + others)
- **CRITICAL** `MGR_ERR` crash-loop safe-sleep entered STOP2 with the wrong EXTI line and no PWR
  internal wake line → never woke → the last-resort backstop bricked a sealed unit. (`b6fb604`)
- **CRITICAL** `MCU_NVM_getID`/`getAddr` read 8 bytes into a 4-byte caller object → 4-byte stack
  overflow on every ID/ADDR read (on the live MAC-init path). (`6e83138`)
- **CRITICAL** `AT+TX` attribute scanned with `%hX` into a 1-byte union → 1-byte overflow.
- `KNS_CS_exit` underflow guard; stray `subghz` wrong-IRQ NVIC enable removed; `qIdx2Str` missing
  comma (DEBUG NULL-deref). (`66cfd55`, `fcf5471`)
- GUI/SMD_PA: STANDBY radio de-init (~500 µA floor) + SPI re-arm after STOP wake. (`1952535`, `a7708fa`)
- Build matrix: `DOPPLER+SPI` now compiles; `UW_DOPPLER+SPI` refused with a clear `#error`. (`ca219bd`)
- `AT+SWSCFG` rejects `max_dive_time_s=0` (+ NVM heal) so the stuck-underwater backstop is never
  disabled. (v3.7.0)
- TX-timeout made coherent: the effective 2× safety margin (TCXO warmup + ~1.7 s TX) is now
  documented + locked, with an on-change `[TIM] TX_TIMEOUT req/eff/TCXO_warmup` diagnostic. No
  behaviour change. (`9f020a3`, `7b9a315`)

### Changed — behaviour notes for `main` users
- `AT+MC` now folds to the 9-bit range (mod 512). Only differs from `main` for the previously
  frame-corrupting `MC > 511` case (a fix).
- `AT+VERSION`: `v0.6` → `v0.8.1`.

### Tests
- Unit suite: **40 suites / 519 checks** (git-bash gcc; `cd Tests && ./scripts/run_tests.sh`).
  Includes `test_audit_fixes` regression locks for the audit fixes.
- Build matrix: all valid `APP × BOARD × COMM × DEBUG` combinations compile (UW_DOPPLER is
  STDALONE+UART-only by design).

### Known limitations / pre-seal bench gates (UW_DOPPLER)
- **[bench]** STOP2 µA soak vs the ~12-month budget (floor ~23 µA on STDALONE is HW-bound: active
  Hall on VBAT + TPS63901 Iq; not firmware-reducible without an HW strap).
- **[bench]** Biofouling proximity/threshold sweep on the SWS detector.
- **[residual]** LSE crystal death **during** a STOP2 sleep (RTC-only wake, IWDG frozen) — low
  probability, no software-only fix; the LSE→LSI latch covers death-at-boot, not mid-sleep.
- **[bench, optional]** `MGR_ERR` snapshot-based silent-IWDG counting + RAM2-retention-on-fault-reset
  are DEFERRED (the conservative always-wipe / not-counted behaviour ships). A `[ERR] true reset
  cause masked by RMVF` WARN was added so silent IWDG hangs are visible at the bench before deciding.
- **[pre-seal]** Dump and verify the persisted NVM config (no operator mis-config sealed in).
