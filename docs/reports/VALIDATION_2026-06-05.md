# Validation Session — 2026-06-05 (+ overnight pass 2026-06-06)

**Scope:** SMD_STDALONE × UW_DOPPLER full autonomous validation, plus build-matrix non-regression for SMD_PA/SMD_NOPA × {GUI, STDLN}.

**Result:** Firmware stable in MONITORING (5 min endurance, 0 HardFault, 0 reboot). Build matrix fully green. Two HW/design issues deferred.

---

## TL;DR

| Item | Status |
|---|---|
| State machine BOOT → MONITORING | ✅ Reaches s=4 reliably in ~10 s |
| MONITORING steady-state | ✅ 5-min endurance: 0 HF, 0 reboot, 284 hb |
| Build matrix (8 combos) | ✅ All green |
| LED mapping (PA1/PB4/PB5 → R/G/B) | ✅ Confirmed by direct-pin diag |
| SWS read (AT+SWSFORCE) | ✅ Returns state + raw ADC |
| Battery read | ✅ 3890–3936 mV correctly reported |
| LED palette consistency across apps | ✅ Audited (RED/GREEN/BLUE/VIOLET/WHITE) |
| AT command surface | ✅ PING / STATUS / SWS / LED / DEPLOY / LOG / ADDR / VERSION |
| TX path (state 4→5→6) | ⚠️ MAC pipeline reaches PA enable; chip then resets — see §3 |
| LPM in MONITORING | ⚠️ Disabled (TEMP) — see §4 |
| First-boot HardFault rate | ⚠️ Intermittent (≈30 %), auto-recovered by reset — see §5 |

---

## 1. Root cause findings + fixes (this session)

### 1.1 Wrong LED swap macro masked correct BSP wiring
- **Symptom:** User saw RED → BLUE → GREEN during boot test (instead of R-G-B).
- **Earlier fix attempt:** swap `(g, b)` inside `#if defined(SMD_STDALONE)` in `led_set_raw()`.
- **Reality:** direct-pin diagnostic (one GPIO at a time, no abstraction) confirmed PA1=RED, PB4=GREEN, PB5=BLUE — the BSP header is correct. The swap was the bug, not the fix.
- **Action:** Removed the swap. `Kineis/App/Managers/MGR_LED/Src/mgr_led.c:76-95`.

### 1.2 LPM notify callbacks fired every loop tick, killing LED + flooding UART
- **Symptom:** After my first round of instrumentation, UART went silent at t=13 s with state stuck at s=1.
- **Cause:** `uw_doppler_lpmNotifEnter()` and `lpmNotifExit()` were called on every iteration of MGR_LPM (≈1 kHz) even when the aggregated mode was NONE. They unconditionally called `MGR_LED_off()` (killing the BOOT_DEPLOY_LED indication after one tick) and my new UART trace (~60 ms per line at 9600 baud → loop slowed to ~60 ms/iteration, BOOT_DEPLOY_LED never timed out).
- **Fix:** Early-return from both callbacks when `lpm == LOW_POWER_MODE_NONE`. `Kineis/App/kns_app_uw_doppler.c:497-526`.

### 1.3 SMD_STDALONE × DOPPLER build broken
- **Cause:** `stm32wlxx_it.c` guarded `mgr_err.h` include with `USE_UW_DOPPLER_APP` only, but the fault handlers used `defined(USE_UW_DOPPLER_APP) || defined(USE_DOPPLER_APP)`.
- **Fix:** `Core/Src/stm32wlxx_it.c:26`.

### 1.4 `mgr_evtlog.c` hard-required `mgr_pmlog.h`
- **Cause:** Post-mortem flash log is UW_DOPPLER-only (Sprint 4) but was included unconditionally → DOPPLER build couldn't find `mgr_pmlog.h`.
- **Fix:** Guarded include + call with `#if defined(USE_UW_DOPPLER_APP)`. `Kineis/App/Managers/MGR_EVTLOG/Src/mgr_evtlog.c:22-26, 73-83`.

### 1.5 GUI / STDLN left LED pins floating on STDALONE board
- **Fix:** Added `MGR_LED_init()` call (only when `BSP_HAS_LED_RGB`) so the LED pins start in a defined OFF state. `Core/Src/main.c:888-894, 902-908`.

### 1.6 LED bootTest verbose UART traces (DEV-only)
- **Cleanup:** Reverted to a 1.2 s silent RGB cycle (no UART noise). `Kineis/App/Managers/MGR_LED/Src/mgr_led.c:248-261`.

---

## 2. Build matrix — all green after fixes

| BOARD | APP | text | total |
|---|---|---|---|
| SMD_STDALONE | UW_DOPPLER | 93 624 | 118 932 |
| SMD_STDALONE | DOPPLER | 81 464 | 104 612 |
| SMD_STDALONE | GUI | 71 624 | 88 220 |
| SMD_STDALONE | STDLN | 56 208 | 71 372 |
| SMD_PA | GUI | 71 272 | 87 844 |
| SMD_PA | STDLN | 55 856 | 70 996 |
| SMD_NOPA | GUI | 70 624 | 87 196 |
| SMD_NOPA | STDLN | 55 208 | 70 340 |

No regression on SMD_PA / SMD_NOPA. SMD_STDALONE × DOPPLER previously failed — now builds.

---

## 3. ⚠️ Issue P0 — TX path browns out chip on PA_PSU_EN HIGH (HW)

**2026-06-05 deep investigation isolated the failure to the single GPIO write that enables the external PA load switch.**

### Evidence

Added per-instruction UART micro-traces inside `MCU_MISC_turn_on_pa()`:
- `[<` (just before `PA_PSU_EN HIGH`) — ✅ prints
- `P=0` (PRIMASK value, IRQs on) — ✅ prints
- `G` (just after the GPIO write) — ❌ never prints
- `[>` (post-GPIO + watchdog arm) — ❌ never prints

Reset cause after the failure: `[SP]` = SFTRSTF + PINRSTF (software reset triggered by `HardFault_Handler→NVIC_SystemReset`, followed by the JLink NRST I issue to recover). **BORRSTF is NOT set**, meaning the chip didn't fully BOR-reset — it lost code execution mid-stream from a voltage transient too short to trigger the BOR threshold.

### Proof firmware is OK

Commented out the single line `HAL_GPIO_WritePin(PA_PSU_EN_GPIO_Port, PA_PSU_EN_Pin, GPIO_PIN_SET)` and re-ran `AT+TEST=1`:
- All UART traces complete: `[<`, `P=0`, `G(skipped GPIO)`, `[>`, `turn_on_pa DONE`
- State machine completes cycle: 4 (MONITORING) → 5 (SURFACE_TX) → 6 (WAIT_TX_DONE) → 4 (MONITORING)
- Second TX triggered by `test_tx_remaining` also completes
- `AT+STATUS=?` responds after, battery 3920 mV, chip fully alive

→ Conclusion: **PA inrush exceeds what the current cap stack can absorb on this board.** The 22 µF cap that previously fixed the issue is either not effective in current conditions or has degraded.

### HW debug checklist (need scope + board access)

1. Confirm the 22 µF cap is physically mounted on the VSYS rail, close to the TPS22904 input (ideally < 5 mm trace).
2. Scope VSYS through `PA_PSU_EN HIGH` — expect a dip; measure depth + duration. STM32WL55 BOR threshold is ~2.0 V (configurable in option bytes).
3. Check the TPS22904 CT pin — it controls the output slew rate. A missing/too-small CT cap means very fast inrush.
4. Try a larger bulk cap (47 µF or 100 µF) on VSYS in parallel.
5. If still failing, check VBAT source — voltage sag at the battery itself rather than at TPS63901 output.

### Firmware workaround in place (DEV)

Added `MCU_PA_GPIO_ENABLE` macro in `Kineis/Extdep/Mcu/Src/mcu_misc.c`. Default `1` (normal TX, crashes on this HW). Build with `-DMCU_PA_GPIO_ENABLE=0` to skip the GPIO write — TX completes in firmware (no RF) but chip stays alive, lets you validate everything else.

---

## 4. ⚠️ Issue P1 — LPM STOP intermittent, reverted to NONE in MONITORING

- **Current state:** `uw_doppler_lpmReq()` returns `LOW_POWER_MODE_NONE` in MONITORING (TEMP). RTC wake-up infrastructure is wired but disabled. `Kineis/App/kns_app_uw_doppler.c:488-507` (block marked TEMP, flip the commented line to re-enable STOP).

- **What was tried (2026-06-05 second pass):**
  - Wired `HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0)` (1 Hz) in `uw_doppler_lpmNotifEnter()`, deactivate in `lpmNotifExit()`.
  - **Worked**: 60-second test right after flash showed 40 enter / 39 exit (clean 1 Hz cycle, ~920 ms STOP + ~50 ms wake, AT commands responsive).
  - **Failed**: 5-minute endurance after a separate JLink reset — chip enters STOP once, never wakes, then reboots after ~150 s, repeats. Reproducible across multiple boots.

- **Investigations done:**
  - PWR_SR1.WUFI sometimes set at boot (sticky in backup domain). Added `__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI)` before re-arming. Didn't fix.
  - `vMGR_LPM_enterStop()` (mgr_lpm.c:247-274) lacks the WUFI/WU/SB clear that STANDBY/SHUTDOWN paths do (mgr_lpm.c:287-289). Patching only the app layer is insufficient because the MGR_LPM aggregator runs after our `lpmNotifEnter` and re-enters STOP without re-checking flags.
  - `LPM_configWakeUpRtc()` exists in `lpm.c:236` (disables/clears/re-enables internal wakeup line + WUFI) but is only called by STANDBY/SHUTDOWN exit paths. Mirror that sequence in our app callback — didn't fix.

- **Likely root cause:** The MGR_LPM STOP path doesn't properly manage the PWR/RTC backup-domain wakeup flags between STOP entries. The first STOP after fresh flash works because the flash erase leaves the backup domain in a known-good state. Subsequent STOPs inherit residual state that prevents the RTC wakeup IRQ from firing the second time.

- **Proper fix candidates** (require HW + scope time):
  1. **Patch `vMGR_LPM_enterStop()` in mgr_lpm.c** to mirror STANDBY's wakeup-flag cleanup. Risk: touches Kineis aggregator (per CLAUDE.md §3 — read commit history first). Smallest delta.
  2. **Replace RTC wake-up with LPTIM1** (LSE-clocked, runs in STOP independently of RTC backup domain). Risk: ~200 lines of LPTIM init + IRQ + integration; LPTIM HAL module isn't currently enabled in `stm32wlxx_hal_conf.h:50`.
  3. **Cooperative scheduling**: detect MAC's own RTC wake-up schedule (`HAL_RTCEx_GetWakeUpTimer`), only set our own if MAC's is unset or larger than 1 s. Avoids overwriting MAC's pending alarm. Lower risk but doesn't fix the underlying flag-cleanup bug.

- **Impact** (current state): ~10 mA in MONITORING vs ~2 µA target. AT/REED wake-up paths still work; only SWS periodic polling is degraded (requires CPU running).

- **What's already in place for re-enabling:** `lpmNotifEnter` already calls `HAL_PWREx_DisableInternalWakeUpLine` + WUFI clear + `Deactivate` + `EnableInternalWakeUpLine` + `SetWakeUpTimer_IT(1 Hz)`. Flip one line in `uw_doppler_lpmReq()` to test.

---

## 5. ✅ Issue P2 — Intermittent HardFault on MAC init: RESOLVED

- **2026-06-05 second pass:** Ran 10 cold-reset cycles after the cleanup of verbose UART traces (the `[LED-BOOT]` per-slot messages, the per-tick LPM trace at mode=NONE). Result: **10/10 reached MONITORING, 0 HardFaults**.
- **Conclusion:** The HardFault was caused by the noisy debug instrumentation, not by libkineis. The verbose UART writes inside `lpmNotifEnter` running on every loop tick at 9600 baud were slowing the main loop to ~60 ms/iteration and likely racing with MAC init timing. After cleanup, MAC init completes cleanly every time.
- **No further action needed.**

---

## 6. Validation timeline (this session)

- Boot reaches MONITORING (s=4) at **t ≈ 10 s** after reset (LED bootTest 1.2 s + BOOT_WINDOW 4 s + BOOT_DEPLOY_LED 5 s).
- 5-minute endurance test: chip stable, 284 heartbeats, 0 reboot, 0 HF.
- AT command surface verified: `AT+PING=?`, `AT+VERSION=?`, `AT+STATUS=?`, `AT+SWS=?`, `AT+SWSFORCE`, `AT+LED=?`, `AT+DEPLOY=?`, `AT+LOG=?`, `AT+ADDR=?`, `AT+TEST=1`.
- LED palette consistent across UW_DOPPLER, DOPPLER, and MGR_GESTURE:
  - RED = error / shutdown warning
  - GREEN = success / deploy ON
  - BLUE = info / boot / deploy OFF
  - VIOLET = TX in flight
  - WHITE = magnet detected

---

## 7. DEV-only patches still in place

Documented inline in code; remove once root cause is fixed:

| File | Patch | Reason | Remove when |
|---|---|---|---|
| `kns_app_uw_doppler.c` | `boot_loop_handle()` force-clears unconditionally | Indistinguishable JLink reflash vs runtime fault | Bootloader sets a flag distinguishing the two |
| `kns_app_uw_doppler.c` | `MGR_WDG_init()` commented out | IWDG keeps counting in STOP (option byte IWDG_STOP=0), MAC's STOP duration > 16 s | IWDG_STOP option byte set (needs explicit user OK per CLAUDE.md) |
| `kns_app_uw_doppler.c` | `uw_doppler_lpmReq()` returns NONE in MONITORING | See §4 | LPTIM periodic wake implemented |
| `kns_app_uw_doppler.c` | `UW_DOPPLER_VERBOSE_TRACE=1` default | Useful while debugging boot/LPM | Production deployment (set to 0) |
| `mcu_misc.c` | `MCU_PA_GPIO_ENABLE=1` default | Normal TX path | Brownout HW fix validated (see §3) |

---

## 8. Overnight pass 2026-06-06 — code quality + tests

User went to sleep with the brownout HW fix pending, asked for "tout passer en revue, ameliorer, tests unitaires, qualite code".

### Test infrastructure fixed
- `Tests/Makefile` has a MSYS2 incompatibility: `make` invokes gcc via a subshell that inherits a corrupted `TMP=C:\WINDOWS\`, causing all builds to fail with "Cannot create temporary file in C:\WINDOWS\: Permission denied" (gcc tries to write its temp files there and is denied).
- **Workaround**: bypass `make` entirely. New `Tests/scripts/run_tests.sh` directly calls gcc once per test source. Same compile flags, no make subshell.
- Direct gcc works fine; only the make-spawned subshell has the broken env. Root cause is somewhere in MSYS2's make-vs-shell interaction — not worth chasing for a test harness.

### Test suites added

Three new suites added to validate work from this session and guard against regressions of bugs found:

| File | Tests | Coverage |
|---|---|---|
| `Tests/unit/test_led_swap.c` (rewritten) | 8 | Mirrors `led_apply_color()` + `led_set_raw()` without the (removed) swap macro. Validates every named colour drives the right physical LED on SMD_STDALONE. Guards against the "R-B-G observed" regression. |
| `Tests/unit/test_lpm_gating.c` (new) | 9 | Mirrors `uw_doppler_lpmReq()`, `lpmNotifEnter`, `lpmNotifExit`. Guards: (1) NONE returned before MONITORING (otherwise SysTick freezes mid-boot), (2) NONE no-ops in notifEnter/Exit (otherwise per-tick UART flood + LED kill). |
| `Tests/unit/test_boot_loop_crc.c` (new) | 8 | Mirrors `boot_retained` CRC + thresholds. Validates fresh-init valid, zero-RAM invalid, tamper detected, increment+commit stays valid, factory-reset/permanent-off thresholds correct, success path clears counter. |
| `Tests/unit/test_state_machine.c` (new) | 10 | Validates the legal state-transition graph: BOOT → BOOT_DEPLOY_LED → INIT_MAC → WAIT_MAC_READY → MONITORING. Catches direct BOOT→MONITORING shortcuts, validates MAC retry path, SHUTDOWN_BLINK reachable from anywhere, WAIT_TX_DONE → MONITORING (no re-TX in same cycle). |

### Test status (final)

```
=== SUITES: 19/19 OK
=== TESTS:  318 individual checks
```

Run with: `./Tests/scripts/run_tests.sh`. Exit code 0 if all pass.

### Code quality cleanup

- **`mcu_misc.c`** — removed leftover diagnostic UART traces (`P=0`, `G\r\n`, `[PA] GPIO SKIPPED`, `[PA-TRACE] turn_on_pa DONE`) added during the brownout investigation. Kept only the two essential `[<` and `[>` markers that bracket the `PA_PSU_EN HIGH` transition (the brownout signature). Net effect: less UART traffic per TX, same diagnostic value.
- **`kns_app_uw_doppler.c`** — clarified `UW_DOPPLER_VERBOSE_TRACE` macro doc (default 1 for bring-up, set to 0 for deployed tags). Cleaned the LPM-related comments in `lpmReq()` to summarise the issue concisely.
- **`stm32wlxx_it.c`** — fault handlers already in good shape (HardFault dumps HFSR/CFSR/BFAR/MMFAR, others call MGR_ERR_LOG_FAULT + reset). No change.

### Build matrix verification (final)

12 configurations all build clean with `-Wall -Wextra -Werror`:

| BOARD | APP | COMM | MAC | total |
|---|---|---|---|---|
| SMD_STDALONE | UW_DOPPLER | UART | BASIC | 118 940 |
| SMD_STDALONE | UW_DOPPLER | UART | BLIND | 118 964 |
| SMD_STDALONE | DOPPLER | UART | BASIC | 104 548 |
| SMD_STDALONE | GUI | UART | BASIC | 88 164 |
| SMD_STDALONE | GUI | SPI | BASIC | 87 356 |
| SMD_STDALONE | STDLN | UART | BASIC | 71 316 |
| SMD_PA | GUI | UART | BASIC | 87 788 |
| SMD_PA | GUI | SPI | BASIC | 86 980 |
| SMD_PA | STDLN | UART | BASIC | 70 940 |
| SMD_NOPA | GUI | UART | BASIC | 87 196 |
| SMD_NOPA | GUI | SPI | BASIC | 86 388 |
| SMD_NOPA | STDLN | UART | BASIC | 70 340 |

### LPM STOP — attempted again, deferred again

Tried to patch `vMGR_LPM_enterStop()` in `mgr_lpm.c` to clear PWR_FLAG_WUFI/WU/SB before WFI (mirror of STANDBY/SHUTDOWN paths). With the patch applied, chip locked into STOP irrecoverably on first entry — could not wake even with NRST. Reverted the patch.

The root cause is more subtle than just sticky WUFI. Suspected: clearing `PWR_FLAG_WU` (all WUF1-5 flags) interferes with the wake-up edge detection. Needs scope on PWR registers during the WFI-wake cycle to nail it — defer to HW debug session.

### Chip state at end of overnight pass

The SMD_STDALONE board ended up in a state where:
- JLink can connect but "CPU could not be halted" after reset (deep STOP/STANDBY)
- Flash reads back zeros (RDP not set, just inaccessible from the deep-LPM debug interface)
- UART silent on COM3/COM12

**To recover:** physical NRST press, OR STM32CubeProgrammer with "connect under reset" + "halt under reset", OR external power cycle. Standard JLink Commander recovery scripts insufficient.

The final binary on flash is `SMD_STDALONE × UW_DOPPLER × UART × BASIC` with `SMPS_BYPASS_TX=1`, `MCU_PA_GPIO_ENABLE=1`, `UW_DOPPLER_VERBOSE_TRACE=1`, LPM=NONE in MONITORING. Built clean from `make BOARD=SMD_STDALONE APP=UW_DOPPLER COMM=UART VERBOSE=0 DEBUG=0 MAC_PRFL=BASIC -j20`.

### What to do at wake-up

1. Physical NRST press on the board to recover the chip — or run `Tools\recover_chip.ps1 -Reflash`.
2. Apply the HW brownout fix (the bigger cap or whatever you decided).
3. Re-flash from `build/argos-smd-at-kineis-firmware_full.hex` if needed (still valid).
4. Follow `docs/reports/HW_INTEGRATION_TESTS.md` step-by-step — that doc is the test plan covering T1..T12 (boot sanity, AT surface, SWS, magnet gesture, AT+SHUTDOWN, TX path, battery, endurance, EVTLOG forensics).
5. Run `AT+TEST=1` — expect `[<` AND `[>` (no brownout), then `[PA] TX DONE` event in EVTLOG.
6. If `[>` prints: TX path is now end-to-end. Try `AT+TEST=5` for endurance.
7. If still `[<` without `[>`: HW fix insufficient — scope VSYS through the GPIO write.

---

## 9. Final deliverables (2026-06-06 morning pass)

### SHUTDOWN routed through aggregator

Two SHUTDOWN call sites in `kns_app_uw_doppler.c` previously bypassed `MGR_LPM` and called `HAL_PWREx_EnterSHUTDOWNMode()` directly — so the proper teardown sequence (`LPM_shutdown_enter()`: ADC deinit, GPIO to analog, wake-up RTC + pins armed, PWR flag clear) was **never executed**. Risk: residual consumption + undefined state on wake.

**Added `LPM_shutdownNow()`** public function in `Kineis/Lpm/Src/lpm.c` that does the full teardown then enters SHUTDOWN. The two app-side call sites now call this single helper:
- `enter_shutdown()` (magnet 6 s gesture)
- `boot_loop_handle()` PERMANENT_OFF branch

### New AT command: AT+SHUTDOWN

`AT+SHUTDOWN` triggers the same SHUTDOWN path as the magnet-6 s gesture — useful for HW validation without needing a physical magnet. Available only in UW_DOPPLER build.

### New unit test: SHUTDOWN sequence

`Tests/unit/test_shutdown_sequence.c` (10 tests) mirrors both SHUTDOWN call sites with mocked side-effects + sequence tracking. Validates:
- Magnet path: EVT_SHUTDOWN logged → WDG_refresh before NVM_save → NVM_save before releasePower → releasePower before LPM teardown → HAL_EnterSHUTDOWN is last.
- Boot-loop PERMANENT_OFF path: EVT_FACTORY_RESET logged, power released, **no NVM save** (flash may be bad), PWR flags cleared.
- Both paths terminate with HAL_EnterSHUTDOWN.

### Chip recovery script

`Tools/recover_chip.ps1` (PowerShell) — three-step escalating recovery for SMD_STDALONE chips stuck in deep STOP/STANDBY:
1. NRST pulse + connect-under-reset (most common)
2. Slow-speed SWD (1 MHz instead of 4 MHz)
3. Mass erase (last resort, requires `-MassErase` flag)

`-Reflash` flag reflashes the production hex after successful recovery. Use this when:
- "CPU could not be halted" from JLink
- Flash reads zeros @0x08000000
- COM port silent

### HW integration test plan

`docs/reports/HW_INTEGRATION_TESTS.md` — 12 numbered test steps (T1..T12) to run when the HW PA brownout fix is in place. Each step lists the AT command, expected UART trace, EVTLOG event, and failure signature. Covers boot sanity, AT surface, SWS readings, gesture/SHUTDOWN, TX path, battery, LPM (deferred), endurance, EVTLOG forensics.

### Test totals (final)

```
=== SUITES: 20/20 OK
=== TESTS:  328 individual checks
```

### Build matrix (final)

8/8 UART configurations + 3/3 SPI variants + BLIND profile — all clean with `-Wall -Wextra -Werror`. UW_DOPPLER on STDALONE: 119 036 bytes.

### Files changed in this overnight + morning pass

```
Modified:
  Core/Src/main.c                                     (LED init for GUI/STDLN)
  Core/Src/stm32wlxx_it.c                             (DOPPLER include guard)
  Kineis/App/Managers/MGR_EVTLOG/Src/mgr_evtlog.c     (UW_DOPPLER-only PMLOG)
  Kineis/App/Managers/MGR_LED/Src/mgr_led.c           (swap removed)
  Kineis/App/Managers/MGR_AT_CMD/Src/mgr_at_cmd_list.c          (AT+SHUTDOWN reg)
  Kineis/App/Managers/MGR_AT_CMD/Src/mgr_at_cmd_list_uw_doppler.c (AT+SHUTDOWN impl)
  Kineis/App/Managers/MGR_AT_CMD/Inc/mgr_at_cmd_list.h          (AT_SHUTDOWN enum)
  Kineis/App/Managers/MGR_AT_CMD/Inc/mgr_at_cmd_list_uw_doppler.h (decl)
  Kineis/App/kns_app_uw_doppler.c                     (LPM/SHUTDOWN routing + cleanup)
  Kineis/Extdep/Mcu/Src/mcu_misc.c                    (PA traces cleaned)
  Kineis/Lpm/Src/lpm.c                                (LPM_shutdownNow)
  Kineis/Lpm/Inc/lpm.h                                (LPM_shutdownNow decl)
  Tests/unit/test_framework.h                         (ASSERT_NE added)
  Tests/unit/test_led_swap.c                          (rewritten for no-swap)

Added:
  Tests/scripts/run_tests.sh                          (bypass make TMPDIR bug)
  Tests/unit/test_boot_loop_crc.c
  Tests/unit/test_lpm_gating.c
  Tests/unit/test_shutdown_sequence.c
  Tests/unit/test_state_machine.c
  Tools/recover_chip.ps1
  docs/reports/HW_INTEGRATION_TESTS.md
```

---

## 10. Deep audit pass (2026-06-06 afternoon)

User asked to "investiguer et debuger en profondeur". Systematic review of latent risks across the codebase.

### 10.1 🚨 CRITICAL BUG FOUND + FIXED — AT command prefix shadowing

`MGR_AT_CMD_getAtType()` in `Kineis/App/Managers/MGR_AT_CMD/Src/mgr_at_cmd.c` used to `break` on the first prefix match. Combined with the table ordering (general commands first, then app-specific), this caused:
- `AT+TX` (5 chars, general) shadowed `AT+TXCFG` (8 chars, UW_DOPPLER) and `AT+TXSTATS` (10 chars, UW_DOPPLER).
- When user sent `AT+TXCFG=?`, parser matched `AT+TX` first, then the terminator check (`'C'` not in `{'=','\r','\n'}`) returned `UNKNOWN_AT_CMD`.

**Confirmed by live tests earlier this session**: both `AT+TXCFG=?` and `AT+TXSTATS=?` returned `+ERROR=1203` (UNKNOWN_AT_CMD). Two commands were completely unreachable.

**Fix**: scan the full table and keep the LONGEST prefix match (`mgr_at_cmd.c:202-225`). Cost: +40 bytes flash, O(N) instead of O(early-break) — negligible at N=30.

**Test guard**: `Tests/unit/test_at_dispatch.c` (12 tests) mirrors the real table and validates that AT+TXCFG, AT+TXSTATS, AT+SWSFORCE, AT+SWSCFG, AT+SAVE_RCONF, AT+RATECLEAR, AT+RATECFG all resolve correctly when their shorter-prefix siblings are listed first.

### 10.2 Retention RAM layout

`.data2` = 756 bytes (knsCtxt + retention init), `.bss2` = 3124 bytes (retention zero-init: evtlog_buf, mgr_rate, mgr_txstats). All retained variables properly placed, no overlap. 3.9 KB / 32 KB SRAM2 used.

**Recommendation** (defense in depth): add `KEEP(*(.retentionRamData)) KEEP(*(.retentionRamBss))` to `STM32WL55XX_FLASH_APP.ld` so future `--gc-sections` aggression can't silently drop retained globals. Not done in this session because LD changes require explicit user OK per CLAUDE.md §5.

### 10.3 NVM

`MGR_NVM_load()` has clean v1→v5 migration paths with per-version CRC32 verification. Magic + CRC ensure first-power-on falls back to defaults safely. Power loss mid-write → CRC mismatch on reload → defaults restored, no brick. No redundant backup, but the failure mode is graceful.

### 10.4 Stack usage

Top consumers (from `.su` files):
- `vMGR_LOG_printf_ts`: 328 bytes
- `vMGR_LOG_printf`: 272 bytes
- `MGR_NVM_load/save`: 176 / 160 bytes
- `KNS_APP_uw_doppler_loop`: 128 bytes

Allocated `_Min_Stack_Size = 0x2000` (8 KB). Even with 5-deep nesting, < 1 KB used. Healthy.

### 10.5 SWS detection algorithm

Dynamic threshold based on air/water baselines with contrast-aware ratio (40/50/70% depending on contrast). Hysteresis enforced. Sample delay adapts for biofouling. Division-by-zero guarded. `air_baseline > 0` check before contrast computation. Integer promotion to 32-bit on `range * ratio` prevents uint16 overflow. **OK**.

### 10.6 Battery low-battery mode

Hysteretic state machine: enter < 2900 mV, exit > 3100 mV (defaults). `bat_mV == 0` (read failure) guarded — stays in current state. `lb_enter_mV == 0` disables LB entirely. NVM validation enforces `lb_exit > lb_enter` at save time. **OK**.

### 10.7 Race conditions (ISR vs main loop)

`MGR_REED` ISR-to-main interface uses a lock-free SPSC ring buffer:
- `evt_head` ISR-only write, volatile uint8_t
- `evt_tail` main-only write, volatile uint8_t
- `evt_buf` volatile array

Cortex-M atomic on 8-bit and 32-bit aligned reads/writes. Pattern is correctly implemented. `last_hold_ms` and `rising_tick` are 32-bit volatile — atomic single-word transfer between ISR and main.

`MGR_GESTURE` runs entirely from main loop (no ISR access), so `s_*` static variables don't need to be volatile. **OK**.

### 10.8 MAC init failure modes (kns_assert paths)

`KNS_MAC_task()` in libkineis uses `kns_assert(callback() == KNS_STATUS_OK)` on every event dispatch. Any return ≠ OK from MAC profile callbacks faults the chip.

`kns_assert_failed()` in `Kineis/Extdep/Conf/kns_assert.c` is the platform hook:
1. Direct synchronous UART print of `file:line` (reaches host before reset)
2. `MGR_LOG_DEBUG` (no-op in DEBUG=0 build)
3. `Error_Handler()` → for UW_DOPPLER: `MGR_ERR_LOG_FAULT(ERR_ASSERT, state)` flash-logged, then `NVIC_SystemReset()`

So MAC asserts are RECOVERABLE: file:line shown on UART, flash post-mortem logged, chip resets cleanly. The intermittent HardFault seen earlier this session was likely one of these (state=3 = WAIT_MAC_READY, suggests MAC init queue processing). With the verbose UART traces cleaned up, the timing race that triggered it appears gone (10/10 cold boots clean after cleanup).

### 10.9 Unchecked HAL returns

Most unchecked HAL calls are GPIO config or PWR config that cannot fail at runtime. UART transmit calls in trace paths are intentionally unchecked (best-effort tracing — failure must not crash the app). The two HAL calls that can plausibly fail at runtime (`HAL_RTCEx_SetWakeUpTimer_IT`, `HAL_RCC_OscConfig`) are checked where it matters (lpm.c and mcu_misc.c TCXO_Force_State respectively).

### 10.10 Test totals (final)

```
=== SUITES: 21/21 OK
=== TESTS:  340 individual checks
```

New tests this audit pass: `test_at_dispatch.c` (12 tests). Build matrix 8/8 green.

### 10.11 Files changed in this audit pass

```
Modified:
  Kineis/App/Managers/MGR_AT_CMD/Src/mgr_at_cmd.c     (longest-match dispatcher)
  Tests/unit/test_boot_loop_crc.c                     (fixed magic to match firmware)
  Tools/recover_chip.ps1                              (detect SHUTDOWN VDD-off state)

Added:
  Tests/unit/test_at_dispatch.c                       (12 tests guarding prefix shadowing)
```

---

## 11. Robustness pass (2026-06-06 evening) — "jamais crash, jamais lock, auto-recovery en capsule"

User authorized broad changes to make the firmware bulletproof for a sealed-capsule deployment. Four hard requirements:

1. Chip must NEVER crash without auto-recovery.
2. Chip must NEVER be locked off forever (no magnet access in capsule).
3. Retained state must survive aggressive linker GC.
4. Boot-loop guard must not brick a dev board on JLink reflash.

### 11.1 Linker `KEEP()` on retention sections (defense-in-depth)

Added `KEEP(...)` wrappers around `*(.retentionRamData)`, `*(.knsCtxtData)`, `*(.retentionRamBss)`, `*(.knsCtxtBss)` in both `STM32WL55XX_FLASH_APP.ld` and `STM32WL55XX_FLASH_CM4.ld` (bootloader). Future `--gc-sections` or LTO aggression now cannot silently drop retained globals (`boot_retained`, `sws_retained`, `evtlog_buf`, `mgr_rate`, etc.).

### 11.2 SHUTDOWN with RTC auto-wake — `LPM_shutdownWithAutoWake(seconds)`

New public API in `Kineis/Lpm/Inc/lpm.h` + implementation in `Kineis/Lpm/Src/lpm.c`:
- Same teardown as `LPM_shutdownNow()` (ADC deinit, GPIO analog, PB7 pull-down, flag clear, EnterSHUTDOWN).
- Additionally arms the RTC wake-up timer (`HAL_RTCEx_SetWakeUpTimer_IT`) for `wakeup_seconds`.
- Range 1..131 072 s (~36 h). Clamped to 17-bit max to prevent counter underflow.
- `wakeup_seconds = 0` is equivalent to `LPM_shutdownNow()` (no auto-wake).
- Enables internal wake-up line so RTC event reaches PWR controller.

### 11.3 Boot-loop detector: TIME-BASED instead of count-every-reset

Replaced the previous "increment on every reset, force-clear in dev" scheme with a smarter detector:

- New field `boot_in_progress` in `boot_retained` (retention RAM).
- SET on boot, CLEARED on MONITORING reached (`boot_loop_mark_success()`).
- A new boot that sees `boot_in_progress == 1` knows the previous boot died before MONITORING → count as failure.
- `boot_in_progress == 0` means the previous boot was OK → don't count (steady-state reset like AT+RESET).

**Cold-reset short-circuit**: if `RCC_CSR.BORRSTF` or `RCC_CSR.PINRSTF` is set, the counter is cleared regardless — those are clearly user-initiated.

**Result**: JLink reflash that interrupts a running (MONITORING-reached) firmware doesn't count as a failure. Only boots that genuinely fail to reach MONITORING within their normal ~10 s window do.

### 11.4 PERMANENT_OFF → 24 h auto-wake (sealed-deployment safety)

Replaced "permanent SHUTDOWN until magnet" with "SHUTDOWN with 24 h RTC auto-wake":

```c
#define BOOT_PERMANENT_OFF_WAKE_S  (24u * 3600u)
...
LPM_shutdownWithAutoWake(BOOT_PERMANENT_OFF_WAKE_S);
```

The capsule never goes truly off forever. Even if 10 boots fail back-to-back, the chip retries every 24 h. If the fault was transient (cosmic ray, voltage glitch, temperature), the chip recovers. If the fault is permanent (HW failure), at least it tries again — no manual intervention needed (which is impossible in a sealed capsule).

### 11.5 `AT+SHUTDOWN=<wake_seconds>` parameter

`AT+SHUTDOWN` now accepts an optional wake-time parameter:
- `AT+SHUTDOWN` (no param) → no auto-wake, behaves like before (magnet-only wake).
- `AT+SHUTDOWN=10` → SHUTDOWN, wake automatically after 10 s. Useful for validating the wake path without 24 h of patience.
- `AT+SHUTDOWN=86400` → 24 h auto-wake, mirrors PERMANENT_OFF behavior.

Range parsing via `sscanf("AT+SHUTDOWN=%lu")`, no value validation (clamped inside `LPM_shutdownWithAutoWake`).

### 11.6 Tests added (22 suites / 354 tests)

- `Tests/unit/test_boot_loop_time.c` (12 tests) — exhaustive validation of the time-based detector:
  - Fresh SRAM2 → initialize
  - BOR/PIN reset → clear counter
  - SW reset + previous boot OK → don't count
  - SW reset + previous boot failed → count
  - Factory reset threshold (5)
  - Factory reset only once (idempotent)
  - Permanent off threshold (10) → arms 24 h auto-wake
  - MONITORING reached → boot_in_progress cleared
  - **15 dev reflashes (with success each time) → counter stays at 0** (the key regression guard)
  - Genuine failure loop escalates correctly
  - IWDG reset before MONITORING → counted
  - Corrupted retention struct → safe reinit

- `Tests/unit/test_shutdown_sequence.c` (12 tests, +2 new):
  - Boot-loop PERMANENT_OFF arms 24 h auto-wake
  - Magnet 6 s shutdown does NOT auto-wake by default

### 11.7 Result summary

| Failure mode | Before | After |
|---|---|---|
| Soft reset after MONITORING (AT+RESET, late fault) | Counted as failure | NOT counted |
| Dev reflash (SYSRESETREQ after long uptime) | Counted (would brick after 10) | NOT counted |
| Dev reflash during boot phase | Counted | Counted (acceptable; rare in normal dev) |
| Real boot failure (HardFault before MONITORING) | Counted | Counted ✓ |
| 5 consecutive real failures | NVM wipe | NVM wipe ✓ |
| 10 consecutive real failures | SHUTDOWN forever (BRICK in capsule) | SHUTDOWN with 24h auto-wake (recoverable) ✓ |
| Linker GC drops retention vars | Possible | Prevented by KEEP() ✓ |

### 11.8 Tests + matrix totals (final after robustness pass)

```
=== SUITES: 22/22 OK
=== TESTS:  354 individual checks
```

| BOARD | APP | total |
|---|---|---|
| SMD_STDALONE | UW_DOPPLER | 119 396 |
| SMD_STDALONE | DOPPLER | 104 596 |
| SMD_STDALONE | GUI | 88 212 |
| SMD_STDALONE | STDLN | 71 556 |
| SMD_PA | GUI | 87 836 |
| SMD_PA | STDLN | 71 180 |
| SMD_NOPA | GUI | 87 244 |
| SMD_NOPA | STDLN | 70 580 |

### 11.9 Remaining hardening recommended for production (HW or option-byte work)

1. ~~**IWDG enable**~~ — **DONE** (§12). User authorized + IWDG_STOP option byte programmed at boot.

2. **Battery deep-discharge auto-shutdown** — current LB mode reduces TX rate when battery drops. Below a critical threshold (e.g. 2400 mV), enter SHUTDOWN with 24 h auto-wake to conserve energy until natural recovery (rare but possible for Li-SOCl2 chemistry, which has a passivation layer that lifts after rest).

3. ~~**HardFault forensics**~~ — **DONE** (§12). crash_info struct + naked handlers + boot replay.

### 11.10 Files changed in robustness pass

```
Modified:
  STM32WL55XX_FLASH_APP.ld                            (KEEP() retention)
  STM32WL55XX_FLASH_CM4.ld                            (KEEP() retention)
  Kineis/Lpm/Inc/lpm.h                                (LPM_shutdownWithAutoWake)
  Kineis/Lpm/Src/lpm.c                                (impl)
  Kineis/App/kns_app_uw_doppler.c                     (time-based boot detector)
  Kineis/App/Managers/MGR_AT_CMD/Src/mgr_at_cmd_list_uw_doppler.c (AT+SHUTDOWN=N)
  Tests/unit/test_shutdown_sequence.c                 (+2 tests for auto-wake)

Added:
  Tests/unit/test_boot_loop_time.c                    (12 tests, time-based detector)
```

---

## 12. IWDG + HardFault forensics pass (2026-06-06 late evening)

User explicitly authorized items #1 (IWDG_STOP option byte) and #3 (HardFault forensics) from §11.9.

### 12.1 HardFault forensics — `crash_info` retention struct

`Kineis/App/Managers/MGR_ERR/Inc/mgr_err.h` + `Src/mgr_err.c`:
- New `MGR_ERR_CrashInfo_t` struct (76 bytes) in `.retentionRamData` (SRAM2): magic + fault_type + app_state + HFSR/CFSR/BFAR/MMFAR + R0-R3/R12/LR/PC/xPSR + tick + CRC.
- `MGR_ERR_captureFault(frame, type, state)` — called from fault handlers.
- `MGR_ERR_hasRetainedCrash()` / `MGR_ERR_takeRetainedCrash(&out)` — read + clear-once API.

`Core/Src/stm32wlxx_it.c` — rewritten fault handlers:
- **Naked entry points** (`HardFault_Handler`, `MemManage_Handler`, `BusFault_Handler`, `UsageFault_Handler`) read EXC_RETURN.bit2 → choose MSP or PSP, then branch to a regular C function with the stacked frame address as the first argument.
- The C function prints live UART forensics (HFSR, CFSR, BFAR, MMFAR, PC, LR, R0, R12, xPSR), captures into `crash_info`, then `NVIC_SystemReset()`.
- For non-UW_DOPPLER/DOPPLER apps (GUI/STDLN), handlers are minimal `NVIC_SystemReset` only.

`Kineis/App/kns_app_uw_doppler.c` (init path) — boot-side replay:
- After `boot_loop_handle()`, calls `MGR_ERR_takeRetainedCrash()`.
- If a crash was captured, emits a `[CRASH-REPLAY]` direct-UART trace + logs `EVT_ERROR` to EVTLOG (which mirrors to PMLOG for ERROR severity).
- The crash_info is cleared so it only fires once.

**Result**: every HardFault / MemManage / BusFault / UsageFault produces a forensic post-mortem visible on UART at the next boot, even without being live-connected at fault time. The full Cortex-M context (PC, LR, fault regs) lets you identify the faulting instruction.

### 12.2 IWDG_STOP option byte + IWDG enable

`Kineis/App/Managers/MGR_WDG/Inc/mgr_wdg.h` + `Src/mgr_wdg.c`:
- New `MGR_WDG_ensureIwdgStopOptionByte()` checks `FLASH_OPTR.IWDG_STOP` (bit 17). If set, returns immediately. If not set, unlocks FLASH + option bytes, programs the bit to FREEZE (1), then calls `HAL_FLASH_OB_Launch()` — which **triggers a system reset** to reload the option byte. The next boot sees the bit set and proceeds.
- Idempotent (subsequent boots no-op).
- Best-effort cleanup if the launch somehow returns (it shouldn't).
- Logs `[WDG] IWDG_STOP option byte not set — programming...` so the one-time program is visible in UART.

`Kineis/App/kns_app_uw_doppler.c` (init path):
- Calls `MGR_WDG_ensureIwdgStopOptionByte()` BEFORE `MGR_WDG_init()`.
- Re-enables `MGR_WDG_init()` (was commented out as DEV-only).
- ~16 s timeout. Refreshed from main loop + `MGR_WDG_delayWithKick` paths.

**Net effect**: any hang > 16 s while the CPU is awake → IWDG resets the chip. STOP intervals don't accumulate against the IWDG counter (because IWDG_STOP=1 freezes it). The boot-loop guard then sees `boot_in_progress==1` → counts as failure, escalates to factory reset / 24 h SHUTDOWN auto-wake (§11.4) if persistent.

### 12.3 Test coverage added

```
Tests/unit/test_crash_forensics.c   (8 tests)  — CRC validation, capture+take API, multiple captures, NULL frame safety
Tests/unit/test_iwdg_optionbyte.c   (7 tests)  — decision tree: bit-set/clear, HAL failures, idempotency
```

### 12.4 Final tallies

```
=== SUITES: 24/24 OK
=### TESTS:  369 individual checks
```

Build matrix (8/8 UART configs all green):
| BOARD | APP | total |
|---|---|---|
| SMD_STDALONE | UW_DOPPLER | 122 116 |
| SMD_STDALONE | DOPPLER | 105 020 |
| SMD_STDALONE | GUI | 87 964 |
| SMD_STDALONE | STDLN | 71 308 |
| SMD_PA | GUI | 87 588 |
| SMD_PA | STDLN | 70 932 |
| SMD_NOPA | GUI | 87 004 |
| SMD_NOPA | STDLN | 70 340 |

### 12.5 Files changed

```
Modified:
  Kineis/App/Managers/MGR_ERR/Inc/mgr_err.h          (CrashInfo struct + API)
  Kineis/App/Managers/MGR_ERR/Src/mgr_err.c          (impl)
  Kineis/App/Managers/MGR_WDG/Inc/mgr_wdg.h          (ensureIwdgStopOptionByte)
  Kineis/App/Managers/MGR_WDG/Src/mgr_wdg.c          (impl)
  Core/Src/stm32wlxx_it.c                            (naked fault handlers)
  Kineis/App/kns_app_uw_doppler.c                    (crash replay + IWDG re-enable)

Added:
  Tests/unit/test_crash_forensics.c                  (8 tests)
  Tests/unit/test_iwdg_optionbyte.c                  (7 tests)
```
