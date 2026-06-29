# Tests — Host-gcc unit harness + HW integration scripts

## Purpose
Off-target test suite for the STM32WL55 Argos/Kineis firmware. The `unit/`
suites compile and run on the host (WSL/MSYS2 gcc) with **no STM32 toolchain
and no hardware** — each suite is a standalone `main()`. The `integration/`
scripts drive the real bootloader DFU path over UART/SPI when a board is
attached. There is no link against the firmware objects; instead each unit
suite **mirrors the logic under test** with HAL/MGR dependencies stubbed.

## Files
| Path | Role |
|------|------|
| [scripts/run_tests.sh](scripts/run_tests.sh) | **Canonical runner.** Compiles every `unit/test_*.c` standalone with gcc, runs each, sums suites + checks. Bypasses the Makefile (see Gotchas). Exit 0 = all pass. |
| [scripts/gen_html_report.sh](scripts/gen_html_report.sh) | Runs given test binaries, parses framework output, emits self-contained `report/index.html`. |
| [unit/test_framework.h](unit/test_framework.h) | The whole framework: counters, `ASSERT_*`, `TEST_START/PASS/FAIL`, `RUN_TEST`, `TEST_SUITE_START/END`, `TEST_SUMMARY`. Header-only, no .c. |
| [unit/Makefile](unit/Makefile) | Alternate runner with `-Werror` + `-fsanitize=address,undefined`. Only lists a 9-suite subset; **stale vs the 42 actual suites** — use the .sh runner instead. |
| `unit/test_*.c` | 42 standalone suites (see overview below). One `main()` per file. |
| [run_all_tests.py](run_all_tests.py) | Python orchestrator: builds/runs the 3 original DFU unit suites, optionally dispatches UART/SPI integration tests (`--uart COMx`, `--spi-mock`, `--all`). |
| [run_tests.bat](run_tests.bat) | Windows-native runner (SPI mock + 3 DFU C suites). Legacy, narrow scope. |
| [integration/test_uart_dfu.py](integration/test_uart_dfu.py) | UART DFU integration test against a live board (`--port COM3`). |
| [integration/test_spi_dfu.py](integration/test_spi_dfu.py) | SPI DFU integration test; `--mock` runs without hardware, `--interface ftdi/buspirate` with HW. |
| [integration/dfu_validation_suite.py](integration/dfu_validation_suite.py) | End-to-end DFU cycle: ping → erase → write → CRC verify → jump. |
| `build/`, `build_asan/` | gcc output (executables). Generated, not source. |
| `report/index.html` | Generated HTML report. |

### Unit suite overview (42 suites)
- **Bootloader / DFU**: `test_crc32`, `test_dfu_protocol`, `test_app_header`,
  `test_spi_protocol`, `test_spi_wedge_recover` (SPICMD_IDLE wedge watchdog),
  `test_memory_safety`.
- **NVM / credentials / message counter**: `test_nvm`, `test_nvm_heal_migration`
  (v7→v8 migration), `test_cred_mirror` (MGR_CRED durability), `test_mc_9bit`,
  `test_mc_wrap`, `test_mc_wear`, `test_seq_mc`, `test_pmlog_torn`,
  `test_sn_uid` (UID-derived serial).
- **LPM / power / shutdown**: `test_lpm_uw` (UW_DOPPLER duty-cycle),
  `test_lpm_deadline`, `test_lpm_gating`, `test_lpm_spi_grace` (STOP-over-SPI),
  `test_shutdown_sequence`, `test_bat_auto_shutdown`, `test_wrap_overflow`
  (HAL tick 49.7-day wrap).
- **SWS (surface/water sensor)**: `test_sws` (5-level algorithm),
  `test_sws_degraded`, `test_sws_l4_guards` (linkit-v4 L4/L5 hardening).
- **UW_DOPPLER app**: `test_state_machine`, `test_tx_interval`,
  `test_tx_cooldown`, `test_cooldown_modes`, `test_cooldown_persist`,
  `test_payload_mini`.
- **Gestures / reed / LED / self-test**: `test_gesture_fsm`,
  `test_reed_hold_actions`, `test_led_swap` (SMD_STDALONE PA1/PB4/PB5),
  `test_selftest_mask`.
- **Boot-loop / crash forensics / AT / hardening**: `test_boot_loop_crc`,
  `test_boot_loop_time`, `test_crash_forensics` (MGR_ERR replay),
  `test_iwdg_optionbyte`, `test_at_dispatch` (longest-prefix match),
  `test_evtlog`, `test_audit_fixes` (memory-safety regression locks).

## Key flows / data structures

### Framework (`test_framework.h`)
Three file-static counters (`tests_run/passed/failed`) and a current-test name.
A suite body looks like:
```c
TEST_SUITE_START("name");
RUN_TEST(test_foo);          /* increments tests_run, prints "[TEST] foo... " */
TEST_SUITE_END();            /* prints "Results: P/N passed" */
TEST_SUMMARY();              /* returns 0 if tests_failed == 0, else 1 */
```
`ASSERT_*` macros (`ASSERT_TRUE/FALSE/EQ/NE/EQ_HEX/STR_EQ/MEM_EQ/NOT_NULL`)
**`return` from the test function on failure** — so each `test_*` is `void`
and may hold several asserts; the trailing `TEST_PASS()` is reached only if
all asserts passed. "Checks" in the summary = number of `RUN_TEST` invocations,
not number of asserts.

### Mirror-the-logic convention (the load-bearing pattern)
The real modules pull in STM32 HAL, retention RAM sections, and the closed
libkineis MAC, none of which link on the host. So a suite does **not** include
the production `.c`. Instead it **copies the algorithm** into the test file as
file-static functions, with the hardware dependencies stubbed, and asserts on
that copy. Canonical examples:
- [unit/test_mc_9bit.c](unit/test_mc_9bit.c): `nvm_setMC`/`nvm_getMC` reimplement
  the `& 0x1FF` clamp from `mcu_nvm.c`; `at_mc_fold` mirrors the AT+MC `% 512`
  fold from `mgr_at_cmd_list_general.c`.
- [unit/test_lpm_uw.c](unit/test_lpm_uw.c): mirrors `MGR_SWS_State_t` enum and
  the `UwLpmDutyCfg_t` struct, then re-implements `detect_surface_wake` etc.
- [unit/test_sws.c](unit/test_sws.c): redefines all `mgr_sws.c` `#define`
  thresholds (with `mgr_sws.c:line` cross-references) and the detection math.

**Invariant:** the mirrored constants/logic MUST track the shipping source. A
drift means the test validates a *different* algorithm than what deploys — this
was an actual audit finding (see the `mgr_sws.c` comment in `test_sws.c`).
When you change a guarded constant in the firmware, update its mirror here.

## Integration
- **No firmware build flags affect the host unit suites** — they are standalone
  and intentionally decoupled from `APP`/`BOARD`/`COMM`/`DEBUG`/`LPM`. That
  decoupling is *why* the mirror convention exists: the suites encode the
  behaviour each flag combination is supposed to produce (e.g. `test_led_swap`
  bakes in the `BOARD=SMD_STDALONE` PA1/PB4/PB5 mapping; `test_lpm_*` encode the
  `APP=UW_DOPPLER` duty-cycle and `LPM` STOP policy) rather than reading the
  flags.
- **Integration scripts DO exercise real flags**: they require a board flashed
  with the bootloader and matching `COMM` transport (UART vs SPI DFU).
- Each unit suite maps to a firmware subsystem: NVM/MC ↔ `Kineis/Extdep/Mcu/`
  (`mcu_nvm.c`), SWS ↔ `mgr_sws.c`, LPM ↔ `mgr_lpm*.c`/`MGR_LPM_UW`, gestures
  ↔ `mgr_gesture.c`, crash forensics ↔ `mgr_err.c`, DFU ↔ the bootloader.
  See [../CLAUDE.md](../CLAUDE.md) "Tests" rules: every fix gets a suite here.

## Gotchas / constraints
- **Use `scripts/run_tests.sh`, NOT `make`.** The script header documents why:
  on MSYS2 the make subshell inherits a corrupted `TMP`, so gcc tries to write
  temp files to `C:\WINDOWS\` and fails. Direct gcc invocation works.
- **The runner globs `build/test_*.exe`** (Windows gcc output). On a pure-Linux
  host where gcc emits extension-less binaries the run-phase loop would match
  nothing — the harness assumes the WSL/MSYS2-on-Windows setup described in
  CLAUDE.md.
- **`unit/Makefile` is stale and partial**: it lists only 9 suites and adds
  `-Werror`+ASan/UBSan (useful for a sanitizer pass), but it is not the source
  of truth for what runs. The full 42-suite set comes from the `.sh` glob.
- **`run_all_tests.py` / `run_tests.bat` only know the 3 original DFU suites**
  (`test_crc32`, `test_dfu_protocol`, `test_app_header`). They are not kept in
  sync with new suites; for full unit coverage use `run_tests.sh`.
- **Check count is parser-derived, not authoritative.** The runner greps suite
  trailers in three formats (`Results: P/N`, `P/N tests`, `N tests`); a suite
  printing a format it can't parse contributes 0 to the total without failing.
- **Current state (2026-06-29): 42/42 suites OK, 535 individual checks** from a
  live `run_tests.sh` run. (CLAUDE.md memory cites 42 suites / ~532 checks — the
  drift is new/expanded suites; the live number is authoritative.)
- **Adding a suite**: create `unit/test_<name>.c`, `#include "test_framework.h"`,
  write `void test_*` functions using `ASSERT_*`, and a `main()` with
  `TEST_SUITE_START` / `RUN_TEST(...)` / `TEST_SUITE_END` / `TEST_SUMMARY`.
  No registration needed — `run_tests.sh` auto-discovers via the `test_*.c`
  glob. If you mirror production logic, cross-reference `file.c:line` in a
  comment so the next person can spot drift.
