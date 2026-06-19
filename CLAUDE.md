# CLAUDE.md — Agent rules for argos-smd-at-kineis-firmware

These rules govern how the Claude agent works on this firmware. They are the
result of a session where the agent over-patched symptoms instead of isolating
root causes — the goal here is to never repeat that.

---

## Golden rules

1. **No fix without root cause confirmed.** If the cause is a hypothesis, say
   so explicitly and propose ONE diagnostic before patching. Never stack
   debug patches.

2. **One change at a time. Build. Test. Then next.** Especially on a HW board
   that bricks easily, never apply multiple intertwined patches without an
   intermediate verification.

3. **Read the last commit message before touching a sensitive file.** Files
   like `lpm.c`, `mcu_misc.c`, `kns_app_uw_doppler.c`, `mgr_err.c`,
   `STM32WL55XX_FLASH_APP.ld`, anything in `Kineis/Extdep/Mcu/`, the
   bootloader — these have load-bearing history. Read `git log -1 --format=%B
   <file>` first.

4. **Always able to return to a stable state in one command.** Keep work in a
   feature branch. Stash temporary debug patches separately (`git stash push
   -m "debug-X"`). Document any safety patches that linger.

5. **Ask before risky actions.** Never touch without explicit user OK:
   - Option Bytes (`FLASH->OPTR`, BOR_LEV, SRAM_RST, NRST_MODE, IWDG_STOP)
   - `FLASH_USER` layout (shared with GUI and STANDALONE apps)
   - Anything that affects clock setup (`SystemClock_Config`, TCXO control)
   - Anything that touches reset/wake paths (HardFault handler, NMI handler,
     `MGR_LPM_enter*`, `boot_loop_handle`, `MGR_ERR_logAndReset`)
   - Bootloader code

6. **Pose questions when ambiguous.** It is better to ask 3 short questions
   than to assume wrong and burn a flash cycle. Default to asking when
   protocol details are not 100% specified.

---

## Investigation discipline

### When a bug is reported

1. Reproduce mentally from the user's log first. Quote the symptom verbatim.
2. List 2–3 candidate causes BEFORE reading any code.
3. Confirm one cause via the smallest possible diagnostic (a print, a
   one-line check). Do not ship a patch yet.
4. Only after the cause is confirmed: propose the fix, get user OK, apply
   the change, build, ask user to flash and verify.

### When adding diagnostic prints

- Mark them `/* DEBUG */` or `[DBG]` so they're greppable.
- Limit to direct-UART (`HAL_UART_Transmit` to `hlpuart1`) so they survive
  log-buffer issues.
- Remove them in the same PR as the root-cause fix; never let debug markers
  ship.

### When proposing a fix

Always state:
- Root cause (file:line)
- Why this fix solves it (mechanism, not "should work")
- What could regress (e.g. "this affects MAC init timing — verify TX still
  fires within X seconds after surface")
- Test plan (unit test + manual flash check)

### When you don't know

Say "I don't know yet" — propose a specific investigation step. Never bluff
a confident answer that is half-guessed (verified mistake during the SWS
audit where the agent gave wrong pin mapping for PB13).

---

## Code editing rules

### Comments
- **Short and clear.** One line max for "why", no novels.
- Comment the WHY (non-obvious constraint, hidden invariant, workaround for
  a specific bug), not the WHAT.
- Never reference the current task/PR (rots over time): no "added for issue
  #42", no "used by X".

### Tests
- For every fix or new logic: add or update a unit test in `Tests/unit/`.
- Mock HW (HAL_*, MGR_*) when needed — current framework allows it.
- Build + run via `cd Tests && make clean && make && make run` (uses WSL gcc
  on Windows).
- HTML report: investigate using `lcov + genhtml` or Unity built-in for
  generating `Tests/report/index.html`.
- Real-device tests live in `Tests/integration/` as Python scripts —
  optional, run when HW is available.

### Style
- Stay in **C** for the firmware (libkineis is C, board sharing with
  GUI/STANDALONE prevents C++ migration for now).
- Modern C99/C11 patterns: `const`-correctness, `static` for file-local,
  `_Static_assert` for compile-time invariants, no global mutables.
- MISRA-like discipline: no recursion, no dynamic allocation in init path,
  no implicit casts, no fall-through without explicit `__attribute__
  ((fallthrough))`.
- Run Valgrind / sanitizer on the unit test binaries to catch leaks and UB.

### Flash layout
- **DO NOT TOUCH `FLASH_USER` layout** — shared with GUI and STANDALONE apps.
  The offsets in `Kineis/Extdep/Mcu/Inc/mcu_flash.h` are load-bearing.
- Carve new regions OUTSIDE `FLASH_USER` (current example: `FLASH_PMLOG @
  0x08032000` and Bootloader @ `0x08033000`).
- Any change to `STM32WL55XX_FLASH_APP.ld` requires explicit user OK.

### Battery / power
- Default to LOWEST consumption. CPU idle → STOP mode (when MAC permits) +
  SWS polling + reed wake. No spurious 1Hz heartbeats in production.
- TX power consumption is out of scope for now (we don't tune MAC TX path).
- Optimize the idle paths (STOP, SHUTDOWN) and SWS sampling cadence.

---

## Documentation discipline

- **No invented documentation files** unless user asks. Update the existing
  `.claude/*.md` and `docs/reports/*.md`.
- `docs/reports/MASTER_AUDIT.md` is the source of truth for the current
  audit state. Keep it updated when findings change.
- Cite file:line for every claim. Cite page numbers when referencing PDFs
  (after `pdftotext` conversion places them in `.claude/stm/*.txt` and
  `.claude/kineis/*.txt`).

---

## Tooling

- **PDF reading**: PDFs in `.claude/stm/` and `.claude/kineis/` have been
  converted to `.txt` via `pdftotext -layout` (from MSYS2). Read the `.txt`
  versions; recreate them with the same command if the PDFs are updated.
- **Build**: `make help` lists the canonical bench/release command lines.
  ALWAYS `make clean` when changing any flag (DEBUG/BOARD/APP/REED_*) —
  make does not track flag changes and silently reuses stale objects.
- **Tests**: `cd Tests && ./scripts/run_tests.sh` (WSL gcc; NOT make).
- **Flash**: `make flash` (full image), `make flash-app`, `make
  flash-recover` (NRST-pulse attach for a deaf/STOP2-stuck board),
  `make reset`, `make erase` (FLASH_USER preserved), `make erase-all`
  (wipes credentials — dangerous).

---

## Communication style

- **French** in the conversation (user preference).
- **Short answers**, no padding, no over-explanation of obvious things.
- **No emojis** in code or in conversation, unless user explicitly asks.
- When stating progress: status, blockers, next step. Don't restate the
  problem the user just stated.
- When proposing a multi-step plan: number the steps, mark which require
  user input vs. which the agent runs autonomously.

---

## Anti-patterns observed in past sessions (avoid)

These actually happened. Listing them so they don't repeat:

- Stacked debug patches that masked each other (boot_loop force-clear, IWDG
  disable, LPM=NONE, state-hang disable, etc.) — confused the symptoms.
- Calling tools/agents in series instead of root-cause analysis.
- Asserting confident facts (`PB13 = ADC_IN5`) without checking the
  datasheet.
- Doing flash modifications without re-reading the linker file constraints.
- Modifying TCXO behaviour without measuring impact on MAC init.
- Adding option-byte writes that bricked the chip.

---

## Reference order when investigating

1. `docs/reports/MASTER_AUDIT.md` — current consolidated state
2. `docs/reports/<specific>.md` — sub-audits
3. `.claude/linkit-uw-behavior.md` + `.claude/sws_analog_implementation.md`
   — linkit-v4 behaviour reference
4. `.claude/kineis/*.txt` — Kineis stack + Doppler reference
5. `.claude/stm/*.txt` — RM0453, DS13105, ANs (ST reference)
6. `Kineis/Doc/krd_fw/html/` and `Kineis/Doc/libkineis/html/` — local
   Doxygen for closed-source Kineis libs
