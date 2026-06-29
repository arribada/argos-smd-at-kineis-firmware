# MGR_* Application Managers

## Purpose

App-layer managers sitting between the closed-source Kineis MAC stack
(`Kineis/`) and the UW_DOPPLER / DOPPLER applications (`Kineis/App/kns_app_*.c`).
Each `MGR_*` folder is a self-contained subsystem (host command parsing, power
management, persistence, peripherals, forensics). They are not threads: every
manager exposes an `init`/`start` + a `task`/`state_handler`/`*EvtProcess` entry
that the app's main loop drives cooperatively. Most are conditionally compiled
per `APP` / `COMM` / `BOARD` (see [Integration](#integration)).

## Managers

| Folder | Role | Gated by |
|---|---|---|
| [MGR_AT_CMD](MGR_AT_CMD) | AT command parser/dispatcher over the UART console; the largest manager | `COMM=UART` (always); UW/DOPPLER cmd lists per `APP` |
| [MGR_SPI_CMD](MGR_SPI_CMD) | SPI "A+" pipelined single-transaction protocol + command handlers (host-MCU control link) | `COMM=SPI` |
| [MGR_LPM_UW](MGR_LPM_UW) | UW_DOPPLER duty-cycle low-power strategy (STOP2 / STANDBY / SHUTDOWN-reed, event-driven idle scheduler) | `APP=UW_DOPPLER` |
| [MGR_SWS](MGR_SWS) | Salt-Water-Switch: 5-level adaptive underwater/surface detection via ADC | `APP=UW_DOPPLER` |
| [MGR_NVM](MGR_NVM) | Persistent config (`NVM_Config_t`) in the last FLASH_USER page, CRC32-checked, versioned v1..v8 | `APP=UW_DOPPLER` |
| [MGR_BAT](MGR_BAT) | Battery voltage via internal `ADC_CHANNEL_VBAT`; TX-allowed gate | `APP=UW_DOPPLER`; `APP=DOPPLER` only on `SMD_STDALONE` |
| [MGR_ERR](MGR_ERR) | Reset/crash forensics: TAMP backup regs + retention-RAM `CrashInfo`; crash-loop guard | `APP=UW_DOPPLER`/`DOPPLER` |
| [MGR_WDG](MGR_WDG) | IWDG ~16 s watchdog (direct registers) + IWDG_STOP option-byte ensure + kicking delay | `APP=UW_DOPPLER`/`DOPPLER` |
| [MGR_EVTLOG](MGR_EVTLOG) | 256-entry circular event log in SRAM2 retention RAM (survives SW/IWDG reset) | `APP=UW_DOPPLER`/`DOPPLER` |
| [MGR_RATE](MGR_RATE) | Sliding-window TX rate limiter (anti-drain last-resort defence), retention ring | `APP=UW_DOPPLER` |
| [MGR_TXSTATS](MGR_TXSTATS) | Persistent TX health counters (attempts/done/timeout/error), retention RAM | `APP=UW_DOPPLER` |
| [MGR_PMLOG](MGR_PMLOG) | Post-mortem flash log: ERROR-severity events mirrored to a dedicated flash page (survives power-off) | `APP=UW_DOPPLER` |
| [MGR_CRED](MGR_CRED) | Credential durability: CRC mirror of the 48-byte page-0 block + boot-time auto-restore | `APP=UW_DOPPLER` |
| [MGR_GESTURE](MGR_GESTURE) | Magnet gesture FSM (2-gesture confirm) → OPERATIONAL / CONFIG / SHUTDOWN; LED feedback | `APP=UW_DOPPLER` (uses LED/REED) |
| [MGR_LED](MGR_LED) | RGB LED driver (active-low, PA1/PB4/PB5), blink/soft-PWM, deployment-mode gating | `BOARD=SMD_STDALONE` (`BSP_HAS_LED_RGB`) |
| [MGR_REED](MGR_REED) | Reed switch (PB6 EXTI, debounced) + power latch (PB7) driver | `BOARD=SMD_STDALONE` (`BSP_HAS_REED`) |

### MGR_AT_CMD (the AT parser)

The console front-end. [mgr_at_cmd.c](MGR_AT_CMD/Src/mgr_at_cmd.c) buffers
incoming UART bytes into an internal FIFO (`MGR_AT_CMD_isPendingAt` /
`MGR_AT_CMD_popNextAt`), then `MGR_AT_CMD_decodeAt` matches the command name
against `cas_atcmd_list_array[]` (a longest-prefix scan, see `enum atcmd_idx_t`
in [mgr_at_cmd_list.h](MGR_AT_CMD/Inc/mgr_at_cmd_list.h)) and dispatches to the
handler's `f_ht_cmd_fun_proc(params, ATCMD_ACTION_MODE | ATCMD_STATUS_MODE)`.
Handlers are split by domain across `mgr_at_cmd_list_general/_mac/_v11/_certif/
_previpass/_user_data/_uw_doppler/_doppler.c`. `MGR_AT_CMD_macEvtProcess` drains
async MAC results (TX-done/timeout, RX/DL) and emits the matching `+...` line.
`MGR_AT_CMD_getLastActivityTick` is consumed by the LPM scheduler as an
"operator on console" hold-off signal. `mgr_at_cmd_common.c` holds the shared
response/logging helpers and `MGR_AT_CMD_mapKnsStatusToError` (folds the
v11 `KNS_STATUS_ABORT` back to the legacy `+ERROR=1` for GUI compatibility).

### MGR_SPI_CMD (the SPI protocol)

The alternative host link, selected with `COMM=SPI`. It is a **pipelined
single-transaction protocol**: each SPI exchange carries the response to the
*previous* command, sidestepping slave-response timing problems
([mgr_spi_cmd.h](MGR_SPI_CMD/Inc/mgr_spi_cmd.h)).
[mgr_spi_protocol.c](MGR_SPI_CMD/Src/mgr_spi_protocol.c) implements the "A+"
framing — magic bytes (`0xAA` request / `0x55` response), 4-byte header
(magic/seq/cmd/len), CRC-8 (CCITT 0x07), and a parser that also auto-detects a
legacy single-byte-command mode. `mgr_spi_cmd.c` is the state machine
(`MGR_SPI_CMD_state_handler`, the `cmdInProgress` latch, `MGR_SPI_CMD_start`,
`MGR_SPI_CMD_macEvtProcess`); the `CmdValue` opcode table and `spicmd_desc_t`
descriptors live in [mgr_spi_cmd_list.h](MGR_SPI_CMD/Inc/mgr_spi_cmd_list.h),
with handlers split into `mgr_spi_cmd_list_general/_mac/_certif/_previpass/
_user_data.c`. The opcode set deliberately mirrors the AT set (READ/WRITE for
ID/ADDR/SECKEY/RCONF/KMAC/LPM/MC/TCXO, TX, DFU enter), so AT and SPI builds
expose the same capabilities. `macStatus` (`MACStatus` enum) is the async MAC
result reported back via `CMD_MAC_STATUS`.

## Key flows / data structures

- **Host command dispatch** — both AT and SPI use a `{name/opcode → handler fn
  ptr}` descriptor array (`atcmd_desc_t` / `spicmd_desc_t`). Write commands that
  need a payload use a two-step REQ→VALUE handshake on SPI
  (`CMD_WRITE_*_REQ` then `CMD_WRITE_*`).
- **Persistence tiers** (most-to-least durable):
  1. Flash, survives power-off: `MGR_NVM` (FLASH_USER page), `MGR_PMLOG`
     (`0x08032000`), `MGR_CRED` mirror (`0x08032800`).
  2. SRAM2 retention `.retentionRamNoload` (survives every SW reset incl.
     STANDBY cold-boot; lost only on VBAT removal): `MGR_LPM_UW` `duty_cfg`,
     SWS calibration, `MGR_ERR` `CrashInfo`.
  3. SRAM2 retention `.retentionRamBss` (zeroed by `Sram2_Init` on
     BOR/NRST/OBL): `MGR_EVTLOG`, `MGR_RATE`, `MGR_TXSTATS`.
  4. TAMP backup registers (BKP2R..BKP7R): `MGR_ERR` reset/crash counters,
     `MGR_GESTURE` persisted mode.
  Each retention region is validated by a magic word + CRC32 and re-inits clean
  on mismatch.
- **LPM scheduling** — `MGR_LPM_UW_idleTick(state, delta_ms, ...)` is the
  preferred entry: spin / SLEEP / STOP2 chosen from `LpmThr` thresholds.
  `MGR_LPM_UW_enterStop2TimedMs` does RTC-compensated wall-clock sleep; STANDBY
  and SHUTDOWN-reed paths are `noreturn` cold-boot exits.
- **Gesture FSM** — `MGR_GESTURE` consumes raw `MGR_REED_Event_t` +
  hold-duration and produces `MGR_GESTURE_Event_t` (ENTER_OPERATIONAL /
  ENTER_CONFIG / REQUEST_SHUTDOWN) plus LED feedback; mode persisted in TAMP.
- **SWS detection** — `MGR_SWS_task` samples PA11 (ADC_IN7) with PA12 power
  control, runs the 5-level (L1..L5) drop detector with adaptive baselines and
  emits `MGR_SWS_State_t` SURFACE/UNDERWATER; calibration is debounce-saved to
  NVM via `MGR_NVM_saveCalibDebounced`.
- **Crash forensics** — fault handlers call `MGR_ERR_captureFault` (naked, MSP
  frame copy) into `.retentionRamNoload`; next boot `MGR_ERR_takeRetainedCrash`
  emits the trace and mirrors ERROR events to PMLOG.

## Integration

- **Build flags**
  - `COMM=UART` → builds `MGR_AT_CMD` (+`USE_UART_DRIVER`).
    `COMM=SPI` → builds `MGR_SPI_CMD` (+`USE_SPI_DRIVER`). They are mutually
    exclusive host links; the two managers expose the same command surface.
  - `APP=UW_DOPPLER` (`USE_UW_DOPPLER_APP`) pulls in SWS/NVM/BAT/ERR/WDG/EVTLOG/
    RATE/TXSTATS/PMLOG/CRED/GESTURE/LPM_UW and the `_uw_doppler` AT handlers.
    `APP=DOPPLER` pulls only ERR/WDG/EVTLOG (+ BAT on `SMD_STDALONE`) and the
    `_doppler` AT handlers. `APP=GUI`/`STDLN` use neither UW manager set.
  - `BOARD=SMD_STDALONE` compiles `MGR_LED` + `MGR_REED` (`BSP_HAS_LED_RGB` /
    `BSP_HAS_REED`); other boards omit them, and callers guard at the call site
    so UW_DOPPLER still links on `SMD_PA`/`SMD_NOPA`/`SMD_OP`.
  - `REED_WKUP3_WIRE` / `REED_WKUP1_WIRE` / `REED_WKUP3` select the SHUTDOWN
    wake path used by `MGR_LPM_UW` / `MGR_REED` (PB6 EXTI is not a WKUP pin).
  - `DEBUG=0` tears down the UART console by design — `MGR_AT_CMD` logging goes
    silent (see project notes on the build-flag trap; always `make clean` on a
    flag change).
- **Upward**: the app loop (`kns_app_uw_doppler.c`) calls each manager's
  `task`/`state_handler`/`EvtProcess`, reads `g_uw_doppler_state_for_err`
  (stamped into EVTLOG/ERR entries) and reacts to gesture/SWS/LPM decisions.
- **Downward**: managers call the MCU wrappers in `Kineis/App/Mcu/` (flash,
  ADC, SPI/UART drivers, AES/NVM) and the closed-source MAC (`kns_mac.h`,
  `KNS_status_t`). `MGR_NVM` depends on `mgr_sws.h`/`mgr_bat.h` struct layouts.
- **Sibling cross-refs**: `MGR_PMLOG` is driven from inside `MGR_EVTLOG_log`;
  `MGR_CRED` guards the same page-0 layout that `mcu_flash.h`/`mcu_nvm.c` read;
  `MGR_GESTURE` sits on top of `MGR_REED` + `MGR_LED`.

## Gotchas / constraints

- **NVM size cap**: `NVM_Config_t` must stay inside one 2 KB flash page and is
  64-bit-write aligned; bump `NVM_VERSION` (now 8) and keep the trailing
  `crc32` field last when adding fields.
- **Credentials are co-located with RMW counters** in page 0; a brownout during
  a counter overflow can blank them, which is exactly why `MGR_CRED` keeps a
  separately-erased mirror. Do **not** move the credential offsets (load-bearing
  contract with the GUI/STANDALONE provisioning tool).
- **PMLOG / CRED flash addresses** (`0x08032000`, `0x08032800`) are hard-coded
  to match `STM32WL55XX_FLASH_APP.ld`; the app ROM was shrunk to 200 K to carve
  these out. Keep them in sync with the linker.
- **PMLOG wear**: only ERROR-severity events are mirrored to flash (one page
  erase per 128 entries); lowering the severity gate would burn the page fast.
- **MGR_BAT pin trap**: PB13 (`VBAT_ADC` in the BSP) is *not* an ADC pin on
  STM32WL55 — the internal `ADC_CHANNEL_VBAT` is used instead.
- **MGR_LED composite colours**: on `SMD_STDALONE` the single anode resistor
  makes WHITE/VIOLET/CYAN/YELLOW degrade to RED unless `MGR_LED_softTick()` is
  wired into `SysTick_Handler` every 1 ms. The same pins are the bootloader SPI
  bus, so there is no LED in the bootloader.
- **MGR_REED debounce vs STOP2**: the LPM client must check
  `MGR_REED_isDebouncing()` before sleeping or a magnet edge can be lost
  mid-debounce; `MGR_REED_blankUntil` suppresses PA-TX-transient coupling into
  the high-impedance reed node.
- **MGR_WDG is one-way**: once `MGR_WDG_init()` starts the IWDG it cannot be
  stopped; `MGR_WDG_ensureIwdgStopOptionByte()` may trigger a reset to program
  the option byte and must run *before* `MGR_WDG_init` and any LPM client.
- **MGR_ERR fault path** uses `MGR_ERR_LOG_FAULT`/`captureFault` with direct
  register / `uwTick` access only — safe from a corrupted stack; do not add
  function calls there.
- **MGR_LPM_UW STANDBY entry** is kept for `AT+STANDBYTEST` only; the production
  duty path is STOP2 (no 6 s MAC re-init per wake, and reed EXTI on PB6 can wake
  STOP2 but not STANDBY).
- **Message counter is 9-bit** (0..511): SPI/AT MC writes fold mod 512;
  exceeding it desyncs the AES IV.
