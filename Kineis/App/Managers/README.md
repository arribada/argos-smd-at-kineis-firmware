# Kineis App Managers (`MGR_*`)

## Purpose

The `MGR_*` managers are the application-layer modules sitting between the host
interface / state machine and the closed-source libkineis MAC + the MCU
wrappers (`Kineis/App/Mcu`). Each manager owns one concern: a host command
protocol (AT or SPI), a sensor/peripheral (SWS, battery, reed, LED), a
persistence store (NVM, event log, post-mortem log, credentials), or a
cross-cutting policy (low-power, watchdog, error tracking, rate limiting).
They are pulled into the build conditionally by the `APP`, `BOARD` and `COMM`
flags; only `MGR_AT_CMD` is always present.

## Files

Each manager is a subfolder with `Inc/` headers and `Src/` sources.

| Manager (folder) | Role | Gating |
| --- | --- | --- |
| `MGR_AT_CMD` | AT command parser/dispatch over UART (FIFO + handler table). | Always (core); UW handlers gated `USE_UW_DOPPLER_APP`, Doppler gated `USE_DOPPLER_APP` |
| `MGR_SPI_CMD` | SPI A+ pipelined protocol + per-command handlers (host alternative to AT). | `COMM=SPI` (`USE_SPI_DRIVER`) |
| `MGR_LPM_UW` | UW_DOPPLER duty-cycle / low-power strategy (STOP2, STANDBY, SHUTDOWN+reed, event-driven idle scheduler). | `APP=UW_DOPPLER` |
| `MGR_SWS` | Salt-Water Switch: 5-level adaptive surface/underwater detection from an ADC RC-charge reading (PA12 power, PA11 = ADC_IN7). | `APP=UW_DOPPLER` |
| `MGR_NVM` | Persistent config in the last FLASH_USER page (`NVM_Config_t`, magic+CRC32, versioned v1..v8). | `APP=UW_DOPPLER` |
| `MGR_BAT` | Battery voltage via internal `ADC_CHANNEL_VBAT`; TX-inhibit threshold. | `APP=UW_DOPPLER`; also `APP=DOPPLER` + `BOARD=SMD_STDALONE` |
| `MGR_ERR` | Reset-cause + crash tracker in TAMP backup regs; HardFault forensics in retention RAM; crash-loop guard. | `APP=UW_DOPPLER` or `DOPPLER` |
| `MGR_WDG` | IWDG watchdog (~16 s, direct register), IWDG_STOP option-byte guard, watchdog-kicking delay. | `APP=UW_DOPPLER` or `DOPPLER` |
| `MGR_EVTLOG` | 256-entry circular event log in SRAM2 retention RAM; mirrors ERROR events to `MGR_PMLOG`. | `APP=UW_DOPPLER` or `DOPPLER` |
| `MGR_RATE` | Persistent sliding-window TX rate limiter (retention ring, magic+CRC32). | `APP=UW_DOPPLER` |
| `MGR_TXSTATS` | Persistent TX outcome counters (attempts/done/timeout/error) in retention RAM. | `APP=UW_DOPPLER` |
| `MGR_PMLOG` | Post-mortem flash log: ERROR-severity events in a dedicated page `@0x08032000`, survives full power-off. | `APP=UW_DOPPLER` |
| `MGR_CRED` | Credential durability: CRC-protected 48-byte mirror `@0x08032800` + boot-time restore of page-0 creds. | `APP=UW_DOPPLER` |
| `MGR_GESTURE` | Magnet-gesture FSM on top of `MGR_REED` (2-gesture confirm protocol: POWER_OFF/OPERATIONAL/CONFIG/shutdown). | `APP=UW_DOPPLER` |
| `MGR_LED` | RGB LED driver (PA1=R, PB4=G, PB5=B, active-LOW); soft-PWM composite colours; gated vs forced writes. | Headers always exposed; calls gated by `BSP_HAS_LED_RGB` at call sites |
| `MGR_REED` | Reed switch (PB6 EXTI, both edges, debounce, hold duration) + power latch (PB7). | Headers always exposed; calls gated by `BSP_HAS_REED` at call sites |

### The two big ones

**`MGR_AT_CMD`** — text command engine over LPUART. A UART RX path fills a
small FIFO of `\0`-terminated frames (`s_atcmdfifo`, `MGR_AT_CMD_isPendingAt` /
`MGR_AT_CMD_popNextAt`). The app loop calls `MGR_AT_CMD_decodeAt`, which
classifies the frame (`MGR_AT_CMD_getAtType`) into an index of
`enum atcmd_idx_t` plus an exec type (action `=...` / status `=?`), then
dispatches through the `cas_atcmd_list_array[]` table of
`struct atcmd_desc_t` (name, min length, handler fn-pointer). Handlers live in
the `mgr_at_cmd_list_*.c` files: `general` (FW/ID/ADDR/SECKEY/RCONF/LPM/MC/
TCXO), `user_data` (TX/RX), `certif` (CW), `mac` + `v11` (KMAC/KCFG/KEVT/DL/
GNSS), `previpass`, `uw_doppler` (the whole `AT_SWS*`/`AT_TX*`/`AT_LOG`/
`AT_LPM*`/`AT_DUTYCFG`/... block), and `doppler`. Longer prefixes are listed
first in the enum to avoid dispatcher shadowing. `MGR_AT_CMD_getLastActivityTick`
feeds the LPM schedulers an "operator on console" signal;
`MGR_AT_CMD_macEvtProcess` (weak) drains MAC TX/RX events as AT responses.

**`MGR_SPI_CMD`** — binary command engine over SPI, the host alternative to AT.
Built on a pipelined single-transaction protocol where each SPI transaction
carries the response to the *previous* command (no timing races). Layered as
`mgr_spi_protocol.c` (the **A+** wire framing: magic `0xAA` request / `0x55`
response, sequence number, CRC-8, with legacy single-byte-command fallback —
see `SpiProtocolContext`, `SpiRequestFrame`, `SpiResponseFrame`) under
`mgr_spi_cmd.c` (`MGR_SPI_CMD_state_handler` called from the main loop; tracks
`cmdInProgress`). Command handlers mirror the AT set and dispatch through
`cas_spicmd_list_array[]` of `struct spicmd_desc_t` (`CmdValue cmd`, `next_cmd`,
handler) split across `mgr_spi_cmd_list_*.c` (`general`/`user_data`/`mac`/
`certif`/`previpass`). Multi-byte writes use a request/size/value 3-step
(`CMD_WRITE_*_REQ` → `_SIZE`/`_VALUE`). DFU entry is `CMD_DFU_ENTER`
(`0x3F`), aligned with the bootloader status codes.

## Key flows / data structures

- **Host command dispatch** — both big managers share the same shape: a
  table of descriptors indexed by a command enum, each with a handler
  function pointer. AT is text+FIFO; SPI is framed+pipelined.
- **`NVM_Config_t`** ([mgr_nvm.h](MGR_NVM/Inc/mgr_nvm.h)) — single versioned
  struct (magic `"CONF"`, `NVM_VERSION=8`, trailing CRC-32/MPEG-2),
  64-bit-aligned, must fit one 2 KB flash page. Holds TX schedule, SWS config +
  runtime calibration, battery, rate-limiter, low-battery, event-driven LPM
  thresholds, payload format.
- **SWS detection** ([mgr_sws.h](MGR_SWS/Inc/mgr_sws.h)) — `MGR_SWS_State_t`
  (UNKNOWN/SURFACE/UNDERWATER) driven by L1–L5 drop detectors + adaptive
  baselines/peak; `MGR_SWS_msUntilNextSample` is the deadline source for the LPM
  scheduler; `MGR_SWS_getFault` reports stuck/out-of-range/no-variance faults.
- **LPM strategy** ([mgr_lpm_uw.h](MGR_LPM_UW/Inc/mgr_lpm_uw.h)) —
  `MGR_LPM_UW_idleTick` (spin / SLEEP / STOP2 by `LpmThr`) is the preferred
  entry; `enterStop2Timed` resumes in-place (RAM + MAC intact),
  `enterStandbyTimed` cold-boots, `enterShutdownReed` powers off until a magnet
  edge. Config + thresholds persist in `.retentionRamNoload`.
- **Persistence tiers (by reset survivability)** —
  - RAM-only config getters/setters: live values.
  - SRAM2 retention BSS (`.retentionRamBss`): `MGR_EVTLOG`, `MGR_RATE`,
    `MGR_TXSTATS` — survive IWDG/software resets, zeroed by BOR/NRST(PIN)/OBL.
  - SRAM2 retention NOLOAD (`.retentionRamNoload`): `MGR_LPM_UW` duty config,
    `MGR_ERR` crash forensics (`MGR_ERR_CrashInfo_t`) — survive cold-boot,
    lost only on VBAT loss.
  - TAMP backup registers: `MGR_ERR` reset/crash counters, `MGR_GESTURE` mode.
  - Flash: `MGR_NVM` config, `MGR_PMLOG` ERROR log, `MGR_CRED` credential
    mirror — survive full power-off / dead battery.
- **Gesture FSM** ([mgr_gesture.h](MGR_GESTURE/Inc/mgr_gesture.h)) —
  `MGR_GESTURE_Mode_t` persisted in TAMP; `MGR_GESTURE_task` reads `MGR_REED`
  events/hold-times and emits `MGR_GESTURE_Event_t` (enter OPERATIONAL/CONFIG,
  request SHUTDOWN) that the app applies. 3 s hold = mode switch question, 6 s =
  shutdown question, 2 s OFF→ON confirm window.
- **Error / forensics** ([mgr_err.h](MGR_ERR/Inc/mgr_err.h)) — `MGR_ERR_Code_t`
  reset reasons; ISR-safe `MGR_ERR_LOG_FAULT` macro (raw register writes) and
  full `MGR_ERR_captureFault` (8-word stacked frame + SCB fault regs →
  retention RAM); `MGR_ERR_checkCrashLoop` parks the chip in STOP on a boot-loop.

## Integration

- **Used by**: the per-app entry points `Kineis/App/kns_app_uw_doppler.c`
  (and `kns_app_doppler.c`) wire these managers into init + the main loop. MAC
  events flow through `MGR_AT_CMD_macEvtProcess` / `MGR_SPI_CMD_macEvtProcess`.
- **Depends on**: the MCU wrappers in `Kineis/App/Mcu` (`mcu_spi_driver`,
  `mcu_flash`, `mcu_nvm`, `mcu_aes`, UART console), `kns_types`, and the BSP
  pin definitions. `MGR_PMLOG` / `MGR_CRED` depend on linker regions
  `FLASH_PMLOG` and the credential-mirror page in `STM32WL55XX_FLASH_APP.ld`.
- **Build flags**:
  - `APP` — selects the manager set. `UW_DOPPLER` pulls in SWS/NVM/BAT/ERR/WDG/
    EVTLOG/RATE/TXSTATS/PMLOG/CRED/GESTURE/LPM_UW; `DOPPLER` pulls a smaller
    subset (ERR/WDG/EVTLOG, BAT on STDALONE); `STDLN`/`GUI` use only the core
    AT/SPI managers.
  - `COMM` — `UART` (`USE_UART_DRIVER`) uses `MGR_AT_CMD`; `SPI`
    (`USE_SPI_DRIVER`) additionally compiles all of `MGR_SPI_CMD`.
  - `BOARD` / `BSP_HAS_LED_RGB`, `BSP_HAS_REED` — `MGR_LED` / `MGR_REED`
    headers are always on the include path but their calls are compiled only on
    boards that have the hardware (e.g. `SMD_STDALONE`). Reed wake-pin variants
    select via `REED_WKUP1_WIRE` / `REED_WKUP3_WIRE` / `REED_WKUP3`.
  - `DEBUG` — `DEBUG=0` tears down the UART console by design, so the AT
    console (`MGR_AT_CMD`) is silent in production builds. `SPI_VERBOSE_LOG`
    (off by default) gates verbose SPI-path logging.
  - Always `make clean` when changing any flag — make does not track flag
    changes and silently reuses stale objects.

## Gotchas / constraints

- **`NVM_Config_t` layout is append-only & versioned.** Total size must stay
  within one 2 KB flash page; fields are 64-bit aligned for flash writes. Bump
  `NVM_VERSION` and add at the end; do not reorder existing fields.
- **Flash regions are load-bearing.** `MGR_PMLOG_FLASH_ADDR` (`0x08032000`) and
  `MGR_CRED_MIRROR_ADDR` (`0x08032800`) must stay in sync with the linker; they
  live *outside* `FLASH_USER`. `MGR_CRED` exists because page 0 of FLASH_USER
  mixes immutable credentials with RMW wear-leveling counters — a brownout
  during a counter RMW can blank the creds, so the mirror auto-restores them at
  boot. The credential offsets are a fixed contract with the GUI/STANDALONE
  provisioning tool; do not move them.
- **`MGR_WDG_ensureIwdgStopOptionByte` programs an option byte and resets the
  chip** (does not return on the program path). Call it early, before any LPM
  client is active.
- **`MGR_ERR` / forensics ordering**: `MGR_ERR_init` must run after
  backup-access is enabled in `main()`; the crash struct must be read before
  `Sram2_Init` zeroes the retention-BSS sections.
- **Retention tiers are not interchangeable.** `.retentionRamBss`
  (EVTLOG/RATE/TXSTATS) is wiped by BOR/NRST(PIN)/OBL; only `.retentionRamNoload`
  (LPM_UW duty cfg, crash forensics) and flash survive a cold boot. Anything
  needed across a STANDBY/SHUTDOWN cold-boot must be in NVM, the NOLOAD section,
  or TAMP.
- **LED composite colours need the soft-PWM tick.** On SMD_STDALONE the RGB has
  a single anode resistor, so WHITE/VIOLET/CYAN/YELLOW degrade to RED unless
  `MGR_LED_softTick()` is wired into `SysTick_Handler` every 1 ms. Use single
  R/G/B codes for reliable display. The LED pins overlap the bootloader SPI bus.
- **`MGR_REED` debounce vs STOP2.** Check `MGR_REED_isDebouncing()` before
  entering STOP2 so the chip does not re-sleep mid-debounce and miss the magnet
  edge; `MGR_REED_blankUntil` suppresses PA TX-transient coupling into the
  high-impedance reed node.
- **SPI A+ vs legacy / `cmdInProgress`.** The protocol auto-detects A+
  (magic-framed) vs legacy (raw command byte). Handlers operate on
  `cmdInProgress`; the pipelined design means a response always lags its request
  by one transaction — host code must account for the one-frame offset.
- **MGR_AT_CMD prefix shadowing.** Command enum order matters: longer-prefix
  commands (e.g. `AT_SWSFORCE` before `AT_SWS`) must precede their shorter
  siblings so the dispatcher matches the intended command.

See sibling subsystems: `Kineis/App/Mcu` (HAL/flash/SPI/AES/NVM wrappers),
`Kineis/App/kns_app_uw_doppler.c` (state machine), and the closed-source
libkineis MAC referenced via `kns_types.h`.
