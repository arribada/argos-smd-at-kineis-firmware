# Kineis/App — Application layer

## Purpose

Top of the firmware stack: the per-application entry/loop functions that drive the
closed-source Kineis MAC to transmit user data, plus the minimalist bare-metal OS
(`KNS_OS` scheduler + `KNS_Q` queues) and small string/FIFO helper libs. One app is
compiled at a time, selected by an `APP=` build flag, and registered as the single
`KNS_OS_TASK_APP` task. The `Managers/` and `Mcu/` subtrees are documented separately.

## Files

### Application entry/loops (`Kineis/App/*.c`)

| File | Role |
|------|------|
| [kns_app.c](kns_app.c) / [kns_app.h](kns_app.h) | Hosts **two** legacy apps: `KNS_APP_stdln_loop` (STDLN — self-test that sends random payload(s) once) and `KNS_APP_gui_init`/`KNS_APP_gui_loop` (GUI — AT/SPI command console in front of the Kineis Device Interface). |
| [kns_app_doppler.c](kns_app_doppler.c) / [.h](kns_app_doppler.h) | **DOPPLER** app: periodic blind transmitter. `KNS_APP_doppler_init` + `KNS_APP_doppler_loop`. Sends N msgs/sequence, sleeps between sequences via TPL5111 `MCU_DONE` pulse or RTC-wake `SHUTDOWN`. NVM config (`DopplerNvmConfig_t`, magic `DPLR`) + CRC32. |
| [kns_app_uw_doppler.c](kns_app_uw_doppler.c) / [.h](kns_app_uw_doppler.h) | **UW_DOPPLER** app (production turtle tag): SWS surface detection drives growth-interval TX bursts. `KNS_APP_uw_doppler_init` + `KNS_APP_uw_doppler_loop`. Largest app; LB (low-battery) mode, jitter, cooldown, episode stats, deploy/reed gestures, retention-RAM state. |

### Kineis OS (`Kineis_os/`)

| File | Role |
|------|------|
| [Kineis_os/KNS_OS/Src/kns_os.c](Kineis_os/KNS_OS/Src/kns_os.c) | Bare-metal cooperative scheduler. `KNS_OS_registerTask()` fills `taskPool[]`; `KNS_OS_main()` is the infinite round-robin loop that also flushes the debug log (gated on `DEBUG`, paused during SPI `PROCESS_CMD`). |
| [Kineis_os/KNS_Q/Src/kns_q.c](Kineis_os/KNS_Q/Src/kns_q.c) | Dispatch shim: `#include`s exactly one backend per `USE_BAREMETAL` / `USE_FREERTOS` / `USE_CMSIS_OS2` (compile error if more than one). |
| [Kineis_os/KNS_Q/Src/kns_q_baremetal.c](Kineis_os/KNS_Q/Src/kns_q_baremetal.c) | Active backend. Static array FIFOs with read/write index roll-over, `KNS_CS_enter/exit` critical sections + per-queue mutex, priority preemption (`KNS_Q_isEvtInHigherPrioQ`), LPM gate (`KNS_Q_isEvtInSomeQ`). |
| [Kineis_os/KNS_Q/Src/kns_q_freertos.c](Kineis_os/KNS_Q/Src/kns_q_freertos.c), [kns_q_cmsis_os2.c](Kineis_os/KNS_Q/Src/kns_q_cmsis_os2.c) | Alternate RTOS backends (not built in this firmware). |
| [Kineis_os/KNS_OS/Inc/kns_os.h](Kineis_os/KNS_OS/Inc/kns_os.h), [Kineis_os/KNS_Q/Inc/kns_q.h](Kineis_os/KNS_Q/Inc/kns_q.h) | Public API. |

### Libs (`Libs/`)

| File | Role |
|------|------|
| [Libs/STRUTIL/Src/strutil_lib.c](Libs/STRUTIL/Src/strutil_lib.c) | Two helpers: `bUTIL_strcmp` (bounded compare) and `u8UTIL_convertCharToHex4bits` (ASCII-hex digit → nibble, `0xFF` on failure). Used by the AT/SPI parsers. |
| [Libs/USERDATA/Src/user_data.c](Libs/USERDATA/Src/user_data.c) / [.h](Libs/USERDATA/Inc/user_data.h) | TX/RX user-data FIFO as a chained list (`sUserDataTxFifoElt_t`), reserve/add/remove/flush + client callbacks, stored in `.retentionRamData`. Used by the GUI app's AT path; **not** used by DOPPLER/UW_DOPPLER (they push TX directly to the MAC). |

> `Managers/` (MGR_AT_CMD, MGR_SPI_CMD, MGR_SWS, MGR_LPM_UW, MGR_LED, MGR_NVM, MGR_ERR, …)
> and `Mcu/` live under this folder but are out of scope here — see their own files.
> The OS task enum (`kns_os_conf.h`) and queue config (`kns_q_conf.{h,c}`) live in
> `Kineis/Appconf/` and `Kineis/Extdep/Conf/` respectively.

## The four apps and their state machines (high level)

All apps are a single function repeatedly called by `KNS_OS_main`; they keep state in
statics (often retention RAM) and drive the MAC purely through queues.

- **STDLN** (`KNS_APP_stdln_loop`): 6-state switch. Init MAC profile → wait OK → push one
  `KNS_MAC_SEND_DATA` with a random payload sized per modulation → wait `TX_DONE` → stop MAC →
  idle. A built-in smoke test (`TEST_PASS`/`TEST_FAIL` markers).
- **GUI** (`KNS_APP_gui_init`/`_loop`): no app state machine of its own. `init` starts the
  command manager (`MGR_AT_CMD_start` over UART, or `MGR_SPI_CMD_start` over SPI) and pushes a
  one-shot `KNS_MAC_INIT`. `loop` just pumps the command decoder and `*_macEvtProcess()`; all TX
  is operator-driven via `AT+TX` and the USERDATA FIFO.
- **DOPPLER** (`DopplerState_t`): `BOOT → CHECK_SCHEDULE → INIT_MAC → WAIT_MAC_READY →
  SEND_MSG → WAIT_TX_DONE → (WAIT_INTERVAL ↺ | SEQUENCE_DONE)`. BASIC profile = app paces each
  message (`msg_interval_s`); BLIND profile = MAC does the retx and one `TX_DONE` ends the
  sequence. `SEQUENCE_DONE` either pulses `MCU_DONE` (TPL5111 cuts power) or RTC-wake `SHUTDOWN`.
- **UW_DOPPLER** (`UwDopplerState_t`): `BOOT → BOOT_DEPLOY_LED → INIT_MAC → WAIT_MAC_READY →
  (CRED_FAIL terminal) → MONITORING ⇄ SURFACE_TX → WAIT_TX_DONE`, plus `SHUTDOWN_BLINK` for the
  reed-hold recovery gesture. SWS detects surface; first TX is immediate, then
  `T(n)=T_initial·(1+growth/100)^n` capped at `tx_max_interval_s`/`tx_max_count`, with jitter,
  cooldown and a low-battery mode. State lives in `.retentionRamNoload` to survive STOP2/SHUTDOWN.

## Key flows / data structures

### Task model
`KNS_OS_TASK_APP`, `KNS_OS_TASK_MAC`, `KNS_OS_TASK_IDLE` (defined in `kns_os_conf.h`,
ordered low→high priority but bare-metal has no real preemption). `Core/Src/main.c` calls
`KNS_OS_registerTask()` for the selected app loop, `KNS_MAC_task`, and `IDLE_task`, then enters
`KNS_OS_main()` which round-robins them forever.

### Queue model (`KNS_Q`)
Five statically-sized FIFOs (`enum KNS_Q_handle_t` in `kns_q_conf.h`), priority ascending:
`KNS_Q_UL_MAC2APP_EVT_LIST` (0) < `KNS_Q_DL_APP2MAC` (1) < `KNS_Q_UL_MAC2APP` (2) <
`KNS_Q_UL_INFRA2MAC` (3) < `KNS_Q_UL_SRVC2MAC` (4). The app↔MAC contract is just two:
- **`KNS_Q_DL_APP2MAC`** — app → MAC. App pushes `KNS_MAC_appEvt_t` (`KNS_MAC_INIT`,
  `KNS_MAC_SEND_DATA`).
- **`KNS_Q_UL_MAC2APP`** — MAC → app. App pops `KNS_MAC_srvcEvt_t` (`KNS_MAC_OK`,
  `KNS_MAC_TX_DONE`, `KNS_MAC_TX_TIMEOUT`, `KNS_MAC_TX_ABORT`, `KNS_MAC_ERROR`).

`KNS_Q_pop` returns `QEMPTY` whenever a higher-priority queue has work, so a lower-priority
consumer yields to let the OS run the higher task — this is how priority is emulated. Each
`q_desc_t` carries an `isLpmBlocker` flag consulted by `KNS_Q_isEvtInSomeQ()`.

### IDLE / LPM hook
`IDLE_task` (in `Core/Src/main.c`) is the lowest task. Per-app blocks disable IRQs, check
`KNS_Q_isEvtInSomeQ() || MGR_AT_CMD_isPendingAt()`, and only call `LPM_enter()` if nothing is
pending — guaranteeing no event is lost across a sleep. UW_DOPPLER/DOPPLER use this path for
STOP/STOP2/SHUTDOWN cycling; the apps themselves register an LPM client
(`MgrLpmClientCb_t`, e.g. `doppler_lpm_client`) so MGR_LPM can query the desired mode and notify
enter/exit.

## Integration

- **APP** (`APP=GUI|STDLN|DOPPLER|UW_DOPPLER` → `-DUSE_GUI_APP` / `USE_STDALONE_APP` /
  `USE_DOPPLER_APP` / `USE_UW_DOPPLER_APP`): selects exactly one app loop; `Core/Src/main.c`
  enforces mutual exclusion (`#error` if >1, default GUI) and wires init + task registration.
- **BOARD** (`SMD_STDALONE|SMD_PA|SMD_NOPA|SMD_OP`): sets `BSP_HAS_*` (LED_RGB, REED_SWITCH,
  PWR_LATCH, VBAT_ADC) which gate LED/battery/reed code. UW_DOPPLER `#error`s unless the board
  has reed + RGB LED + PWR latch (SMD_STDALONE only).
- **COMM** (`UART`→`USE_UART_DRIVER`, `SPI`→`USE_SPI_DRIVER`): GUI uses MGR_AT_CMD (UART) or
  MGR_SPI_CMD (SPI). DOPPLER/UW_DOPPLER are UART-only — UW_DOPPLER `#error`s on `USE_SPI_DRIVER`,
  and the Makefile refuses `COMM=SPI` for both.
- **MAC_PRFL** (`BASIC|BLIND|…`): chooses `KNS_MAC_PRFL_*` at `KNS_MAC_INIT`. UW_DOPPLER hard-
  requires BASIC (`#error` otherwise — it owns its own scheduling); STDLN/DOPPLER accept BASIC or
  BLIND (`prflBlindUserCfg`).
- **DEBUG / VERBOSE**: enable `MGR_LOG_*` and the `KNS_OS_main` log flush; `PRINT_TEST_ASSERT`
  in STDLN. **LPM** (`-DLPM_SHUTDOWN_ENABLED`): DOPPLER without `MCU_DONE` `#error`s without it.
- **OS backend** (`USE_BAREMETAL=1`): selects `kns_q_baremetal.c` and the bare-metal `IDLE_task`
  LPM gating; the FreeRTOS/CMSIS-OS2 paths exist but are unused here.

Downstream this layer feeds the closed-source **libkineis MAC** (`KNS_MAC_*`, `KNS_CFG_*`) via
the queues, and leans on the sibling **Managers** (AT/SPI command, SWS, LPM_UW, LED, NVM, ERR,
EVTLOG, BAT, REED, WDG, CRED) and **Mcu** wrappers.

## Gotchas / constraints

- **DPL-001 / DOPPLER NULL-deref trap** (kns_app_doppler.c loop): DOPPLER/UW_DOPPLER must **not**
  call `MGR_AT_CMD_macEvtProcess()` — it pops `KNS_Q_UL_MAC2APP` (the queue the app's own
  `process_mac_events()` owns) then derefs `USERDATA_txFifoGetFirst()`, which is NULL for these
  apps (TX is pushed directly, never via the USERDATA FIFO) → `kns_assert` → reset loop. Only the
  AT *decoder* is pumped for config.
- **One app, one MAC profile, compile-locked**: UW_DOPPLER `#error`s on BLIND/SATDET/BLIND_POS and
  on SPI/wrong board; runtime `AT+KMAC` also refuses profile switches. Don't relax these.
- **FIFO sizing**: every `KNS_Q` queue is allocated one element longer than needed (the
  always-empty slot is how full/empty is distinguished) — see `kns_q.h` and `kns_q_conf.h`.
- **Retention RAM**: app state and the USERDATA FIFO live in `.retentionRamData` /
  `.retentionRamNoload` so they survive STOP/STANDBY/SHUTDOWN — don't move them to plain `.bss`.
- **Message Counter is 9-bit**: UW_DOPPLER session MC (`setSessionMC`) must stay 0..511 or the AES
  IV desyncs the frame.
- **Log flush timing**: `KNS_OS_main` skips `MGR_LOG_flush()` while SPI is in `PROCESS_CMD` so a
  response isn't delayed — keep that guard if you touch the loop.
- **`KNS_Q_create` is a static-config validator**, not an allocator, in bare-metal: it only checks
  the requested length/element-size match the pre-defined `qPool[]` descriptor.
