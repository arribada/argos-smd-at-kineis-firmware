# Kineis/App/Mcu — Application-side MCU console drivers

## Purpose

MCU "wrapper" layer that the Kineis application talks to for host I/O. It provides
two physical transports for the same logical role (host command in, response out):
a UART/LPUART AT-command console (`COMM=UART`) and an SPI-slave command channel
(`COMM=SPI`). Only one is compiled in per build, selected by the `COMM` Makefile
flag. These are concrete board implementations of the generic `MCU_AT_CONSOLE_*`
and `MCU_SPI_DRIVER_*` APIs consumed by the managers in
[Kineis/App/Managers](../Managers).

## Files

| File | Role |
|------|------|
| [Inc/mcu_at_console.h](Inc/mcu_at_console.h) | Public AT-console API: `MCU_AT_CONSOLE_register`, `MCU_AT_CONSOLE_send` (printf-style), `MCU_AT_CONSOLE_send_dataBuf` (binary→hex ASCII). |
| [Src/mcu_at_console.c](Src/mcu_at_console.c) | Thin dispatcher: `#include`s the per-MCU impl (`mcu_at_console_axm0.c` if `AXM0`, else `mcu_at_console_stm.c`). |
| [Src/mcu_at_console_stm.c](Src/mcu_at_console_stm.c) | STM32WL/WLE implementation of the AT console over UART/LPUART, char-by-char RX ISR streaming + UART-priority TX. |
| [Inc/mcu_spi_driver.h](Inc/mcu_spi_driver.h) | Public SPI-slave API + protocol constants (`SPI_TRANSACTION_SIZE`, idle/busy patterns), `SPI_Buffer`, `SpiState`, `SPI_Stats_t`. Guarded by `USE_SPI_DRIVER`. |
| [Src/mcu_spi_driver.c](Src/mcu_spi_driver.c) | SPI1-slave driver: fixed 64-byte pipelined DMA TxRx, transaction-end polling, error/abort/reset recovery. Used only when `COMM=SPI`. |

> Note: the header carries a stale `@file mcu_spi_console.h` doc tag; the real file
> is `mcu_spi_driver.h`. There is no `mcu_spi_driver_*` per-MCU split — only one
> STM impl exists.

## Key flows / data structures

### AT console (UART) — `mcu_at_console_stm.c`
- **RX**: `MCU_AT_CONSOLE_register(huart, rx_evt_cb)` arms `KINEIS_UART_StartRx_IT`,
  which installs a custom ISR `KINEIS_RxISR_8BIT`. That ISR drains every byte in
  `LPUART1->RDR` in one interrupt and invokes the client callback `rxEvtCb` once
  per burst so AT chars are treated as a continuous stream. On RX-buffer overflow
  it wraps to the start (whole buffer may be lost — by design, "garbage" case).
- **TX**: `MCU_AT_CONSOLE_send` formats via `vsnprintf` into `uartTxBuf` (no
  overflow), then blocks on `HAL_UART_Transmit`. It wraps the send in
  `MGR_LOG_pause()/MGR_LOG_resume()` so AT responses take absolute priority over
  ring-buffered debug logs (see [MGR_LOG](../../Extdep/MGR_LOG)).
- **Diagnostics**: `g_at_isr_bytes`, `g_at_parse_calls`, `g_at_cb_null` localise a
  broken AT pipeline (consumed by the heartbeat trace in
  [kns_app_uw_doppler.c](../kns_app_uw_doppler.c)).
- `MCU_AT_CONSOLE_send_dataBuf` emits a bit-length buffer as hex ASCII, handling a
  trailing partial nibble.

### SPI slave (pipelined single-transaction) — `mcu_spi_driver.c`
Every transaction is a fixed **64 bytes** (`SPI_TRANSACTION_SIZE`). The master
sends command N while the slave simultaneously shifts out the response to command
**N-1**. Because the response is already staged in the TX buffer before CS asserts,
there are no slave-prep timing races. For an immediate reply the master sends a NOP
after the real command. Two filler patterns flow out when no real data is staged:
`SPI_IDLE_PATTERN` (0xAA) and `SPI_BUSY_PATTERN` (0xBB).

Buffers/state (defined here, declared `extern` in the header):
- `spiTxBuf`/`spiRxBuf` and the `SPI_Buffer rxBuf`/`txBuf` wrappers (`data`,
  `size`, `next_req`).
- `volatile bool response_ready` — TX buffer holds a real response.
- `SPI_Stats_t spi_stats` — rx/tx/error/timeout/reset counters + `last_error`.
- `SpiState spiState` is the manager's state enum (`SPICMD_IDLE`, `_PROCESS_CMD`,
  `_ERROR`, …); it lives in [mgr_spi_cmd.c](../Managers/MGR_SPI_CMD/Src/mgr_spi_cmd.c)
  and the driver only writes it on the read/arm paths.

Driver entry points:
- `MCU_SPI_DRIVER_register(handle, cb)` — stashes the `SPI_HandleTypeDef*`, fills
  idle, fires the first `MCU_SPI_DRIVER_read`. The `rx_spi_evt_cb` arg is **ignored**
  (pipelined mode does not use a per-RX callback; legacy signature kept).
- `MCU_SPI_DRIVER_read()` — re-arms `HAL_SPI_TransmitReceive_DMA` for the next 64B
  transaction with whatever is currently staged (response or idle), aborting/clearing
  a non-READY SPI first. `MCU_SPI_DRIVER_writeread` is a macro alias to this.
- `MCU_SPI_DRIVER_set_response(data, len)` — copies a response (≤64B) into the TX
  buffer, idle-pads the tail, sets `response_ready`.
- `MCU_SPI_DRIVER_arm_busy()` — stages 0xBB and re-arms, so the host sees "busy"
  during a long op (e.g. flash write) and retries.
- `MCU_SPI_DRIVER_check_transaction_end(&n)` — **the wedge-safe completion path**.
  Instead of trusting `HAL_SPI_TxRxCpltCallback` (which a short/early-CS host may
  never trigger), it polls the RX DMA counter (`__HAL_DMA_GET_COUNTER`) and declares
  the transaction done once the byte count is stable for `RX_STABLE_TIMEOUT_MS` (3ms;
  64B @125kHz ≈ 4ms). `expected_transfer_size` is cached because HAL may zero
  `RxXferSize` after completion.
- `MCU_SPI_DRIVER_abort_transfer()` — `HAL_SPI_Abort` + clear detection state.
- `MCU_SPI_DRIVER_reset(hspi)` — full recovery: abort, `SPI1` force/release reset,
  disable IT, clear pending IRQ, `DeInit`/`Init`, re-arm. Bumps `reset_count`.

ISR callbacks defined here override the HAL weak ones:
- `HAL_SPI_TxRxCpltCallback` — asserts SPI1, bumps rx/tx counters, clears the
  timeout tick (state machine does the real work via polling).
- `HAL_SPI_ErrorCallback` — clears OVR/MODF/FRE and continues; mid-transaction
  errors from early CS-release are treated as normal.

### Pairing with MGR_SPI_CMD
The driver is the byte transport; [MGR_SPI_CMD](../Managers/MGR_SPI_CMD) is the
protocol/state machine. Round-trip:
1. `MGR_SPI_CMD_init` → `MCU_SPI_DRIVER_register` arms the first DMA.
2. The manager's poll (in `SPICMD_IDLE`) calls `MCU_SPI_DRIVER_check_transaction_end`;
   on completion it `MCU_SPI_DRIVER_abort_transfer`s and parses the 64B frame.
3. After processing, `MGR_SPI_CMD_process_cmd` (and `mgr_spi_cmd_common.c`) call
   `MCU_SPI_DRIVER_set_response(txBuf.data, txBuf.next_req)` to stage the reply for
   the *next* transaction (the pipeline), then `MCU_SPI_DRIVER_read` to re-arm.
4. On error states the manager calls `MCU_SPI_DRIVER_reset`. A wedge-recovery
   watchdog lives on the manager side (`SPICMD_IDLE` tick), backed by this driver's
   abort/reset primitives.

## Integration

- **`COMM`** (the dominant flag): `COMM=SPI` defines `USE_SPI_DRIVER` and pulls
  `mcu_spi_driver.c` + the `MGR_SPI_CMD` sources + `Core/Src/spi.c` +
  `stm32wlxx_hal_spi.c` into the build (Makefile ~L431-439). `COMM=UART` defines
  `USE_UART_DRIVER` and the AT console path is used instead. The whole SPI header
  body is `#ifdef USE_SPI_DRIVER`. `mcu_at_console.c` is always compiled (Makefile
  ~L228); the SPI driver is not unless `COMM=SPI`.
- **`APP`**: `UW_DOPPLER` (the turtle-tracker app) is **UART-only** — the Makefile
  hard-`$(error)`s on `COMM=SPI` for it (~L131-137); SPI is for `APP=GUI`/`STDLN`
  hosts. So in the default deployment build, only the AT console here is live.
- **`AXM0`**: selects the non-STM AT console impl in `mcu_at_console.c`.
- **`USE_HDA4`**: enlarges TX/RX buffers to 2560B (else 256B) in both the AT console
  and the SPI header.
- **`DEBUG`/log verbosity**: driver chatter uses `MGR_LOG_DEBUG` and the
  `SPI_LOG_VERBOSE` macro, which is a no-op unless `SPI_VERBOSE_LOG` is defined in
  [mgr_spi_cmd.h](../Managers/MGR_SPI_CMD/Inc/mgr_spi_cmd.h). With `DEBUG=0` the
  console UART is also torn down by design (see prod-UART teardown).
- **`LPM`**: SPI must be re-armed after wake; the LPM path references
  `MCU_SPI_DRIVER_*` (see [lpm.c](../../Lpm/Src/lpm.c)). Known caveat per project
  notes: SPI works in all modes except deep LPM.

## Gotchas / constraints

- **64 bytes is hard-coded.** All sizing (`SPI_CMD_MAX_SIZE`/`SPI_RESPONSE_MAX_SIZE`
  = 32 each) assumes the fixed 64B transaction. `set_response` rejects `len > 64`.
- **Completion is polled, not interrupt-driven.** Hosts that release CS early or
  send <64B never fire `TxRxCpltCallback`; rely on `check_transaction_end`'s
  DMA-counter stability timer (3ms). Mid-transaction OVR/FRE/MODF errors are
  expected and swallowed.
- **`register` callback arg is dead** in pipelined mode — pass `NULL`. Don't assume
  a per-RX event fires.
- **Pipeline lag**: a response is shifted out on the transaction *after* the command;
  the master must clock an extra (NOP) transaction to read it immediately.
- **Header doc tag is stale** (`mcu_spi_console.h`), and `MCU_SPI_DRIVER_check_tx_complete()`
  is a compatibility shim hard-wired to `true`.
- **Single global handle**: `hspi_handle` is file-static and SPI1-only
  (`HAL_SPI_TxRxCpltCallback` asserts on any other instance). No reentrancy.
- **AT RX overflow loses the whole buffer** (intentional, non-nominal); no circular
  buffer (see `@todo` in `KINEIS_RxISR_8BIT`).
- **AT TX blocks** with a 500ms `HAL_UART_Transmit` timeout and pauses debug logs;
  long responses stall the caller for that window.
