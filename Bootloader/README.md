# Bootloader — Standalone DFU Bootloader (STM32WL55)

## Purpose

Standalone in-application firmware-update (DFU) bootloader for the STM32WL55,
built as a **separate binary** (own [Makefile](Makefile),
[linker script](STM32WL55XX_BL.ld), [startup_bl.s](startup_bl.s)) and flashed at
`0x08033000`. It receives a new application image over UART or SPI, programs it
into the application region `0x08000000–0x0803_1FFF`, validates a CRC-32, and
jumps to the app. It is reached after the running app receives `AT+BOOT` and
resets/jumps with a DFU flag set.

## Files

| File | Role |
|------|------|
| [Src/bl_main.c](Src/bl_main.c) | `main()`, HW/clock init, the `bl_state_t` state machine (`bl_run`), protocol detection, app-validity check, and `bl_jump_to_app()`. Also early-LPUART1 debug, IWDG petting, `HardFault_Handler`, optional `BL_LED`. |
| [Src/bl_dfu.c](Src/bl_dfu.c) | Transport-agnostic DFU command handlers (`bl_dfu_process_cmd` + `bl_dfu_cmd_*`). Owns the `dfu_context_t` session state (erase_done, write_addr, accumulated CRC, verify_passed). |
| [Src/bl_flash.c](Src/bl_flash.c) | Bounded flash erase/write/read/verify (64-bit doubleword programming), region guards, and RTC/SRAM DFU-flag read/clear. |
| [Src/bl_crc.c](Src/bl_crc.c) | Software CRC-32/MPEG-2 (poly `0x04C11DB7`, init `0xFFFFFFFF`, no reflection / no final XOR), streaming accumulate + flash-region CRC. |
| [Src/bl_uart.c](Src/bl_uart.c) | LPUART1 (PA2/PA3) IRQ-driven `AT+DFU=...` line transport: ring buffer, line assembly, ASCII/hex response formatting (`+DFU=OK`/`+DFU=ERR,...`). |
| [Src/bl_spi.c](Src/bl_spi.c) | SPI1 **slave**, polled (PA1 SCK, PA15 NSS, PB4 MISO, PB5 MOSI). Transaction framing, idle/busy patterns, multi-transaction WRITE/READ staging. |
| [Src/bl_spi_protocol.c](Src/bl_spi_protocol.c) | "A+" SPI frame layer: `[0xAA][SEQ][CMD][LEN][DATA][CRC8]` request / `[0x55]...` response, CRC-8 (poly `0x07`), legacy direct-command fallback, extended status. |
| [Src/bl_syscalls.c](Src/bl_syscalls.c) | Weak `_read/_write/_close/_lseek` stubs to suppress newlib-nano `-lnosys` warnings; no functional behaviour. |
| [Inc/bl_config.h](Inc/bl_config.h) | Single source of truth for flash layout, DFU flag addresses/magics, command IDs, response codes, `bl_state_t`, buffer sizes. Doxygen `@page` documents the boot flow. |
| [Inc/bl_app_header.h](Inc/bl_app_header.h) | 256-byte `app_header_t` (magic `"KINE"`, version, `app_crc32`, addresses, HW-compat flags) + inline validators. `_Static_assert` enforces 256 bytes. |
| [Inc/bl_main.h](Inc/bl_main.h) / [Inc/bl_dfu.h](Inc/bl_dfu.h) / [Inc/bl_flash.h](Inc/bl_flash.h) / [Inc/bl_crc.h](Inc/bl_crc.h) / [Inc/bl_uart.h](Inc/bl_uart.h) / [Inc/bl_spi.h](Inc/bl_spi.h) | Public interfaces for each module. |
| [Makefile](Makefile) | Standalone build (`arm-none-eabi-gcc`, `-Os`, `nano.specs`), `PROTOCOL`/`DEBUG`/`BL_LED`/`BL_UART_BAUD` flags, JLink `flash` target writing `0x08033000`. |
| [STM32WL55XX_BL.ld](STM32WL55XX_BL.ld) | Bootloader-only memory map: `ROM @0x08033000 (32K)`, `RAM_NOINIT @0x2000FFF8` (shared DFU flag), `_app_addr = 0x08000000`. |

## Key flows / data structures

### Boot / DFU entry flow

1. App receives `AT+BOOT` → sets DFU magic `0x4446554D` ("DFUM") in **TAMP_BKP0R**
   (`0x4000B100`) and SRAM (`0x2000FFF8`), the protocol selector in SRAM
   (`0x2000FFFC`: `"UART"`/`"SPI!"`/`0`=auto), then either `NVIC_SystemReset()`
   or a direct `SCB->VTOR`+MSP jump to `0x08033000` (see
   [Core/Src/main.c](../Core/Src/main.c) `jumpToBootloader`).
2. BL `main()` captures the SRAM/TAMP flags **before** any memory init, prints a
   banner over LPUART1, clears the flags, then runs `bl_init()` → `bl_run()`.
3. State machine in `bl_run()`:
   `INIT → CHECK_APP → DETECT_PROTOCOL → DFU_UART | DFU_SPI → VALIDATE → JUMP_APP`.
   - `INIT`: DFU requested → `DETECT_PROTOCOL`; else `CHECK_APP`.
   - `CHECK_APP`: valid app → `JUMP_APP`, else `DETECT_PROTOCOL`.
   - `DETECT_PROTOCOL`: forced protocol skips detection; otherwise a
     `BL_DETECTION_TIMEOUT_MS` (3 s) **race** between SPI and UART (SPI checked
     first; UART RX flushed after a 5 ms settle to drop AF-switch noise).
   - `JUMP_APP`: `bl_jump_to_app()` — reads SP/entry from the app vector (or
     `app_header_t` if present), validates SP in RAM and entry in flash, resets
     SPI1/DMA1, disables IRQs/SysTick, sets `VTOR`, `__set_MSP`, branches.

### DFU command set (`dfu_cmd_t`, shared UART/SPI)

`PING(0x01) GET_INFO(0x02) ERASE(0x03) WRITE(0x04) READ(0x05) VERIFY(0x06)
RESET(0x07) JUMP(0x08) GET_STATUS(0x09) ABORT(0x0A) SET_HEADER(0x0B) ENTER(0x0F)`.
SPI uses the same IDs +`SPI_CMD_DFU_BASE` (`0x30`).

Normal session: **ERASE** (starts session, resets streaming CRC) → repeated
**WRITE** `[addr(4)][data]` (64-bit aligned, end-bounded into the app region,
write-back verified, CRC accumulated) → **VERIFY** `[crc(4)]` (compares against
`bl_crc32_get()`) → **JUMP** (gated by `bl_dfu_can_jump()` = verify_passed).

### `dfu_context_t` (bl_dfu.c)

Tracks `session_active`, `erase_done`, `write_addr`, `received_size`,
`expected_crc`/`calculated_crc`, `verify_passed`, `last_error`.
`WRITE` is rejected with `NOT_READY` unless a prior `ERASE` set
`session_active && erase_done`; `JUMP` requires `verify_passed`.

## Integration

- **App ↔ BL contract** is the shared trio in [bl_config.h](Inc/bl_config.h) and
  mirrored in [Core/Src/main.c](../Core/Src/main.c): DFU magic `"DFUM"`, TAMP/SRAM
  flag addresses, protocol selector values, and `BOOTLOADER_ADDR 0x08033000`.
  Keep both files in sync — these are a wire ABI.
- **Flash regions** (256 KB total): application `0x08000000` (`APP_FLASH_SIZE
  0x32000` = 200 KB usable), bootloader `0x08033000` (32 KB), then `FLASH_PMLOG
  @0x08032000` (credential mirror, reserved) and **FLASH_USER credentials
  @0x0803B000** (Kineis ID/ADDR/SECKEY). Sibling subsystems:
  [mcu_flash.h](../Kineis/Extdep/Mcu/Inc/mcu_flash.h) owns `FLASH_USER`;
  the app linker [STM32WL55XX_FLASH_APP.ld](../STM32WL55XX_FLASH_APP.ld) carves
  these regions. The BL never erases its own region or `FLASH_USER` via
  `bl_flash_addr_in_bootloader` + the app-bounded WRITE guard.
- **Build flags** (this folder's [Makefile](Makefile), independent of the app build):
  - `PROTOCOL=SPI` (default) / `PROTOCOL=UART` — both transports are always
    compiled in; the flag only sets the **default UART baud** (`BL_PROTOCOL_UART`
    → 9600, else 115200) and the boot-banner baud.
  - `BL_UART_BAUD=<n>` — overrides UART-DFU baud independently of `PROTOCOL`
    (e.g. UW_DOPPLER console at 115200).
  - `DEBUG=1` — defines `BL_DEBUG`, enabling `BL_DBG`/CRC trace over LPUART1.
  - `BL_LED=1` — opt-in RGB activity scan on PA1/PB4/PB5 (off by default so
    deployed bootloaders stay byte-identical; those pins are also the SPI1 bus).
  - There is **no app-side `BOARD`/`COMM`/`LPM`/`APP`** flag here — the BL is
    board-agnostic and does not link libkineis, HAL LPM, or the app managers.

## Gotchas / constraints

- **IWDG inheritance.** The app arms IWDG (~16 s, cannot be disabled by SW and
  survives reset). `bl_run()` pets `IWDG->KR = 0xAAAA` every loop iteration; the
  longest single op (full app erase ≈ 2.8 s) stays under budget. Without this
  the UART DFU stalls deterministically (~40 %) — see the long comment in
  [bl_main.c](Src/bl_main.c).
- **APP size cap is 200 KB, not 204 KB.** `APP_FLASH_SIZE = 0x32000` stops below
  `FLASH_PMLOG @0x08032000`. WRITE must stay `<= 0x32000`; the WRITE handler
  bounds `address + aligned_len - 1` into the app region precisely to avoid
  overrunning into the credential mirror.
- **READ is intentionally unbounded (BL-01).** `bl_dfu_cmd_read` only clamps to
  the full 256 KB flash, so a host in DFU mode can read the AES secret key /
  credentials. Left open on purpose (confidentiality not a current priority);
  the fix (apply the same `bl_flash_addr_in_app` guard as WRITE) is documented
  inline in [bl_dfu.c](Src/bl_dfu.c).
- **Flash-resident BL state is DEAD CODE.** `BL_STATE_FLASH_ADDR 0x0803B000`
  aliases `FLASH_USER` page 0 (credentials). It is compiled out
  (`BL_STATE_PERSIST_ENABLED 0`) with a `_Static_assert` landmine; production
  DFU signalling uses TAMP_BKP0R + SRAM only. Do **not** enable it without
  relocating the address first.
- **SPI is slave + polling, NSS-driven** (`bl_spi_irq_handler` is a no-op). The
  poll loops feed the TX FIFO with `0xAA` idle and time out on a CPU-cycle count
  (~200 ms) to tolerate up to ~4 MHz SPI; `bl_spi_wait_tx_done()` MUST be called
  before VALIDATE/JUMP/RESET or only the 4 pre-filled FIFO bytes (no CRC) reach
  the master. Sibling app-side SPI lives in
  [MGR_SPI_CMD](../Kineis/App/Managers/MGR_SPI_CMD).
- **Two SPI protocol modes.** A+ frames (magic `0xAA`/`0x55`, CRC-8) are CRC-
  enforced; a legacy direct-command byte (`0x30–0x3F`) is accepted with
  `crc_valid = true`.
- **UART line discipline.** A command line that exceeds `BL_CMD_BUFFER_SIZE`
  (600) is dropped whole (`cmd_overflow`) rather than acted on as a truncated
  WRITE; chunks are 248 bytes (`31×8`) for doubleword alignment.
- **App validation has a legacy fallback.** If no valid `app_header_t` (magic
  `"KINE"`) exists, `bl_check_app_valid` accepts a plausible raw vector table at
  `0x08000000` (SP in SRAM, entry in flash) — so a header is optional.
- **Standalone build.** This folder builds and flashes on its own
  (`make PROTOCOL=... [DEBUG=1] [BL_LED=1]`, `make flash`); it is not part of the
  app Makefile. Always `make clean` when changing a flag (objects don't track
  flag changes).
