# Bootloader — Standalone DFU bootloader (STM32WL55)

## Purpose

Standalone in-application DFU (device firmware update) bootloader for the
STM32WL55, built as a **separate image** that lives at `0x08033000` (after the
204 KB application, before the Kineis credentials). It re-flashes the
application region over **UART** (`AT+DFU=` text commands) or **SPI** (binary
A+/legacy protocol), validates the new image (header + CRC-32/MPEG-2), and
jumps to it. It never lets the host write the bootloader region or the Kineis
credential pages.

## Files

| File | Role |
|------|------|
| [Src/bl_main.c](Src/bl_main.c) | Entry point, clock/HW init, DFU-flag capture, the `bl_state_t` state machine (`bl_run`), protocol auto-detect/forced path, validated `bl_jump_to_app`, IWDG refresh, `early_debug_print` over LPUART1, optional `BL_LED` activity indicator |
| [Inc/bl_main.h](Inc/bl_main.h) | Public API for init/run/state/jump/version; `BL_DBG` macro (gated on `BL_DEBUG`) |
| [Src/bl_flash.c](Src/bl_flash.c) | Bounded flash erase/write/read/verify (64-bit aligned), region guards, DFU-request flag read/clear via TAMP+SRAM |
| [Inc/bl_flash.h](Inc/bl_flash.h) | Flash API + `bl_flash_status_t`; declares the flash-resident state functions (compiled out) |
| [Src/bl_dfu.c](Src/bl_dfu.c) | Transport-agnostic DFU command handlers (`PING`/`INFO`/`ERASE`/`WRITE`/`READ`/`VERIFY`/`SET_HEADER`/`ABORT`/`STATUS`); `dfu_context_t` session state; `bl_dfu_can_jump` gate |
| [Inc/bl_dfu.h](Inc/bl_dfu.h) | DFU handler API + `dfu_context_t` |
| [Src/bl_uart.c](Src/bl_uart.c) | UART transport: LPUART1 IRQ ring-buffer RX, `AT+DFU=` line assembly, `+DFU=OK/ERR` responses |
| [Inc/bl_uart.h](Inc/bl_uart.h) | UART transport API |
| [Src/bl_spi.c](Src/bl_spi.c) | SPI **slave** transport (polling, NSS=PA15 hard input), fixed 280-byte transactions, idle/busy patterns, multi-transaction WRITE/READ handshakes |
| [Inc/bl_spi.h](Inc/bl_spi.h) | SPI transport API |
| [Src/bl_spi_protocol.c](Src/bl_spi_protocol.c) | A+ frame parse/build (magic `0xAA`/`0x55`, seq, CRC-8 CCITT), legacy-mode fallback, extended-status builder |
| [Inc/bl_spi_protocol.h](Inc/bl_spi_protocol.h) | A+ frame layout, status codes, `bl_extended_status_t`, op-state enum |
| [Src/bl_crc.c](Src/bl_crc.c) | Software CRC-32/MPEG-2 (poly `0x04C11DB7`, init `0xFFFFFFFF`, no reflect/no final XOR) — single-shot, accumulate, and flash-region variants |
| [Inc/bl_crc.h](Inc/bl_crc.h) | CRC API |
| [Src/bl_syscalls.c](Src/bl_syscalls.c) | Weak `_read/_write/_close/_lseek` stubs (suppress newlib `-lnosys` warnings; no functional change) |
| [Inc/bl_config.h](Inc/bl_config.h) | **Single source of truth**: flash/RAM layout, magic values, DFU command/response enums, `bl_state_t`, timeouts, buffer sizes |
| [Inc/bl_app_header.h](Inc/bl_app_header.h) | 256-byte `app_header_t` (magic `"KINE"`, version, app CRC-32, addresses) + inline validators |
| [STM32WL55XX_BL.ld](STM32WL55XX_BL.ld) | Linker: ROM `@0x08033000` (32K), `.noinit` DFU flag `@0x2000FFF8`, 4K stack |
| [startup_bl.s](startup_bl.s) | Vector table + `Reset_Handler` |
| [Makefile](Makefile) | Builds with `arm-none-eabi-gcc`; `PROTOCOL`, `DEBUG`, `BL_LED`, `BL_UART_BAUD` flags; `flash` target (J-Link, erases `0x08033000`–`0x0803B000`) |

## Key flows / data structures

### Boot / DFU entry flow

1. The **application** receives `AT+BOOT` (or the SPI/AT BOOT command), writes
   `DFU_REQUEST_MAGIC` (`0x4446554D` = `"DFUM"`) to **TAMP_BKP0R**
   (`0x4000B100`, survives reset) and the **SRAM** flag (`0x2000FFF8`),
   optionally writes a protocol selector to `0x2000FFFC`
   (`DFU_PROTO_UART`/`DFU_PROTO_SPI`/`DFU_PROTO_NONE`), then transfers control
   to the bootloader at `0x08033000` (see `Core/Src/main.c`).
2. `main()` (bl_main.c) captures the SRAM/proto flags **before** any memory
   init, enables backup-domain access, reads TAMP, sets `SCB->VTOR =
   BL_FLASH_BASE`, prints the banner, and **clears all three flags**.
3. `bl_run()` drives `bl_state_t`:
   `INIT → CHECK_APP / DETECT_PROTOCOL → DFU_UART|DFU_SPI → VALIDATE →
   JUMP_APP` (with an `ERROR` recovery state that loops back to the DFU state).
   - `DETECT_PROTOCOL`: a **forced** proto flag skips detection; otherwise a
     ~3 s race (`BL_DETECTION_TIMEOUT_MS`) checks SPI first, then UART.
   - If no DFU was requested and the app is valid, it jumps straight to the app.

### DFU session (the contract, enforced in `bl_dfu.c`)

`ERASE` (starts session, erases app region, resets CRC) → repeated `WRITE`
(`[addr(4)][data]`, 64-bit aligned, CRC accumulated) → `VERIFY` (`[crc(4)]`,
compares accumulated CRC vs host) → `JUMP`. `JUMP` is gated by
`bl_dfu_can_jump()` which requires `session_active && erase_done &&
verify_passed`. `WRITE` returns `NOT_READY` if `ERASE` was not run first.

### Application validation & jump

`bl_check_app_valid()` first tries the `app_header_t` path
(`bl_validate_app_header` + `bl_validate_app_crc` over `bl_crc32_flash`); if no
valid `"KINE"` header is present it **falls back** to a sanity check of the
reset vector at `0x08000000` (stack pointer in SRAM, entry point in app flash).
`bl_jump_to_app()` re-validates SP/PC, resets SPI1/DMA1, disables IRQs, clears
SysTick, sets `SCB->VTOR`, `__set_MSP`, and branches to the reset handler.

### Transports

- **UART** ([bl_uart.c](Src/bl_uart.c)): LPUART1 (PA2 TX / PA3 RX), IRQ ring
  buffer, line-terminated `AT+DFU=...` parsing; oversized lines are dropped
  whole (never acted on as a truncated `WRITE`). Responses are
  `+DFU=OK[,data]` / `+DFU=ERR,<reason>`.
- **SPI** ([bl_spi.c](Src/bl_spi.c) + [bl_spi_protocol.c](Src/bl_spi_protocol.c)):
  SPI1 **slave**, polled, NSS = PA15 hard input, MISO pre-filled with
  `BL_SPI_IDLE_PATTERN` (`0xAA`). A+ frame = `[0xAA][SEQ][CMD][LEN][DATA][CRC8]`;
  response = `[0x55][SEQ][STATUS][LEN][DATA][CRC8]`. SPI command IDs are
  `SPI_CMD_DFU_BASE (0x30)` + `dfu_cmd_t`. `WRITE`/`READ` use a two-transaction
  REQ/DATA handshake. `bl_spi_wait_tx_done()` must run before any state change
  (JUMP/RESET/VALIDATE) so the full response (incl. CRC) is clocked out.

### Important structs

- `dfu_context_t` ([bl_dfu.h](Inc/bl_dfu.h)) — per-session state: write addr,
  received size, expected/calculated CRC, and the `session_active`/
  `erase_done`/`verify_passed` gates.
- `app_header_t` ([bl_app_header.h](Inc/bl_app_header.h)) — 256-byte
  (`_Static_assert`) image descriptor at `0x08000200`.
- `bl_spi_protocol_ctx_t` / `bl_extended_status_t` — A+ parser state and the
  `GET_STATUS` reply consumed by the SPI host.

## Integration

- **Separate build, separate image.** This is its own Makefile and linker
  script — **not** compiled into the main firmware. It is flashed once to
  `0x08033000` (`make flash` here) and re-used across app updates.
- **Layout coupling with the app** (all in [bl_config.h](Inc/bl_config.h),
  must stay in sync with the app's `STM32WL55XX_FLASH_APP.ld` and
  `Kineis/Extdep/Mcu/Inc/mcu_flash.h`):
  - App region `0x08000000`–`0x08031FFF` (`APP_FLASH_SIZE = 0x32000`, **200 KB**,
    *not* the 204 KB up to the BL — the app linker carves out `FLASH_PMLOG`
    `@0x08032000`, whose 2nd page mirrors the Kineis credentials for brick
    recovery).
  - Bootloader `0x08033000`–`0x0803AFFF` (32 KB).
  - `FLASH_USER` (Kineis credentials/config) `0x0803B000`–`0x0803FFFF` (20 KB).
- **Hand-off contract** with `Core/Src/main.c`: the magic, flag addresses
  (`TAMP_BKP0R`, SRAM `0x2000FFF8`/`0x2000FFFC`) and proto values are duplicated
  on both sides — change them together. Cross-references the app's AT layer
  (`MGR_AT_CMD`, `AT+BOOT`) and SPI layer (`MGR_SPI_CMD`).
- **CRC parity**: `bl_crc.c` is byte-compatible with the Zephyr host
  `argos_dfu_crc32()`; the SPI transaction size (280) matches that host driver.
- **Build flags** (this folder's Makefile):
  - `PROTOCOL=SPI|UART` (default `SPI`) — both transports are **always
    compiled in** (`USE_SPI_DRIVER` is always defined); this flag only sets the
    default UART baud (`UART` → 9600, `SPI` → 115200 for debug output) and the
    `BL_PROTOCOL_*` define.
  - `BL_UART_BAUD=<n>` — override UART-DFU baud independent of `PROTOCOL`
    (e.g. a UW_DOPPLER board whose console is 115200 needs `PROTOCOL=UART
    BL_UART_BAUD=115200`).
  - `DEBUG=1` — defines `DEBUG`→`BL_DEBUG`, enabling `BL_DBG`/`[CMD]`/`[CRC]`
    traces on LPUART1. Off in release.
  - `BL_LED=1` — opt-in RGB activity indicator (PA1/PB4/PB5, active-low,
    R→G→B scan); off by default so deployed bootloaders stay byte-identical.
  - The app-side `APP`/`BOARD`/`COMM`/`LPM` flags do **not** apply here — the
    bootloader has its own minimal HAL set and no Kineis/MAC/LPM code.

## Gotchas / constraints

- **IWDG is inherited, not started here.** The app arms IWDG (~16 s, cannot be
  disabled by software, survives reset). `bl_run()` refreshes it every
  iteration (`IWDG->KR = 0xAAAA`); without it UART DFU stalls deterministically
  after ~16 s (~44 KB) and looks like "flash fails at 40%". The longest single
  op (full app erase ≈2.8 s) stays under budget — keep page-erase short.
- **Flash region guards are the safety net.** `bl_flash_erase_page` /
  `bl_flash_write_doubleword` refuse the bootloader region; `bl_dfu_cmd_write`
  bounds **both** start and (alignment-padded) end to the app region via
  `bl_flash_addr_in_app`, so a top-of-region chunk can't overrun into
  `FLASH_PMLOG`/credentials. Writes are 64-bit aligned and read-back verified.
- **`DFU_CMD_READ` is intentionally unbounded** (BL-01, audit 2026-06-27):
  unlike `WRITE`, it has no app-region clamp, so a host in DFU mode can read
  the entire 256 KB flash — **including the Kineis AES secret key/credentials**
  at `0x0803B000` and the `FLASH_PMLOG` mirror. Physical/host access is
  required (post `AT+BOOT`). The fix (same guard as `WRITE`) is documented
  in-line in [bl_dfu.c](Src/bl_dfu.c) but deliberately not applied.
- **Flash-resident BL state is DEAD CODE.** `BL_STATE_FLASH_ADDR` aliases
  `FLASH_USER` page 0 (the credentials). It is compiled out
  (`BL_STATE_PERSIST_ENABLED 0`) so `bl_flash_write_bl_state` can never erase
  the credentials; production DFU signalling uses TAMP_BKP0R + SRAM only. The
  guarded `_Static_assert` in [bl_config.h](Inc/bl_config.h) fails the build if
  it is re-enabled without relocating the page first. Do **not** flip this flag.
- **SPI is a polled slave.** Tight RX/TX loops avoid `HAL_GetTick()` in the
  inner loop to prevent TX-FIFO underrun up to ~4 MHz SPI; only the first 4
  response bytes are pre-filled into the FIFO, so `bl_spi_wait_tx_done()` is
  mandatory before JUMP/RESET/VALIDATE or the response (incl. CRC) is truncated.
- **SPI bus pins overlap the SMD_STDALONE RGB LEDs** (PA1/PB4/PB5). `BL_LED`
  drives them only in the UART-DFU states and turns them off before any SPI
  init or app jump, so the bus is never disturbed.
- **`BL_CMD_BUFFER_SIZE` (600) must hold a full `WRITE` line.** At
  `BL_CHUNK_SIZE=248` a `WRITE` line is ~520 chars; undersizing silently
  truncates it into a short `WRITE` and corrupts the image. RX ring is 1024 to
  absorb the next line arriving while a flash write blocks the consumer.
- **Header is optional.** Current builds may ship without a `"KINE"` header;
  validation then relies on the vector-table fallback. `SET_HEADER` exists for
  future signed/headered images.
