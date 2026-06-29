# Core — MCU boot, clocks, peripherals and ISRs

## Purpose
STM32CubeMX-style hardware bring-up layer for the STM32WL55 Argos/Kineis
tracker. Owns the boot sequence (reset-cause capture, DFU-flag check, option-byte
guard, SRAM2 retention), `SystemClock_Config`, every CubeMX peripheral init
(`MX_*_Init`) plus its HAL MSP, all interrupt service routines, and the
per-board pin map (BSP). Everything above this layer (Kineis MAC/APP, the
`MGR_*` managers, `lpm.c`) is brought up from [`main.c`](Core/Src/main.c).

## Files
| File | Role |
|------|------|
| [Src/main.c](Core/Src/main.c) | `main()` boot sequence, `SystemClock_Config`, `IDLE_task` (LPM entry), DFU jump, OB/SRAM-retention guard, `Error_Handler`, `assert_failed` |
| [Inc/main.h](Core/Inc/main.h) | Common defines; selects the BSP header from the `SMD_*` board macro; `request_dfu_mode` / `SystemClock_armLsiFallback` prototypes |
| [Src/gpio.c](Core/Src/gpio.c) | `MX_GPIO_Init` — unused pins to analog, PA_PSU_EN/SEL drive, board-conditional pin carve-outs (LED/REED/VBAT) |
| [Src/usart.c](Core/Src/usart.c) | LPUART1 console (`MX_LPUART1_UART_Init`, MSP), `APP_UART_*` enable/RX-activity helpers |
| [Src/spi.c](Core/Src/spi.c) | SPI1 slave + DMA (`MX_SPI1_Init`, MSP) — only built with `USE_SPI_DRIVER` |
| [Src/subghz.c](Core/Src/subghz.c) | SubGHz radio bring-up (`MX_SUBGHZ_Init`) incl. post-bootloader radio reset/wake |
| [Src/rtc.c](Core/Src/rtc.c) | RTC init (`MX_RTC_Init`), LPM-aware re-init, LSE/LSI clock-source selection in MSP |
| [Src/tim.c](Core/Src/tim.c) | TIM16 1 ms base (`MX_TIM16_Init`) used by `mcu_tim.c` for MAC TX timing |
| [Src/adc.c](Core/Src/adc.c) | ADC on PA11/IN7 for SWS + VREFINT; `ADC_ReadValueChecked` (UW_DOPPLER only) |
| [Src/stm32wlxx_it.c](Core/Src/stm32wlxx_it.c) | All ISRs: SysTick, fault handlers, RTC_WKUP, TIM16, LPUART1, SUBGHZ, SPI1/DMA, EXTI15_10 (NSS wake) |
| [Src/stm32wlxx_hal_msp.c](Core/Src/stm32wlxx_hal_msp.c) | Global `HAL_MspInit` (empty stub) |
| [Src/system_stm32wlxx.c](Core/Src/system_stm32wlxx.c) | CMSIS `SystemInit` / `SystemCoreClock` (vendor file) |
| [Src/app_header.c](Core/Src/app_header.c) | `application_header` placed at `0x08000200` for bootloader validation (CRC filled by `create_dfu.py`) |
| [Src/syscalls.c](Core/Src/syscalls.c), [Src/sysmem.c](Core/Src/sysmem.c) | newlib syscall stubs and `_sbrk` heap |
| [Inc/bsp/bsp_smd_*.h](Core/Inc/bsp/) | Per-board pin maps + capability flags (see BSP model below) |

## Boot sequence (`main()`)
1. `ensure_sram_preserved_on_reset()` — checks `FLASH_OPTR.SRAM_RST`; if SRAM
   would be wiped on reset, gates the OB write behind a PVD VDD check
   (`vdd_safe_for_ob_program`, level 3 ≈ 2.5 V), programs the bit and resets.
2. `check_dfu_request()` — reads the DFU magic from TAMP backup register
   `BKP0R`/`BKP1R` (survives reset) or the SRAM fallback `0x2000FFF8`; if valid
   and the bootloader vector at `0x08033000` looks sane, relocates `SCB->VTOR`,
   sets MSP and jumps to the bootloader.
3. `mspFillup()` — paints heap/stack with `0x55`/`0xAA` for overflow detection
   (checked later by `assertMspOverflow`).
4. Captures raw `RCC->CSR` (`g_boot_rcc_csr_raw`) and `PWR->EXTSCR`
   (`g_boot_pwr_extscr_raw`) **before** clearing the PINRST flag, so app code can
   read the true reset cause after HAL wipes it.
5. `SMD_STDALONE`-only: clears any stale PWR pull-downs on PC1/PB7 and drives
   PC1 (TPS63901 VSEL) HIGH bare-metal so `SystemClock_Config` runs at 3V3.
6. `HAL_Init()` → `SystemClock_Config()` → `LPM_SystemClockConfig()`.
7. `MX_GPIO_Init`, `MX_LPUART1_UART_Init`, `MX_SUBGHZ_Init`, `MX_TIM16_Init`,
   `MX_RTC_Init`; then `HAL_RTCEx_DeactivateWakeUpTimer` + `MCU_TIM_resetState`
   to kill any stale wake source / dangling callback table left in RTC backup
   regs after a reflash. `MX_SPI1_Init` / `MX_ADC_Init` are conditional.
8. Branches on `LPM_getMode()` (the persisted LPM, stored in RTC backup) to do
   wake-specific init: `Sram2_Init()` after SHUTDOWN, flag clears for
   STANDBY/STOP, RAM2 re-init on cold boot.
9. Registers Kineis OS queues + MAC/APP/IDLE tasks, then `KNS_OS_main()`.

`SystemClock_Config` brings up **HSI16 + PLL** (sysclk 48 MHz, scale 1) — a core
clock failure is fatal (`Error_Handler`). The RTC reference clock is configured
**separately**: LSE 32.768 kHz preferred, **LSI fallback** if the crystal does
not start (sets `g_rtc_use_lsi`, read by `rtc.c`'s `HAL_RTC_MspInit`). A latched
prior LSE death (`g_rtc_force_lsi` in `.retentionRamNoload`, armed by
`SystemClock_armLsiFallback`) forces LSI directly so a marginal crystal cannot
reset-loop.

## Key flows / data structures
- **IDLE_task** (`main.c`): the OS idle hook that enters low power. Per-APP
  variants all funnel to `LPM_enter()` after, on baremetal, disabling IRQs and
  re-checking `KNS_Q_isEvtInSomeQ()` / `MGR_AT_CMD_isPendingAt()`. Refuses
  STANDBY/SHUTDOWN while the wake pin (`EXT_WKUP_BUTTON`) is asserted HIGH; honors
  `LPM_spiStopGraceActive()` to keep the slave awake for a retried SPI txn.
- **Fault forensics** (`stm32wlxx_it.c`): in UW_DOPPLER/DOPPLER builds,
  HardFault/MemManage/BusFault/UsageFault use naked trampolines
  (`NAKED_FAULT_TRAMPOLINE`) that recover the stacked frame (MSP/PSP via
  `EXC_RETURN` bit 2) and call `fault_handler_c`, which prints SCB fault regs,
  persists the frame via `MGR_ERR_captureFault`, then `NVIC_SystemReset`. Other
  builds use bare reset handlers.
- **EXTI15_10_IRQHandler** (SPI builds): clears the PA15/NSS falling-edge flag
  used to wake from STOP; the SPI re-init is done in `LPM_stop_exit` and the
  triggering transaction is lost (host must retry).
- **RTC_WKUP_IRQHandler** is also *called manually* from `main()` after a
  STANDBY/SHUTDOWN wake to replay a possible RTC-timer wake.
- `app_header_t` (`app_header.c`): 256-byte struct at `0x08000200` with magic
  `"KINE"`, version, CRC and HW-compat flags for the DFU bootloader to validate.

## Integration
- **BOARD** (`SMD_PA` / `SMD_NOPA` / `SMD_STDALONE` / `SMD_OP`): selects the BSP
  header in `main.h`; missing → `#error`. Each header defines the board name,
  capability flags (`BSP_HAS_EXTERNAL_PA`, `BSP_HAS_LED_RGB`,
  `BSP_HAS_REED_SWITCH`, `BSP_HAS_PWR_LATCH`, `BSP_HAS_VBAT_ADC`), and pin
  macros (`PA_PSU_EN/SEL`, `EXT_WKUP_BUTTON`, LED/REED/VBAT/SWS). `SMD_STDALONE`
  is the sealed UW tracker (RGB LED on PA1/PB4/PB5, reed on PB6, power latch on
  PB7, SWS on PA11/PA12); `SMD_PA`/`SMD_OP` have external PA only; `SMD_NOPA` has
  no PA; `SMD_OP` is currently a `SMD_PA` clone. Capability flags gate big blocks
  in `main.c` and the ISRs (e.g. `BSP_HAS_LED_RGB` adds `MGR_LED_softTick()` to
  SysTick).
- **APP** (`USE_GUI_APP` / `USE_STDALONE_APP` / `USE_UW_DOPPLER_APP` /
  `USE_DOPPLER_APP`, mutually exclusive): picks which APP task is registered and
  which managers init. UW_DOPPLER/DOPPLER turn on fault forensics, `MGR_ERR`,
  crash-loop check, and route `Error_Handler` to `MGR_ERR_logAndReset` instead of
  spinning/`+RST`.
- **COMM** (`USE_UART_DRIVER` vs `USE_SPI_DRIVER`): UART builds register the AT
  console on LPUART1; SPI builds compile in `spi.c`, the SPI/DMA ISRs and the
  NSS-EXTI wake. SPI is refused for the (UW_)DOPPLER apps (PA15 = NSS = MCU_DONE).
- **DEBUG**: enables the BOOT-cause trace, `__HAL_DBGMCU_FREEZE_RTC`, and the
  `request_dfu_mode` UART dump; release builds make `assert_param` a no-op.
- **LPM**: `LPM_getMode()`/`LPM_forceMode()` (in [`lpm.c`](Kineis)) drive the
  wake-specific init switches and `IDLE_task`. `MX_RTC_Init` and `MX_GPIO_Init`
  branch on the persisted mode. Build flag `LPM_SHUTDOWN_ENABLED` widens the
  allowed persisted-mode range.
- Peripheral handles (`hlpuart1`, `hspi1`, `hrtc`, `htim16`, `hsubghz`, `hadc`)
  are the globals consumed by the `mcu_*` wrappers and the `MGR_*` managers.

## Gotchas / constraints
- **TIM16 prescaler `31999` is load-bearing**: `mcu_tim.c` arms the MAC
  TX-timeout as `timeout_ms*2-1` counts (deliberate 2× margin). Changing it
  changes that margin — re-check TCXO warm-up / max-TX headroom first.
- **Option-byte writes are dangerous**: `ensure_sram_preserved_on_reset` programs
  `FLASH_OPTR` only on a fresh chip and only when the PVD says VDD is safe; an OB
  write on a sagging supply is an unrecoverable brick. Do not remove the PVD gate.
- **`g_boot_rcc_csr_raw` is currently informational only**: the `Sram2_Init` gate
  in the cold-boot path reads the *already-cleared* `RCC->CSR` (RMVF ran at the
  PINRST check), so RAM2 retention across IWDG/SW faults is effectively OFF —
  conservative clean-RAM behaviour pending a bench-validated snapshot gate.
- **`MX_SUBGHZ_Init` force-resets the radio** (SPI clock off, `FORCE_RESET`, NSS
  toggle, CLR_ERRORS/CLR_IRQ, STDBY_RC) because the bootloader leaves the radio
  in an unknown state — required for IRQs to fire after a DFU jump.
- **STDALONE PA1/PB4/PB5 are dual-use**: RGB-LED cathodes in UART builds, SPI
  SCK/MISO/MOSI otherwise — `gpio.c` carves them conditionally on `SMD_STDALONE`.
- **`MX_GPIO_Init` drives PA_PSU_EN LOW and VSEL HIGH** at boot; on STDALONE the
  early bare-metal PC1 drive in `main()` is intentionally redundant (both
  idempotent) to survive the 1V8→3V3 VSEL ramp.
- **LPUART1 clock/baud depend on COMM/APP**: HSI16 @ 115200 for SPI or
  UW_DOPPLER (verbose), LSE @ 9600 for STDLN/GUI UART (low power). The
  `LPUART1_EXTI_ENABLE_IT` (IM28) keeps RX wake working in stop modes.
- **ADC `ADC_ReadValueChecked` returns false (not 0) on HAL failure** — a real
  fix so a dead ADC no longer reads as 0 → SWS flipping to SURFACE. Callers hold
  their last valid sample on false.

See sibling subsystems: `Kineis/` (`lpm.c`, `mcu_tim.c`, `mcu_misc.c`, MAC/APP),
`MGR_*` managers (`mgr_err`, `mgr_led`, `mgr_reed`, `mgr_sws`, `mgr_at_cmd`), and
the DFU bootloader at `0x08033000`.
