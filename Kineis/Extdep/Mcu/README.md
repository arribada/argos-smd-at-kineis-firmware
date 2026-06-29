# Kineis MCU Wrappers (`Kineis/Extdep/Mcu`)

## Purpose
Board/MCU abstraction layer (`MCU_*` API) that the closed-source `libkineis`
MAC stack links against. It implements every external dependency the stack
expects: non-volatile credential storage, message/wakeup counters, AES-CBC,
external-PA / TCXO / VSEL control, and the timer + RTC-wakeup callback table.
This is the integration seam between the STM32WL55 HAL (`Core/`, `Drivers/`)
and the opaque Kineis library — get it wrong and the MAC misbehaves silently.

## Files
| File | Role |
|------|------|
| [Inc/mcu_flash.h](Inc/mcu_flash.h) | **Load-bearing** `FLASH_USER` layout: offsets for ID/ADDR/SECKEY/RADIOCONF/app-vars + the two wear-leveling counter areas. Shared with GUI/STANDALONE apps. |
| [Src/mcu_flash.c](Src/mcu_flash.c) | Raw flash R/W (`MCU_FLASH_read/write`, page-erase+reprogram), 64-bit wear-leveling counter engine, MSG/WKU counter accessors. |
| [Inc/mcu_nvm.h](Inc/mcu_nvm.h) / [Src/mcu_nvm.c](Src/mcu_nvm.c) | Credential + counter API the lib calls: ID, ADDR, SN, radio-conf zone, message counter (MC), wakeup counter (WUC). Holds the 9-bit MC clamp + RAM high-water cache. |
| [Inc/mcu_aes.h](Inc/mcu_aes.h) / [Src/mcu_aes.c](Src/mcu_aes.c) | AES-128-CBC wrapper used by the protocol; stores/loads the Device Secret Key (DSK) from flash, falls back to a hardcoded test key when flash is erased. |
| [Inc/aes.h](Inc/aes.h) / [Src/aes.c](Src/aes.c) | Vendored Brian Gladman 8-bit AES (prekeyed enc/dec + CBC). Third-party, do not modify. |
| [Inc/mcu_misc.h](Inc/mcu_misc.h) / [Src/mcu_misc.c](Src/mcu_misc.c) | External-PA on/off, RF power settings (`rfSettings_t`), TCXO force/warmup, PA stuck-watchdog, VSEL (TPS63901 voltage select). |
| [Inc/mcu_tim.h](Inc/mcu_tim.h) / [Src/mcu_tim.c](Src/mcu_tim.c) | TX-timeout (TIM16) + TX-period (RTC wakeup) timers; HAL callback override dispatching through a retained callback table. |
| [Inc/mcu_nvm_blind_pos.h](Inc/mcu_nvm_blind_pos.h) / [Src/mcu_nvm_blind_pos.c](Src/mcu_nvm_blind_pos.c) | **Stub** for the BLIND_POS MAC profile (PER map / abacus / UL-cal). All return `KNS_STATUS_DISABLED` — BLIND_POS is not used on any SMD board. |

## Key flows / data structures

### FLASH_USER layout ([mcu_flash.h](Inc/mcu_flash.h))
`FLASH_USER_START_ADDR = 0x0803B000`, runs to flash end. Page 0 holds the
credentials at fixed offsets:
`ID(8) → ADDR(8) → SECKEY(16) → RADIOCONF(16) → app-vars(8) → MSG_OF(8) → WKU_OF(8)`.
The wear-leveling slot arrays start one page later
(`FLASH_MSG_COUNTER_WL_START_ADDR @ +FLASH_PAGE_SIZE`, 1024 dwords each, then
the WKU WL area), and `FLASH_NVM_CONFIG_ADDR` reserves the last free page
(currently unused: no CRC32/migration record is written today, despite the
name). **These offsets are an ABI shared with the GUI/STANDALONE images — do
not change them.**

### Wear-leveled counters ([mcu_flash.c](Src/mcu_flash.c))
`read_wear_counter` / `increment_wear_counter` / `set_wear_counter` /
`reset_wear_counter` implement a slot-program scheme: each `+1` programs one
erased dword (`0xFFFF…` → `0`) with **no page erase**; only when the 1024-slot
area fills does it bump the overflow word and erase-reset the area. Value =
`overflow * wl_size + valid_index`. `MCU_FLASH_{read,increment,reset,set}_{msg,wku}_counter`
are thin wrappers selecting the MSG vs WKU area. `MCU_FLASH_reset_msg_counter`
is currently a dead maintenance hook (no caller).

### Message counter — 9-bit clamp + RAM high-water ([mcu_nvm.c](Src/mcu_nvm.c))
The Argos/Kineis MC is a **9-bit field (0..511)**. `MCU_NVM_getMC`/`setMC`
mask `& 0x1FF` at every boundary because the lib writes the raw value into
both the 9-bit header and the 16-bit AES-CTR counter; an MC > 511 desyncs the
IV and corrupts the whole frame. `setMC` serves the value from a RAM cache
(`message_counter`, in `.msgCntSectionData`) and treats flash as a forward-only
high-water mark: small forward step → slot increments; small backward step
(per-sequence rollback) → RAM only; large jump (`AT+MC=<n>`) → genuine rewrite
realigned monotonically. This bounds flash wear (10k-cycle endurance) at
deployment TX rates. `mc_flash_shadow` is the 64-bit count known-in-flash.

### Credential getters
`MCU_NVM_getID/getAddr` and `MCU_AES_get_device_sec_key` read fixed-size flash
slots and substitute a built-in test value when the slot is all-`0xFF`
(erased). ID/ADDR use an 8-byte slot for a 4-byte value, reading/writing via a
sized local so the 4-byte caller object is never over-run. `MCU_NVM_getSN`
derives a 14-char serial (`"SMD"+3hex+8hex`) from the read-only 96-bit die UID
(`UID_BASE`) — survives any erase, no NUL terminator (caller appends).
`MCU_NVM_getRadioConfZonePtr` prefers a valid 16-byte flash RADIOCONF, else
falls back to the compile-time `radioConfZone[]`.

### AES ([mcu_aes.c](Src/mcu_aes.c))
`MCU_AES_128_init(NULL)` loads the DSK from flash (or test key); a non-NULL key
overrides it. `MCU_AES_128_cbc_encrypt/decrypt` forward to the vendored
`aes_cbc_*`. Single static `aes_context ctx`.

### Timers ([mcu_tim.c](Src/mcu_tim.c))
Two handlers: `MCU_TIM_HDLR_TX_TIMEOUT` (TIM16, 1 ms tick) and
`MCU_TIM_HDLR_TX_PERIOD` (RTC wakeup, 1 s tick). The callback table
`timer[]` lives in `.lpmSection` (mapped to **RTC backup registers**) so it
survives STOP/SHUTDOWN. `MCU_TIM_resetState()` must be called at boot:
the section is not loaded by the C runtime, so a stale `isr_cb` left by a prior
firmware would HardFault on dispatch. `HAL_TIM_PeriodElapsedCallback` /
`HAL_RTCEx_WakeUpTimerEventCallback` are overridden here and run
`mcu_tim_cb_is_valid()` (NULL + in-flash-range) before calling.

## Integration
- **Consumed by `libkineis`** (closed source) plus app code in `Kineis/` and
  `Core/`. The `MCU_*` symbol set is the contract — signatures must match
  `kns_types.h` expectations.
- Calls into the STM32 HAL (`stm32wlxx_hal.h`) and CubeMX-generated handles
  `htim16` ([Core tim.c](../../../Core/Src/tim.c)), `hrtc` (rtc.c), `hlpuart1`
  (usart.c). Logging via `mgr_log.h`.
- Sibling subsystems: power/LPM in `Kineis/` (`mgr_lpm*`) depend on the
  retained `timer[]` table and on `MCU_MISC_VSEL_*`; the bootloader shares the
  TAMP/RTC-backup region (see DFU-aliasing note below); LED shedding hooks
  `mgr_led.h` in `turn_on_pa`.
- **Build flags that matter:**
  - `BOARD` (`SMD_STDALONE` / `SMD_PA` / `SMD_OP` / `SMD_NOPA`): selects PA
    GPIO handling, RF power min/max + HPA vs external-PA gain in
    `MCU_MISC_getSettingsHwRf`, the default `radioConfZone[]`, and whether
    `VSEL`/PC1 is driven. On `SMD_STDALONE` PC1 = TPS63901 VSEL (whole-board
    VSYS), so it is left high-Z during PA inrush.
  - `DEBUG`: enables the gated TRACE-grade UART markers (`[PA-TRACE]`, `[<`/`[>`
    brownout signature) and flash program-error prints.
  - `MCU_PA_GPIO_ENABLE=0`: build-time switch to skip the PA enable GPIO write
    (validate TX stack without RF on brownout-prone boards).
  - `USE_SMPS_BYPASS_TX`: force SMPS to LDO/bypass around PA-on for TCXO noise.
  - `TCXO_FORCE_STATE_ENABLED` (default 1): drive HSE bypass-power when the MAC
    asks for the radio.
  - APP/LPM: BLIND_POS stubs return `DISABLED` (UW_DOPPLER uses BASIC). The
    `.lpmSection`/`.msgCntSectionData` placement depends on the linker script
    `STM32WL55XX_FLASH_APP.ld`.

## Gotchas / constraints
- **FLASH_USER offsets are an ABI.** Page 0 holds the live credentials; the OF
  (overflow) words sit on page 0 too, so the rare WL-area-full event does a
  page-0 erase+reprogram — a brownout there can corrupt credentials (the RAM
  high-water cache keeps this rare; credentials are not CRC-checked). Carve any
  new region *outside* FLASH_USER. Editing the linker needs explicit sign-off.
- **`MCU_FLASH_write` is page-granular and destructive**: it backs up the whole
  page to a static buffer, erases, and reprograms double-word by double-word
  (3-retry, IRQs disabled). It rejects writes outside FLASH_USER, across a page
  boundary, or not 64-bit aligned. Compiled `-O0` (`#pragma GCC optimize`) on
  purpose.
- **Never let MC exceed 511** — the `& 0x1FF` clamp at every getter/setter is
  load-bearing; removing it ships a corrupt first frame after boot.
- **`timer[]` is in RTC backup** and not zeroed by the runtime: call
  `MCU_TIM_resetState()` at boot before the lib arms any timer, or risk a
  stale-pointer HardFault after reflash.
- **DFU-flag aliasing:** `.lpmSection` is placed at the RTC backup origin, so
  `timer[0]` overlaps TAMP BKP0R/BKP1R which the bootloader uses as a DFU flag.
  Safe only because the app routes DFU through the SRAM flag — do not add a
  TAMP-based DFU path while the MAC owns `timer[0]`.
- **TX_TIMEOUT is armed at 2× the requested ms on purpose** (`cnt = ms*2-1`)
  for warmup + ~1.7 s max-TX headroom — do not "fix" it without rechecking
  margins. `MCU_TIM_getCount` for TX_TIMEOUT has no live caller.
- **TCXO force may time out** on `SMD_STDALONE` (VDDTCXO gated by SubGHz, not
  PB0); this is non-fatal and TX still works.
- **`MCU_MISC_VSEL_set(false)` drops the whole board to 1.8 V** — radio can't
  TX, TCXO may stop, flash/SPI writes can glitch. App-level deep-idle only,
  never from MAC/ISR, always restore HIGH before radio activity.
- **DSK / test credentials**: the hardcoded `test_device_*` values and
  `test_device_secret_key` are placeholders; real credentials come from Kineis
  and overwrite flash. The DSK is stored in plaintext flash (no key-wrapping
  yet — noted `WARNING` in `mcu_aes.h`).
- The header comment claiming "NVM config + CRC32 + migration" does **not**
  match the code: `FLASH_NVM_CONFIG_ADDR` is reserved but unused, and there is
  no CRC32 or migration logic in this folder today.
