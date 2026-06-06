# PROJECT_RULES.md — argos-smd-at-kineis-firmware

Project context, hardware constraints, protocol specs, and engineering
constraints specific to this firmware. Read this BEFORE making any HW-level
changes.

---

## 1. Project goal

Build a satellite tracker firmware for **marine turtles** using:
- **Kineis Argos network** (Doppler localization)
- **STM32WL55JC** as single-MCU (sub-GHz radio integrated)
- **SMD STANDALONE board** with reed-switch UI, RGB LED, SWS electrode,
  battery

The deployed tag is encapsulated in epoxy. Once deployed, the **magnet
gesture protocol is the only way to interact** with the device. UART is
available for debug/dev only (no USB on this module).

### Apps and BSPs in this repo

The codebase supports multiple targets:

| App | BSP | Purpose |
|---|---|---|
| `USE_UW_DOPPLER_APP` | `SMD_STDALONE` | Turtle tag (this work) |
| `USE_DOPPLER_APP` | `SMD_PA`, `SMD_OP` | Other Doppler trackers |
| `USE_GUI_APP` | various | AT-command interface for PC GUI |

**Common code (`Drivers/`, `Kineis/Extdep/`, `Core/`, linker `STM32WL55XX_FLASH_APP.ld`, flash layout offsets) must remain compatible with all three.**
Do not refactor in a way that breaks GUI or DOPPLER builds.

---

## 2. Hardware — STM32WL55JC + STANDALONE board

### MCU
- STM32WL55JC8U6 (M4 core only used; M0+ dormant on this firmware)
- 256 KB Flash, 64 KB SRAM (32 KB SRAM1 + 32 KB SRAM2 with retention)
- Integrated sub-GHz radio (SX126x-class)
- Clock: 48 MHz from HSI + PLL (per `SystemClock_Config`)
- TCXO control via PB0 (HSEBYPPWR) — see "TCXO" section below

### Pinout (validated against DS13105)

| Pin | Function | Code symbol |
|---|---|---|
| **PA0** | UART RX | `LPUART_RX_Pin` |
| **PA1** | UART TX | `LPUART_TX_Pin` (also SCK if SPI) |
| **PA11** | SWS analog input → **ADC_IN7** | `SWS_IN_Pin` |
| **PA12** | SWS electrode power control | `SWS_POWER_PIN` |
| **PA13/14** | SWDIO / SWCLK | (debug) |
| **PA15** | SPI NSS (when SPI build) | — |
| **PB0** | HSEBYPPWR (TCXO supply control) | (clock) |
| **PB3** | SWO + PWR_WAKEUP_PIN3 (HIGH wake) | (debug + wake) |
| **PB4** | LED_GREEN or SPI MISO | `LED_GREEN_Pin` |
| **PB5** | LED_BLUE or SPI MOSI | `LED_BLUE_Pin` |
| **PB6** | Reed switch (EXTI both edges, pull-down) | `REED_MCU_Pin` |
| **PB7** | PWR_LATCH (HIGH = board powered) | `PWR_LATCH_Pin` |
| **PB9** | BAT_SENSE_EN (HIGH = enable divider) | `VBAT_EN_Pin` |
| **PB13** | BAT_SENSE → **ADC_IN0** (via 120k/300k divider) | — |
| **PC0** | PA_PSU_EN (HIGH = PA powered) | `PA_PSU_EN_Pin` |

**LED is APHF1608LSEEQBDZGKC** (common-anode RGB) — per datasheet:
- Pin 1 = common anode
- Pin 2 = RED
- Pin 3 = GREEN
- Pin 4 = BLUE

User-confirmed schematic has pin 3 and 4 swapped vs datasheet → on this
board, firmware `LED_GREEN` actually lights BLUE, and `LED_BLUE` lights
GREEN. **The firmware MGR_LED color mapping must be swapped to compensate**
(action item, not yet applied).

### Battery
- LiPo or 2x AA, 2.5 V – 4.2 V range
- Read via PB13 + external divider (R4=120k / R5=300k)
- BAT_SENSE max ≈ 2.91 V at ADC input (when VBAT = 4.2 V)
- VBAT measurement: enable PB9 → wait 2 ms → ADC read CH0 → disable PB9
- VREFINT used for accurate VDDA calculation

### Power latch
- PWR_LATCH (PB7) HIGH = board powered via external regulator
- Setting PWR_LATCH LOW kills the board (only reed magnet circuit can
  re-trigger power)
- The reed magnet activates an external HW circuit that pulses PWR_LATCH
  HIGH on magnet ON → MCU then drives PB7 HIGH to maintain its own power

---

## 3. TCXO control (critical — has bitten us)

- On STANDALONE board, the TCXO is supplied via VDDTCXO which IS controlled
  by the MCU's HSEBYPPWR (PB0).
- The MAC stack calls `MCU_MISC_TCXO_Force_State(true)` before any radio
  operation, which:
  1. Enables HSE in BYPASS_PWR mode (`RCC_OscInitStruct.HSEState =
     RCC_HSE_BYPASS_PWR`)
  2. Waits for HSERDY
  3. Returns OK only when TCXO is stable
- **`TCXO_FORCE_STATE_ENABLED` MUST be 1** on STANDALONE — disabling it
  breaks MAC init (HSE never asserts, radio config decryption fails or
  reads stale data).
- **`tcxo_warmup_time_ms` MUST be ≥ 2000ms** on STANDALONE (TCXO + PA rail
  need this long to stabilize per the GR5504 PA characteristics).

**Both values are HEAD baseline. Past attempts to reduce them broke MAC.
Do not change without explicit user OK and lab verification.**

---

## 4. Flash layout (DO NOT MODIFY — shared GUI/DOPPLER/UW_DOPPLER)

```
0x08000000 – 0x08031FFF  ROM (200 KB)         App code + vectors + header
0x08032000 – 0x08032FFF  FLASH_PMLOG (4 KB)   Post-mortem log (UW_DOPPLER)
0x08033000 – 0x0803AFFF  Bootloader (32 KB)   DFU updater
0x0803B000 – 0x0803FFFF  FLASH_USER (20 KB)   Kineis credentials + NVM
                                              (offsets defined in mcu_flash.h)
```

### FLASH_USER offsets (per `Kineis/Extdep/Mcu/Inc/mcu_flash.h`) — IMMUTABLE

| Offset | Size | Content |
|---|---|---|
| 0 | 8 | FLASH_ID |
| 8 | 8 | FLASH_ADDR |
| 16 | 16 | FLASH_SECKEY (DSK) |
| 32 | 16 | FLASH_RADIOCONF (AES 16 bytes) |
| 48 | 8 | FLASH_APP_VARS (legacy GUI/STANDALONE shared) |
| 56 | 8 | FLASH_MSG_COUNTER_OF |
| 64 | 8 | FLASH_WKU_COUNTER_OF |
| 2048 | 8192 | FLASH_MSG_COUNTER_WL (wear leveling) |
| 10240 | 8192 | FLASH_WKU_COUNTER_WL (wear leveling) |
| 18432 | ≤2048 | FLASH_NVM_CONFIG (UW_DOPPLER app config) |

**Rules:**
- New persistent data → carve a region OUTSIDE FLASH_USER (like PMLOG)
- Never reuse FLASH_APP_VARS, even if it looks unused — shared with other
  apps
- `MGR_NVM_reset()` ONLY wipes offset 18432 (NVM_Config). It does NOT touch
  credentials or radio config — safe to call on factory reset

---

## 5. Retention RAM (SRAM2) layout

```
0x20008000 – 0x2000FFF7  SRAM2 (32 KB - 8)
   .data2:
     .knsCtxtData            MAC stack context (Kineis closed-source)
     .retentionRamData       App-defined retained globals
   .bss2:
     .knsCtxtBss             MAC stack BSS
     .retentionRamBss        EVTLOG ring, rate limiter, TX stats

0x2000FFF8 – 0x2000FFFF  RAM_NOINIT (8 bytes)   DFU flag (survives reset)

0x4000B100               RTC TAMP BKP registers
   .lpmSection             LPM context (lpm_ctxt)
   .msgCntSectionData      Msg counter shadow
   (BKP2R-BKP7R)           MGR_ERR diagnostic regs
```

### Retention rules
- `boot_retained`, `sws_retained`, EVTLOG: in `.retentionRamData/Bss`
  (SRAM2) — survive NRST/IWDG/SW reset thanks to `SRAM_RST` option byte
- `.knsCtxtData/Bss`: re-initialized by `Sram2_Init()` on every default
  boot path (because the MAC stack requires fresh state and cannot share
  retention with our app data — linker groups them in the same `.data2`
  section)
- `lpm_ctxt` and message counter shadow: in TAMP backup → survive
  STANDBY/SHUTDOWN too; only wiped by full VBAT off

### Known trap: `SRAM_RST` option byte
- `Core/Src/main.c:439` `ensure_sram_preserved_on_reset()` writes the
  option byte ON (preserve SRAM on system reset)
- Consequence: `boot_retained.consecutive_failures` does not reset on NRST
  → can trap the board in PERMANENT_OFF SHUTDOWN once a real bug pushes
  the counter past 10
- **Mitigation**: `boot_loop_handle()` should clear the counter when reset
  cause is `BORRSTF` (cold power-on) or `PINRSTF` (user pressed reset).
  Currently force-cleared unconditionally as a safety patch — to be
  hardened.

---

## 6. UW_DOPPLER operational protocol

### Operating modes

| Mode | LED feedback | Behaviour |
|---|---|---|
| **POWER_OFF** | none (board off) | SHUTDOWN — only reed magnet wakes |
| **OPERATIONAL** | LED off except on transitions | Normal: SWS polling, TX when surface detected, MAC active |
| **CONFIG** | slow blink BLUE; solid BLUE when GUI connected | UART AT command access, GUI configuration |

### Magnet 2-gesture protocol (user-validated)

**Confirmation principle**: any mode change shows a fast blink (the
question) — user must OFF→ON the magnet within the 2-second window to
confirm. No toggle within 2 s → revert to previous mode.

```
POWER_OFF
  └─ Magnet ON
       └─ LED WHITE solid (boot indicator)
       └─ Then 5 × BLUE slow blink (boot ok)
       └─ → OPERATIONAL

OPERATIONAL
  ├─ Magnet held 3 s
  │    └─ Fast BLUE blink (2 s window: "go to CONFIG?")
  │         ├─ User OFF→ON within 2 s → Slow BLUE blink → CONFIG
  │         └─ No toggle → revert to OPERATIONAL
  │
  └─ Magnet held 6 s+ (TBD: also available from OPERATIONAL?)
       └─ Fast RED blink (2 s window: "SHUTDOWN?")
            ├─ User OFF→ON within 2 s → LED off → SHUTDOWN
            └─ No toggle → revert to OPERATIONAL

CONFIG
  ├─ Slow BLUE blink (idle CONFIG)
  ├─ Solid BLUE (GUI session active)
  ├─ Magnet held 3 s
  │    └─ Fast GREEN blink (2 s window: "back to OPERATIONAL?")
  │         ├─ User OFF→ON within 2 s → Slow GREEN blink → OPERATIONAL
  │         └─ No toggle → revert to CONFIG
  │
  └─ Magnet held 6 s+
       └─ Fast RED blink → confirm → SHUTDOWN
```

### Timing constants
- Confirmation window: 2000 ms
- Slow blink cadence: TBD (250 ms on / 250 ms off?)
- Fast blink cadence: TBD (100 ms on / 100 ms off?)
- POWER_OFF wake blink count: 5
- Magnet hold thresholds: 3 s (mode switch question), 6 s (shutdown
  question)

---

## 7. SWS algorithm (Saltwater Switch)

Reference: `.claude/linkit-uw-behavior.md`,
`.claude/sws_analog_implementation.md`, and the latest `arribada/linkit-v4-core`.

### Goal
Detect surface vs underwater fast (<2 s when surfacing) and robust to
biofouling over months of deployment.

### Implementation
- 5-level surface detection (L1: instant drop / L2: 2-step / L3: MA trend
  / L4: water baseline drop / L5: dive peak safety)
- Adaptive air/water baselines, biofouling compensation
- Anti-spike observed_peak filter
- Dive timeout escalation with surface lockout
- Continuous coherence check
- Adaptive sample-delay (RC settle window) per contrast

Already ported in `Kineis/App/Managers/MGR_SWS/Src/mgr_sws.c`.

### Known divergences from linkit-v4 main
| Constant | Ours | linkit-v4 | Action |
|---|---|---|---|
| `L4_DROP_PERCENT` | 8 % | 15 % | Align to 15 % (pending user OK) |
| `pulse_on_max_us` | 1000 µs | 10000 µs | Extend for biofouling (pending) |

### Still missing vs upstream
- Guided calibration FSM (CalibPhase 8 states + LED feedback)
- Test mode API
- Diagnostics struct (CRC-protected counters)
- Status / heartbeat notify callbacks
- Operator hints `CAL_OFFSET_HINT_AIR/WATER`

---

## 8. Power consumption budget

Battery is the primary constraint — turtle deployments must last months.

### Targets (proposed, pending validation)
| State | Target | Notes |
|---|---|---|
| SHUTDOWN (POWER_OFF, reed wait) | < 5 µA | Only reed circuit + BOR alive |
| OPERATIONAL idle (between SWS polls) | < 50 µA avg | STOP2 + RTC wake, SWS off |
| SWS poll burst | ~1 mA × few ms | PA12 ON + ADC + PA12 OFF |
| MAC TX | ~60 mA × seconds | Not in scope for now |

### Rules
- CPU idle MUST be in STOP (when MAC permits) — not NONE.
- LPM management is owned by MGR_LPM aggregator + KSTK_lpmReq (Kineis).
  Do not override with `LPM_setForcedMode` unless investigating.
- Any peripheral added must have a DeInit path called before
  STOP/STANDBY/SHUTDOWN (current bug: ADC not deinit before
  STANDBY/SHUTDOWN — pending fix).
- LED, UART debug, heartbeat: DEBUG-only (`#if defined(DEBUG)`). Production
  builds (no DEBUG flag) must have zero spurious wake-ups.

---

## 9. Safety / recovery

### Reset-cause guards already present
- `boot_loop_handle()`: SRAM2 counter, factory reset at 5, perma-off at 10
- `MGR_ERR_checkCrashLoop()`: TAMP counter, 1 h safe sleep at 10
- IWDG: 16 s timeout
- HardFault handler: dumps registers + NVIC reset
- PA watchdog: detects stuck PA → log + reset

### Current temporary safety patches (to remove once root cause fixed)
| Patch | File | Reason |
|---|---|---|
| `boot_loop_handle` force-clear | `kns_app_uw_doppler.c:192` | SRAM_RST traps the counter |
| `MGR_ERR_checkCrashLoop` force-clear | `mgr_err.c:175` | 1 h sleep would hide root cause |
| `MGR_WDG_init()` commented out | `kns_app_uw_doppler.c` | MAC starves loop > 16 s |

These are in the working tree, NOT committed. To be removed once Phase 1
ADC fix is shipped and the MAC starvation cause is known.

### Recovery procedure for a bricked board
1. NRST manual: should always work — if not, problem is at HW level
2. Power-cycle (battery removal 30 s) — clears SRAM2 but not TAMP
3. JLink + unlock + erase + reflash:
   ```
   JLinkExe -device STM32WL55JC -if SWD -speed 4000 -autoconnect 1
   > halt
   > unlock STM32WL5x
   > erase
   > loadbin build/argos-smd-at-kineis-firmware.bin 0x08000000
   > reset
   > go
   ```
4. If JLink can't connect: try "connect under reset" (`r0`) or use the
   STM32 ROM bootloader via BOOT0 pin

---

## 10. Linkit-v4 reference

We follow `arribada/linkit-v4-core` for behaviour patterns (SWS,
magnet/reed gestures, UW detector). Our code is C; linkit is C++. Port
behaviour, not code structure verbatim.

When a divergence is found between our firmware and linkit-v4-core upstream:
1. Document the divergence in MASTER_AUDIT
2. Ask user before aligning (linkit might have changed for reasons not
   applicable to our HW)

---

## 11. Open questions (for user, pending validation)

1. `L4_DROP_PERCENT` alignment: 8 % → 15 % ? (linkit-v4 main says 15)
2. Magnet 6s+ SHUTDOWN: also available from OPERATIONAL, or only from
   CONFIG?
3. Slow / fast blink exact cadences (ms)?
4. `MGR_GESTURE` new module on top of `MGR_REED`, or integrate inside
   `MGR_REED`?
5. Test framework: keep custom, or migrate to Unity for HTML reports built-in?
6. Real-device test suite: priorities (which scenarios to automate first)?
