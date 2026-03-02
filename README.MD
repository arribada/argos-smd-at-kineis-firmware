![N|Solid](https://arribada.org/wp-content/uploads/2022/01/arribada_web_logo_g.svg)
# Argos SMD AT Kineis Firmware

This repository contains the STM32WL firmware for the Argos SMD module using the Kineis stack. It provides three application modes for STM32WL55xx-based hardware reference designs (KRD_MP):

- **GUI**: AT command interface (UART or SPI) compatible with KimGUI
- **Standalone**: Single periodic SAT message transmission at startup
- **UW_DOPPLER**: Autonomous underwater/surface tracker with satellite TX scheduling

The project is based on `krd_fw_package_v8.5.1` from CLS and has been adapted for the [argos-smd](https://github.com/arribada/argos-smd-hw) module.

To start, look at the [Arduino sample](https://github.com/arribada/argos-smd-test-arduino) code used for certification or the [Zephyr UART driver](https://github.com/arribada/argos-smd-driver-zephyr) tested with Feather nRF52840.

> **Warning**: Since version v8.5.1, the MAC profile is not loaded at startup — reload it thanks to KMAC commands (`AT+KMAC=0` and `AT+KMAC=1`).

## Features

- **GUI mode** (`APP=GUI`): AT command interface compatible with KimGUI (UART or SPI)
- **Standalone mode** (`APP=STDLN`): Single periodic SAT message transmission at startup
- **UW_DOPPLER tracker mode** (`APP=UW_DOPPLER`): Autonomous underwater/surface detection with satellite TX scheduling, battery monitoring, NVM persistence, event logging, and reed switch shutdown
- Configurable MAC profiles: BASIC (single TX) or BLIND (repeated TX with Doppler)
- Low Power Mode management: NONE, SLEEP, STOP, STANDBY, SHUTDOWN
- DFU bootloader (over UART or SPI) for field firmware updates
- Multiple board variants: SMD_PA, SMD_NOPA, SMD_STDALONE, SMD_OP

## Application Modes

### GUI Mode (`APP=GUI`) — Default

AT command interface for host-controlled operation. The module responds to AT commands over UART (9600 baud) or SPI (115200 baud) and is compatible with the KimGUI tool from CLS.

### Standalone Mode (`APP=STDLN`)

Minimal mode that transmits a single satellite message at startup. Useful for quick tests and certification.

### UW_DOPPLER Tracker Mode (`APP=UW_DOPPLER`)

Autonomous underwater/surface tracker designed for wildlife tracking. Requires `BOARD=SMD_STDALONE`.

#### Principle of Operation

1. **Power on**: Reed switch magnet powers the board via PWR_LATCH (PB7). White LED blinks 10x to confirm boot.
2. **Surface/water detection**: Salt Water Switch (SWS) on PA11/PA12 continuously measures conductivity via ADC. Configurable thresholds determine surface vs. underwater state.
3. **TX scheduling**: When the device surfaces, satellite messages are transmitted using the configured MAC profile. TX intervals grow exponentially (configurable initial interval, growth percent, max interval, max count).
4. **Dive tracking**: The device tracks dive duration and only triggers TX after a configurable minimum surface time.
5. **Battery monitoring**: Internal ADC reads VBAT/3 channel with VREFINT calibration (3300mV factory reference). Battery voltage (mV) is included in the TX payload.
6. **NVM persistence**: TX configuration, SWS thresholds, deploy mode, and LED mode are saved to flash via `AT+SAVE` and survive power cycles.
7. **Event logging**: A circular buffer of 256 events in SRAM2 retention RAM survives all resets except power-off. View with `AT+LOG=?`.
8. **Shutdown**: Hold the reed switch magnet for >10 seconds. White LED blinks 10x. NVM save. PWR_LATCH pulled LOW. Board powers off completely.
9. **Low power**: Between SWS measurements, the MCU enters STOP mode (SysTick suspended, RTC tick compensation on wake).

#### State Machine

```
BOOT ──> BOOT_DEPLOY_LED ──> INIT_MAC ──> WAIT_MAC_READY ──> MONITORING
                                                                  │
                                    ┌─────────────────────────────┤
                                    │                             │
                               SURFACE_TX ──> WAIT_TX_DONE ──────┘
                                    │
                           SHUTDOWN_BLINK (reed hold >10s)
```

| State | Description | Timeout |
|-------|-------------|---------|
| `BOOT` | White LED blink 10x to confirm power-on | 10s |
| `BOOT_DEPLOY_LED` | Deploy mode LED feedback (green=deployed, red=not) | 5s |
| `INIT_MAC` | Initialize Kineis MAC profile (BLIND or BASIC) | — |
| `WAIT_MAC_READY` | Wait for MAC profile init confirmation | 30s |
| `MONITORING` | SWS measurements, reed switch polling, LPM STOP between cycles | — |
| `SURFACE_TX` | Build TX payload and submit to MAC | — |
| `WAIT_TX_DONE` | Wait for MAC TX_DONE or TX_TIMEOUT event | 60s |
| `SHUTDOWN_BLINK` | White LED blink 10x, then NVM save + power off | 10s |

All waiting states have hardware watchdog backup (IWDG, 16s timeout). If a state timeout fires, an `EVT_TIMEOUT` is logged, error is recorded to TAMP registers, and the system resets.

#### TX Scheduling Algorithm

TX intervals follow an exponential growth pattern:

```
T(n) = T_initial * (1 + growth/100)^n
```

Where:
- `T_initial` = initial interval (default 10s, configurable via `AT+TXCFG`)
- `growth` = growth percentage per TX (default 10%)
- The interval is capped at `T_max` (default 600s)
- TX count is limited to `max_count` (default 0 = unlimited)

The first TX after surface detection is **immediate**. Subsequent TXs use the growing interval. TX scheduling resets when the device goes back underwater.

Both MAC profiles are supported:
- **BLIND** (`MAC_PRFL=BLIND`): retx_nb=0, retx_period_s=60, nb_parallel_msg=1. The application handles all TX scheduling; the MAC profile sends each message once.
- **BASIC** (`MAC_PRFL=BASIC`): immediate single TX per request. Same application-level scheduling.

#### Salt Water Switch (SWS) Detection Algorithm

The SWS uses an adaptive biofouling-resistant detection algorithm:

1. **ADC measurement**: PA12 drives the sensor, PA11 reads conductivity via ADC_IN7 (12-bit, 0-4095). Higher values = more conductive = underwater.
2. **Moving average filter**: 2-sample history smooths noise.
3. **Threshold detection**: Dynamic threshold computed from adaptive air/water baselines using a configurable ratio (default 35%). Hysteresis band (14%) prevents oscillation.
4. **Biofouling compensation**: Three-tier rapid transition detection catches surface events even when biofouling shifts the baseline upward:
   - **Tier 1**: Large instant drop (>25% and >75 ADC units)
   - **Tier 2**: Medium drop + trend + below water baseline (>12% and >38 ADC units)
   - **Tier 3**: Small drop + strong trend (>8% and >25 ADC units, 2+ consecutive decreases)
5. **Trend + variance analysis**: Tracks consecutive decreasing readings and variance in a sliding window (8 samples). High variance combined with cumulative drop triggers surface override.
6. **EMA water baseline**: Exponential moving average (alpha=19%) adapts the water baseline during dives.
7. **Extended dive recalibration**: After 60s underwater, recalibrates water baseline every 30s.
8. **Surface adaptation**: After stable surface time (>10s), recalibrates air baseline from recent readings.
9. **Max dive timeout**: Forced surface state after configurable duration (default 7200s).
10. **Min surface lockout**: Prevents re-submersion detection for configurable duration (default 10s).

#### TX Payload Format

9 bytes, sent as user data in each satellite message:

| Byte | Content | Description |
|------|---------|-------------|
| 0 | State | 0=init, 1=surface, 2=underwater |
| 1-2 | SWS ADC | Raw 12-bit ADC value (big-endian) |
| 3 | TX count | Number of TXs in current surface event |
| 4-5 | Battery | Voltage in mV (big-endian) |
| 6-8 | Reserved | Zeros (available for future use) |

#### Robustness Features

| Feature | Implementation | Persistence |
|---------|---------------|-------------|
| **Watchdog** | IWDG 16s timeout, refreshed every loop iteration | Hardware (cannot be stopped) |
| **State timeouts** | All waiting states have configurable timeouts | — |
| **Error tracker** | TAMP backup registers BKP2R-BKP7R | Survives all resets (not VBAT removal) |
| **Event log** | 256-entry circular buffer in SRAM2 (2KB) | Survives all resets (not power-off) |
| **NVM config** | CRC32-protected flash page (last 2KB page) | Survives power cycles |
| **Fault handlers** | HardFault/BusFault log error to TAMP, then reset | TAMP registers |
| **LPM tick compensation** | RTC-based tick correction after STOP mode wake | — |

#### Typical Build Command

```bash
make full APP=UW_DOPPLER BOARD=SMD_STDALONE LPM=STOP MAC_PRFL=BLIND COMM=UART DEBUG=1 -j20
```

## Board Variants

| Feature | SMD_PA | SMD_NOPA | SMD_STDALONE | SMD_OP |
|---------|--------|----------|--------------|--------|
| External PA | Yes | No | Yes | Yes |
| RGB LED (R/G/B) | — | — | PA1/PB4/PB5 | — |
| Reed switch | — | — | PB6 | — |
| Power latch | — | — | PB7 | — |
| VBAT monitoring | — | — | PB9 (enable) + internal ADC | — |
| Salt Water Switch | — | — | PA11 (ADC) / PA12 (drive) | — |
| Use case | Standard AT module | No-PA AT module | Autonomous tracker | Operational (same as PA) |

### SMD_PA (`BOARD=SMD_PA`) — Default
Standard board with external power amplifier. PA control via VPA_EN (PC0) and VSEL (PC1). Suitable for GUI and Standalone modes.

### SMD_NOPA (`BOARD=SMD_NOPA`)
Same as SMD_PA but without external PA. Uses the STM32WL internal RF. Lower TX power, lower cost.

### SMD_STDALONE (`BOARD=SMD_STDALONE`)
Full-featured board for autonomous tracker operation. Includes:
- **RGB LED** (active LOW): Red (PA1), Green (PB4), Blue (PB5)
- **Reed switch** (PB6, active HIGH): Magnetic on/off control with debounce
- **PWR_LATCH** (PB7): External pull-up keeps board powered; pull LOW to shutdown
- **VBAT_EN** (PB9): Enables battery voltage measurement circuit
- **SWS** (PA11/PA12): Salt Water Switch for underwater detection
- **PA** (PC0/PC1): External power amplifier control

Required for `APP=UW_DOPPLER`. The MGR_LED and MGR_REED drivers are automatically included when this board is selected.

### SMD_OP (`BOARD=SMD_OP`)
Operational variant. Currently identical to SMD_PA (placeholder for future differentiation).

## Getting Started

### Configuration

#### Credentials Management

> **Warning**: This firmware requires proper configuration of **credentials** and **radio settings**. Misconfigured settings may lead to improper operation or device malfunction.

- Contact CLS group (user office) to obtain the required credentials (ID, address, secret key) for the stack.
- Securely store the credentials. The provided package stores them in RAM/FLASH without protection.
- Update `device_id` and `address` in the `mcu_nvm.c` file.

Note: Currently, the ID must be changed at build time. A command to modify it will be added soon.

#### Radio Configuration Management

- Obtain radio configuration binary files from CLS group (user office) or use one defined in `mcu_nvm.c`.
- Store configuration data securely. In the provided package, the configuration is hardcoded as a byte table in `mcu_nvm.c`.
- Alternatively:
  - Use the `AT+RCONF=` command.
  - Modify the configuration directly by commenting/uncommenting the corresponding settings in `mcu_nvm.c`.

### MAC Profile Configuration Management

- Choose between **BASIC** and **BLIND** MAC profiles by setting `MAC_PRFL=` in the build command.
- For UW_DOPPLER, either profile works. The application handles TX scheduling directly; the MAC profile is only responsible for the single-message transmission.
- BLIND config: `retx_nb=0`, `retx_period_s=60`, `nb_parallel_msg=1`.

## Building the Project

### From STM32CubeIDE

1. Install STM32CubeIDE.
2. Open the project from the Kineis package archive.
3. Clean the project before rebuilding.
4. Build the project to generate the binary file.

### From Terminal

1. Ensure GNU Make and the ARM toolchain (`arm-none-eabi-gcc`) are available.
2. Clean the current build before rebuilding: `make clean`
3. Run `make` from the root directory.

### Build Variables

| Variable | Values | Default | Description |
|----------|--------|---------|-------------|
| `APP` | `GUI`, `STDLN`, `UW_DOPPLER` | `GUI` | Application mode |
| `BOARD` | `SMD_PA`, `SMD_NOPA`, `SMD_STDALONE`, `SMD_OP` | `SMD_PA` | Hardware board variant |
| `COMM` | `UART`, `SPI` | `UART` | Communication interface |
| `MAC_PRFL` | `BASIC`, `BLIND` | `BASIC` | Kineis MAC profile |
| `LPM` | `NONE`, `SLEEP`, `STOP`, `STANDBY`, `SHUTDOWN` | `NONE` | Deepest low power mode allowed |
| `DEBUG` | `0`, `1` | `1` | Enable debug symbols and -Og optimization |
| `VERBOSE` | `0`, `1` | `1` | Enable verbose debug log output |
| `SMPS_BYPASS_TX` | `0`, `1` | `0` | Use LDO mode (bypass SMPS) during TX for TCXO stability |

### Build Options

#### Application Only (Default)
```bash
# GUI mode — Debug UART
make DEBUG=1 VERBOSE=1 -j20

# GUI mode — Release UART
make DEBUG=0 VERBOSE=0 -j20

# GUI mode — Debug SPI
make DEBUG=1 VERBOSE=1 COMM=SPI -j20

# GUI mode — Release SPI
make DEBUG=0 VERBOSE=0 COMM=SPI -j20

# UW_DOPPLER tracker — Debug UART
make APP=UW_DOPPLER BOARD=SMD_STDALONE MAC_PRFL=BLIND LPM=STOP DEBUG=1 VERBOSE=1 -j20
```

#### Bootloader Only
```bash
# Build bootloader for SPI mode (default)
make bootloader COMM=SPI

# Build bootloader for UART mode
make bootloader COMM=UART
```

#### Combined Firmware (App + Bootloader)
Build both application and bootloader, then merge them into a single flashable image:
```bash
# GUI — Debug SPI
make full COMM=SPI DEBUG=1 VERBOSE=1 -j20

# GUI — Release SPI
make full COMM=SPI DEBUG=0 VERBOSE=0 -j20

# UW_DOPPLER tracker — Release UART
make full APP=UW_DOPPLER BOARD=SMD_STDALONE MAC_PRFL=BLIND LPM=STOP COMM=UART DEBUG=0 VERBOSE=0 -j20
```

Output files:
- `build/argos-smd-at-kineis-firmware_full.bin` - Binary for flashing
- `build/argos-smd-at-kineis-firmware_full.hex` - Intel HEX format

#### DFU Binary (For OTA Updates)
Generate a binary file for Device Firmware Update via the bootloader:
```bash
make dfu COMM=SPI -j20
```

Output: `build/argos-smd-at-kineis-firmware_dfu.bin`

### Flashing

#### Using Make (JLink)
```bash
# Flash application only
make flash-app

# Flash bootloader only
make flash-bl

# Flash combined firmware (app + bootloader)
make flash-full
```

#### Using VS Code Tasks
- **Build Full (App + Bootloader)** - Compile everything
- **Flash Full (App + Bootloader)** - Build and flash combined image
- **Generate DFU Binary** - Create DFU update file

### Memory Layout

| Region | Address Range | Size | Description |
|--------|---------------|------|-------------|
| Application | `0x08000000` - `0x08032FFF` | 204 KB | Main firmware |
| Bootloader | `0x08033000` - `0x0803AFFF` | 32 KB | DFU bootloader |
| Flash User Page 0 | `0x0803B000` - `0x0803B7FF` | 2 KB | Kineis config (ID, ADDR, key, radio, counters overflow) |
| Flash User Pages 1-4 | `0x0803B800` - `0x0803D7FF` | 8 KB | MSG counter wear-leveling |
| Flash User Pages 5-8 | `0x0803D800` - `0x0803F7FF` | 8 KB | WKU counter wear-leveling |
| NVM Config | `0x0803F800` - `0x0803FFFF` | 2 KB | App config persistence (UW_DOPPLER) |

#### RAM Layout (UW_DOPPLER)

| Section | Memory | Size | Description |
|---------|--------|------|-------------|
| `.retentionRamData` | SRAM2 | Variable | State machine state, LPM context (survives STANDBY) |
| `.retentionRamBss` | SRAM2 | ~2 KB | Event log buffer (survives resets, not power-off) |
| TAMP BKP2R-BKP7R | Backup domain | 24 bytes | Error tracker (survives all resets, not VBAT removal) |

### Clean

```bash
# Clean application only
make clean

# Clean bootloader only
make bootloader-clean

# Clean everything
make clean-all
```

### Unit Tests

Unit tests are in the `Tests/` directory and run on the host (x86) using GCC:

```bash
cd Tests
make        # Build all tests
make run    # Run all tests
```

Test suites:
- `test_crc32` — CRC32 (MPEG-2) implementation correctness
- `test_dfu_protocol` — DFU AT command parsing, address validation, SPI commands
- `test_app_header` — Application header structure validation
- `test_spi_protocol` — SPI protocol framing, timing, state machine
- `test_evtlog` — Event log circular buffer, wrap-around, warm boot preservation
- `test_nvm` — NVM save/load/reset, CRC integrity, struct alignment
- `test_tx_interval` — TX interval exponential growth computation
- `test_memory_safety` — Memory bounds, canary checks, fault injection

## AT Commands

### General Commands (All Modes)

| Command | Mode | Description |
|---------|------|-------------|
| `AT+PING` | Action | Ping (returns `+OK`) |
| `AT+VERSION=?` | Status | AT command protocol version |
| `AT+FW=?` | Status | Firmware build string (app mode, MAC profile, board, comm) |
| `AT+ADDR=?` | Status | Device address (hex) |
| `AT+ADDR=<hex>` | Action | Set device address |
| `AT+ID=?` | Status | Device ID (hex) |
| `AT+ID=<hex>` | Action | Set device ID |
| `AT+SN=?` | Status | Device serial number |
| `AT+SECKEY=?` | Status | Secret key (hex) |
| `AT+SECKEY=<hex>` | Action | Set secret key |
| `AT+RCONF=?` | Status | Radio configuration |
| `AT+RCONF=<hex>` | Action | Set radio configuration |
| `AT+RCONFRAW=?` | Status | Raw radio config (16 bytes hex) |
| `AT+SAVE_RCONF` | Action | Save radio config to flash |
| `AT+LPM=?` | Status | Current low power mode |
| `AT+LPM=<mode>` | Action | Set low power mode |
| `AT+MC=?` | Status | Message counter for next TX |
| `AT+TCXO_WU=?` | Status | TCXO warm-up time (ms) |
| `AT+TCXO_WU=<ms>` | Action | Set TCXO warm-up time |
| `AT+TX=<hex>` | Action | Transmit user data |
| `AT+CW=<params>` | Action | Continuous wave / modulated wave test |
| `AT+UDATE=<epoch>` | Action | Set UTC date/time |
| `AT+KMAC=?` | Status | Current MAC profile |
| `AT+KMAC=<id>` | Action | Set MAC profile |
| `AT+PREPASS_EN=?` | Status | PREVIPASS enable state |
| `AT+BOOT` | Action | Enter DFU bootloader mode |

### UW_DOPPLER Commands (`APP=UW_DOPPLER` only)

These commands are available in addition to the general commands when the firmware is built with `APP=UW_DOPPLER`.

#### AT+SWS — Salt Water Switch Status / Enable

```
AT+SWS=?
+SWS=<state>,<adc>,<air_baseline>,<water_baseline>,<threshold_min>,<enabled>
+OK

AT+SWS=<0|1>
+OK
```

| Field | Type | Description |
|-------|------|-------------|
| `state` | uint | 0=unknown, 1=surface, 2=underwater |
| `adc` | uint | Last raw ADC reading (0-4095) |
| `air_baseline` | uint | Current air baseline |
| `water_baseline` | uint | Current water baseline |
| `threshold_min` | uint | Minimum detection threshold |
| `enabled` | uint | 1=enabled, 0=disabled |

#### AT+SWSCFG — SWS Configuration

```
AT+SWSCFG=?
+SWSCFG=<thr_min>,<thr_max>,<air_init>,<water_init>,<interval_ms>,<max_dive_s>,<min_surf_s>
+OK

AT+SWSCFG=<thr_min>,<thr_max>,<air_init>,<water_init>,<interval_ms>,<max_dive_s>,<min_surf_s>
+OK
```

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `thr_min` | uint16 | 0-4095 | 13 | Min valid ADC value (noise floor) |
| `thr_max` | uint16 | 0-4095 | 2000 | Max valid ADC value (saturation) |
| `air_init` | uint16 | 0-4095 | 50 | Initial air baseline |
| `water_init` | uint16 | 0-4095 | 750 | Initial water baseline |
| `interval_ms` | uint32 | >0 | 1000 | Measurement interval (ms) |
| `max_dive_s` | uint32 | — | 7200 | Max dive time before forced surface (s) |
| `min_surf_s` | uint32 | — | 10 | Min surface time before re-submersion (s) |

Validation: `thr_min < thr_max`, `water_init > air_init`, `interval_ms > 0`.

#### AT+SWSFORCE — Force SWS Measurement

```
AT+SWSFORCE
+SWS=<state>,<adc>
+OK
```

Triggers an immediate SWS measurement regardless of the interval timer. Returns the current state and raw ADC value.

#### AT+TXCFG — TX Scheduling Configuration

```
AT+TXCFG=?
+TXCFG=<interval_s>,<growth_pct>,<max_interval_s>,<max_count>
+OK

AT+TXCFG=<interval_s>,<growth_pct>,<max_interval_s>,<max_count>
+OK
```

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `interval_s` | uint16 | 1-65535 | 10 | Initial TX interval (seconds) |
| `growth_pct` | uint8 | 0-255 | 10 | Interval growth percentage per TX |
| `max_interval_s` | uint16 | 1-65535 | 600 | Maximum TX interval cap (seconds) |
| `max_count` | uint8 | 0-255 | 0 | Max TX count per surface (0=unlimited) |

#### AT+DEPLOY — Deploy Mode

```
AT+DEPLOY=?
+DEPLOY=<mode>
+OK

AT+DEPLOY=<0|1>
+OK
```

- `0` = Not deployed: SWS runs but no satellite TX.
- `1` = Deployed: Full operation with satellite TX on surface events.

#### AT+LED — LED Mode

```
AT+LED=?
+LED=<mode>
+OK

AT+LED=<0|1|2>
+OK
```

| Mode | Description |
|------|-------------|
| 0 | LED always off |
| 1 | LED enabled (state-dependent colors) |
| 2 | LED enabled for 24h then auto-off |

Only available on boards with `BSP_HAS_LED_RGB` (SMD_STDALONE). Returns `+LED=N/A` on other boards.

#### AT+LOG — Event Log

```
AT+LOG=?
+LOG=<count>
#000 t=00012345 e=00 s=04 d=0000
#001 t=00012400 e=05 s=04 d=0512
...
+OK

AT+LOG
+OK
```

**Status mode** (`AT+LOG=?`): Dumps all events (oldest first). Each line:
- `#NNN` — entry index
- `t=` — HAL tick (ms since boot)
- `e=` — event type (see table below)
- `s=` — UW_DOPPLER state at time of event
- `d=` — context data

**Action mode** (`AT+LOG`): Clears the event log.

**Event types:**

| Code | Name | Data Field |
|------|------|------------|
| 00 | `EVT_BOOT` | Reset cause (RCC_CSR bits) |
| 01 | `EVT_STATE_CHANGE` | New state |
| 02 | `EVT_REED_ON` | 0 |
| 03 | `EVT_REED_OFF` | Hold duration / 100 (centiseconds) |
| 04 | `EVT_SWS_SURFACE` | ADC value |
| 05 | `EVT_SWS_UNDERWATER` | ADC value |
| 06 | `EVT_TX_START` | TX count |
| 07 | `EVT_TX_DONE` | 0 |
| 08 | `EVT_TX_TIMEOUT` | 0 |
| 09 | `EVT_MAC_READY` | 0 |
| 10 | `EVT_MAC_ERROR` | Error code |
| 11 | `EVT_BAT` | Voltage in mV |
| 12 | `EVT_SHUTDOWN` | 0 |
| 13 | `EVT_TIMEOUT` | Timed-out state |
| 14 | `EVT_ERROR` | Error code |

#### AT+SAVE — Save Configuration to Flash

```
AT+SAVE
+OK
```

Saves all current configuration (TX scheduling, SWS thresholds, deploy mode, LED mode) to flash with CRC32 integrity protection. Configuration is automatically loaded on next boot.

> **Note**: Configuration is also auto-saved during shutdown (reed switch hold). Use `AT+SAVE` after changing settings via `AT+TXCFG`, `AT+SWSCFG`, `AT+DEPLOY`, or `AT+LED` to persist changes immediately.

## Architecture

### Module Overview (UW_DOPPLER)

```
┌──────────────────────────────────────────────────┐
│                  kns_app_uw_doppler.c             │
│             (State machine + TX scheduling)       │
├──────────────────────────────────────────────────┤
│  MGR_SWS   │  MGR_BAT  │  MGR_REED │  MGR_LED   │
│  (Water    │  (Battery │  (Reed    │  (RGB LED  │
│  detection)│  voltage) │  switch)  │  feedback) │
├────────────┼───────────┼───────────┼────────────┤
│  MGR_NVM   │  MGR_ERR  │  MGR_WDG  │ MGR_EVTLOG │
│  (Flash    │  (TAMP    │  (IWDG    │ (SRAM2     │
│  config)   │  errors)  │  16s WDG) │ event log) │
├──────────────────────────────────────────────────┤
│  MGR_AT_CMD (AT command interface over LPUART1)  │
├──────────────────────────────────────────────────┤
│  Kineis Stack (KNS_MAC + KNS_Q + KNS_CFG)       │
├──────────────────────────────────────────────────┤
│  LPM (Low Power Manager: SLEEP / STOP / STANDBY)│
├──────────────────────────────────────────────────┤
│  STM32WL55 HAL + CMSIS                          │
└──────────────────────────────────────────────────┘
```

### Source Tree (UW_DOPPLER-specific)

```
Kineis/App/
├── kns_app_uw_doppler.c          # Main state machine and TX scheduling
├── kns_app_uw_doppler.h          # Public API
└── Managers/
    ├── MGR_AT_CMD/
    │   ├── Inc/mgr_at_cmd_list_uw_doppler.h   # UW_DOPPLER AT command declarations
    │   └── Src/mgr_at_cmd_list_uw_doppler.c   # AT command handlers (SWS, TXCFG, LED, etc.)
    ├── MGR_SWS/
    │   ├── Inc/mgr_sws.h          # SWS API and config struct
    │   └── Src/mgr_sws.c          # Biofouling-adaptive detection algorithm
    ├── MGR_BAT/
    │   ├── Inc/mgr_bat.h          # Battery monitoring API
    │   └── Src/mgr_bat.c          # VBAT/3 + VREFINT ADC measurement
    ├── MGR_NVM/
    │   ├── Inc/mgr_nvm.h          # NVM config struct and API
    │   └── Src/mgr_nvm.c          # Flash read/write with CRC32
    ├── MGR_ERR/
    │   ├── Inc/mgr_err.h          # Error codes and TAMP register layout
    │   └── Src/mgr_err.c          # Reset tracking and error persistence
    ├── MGR_WDG/
    │   ├── Inc/mgr_wdg.h          # Watchdog API
    │   └── Src/mgr_wdg.c          # IWDG direct register configuration
    ├── MGR_EVTLOG/
    │   ├── Inc/mgr_evtlog.h       # Event types and log API
    │   └── Src/mgr_evtlog.c       # SRAM2 circular buffer implementation
    ├── MGR_LED/
    │   ├── Inc/mgr_led.h          # RGB LED API (colors, modes, blink)
    │   └── Src/mgr_led.c          # GPIO driver (active LOW, non-blocking blink)
    └── MGR_REED/
        ├── Inc/mgr_reed.h         # Reed switch API (events, hold duration)
        └── Src/mgr_reed.c         # EXTI interrupt + power latch control
```

### NVM Configuration Structure

The NVM stores all user-configurable parameters in a single flash page with CRC32 protection:

```c
typedef struct {
    uint32_t magic;                     // 0x434F4E46 ("CONF")
    uint8_t  version;                   // Schema version (currently 1)
    uint8_t  deploy_mode;               // 0=not deployed, 1=deployed
    uint8_t  led_mode;                  // MGR_LED_Mode_t
    // TX scheduling
    uint16_t tx_initial_interval_s;     // Default: 10
    uint8_t  tx_growth_percent;         // Default: 10
    uint8_t  tx_max_count;              // Default: 0 (unlimited)
    uint16_t tx_max_interval_s;         // Default: 600
    // SWS detection
    uint16_t sws_threshold_min;         // Default: 13
    uint16_t sws_threshold_max;         // Default: 2000
    uint16_t sws_initial_air_baseline;  // Default: 50
    uint16_t sws_initial_water_baseline;// Default: 750
    uint32_t sws_test_interval_ms;      // Default: 1000
    uint32_t sws_max_dive_time_s;       // Default: 7200
    uint32_t sws_min_surface_time_s;    // Default: 10
    uint8_t  sws_enabled;               // Default: true
    uint32_t crc32;                     // CRC-32/MPEG-2 of all bytes before this field
} NVM_Config_t;
```

### Initialization Order

```
main() →
  HAL_Init()
  SystemClock_Config()
  enable_backup_access()     // Enable TAMP registers
  MX_GPIO_Init()
  MX_LPUART1_UART_Init()    // 9600 baud
  MX_RTC_Init()
  MX_ADC_Init()
  KNS_OS_init()
  LPM_init()
  MGR_ERR_init()             // Log previous reset cause
  MGR_REED_init()            // Reed switch + power latch
  MGR_LED_init()             // RGB LED GPIOs
  KNS_APP_uw_doppler_init()  // NVM load + EVTLOG init + MAC config
  MGR_SWS_init()             // SWS GPIO + ADC + initial calibration (uses NVM config)
  KNS_OS_main()              // Start scheduler
```

## Documentation

### Doxygen

Auto-generated Doxygen documentation can be built with:

```bash
make doc
```

Output is in `Kineis/Doc/krd_fw/html/index.html`.

All public APIs have Doxygen `@file`, `@brief`, `@param`, and `@return` annotations in their header files.

## License

This project is licensed under the GNU General Public License v3. See the `LICENSE` file for details.

## Todo / In Progress

- ~~SPI driver instead of UART~~ Done
- ~~Programming firmware by SPI~~ Done (DFU bootloader)
- ~~UW_DOPPLER autonomous tracker~~ Done (SWS, TX scheduling, NVM, battery, reed shutdown, LPM)
- ~~Event logging~~ Done (SRAM2 retention, AT+LOG)
- ~~Watchdog + error tracking~~ Done (IWDG 16s, TAMP backup registers)
- ~~Unit tests~~ Done (179 tests: CRC32, DFU, SPI, NVM, EVTLOG, TX interval, memory safety)
- Add and manage CLS ID at runtime
