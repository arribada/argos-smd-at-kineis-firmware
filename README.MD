![N|Solid](https://arribada.org/wp-content/uploads/2022/01/arribada_web_logo_g.svg)
# Argos SMD AT Kineis Firmware

This repository contains the STM32WL firmware for the Argos SMD module using the Kineis stack. It provides two demo application examples (standalone or AT commands) for STM32WL55xx-based hardware reference designs (KRD_MP). 

The project is based on `krd_fw_package_v8.5.1` from CLS and has been adapted for the [argos-smd](https://github.com/arribada/argos-smd-hw) module.

In GUI mode, the module responds to AT commands and interacts with the KimGUI from CLS.

To start, look at the [Arduino sample](https://github.com/arribada/argos-smd-test-arduino) code used for certification or the [Zephyr UART driver](https://github.com/arribada/argos-smd-driver-zephyr) tested with Feather nRF52840.

⚠️ Warning : Since version v8.5.1, the MAC profile is not loaded at startup reload it thanks to KMAC commands (AT+KMAC=0 and AT+KMAC=1).

⚠️ Warning : on v1.3 the Message Counter (MC) is stored in RAM retention. Powerdown the module will reset the message counter. If you send message always with the same message counter, CLS dashboard will not process all the messages as new message but as a repeated one. Fixed on version 2.
## Features

- Compiles standalone or GUI applications by default
- Supports periodic SAT message transmission (Standalone)
- Configurable MAC profiles and radio settings
- Low Power Mode (LPM) management

## Getting Started

### Configuration

#### Credentials Management

⚠️ Warning
This firmware requires proper configuration of **credentials** and **radio settings**.  
Misconfigured settings may lead to improper operation or device malfunction.  


- Contact CLS group (user office) to obtain the required credentials (ID, address, secret key) for the stack.
- Securely store the credentials. The provided package stores them in RAM/FLASH without protection.
- Update `device_id` and `address` in the `mcu_nvm.c` file.

Note: Currently, the ID must be changed at build time. A command to modify it will be added soon.


#### Radio Configuration Management

- Obtain radio configuration binary files from CLS group (user office) or used one defined into mcu_nvm.c file.
- Store configuration data securely. In the provided package, the configuration is hardcoded as a byte table in `mcu_nvm.c`.
- Alternatively:
  - Use the `AT+RCONF=` command.
  - Modify the configuration directly by commenting/uncommenting the corresponding settings in the `mcu_nvm.c` file.

### MAC Profile Configuration Management

- Choose between **BASIC** and **BLIND** MAC profiles by setting compilation flags in makefile .
- Configure parameters such as the repetition period and the number of repetitions for the BLIND profile.

## Building the Project

### From STM32CubeIDE

1. Install STM32CubeIDE.
2. Open the project from the Kinéis package archive.
3. Clean the project before rebuilding.
4. Build the project to generate the binary file.

### From Terminal

1. Ensure the GNU Make and ARM toolchain are available.
2. Clean the current build before rebuilding: `make Clean`
3. Run `make`from the root directory


### Compilation Flags

- Adjust log levels, LPM levels, and board types in the `Makefile` as needed.

## AT Commands

The available AT commands for the GUI application are managed by the AT command manager. Key commands include:

### General Commands:
- `AT+VERSION`: Get the AT commands version
- `AT+PING`: Ping command
- `AT+FW`: Get the firmware version
- `AT+ADDR`: Get/Set the device address
- `AT+SECKEY`: Get/Set the device secret key (Warning No AES encryption for the moment)
- `AT+ID`: Get/Set the device ID
- `AT+SN`: Get the device serial number
- `AT+RCONF`: Get/Set radio configuration
- `AT+SAVE_RCONF`: Save the radio configuration to Flash
- `AT+LPM`: Get/Set low power mode


### Forward Message Commands:
- `AT+TX`: Transmit data

### Certification Commands:
- `AT+CW`: Continuous Wave/MW commands

### Satellite Pass Predictions Commands:
- `AT+PREPASS_EN`: Get/Set the PREVIPASS algorithm
- `AT+UDATE`: Update UTC date/time

### MAC command
- `AT+KMAC`: Get/Set the MAC profile

Refer to the `mgr_at_cmd_list.h` file for the complete list of supported AT commands.

## Documentation

Auto-generated Doxygen documentation is available in the `Kineis/Doc` folder. Open `index.html` for detailed information.

## License

This project is licensed under the GNU General Public License v3. See the `LICENSE` file for details.

# Todo / in progress

- SPI driver instead of UART
- Add and manage CLS ID
- Programming firmware by SPI
