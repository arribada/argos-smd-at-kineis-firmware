##########################################################################################################################
# Bootloader Makefile for STM32WL55
##########################################################################################################################

# Force bash as the shell on Windows (Git Bash). Recipes use bash/Unix syntax.
ifeq ($(OS),Windows_NT)
SHELL := C:/Program Files/Git/bin/bash.exe
.SHELLFLAGS := -c
endif

######################################
# target
######################################
TARGET = bootloader

######################################
# building variables
######################################
# Debug mode: set DEBUG=1 on command line to enable debug prints
# Release: make PROTOCOL=SPI (or UART) - no debug output
# Debug:   make PROTOCOL=SPI DEBUG=1   - with debug output
DEBUG ?= 0
OPT = -Os

# Protocol mode: SPI (default) or UART
# Usage: make PROTOCOL=UART  or  make PROTOCOL=SPI
PROTOCOL ?= SPI

#######################################
# paths
#######################################
BUILD_DIR = build
BL_DIR = .
ROOT_DIR = ..

######################################
# source
######################################
# C sources
C_SOURCES =  \
$(BL_DIR)/Src/bl_main.c \
$(BL_DIR)/Src/bl_flash.c \
$(BL_DIR)/Src/bl_crc.c \
$(BL_DIR)/Src/bl_dfu.c \
$(BL_DIR)/Src/bl_uart.c \
$(BL_DIR)/Src/bl_spi.c \
$(BL_DIR)/Src/bl_spi_protocol.c \
$(BL_DIR)/Src/bl_syscalls.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_cortex.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_rcc.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_rcc_ex.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_gpio.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_dma.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_dma_ex.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_flash.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_flash_ex.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_pwr.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_pwr_ex.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_uart.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_uart_ex.c \
$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Src/stm32wlxx_hal_spi.c \
$(ROOT_DIR)/Core/Src/system_stm32wlxx.c

# ASM sources
ASM_SOURCES = $(BL_DIR)/startup_bl.s

#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

#######################################
# CFLAGS
#######################################
CPU = -mcpu=cortex-m4
MCU = $(CPU) -mthumb

# C defines
# USE_SPI_DRIVER is always defined to enable HAL SPI module (both modes use SPI)
C_DEFS =  \
-DCORE_CM4 \
-DUSE_HAL_DRIVER \
-DSTM32WL55xx \
-DBL_BUILD \
-DUSE_SPI_DRIVER

# Protocol-specific defines
# Both UART and SPI are always supported, but the default baudrate changes
ifeq ($(PROTOCOL), UART)
C_DEFS += -DBL_PROTOCOL_UART
$(info Building bootloader for UART mode (DFU baud=$(BL_UART_BAUD), SPI still enabled))
else
C_DEFS += -DBL_PROTOCOL_SPI
$(info Building bootloader for SPI mode (115200 baud for UART debug))
endif

ifeq ($(DEBUG), 1)
C_DEFS += -DDEBUG
endif

# Optional board activity LED indicator (e.g. SMD_STDALONE RGB on PA1/PB4/PB5).
# OFF by default so already-deployed bootloaders stay byte-identical.
# Enable only for a board with free LED pins in UART-DFU: make BL_LED=1
ifeq ($(BL_LED), 1)
C_DEFS += -DBL_LED
$(info Bootloader LED indicator ENABLED (RGB on PA1/PB4/PB5))
endif

# Optional explicit UART DFU baud override (decouples baud from PROTOCOL).
# Unset by default -> baud follows PROTOCOL (deployed builds unchanged).
# UW_DOPPLER board console is 115200: make PROTOCOL=UART BL_UART_BAUD=115200
ifdef BL_UART_BAUD
C_DEFS += -DBL_UART_BAUDRATE=$(BL_UART_BAUD)
$(info Bootloader UART DFU baud forced to $(BL_UART_BAUD))
endif

# C includes
C_INCLUDES =  \
-I$(BL_DIR)/Inc \
-I$(ROOT_DIR)/Core/Inc \
-I$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Inc \
-I$(ROOT_DIR)/Drivers/STM32WLxx_HAL_Driver/Inc/Legacy \
-I$(ROOT_DIR)/Drivers/CMSIS/Device/ST/STM32WLxx/Include \
-I$(ROOT_DIR)/Drivers/CMSIS/Include

# compile gcc flags
ASFLAGS = $(MCU) $(OPT) -pipe -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -pipe -Wall -Wextra -fdata-sections -ffunction-sections
CFLAGS += -Wno-unused-parameter

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MD -MP -MF"$(@:%.o=%.d)"

#######################################
# LDFLAGS
#######################################
LDSCRIPT = $(BL_DIR)/STM32WL55XX_BL.ld

LIBS = -lc -lm -lnosys
LDFLAGS = $(MCU) -pipe -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections -static
# Silence the benign "LOAD segment with RWX permissions" hint from ld >=2.39
# (RAM is (xrw); harmless on Cortex-M bare-metal). No effect on the output bytes.
# Only exists in ld >= 2.39 (Arm GNU Toolchain 12+); older toolchains (ld 2.35)
# FAIL the link on the unknown option. Probe the linker and add it only when
# supported so the bootloader builds on any toolchain vintage.
LDFLAG_NOWARN_RWX := $(shell $(CC) -Wl,--no-warn-rwx-segments -nostdlib -x c /dev/null -o /dev/null > /dev/null 2>&1 && echo -Wl,--no-warn-rwx-segments)
LDFLAGS += $(LDFLAG_NOWARN_RWX)

#######################################
# build the bootloader
#######################################
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	@echo '==== Bootloader build completed ===='
	$(SZ) -t $(BUILD_DIR)/$(TARGET).elf

# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	TMPDIR=/tmp TMP=/tmp TEMP=/tmp $(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# flash
#######################################
JLINK_SERIAL ?= 801035790
JLINK_SPEED ?= 4000

flash: $(BUILD_DIR)/$(TARGET).hex
	@echo "Flashing bootloader to 0x08033000..."
	JLinkExe -device STM32WL55JC -if SWD -speed $(JLINK_SPEED) -SelectEmuBySN $(JLINK_SERIAL) -CommanderScript flash_bl.jlink

flash_bl.jlink: $(BUILD_DIR)/$(TARGET).hex
	@echo "h" > flash_bl.jlink
	@echo "erase 0x08033000 0x0803B000" >> flash_bl.jlink
	@echo "loadfile $(BUILD_DIR)/$(TARGET).hex" >> flash_bl.jlink
	@echo "r" >> flash_bl.jlink
	@echo "g" >> flash_bl.jlink
	@echo "exit" >> flash_bl.jlink

.PHONY: all clean flash

-include $(wildcard $(BUILD_DIR)/*.d)
