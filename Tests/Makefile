#######################################
# Bootloader Unit Tests Makefile
#######################################

# Compiler - use WSL gcc if native gcc not available (Windows)
ifeq ($(shell which gcc 2>/dev/null),)
    # No native gcc, try WSL
    CC = wsl.exe gcc
    RUN_PREFIX = wsl.exe ./
else
    CC = gcc
    RUN_PREFIX = ./
endif
CFLAGS = -Wall -Wextra -g -O0 -std=c99

# Directories
UNIT_DIR = unit
BUILD_DIR = build

# Test executables - Bootloader
TEST_CRC32 = $(BUILD_DIR)/test_crc32
TEST_DFU = $(BUILD_DIR)/test_dfu_protocol
TEST_HEADER = $(BUILD_DIR)/test_app_header
TEST_SPI = $(BUILD_DIR)/test_spi_protocol

# Test executables - Tracker (UW_DOPPLER)
TEST_EVTLOG = $(BUILD_DIR)/test_evtlog
TEST_NVM = $(BUILD_DIR)/test_nvm
TEST_TX_INTERVAL = $(BUILD_DIR)/test_tx_interval
TEST_MEMORY_SAFETY = $(BUILD_DIR)/test_memory_safety

# Test executables - Phase A-F (reed/power-down/cooldown/self-test)
TEST_COOLDOWN_MODES   = $(BUILD_DIR)/test_cooldown_modes
TEST_REED_HOLD        = $(BUILD_DIR)/test_reed_hold_actions
TEST_BAT_AUTO_SHTDN   = $(BUILD_DIR)/test_bat_auto_shutdown
TEST_SELFTEST_MASK    = $(BUILD_DIR)/test_selftest_mask
TEST_COOLDOWN_PERSIST = $(BUILD_DIR)/test_cooldown_persist
TEST_LED_SWAP         = $(BUILD_DIR)/test_led_swap
TEST_GESTURE_FSM      = $(BUILD_DIR)/test_gesture_fsm

# All tests
TESTS = $(TEST_CRC32) $(TEST_DFU) $(TEST_HEADER) $(TEST_SPI) \
        $(TEST_EVTLOG) $(TEST_NVM) $(TEST_TX_INTERVAL) $(TEST_MEMORY_SAFETY) \
        $(TEST_COOLDOWN_MODES) $(TEST_REED_HOLD) $(TEST_BAT_AUTO_SHTDN) \
        $(TEST_SELFTEST_MASK) $(TEST_COOLDOWN_PERSIST) $(TEST_LED_SWAP) \
        $(TEST_GESTURE_FSM)

# Default target
all: $(BUILD_DIR) $(TESTS)
	@echo ""
	@echo "=========================================="
	@echo "All unit tests built successfully!"
	@echo "Run 'make run' to execute all tests"
	@echo "=========================================="

# Create build directory
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Build test executables
$(TEST_CRC32): $(UNIT_DIR)/test_crc32.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_DFU): $(UNIT_DIR)/test_dfu_protocol.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_HEADER): $(UNIT_DIR)/test_app_header.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_SPI): $(UNIT_DIR)/test_spi_protocol.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

# Tracker (UW_DOPPLER) tests
$(TEST_EVTLOG): $(UNIT_DIR)/test_evtlog.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_NVM): $(UNIT_DIR)/test_nvm.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_TX_INTERVAL): $(UNIT_DIR)/test_tx_interval.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_MEMORY_SAFETY): $(UNIT_DIR)/test_memory_safety.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

# Phase A-F build rules
$(TEST_COOLDOWN_MODES): $(UNIT_DIR)/test_cooldown_modes.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_REED_HOLD): $(UNIT_DIR)/test_reed_hold_actions.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_BAT_AUTO_SHTDN): $(UNIT_DIR)/test_bat_auto_shutdown.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_SELFTEST_MASK): $(UNIT_DIR)/test_selftest_mask.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_COOLDOWN_PERSIST): $(UNIT_DIR)/test_cooldown_persist.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_LED_SWAP): $(UNIT_DIR)/test_led_swap.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

$(TEST_GESTURE_FSM): $(UNIT_DIR)/test_gesture_fsm.c $(UNIT_DIR)/test_framework.h
	$(CC) $(CFLAGS) -I$(UNIT_DIR) $< -o $@

# Run all unit tests
run: all
	@echo ""
	@echo "=========================================="
	@echo "Running Unit Tests"
	@echo "=========================================="
	@echo ""
	@failed=0; \
	for test in $(TESTS); do \
		echo "Running $$test..."; \
		$(RUN_PREFIX)$$test || failed=1; \
	done; \
	if [ $$failed -eq 0 ]; then \
		echo ""; \
		echo "=========================================="; \
		echo "All unit tests PASSED!"; \
		echo "=========================================="; \
	else \
		echo ""; \
		echo "=========================================="; \
		echo "Some tests FAILED!"; \
		echo "=========================================="; \
		exit 1; \
	fi

# Generate an HTML report at report/index.html with per-suite results.
# Pure-bash generator — no dependencies beyond what already builds the tests.
report: all
	@mkdir -p report
	@echo "Generating HTML test report..."
	@./scripts/gen_html_report.sh $(TESTS) report/index.html
	@echo "Report: $$(pwd)/report/index.html"

# Run individual tests
run-crc32: $(TEST_CRC32)
	$(RUN_PREFIX)$(TEST_CRC32)

run-dfu: $(TEST_DFU)
	$(RUN_PREFIX)$(TEST_DFU)

run-header: $(TEST_HEADER)
	$(RUN_PREFIX)$(TEST_HEADER)

run-spi: $(TEST_SPI)
	$(RUN_PREFIX)$(TEST_SPI)

run-evtlog: $(TEST_EVTLOG)
	$(RUN_PREFIX)$(TEST_EVTLOG)

run-nvm: $(TEST_NVM)
	$(RUN_PREFIX)$(TEST_NVM)

run-tx-interval: $(TEST_TX_INTERVAL)
	$(RUN_PREFIX)$(TEST_TX_INTERVAL)

run-memory-safety: $(TEST_MEMORY_SAFETY)
	$(RUN_PREFIX)$(TEST_MEMORY_SAFETY)

run-cooldown-modes: $(TEST_COOLDOWN_MODES)
	$(RUN_PREFIX)$(TEST_COOLDOWN_MODES)

run-reed-hold: $(TEST_REED_HOLD)
	$(RUN_PREFIX)$(TEST_REED_HOLD)

run-bat-auto-shutdown: $(TEST_BAT_AUTO_SHTDN)
	$(RUN_PREFIX)$(TEST_BAT_AUTO_SHTDN)

run-selftest-mask: $(TEST_SELFTEST_MASK)
	$(RUN_PREFIX)$(TEST_SELFTEST_MASK)

run-cooldown-persist: $(TEST_COOLDOWN_PERSIST)
	$(RUN_PREFIX)$(TEST_COOLDOWN_PERSIST)

# Clean
clean:
	rm -rf $(BUILD_DIR)

# Integration tests (Python)
integration-uart:
	@echo "Running UART integration tests..."
	@echo "Usage: python integration/test_uart_dfu.py --port COMx"

integration-spi:
	@echo "Running SPI integration tests..."
	python integration/test_spi_dfu.py --mock

integration-spi-mock:
	python integration/test_spi_dfu.py --mock --verbose

.PHONY: all run clean run-crc32 run-dfu run-header run-spi run-evtlog run-nvm run-tx-interval run-memory-safety integration-uart integration-spi integration-spi-mock
