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

# Test executables
TEST_CRC32 = $(BUILD_DIR)/test_crc32
TEST_DFU = $(BUILD_DIR)/test_dfu_protocol
TEST_HEADER = $(BUILD_DIR)/test_app_header
TEST_SPI = $(BUILD_DIR)/test_spi_protocol

# All tests
TESTS = $(TEST_CRC32) $(TEST_DFU) $(TEST_HEADER) $(TEST_SPI)

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

# Run individual tests
run-crc32: $(TEST_CRC32)
	$(RUN_PREFIX)$(TEST_CRC32)

run-dfu: $(TEST_DFU)
	$(RUN_PREFIX)$(TEST_DFU)

run-header: $(TEST_HEADER)
	$(RUN_PREFIX)$(TEST_HEADER)

run-spi: $(TEST_SPI)
	$(RUN_PREFIX)$(TEST_SPI)

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

.PHONY: all run clean run-crc32 run-dfu run-header run-spi integration-uart integration-spi integration-spi-mock
