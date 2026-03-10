# Unit test Makefile for argos-smd-at-kineis-firmware
# Usage:
#   make run        - Build and run all tests
#   make run-sws    - Build and run SWS tests only
#   make clean      - Remove build artifacts

CC      = gcc
CFLAGS  = -Wall -Wextra -Werror -Og -g -std=c11 -fsanitize=address,undefined
LDFLAGS = -fsanitize=address,undefined -lm
BUILD   = build

TESTS = test_crc32 test_dfu_protocol test_app_header test_spi_protocol \
        test_evtlog test_nvm test_tx_interval test_memory_safety \
        test_sws

.PHONY: all run clean $(addprefix run-,$(TESTS))

all: $(addprefix $(BUILD)/,$(TESTS))

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/%: %.c test_framework.h | $(BUILD)
	@$(CC) $(CFLAGS) -o $@ $< $(LDFLAGS)

run: all
	@pass=0; fail=0; total=0; \
	for t in $(TESTS); do \
		total=$$((total + 1)); \
		echo ""; \
		if $(BUILD)/$$t; then \
			pass=$$((pass + 1)); \
		else \
			fail=$$((fail + 1)); \
			echo ">>> SUITE FAILED: $$t"; \
		fi; \
	done; \
	echo ""; \
	echo "========================================"; \
	echo "ALL SUITES: $$total total, $$pass passed, $$fail failed"; \
	echo "========================================"; \
	[ $$fail -eq 0 ]

$(addprefix run-,$(TESTS)): run-%: $(BUILD)/%
	@$(BUILD)/$*

clean:
	@rm -rf $(BUILD)
