/**
 * @file    mgr_err.h
 * @brief   Error tracker using TAMP backup registers
 *
 * Persists error context (reset count, last fault, last state) across
 * system resets using STM32WL TAMP backup registers BKP2R-BKP7R.
 * These survive all resets except VBAT removal.
 *
 * Usage:
 *   - Call MGR_ERR_init() early in main() to log previous reset cause
 *   - Call MGR_ERR_log() before NVIC_SystemReset() to save context
 *   - From fault handlers, use MGR_ERR_logFault() (direct register, no function calls)
 */

#ifndef MGR_ERR_H
#define MGR_ERR_H

#include <stdint.h>

/* TAMP backup register addresses (STM32WL55) */
#define TAMP_BKP2R_ADDR  0x4000B108UL
#define TAMP_BKP3R_ADDR  0x4000B10CUL
#define TAMP_BKP4R_ADDR  0x4000B110UL
#define TAMP_BKP5R_ADDR  0x4000B114UL
#define TAMP_BKP6R_ADDR  0x4000B118UL
#define TAMP_BKP7R_ADDR  0x4000B11CUL

/* Register layout:
 * BKP2R = reset counter (incremented each boot)
 * BKP3R = last reset cause (RCC_CSR flags snapshot)
 * BKP4R = last error code (MGR_ERR_Code_t)
 * BKP5R = last UW_DOPPLER state at time of error
 * BKP6R = last HAL tick at time of error
 * BKP7R = reserved
 */

typedef enum {
	ERR_NONE = 0,
	ERR_HARDFAULT,
	ERR_BUSFAULT,
	ERR_MEMMANAGE,
	ERR_USAGEFAULT,
	ERR_NMI,
	ERR_ASSERT,
	ERR_MAC_TIMEOUT,
	ERR_MAC_INIT_FAIL,
	ERR_TX_TIMEOUT,
	ERR_STACK_OVERFLOW,
	ERR_WDG_RESET,
} MGR_ERR_Code_t;

/**
 * @brief Initialize error tracker
 *
 * Reads previous reset cause from RCC_CSR, logs boot info,
 * increments reset counter, clears error code for this session.
 * Must be called after enable_backup_access() in main().
 */
void MGR_ERR_init(void);

/**
 * @brief Log an error code to TAMP registers
 *
 * Stores error code, current UW_DOPPLER state, and HAL tick.
 * Does NOT trigger reset - caller must do that if needed.
 *
 * @param code Error code to store
 */
void MGR_ERR_log(MGR_ERR_Code_t code);

/**
 * @brief Log error and trigger system reset
 *
 * Calls MGR_ERR_log() then NVIC_SystemReset().
 * Does not return.
 *
 * @param code Error code to store
 */
void MGR_ERR_logAndReset(MGR_ERR_Code_t code) __attribute__((noreturn));

/** @brief Get total reset count since VBAT power-on */
uint32_t MGR_ERR_getResetCount(void);

/** @brief Get last error code from previous session */
MGR_ERR_Code_t MGR_ERR_getLastError(void);

/**
 * @brief Log fault from ISR context (direct register access, no function calls)
 *
 * This macro is safe to call from HardFault/BusFault handlers where
 * the stack may be corrupted. Uses direct memory-mapped register writes.
 *
 * @param code  MGR_ERR_Code_t value
 * @param state Current UW_DOPPLER state (or 0xFF if unknown)
 */
#define MGR_ERR_LOG_FAULT(code, state) do { \
	*((volatile uint32_t *)TAMP_BKP4R_ADDR) = (uint32_t)(code); \
	*((volatile uint32_t *)TAMP_BKP5R_ADDR) = (uint32_t)(state); \
	*((volatile uint32_t *)TAMP_BKP6R_ADDR) = SysTick->VAL; \
} while (0)

#endif /* MGR_ERR_H */
