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
#include <stdbool.h>

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
 * BKP7R = consecutive crash counter (resets after stable uptime)
 */

/** Crash loop protection thresholds */
#define MGR_ERR_CRASH_LOOP_MAX       10    /**< Max consecutive short-lived boots */
#define MGR_ERR_CRASH_LOOP_MIN_UP_MS 30000 /**< Min uptime (ms) to consider boot "stable" */
#define MGR_ERR_CRASH_LOOP_SLEEP_S   3600  /**< Sleep duration (s) when crash loop detected */

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
	ERR_PA_STUCK,        /**< External PA stayed ON beyond watchdog threshold */
	ERR_BOOT_LOOP,       /**< Application never reached MONITORING after factory reset */
	ERR_STATE_HANG,      /**< State machine stuck in a non-steady state past MAX_STATE_HANG_MS */
	ERR_CREDS_BLANK,     /**< Page-0 credentials blank and no valid mirror — re-provision over AT */
	ERR_RTC_DEAD,        /**< RTC clock not ticking before STOP2 (LSE crystal death) — reset to LSI fallback */
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

/** @brief Get consecutive crash count (short-lived boots) */
uint32_t MGR_ERR_getCrashCount(void);

/**
 * @brief Check for crash loop and enter safe sleep if detected
 *
 * Must be called early in init. If too many consecutive short-lived
 * boots are detected, enters STOP mode for MGR_ERR_CRASH_LOOP_SLEEP_S
 * seconds before retrying, to preserve battery.
 *
 * @return true if crash loop was detected (device just woke from safe sleep)
 */
bool MGR_ERR_checkCrashLoop(void);

/**
 * @brief Log fault from ISR context (direct register access, no function calls)
 *
 * This macro is safe to call from HardFault/BusFault handlers where
 * the stack may be corrupted. Uses direct memory-mapped register writes.
 *
 * Uses uwTick (same backing variable as HAL_GetTick()) instead of
 * SysTick->VAL so crash loop detection can compare against MIN_UP_MS.
 * uwTick is a simple volatile uint32_t read — safe from fault context.
 *
 * @param code  MGR_ERR_Code_t value
 * @param state Current UW_DOPPLER state (or 0xFF if unknown)
 */
#define MGR_ERR_LOG_FAULT(code, state) do { \
	extern volatile uint32_t uwTick; \
	*((volatile uint32_t *)TAMP_BKP4R_ADDR) = (uint32_t)(code); \
	*((volatile uint32_t *)TAMP_BKP5R_ADDR) = (uint32_t)(state); \
	*((volatile uint32_t *)TAMP_BKP6R_ADDR) = uwTick; \
} while (0)

/* ---- HardFault forensics ----------------------------------------------- */

/** Full Cortex-M fault context, stored in SRAM2 retention RAM. Survives the
 *  NVIC_SystemReset triggered by the fault handler so the next boot can
 *  emit a forensic trace to UART + EVTLOG + PMLOG. */
typedef struct {
	uint32_t magic;       /**< 0xC4A5417F = "CRASH_INFO_v1" sentinel */
	uint8_t  fault_type;  /**< MGR_ERR_Code_t */
	uint8_t  app_state;   /**< g_uw_doppler_state_for_err at fault */
	uint16_t _pad;
	uint32_t hfsr;
	uint32_t cfsr;
	uint32_t bfar;
	uint32_t mmfar;
	uint32_t pc;          /**< Faulting PC (from stacked frame) */
	uint32_t lr;          /**< LR before fault (from stacked frame) */
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t xpsr;
	uint32_t tick;        /**< HAL tick at fault */
	uint32_t crc;
} MGR_ERR_CrashInfo_t;

#define MGR_ERR_CRASH_MAGIC  0xC4A5417FUL

/**
 * @brief Capture a fault into the retention-RAM crash_info struct.
 *
 * Called from the naked fault handlers in stm32wlxx_it.c. The handler reads
 * the active stack pointer (MSP for bare-metal builds), copies the 8-word
 * stacked frame + SCB fault registers into the struct, computes a CRC and
 * commits. Caller then invokes NVIC_SystemReset().
 *
 * @param frame   Pointer to the 8-word stacked exception frame on the SP.
 *                Layout: [0]=R0 [1]=R1 [2]=R2 [3]=R3 [4]=R12 [5]=LR
 *                        [6]=PC [7]=xPSR
 * @param fault_type One of ERR_HARDFAULT / ERR_MEMMANAGE / ERR_BUSFAULT /
 *                   ERR_USAGEFAULT / ERR_ASSERT.
 * @param app_state  Application state at fault (g_uw_doppler_state_for_err).
 */
void MGR_ERR_captureFault(uint32_t *frame, uint8_t fault_type, uint8_t app_state);

/**
 * @brief Return true if a valid crash_info from a previous boot is present.
 *
 * Use this on boot to decide whether to emit a forensic trace + clear the
 * struct. Validates both the magic sentinel and the CRC.
 */
bool MGR_ERR_hasRetainedCrash(void);

/**
 * @brief Copy the retained crash_info (if any) and clear the original.
 *
 * @param out  Receives the crash info. Untouched if hasRetainedCrash() == false.
 * @return true if a crash was retrieved.
 */
bool MGR_ERR_takeRetainedCrash(MGR_ERR_CrashInfo_t *out);

#endif /* MGR_ERR_H */
