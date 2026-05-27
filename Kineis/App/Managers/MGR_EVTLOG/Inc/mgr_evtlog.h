/**
 * @file    mgr_evtlog.h
 * @brief   Event logger - circular buffer of 256 structured events in SRAM2 retention RAM
 *
 * Survives all resets (watchdog, software, brownout) except full power-off.
 * Each entry is 8 bytes: tick(4) + type(1) + state(1) + data(2).
 *
 * Usage:
 *   - Call MGR_EVTLOG_init() early in init (validates magic, resets if garbage)
 *   - Call MGR_EVTLOG_log(type, data) at each significant event
 *   - Use AT+LOG=? to dump the log over UART
 */

#ifndef MGR_EVTLOG_H
#define MGR_EVTLOG_H

#include <stdint.h>

#define MGR_EVTLOG_MAX_ENTRIES  256
#define MGR_EVTLOG_MAGIC        0x4C4F4721  /* "LOG!" */

typedef enum {
	EVT_BOOT = 0,        /**< System boot (data = reset cause bits) */
	EVT_STATE_CHANGE,    /**< State transition (data = new_state) */
	EVT_REED_ON,         /**< Magnet detected (data = 0) */
	EVT_REED_OFF,        /**< Magnet removed (data = hold_ms / 100) */
	EVT_SWS_SURFACE,     /**< Surface detected (data = adc value) */
	EVT_SWS_UNDERWATER,  /**< Underwater detected (data = adc value) */
	EVT_TX_START,        /**< TX started (data = tx_count) */
	EVT_TX_DONE,         /**< TX completed (data = 0) */
	EVT_TX_TIMEOUT,      /**< TX timeout (data = 0) */
	EVT_MAC_READY,       /**< MAC initialized OK (data = 0) */
	EVT_MAC_ERROR,       /**< MAC error (data = error code) */
	EVT_BAT,             /**< Battery reading (data = mV) */
	EVT_SHUTDOWN,        /**< Shutdown triggered (data = 0) */
	EVT_TIMEOUT,         /**< State timeout (data = timed-out state) */
	EVT_ERROR,           /**< Error logged (data = MGR_ERR_Code_t) */
	EVT_PA_STUCK,        /**< PA watchdog tripped (data = on-duration / 100 ms) */
	EVT_RATE_BLOCKED,    /**< TX blocked by rate limiter (data = retry_in_s capped to u16) */
	EVT_COOLDOWN_BLOCK,  /**< TX blocked by surface cooldown (data = remaining_s) */
	EVT_TX_BACKOFF,      /**< Backoff applied after consecutive errors (data = backoff_s) */
	EVT_BOOT_FAIL,       /**< Boot reached fail counter (data = consecutive_failures) */
	EVT_FACTORY_RESET,   /**< Factory reset triggered by boot-loop (data = reason) */
	EVT_LB_ENTER,        /**< Entered low-battery mode (data = bat_mV) */
	EVT_LB_EXIT,         /**< Left low-battery mode (data = bat_mV) */
	EVT_STATE_HANG,      /**< State machine hung > MAX_STATE_HANG_MS (data = state) */
	EVT_SWS_FAULT,       /**< SWS sensor fault detected (data = MGR_SWS_Fault_t bits) */
} MGR_EVTLOG_Type_t;

/**
 * @brief Severity classification for an event type.
 *
 * Implicit severity (looked up from the type, not stored in the entry) so the
 * on-RAM struct stays at 8 bytes and we don't break existing retention buffers.
 * The GUI can use the same enum to colour-code rows; the AT+LOG dumper prefixes
 * each line with a single character (T/I/W/E) for quick visual scanning.
 */
typedef enum {
	EVT_SEV_TRACE = 0,   /**< Fine-grain debug, normally filtered out */
	EVT_SEV_INFO,        /**< Expected operational event */
	EVT_SEV_WARN,        /**< Recoverable anomaly (retry, blocked, etc) */
	EVT_SEV_ERROR,       /**< Failure that triggered or will trigger a reset */
} MGR_EVTLOG_Severity_t;

/** @brief Lookup the canonical severity for an event type. */
MGR_EVTLOG_Severity_t MGR_EVTLOG_getSeverity(MGR_EVTLOG_Type_t type);

/** @brief Single-character tag for severity (T/I/W/E). */
char MGR_EVTLOG_severityChar(MGR_EVTLOG_Severity_t sev);

typedef struct {
	uint32_t tick;    /**< HAL_GetTick() timestamp */
	uint8_t  type;    /**< MGR_EVTLOG_Type_t */
	uint8_t  state;   /**< Current UW_DOPPLER state at time of event */
	uint16_t data;    /**< Context-dependent data */
} MGR_EVTLOG_Entry_t;

/**
 * @brief Initialize event log
 *
 * Checks magic word in SRAM2 retention buffer. If invalid (first boot or
 * power cycle), resets the buffer. If valid, log is preserved from previous
 * reset cycle.
 */
void MGR_EVTLOG_init(void);

/**
 * @brief Log an event
 *
 * Writes a new entry to the circular buffer. Automatically fills tick
 * from HAL_GetTick() and state from g_uw_doppler_state_for_err.
 *
 * @param type  Event type
 * @param data  Context-dependent data (e.g. adc value, mV, error code)
 */
void MGR_EVTLOG_log(MGR_EVTLOG_Type_t type, uint16_t data);

/**
 * @brief Get number of valid entries in the log
 * @return Entry count (0-256)
 */
uint16_t MGR_EVTLOG_count(void);

/**
 * @brief Get an entry by index (0 = oldest)
 * @param index  Entry index, 0-based from oldest
 * @return Pointer to entry, or NULL if index out of range
 */
const MGR_EVTLOG_Entry_t *MGR_EVTLOG_get(uint16_t index);

/**
 * @brief Clear all entries (reset buffer)
 */
void MGR_EVTLOG_clear(void);

#endif /* MGR_EVTLOG_H */
