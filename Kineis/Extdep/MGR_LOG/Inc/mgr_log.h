/* SPDX-License-Identifier: no SPDX license */
/**
 * @file mgr_log.h
 * @author Kinéis
 * @brief logger main header file
 *
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Kineis. All rights reserved.</center></h2>
 *
 * This software component is licensed by Kineis under Ultimate Liberty license, the "License";
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * * kineis.com
 *
 */

/**
 * @page mgr_log_page LOGGER
 *
 * This page is presenting the logging strategy for the kineis stack (\ref MGR_LOG).
 *
 * so far, two levels of logging are available: DEBUG, VERBOSE. The logging routines are defined
 * through static macros.
 *
 * Depending on the enabled compilation flags, the source code of the kineis stack will invoke
 * empty macros (DEBUG=0, VERBOSE=0) or calls to a routine (\ref vMGR_LOG_printf if DEBUG=1 or
 * VERBOSE=1). This strategy allow to limit the footprint an dimpact on real-time at maximum
 * when logging is disabled.
 *
 * Then, a third static compilation is impacting \ref vMGR_LOG_printf routine:
 * * if USE_LOCAL_PRINTF is defined, vMGR_LOG_printf is a real function coded in \ref mgr_log.c
 * * if USE_LOCAL_PRINTF is NOT  defined, vMGR_LOG_printf is actually replace by legacy printf
 * function.
 *
 * By defult, in most of the packages delivered by Kineis, this Log Manager is actually outpûting
 * the log to a dedicated console (via UART).
 *
 */

/**
 * @addtogroup MGR_LOG
 * @{
 */

#ifndef MGR_LOG_H
#define MGR_LOG_H

/* Includes ------------------------------------------------------------------*/

#ifdef DEBUG
#include "mgr_log_rtc.h"
#endif

/* Defines -------------------------------------------------------------------*/

/* Legacy macros — preserved for the 500+ existing call sites.
 *
 * In DEBUG=1 they route to the new level system at TRACE (runtime-gated by
 * AT+LOGLVL). In DEBUG=0 they remain compile-time no-ops to keep the
 * production binary footprint unchanged. New code SHOULD use the
 * severity-named macros declared further down (MGR_LOG_TRACE / INFO / WARN
 * / ERR) which work in both builds. */
#ifdef DEBUG

/* Forward-declare the runtime gate so the macros below can reference it
 * before the function prototypes appear (USE_LOCAL_PRINTF block). */
#define MGR_LOG_DEBUG(...)		MGR_LOG_AT(MGR_LOG_LVL_TRACE, __VA_ARGS__)
#define MGR_LOG_DEBUG_RAW(...)		do { \
		if (MGR_LOG_passes(MGR_LOG_LVL_TRACE)) \
			vMGR_LOG_printf(__VA_ARGS__); \
	} while (0)

#ifdef VERBOSE
#define MGR_LOG_VERBOSE(...)		MGR_LOG_AT(MGR_LOG_LVL_TRACE, __VA_ARGS__)
#define MGR_LOG_VERBOSE_RAW(...)	do { \
		if (MGR_LOG_passes(MGR_LOG_LVL_TRACE)) \
			vMGR_LOG_printf(__VA_ARGS__); \
	} while (0)
#else  // else VERBOSE
#define MGR_LOG_VERBOSE(...)		do {} while (0)
#define MGR_LOG_VERBOSE_RAW(...)	do {} while (0)
#endif // end VERBOSE


#else // not DEBUG

#define MGR_LOG_DEBUG(...)		do {} while (0)
#define MGR_LOG_DEBUG_RAW(...)		do {} while (0)
#define MGR_LOG_VERBOSE(...)		do {} while (0)
#define MGR_LOG_VERBOSE_RAW(...)	do {} while (0)

#endif // end not DEBUG

/* Functions -----------------------------------------------------------------*/

/**
 * @brief Write formatted string into a buffer with variable number of
 * parameters with a specified maximum number of chars to write.
 *
 * This function is NON-BLOCKING - it adds the message to a ring buffer.
 * Call MGR_LOG_flush() from main loop to actually send the data via UART.
 *
 * @param[in] format: format of the string (same syntax as printf)
 *
 * @return none
 */

#ifdef USE_LOCAL_PRINTF
#include <stdbool.h>
#include <stdint.h>

void vMGR_LOG_printf(const char *format, ...);
void vMGR_LOG_printf_ts(const char *format, ...);

/** @brief Runtime gate on spontaneous logs. AT+UARTLOG=0 silences the
 *  whole log ring (saves UART current + host parser noise) while keeping
 *  AT responses intact. Default is enabled. Persisted in retention NOLOAD. */
void vMGR_LOG_setEnabled(bool enabled);
bool vMGR_LOG_isEnabled(void);

/** @brief Forget the runtime override and fall back to the compile-time
 *  default (DEBUG=0 -> off, DEBUG=1 -> on). Called by the gesture FSM when
 *  leaving CONFIG so the temporary force-on doesn't persist. */
void vMGR_LOG_resetToDefault(void);

/** @brief Log severity classification.
 *
 * Ordered low-to-high so a runtime threshold "emit if level >= threshold"
 * naturally silences chatty levels first. NONE is a sentinel meaning
 * "drop everything" (useful for race-time silent runs). */
typedef enum {
	MGR_LOG_LVL_TRACE   = 0,  /**< per-sample chatter (hb, [SWS], KNS_Q...). */
	MGR_LOG_LVL_INFO    = 1,  /**< state transitions, TX lifecycle, modes. */
	MGR_LOG_LVL_WARNING = 2,  /**< recoverable: rate limit, low-bat, backoff. */
	MGR_LOG_LVL_ERROR   = 3,  /**< unrecoverable: HW fault, TX timeout, IWDG. */
	MGR_LOG_LVL_NONE    = 4,  /**< total silence — emits nothing. */
} MGR_LOG_Level_t;

/** @brief Compile-time default level. Overridable from the Makefile via
 *  `make ... LOG_LEVEL=<n>` which sets `-DLOG_DEFAULT_LEVEL=<n>`.
 *
 *  INFO by default in ALL builds: the TRACE firehose (hb heartbeat, one
 *  [SWS] line per sample, KNS_Q push/pop dumps...) saturates the 115200
 *  console; under STOP2 duty-cycling (~50 ms TX windows) every line then
 *  arrives seconds late and the whole bench FEELS laggy — reed, SWS and
 *  AT all appear delayed when only the console is. Opt back into the
 *  firehose at runtime with AT+LOGLVL=0 when chasing a specific bug. */
#ifndef LOG_DEFAULT_LEVEL
#ifdef DEBUG
#  define LOG_DEFAULT_LEVEL  MGR_LOG_LVL_INFO
#else
   /* Production: failures only — pairs with the quiet UARTLOG default so a
    * parser GUI sees a clean stream. AT+LOGLVL overrides at runtime. */
#  define LOG_DEFAULT_LEVEL  MGR_LOG_LVL_ERROR
#endif
#endif

/** @brief Get / set the current runtime threshold. Persisted in retention
 *  NOLOAD so it survives every soft reset; VBAT loss reverts to default. */
void            MGR_LOG_setLevel(MGR_LOG_Level_t level);
MGR_LOG_Level_t MGR_LOG_getLevel(void);

/** @brief True iff a message of this severity should be emitted now. Cheap
 *  inline gate suitable for hot paths (single comparison + an isEnabled()
 *  check that itself is two reads). */
bool            MGR_LOG_passes(MGR_LOG_Level_t level);

/** @brief Return the 4-char tag string ("[T] " / "[I] " / "[W] " / "[E] ")
 *  for the given level. Useful for direct-UART traces that bypass the ring
 *  but still want their level visible in the output stream.
 *
 *  Example: `snprintf(buf, sz, "%shb t=%lu ...", MGR_LOG_levelTag(MGR_LOG_LVL_TRACE), tick);` */
const char *    MGR_LOG_levelTag(MGR_LOG_Level_t level);

/** @brief Level-aware ring-buffer printf. Prefix in the emitted line is
 *  `<RTC ts> <level-tag> <payload>`. The macros below route through this. */
void vMGR_LOG_printf_ts_lvl(MGR_LOG_Level_t level, const char *format, ...);

/** @brief Severity-tagged log macros. All share the timestamp-prefixed
 *  ring-buffer path. Output format:
 *      `2026/06/10 00:11:02 [I] [UW_DPL] Surface detected, starting TX`
 *  Cost when filtered out: one comparison + one isEnabled() check.
 *
 *  The legacy MGR_LOG_DEBUG / MGR_LOG_VERBOSE macros below are aliased to
 *  TRACE so existing call sites need no edit. New code should use the
 *  severity-named macros directly. */
#define MGR_LOG_AT(level, ...)  do { \
	if (MGR_LOG_passes(level)) \
		vMGR_LOG_printf_ts_lvl((level), __VA_ARGS__); \
} while (0)

#define MGR_LOG_TRACE(...)   MGR_LOG_AT(MGR_LOG_LVL_TRACE,   __VA_ARGS__)
#define MGR_LOG_INFO(...)    MGR_LOG_AT(MGR_LOG_LVL_INFO,    __VA_ARGS__)
#define MGR_LOG_WARN(...)    MGR_LOG_AT(MGR_LOG_LVL_WARNING, __VA_ARGS__)
#define MGR_LOG_ERR(...)     MGR_LOG_AT(MGR_LOG_LVL_ERROR,   __VA_ARGS__)


/**
 * @brief Flush log ring buffer to UART (call from main loop)
 *
 * This function sends buffered log data via UART.
 * It sends a limited number of bytes per call to avoid blocking too long.
 * Should be called regularly from the main loop.
 *
 * @return Number of bytes sent
 */
uint16_t MGR_LOG_flush(void);

/**
 * @brief Flush all pending log data (blocking until empty)
 *
 * Use this before entering low power mode or at shutdown.
 */
void MGR_LOG_flush_all(void);

/**
 * @brief Get log overflow count for diagnostics
 * @return Number of characters dropped due to buffer overflow
 */
uint32_t MGR_LOG_get_overflow_count(void);

/**
 * @brief Check if log buffer has pending data
 * @return true if there is data to flush
 */
bool MGR_LOG_has_pending(void);

/**
 * @brief Pause log flushing (for AT console priority)
 *
 * While paused, MGR_LOG_flush() returns immediately without sending.
 * Logs are still buffered in the ring buffer.
 * Use this to give AT UART responses absolute priority.
 */
void MGR_LOG_pause(void);

/**
 * @brief Resume log flushing after pause
 */
void MGR_LOG_resume(void);

/**
 * @brief Check if log flushing is paused
 * @return true if paused
 */
bool MGR_LOG_is_paused(void);

/**
 * @brief Check if log UART transmission is in progress
 * @return true if UART is busy transmitting logs
 */
bool MGR_LOG_is_uart_busy(void);

#else // not USE_LOCAL_PRINTF

#define vMGR_LOG_printf			printf
#define MGR_LOG_flush()			(0)
#define MGR_LOG_flush_all()		do {} while(0)
#define MGR_LOG_get_overflow_count()	(0)
#define MGR_LOG_has_pending()		(false)
#define MGR_LOG_pause()			do {} while(0)
#define MGR_LOG_resume()		do {} while(0)
#define MGR_LOG_is_paused()		(false)
#define MGR_LOG_is_uart_busy()		(false)

#endif // end USE_LOCAL_PRINTF

#endif // end MGR_LOG_H

/**
 * @}
 */
