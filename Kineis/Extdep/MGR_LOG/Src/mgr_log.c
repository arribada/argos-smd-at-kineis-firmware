// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_log.c
 * @author Kinéis
 * @brief logger main file - Thread-safe non-blocking ring buffer implementation
 */

/**
 * @addtogroup MGR_LOG
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "mgr_log_conf.h"
#include "mgr_log.h"
#include "kns_app_conf.h"
#include "usart.h"
#include STM32_HAL_H
#include STM32_HAL_RTC_H


#ifdef USE_LOCAL_PRINTF

/* Defines -------------------------------------------------------------------*/

#define LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN 256

/** @brief Ring buffer size for non-blocking logging.
 *  Larger buffer = more messages can be queued before overflow
 *  Must be power of 2 for efficient modulo operation
 *  Note: With 115200 baud (12x faster than 9600), 4KB buffer
 *  provides equivalent capacity to 48KB at 9600 baud */
#define LOG_RING_BUFFER_SIZE 4096

/** @brief Maximum bytes to send per flush call to avoid blocking too long */
#define LOG_FLUSH_MAX_BYTES 1024

/** @brief UART transmit timeout per flush (short to avoid blocking) */
#define LOG_FLUSH_TIMEOUT_MS 50

/* Critical section macros */
#define LOG_ENTER_CRITICAL()  uint32_t primask = __get_PRIMASK(); __disable_irq()
#define LOG_EXIT_CRITICAL()   __set_PRIMASK(primask)

/* Variables -----------------------------------------------------------------*/

/** @brief Ring buffer for non-blocking logging */
static volatile char log_ring_buffer[LOG_RING_BUFFER_SIZE];
static volatile uint16_t log_ring_head = 0;  /* Write position */
static volatile uint16_t log_ring_tail = 0;  /* Read position */
static volatile uint32_t log_overflow_count = 0;  /* Overflow counter for diagnostics */
static volatile bool log_in_use = false;  /* Mutex for formatting buffer */

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Get number of bytes available in ring buffer (NOT thread-safe, call within critical section)
 */
static inline uint16_t log_ring_available_unsafe(void)
{
	return (log_ring_head - log_ring_tail) & (LOG_RING_BUFFER_SIZE - 1);
}

/**
 * @brief Get free space in ring buffer (NOT thread-safe)
 */
static inline uint16_t log_ring_free_space(void)
{
	return (LOG_RING_BUFFER_SIZE - 1) - ((log_ring_head - log_ring_tail) & (LOG_RING_BUFFER_SIZE - 1));
}

/**
 * @brief Add a string to ring buffer atomically - ALL OR NOTHING
 * @param str String to add
 * @param len Length of string
 * @return Number of characters actually added (0 if not enough space)
 */
static uint16_t log_ring_put_string(const char *str, uint16_t len)
{
	LOG_ENTER_CRITICAL();

	/* Check if there's enough space for the ENTIRE message */
	uint16_t free = log_ring_free_space();
	if (free < len) {
		/* Not enough space - drop entire message to avoid losing \r\n */
		log_overflow_count++;
		LOG_EXIT_CRITICAL();
		return 0;
	}

	/* Write all characters */
	uint16_t head = log_ring_head;
	for (uint16_t i = 0; i < len; i++) {
		log_ring_buffer[head] = str[i];
		head = (head + 1) & (LOG_RING_BUFFER_SIZE - 1);
	}
	log_ring_head = head;

	LOG_EXIT_CRITICAL();
	return len;
}

/**
 * @brief Get characters from ring buffer atomically
 * @param buf Buffer to fill
 * @param max_count Maximum characters to get
 * @return Number of characters retrieved
 */
static uint16_t log_ring_get_string(uint8_t *buf, uint16_t max_count)
{
	LOG_ENTER_CRITICAL();

	uint16_t count = 0;
	uint16_t tail = log_ring_tail;

	while (count < max_count && log_ring_head != tail) {
		buf[count++] = log_ring_buffer[tail];
		tail = (tail + 1) & (LOG_RING_BUFFER_SIZE - 1);
	}

	log_ring_tail = tail;

	LOG_EXIT_CRITICAL();
	return count;
}

/* External RTC handle for timestamp formatting */
extern RTC_HandleTypeDef hrtc;

/* Functions -----------------------------------------------------------------*/

/**
 * @brief Thread-safe non-blocking printf with timestamp to ring buffer
 *
 * This function formats timestamp + message together in one buffer and adds
 * it atomically to the ring buffer. This prevents log interleaving issues
 * that occur when timestamp and message are added separately.
 */
void vMGR_LOG_printf_ts(const char *format, ...)
{
	char local_buf[LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN];
	RTC_DateTypeDef sdate;
	RTC_TimeTypeDef stime;
	va_list args;
	int timestamp_len, msg_len, total_len;

	/* Get RTC time first */
	HAL_RTC_WaitForSynchro(&hrtc);
	HAL_RTC_GetTime(&hrtc, &stime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sdate, RTC_FORMAT_BIN);

	/* Format timestamp at start of buffer */
	timestamp_len = snprintf(local_buf, LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN,
							 "%04d/%02d/%02d %02d:%02d:%02d ",
							 2000 + sdate.Year, sdate.Month, sdate.Date,
							 stime.Hours, stime.Minutes, stime.Seconds);

	if (timestamp_len < 0) timestamp_len = 0;
	if (timestamp_len >= LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN) {
		timestamp_len = LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN - 1;
	}

	/* Format message right after timestamp */
	va_start(args, format);
	msg_len = vsnprintf(local_buf + timestamp_len,
						LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN - timestamp_len,
						format, args);
	va_end(args);

	if (msg_len < 0) msg_len = 0;
	if (timestamp_len + msg_len >= LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN) {
		msg_len = LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN - timestamp_len - 1;
	}

	/* Add entire formatted message to ring buffer atomically */
	total_len = timestamp_len + msg_len;
	if (total_len > 0) {
		log_ring_put_string(local_buf, (uint16_t)total_len);
	}
}

/**
 * @brief Thread-safe non-blocking printf to ring buffer
 *
 * This function formats the message and adds it to a ring buffer atomically.
 * It does NOT block on UART transmission.
 * Call MGR_LOG_flush() from main loop to actually send the data via UART.
 */
void vMGR_LOG_printf(const char *format, ...)
{
	/* Use local buffer on stack to avoid conflicts between concurrent calls */
	char local_buf[LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN];
	va_list args;
	int len;

	/* Format the message into local buffer */
	va_start(args, format);
	len = vsnprintf(local_buf, LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN - 1, format, args);
	va_end(args);

	/* Limit length to buffer size */
	if (len < 0) {
		len = 0;
	} else if (len >= LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN) {
		len = LOGGING_PURPOSE_STORAGE_MAX_MESSAGE_LEN - 1;
	}

	/* Add formatted message to ring buffer atomically */
	if (len > 0) {
		log_ring_put_string(local_buf, (uint16_t)len);
	}
}

/**
 * @brief Flush log ring buffer to UART (call from main loop)
 *
 * This function sends buffered log data via UART.
 * It sends up to LOG_FLUSH_MAX_BYTES per call to avoid blocking too long.
 * Should be called regularly from the main loop.
 *
 * @return Number of bytes sent
 */
uint16_t MGR_LOG_flush(void)
{
	static uint8_t flush_buf[LOG_FLUSH_MAX_BYTES];
	uint16_t count;

	/* Get characters from ring buffer atomically */
	count = log_ring_get_string(flush_buf, LOG_FLUSH_MAX_BYTES);

	/* Send collected data via UART */
	if (count > 0) {
		HAL_UART_Transmit(&log_uart, flush_buf, count, LOG_FLUSH_TIMEOUT_MS);
	}

	return count;
}

/**
 * @brief Flush all pending log data (blocking until empty)
 *
 * Use this before entering low power mode or at shutdown.
 */
void MGR_LOG_flush_all(void)
{
	while (MGR_LOG_has_pending()) {
		MGR_LOG_flush();
	}
}

/**
 * @brief Get log overflow count for diagnostics
 * @return Number of characters dropped due to buffer overflow
 */
uint32_t MGR_LOG_get_overflow_count(void)
{
	return log_overflow_count;
}

/**
 * @brief Check if log buffer has pending data
 * @return true if there is data to flush
 */
bool MGR_LOG_has_pending(void)
{
	LOG_ENTER_CRITICAL();
	bool pending = (log_ring_head != log_ring_tail);
	LOG_EXIT_CRITICAL();
	return pending;
}

#endif // end USE_LOCAL_PRINTF

/**
 * @}
 */
