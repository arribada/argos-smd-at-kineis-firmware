/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    mcu_spi_console.h
 * @brief   MCU wrapper for SPI console - Pipelined Single-Transaction Protocol.
 *
 * This driver implements a pipelined single-transaction SPI protocol:
 * - Each transaction is a fixed 64 bytes
 * - Master sends command, slave simultaneously sends response to PREVIOUS command
 * - No timing issues: response is always ready before master reads it
 *
 * @author Arribada
 */

/**
 * @addtogroup MCU_APP_WRAPPERS
 * @brief MCU wrappers used by the Kineis Application example.
 * @{
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCU_SPI_CONSOLE_H
#define __MCU_SPI_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "kns_app_conf.h" // for STM32 HAL includes on UART
#include STM32_HAL_H

/* Protocol configuration - Pipelined single-transaction */
#define SPI_TRANSACTION_SIZE    64   /**< Fixed transaction size in bytes */
#define SPI_CMD_MAX_SIZE        32   /**< Max command size (first half of transaction) */
#define SPI_RESPONSE_MAX_SIZE   32   /**< Max response size (fits in transaction) */
#define SPI_IDLE_PATTERN        0xAA /**< Idle pattern (no response ready) */
#define SPI_BUSY_PATTERN        0xBB /**< Busy pattern (processing) */

#ifdef USE_HDA4
#define TXBUF_SIZE 2560
#define RXBUF_SIZE 2560
#else
#define TXBUF_SIZE 256
#define RXBUF_SIZE 256
#endif

/* Thread safety macros for ISR/main context synchronization */
#define SPI_ENTER_CRITICAL() __disable_irq()
#define SPI_EXIT_CRITICAL()  __enable_irq()

/**
 * @brief Structure for SPI driver statistics and diagnostics.
 */
typedef struct {
    uint32_t rx_count;        /**< Total successful RX transactions */
    uint32_t tx_count;        /**< Total successful TX transactions */
    uint32_t error_count;     /**< Total error occurrences */
    uint32_t timeout_count;   /**< Total timeout occurrences */
    uint32_t reset_count;     /**< Total driver resets */
    uint32_t last_error;      /**< Last HAL error code */
} SPI_Stats_t;

extern SPI_Stats_t spi_stats; /**< Global SPI statistics */

/**
 * @brief Structure representing an SPI data buffer.
 */
typedef struct {
    uint8_t *data;      /**< Pointer to the data buffer */
    uint16_t size;      /**< Current size of valid data in the buffer */
    uint16_t next_req;  /**< Number of elements expected to be read/sent next */
} SPI_Buffer;

extern SPI_Buffer rxBuf;   /**< Global SPI reception buffer */
extern SPI_Buffer txBuf;   /**< Global SPI transmission buffer */

/**
 * @brief Enumeration of SPI console states (simplified for pipelined protocol).
 */
typedef enum {
    SPICMD_UNKNOWN,           /**< Unknown state */
    SPICMD_INIT,              /**< SPI console initialization state */
    SPICMD_IDLE,              /**< Idle state, DMA armed waiting for transaction */
    SPICMD_PROCESS_CMD,       /**< Processing received command, preparing response */
    SPICMD_ERROR              /**< Error state */
} SpiState;

extern volatile SpiState spiState;         /**< Current state of the SPI console */
extern volatile uint32_t startTickTimeout; /**< Timeout tick value for SPI operations */

/* Flag indicating if a valid response is ready in TX buffer */
extern volatile bool response_ready;

/* Functions prototypes ------------------------------------------------------*/

/**
 * @brief Start the SPI console for command reception.
 *
 * @param[in] context Pointer to SPI handle.
 * @param[in] rx_spi_evt_cb Callback function (not used in pipelined mode, can be NULL).
 *
 * @return true if the SPI console is successfully started, false otherwise.
 */
bool MCU_SPI_DRIVER_register(void *context,
		int8_t (*rx_spi_evt_cb)(SPI_Buffer *rx, SPI_Buffer *tx));

/**
 * @brief Perform an SPI read operation (arm DMA for next transaction).
 *
 * In pipelined mode, this arms the DMA with the current TX buffer content
 * (previous response or idle pattern) and waits for the next transaction.
 *
 * @return HAL status of the operation.
 */
HAL_StatusTypeDef MCU_SPI_DRIVER_read(void);

/**
 * @brief Arm DMA with BUSY pattern.
 *
 * Call this before starting long operations (like flash write) so that
 * the master can detect the slave is busy and retry later.
 * This prevents the slave from missing transactions during processing.
 *
 * @return HAL status of the operation.
 */
HAL_StatusTypeDef MCU_SPI_DRIVER_arm_busy(void);

/**
 * @brief Set response data for the next transaction.
 *
 * Call this after processing a command to set the response that will be
 * sent in the next transaction. The response is copied to the TX buffer.
 *
 * @param[in] data Pointer to response data
 * @param[in] len Length of response data
 * @return true if response was set, false if buffer overflow
 */
bool MCU_SPI_DRIVER_set_response(const uint8_t *data, uint16_t len);

/**
 * @brief Check if SPI transaction has ended.
 *
 * @param[out] bytes_received Pointer to store the number of bytes received.
 * @return true if transaction ended and data is available, false otherwise.
 */
bool MCU_SPI_DRIVER_check_transaction_end(uint16_t *bytes_received);

/**
 * @brief Abort current SPI transfer and return to idle state.
 */
void MCU_SPI_DRIVER_abort_transfer(void);

/**
 * @brief Reset the SPI interface.
 *
 * @param[in] hspi Pointer to the SPI handle.
 * @return true if successfully reset, false otherwise.
 */
bool MCU_SPI_DRIVER_reset(SPI_HandleTypeDef *hspi);

/* Legacy functions - kept for compatibility but simplified */
#define MCU_SPI_DRIVER_writeread MCU_SPI_DRIVER_read
#define MCU_SPI_DRIVER_check_tx_complete() (true)

#ifdef __cplusplus
}
#endif

#endif /* __MCU_SPI_CONSOLE_H */

/**
 * @}
 */
