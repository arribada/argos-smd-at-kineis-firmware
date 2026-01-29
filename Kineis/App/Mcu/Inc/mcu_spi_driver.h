/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    mcu_spi_console.h
 * @brief   MCU wrapper for SPI console.
 *
 * This console is used to communicate SPI commands and responses with the device.
 * Depending on your application needs, it could be based on a UART link, I2C, or other interfaces.
 *
 * **Reception**
 *
 * On the reception side, this wrapper should handle incoming data as a stream of ASCII characters.
 * The client of this wrapper must register a callback that is invoked each time new data is received.
 *
 * **Transmission**
 *
 * On the transmission side, it is possible to send formatted strings similarly to printf.
 * A dedicated API is also available to send the content of a binary buffer by converting binary data
 * into ASCII printable strings.
 *
 * @note The client's callback function will be invoked in interrupt context and must therefore execute quickly.
 *
 * @author Arribada
 */

/**
 * @addtogroup MCU_APP_WRAPPERS
 * @brief MCU wrappers used by the Kineis Application example.
 *
 * One must implement these APIs according to the microcontroller and available platform resources.
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
 * @brief Enumeration of SPI console states.
 */
typedef enum {
    SPICMD_UNKNOWN,           /**< Unknown state */
    SPICMD_INIT,              /**< SPI console initialization state */
    SPICMD_IDLE,              /**< Idle state, waiting for a TX request */
    SPICMD_PROCESS_CMD,       /**< Processing an incoming command */
    SPICMD_WAITING_RX,        /**< Waiting for RX data of a specified length */
    SPICMD_WAITING_TX,        /**< Waiting for TX data to be sent */
    SPICMD_WAITING_MAC_EVT,   /**< Waiting for a MAC event (e.g., TX done, RX done) */
    SPICMD_ERROR              /**< Error state */
} SpiState;

extern volatile SpiState spiState;         /**< Current state of the SPI console */
extern volatile uint32_t startTickTimeout; /**< Timeout tick value for SPI operations */

/* Functions prototypes ------------------------------------------------------*/

/**
 * @brief Start the SPI console for command reception.
 *
 * This function registers a client's callback used to process the incoming SPI data stream.
 * The callback is invoked each time new data is received.
 *
 * @attention The callback may be executed in an interrupt context, so it must execute quickly.
 *
 * The format of the client's callback is:
 * @code
 * int8_t (*rx_spi_evt_cb)(SPI_Buffer *rx, SPI_Buffer *tx);
 * @endcode
 * The callback should process the received data in the SPI_Buffer and update the buffer as necessary.
 *
 * @param[in] context Pointer to a context object (e.g., an SPI handle) that may be used to configure the console.
 * @param[in] rx_spi_evt_cb Pointer to the callback function invoked on each SPI reception event.
 *
 * @return true if the SPI console is successfully started, false otherwise.
 */
bool MCU_SPI_DRIVER_register(void *context,
		int8_t (*rx_spi_evt_cb)(SPI_Buffer *rx, SPI_Buffer *tx));

/**
 * @brief Send an SPI command response to the console.
 *
 * This function sends a formatted string as an SPI command response. It functions similarly to printf,
 * and may implement only a subset of printf functionality.
 *
 * @note This function can be redirected to a standard printf if available in your application.
 *
 * @param[in] format Format string (see printf documentation for details).
 * @param[in] ... Additional arguments corresponding to the format.
 */
void MCU_SPI_DRIVER_send(const char *format, ...);

/**
 * @brief Perform an SPI read operation.
 *
 * This function initiates a read operation on the SPI interface.
 *
 * @return HAL status of the read operation.
 */
HAL_StatusTypeDef MCU_SPI_DRIVER_read();

/**
 * @brief Perform a simultaneous SPI write and read operation.
 *
 * This function initiates a write/read operation on the SPI interface.
 *
 * @return HAL status of the write/read operation.
 */
HAL_StatusTypeDef MCU_SPI_DRIVER_writeread();

/**
 * @brief Send the content of a binary data buffer as an SPI command response.
 *
 * This function converts the binary data into an ASCII representation and sends it via the SPI console.
 *
 * @param[in] pu8_inDataBuff Pointer to the binary data buffer.
 * @param[in] u16_dataLenBit Length of the data buffer in bits.
 */
void MCU_SPI_DRIVER_send_dataBuf(uint8_t *pu8_inDataBuff, uint16_t u16_dataLenBit);

/**
 * @brief Reset the SPI interface.
 *
 * This function resets the SPI interface using the provided SPI handle.
 *
 * @param[in] hspi Pointer to the SPI handle.
 *
 * @return true if the SPI interface is successfully reset, false otherwise.
 */
bool MCU_SPI_DRIVER_reset(SPI_HandleTypeDef *hspi);

/**
 * @brief Check if SPI transaction has ended by detecting idle state.
 *
 * This function detects when a SPI transaction has completed by monitoring
 * the BSY flag. When the master finishes clocking, SPI becomes idle.
 *
 * @param[out] bytes_received Pointer to store the number of bytes received.
 * @return true if transaction ended and data is available, false otherwise.
 */
bool MCU_SPI_DRIVER_check_transaction_end(uint16_t *bytes_received);

/**
 * @brief Abort current SPI transfer and return to idle state.
 *
 * This should be called when a transaction has ended to properly terminate
 * the HAL transfer before processing data.
 */
void MCU_SPI_DRIVER_abort_transfer(void);

/**
 * @brief Check if TX transaction has completed for two-transaction protocol.
 *
 * In the two-transaction protocol, after the error callback re-arms for
 * transaction 2, this function polls for completion. When the master clocks
 * out the response, this returns true.
 *
 * @return true if TX transaction completed (response was sent), false if still waiting.
 */
bool MCU_SPI_DRIVER_check_tx_complete(void);

#ifdef __cplusplus
}
#endif

#endif /* __MCU_SPI_CONSOLE_H */

/**
 * @}
 */
