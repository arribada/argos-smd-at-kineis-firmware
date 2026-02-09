/**
 * @file mgr_spi_cmd_common.h
 * @brief Header file for the common part of the SPI command manager.
 *
 * This file includes the necessary declarations and prototypes used by the AT command manager to interact with the SPI driver.
 * It defines common error handling, logging functions, and low-level SPI operations.
 *
 * @author
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MGR_SPI_CMD_COMMON_H
#define __MGR_SPI_CMD_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "strutil_lib.h"
#include "kineis_sw_conf.h" // get ERROR_RETURN_T types
#include "mcu_spi_driver.h"
#include "kns_mac.h"

/* Defines ------------------------------------------------------------------*/

/**
 * @brief Enumeration of MAC statuses.
 *
 * This enumeration defines various status codes related to the MAC (Media Access Control) layer operations.
 * These are returned via CMD_MAC_STATUS (0x03) and indicate async operation results.
 */
typedef enum {
    MAC_UNKNOWN         = 0x00, /**< Unknown MAC status (initial state). */
    MAC_OK              = 0x01, /**< MAC ready, no pending operation. */
    MAC_TX_DONE         = 0x02, /**< Transmission completed successfully. */
    MAC_TX_SIZE_ERROR   = 0x03, /**< Error due to incorrect transmission size. */
    MAC_TXACK_DONE      = 0x04, /**< Transmission with acknowledgment completed. */
    MAC_TX_TIMEOUT      = 0x05, /**< Transmission timed out (no satellite). */
    MAC_TXACK_TIMEOUT   = 0x06, /**< Acknowledgment timeout after transmission. */
    MAC_RX_ERROR        = 0x07, /**< Error during reception. */
    MAC_RX_TIMEOUT      = 0x08, /**< Reception timed out. */
    MAC_ERROR           = 0x09, /**< General MAC error. */
    /* Extended status codes for additional async events */
    MAC_TX_IN_PROGRESS  = 0x0A, /**< TX operation in progress (queued). */
    MAC_RX_RECEIVED     = 0x0B, /**< Downlink frame received. */
    MAC_SAT_DETECTED    = 0x0C, /**< Satellite detected. */
    MAC_SAT_LOST        = 0x0D, /**< Satellite lost. */
    MAC_RF_ABORTED      = 0x0E, /**< RF operation aborted. */
} MACStatus;

extern volatile MACStatus macStatus;

/* Functions prototypes -----------------------------------------------------*/

/**
 * @brief Log a failed message to the debug interface.
 *
 * This function logs an error message corresponding to the provided error type using the SPI transmit buffer.
 *
 * @param[in] eErrorType The error type as defined by the ERROR_RETURN_T enumeration.
 * @param[out] tx Pointer to the SPI transmit buffer used to send the log message.
 *
 * @return Always returns false to indicate an error condition.
 */
bool bMGR_SPI_CMD_logFailedMsg(enum ERROR_RETURN_T eErrorType, SPI_Buffer *tx);

/**
 * @brief Read data using the SPI_Read_IT command.
 *
 * This function initiates a non-blocking read operation on the SPI interface using the SPI_Read_IT command.
 * It waits for a specific amount of data defined by the SPI_Buffer size.
 *
 * @return The byte read from the SPI interface. In case of an error, the returned value may indicate an error condition.
 */
uint8_t bMGR_SPI_DRIVER_read();

/**
 * @brief Perform a simultaneous write and read operation using the TransmitRead_IT command.
 *
 * This function initiates a non-blocking write/read operation on the SPI interface using the TransmitRead_IT command.
 * It transmits data while simultaneously reading the response, with the operation size defined by the SPI_Buffer.
 *
 * @return The byte read from the SPI interface after performing the write operation. In case of an error, the returned value may indicate an error condition.
 */
uint8_t bMGR_SPI_DRIVER_writeread();

#ifdef __cplusplus
}
#endif

#endif /* __MGR_SPI_CMD_COMMON_H */

/**
 * @}
 */
