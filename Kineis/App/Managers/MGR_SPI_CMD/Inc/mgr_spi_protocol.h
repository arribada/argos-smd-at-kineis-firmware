/* SPDX-License-Identifier: no SPDX license */
/**
 * @file mgr_spi_protocol.h
 * @brief SPI Protocol A+ - Frame handling and integrity checking
 *
 * This module implements the A+ protocol for SPI communication with:
 * - Magic byte framing (0xAA request, 0x55 response)
 * - Sequence numbers for transaction tracking
 * - CRC-8 integrity checking
 * - Backward compatibility with legacy protocol
 *
 * @author Arribada
 */

#ifndef __MGR_SPI_PROTOCOL_H
#define __MGR_SPI_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "mcu_spi_driver.h"

/* Defines -------------------------------------------------------------------*/

/** @brief Magic bytes for frame detection */
#define SPI_MAGIC_REQUEST   0xAA  /**< Master to Slave frame marker */
#define SPI_MAGIC_RESPONSE  0x55  /**< Slave to Master frame marker */

/** @brief Protocol frame sizes */
#define SPI_FRAME_HEADER_SIZE   4   /**< Magic + Seq + Cmd/Status + Len */
#define SPI_FRAME_CRC_SIZE      1   /**< CRC-8 */
#define SPI_FRAME_MIN_SIZE      (SPI_FRAME_HEADER_SIZE + SPI_FRAME_CRC_SIZE)
#define SPI_FRAME_MAX_DATA      250 /**< Maximum data payload size */
#define SPI_FRAME_MAX_SIZE      (SPI_FRAME_MIN_SIZE + SPI_FRAME_MAX_DATA)

/** @brief Frame field offsets */
#define SPI_FRAME_MAGIC_OFFSET  0
#define SPI_FRAME_SEQ_OFFSET    1
#define SPI_FRAME_CMD_OFFSET    2   /**< For request frames */
#define SPI_FRAME_STATUS_OFFSET 2   /**< For response frames */
#define SPI_FRAME_LEN_OFFSET    3
#define SPI_FRAME_DATA_OFFSET   4

/* Enumerations --------------------------------------------------------------*/

/**
 * @brief Protocol status codes returned in response frames
 * @note Aligned with bootloader bl_prot_status_t for consistency
 */
typedef enum {
    /* Standard status codes (0x00-0x0F) - aligned with bootloader DFU */
    SPI_PROT_STATUS_OK              = 0x00, /**< Command successful */
    SPI_PROT_STATUS_ERROR           = 0x01, /**< Generic error */
    SPI_PROT_STATUS_CRC_ERROR       = 0x02, /**< CRC mismatch (DFU data) */
    SPI_PROT_STATUS_ADDR_ERROR      = 0x03, /**< Invalid address */
    SPI_PROT_STATUS_SIZE_ERROR      = 0x04, /**< Invalid size/length */
    SPI_PROT_STATUS_FLASH_ERROR     = 0x05, /**< Flash operation failed */
    SPI_PROT_STATUS_BUSY            = 0x06, /**< Device busy, retry later */
    SPI_PROT_STATUS_INVALID_CMD     = 0x07, /**< Unknown command */
    SPI_PROT_STATUS_TIMEOUT         = 0x08, /**< Operation timeout */
    SPI_PROT_STATUS_NOT_READY       = 0x09, /**< Prerequisite not met */
    SPI_PROT_STATUS_INVALID_HEADER  = 0x0A, /**< Invalid header */
    SPI_PROT_STATUS_VERIFY_ERROR    = 0x0B, /**< Verification failed */

    /* Protocol-specific errors (0x10+) */
    SPI_PROT_STATUS_FRAME_CRC_ERROR = 0x10, /**< Frame CRC mismatch */
    SPI_PROT_STATUS_SEQ_ERROR       = 0x11, /**< Sequence number error */
    SPI_PROT_STATUS_FRAME_ERROR     = 0x12, /**< Malformed frame */
} SpiProtocolStatus;

/**
 * @brief Protocol mode detection
 */
typedef enum {
    SPI_PROTOCOL_UNKNOWN = 0,   /**< Not yet determined */
    SPI_PROTOCOL_LEGACY,        /**< Legacy protocol (direct command) */
    SPI_PROTOCOL_A_PLUS,        /**< New A+ protocol with framing */
} SpiProtocolMode;

/**
 * @brief Protocol parser states
 */
typedef enum {
    SPI_PROT_IDLE = 0,      /**< Waiting for first byte */
    SPI_PROT_RX_HEADER,     /**< Receiving header (SEQ, CMD, LEN) */
    SPI_PROT_RX_DATA,       /**< Receiving data payload */
    SPI_PROT_RX_CRC,        /**< Receiving CRC byte */
    SPI_PROT_FRAME_READY,   /**< Complete frame received */
    SPI_PROT_ERROR,         /**< Protocol error occurred */
} SpiProtocolParseState;

/* Types ---------------------------------------------------------------------*/

/**
 * @brief Parsed request frame structure
 */
typedef struct {
    uint8_t sequence;           /**< Sequence number */
    uint8_t command;            /**< Command code */
    uint8_t data_len;           /**< Data length */
    uint8_t data[SPI_FRAME_MAX_DATA]; /**< Data payload */
    uint8_t received_crc;       /**< CRC received in frame */
    uint8_t computed_crc;       /**< CRC computed locally */
    bool crc_valid;             /**< True if CRCs match */
} SpiRequestFrame;

/**
 * @brief Response frame builder structure
 */
typedef struct {
    uint8_t sequence;           /**< Echo of request sequence */
    SpiProtocolStatus status;   /**< Response status */
    uint8_t data_len;           /**< Data length */
    uint8_t data[SPI_FRAME_MAX_DATA]; /**< Response data */
} SpiResponseFrame;

/**
 * @brief Protocol context structure
 */
typedef struct {
    SpiProtocolMode mode;           /**< Current protocol mode */
    SpiProtocolParseState state;    /**< Parser state */
    uint8_t last_sequence;          /**< Last received sequence number */
    uint8_t rx_index;               /**< Current RX buffer index */
    SpiRequestFrame request;        /**< Parsed request */
    SpiResponseFrame response;      /**< Response to send */
    uint32_t crc_error_count;       /**< CRC error counter */
    uint32_t seq_error_count;       /**< Sequence error counter */
    uint32_t frame_count;           /**< Total frames processed */
} SpiProtocolContext;

/* Extern Variables ----------------------------------------------------------*/
extern SpiProtocolContext spi_protocol_ctx;

/* Function Prototypes -------------------------------------------------------*/

/**
 * @brief Initialize the protocol context
 */
void MGR_SPI_PROTOCOL_init(void);

/**
 * @brief Calculate CRC-8 (CCITT polynomial 0x07)
 *
 * @param[in] data Pointer to data buffer
 * @param[in] length Number of bytes to process
 * @return Computed CRC-8 value
 */
uint8_t MGR_SPI_PROTOCOL_crc8(const uint8_t *data, uint16_t length);

/**
 * @brief Process received byte and detect protocol mode
 *
 * @param[in] byte Received byte
 * @return true if a complete frame is ready for processing
 */
bool MGR_SPI_PROTOCOL_rx_byte(uint8_t byte);

/**
 * @brief Check if protocol detected legacy mode
 *
 * @return true if legacy protocol detected
 */
bool MGR_SPI_PROTOCOL_is_legacy(void);

/**
 * @brief Get the detected command (for legacy mode)
 *
 * @return Command code
 */
uint8_t MGR_SPI_PROTOCOL_get_legacy_cmd(void);

/**
 * @brief Get the parsed request frame (for A+ mode)
 *
 * @return Pointer to parsed request frame
 */
SpiRequestFrame* MGR_SPI_PROTOCOL_get_request(void);

/**
 * @brief Build response frame into TX buffer
 *
 * @param[out] tx_buf Pointer to TX buffer
 * @param[in] sequence Sequence number to echo
 * @param[in] status Response status
 * @param[in] data Response data (can be NULL if len=0)
 * @param[in] data_len Response data length
 * @return Total frame size (including header and CRC)
 */
uint16_t MGR_SPI_PROTOCOL_build_response(uint8_t *tx_buf, uint8_t sequence,
                                          SpiProtocolStatus status,
                                          const uint8_t *data, uint8_t data_len);

/**
 * @brief Build error response frame
 *
 * @param[out] tx_buf Pointer to TX buffer
 * @param[in] sequence Sequence number to echo
 * @param[in] status Error status
 * @return Total frame size
 */
uint16_t MGR_SPI_PROTOCOL_build_error_response(uint8_t *tx_buf, uint8_t sequence,
                                                SpiProtocolStatus status);

/**
 * @brief Reset protocol parser state
 */
void MGR_SPI_PROTOCOL_reset(void);

/**
 * @brief Process a complete received buffer
 *
 * This function parses an entire received buffer to extract the frame.
 * It handles both legacy (single command byte) and A+ protocol frames.
 *
 * @param[in] data Pointer to received data buffer
 * @param[in] length Number of bytes in buffer
 * @return true if a complete valid frame was parsed, false otherwise
 */
bool MGR_SPI_PROTOCOL_process_buffer(const uint8_t *data, uint16_t length);

/**
 * @brief Get protocol statistics string for debugging
 *
 * @param[out] buf Output buffer
 * @param[in] buf_size Buffer size
 * @return Number of characters written
 */
int MGR_SPI_PROTOCOL_get_stats(char *buf, uint16_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* __MGR_SPI_PROTOCOL_H */
