/**
 * @file    bl_spi_protocol.h
 * @brief   SPI Protocol A+ for bootloader DFU - Compatible with application protocol
 * @date    2025
 *
 * This module implements the A+ protocol for SPI communication with:
 * - Magic byte framing (0xAA request, 0x55 response)
 * - Sequence numbers for transaction tracking
 * - CRC-8 integrity checking (CCITT polynomial 0x07)
 * - Backward compatibility with legacy protocol (direct command byte)
 *
 * Frame structure:
 *   Request:  [0xAA][SEQ][CMD][LEN][DATA...][CRC8]
 *   Response: [0x55][SEQ][STATUS][LEN][DATA...][CRC8]
 */

#ifndef BL_SPI_PROTOCOL_H
#define BL_SPI_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "bl_config.h"

/*******************************************************************************
 * PROTOCOL CONSTANTS
 ******************************************************************************/

/** Magic bytes for frame detection */
#define BL_SPI_MAGIC_REQUEST    0xAA    /**< Master to Slave frame marker */
#define BL_SPI_MAGIC_RESPONSE   0x55    /**< Slave to Master frame marker */

/** Protocol frame sizes */
#define BL_SPI_FRAME_HEADER_SIZE    4   /**< Magic + Seq + Cmd/Status + Len */
#define BL_SPI_FRAME_CRC_SIZE       1   /**< CRC-8 */
#define BL_SPI_FRAME_MIN_SIZE       (BL_SPI_FRAME_HEADER_SIZE + BL_SPI_FRAME_CRC_SIZE)
#define BL_SPI_FRAME_MAX_DATA       255 /**< Maximum data payload size (1-byte LEN field) */
#define BL_SPI_FRAME_MAX_SIZE       (BL_SPI_FRAME_MIN_SIZE + BL_SPI_FRAME_MAX_DATA)

/** Frame field offsets */
#define BL_SPI_FRAME_MAGIC_OFFSET   0
#define BL_SPI_FRAME_SEQ_OFFSET     1
#define BL_SPI_FRAME_CMD_OFFSET     2   /**< For request frames */
#define BL_SPI_FRAME_STATUS_OFFSET  2   /**< For response frames */
#define BL_SPI_FRAME_LEN_OFFSET     3
#define BL_SPI_FRAME_DATA_OFFSET    4

/*******************************************************************************
 * PROTOCOL STATUS CODES (Extended from dfu_response_t)
 ******************************************************************************/

/**
 * @brief Protocol status codes for A+ responses
 * @note Values 0x00-0x0F match dfu_response_t for compatibility
 */
typedef enum {
    /* Standard DFU responses */
    BL_PROT_OK              = 0x00, /**< Command successful */
    BL_PROT_ERROR           = 0x01, /**< Generic error */
    BL_PROT_CRC_ERROR       = 0x02, /**< CRC mismatch (DFU or frame) */
    BL_PROT_ADDR_ERROR      = 0x03, /**< Invalid address */
    BL_PROT_SIZE_ERROR      = 0x04, /**< Invalid size */
    BL_PROT_FLASH_ERROR     = 0x05, /**< Flash operation failed */
    BL_PROT_BUSY            = 0x06, /**< Device busy, retry later */
    BL_PROT_INVALID_CMD     = 0x07, /**< Unknown command */
    BL_PROT_TIMEOUT         = 0x08, /**< Operation timeout */
    BL_PROT_NOT_READY       = 0x09, /**< Prerequisite not met */
    BL_PROT_INVALID_HEADER  = 0x0A, /**< Invalid app header */
    BL_PROT_VERIFY_ERROR    = 0x0B, /**< Verification failed */

    /* Protocol-specific errors */
    BL_PROT_FRAME_CRC_ERROR = 0x10, /**< Frame CRC mismatch */
    BL_PROT_SEQ_ERROR       = 0x11, /**< Sequence number error */
    BL_PROT_FRAME_ERROR     = 0x12, /**< Malformed frame */
} bl_prot_status_t;

/*******************************************************************************
 * PROTOCOL MODE
 ******************************************************************************/

/**
 * @brief Protocol mode detection
 */
typedef enum {
    BL_PROT_MODE_UNKNOWN = 0,   /**< Not yet determined */
    BL_PROT_MODE_LEGACY,        /**< Legacy protocol (direct command byte) */
    BL_PROT_MODE_A_PLUS,        /**< A+ protocol with framing */
} bl_prot_mode_t;

/*******************************************************************************
 * DFU EXTENDED STATUS (for GET_STATUS command)
 ******************************************************************************/

/**
 * @brief DFU operation state
 */
typedef enum {
    BL_DFU_STATE_IDLE = 0,      /**< No operation in progress */
    BL_DFU_STATE_ERASING,       /**< Erase in progress */
    BL_DFU_STATE_WRITING,       /**< Write in progress */
    BL_DFU_STATE_VERIFYING,     /**< CRC verification in progress */
    BL_DFU_STATE_READY,         /**< Ready for next operation */
    BL_DFU_STATE_COMPLETE,      /**< DFU complete, ready to jump */
    BL_DFU_STATE_ERROR,         /**< Error occurred */
} bl_dfu_op_state_t;

/**
 * @brief Extended status structure for GET_STATUS
 */
typedef struct __attribute__((packed)) {
    uint8_t protocol_version;   /**< Protocol version (0x01 for A+) */
    uint8_t bootloader_state;   /**< bl_state_t */
    uint8_t dfu_op_state;       /**< bl_dfu_op_state_t - current operation */
    uint8_t last_error;         /**< Last error code */
    uint8_t session_active;     /**< DFU session active flag */
    uint8_t erase_done;         /**< Erase completed flag */
    uint8_t verify_passed;      /**< CRC verification passed */
    uint8_t reserved;           /**< Reserved for alignment */
    uint32_t received_bytes;    /**< Total bytes received */
    uint32_t write_address;     /**< Current write address */
    uint32_t expected_crc;      /**< Expected CRC (from VERIFY cmd) */
    uint32_t calculated_crc;    /**< Calculated CRC so far */
    uint32_t frame_count;       /**< Total frames processed */
    uint32_t crc_error_count;   /**< Frame CRC errors */
} bl_extended_status_t;

/*******************************************************************************
 * PARSED FRAME STRUCTURES
 ******************************************************************************/

/**
 * @brief Parsed request frame
 */
typedef struct {
    uint8_t sequence;           /**< Sequence number */
    uint8_t command;            /**< Command code */
    uint8_t data_len;           /**< Data length */
    uint8_t data[BL_SPI_FRAME_MAX_DATA]; /**< Data payload */
    uint8_t received_crc;       /**< CRC received in frame */
    uint8_t computed_crc;       /**< CRC computed locally */
    bool crc_valid;             /**< True if CRCs match */
} bl_spi_request_t;

/**
 * @brief Protocol context
 */
typedef struct {
    bl_prot_mode_t mode;        /**< Current protocol mode */
    uint8_t last_sequence;      /**< Last received sequence */
    bl_spi_request_t request;   /**< Parsed request */
    bl_dfu_op_state_t op_state; /**< Current DFU operation state */
    uint32_t frame_count;       /**< Total frames processed */
    uint32_t crc_error_count;   /**< CRC error counter */
    uint32_t seq_error_count;   /**< Sequence error counter */
} bl_spi_protocol_ctx_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Initialize protocol context
 */
void bl_spi_protocol_init(void);

/**
 * @brief Reset protocol state (for new transaction)
 */
void bl_spi_protocol_reset(void);

/**
 * @brief Calculate CRC-8 (CCITT polynomial 0x07)
 * @param data Pointer to data
 * @param length Number of bytes
 * @return Computed CRC-8
 */
uint8_t bl_spi_protocol_crc8(const uint8_t *data, uint16_t length);

/**
 * @brief Process received buffer and detect protocol
 * @param data Received data buffer
 * @param length Buffer length
 * @return true if valid frame parsed
 */
bool bl_spi_protocol_process(const uint8_t *data, uint16_t length);

/**
 * @brief Check if legacy mode detected
 * @return true if legacy protocol
 */
bool bl_spi_protocol_is_legacy(void);

/**
 * @brief Get parsed request (for A+ mode)
 * @return Pointer to parsed request
 */
bl_spi_request_t* bl_spi_protocol_get_request(void);

/**
 * @brief Get command code (works for both legacy and A+)
 * @return Command code
 */
uint8_t bl_spi_protocol_get_command(void);

/**
 * @brief Get protocol context
 * @return Pointer to protocol context
 */
bl_spi_protocol_ctx_t* bl_spi_protocol_get_ctx(void);

/**
 * @brief Build A+ response frame
 * @param tx_buf Output buffer
 * @param sequence Sequence number to echo
 * @param status Response status
 * @param data Response data (can be NULL)
 * @param data_len Data length
 * @return Total frame size
 */
uint16_t bl_spi_protocol_build_response(uint8_t *tx_buf, uint8_t sequence,
                                         bl_prot_status_t status,
                                         const uint8_t *data, uint8_t data_len);

/**
 * @brief Build legacy response (status + data, no framing)
 * @param tx_buf Output buffer
 * @param status Response status
 * @param data Response data (can be NULL)
 * @param data_len Data length
 * @return Total size
 */
uint16_t bl_spi_protocol_build_legacy_response(uint8_t *tx_buf,
                                                bl_prot_status_t status,
                                                const uint8_t *data, uint8_t data_len);

/**
 * @brief Set DFU operation state
 * @param state New operation state
 */
void bl_spi_protocol_set_op_state(bl_dfu_op_state_t state);

/**
 * @brief Get DFU operation state
 * @return Current operation state
 */
bl_dfu_op_state_t bl_spi_protocol_get_op_state(void);

/**
 * @brief Build extended status response
 * @param status Output buffer for status structure
 */
void bl_spi_protocol_build_extended_status(bl_extended_status_t *status);

/**
 * @brief Check if command is valid DFU command (for legacy detection)
 * @param cmd Command byte
 * @return true if valid DFU command
 */
bool bl_spi_protocol_is_valid_dfu_cmd(uint8_t cmd);

#endif /* BL_SPI_PROTOCOL_H */
