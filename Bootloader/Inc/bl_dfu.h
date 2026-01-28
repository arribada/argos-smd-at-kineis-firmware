/**
 * @file    bl_dfu.h
 * @brief   DFU protocol handler for bootloader
 * @date    2025
 */

#ifndef BL_DFU_H
#define BL_DFU_H

#include <stdint.h>
#include <stdbool.h>
#include "bl_config.h"

/**
 * @brief DFU context structure
 */
typedef struct {
    uint32_t total_size;        /**< Total firmware size to receive */
    uint32_t received_size;     /**< Bytes received so far */
    uint32_t write_addr;        /**< Current write address */
    uint32_t expected_crc;      /**< Expected CRC32 from host */
    uint32_t calculated_crc;    /**< Calculated CRC32 */
    bool     session_active;    /**< DFU session is active (ERASE was called) */
    bool     erase_done;        /**< Flash has been erased */
    bool     verify_passed;     /**< CRC verification passed */
    bool     header_written;    /**< Header has been written */
    dfu_response_t last_error;  /**< Last error code */
} dfu_context_t;

/**
 * @brief Initialize DFU handler
 */
void bl_dfu_init(void);

/**
 * @brief Reset DFU context for new transfer
 */
void bl_dfu_reset(void);

/**
 * @brief Process a DFU command
 * @param cmd Command ID
 * @param data Command data payload
 * @param data_len Data length
 * @param response Response buffer
 * @param response_len Pointer to response length (in: max, out: actual)
 * @return Response code
 */
dfu_response_t bl_dfu_process_cmd(dfu_cmd_t cmd, const uint8_t* data,
                                   uint16_t data_len, uint8_t* response,
                                   uint16_t* response_len);

/**
 * @brief Handle PING command
 * @param response Response buffer
 * @param response_len Response length
 * @return Response code
 */
dfu_response_t bl_dfu_cmd_ping(uint8_t* response, uint16_t* response_len);

/**
 * @brief Handle GET_INFO command
 * @param response Response buffer
 * @param response_len Response length
 * @return Response code
 */
dfu_response_t bl_dfu_cmd_get_info(uint8_t* response, uint16_t* response_len);

/**
 * @brief Handle ERASE command
 * @return Response code
 */
dfu_response_t bl_dfu_cmd_erase(void);

/**
 * @brief Handle WRITE command
 * @param data Write data (address + payload)
 * @param data_len Data length
 * @return Response code
 */
dfu_response_t bl_dfu_cmd_write(const uint8_t* data, uint16_t data_len);

/**
 * @brief Handle READ command
 * @param data Read request (address + length)
 * @param data_len Request length
 * @param response Response buffer
 * @param response_len Response length
 * @return Response code
 */
dfu_response_t bl_dfu_cmd_read(const uint8_t* data, uint16_t data_len,
                               uint8_t* response, uint16_t* response_len);

/**
 * @brief Handle VERIFY command
 * @param data Expected CRC32
 * @param data_len Data length
 * @return Response code
 */
dfu_response_t bl_dfu_cmd_verify(const uint8_t* data, uint16_t data_len);

/**
 * @brief Handle GET_STATUS command
 * @param response Response buffer
 * @param response_len Response length
 * @return Response code
 */
dfu_response_t bl_dfu_cmd_get_status(uint8_t* response, uint16_t* response_len);

/**
 * @brief Handle SET_HEADER command
 * @param data Header data
 * @param data_len Data length
 * @return Response code
 */
dfu_response_t bl_dfu_cmd_set_header(const uint8_t* data, uint16_t data_len);

/**
 * @brief Handle ABORT command
 * @return Response code
 */
dfu_response_t bl_dfu_cmd_abort(void);

/**
 * @brief Get current DFU context
 * @return Pointer to DFU context
 */
const dfu_context_t* bl_dfu_get_context(void);

/**
 * @brief Check if DFU transfer is complete
 * @return true if complete and verified
 */
bool bl_dfu_is_complete(void);

/**
 * @brief Check if jump to application is allowed
 * @return true if CRC verification passed and session is complete
 */
bool bl_dfu_can_jump(void);

/**
 * @brief Check if a DFU session is active
 * @return true if ERASE was called and session not aborted
 */
bool bl_dfu_session_active(void);

#endif /* BL_DFU_H */
