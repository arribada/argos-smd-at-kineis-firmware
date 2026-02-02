// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_spi_cmd_common.c
 * @author Kineis
 * @brief Common SPI command helpers - Pipelined protocol version
 *
 * In pipelined protocol:
 * - Response is set in TX buffer for NEXT transaction
 * - No need to wait for TX completion
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "mgr_spi_cmd_common.h"
#include "mgr_spi_cmd.h"
#include "mgr_spi_protocol.h"
#include "user_data.h"
#include "kns_mac.h"
#include "mgr_log.h"

/* External SPI buffers from mcu_spi_driver.c */
extern SPI_Buffer txBuf;

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Map ERROR_RETURN_T to SpiProtocolStatus
 * @param eErrorType Application error type
 * @return Corresponding protocol status code
 */
static SpiProtocolStatus map_error_to_protocol_status(enum ERROR_RETURN_T eErrorType)
{
    /* Map common errors to protocol status codes */
    switch (eErrorType) {
        case ERROR_PARAMETER_FORMAT:
        case ERROR_MISSING_PARAMETERS:
        case ERROR_TOO_MANY_PARAMETERS:
            return SPI_PROT_STATUS_SIZE_ERROR;

        case ERROR_UNKNOWN_AT_CMD:
            return SPI_PROT_STATUS_INVALID_CMD;

        case ERROR_DATA_QUEUE_FULL:
        case ERROR_DATA_QUEUE_EMPTY:
            return SPI_PROT_STATUS_NOT_READY;

        case ERROR_INVALID_USER_DATA_LENGTH:
            return SPI_PROT_STATUS_SIZE_ERROR;

        case ERROR_UNKNOWN:
        case ERROR_INCOMPATIBLE_VALUE:
        default:
            return SPI_PROT_STATUS_ERROR;
    }
}

/* Public functions ----------------------------------------------------------*/

bool bMGR_SPI_CMD_logFailedMsg(enum ERROR_RETURN_T eErrorType, SPI_Buffer *tx)
{
    MGR_LOG_DEBUG("+ERROR=%i\r\n", eErrorType);

    /* Map application error to protocol status */
    SpiProtocolStatus prot_status = map_error_to_protocol_status(eErrorType);

    /* Check if we're in A+ protocol mode */
    if (!MGR_SPI_PROTOCOL_is_legacy() && spi_protocol_ctx.mode == SPI_PROTOCOL_A_PLUS) {
        /* For A+ protocol, send error response using protocol frame
         * Frame status contains the protocol error code
         * Payload contains the detailed application error code for debugging */
        SpiRequestFrame *req = MGR_SPI_PROTOCOL_get_request();

        /* Build error response with application error code as payload */
        uint8_t error_data[2];
        error_data[0] = (uint8_t)((eErrorType >> 8) & 0xFF);
        error_data[1] = (uint8_t)(eErrorType & 0xFF);

        uint16_t frame_len = MGR_SPI_PROTOCOL_build_response(
            tx->data,
            req->sequence,
            prot_status,    /* Protocol status (aligned with bootloader) */
            error_data,
            2               /* Only 2 bytes: ERROR_RETURN_T value */
        );
        tx->next_req = frame_len;
        MGR_LOG_DEBUG("%s:: A+ error: SEQ=%u status=0x%02X err=%u\r\n",
                      __func__, req->sequence, prot_status, eErrorType);
    } else {
        /* Legacy protocol: Send protocol status + error code */
        tx->data[0] = (uint8_t)prot_status;
        tx->data[1] = (uint8_t)((eErrorType >> 8) & 0xFF);
        tx->data[2] = (uint8_t)(eErrorType & 0xFF);
        tx->next_req = 3;
    }

    /* In pipelined protocol, response is set for next transaction */
    MCU_SPI_DRIVER_set_response(tx->data, tx->next_req);
    return false;
}

uint8_t bMGR_SPI_DRIVER_read(void)
{
    /* In pipelined protocol, this is called after processing to start next transaction.
     * The response (if any) is already set in TX buffer.
     * This function is now essentially a no-op - the main loop handles DMA re-arming. */
    return HAL_OK;
}

uint8_t bMGR_SPI_DRIVER_writeread(void)
{
    /* Check if we're in A+ protocol mode */
    if (!MGR_SPI_PROTOCOL_is_legacy() && spi_protocol_ctx.mode == SPI_PROTOCOL_A_PLUS) {
        /* Get the request frame for sequence number */
        SpiRequestFrame *req = MGR_SPI_PROTOCOL_get_request();

        /* The handler has written payload data to txBuf.data[0..next_req-1]
         * We need to wrap it in A+ protocol frame format:
         * [0x55 SEQ STATUS LEN DATA CRC]
         */
        /* CRITICAL FIX: Use uint16_t to prevent silent truncation */
        uint16_t payload_len = txBuf.next_req;
        uint8_t payload_backup[SPI_FRAME_MAX_DATA];

        /* Validate payload length before backup */
        if (payload_len > SPI_FRAME_MAX_DATA) {
            MGR_LOG_DEBUG("[ERROR] Payload too large: %u > %u\r\n",
                          payload_len, SPI_FRAME_MAX_DATA);
            payload_len = SPI_FRAME_MAX_DATA;  /* Truncate with warning */
        }

        /* Backup payload data (handlers wrote to txBuf.data[0..]) */
        if (payload_len > 0) {
            memcpy(payload_backup, txBuf.data, payload_len);
        }

        /* Build A+ protocol response frame */
        /* Note: payload_len is validated above to be <= SPI_FRAME_MAX_DATA */
        uint16_t frame_len = MGR_SPI_PROTOCOL_build_response(
            txBuf.data,           /* Output buffer */
            req->sequence,        /* Echo request sequence number */
            SPI_PROT_STATUS_OK,   /* Status = OK */
            payload_backup,       /* Payload data */
            (uint8_t)payload_len  /* Payload length (validated above) */
        );

        txBuf.next_req = frame_len;
        SPI_LOG_VERBOSE("RSP BUILT: A+ seq=%u len=%u [%02X %02X %02X %02X %02X %02X]\r\n",
                      req->sequence, frame_len,
                      txBuf.data[0], txBuf.data[1], txBuf.data[2],
                      txBuf.data[3], txBuf.data[4], txBuf.data[5]);
    } else {
        SPI_LOG_VERBOSE("RSP BUILT: Legacy len=%u data[0]=0x%02X\r\n",
                      txBuf.next_req, txBuf.data[0]);
    }

    /* In pipelined protocol, set response for next transaction.
     * The caller (MGR_SPI_CMD_process_cmd) will call MCU_SPI_DRIVER_set_response()
     * after we return. But we can also call it here to make sure it's set.
     * Actually, the response is already in txBuf.data with length txBuf.next_req.
     * The main loop's state machine will handle setting it. */

    return HAL_OK;
}

/**
 * @}
 */
