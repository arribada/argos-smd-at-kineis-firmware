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

/* Public functions ----------------------------------------------------------*/

bool bMGR_SPI_CMD_logFailedMsg(enum ERROR_RETURN_T eErrorType, SPI_Buffer *tx)
{
    MGR_LOG_DEBUG("+ERROR=%i\r\n", eErrorType);

    /* Check if we're in A+ protocol mode */
    if (!MGR_SPI_PROTOCOL_is_legacy() && spi_protocol_ctx.mode == SPI_PROTOCOL_A_PLUS) {
        /* For A+ protocol, send error response using protocol frame */
        SpiRequestFrame *req = MGR_SPI_PROTOCOL_get_request();

        /* Build error response with error code as payload */
        uint8_t error_data[3];
        error_data[0] = MAC_ERROR;
        error_data[1] = (uint8_t)((eErrorType >> 8) & 0xFF);
        error_data[2] = (uint8_t)(eErrorType & 0xFF);

        uint16_t frame_len = MGR_SPI_PROTOCOL_build_response(
            tx->data,
            req->sequence,
            SPI_PROT_STATUS_ERROR,  /* Status = ERROR */
            error_data,
            3
        );
        tx->next_req = frame_len;
        MGR_LOG_DEBUG("%s:: A+ error response: SEQ=%u frame_len=%u\r\n",
                      __func__, req->sequence, frame_len);
    } else {
        /* Legacy protocol: Send error code directly */
        tx->data[0] = MAC_ERROR;
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
        uint8_t payload_len = (uint8_t)txBuf.next_req;
        uint8_t payload_backup[SPI_FRAME_MAX_DATA];

        /* Backup payload data (handlers wrote to txBuf.data[0..]) */
        if (payload_len > 0 && payload_len <= SPI_FRAME_MAX_DATA) {
            memcpy(payload_backup, txBuf.data, payload_len);
        }

        /* Build A+ protocol response frame */
        uint16_t frame_len = MGR_SPI_PROTOCOL_build_response(
            txBuf.data,           /* Output buffer */
            req->sequence,        /* Echo request sequence number */
            SPI_PROT_STATUS_OK,   /* Status = OK */
            payload_backup,       /* Payload data */
            payload_len           /* Payload length */
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
