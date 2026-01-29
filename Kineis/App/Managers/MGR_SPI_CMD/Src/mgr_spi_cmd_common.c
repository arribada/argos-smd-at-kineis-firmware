// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_at_cmd_common.c
 * @author Kineis
 * @brief common part of the AT cmd manager (logging, AT cmd response api)
 */

/**
 * @addtogroup MGR_AT_CMD
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

	MCU_SPI_DRIVER_writeread();
	return false;
}
uint8_t bMGR_SPI_DRIVER_read()
{
	uint8_t ret = 0;
	ret = MCU_SPI_DRIVER_read();
	return ret;

}
uint8_t bMGR_SPI_DRIVER_writeread()
{
	uint8_t ret = 0;

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
		SPI_LOG_VERBOSE("%s:: A+ response: SEQ=%u LEN=%u frame_len=%u\r\n",
					  __func__, req->sequence, payload_len, frame_len);
	}

	ret = MCU_SPI_DRIVER_writeread();
	return ret;
}


/**
 * @}
 */
