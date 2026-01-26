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
#include "mgr_spi_cmd_common.h"
#include "user_data.h"
#include "kns_mac.h"
#include "mgr_log.h"

/* Public functions ----------------------------------------------------------*/

bool bMGR_SPI_CMD_logFailedMsg(enum ERROR_RETURN_T eErrorType, SPI_Buffer *tx)
{
	MGR_LOG_DEBUG("+ERROR=%i\r\n", eErrorType);
	/* Send error code to SPI master:
	 * Byte 0: MAC_ERROR (0x09) to indicate error
	 * Byte 1: High byte of error code
	 * Byte 2: Low byte of error code
	 */
	tx->data[0] = MAC_ERROR;
	tx->data[1] = (uint8_t)((eErrorType >> 8) & 0xFF);
	tx->data[2] = (uint8_t)(eErrorType & 0xFF);
	tx->next_req = 3;
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
	ret = MCU_SPI_DRIVER_writeread();
	return ret;

}


/**
 * @}
 */
