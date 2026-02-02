// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_at_cmd_list_mac.c
 * @author  Kinéis
 * @brief subset of AT commands concerning Kinéis Medium Acces Channel (MAC).
 *
 * @note This subset of commands are actually stubs, only present because GUI is sending such AT
 *       commands to KIM device.
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "mcu_spi_driver.h"
//#include "mgr_spi_cmd_list_user_data.h"
#include "mgr_spi_cmd_list_mac.h"
#include "mgr_spi_cmd_list.h"
#include "kns_q.h"
#include "kns_mac.h"
#include "mgr_log.h"
#include "kns_assert.h"

/* Functions -----------------------------------------------------------------*/

bool bMGR_SPI_CMD_READKMAC_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	(void)rx;  /* Unused parameter - command only returns data */
	HAL_StatusTypeDef ret = HAL_OK;
	struct KNS_MAC_prflInfo_t prfl_info;
	uint8_t *prflCfgPtr;

	KNS_MAC_getPrflInfo(&prfl_info);
	prflCfgPtr = &prfl_info.prflCfgPtr;
	tx->data[0] = prfl_info.id;
	memcpy(&(tx->data[1]), prflCfgPtr, sizeof(prfl_info) - 1);
	tx->next_req = sizeof(prfl_info);
	ret = bMGR_SPI_DRIVER_writeread();
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_WRITEKMACREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;
	tx->data[0] = rx->data[0];
	rx->next_req = CMD_WRITEKMAC_WAIT_LEN; // Only waiting profile id for the moment
	ret = bMGR_SPI_DRIVER_read();
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_WRITEKMAC_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;
	enum KNS_status_t status = KNS_STATUS_OK;
	struct KNS_MAC_appEvt_t appEvt = {
		.id = KNS_MAC_INIT,
		.init_prfl_ctxt = {
			.id =  KNS_MAC_PRFL_NONE,
			.blindCfg = {0}
		}
	};
	appEvt.init_prfl_ctxt.id = rx->data[1];

	status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
	switch (status) {
		case KNS_STATUS_QFULL:
			return bMGR_SPI_CMD_logFailedMsg(ERROR_DATA_QUEUE_FULL, tx);
		break;
		case KNS_STATUS_OK:
		break;
		default:
			return bMGR_SPI_CMD_logFailedMsg(ERROR_UNKNOWN, tx);
		break;
	}

	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_read();
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}
/**
 * @}
 */
