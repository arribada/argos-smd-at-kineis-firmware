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
#include "mgr_spi_protocol.h"
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
	(void)rx;  /* Unused in pipelined protocol */
	HAL_StatusTypeDef ret = HAL_OK;

	/* Send ACK response for pipelined protocol */
	tx->data[0] = 1;  /* OK indicator */
	tx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

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
	/* Copy the profile config context (mirror of AT+KMAC) when the master sent
	 * it in the A+ payload: [cmd][id][7 ctx bytes]. Three gates so every
	 * existing flow stays byte-identical:
	 *  - BLIND id only: BASIC/NONE keep blindCfg={0} exactly as before.
	 *  - A+ frames only: in legacy mode rx->size is the RAW DMA clock count
	 *    (mgr_spi_cmd.c SPICMD_IDLE), so a master clocking fixed-size
	 *    transactions would pass the length check and bus padding would be
	 *    copied. A+ size is exact (data_len+1) and CRC-checked.
	 *  - Length: the 7 ctx bytes must actually be present. */
	if (appEvt.init_prfl_ctxt.id == KNS_MAC_PRFL_BLIND &&
	    !MGR_SPI_PROTOCOL_is_legacy() &&
	    rx->size >= 2u + sizeof(struct KNS_MAC_BLIND_usrCfg_t))
		memcpy(&appEvt.init_prfl_ctxt.prflCfgPtr, &rx->data[2],
		       sizeof(struct KNS_MAC_BLIND_usrCfg_t));

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

	/* Send success response */
	tx->data[0] = 1;
	tx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

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
