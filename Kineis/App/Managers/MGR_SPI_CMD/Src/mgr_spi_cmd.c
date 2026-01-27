// SPDX-License-Identifier: no SPDX license
/**
 * @file   mgr_spi_cmd.c
 * @author Arribada
 * @brief  TBD
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */


/* Includes -------------------------------------------------------------------------------------*/
#include <string.h>

#include "kns_types.h"
#include "mgr_spi_cmd.h"
// #include "mgr_at_cmd_list.h"
#include "kineis_sw_conf.h"
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"
#include "mcu_nvm.h"
#include "mgr_at_cmd_list_user_data.h"
#include "kns_mac.h"
#include "kns_q.h"
#include "user_data.h"
#include "mcu_spi_driver.h"
#include "mgr_spi_cmd_common.h"
#include "mgr_spi_cmd_list_general.h"
#include "mgr_spi_cmd_list.h"
#include "kns_cfg.h"
#include "mcu_misc.h"
/* Defines --------------------------------------------------------------------------------------*/


/* Structure Declaration ------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------*/
SpiState spiState = SPICMD_INIT;
MACStatus macStatus = MAC_OK;
CmdValue cmdInProgress = CMD_NONE;

/* Private functions ----------------------------------------------------------------------------*/
/**
 * @brief
 *
 * @attention This fct may be called from ISR context
 *
 * @note
 *
 * @param[in,out] pu8_RxBuffer pointer to start of RX buffer
 * @param[in,out] pi16_nbRxValidChar number of valid charecters in RX buffer
 *
 * @retval Return number of byt to read
 */
static int8_t MGR_SPI_CMD_parseStreamCb(SPI_Buffer *rx, SPI_Buffer *tx)
{
	if (rx->size > 0)
	{
		cmdInProgress = rx->data[0];

		// Store SPI state before to handle cmd in TX in case we want to read current spi state
		if ((cmdInProgress == CMD_SPI_STATUS) || (cmdInProgress == CMD_READ_SPIMAC_STATE)){
			tx->data[0] = spiState;
		}
		spiState = SPICMD_PROCESS_CMD;
	}
	else {
		MGR_LOG_DEBUG("%s: RX size less than 0\r\n",__func__);
		cmdInProgress = CMD_NONE;
		spiState = SPICMD_IDLE;
	}

	return cmdInProgress;
}



static bool MGR_SPI_CMD_process_cmd(uint8_t cmd)
{
	bool ret = true;

	MGR_LOG_DEBUG("%s:: Process %u \r\n",__func__, cmd);
	if ((cmd > 0) && (cmd < SPICMD_MAX_COUNT))
	{
		/* Check if handler function is valid before calling */
		if (cas_spicmd_list_array[cmd].f_ht_cmd_fun_proc != NULL)
		{
			ret = cas_spicmd_list_array[cmd].f_ht_cmd_fun_proc(&rxBuf, &txBuf);
		} else {
			MGR_LOG_DEBUG("%s:: NULL handler for cmd %u\r\n", __func__, cmd);
			rxBuf.next_req = 1;
			bMGR_SPI_DRIVER_read();
			ret = false;
		}
	} else {
		MGR_LOG_DEBUG("NONE/Unknown command %u\r\n", cmd);
		rxBuf.next_req = 1;
		bMGR_SPI_DRIVER_read();
		ret = false;
	}
	return (ret);
}

/* Functions ------------------------------------------------------------------------------------*/

bool MGR_SPI_CMD_start(void *context)
{
	//spi handler stored and receive interrupt one byte running
	bool ret = MCU_SPI_DRIVER_register(context, MGR_SPI_CMD_parseStreamCb);
	if (!ret) {
		spiState = SPICMD_ERROR;
	}
	return ret;
}


void MGR_SPI_CMD_state_handler() {
	bool ret = true;
   	switch (spiState) {
       case SPICMD_INIT:
	   	   // Only in case of restart service and spi ptr should not be changed.
    	   MGR_LOG_DEBUG("SPI_CMD_INIT\r\n");
		   if (MGR_SPI_CMD_start(NULL))
		   {
				spiState = SPICMD_IDLE;
		   } else {
				spiState = SPICMD_ERROR;
		   }
           break;
       case SPICMD_IDLE:
    	   MGR_LOG_DEBUG("SPI_CMD_IDLE\r\n");
    	   // start RX IT for waiting command:
		   rxBuf.next_req = 1;
		   cmdInProgress = CMD_NONE;
		   HAL_StatusTypeDef res = bMGR_SPI_DRIVER_read();
		   if (res != HAL_OK)
		   {
			    MGR_LOG_DEBUG("%s:: Failed to start RX IT, resetting...\r\n", __func__);
				spiState = SPICMD_ERROR;	
		   }
           break;
       case SPICMD_PROCESS_CMD:
    	    MGR_LOG_DEBUG("SPI_CMD_PROCESSCMD\r\n");
			ret = MGR_SPI_CMD_process_cmd(cmdInProgress);
			if (!ret)
			{
				MGR_LOG_DEBUG("%s:: failed to process cmd %u\r\n", __func__, cmdInProgress);
				spiState = SPICMD_IDLE;
			}
           break;
       case SPICMD_WAITING_RX:
    	   /* Timeout for all RX operations to prevent blocking */
    	   if (startTickTimeout == 0)
    	   {
    		   startTickTimeout = HAL_GetTick();
    	   }

    	   if ((HAL_GetTick() - startTickTimeout) > CMD_IT_TIMEOUT)
		   {
			   MGR_LOG_DEBUG("%s::SPICMD_WAITING_RX::Read timeout (req=%u)!\r\n", __func__, rxBuf.next_req);
			   spi_stats.timeout_count++;
			   spiState = SPICMD_ERROR;
		   }
           break;
       case SPICMD_WAITING_TX:
    	   /* Timeout for all TX operations to prevent blocking */
    	   if (startTickTimeout == 0)
    	   {
    		   startTickTimeout = HAL_GetTick();
    	   }

    	   if ((HAL_GetTick() - startTickTimeout) > CMD_IT_TIMEOUT)
		   {
			   MGR_LOG_DEBUG("%s::SPICMD_WAITING_TX::Write timeout (req=%u)!\r\n", __func__, txBuf.next_req);
			   spi_stats.timeout_count++;
			   spiState = SPICMD_ERROR;
		   }
           break;
       case SPICMD_ERROR:
		    MGR_LOG_DEBUG("%s:: SPI error, resetting...\r\n", __func__);
		    ret = MCU_SPI_DRIVER_reset(NULL);
			if (!ret)
			{
				MGR_LOG_DEBUG("%s:: failed to reset %u\r\n", __func__, cmdInProgress);
			}
           break;
       default:
           MGR_LOG_DEBUG("%s:: Unknown spiState: %u\r\n", __func__, spiState);
		   spiState = SPICMD_ERROR;
           break;
   }
}


enum KNS_status_t MGR_SPI_CMD_macEvtProcess(void)
{
	/** Empty weak core, can be overwritten, depending on AT cmd processed */
	enum KNS_status_t cbStatus;
	struct KNS_MAC_srvcEvt_t srvcEvt;
	struct sUserDataTxFifoElt_t *spUserDataMsg = USERDATA_txFifoGetFirst();

	cbStatus = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&srvcEvt);

	if (cbStatus != KNS_STATUS_OK)
		return cbStatus;

	//MGR_LOG_DEBUG("macEVT status %u\r\n", cbStatus);
	/** get pointer to user data FIFO element when possible */
	switch (srvcEvt.id) {
		case (KNS_MAC_TX_DONE):
		case (KNS_MAC_TXACK_DONE):
		case (KNS_MAC_TX_TIMEOUT):
		case (KNS_MAC_TXACK_TIMEOUT):
		case (KNS_MAC_RX_ERROR):
		case (KNS_MAC_RX_TIMEOUT):
			spUserDataMsg = USERDATA_txFifoFindPayload(srvcEvt.tx_ctxt.data,
				srvcEvt.tx_ctxt.data_bitlen);
			kns_assert(spUserDataMsg != NULL);
			macStatus = MAC_RX_TIMEOUT;
		break;
		case (KNS_MAC_ERROR):
			if (srvcEvt.app_evt == KNS_MAC_SEND_DATA) {
				spUserDataMsg = USERDATA_txFifoFindPayload(srvcEvt.tx_ctxt.data,
					srvcEvt.tx_ctxt.data_bitlen);
				MCU_MISC_TCXO_Force_State(false);
				kns_assert(spUserDataMsg != NULL);
				macStatus = MAC_ERROR;
			}
		break;
		default:
		break;
	}

	/** process event */
	switch (srvcEvt.id) {
	case (KNS_MAC_TX_DONE):
		MGR_LOG_DEBUG("MGR_SPI_CMD TX_DONE callback reached\r\n");
		kns_assert(spUserDataMsg->bIsToBeTransmit);
		/** Upon TX done of a mail request message, it means some DL_BC was received
		 * Thus, UL ACK of DL_BC will transmitted by lower layer internally just
		 * after the end of this callback.
		 * AT response "+TX=..." will be sent to UART once UL-ACK is really
		 * transmitted, cf TXACK_DONE below
		 */
		if (spUserDataMsg->u8Attr.sf == ATTR_MAIL_REQUEST) {
			//TODO no service used for the moment
		} else {
			/** @todo Should check integrity between data reported by event
			 * above and the one stored in user data buffer
			 *
			 * So far, as only one user data, consider this is the correct one:
			 * * notify host with AT cmd response then
			 * * free element from user data buffer.
			 */
			USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
		}
		MCU_MISC_TCXO_Force_State(false);
		macStatus = MAC_TX_DONE;
		cbStatus = KNS_STATUS_OK;
		break;
	case (KNS_MAC_TXACK_DONE):
		MGR_LOG_DEBUG("MGR_SPI_CMD TXACK_DONE callback reached\r\n");
		kns_assert(spUserDataMsg->bIsToBeTransmit);
		/** Upon TX done of a mail request message, it means some DL_BC was received
		 * previously and UL ACK of DL_BC was just transmitted.
		 * Send +TX= instead of +TACK=, meaning this is the real end of TX data
		 * transmission
		 */

		if (spUserDataMsg->u8Attr.sf == ATTR_MAIL_REQUEST)
			macStatus = MAC_TX_DONE;
		else
			macStatus = MAC_TXACK_DONE;
		MCU_MISC_TCXO_Force_State(false);

		USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
		cbStatus = KNS_STATUS_OK;
		break;
	case (KNS_MAC_TX_TIMEOUT):
		MGR_LOG_DEBUG("MGR_SPI_CMD TX_TIMEOUT callback reached\r\n");
		kns_assert(spUserDataMsg->bIsToBeTransmit);
		/** @todo Should check integrity between data reported by event above and
		 * the one stored in user data buffer
		 *
		 * So far, as only one user data, consider this is the correct one:
		 * * notify host with AT cmd response then
		 * * free element from user data buffer.
		 */
		MCU_MISC_TCXO_Force_State(false);
		USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
		macStatus = MAC_TX_TIMEOUT;
		cbStatus = KNS_STATUS_TIMEOUT;
		break;
	case (KNS_MAC_TXACK_TIMEOUT):
		MGR_LOG_DEBUG("MGR_SPI_CMD TXACK_TIMEOUT callback reached\r\n");
		MCU_MISC_TCXO_Force_State(false);
		kns_assert(spUserDataMsg->bIsToBeTransmit);
		USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
		macStatus = MAC_TXACK_TIMEOUT;
		cbStatus = KNS_STATUS_TIMEOUT;
		break;
	case (KNS_MAC_RX_ERROR):  /**< RX error during TRX frame, report TX failure then */
		MGR_LOG_DEBUG("MGR_SPI_CMD RX ERROR callback reached\r\n");
		if (spUserDataMsg->bIsToBeTransmit) {
			/** @todo Should check integrity between data reported by event
			 * above and the one stored in user data buffer
			 *
			 * So far, as only one user data, consider this is the correct one:
			 * * notify host with AT cmd response then
			 * * free element from user data buffer.
			 */
			USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
			macStatus = MAC_RX_ERROR;
			cbStatus = KNS_STATUS_RF_ERR;
		} else {
			MGR_LOG_DEBUG("MGR_SPI_CMD RX callback reached\r\n");
			MGR_LOG_DEBUG("RX ERROR  unexpected event received.\r\n");
			macStatus = MAC_ERROR;
			cbStatus = KNS_STATUS_ERROR;
		}
	break;
	case (KNS_MAC_RX_TIMEOUT): /**< RX window reached during TRX, report failure then */
	/** @attention so far, TX failure is reported upon TRX RX timeout when:
	 * * no DL-ACK received for a TX requesting ACK (this case is fine)
		 * * no DL-BC received for a TX mail request. Actually this may really occur
		 * as no BC was available on board (protocol to be discussed)
		 */
		MGR_LOG_DEBUG("MGR_SPI_CMD RX timeout callback reached\r\n");
		/** @todo Should check integrity between data reported by event above and
		 * the one stored in user data buffer
		 *
		 * So far, as only one user data, consider this is the correct one:
		 * * notify host with AT cmd response then
		 * * free element from user data buffer.
		 */
		USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
		macStatus = MAC_RX_TIMEOUT;
		cbStatus = KNS_STATUS_TIMEOUT;
	break;
	case (KNS_MAC_OK):
		MGR_LOG_DEBUG("MGR_SPI_CMD MAC reported OK to previous command.\r\n");
		if (srvcEvt.app_evt == KNS_MAC_STOP_SEND_DATA)
			kns_assert(USERDATA_txFifoFlush() == true);
		macStatus = MAC_OK;
		cbStatus = KNS_STATUS_OK;
	break;
	case (KNS_MAC_ERROR):
		MGR_LOG_DEBUG("MGR_SPI_CMD MAC reported ERROR to previous command.\r\n");
		if (srvcEvt.app_evt == KNS_MAC_SEND_DATA)
			USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
		macStatus = MAC_ERROR;
		cbStatus = KNS_STATUS_ERROR;
	break;
	default:
		kns_assert(0);
		macStatus = MAC_ERROR;
		cbStatus = KNS_STATUS_ERROR;
	break;
	}

	return cbStatus;
}

/**
 * @}
 */
