// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_at_cmd_list_user_data.c
 * @author Kineis
 * @brief subset of AT commands concerning user data manipulation such as TX
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "kns_types.h"
#include "user_data.h"
#include "mgr_spi_cmd_list.h"
#include "mgr_spi_cmd_list_user_data.h"
#include "kns_q.h"
#include "kns_mac.h"
#include "kineis_sw_conf.h"  // for assert include below and ERROR_RETURN_T type
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"
#include "mcu_misc.h"
#ifdef USE_TX_LED // Light on a GPIO when TX occurs
#include "main.h"
#endif

/* External variable for MAC status acknowledgment tracking */
extern bool macStatusAcknowledged;

uint16_t userTxPayloadSize;
/* Private macro -------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/

/** @brief  Set/clear a GPIO around transmission
 *
 * @note It is assumed a GPIO named LED1 is defined. Compile with USE_TX_LED to call STM32 HAL APIs
 *
 * @param[in] state: 1 set gpio, 0 clear GPIO
 */
//#if defined(USE_UART_DRIVER)
//#ifdef USE_TX_LED // Light on a GPIO when TX occurs
//static void Set_TX_LED(uint8_t state)
//{
//	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state);
//}
//#else
//static void Set_TX_LED(__attribute__((unused)) uint8_t state)
//{
//}
//#endif
//
//__attribute__((unused))
///** @brief  Log array of uint8_t
// * @param[in] data pointer to table
// * @param[in] len number of bytes to log from the table
// */
//static void MGR_LOG_array(__attribute__((unused)) uint8_t *data, uint16_t len)
//{
//	uint16_t i;
//
//	for (i = 0; i < len; i++)
//		MGR_LOG_DEBUG_RAW("%02X", data[i]);
//	MGR_LOG_DEBUG_RAW("\r\n");
//}
//#endif



/* Public functions ----------------------------------------------------------*/

uint16_t u16MGR_SPI_CMD_convertAsciiBinary(uint8_t *pu8InputBuffer, uint16_t u16_charNb)
{
	uint16_t u16_index = 0;
	uint8_t u8_high, u8_low;

	for (u16_index = 0; u16_index < (u16_charNb / 2); u16_index++) {
		u8_high = u8UTIL_convertCharToHex4bits(pu8InputBuffer[2 * u16_index]);
		u8_low = u8UTIL_convertCharToHex4bits(pu8InputBuffer[2 * u16_index + 1]);
		if ((u8_high != 0xFF) && (u8_low != 0xFF))
			pu8InputBuffer[u16_index] = (uint8_t)((u8_high << 4) | u8_low);
		else
			return 0;
	}

	/** Case : characters number is an odd number */
	if (u16_charNb % 2 != 0) {
		u8_high = u8UTIL_convertCharToHex4bits(pu8InputBuffer[u16_charNb - 1]);
		u8_low = 0;
		if (u8_high != 0xFF)
			pu8InputBuffer[u16_index++] = (uint8_t)((u8_high << 4) | u8_low);
		else
			return 0;
	}

	/** Set other bytes to zero */
	for (; u16_index < u16_charNb; u16_index++)
		pu8InputBuffer[u16_index] = 0;

	/** Return data length in bits */
	return (uint16_t)(((u16_charNb / 2) * 8) + ((u16_charNb % 2) ? 4 : 0));
}


/**
 * @brief
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_SPI_CMD_WRITETXREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	(void)rx;  /* Unused - request has no payload */
	HAL_StatusTypeDef ret = HAL_OK;

	userTxPayloadSize = 0;

	/* Send ACK response */
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
 * @brief
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_SPI_CMD_WRITETXSIZE_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;

	/* Extract size from payload (little-endian: LSB first) */
	userTxPayloadSize = (uint16_t)(rx->data[1] | (rx->data[2] << 8));
	MGR_LOG_DEBUG("TX size received: %u bytes\r\n", userTxPayloadSize);

	if (userTxPayloadSize > USERDATA_TX_PAYLOAD_MAX_SIZE) {
		MGR_LOG_DEBUG("[ERROR] TX size too large: %u > %u\r\n",
		              userTxPayloadSize, USERDATA_TX_PAYLOAD_MAX_SIZE);
		macStatus = MAC_TX_SIZE_ERROR;
		return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	}

	/* Send ACK response */
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
 * @brief
 *
 * @return true if command is correctly received and processed, false if error
 */
bool bMGR_SPI_CMD_WRITETX_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;
	enum KNS_status_t status = KNS_STATUS_OK;
	struct sUserDataTxFifoElt_t *spUserDataMsg;
	union sUserDataAttribute_t u8UserDataAttr;
	uint8_t *pu8UserDataBuf;
	uint16_t u16UserDataBitlen;
	uint16_t idx;

	struct KNS_MAC_appEvt_t appEvtTx = {
		.id = KNS_MAC_SEND_DATA,
		.data_ctxt = {
			.usrdata = {0},
			.usrdata_bitlen = 0,      /* to be filled-up later below */
			.sf = KNS_SF_NO_SERVICE,
		}
	};

	/* Validate payload size early - before any blocking operations */
	if (userTxPayloadSize == 0) {
		MGR_LOG_DEBUG("[ERROR] TX size not set (call WRITE_TX_SIZE first)\r\n");
		return bMGR_SPI_CMD_logFailedMsg(ERROR_INVALID_USER_DATA_LENGTH, tx);
	}

	/* Reserve FIFO element early to detect queue full before sending ACK */
	spUserDataMsg = USERDATA_txFifoReserveElt();
	if (spUserDataMsg == NULL) {
		MGR_LOG_VERBOSE("[ERROR] TX FIFO full, cannot get extra data.\r\n");
		return bMGR_SPI_CMD_logFailedMsg(ERROR_DATA_QUEUE_FULL, tx);
	}

	/* Validate payload fits in buffer */
	if (userTxPayloadSize > sizeof(spUserDataMsg->u8DataBuf)) {
		MGR_LOG_DEBUG("[ERROR] Payload size exceeds buffer: %u > %u\r\n",
		              userTxPayloadSize, (uint16_t)sizeof(spUserDataMsg->u8DataBuf));
		USERDATA_txFifoRemoveElt(spUserDataMsg);
		return bMGR_SPI_CMD_logFailedMsg(ERROR_INVALID_USER_DATA_LENGTH, tx);
	}

	/* CRITICAL: Send ACK response IMMEDIATELY before any blocking operations.
	 * This prepares the response for the next SPI transaction (NOP from master).
	 * The actual TX processing (TCXO warmup, MAC queue) happens asynchronously. */
	tx->data[0] = 1;  /* ACK: TX data received and queued */
	tx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

	if (ret != HAL_OK) {
		USERDATA_txFifoRemoveElt(spUserDataMsg);
		return false;
	}

	/* Now do the actual TX processing (this may take time but response is ready) */

	/* Clear the buffer */
	for (idx = 0; idx < sizeof(spUserDataMsg->u8DataBuf); idx++)
		spUserDataMsg->u8DataBuf[idx] = 0;

	pu8UserDataBuf = spUserDataMsg->u8DataBuf;
	kns_assert(pu8UserDataBuf != NULL);

	/* Copy the raw bytes received via SPI */
	for (idx = 0; idx < userTxPayloadSize; idx++) {
		pu8UserDataBuf[idx] = rx->data[1 + idx];
	}

	/* Default attribute: raw data, no special service */
	u8UserDataAttr.u8_raw = 0x0;

	/* Set the bit length for the message */
	u16UserDataBitlen = userTxPayloadSize * 8;
	spUserDataMsg->u16DataBitLen = u16UserDataBitlen;
	spUserDataMsg->u8Attr = u8UserDataAttr;

	/* Add the message to the FIFO */
	kns_assert(USERDATA_txFifoAddElt(spUserDataMsg, true));

	/* Populate the application event structure */
	for (idx = 0; idx < sizeof(appEvtTx.data_ctxt.usrdata); idx++)
		appEvtTx.data_ctxt.usrdata[idx] = spUserDataMsg->u8DataBuf[idx];

	appEvtTx.data_ctxt.usrdata_bitlen = spUserDataMsg->u16DataBitLen;
	appEvtTx.data_ctxt.sf = (enum KNS_serviceFlag_t)(spUserDataMsg->u8Attr.sf);

	/* Enable TCXO and wait for warmup (same as UART flow) */
	MCU_MISC_TCXO_Force_State(true);
	uint32_t tcxo_warmup_ms = 0;
	MCU_MISC_TCXO_get_warmup(&tcxo_warmup_ms);
	HAL_Delay(tcxo_warmup_ms);

	/* Push the event to the MAC layer */
	status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvtTx);
	switch (status) {
		case KNS_STATUS_QFULL:
			MGR_LOG_VERBOSE("[ERROR] TX queue full after ACK sent.\r\n");
			USERDATA_txFifoRemoveElt(spUserDataMsg);
			MCU_MISC_TCXO_Force_State(false);
			MCU_MISC_turn_off_pa();
			macStatus = MAC_ERROR;
			macStatusAcknowledged = true;  /* Allow reset on next read */
			break;
		case KNS_STATUS_OK:
			/* Successfully pushed the event - TX is now in progress
			 * Reset any previous terminal status since we're starting a new TX */
			macStatus = MAC_TX_IN_PROGRESS;
			macStatusAcknowledged = true;  /* Previous status cleared, new TX started */
			MGR_LOG_DEBUG("TX queued: %u bytes\r\n", userTxPayloadSize);
			break;
		default:
			MGR_LOG_VERBOSE("[ERROR] Unknown status when pushing TX data.\r\n");
			USERDATA_txFifoRemoveElt(spUserDataMsg);
			MCU_MISC_TCXO_Force_State(false);
			MCU_MISC_turn_off_pa();
			macStatus = MAC_ERROR;
			macStatusAcknowledged = true;  /* Allow reset on next read */
			break;
	}

	/* Note: ACK was already sent, so return true.
	 * Master should poll MAC_STATUS to get actual TX result. */
	return true;
}


/**
 * @}
 */
