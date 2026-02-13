// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_at_cmd_list_user_data.c
 * @author Kineis
 * @brief subset of AT commands concerning user data manipulation such as TX
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>

#include "kns_types.h"
#include "user_data.h"
#include "mgr_at_cmd_list_user_data.h"
#include "kns_q.h"
#include "kns_mac.h"
#include "kineis_sw_conf.h"  // for assert include below and ERROR_RETURN_T type
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"
#include "mcu_misc.h"

#include "main.h"

/* Private macro -------------------------------------------------------------*/

/* Private functions ----------------------------------------------------------*/

/** @brief  Set/clear a GPIO around transmission
 *
 * @note It is assumed a GPIO named LED1 is defined. Compile with USE_TX_LED to call STM32 HAL APIs
 *
 * @param[in] state: 1 set gpio, 0 clear GPIO
 */
#ifdef USE_TX_LED // Light on a GPIO when TX occurs
static void Set_TX_LED(uint8_t state)
{
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, state);
}
#else
static void Set_TX_LED(__attribute__((unused)) uint8_t state)
{
}
#endif

__attribute__((unused))
/** @brief  Log array of uint8_t
 * @param[in] data pointer to table
 * @param[in] len number of bytes to log from the table
 */
static void MGR_LOG_array(__attribute__((unused)) uint8_t *data, uint16_t len)
{
	uint16_t i;

	for (i = 0; i < len; i++)
		MGR_LOG_DEBUG_RAW("%02X", data[i]);
	MGR_LOG_DEBUG_RAW("\r\n");
}

/** @brief Handle new TX data, this is the core function of AT+TX cmd
 *
 * @attention This fct assumes user data set by user is multiple of 8 bits.
 *
 * This fct is sensible from security point of view, as it is USER entry. It should be robust to
 * overflow.
 *
 * Maximum allowed data payload, in bits, is:
 *  * for A4 VLD protocols: 24 bits
 * All this will be checked in details in lower-layer frame formating.
 *
 * Here, at AT cmd and USERDATA level, it only checks incoming data is not exceeding the USERDATA
 * buffer size.
 *
 * @param[in] pu8_cmdParamString: string containing AT command
 * @param[in] pcAtCmdPattern: AT command pattern to parse
 *
 * @return true if data is correctly processed, else otherwise
 */
static bool bMGR_AT_CMD_handleNewTxData(uint8_t *pu8_cmdParamString, const char *pcAtCmdPattern)
{
	enum KNS_status_t status = KNS_STATUS_OK;
	struct sUserDataTxFifoElt_t *spUserDataMsg;
	union sUserDataAttribute_t u8UserDataAttr;
	uint8_t *pu8UserDataBuf;
	int16_t i16_scan_param_res;
	uint16_t u16UserDataCharNb;
	uint16_t u16UserDataBitlen;
	uint16_t idx;

//	enum KNS_status_t knsStatus;
	struct KNS_MAC_appEvt_t appEvt = {
		.id = KNS_MAC_SEND_DATA,
		.data_ctxt = {
			.usrdata = {0},
			.usrdata_bitlen = 0,      /* to be filled-up later below */
			.sf = KNS_SF_NO_SERVICE,
		}
	};

	MCU_MISC_TCXO_Force_State(true);
	uint32_t tcxo_warmup_ms = 0;
	MCU_MISC_TCXO_get_warmup(&tcxo_warmup_ms);
	HAL_Delay(tcxo_warmup_ms);
	spUserDataMsg = USERDATA_txFifoReserveElt();
	if (spUserDataMsg != NULL) {

		for (idx = 0; idx < sizeof(spUserDataMsg->u8DataBuf); idx++)
			spUserDataMsg->u8DataBuf[idx] = 0;
		pu8UserDataBuf =  spUserDataMsg->u8DataBuf;
		kns_assert(pu8UserDataBuf != NULL);


		/** Extract USER DATA from pu8_cmdParamString */
		i16_scan_param_res = (int16_t)sscanf((const char *)pu8_cmdParamString, pcAtCmdPattern,
					    pu8UserDataBuf,
					    &u8UserDataAttr.u8_raw);
		u16UserDataCharNb = (uint16_t)strlen((const char *)pu8UserDataBuf);
		MGR_LOG_VERBOSE("[%s %d] %d %d\r\n", __func__, __LINE__, i16_scan_param_res, u16UserDataCharNb);

		/** Assert when the number of characters received from AT command is bigger than
		 * authorized
		 */
		kns_assert(u16UserDataCharNb <= (USERDATA_TX_PAYLOAD_MAX_SIZE -1 ));

		switch (i16_scan_param_res) {
		case 1:/** Case ARGOS Message with user data only */
			u8UserDataAttr.u8_raw = 0x0; /* default attribute to data, no service */
			/* fall through */
		case 2:/** Case ARGOS Message with user data + optional attribute:
			 * User data should be converted form string format to Hex format
			 */
			u16UserDataBitlen = u16MGR_AT_CMD_convertAsciiBinary(pu8UserDataBuf,
									     u16UserDataCharNb);
			if (u16UserDataBitlen <= (USERDATA_TX_DATAFIELD_SIZE * 8)) {
				spUserDataMsg->u16DataBitLen = u16UserDataBitlen;
				spUserDataMsg->u8Attr = u8UserDataAttr;
				kns_assert(USERDATA_txFifoAddElt(spUserDataMsg, true));

				for (idx = 0; idx < sizeof(appEvt.data_ctxt.usrdata); idx++)
					appEvt.data_ctxt.usrdata[idx] =
						spUserDataMsg->u8DataBuf[idx];
				appEvt.data_ctxt.usrdata_bitlen = spUserDataMsg->u16DataBitLen;
				appEvt.data_ctxt.sf =
					(enum KNS_serviceFlag_t)(spUserDataMsg->u8Attr.sf);

				/** @note appEvt.send_ctxt already filled-up at declaration */
				appEvt.id = KNS_MAC_SEND_DATA;
				status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
				if (status != KNS_STATUS_OK)
					return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
				return true;
			}
			MGR_LOG_VERBOSE("[ERROR] User data is badly formatted (check length)\r\n");
			return bMGR_AT_CMD_logFailedMsg(ERROR_INVALID_USER_DATA_LENGTH);
		case 0: /* Case ARGOS Message without user data */
		default:
			MGR_LOG_VERBOSE("[ERROR] AT+TX command is badly formatted\r\n");
			return bMGR_AT_CMD_logFailedMsg(ERROR_MISSING_PARAMETERS);
		}
	} else {
		MGR_LOG_VERBOSE("[ERROR] TX FIFO full, cannot get extra data.\r\n");
		return bMGR_AT_CMD_logFailedMsg(ERROR_DATA_QUEUE_FULL);
	}
}

/* Public functions ----------------------------------------------------------*/

uint16_t u16MGR_AT_CMD_convertAsciiBinary(uint8_t *pu8InputBuffer, uint16_t u16_charNb)
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
 * @brief Converts an ASCII numeric string to a uint32_t integer.
 *
 * @param pu8InputBuffer Pointer to the numeric ASCII string.
 * @param u32OutputValue Pointer to store the converted integer.
 * @return uint16_t Number of bits required to represent the value, or 0 if overflow occurs.
 */
uint16_t u16MGR_AT_CMD_convertAsciiToInt32(uint8_t *pu8InputBuffer, uint32_t *u32OutputValue)
{
    char *endPtr;
    unsigned long int tempValue;

    // Convert ASCII string to unsigned long integer
    tempValue = strtoul((char *)pu8InputBuffer, &endPtr, 10);

    // Check for conversion errors (no valid digits, or overflow)
    if (*endPtr != '\0' || tempValue > UINT32_MAX) {
        *u32OutputValue = 0;
        return 0; // Overflow occurred
    }

    *u32OutputValue = (uint32_t)tempValue;

    // Compute the number of bits required to represent the value
    uint16_t bitCount = 0;
    uint32_t temp = *u32OutputValue;
    while (temp > 0) {
        temp >>= 1;
        bitCount++;
    }

    return (bitCount > 32) ? 0 : bitCount; // Redundant check, but ensures safety
}

bool bMGR_AT_CMD_TX_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode)
{
	/** @attention pattern length below shall not be longer than the length defined by
	 * FRAME_MAX_LEN.
	 *
	 * So far, here in LDA2 example; limit size to 48 chars, i.e. 24 bytes
	 */
#ifdef USE_HDA4
	static const char cAtCmdPattern[] = "AT+TX=%1265[0-9A-Fa-f],0x%hX";
#else
	static const char cAtCmdPattern[] = "AT+TX=%49[0-9A-Fa-f],0x%hX";
#endif

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MGR_LOG_VERBOSE("[ERROR] Status mode is unauthorized for this AT cmd\r\n");
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}

	if (bMGR_AT_CMD_handleNewTxData(pu8_cmdParamString, cAtCmdPattern))
		return true;
	else
		return false;
}

#ifdef USE_RX_STACK
bool bMGR_AT_CMD_RX_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode)
{
	int16_t scanParamRes;
	uint16_t rxMode = 0;
	struct KNS_MAC_appEvt_t appEvt;
	enum KNS_status_t status = KNS_STATUS_OK;

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MGR_LOG_VERBOSE("[ERROR] Status mode is unauthorized for this AT cmd\r\n");
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}

	scanParamRes = sscanf((const char *)pu8_cmdParamString,
			(const char *)"AT+RX=%hu",
			&rxMode);

	if (scanParamRes == 1) {
		switch (rxMode) {
		case 0:
			MGR_LOG_VERBOSE("stop RX\r\n");
			appEvt.id = KNS_MAC_RX_STOP;
			status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
			if (status != KNS_STATUS_OK)
				return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
			return true;
		break;
		case 1:
		{
			MGR_LOG_VERBOSE("start RX\r\n");
			appEvt.id = KNS_MAC_RX_START;
			status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
			if (status != KNS_STATUS_OK)
				return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
			return true;
		}
		break;
		default:
			return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
		break;
		}
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
}
#endif

enum KNS_status_t MGR_AT_CMD_macEvtProcess(void)
{
	enum KNS_status_t cbStatus;
	struct KNS_MAC_srvcEvt_t srvcEvt;
	struct sUserDataTxFifoElt_t *spUserDataMsg = USERDATA_txFifoGetFirst();

	cbStatus = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&srvcEvt);

	if (cbStatus != KNS_STATUS_OK)
		return cbStatus;

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
	break;
	case (KNS_MAC_ERROR):
		if (srvcEvt.app_evt == KNS_MAC_SEND_DATA) {
			spUserDataMsg = USERDATA_txFifoFindPayload(srvcEvt.tx_ctxt.data,
				srvcEvt.tx_ctxt.data_bitlen);
			MCU_MISC_TCXO_Force_State(false);
			MCU_MISC_turn_off_pa();
			kns_assert(spUserDataMsg != NULL);
		}
	break;
	default:
	break;
	}

	/** process event */
	switch (srvcEvt.id) {
	case (KNS_MAC_TX_DONE):
		MCU_MISC_TCXO_Force_State(false);
		MCU_MISC_turn_off_pa();
		kns_assert(spUserDataMsg->bIsToBeTransmit);
		/** Upon TX done of a mail request message, it means some DL_BC was received
		 * Thus, UL ACK of DL_BC will transmitted by lower layer internally just
		 * after the end of this callback.
		 * AT response "+TX=..." will be sent to UART once UL-ACK is really
		 * transmitted, cf TXACK_DONE below
		 */
		if (spUserDataMsg->u8Attr.sf == ATTR_MAIL_REQUEST) {
			Set_TX_LED(0);
		} else {
			/** @todo Should check integrity between data reported by event
			 * above and the one stored in user data buffer
			 *
			 * So far, as only one user data, consider this is the correct one:
			 * * notify host with AT cmd response then
			 * * free element from user data buffer.
			 */
			bMGR_AT_CMD_sendResponse(ATCMD_RSP_TXOK, (void *)spUserDataMsg);
			USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
			Set_TX_LED(0);
		}
		cbStatus = KNS_STATUS_OK;
	break;
	case (KNS_MAC_TXACK_DONE):
//		MGR_LOG_DEBUG("MGR_AT_CMD TXACK_DONE callback reached\r\n");
		kns_assert(spUserDataMsg->bIsToBeTransmit);
		/** Upon TX done of a mail request message, it means some DL_BC was received
		 * previously and UL ACK of DL_BC was just transmitted.
		 * Send +TX= instead of +TACK=, meaning this is the real end of TX data
		 * transmission
		 */
		if (spUserDataMsg->u8Attr.sf == ATTR_MAIL_REQUEST)
			bMGR_AT_CMD_sendResponse(ATCMD_RSP_TXOK, (void *)spUserDataMsg);
		else
			bMGR_AT_CMD_sendResponse(ATCMD_RSP_TXACKOK, NULL);

		MCU_MISC_TCXO_Force_State(false);
		MCU_MISC_turn_off_pa();
		USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
		Set_TX_LED(0);
		cbStatus = KNS_STATUS_OK;
	break;
	case (KNS_MAC_TX_TIMEOUT):
		MCU_MISC_TCXO_Force_State(false);
		MCU_MISC_turn_off_pa();
		kns_assert(spUserDataMsg->bIsToBeTransmit);
		/** @todo Should check integrity between data reported by event above and
		 * the one stored in user data buffer
		 *
		 * So far, as only one user data, consider this is the correct one:
		 * * notify host with AT cmd response then
		 * * free element from user data buffer.
		 */
		bMGR_AT_CMD_sendResponse(ATCMD_RSP_TXTIMEOUT, (void *)spUserDataMsg);
		USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
		Set_TX_LED(0);
		cbStatus = KNS_STATUS_TIMEOUT;
	break;
	case (KNS_MAC_TXACK_TIMEOUT):
		MCU_MISC_TCXO_Force_State(false);
		MCU_MISC_turn_off_pa();
		kns_assert(spUserDataMsg->bIsToBeTransmit);
		bMGR_AT_CMD_sendResponse(ATCMD_RSP_TXACKNOTOK, NULL);
		USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
		Set_TX_LED(0);
		cbStatus = KNS_STATUS_TIMEOUT;
	break;
	case (KNS_MAC_RX_ERROR):  /**< RX error during TRX frame, report TX failure then */
		MCU_MISC_TCXO_Force_State(false);
		MCU_MISC_turn_off_pa();
		if (spUserDataMsg->bIsToBeTransmit) {
//			MGR_LOG_DEBUG("RX enable ERROR (%d bits = %d bytes + %d bits): 0x",
//				srvcEvt.tx_ctxt.data_bitlen,
//				srvcEvt.tx_ctxt.data_bitlen>>3,
//				srvcEvt.tx_ctxt.data_bitlen&0x07);
//			MGR_LOG_array(srvcEvt.tx_ctxt.data,
//				(srvcEvt.tx_ctxt.data_bitlen+7)>>3);
			/** @todo Should check integrity between data reported by event
			 * above and the one stored in user data buffer
			 *
			 * So far, as only one user data, consider this is the correct one:
			 * * notify host with AT cmd response then
			 * * free element from user data buffer.
			 */
			bMGR_AT_CMD_sendResponse(ATCMD_RSP_RXERROR, (void *)spUserDataMsg);
			USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
			Set_TX_LED(0);
			cbStatus = KNS_STATUS_RF_ERR;
		} else {
			MGR_LOG_DEBUG("MGR_AT_CMD RX callback reached\r\n");
			MGR_LOG_DEBUG("RX ERROR  unexpected event received.\r\n");
			cbStatus = KNS_STATUS_ERROR;
		}
	break;
	case (KNS_MAC_RX_TIMEOUT): /**< RX window reached during TRX, report failure then */
		MCU_MISC_TCXO_Force_State(false);
		MCU_MISC_turn_off_pa();
		/** @attention so far, TX failure is reported upon TRX RX timeout when:
		 * * no DL-ACK received for a TX requesting ACK (this case is fine)
		 * * no DL-BC received for a TX mail request. Actually this may really occur
		 * as no BC was available on board (protocol to be discussed)
		 */
//		MGR_LOG_DEBUG("MGR_AT_CMD RX timeout callback reached\r\n");
//		MGR_LOG_DEBUG("ERROR: no DL frm for (%d bits = %d bytes + %d bits): 0x",
//			srvcEvt.tx_ctxt.data_bitlen,
//			srvcEvt.tx_ctxt.data_bitlen>>3,
//			srvcEvt.tx_ctxt.data_bitlen&0x07);
//		MGR_LOG_array(srvcEvt.tx_ctxt.data, (srvcEvt.tx_ctxt.data_bitlen+7)>>3);
		/** @todo Should check integrity between data reported by event above and
		 * the one stored in user data buffer
		 *
		 * So far, as only one user data, consider this is the correct one:
		 * * notify host with AT cmd response then
		 * * free element from user data buffer.
		 */
		bMGR_AT_CMD_sendResponse(ATCMD_RSP_RXTIMEOUT, (void *)spUserDataMsg);
		USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
		Set_TX_LED(0);
		cbStatus = KNS_STATUS_TIMEOUT;
	break;
#ifdef USE_RX_STACK
	case (KNS_MAC_DL_ACK):
	case (KNS_MAC_DL_BC):
//		MGR_LOG_DEBUG("MGR_AT_CMD DL callback reached\r\n");
//		MGR_LOG_DEBUG("decoded msg (%d bits = %d bytes + %d bits): 0x",
//			srvcEvt.rx_ctxt.data_bitlen,
//			srvcEvt.rx_ctxt.data_bitlen>>3,
//			srvcEvt.rx_ctxt.data_bitlen&0x07);
//		MGR_LOG_array(srvcEvt.rx_ctxt.data, (srvcEvt.rx_ctxt.data_bitlen+7)>>3);
		/** @todo Should check integrity between data reported by event above and
		 * the one stored in user data buffer
		 *
		 * So far, as only one user data, consider this is the correct one:
		 * * notify host with AT cmd response then
		 * * free element from user data buffer.
		 */
		bMGR_AT_CMD_sendResponse(ATCMD_RSP_DLOK, (void *)&(srvcEvt.rx_ctxt));
		cbStatus = KNS_STATUS_OK;
	break;
	case (KNS_MAC_RX_RECEIVED):
//		MGR_LOG_DEBUG("MGR_AT_CMD RX callback reached\r\n");
//		MGR_LOG_DEBUG("bitstream (%d bits = %d bytes + %d bits): 0x",
//			srvcEvt.rx_ctxt.data_bitlen,
//			srvcEvt.rx_ctxt.data_bitlen>>3,
//			srvcEvt.rx_ctxt.data_bitlen&0x07);
//		MGR_LOG_array(srvcEvt.rx_ctxt.data, (srvcEvt.rx_ctxt.data_bitlen+7)>>3);
		/** @todo Should check integrity between data reported by event above and
		 * the one stored in user data buffer
		 *
		 * So far, as only one user data, consider this is the correct one:
		 * * notify host with AT cmd response then
		 * * free element from user data buffer.
		 */
		bMGR_AT_CMD_sendResponse(ATCMD_RSP_RXOK, (void *)&(srvcEvt.rx_ctxt));
		cbStatus = KNS_STATUS_OK;
	break;
	case (KNS_MAC_SAT_DETECTED):
//		MGR_LOG_DEBUG("MGR_AT_CMD SAT detect callback reached\r\n");
		bMGR_AT_CMD_sendResponse(ATCMD_RSP_SATDET, (void *)&(srvcEvt.satdet_ctxt));
		cbStatus = KNS_STATUS_OK;
	break;
	case (KNS_MAC_SAT_LOST):
		bMGR_AT_CMD_sendResponse(ATCMD_RSP_SATLOST, NULL);
		cbStatus = KNS_STATUS_OK;
	break;
	case (KNS_MAC_SAT_DETECT_TIMEOUT):
		bMGR_AT_CMD_sendResponse(ATCMD_RSP_SATDETTO, NULL);
		cbStatus = KNS_STATUS_OK;
	break;
#endif
	case (KNS_MAC_OK):
//		MGR_LOG_DEBUG("MGR_AT_CMD MAC reported OK to previous command.\r\n");
		bMGR_AT_CMD_logSucceedMsg();
		if (srvcEvt.app_evt == KNS_MAC_SEND_DATA)
			Set_TX_LED(1);
		if (srvcEvt.app_evt == KNS_MAC_STOP_SEND_DATA)
			kns_assert(USERDATA_txFifoFlush() == true);
		cbStatus = KNS_STATUS_OK;
	break;
	case (KNS_MAC_ERROR):
//		MGR_LOG_DEBUG("MGR_AT_CMD MAC reported ERROR to previous command.\r\n");
		bMGR_AT_CMD_logFailedMsg((enum KNS_status_t)srvcEvt.status);
		if (srvcEvt.app_evt == KNS_MAC_SEND_DATA)
			USERDATA_txFifoRemoveElt(spUserDataMsg);/* Free as host notified */
		cbStatus = KNS_STATUS_ERROR;
	break;
	case (KNS_MAC_RF_ABORTED):
		/* RF operation was aborted (e.g. user requested stop) */
		MCU_MISC_TCXO_Force_State(false);
		MCU_MISC_turn_off_pa();
		bMGR_AT_CMD_sendResponse(ATCMD_RSP_RFABORTED, NULL);
		if (spUserDataMsg != NULL && spUserDataMsg->bIsToBeTransmit)
			USERDATA_txFifoRemoveElt(spUserDataMsg);
		Set_TX_LED(0);
		cbStatus = KNS_STATUS_OK;
	break;
	default:
		kns_assert(0);
		cbStatus = KNS_STATUS_ERROR;
	break;
	}

	return cbStatus;
}

/**
 * @}
 */
