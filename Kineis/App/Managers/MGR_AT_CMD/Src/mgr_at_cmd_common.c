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
#include "mgr_at_cmd_common.h"
#include "mcu_at_console.h"
#include "user_data.h"
#include "kns_mac.h"


/* Public functions ----------------------------------------------------------*/

bool bMGR_AT_CMD_logSucceedMsg(void)
{
	MCU_AT_CONSOLE_send("+OK\r\n");
	return true;
}

bool bMGR_AT_CMD_logFailedMsg(enum ERROR_RETURN_T eErrorType)
{
	MCU_AT_CONSOLE_send("+ERROR=%i\r\n", eErrorType);
	return false;
}

bool bMGR_AT_CMD_sendResponse(enum atcmd_rsp_type_t atcmd_response_type, void *atcmd_rsp_data)
{

	switch (atcmd_response_type) {
	case ATCMD_RSP_TXOK:
	{
		if (atcmd_rsp_data != NULL) {
			struct sUserDataTxFifoElt_t *spUserDataMsg =
						(struct sUserDataTxFifoElt_t *)atcmd_rsp_data;
			uint8_t *pu8UserDataPtr = spUserDataMsg->u8DataBuf;
			uint16_t u16UserDataBitlen = spUserDataMsg->u16DataBitLen;

			MCU_AT_CONSOLE_send("+TX=0,");
			MCU_AT_CONSOLE_send_dataBuf(pu8UserDataPtr, u16UserDataBitlen);
			MCU_AT_CONSOLE_send("\r\n");
		}
		return true;
	}
	break;
	case ATCMD_RSP_RXTIMEOUT:
	case ATCMD_RSP_TXTIMEOUT:
	case ATCMD_RSP_RXERROR:
	{
		if (atcmd_rsp_data != NULL) {
			struct sUserDataTxFifoElt_t *spUserDataMsg =
						(struct sUserDataTxFifoElt_t *)atcmd_rsp_data;
			uint8_t *pu8UserDataPtr = spUserDataMsg->u8DataBuf;
			uint16_t u16UserDataBitlen = spUserDataMsg->u16DataBitLen;
			enum ERROR_RETURN_T error_id;

			if (atcmd_response_type == ATCMD_RSP_RXERROR)
				error_id = (enum ERROR_RETURN_T) KNS_STATUS_RF_ERR;
			else
				error_id = (enum ERROR_RETURN_T) KNS_STATUS_TIMEOUT;

			MCU_AT_CONSOLE_send("+TX=%d,", error_id);
			MCU_AT_CONSOLE_send_dataBuf(pu8UserDataPtr, u16UserDataBitlen);
			MCU_AT_CONSOLE_send("\r\n");
		}
		return true;
	}
	break;
	case ATCMD_RSP_TXACKOK:
	{
		MCU_AT_CONSOLE_send("+TXACK=0\r\n");
		return true;
	}
	break;
	case ATCMD_RSP_TXACKNOTOK:
	{
		enum ERROR_RETURN_T error_id = (enum ERROR_RETURN_T) KNS_STATUS_TIMEOUT;

		MCU_AT_CONSOLE_send("+TXACK=%d\r\n", error_id);
		return true;
	}
	break;
	case ATCMD_RSP_SATDET:
	{
		if (atcmd_rsp_data != NULL) {
			struct KNS_MAC_SATDET_ctxt_t *det_info =
						(struct KNS_MAC_SATDET_ctxt_t *)atcmd_rsp_data;
			uint32_t detected_freq = det_info->detected_freq;
			uint32_t detect_duration = det_info->detect_duration;
			float rssi = det_info->rssi;

			MCU_AT_CONSOLE_send("+SATDET=%ld,%ld,%f\r\n", detect_duration,
					detected_freq, rssi);
		}
		return true;
	}
	break;
	case ATCMD_RSP_SATLOST:
	{
		MCU_AT_CONSOLE_send("+SATLOST=\r\n");
		return true;
	}
	break;
	case ATCMD_RSP_SATDETTO:
	{
		MCU_AT_CONSOLE_send("+SATDET=0,0,0\r\n");
		return true;
	}
	break;
	case ATCMD_RSP_RXOK:
	{
		if (atcmd_rsp_data != NULL) {
			struct KNS_MAC_RX_frm_ctxt_t *receivedFrm =
						(struct KNS_MAC_RX_frm_ctxt_t *)atcmd_rsp_data;
			uint8_t *rxFrmDataPtr = receivedFrm->data;
			uint16_t rxFrmDataBitlen = receivedFrm->data_bitlen;
			float rssi = receivedFrm->rssi;

			MCU_AT_CONSOLE_send("+RX=");
			MCU_AT_CONSOLE_send_dataBuf(rxFrmDataPtr, rxFrmDataBitlen);
			MCU_AT_CONSOLE_send(",%f\r\n", rssi);
		}
		return true;
	}
	break;
	case ATCMD_RSP_DLOK:
	{
		if (atcmd_rsp_data != NULL) {
			struct KNS_MAC_RX_frm_ctxt_t *reeceivedMsg =
						(struct KNS_MAC_RX_frm_ctxt_t *)atcmd_rsp_data;
			uint8_t *rxMsgDataPtr = reeceivedMsg->data;
			uint16_t rxMsgDataBitlen = reeceivedMsg->data_bitlen;

			MCU_AT_CONSOLE_send("+DL=");
			MCU_AT_CONSOLE_send_dataBuf(rxMsgDataPtr, rxMsgDataBitlen);
			MCU_AT_CONSOLE_send("\r\n");
		}
		return true;
	}
	break;
	default:
		/* Unknown response type - notify user */
		MCU_AT_CONSOLE_send("+ERROR=%i\r\n", (int)ERROR_UNKNOWN);
		break;
	}
	return false;
}

/**
 * @}
 */
