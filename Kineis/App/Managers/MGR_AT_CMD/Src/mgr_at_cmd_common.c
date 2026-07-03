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
#include "kns_mac_prfl_cfg.h"

/* Compile-time guards on the krd_fw ABI values we depend on. If a future
 * lib upgrade silently shifts any of these, the build fails here with a
 * clear message instead of producing a tag that talks past its GUI.
 * Update both halves together and bump the AT version when adjusting. */
_Static_assert(KNS_STATUS_OK          == 0,    "KNS_STATUS_OK position");
_Static_assert(KNS_STATUS_ERROR       == 1,    "KNS_STATUS_ERROR position");
_Static_assert(KNS_STATUS_BAD_SETTING == 6,    "v10 ABI position preserved");
_Static_assert(KNS_STATUS_ABORT       == 7,    "v11 inserted ABORT after BAD_SETTING");
_Static_assert(KNS_STATUS_RF_ERR      == 100,  "RF_ERR fixed slot");
_Static_assert(KNS_STATUS_MAX         == 1000, "ERROR_RETURN_T base assumes this");
_Static_assert(KNS_MAC_PRFL_NONE      == 0,    "AT+KMAC reset path uses NONE=0");
_Static_assert(KNS_MAC_PRFL_BASIC     == 1,    "UW_DOPPLER lock checks BASIC=1");
_Static_assert(KNS_MAC_PRFL_BLIND     == 2,    "GUI may send BLIND=2");
_Static_assert(KNS_MAC_PRFL_MAX       == 5,    "v11 adds BLIND_POS, total 5 profiles");
_Static_assert(sizeof(struct KNS_MAC_BLIND_usrCfg_t) == 7,
               "v11 grew BLIND_usrCfg_t by 1 byte (per_offset); "
               "AT+KMAC=2 buffer must accommodate 14 hex chars");


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

enum ERROR_RETURN_T MGR_AT_CMD_mapKnsStatusToError(enum KNS_status_t s)
{
	if (s == KNS_STATUS_ABORT)
		return (enum ERROR_RETURN_T)KNS_STATUS_ERROR;
	return (enum ERROR_RETURN_T)s;
}

/* Last Kineis TX frame handler, captured from the MAC event by
 * MGR_AT_CMD_macEvtProcess before a +TX response is emitted. It is APPENDED at
 * the END: "+TX=<code>,<data>,<hdlr>" so a legacy host parsing only
 * <code>,<data> stays compatible. NOTE: this DIVERGES from the KIM reference,
 * which puts the handler FIRST (+TX=<hdlr>,<code>,<data>) and breaks backward
 * compat. AT TX FIFO depth is 1, so one pending handler is enough. */
static int16_t s_txFrmHdlr = -1;

void MGR_AT_CMD_setTxFrmHdlr(int16_t frm_hdlr)
{
	s_txFrmHdlr = frm_hdlr;
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
			/* Handler appended at the END for backward compat (see s_txFrmHdlr). */
			MCU_AT_CONSOLE_send(",%d\r\n", s_txFrmHdlr);
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
			/* Handler appended at the END for backward compat (see s_txFrmHdlr). */
			MCU_AT_CONSOLE_send(",%d\r\n", s_txFrmHdlr);
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
	case ATCMD_RSP_RFABORTED:
	{
		MCU_AT_CONSOLE_send("+RFABORT=0\r\n");
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
