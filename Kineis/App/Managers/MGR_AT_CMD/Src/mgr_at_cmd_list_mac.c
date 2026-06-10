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
 * @addtogroup MGR_AT_CMD
 * @{
 */


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "mcu_at_console.h"
#include "mgr_at_cmd_list_user_data.h"
#include "mgr_at_cmd_list_mac.h"
#include "kns_q.h"
#include "kns_mac.h"
#include "mgr_log.h"
#include "kns_assert.h"

/* Functions -----------------------------------------------------------------*/

bool bMGR_AT_CMD_KMAC_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode)
{
#if defined(USE_UW_DOPPLER_APP)
	/* UW_DOPPLER hardcodes the MAC profile to BASIC at boot and the
	 * surface-driven TX path is the only one in this build — re-init via
	 * AT+KMAC is meaningless. The GUI sends it as part of its KIM-device
	 * discovery probe, so we acknowledge synchronously instead of pushing
	 * an event the app loop never replies to (legacy behaviour was a
	 * 1.5 s GUI timeout on every connect). */
	(void)pu8_cmdParamString;
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		/* Hardcoded BASIC profile = id 1; report a zero-filled context
		 * so the GUI doesn't choke on a missing comma. */
		MCU_AT_CONSOLE_send("+KMAC=1,0000000000\r\n");
		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logSucceedMsg();
#else
	int16_t scan_param_res;
	uint16_t prflCtxtCharNb;
	enum KNS_status_t status = KNS_STATUS_OK;
	struct KNS_MAC_prflInfo_t prfl_info;
	uint8_t *prflCfgPtr;
	uint16_t idx;

	uint8_t prflCtxtData[13]; // 6 bytes doubled due to sscanf ASCII + end \0
	struct KNS_MAC_appEvt_t appEvt = {
		.id = KNS_MAC_INIT,
		.init_prfl_ctxt = {
			.id =  KNS_MAC_PRFL_NONE,
			.blindCfg = {0}
		}
	};

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		KNS_MAC_getPrflInfo(&prfl_info);
		prflCfgPtr = &prfl_info.prflCfgPtr;
		MCU_AT_CONSOLE_send("+KMAC=%d,",prfl_info.id);
		// log profile context, reduce size by 1 due to `id` field of KNS_MAC_prflInfo_t
		for (idx = 0; idx < (sizeof(prfl_info) - 1); idx++)
			MCU_AT_CONSOLE_send("%02X", prflCfgPtr[idx]);
		MCU_AT_CONSOLE_send("\r\n");

		return true;
	}

	{
		int16_t temp_id = 0;
		scan_param_res = (int16_t)sscanf((const char *)pu8_cmdParamString,
				(const char *)"AT+KMAC=%hd,%12[0-9A-Fa-f]",
				&temp_id,
				prflCtxtData);
		appEvt.init_prfl_ctxt.id = (enum KNS_MAC_prflId_t)temp_id;
	}
	prflCtxtCharNb = (uint16_t)strlen((const char *)prflCtxtData);

	/* Only one Tr parameter */
	if (scan_param_res == 0)
		/* Tr and pmt_mode parameters */
		return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);

	if (scan_param_res == 1) {
		/* Do not allow "AT+KMAC=<prflID>," or "AT+KMAC=<prflID>,<prflCtxt>
		 * where "somedata" is not an integer.
		 */
		if (strchr((const char *)pu8_cmdParamString, ',') != NULL)
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);

	} else if (scan_param_res == 2) {
		/** Assert when the number of characters received from AT command is bigger than
		 * authorized
		 */
		kns_assert(prflCtxtCharNb <= (sizeof(prflCtxtData) -1 ));
		u16MGR_AT_CMD_convertAsciiBinary(prflCtxtData, prflCtxtCharNb);
		memcpy(&appEvt.init_prfl_ctxt.prflCfgPtr, prflCtxtData,
			sizeof(struct KNS_MAC_BLIND_usrCfg_t));
	}

	status = KNS_Q_push(KNS_Q_DL_APP2MAC, (void *)&appEvt);
	if (status != KNS_STATUS_OK)
		return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)status);
	return true;
#endif
}

/**
 * @}
 */
