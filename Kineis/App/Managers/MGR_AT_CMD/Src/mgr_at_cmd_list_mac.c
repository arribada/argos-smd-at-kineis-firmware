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
	/* UW_DOPPLER is locked to MAC profile BASIC at compile time (see the
	 * compile-time guard at the top of kns_app_uw_doppler.c). The AT
	 * command runtime path enforces the same rule:
	 *   - status query → report BASIC + zero-filled context
	 *   - action with id ∈ {NONE=0, BASIC=1} → ack (id=0 is the rare
	 *     "reset profile" sequence the GUI sometimes sends, harmless
	 *     because we don't actually re-init the MAC here)
	 *   - any other id (BLIND=2, SATDET=3, BLIND_POS=4) → refuse loud,
	 *     emit a WARN log so the operator knows a non-validated profile
	 *     was attempted and the request was discarded.
	 *
	 * Refusing at the AT layer prevents an operator who's reading stale
	 * documentation from "trying BLIND to see what happens" — the
	 * app-side scheduler would conflict with BLIND retx and silently
	 * waste battery on underwater TX. */
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MCU_AT_CONSOLE_send("+KMAC=1,0000000000\r\n");
		return bMGR_AT_CMD_logSucceedMsg();
	}

	int16_t requested_id = -1;
	if (pu8_cmdParamString != NULL) {
		(void)sscanf((const char *)pu8_cmdParamString,
		             "AT+KMAC=%hd", &requested_id);
	}

	if (requested_id == (int16_t)KNS_MAC_PRFL_NONE ||
	    requested_id == (int16_t)KNS_MAC_PRFL_BASIC) {
		return bMGR_AT_CMD_logSucceedMsg();
	}

	MGR_LOG_WARN("[KMAC] Profile %d refused — UW_DOPPLER only "
	             "validates BASIC. Use a non-UW_DOPPLER build for "
	             "BLIND/SATDET/BLIND_POS.\r\n", requested_id);
	return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
#else
	int16_t scan_param_res;
	uint16_t prflCtxtCharNb;
	enum KNS_status_t status = KNS_STATUS_OK;
	struct KNS_MAC_prflInfo_t prfl_info;
	uint8_t *prflCfgPtr;
	uint16_t idx;

	/* v11.1.0 BLIND_usrCfg_t grew to 7 bytes (added per_offset field), so the
	 * ASCII buffer is now 14 hex chars + NUL. Memset to 0 so a GUI still
	 * sending the legacy 12-char payload yields per_offset=0 (compatible
	 * with v10 behaviour).
	 */
	uint8_t prflCtxtData[15] = {0};
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
				(const char *)"AT+KMAC=%hd,%14[0-9A-Fa-f]",
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

#include "kns_cfg.h"

bool bMGR_AT_CMD_KCFG_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		union KNS_CFG_bitmap_t cfg = KNS_CFG_getCfg();
		MCU_AT_CONSOLE_send("+KCFG=%lu\r\n", (unsigned long)cfg.raw);
		return bMGR_AT_CMD_logSucceedMsg();
	}

	uint32_t raw = 0;
	if (pu8_cmdParamString == NULL ||
	    sscanf((const char *)pu8_cmdParamString, "AT+KCFG=%lu",
	           (unsigned long *)&raw) != 1)
		return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);

	union KNS_CFG_bitmap_t cfg = { .raw = raw };
	enum KNS_status_t st = KNS_CFG_setCfg(cfg);
	if (st != KNS_STATUS_OK)
		return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T)st);
	return bMGR_AT_CMD_logSucceedMsg();
}

bool bMGR_AT_CMD_KEVT_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode)
{
	(void)pu8_cmdParamString;
	if (e_exec_mode != ATCMD_STATUS_MODE)
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);

	/* Best-effort drain of the MAC-to-APP queue. On UW_DOPPLER builds the
	 * app's own scheduler is already consuming the queue, so this almost
	 * always emits zero events — that's fine. The GUI uses this for diag. */
	struct KNS_MAC_srvcEvt_t evt;
	unsigned count = 0;
	while (KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&evt) == KNS_STATUS_OK) {
		MCU_AT_CONSOLE_send("+KEVT=%u,%u,%u\r\n",
		                    (unsigned)evt.id, (unsigned)evt.status,
		                    (unsigned)evt.app_evt);
		count++;
		if (count >= 32u) /* safety bound */
			break;
	}
	MCU_AT_CONSOLE_send("+KEVT=count,%u\r\n", count);
	return bMGR_AT_CMD_logSucceedMsg();
}

/**
 * @}
 */
