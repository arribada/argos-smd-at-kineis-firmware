// SPDX-License-Identifier: no SPDX license
/**
 * @file    mgr_at_cmd_list_v11.c
 * @brief   AT commands introduced by krd_fw v11.1.0 — local handlers.
 *
 * See mgr_at_cmd_list_v11.h for the contract.
 */

#include <stdio.h>
#include <stdbool.h>
#include "main.h"  /* BSP_HAS_* */
#include "mcu_at_console.h"
#include "mgr_at_cmd_list_v11.h"

bool bMGR_AT_CMD_DL_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode)
{
	(void)pu8_cmdParamString;
	(void)e_exec_mode;
#if !defined(BSP_HAS_DOWNLINK) || (BSP_HAS_DOWNLINK == 0)
	return bMGR_AT_CMD_logFailedMsg(ERROR_FEATURE_NOT_AVAILABLE);
#else
	/* Real DL handler would push KNS_MAC_RX_START / KNS_MAC_RX_STOP to the
	 * MAC queue here. None of our boards currently ship the SubGHz RX path
	 * with the proper credentials, so this branch is reserved for a future
	 * KIM-class board. */
	return bMGR_AT_CMD_logFailedMsg(ERROR_FEATURE_NOT_AVAILABLE);
#endif
}

bool bMGR_AT_CMD_GNSS_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode)
{
	(void)pu8_cmdParamString;
	(void)e_exec_mode;
#if !defined(BSP_HAS_GNSS) || (BSP_HAS_GNSS == 0)
	return bMGR_AT_CMD_logFailedMsg(ERROR_FEATURE_NOT_AVAILABLE);
#else
	/* Real GNSS handler would parse lat/lon/alt and pass to the BLIND_POS
	 * profile init or to a GNSS module driver. */
	return bMGR_AT_CMD_logFailedMsg(ERROR_FEATURE_NOT_AVAILABLE);
#endif
}
