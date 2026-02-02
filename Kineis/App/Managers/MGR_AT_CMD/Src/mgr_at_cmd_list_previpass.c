// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_at_cmd_list_previpass.c
 * @author Kinéis
 * @brief subset of AT commands concerning satellite PASS predictions, usefull for Medium Acces
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
#include "mcu_at_console.h"
#include "mgr_at_cmd_list_previpass.h"

/* Functions -----------------------------------------------------------------*/

bool bMGR_AT_CMD_PREPASS_EN_cmd(uint8_t *pu8_cmdParamString, enum atcmd_type_t e_exec_mode)
{
	int16_t i16_scan_id_param_res;
	uint8_t u8_previpassSt = 0;

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MCU_AT_CONSOLE_send("+PREPASS_EN=%d\r\n", u8_previpassSt);
		return true;
	}

	i16_scan_id_param_res = (int16_t)sscanf((const char *)pu8_cmdParamString,
			(const char *)"AT+PREPASS_EN=%hhu",
			&u8_previpassSt);

	if (i16_scan_id_param_res == 1) {
		/* allow only enable/disable */
		return bMGR_AT_CMD_logSucceedMsg();
	}

	return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
}

bool bMGR_AT_CMD_UDATE_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode __attribute__((unused)))
{
	return bMGR_AT_CMD_logSucceedMsg();
}
/**
 * @}
 */
