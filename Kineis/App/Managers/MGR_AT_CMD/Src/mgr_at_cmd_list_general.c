// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_at_cmd_list_general.c
 * @author  Kineis
 * @brief subset of AT commands concerning general purpose (get ID, FW version, ...)
 */

/**
 * @addtogroup MGR_AT_CMD
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "kns_types.h"
#include "mgr_at_cmd_common.h"
#include "mgr_at_cmd_list.h"
#include "mgr_at_cmd_list_general.h"
#include "mgr_at_cmd_list_user_data.h"
#include "mcu_at_console.h"
/* @todo PRODEV-68: remove specific flag when HW setting check is implemented on all platforms */
#if defined(KRD_FW_MP) ||  defined(KRD_FW_LP)
#include "mcu_misc.h" // use to check RF HW setting vers radio confoguration
#endif
#include "kns_cfg.h"
#include "lpm.h" // used for AT+LPM command all is hardcoded so far
#include "build_info.h"
#include "mgr_log.h"
#include "mcu_nvm.h"
#include "mcu_aes.h"


/* Functions -----------------------------------------------------------------*/

bool bMGR_AT_CMD_VERSION_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	uint8_t idx;

	if (e_exec_mode == ATCMD_ACTION_MODE) {
		MGR_LOG_VERBOSE("[ERROR] Action mode is unauthorized for this AT cmd\r\n");
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}

	MCU_AT_CONSOLE_send("+VERSION=%s", atcmd_version);
	for (idx=0; idx < ATCMD_MAX_COUNT; idx++) {
		MCU_AT_CONSOLE_send(",%s", cas_atcmd_list_array[idx].pu8_cmdNameString);

	}
	MCU_AT_CONSOLE_send("\r\n");

	return bMGR_AT_CMD_logSucceedMsg();
}

bool bMGR_AT_CMD_PING_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		MGR_LOG_VERBOSE("[ERROR] Action mode is unauthorized for this AT cmd\r\n");
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}
	return bMGR_AT_CMD_logSucceedMsg();
}

bool bMGR_AT_CMD_FW_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		MGR_LOG_VERBOSE("[ERROR] Action mode is unauthorized for this AT cmd\r\n");
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}

	MCU_AT_CONSOLE_send("+FW=%s\r\n", uc_fw_vers_commit_id);

	return bMGR_AT_CMD_logSucceedMsg();
}

bool bMGR_AT_CMD_SECKEY_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	uint8_t sec_key[DSK_BYTE_LENGTH];
	uint16_t nbBits;

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		if (MCU_AES_get_device_sec_key(sec_key) != KNS_STATUS_OK)
			/* TODO: add a new error code ? */
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		char sec_key_str[33];
		for (int i = 0; i < 16; i++) {
			sprintf(&sec_key_str[i * 2], "%02x", sec_key[i]);
		}
		sec_key_str[32] = '\0';
		MCU_AT_CONSOLE_send("+SECKEY=%s\r\n", sec_key_str);

		return bMGR_AT_CMD_logSucceedMsg();
	} else if (e_exec_mode == ATCMD_ACTION_MODE) {
		while(pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] == '\r' ||
				pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] == '\n')
		{
			pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] = '\0';
		}
			nbBits = u16MGR_AT_CMD_convertAsciiBinary(pu8_cmdParamString + 10,
				strlen((char*)(pu8_cmdParamString + 10)));
			
			if (nbBits != DSK_BYTE_LENGTH*8)
			{
				MGR_LOG_DEBUG("%s::nbBits error %u should be %u\r\n", __func__, nbBits, DSK_BYTE_LENGTH*8);
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			
			if (MCU_AES_set_device_sec_key(pu8_cmdParamString + 10) != KNS_STATUS_OK)
			{
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			return bMGR_AT_CMD_logSucceedMsg();	
	} else {
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}
}

bool bMGR_AT_CMD_ADDR_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	uint8_t dev_addr[DEVICE_ADDR_LENGTH];
	uint16_t nbBits;

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		status = KNS_CFG_getAddr(dev_addr);
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		MCU_AT_CONSOLE_send("+ADDR=%02x%02x%02x%02x\r\n", dev_addr[0], dev_addr[1],
								  dev_addr[2], dev_addr[3]);

		return bMGR_AT_CMD_logSucceedMsg();
	} else if (e_exec_mode == ATCMD_ACTION_MODE) {
		while(pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] == '\r' ||
				pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] == '\n')
		{
			pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] = '\0';
		}
			nbBits = u16MGR_AT_CMD_convertAsciiBinary(pu8_cmdParamString + 8,
				strlen((char*)(pu8_cmdParamString + 8)));
			
			if ((nbBits != 32) && (nbBits != 28))
			{
				MGR_LOG_DEBUG("%s::nbBits error %u\r\n", __func__, nbBits);
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			
			if (MCU_NVM_setAddr(pu8_cmdParamString + 8) != KNS_STATUS_OK)
			{
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			return bMGR_AT_CMD_logSucceedMsg();	
	} else {
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}
}
bool bMGR_AT_CMD_ID_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	uint32_t dev_id;
	uint16_t nbBits;
	uint16_t nbChar;
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		status = KNS_CFG_getId(&dev_id);
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		/* ID is printed as a number, with decimal representation.
		 * ID is stored in memory in little endian format.
		 */
		MCU_AT_CONSOLE_send("+ID=%d\r\n", dev_id);

		return bMGR_AT_CMD_logSucceedMsg();
	} else if (e_exec_mode == ATCMD_ACTION_MODE) {
		while(pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] == '\r' ||
				pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] == '\n')
		{
			pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] = '\0';
		}

			//MGR_LOG_DEBUG("%s",__func__);
			nbChar = strlen((char*)pu8_cmdParamString+6);
			if (nbChar > 7)
			{
				MGR_LOG_DEBUG("[ERROR] Invalid ID length\r\n");	
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			
			nbBits = u16MGR_AT_CMD_convertAsciiToInt32(pu8_cmdParamString + 6, &dev_id);

			if ((nbBits > 32) && (nbBits < 1))
			{
				MGR_LOG_DEBUG("[ERROR] Invalid ID length conversion\r\n");
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}

			
			if (MCU_NVM_setID(&dev_id) != KNS_STATUS_OK)
			{
				MGR_LOG_DEBUG("[ERROR] failed to set ID\r\n");
				return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
			}
			return bMGR_AT_CMD_logSucceedMsg();	
	} else {
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}
}

bool bMGR_AT_CMD_SN_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	uint8_t dev_sn[DEVICE_SN_LENGTH + 1];

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		status = KNS_CFG_getSN(dev_sn);
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		dev_sn[DEVICE_SN_LENGTH] = '\0';
		MCU_AT_CONSOLE_send("+SN=%s\r\n", dev_sn);

		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_RCONF_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	struct KNS_CFG_radio_t radio_cfg;
	uint8_t modulation[8];
	uint16_t nbBits;
#if defined(KRD_FW_MP) ||  defined(KRD_FW_LP)
	struct rfSettings_t hwSettings;
#endif

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		status = KNS_CFG_getRadioInfo(&radio_cfg);
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		if (radio_cfg.modulation == KNS_TX_MOD_LDA2)
			strcpy((char*)modulation, "LDA2");
		else if (radio_cfg.modulation == KNS_TX_MOD_LDA2L)
			strcpy((char*)modulation, "LDA2L");
		else if (radio_cfg.modulation == KNS_TX_MOD_VLDA4)
			strcpy((char*)modulation, "VLDA4");
		else if (radio_cfg.modulation == KNS_TX_MOD_HDA4)
			strcpy((char*)modulation, "HDA4");
		else if (radio_cfg.modulation == KNS_TX_MOD_LDK)
			strcpy((char*)modulation, "LDK");
		else
			strcpy((char*)modulation, "UNKNOWN");
		MCU_AT_CONSOLE_send("+RCONF=%u,%u,%d,%s\r\n", radio_cfg.min_frequency,
			radio_cfg.max_frequency, radio_cfg.rf_level, modulation);

		/** @todo PRODEV-68: check radio conf versus HW settings for other platforms */
#if defined(KRD_FW_MP) ||  defined(KRD_FW_LP)
		if (MCU_MISC_getSettingsHwRf(radio_cfg.rf_level, &hwSettings) == KNS_STATUS_OK)
			return bMGR_AT_CMD_logSucceedMsg();
		return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
#else
		return bMGR_AT_CMD_logSucceedMsg();
#endif
	}
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		while(pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] == '\r' ||
			pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] == '\n')
			pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] = '\0';
		nbBits = u16MGR_AT_CMD_convertAsciiBinary(pu8_cmdParamString + 9,
			strlen((char*)(pu8_cmdParamString + 9)));

		if (nbBits != 128)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) KNS_STATUS_RADIO_CONF_BAD);

		status = KNS_CFG_setRadioInfo(pu8_cmdParamString + 9);
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_SAVE_RCONF_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	if (e_exec_mode == ATCMD_ACTION_MODE) {
		status = KNS_CFG_saveRadioInfo();
		if (status != KNS_STATUS_OK)
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		return bMGR_AT_CMD_logSucceedMsg();
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_LPM_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	int16_t scanParamRes;
	uint16_t lpm; /** @todo PRODEV-88:need to keep uint16_t remove compilation error.
	 	       * leads to hardfault error if using uint8_t
	 	       */

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MCU_AT_CONSOLE_send("+LPM=0x%X\r\n", lpm_config.allowedLPMbitmap);
		return bMGR_AT_CMD_logSucceedMsg();
	}
	if (e_exec_mode == ATCMD_ACTION_MODE) {

		/** Extract USER DATA from pu8_cmdParamString */
		scanParamRes = sscanf((const char *)pu8_cmdParamString, (const char *)"AT+LPM=0x%hX", &lpm);
		if (scanParamRes == 1) {
			lpm &= LOW_POWER_MODE_NONE | LOW_POWER_MODE_SLEEP | LOW_POWER_MODE_STOP |
				LOW_POWER_MODE_STANDBY | LOW_POWER_MODE_SHUTDOWN;
			lpm_config.allowedLPMbitmap = (uint8_t)lpm;
			bMGR_AT_CMD_logSucceedMsg();
		}
		if (scanParamRes != 1)
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
	}

	return false;
}

bool bMGR_AT_CMD_MC_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	enum KNS_status_t status;
	uint16_t mc;

	if (e_exec_mode == ATCMD_STATUS_MODE) {
		status = KNS_CFG_getMC(&mc);
		if (status != KNS_STATUS_OK) {
			return bMGR_AT_CMD_logFailedMsg((enum ERROR_RETURN_T) status);
		}
		
		MCU_AT_CONSOLE_send("+MC=%d\r\n", mc);
		return bMGR_AT_CMD_logSucceedMsg();
	} else if(e_exec_mode == ATCMD_ACTION_MODE) {
		if (sscanf((char*)pu8_cmdParamString, "%*[^=]= %hu", &mc) != 1)
		{
			MGR_LOG_DEBUG("Missing parameter: AT+MC=VALUE\r\n");
			return bMGR_AT_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT);
		} else {
			if (KNS_CFG_setMC(mc) == KNS_STATUS_OK)
			{
				MGR_LOG_DEBUG("+MC=%u\r\n", mc);
				return bMGR_AT_CMD_logSucceedMsg();
			} else {
				MGR_LOG_DEBUG("Failed to update MC=%u\r\n", mc);
			}
		}
	}
	return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
}

bool bMGR_AT_CMD_TCXO_cmd(uint8_t *pu8_cmdParamString __attribute__((unused)),
	enum atcmd_type_t e_exec_mode)
{
	uint32_t tcxo_ms;
	if (e_exec_mode == ATCMD_STATUS_MODE) {
		MCU_MISC_TCXO_get_warmup(&tcxo_ms);
		MCU_AT_CONSOLE_send("+TCXO=%d\r\n", tcxo_ms);
		return bMGR_AT_CMD_logSucceedMsg();
	} else if (e_exec_mode == ATCMD_ACTION_MODE) {
		uint16_t nbChar;
		while(pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] == '\r' ||
				pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] == '\n')
		{
			pu8_cmdParamString[strlen((char*)pu8_cmdParamString) - 1] = '\0';
		}

			MGR_LOG_DEBUG("%s",__func__);
			nbChar = strlen((char*)pu8_cmdParamString+11);
			if (nbChar > 5)
			{
				MGR_LOG_DEBUG("[ERROR] Invalid TCXO MS length, max value is 30 000ms\r\n");	
				return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
			}
			
			u16MGR_AT_CMD_convertAsciiToInt32(pu8_cmdParamString + 11, &tcxo_ms);

			if (tcxo_ms > 30000)
			{
				MGR_LOG_DEBUG("[ERROR] Invalid ID TCXO Warmup time, should be less than 30 000ms\r\n");
				return bMGR_AT_CMD_logFailedMsg(ERROR_INCOMPATIBLE_VALUE);
			}

			MCU_MISC_TCXO_set_warmup(tcxo_ms);
			return bMGR_AT_CMD_logSucceedMsg();	
	} else {
		return bMGR_AT_CMD_logFailedMsg(ERROR_UNKNOWN_AT_CMD);
	}
}

/**
 * @}
 */
