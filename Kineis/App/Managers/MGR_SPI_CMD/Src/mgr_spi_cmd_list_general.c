// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_at_cmd_list_general.c
 * @author  Kineis
 * @brief subset of AT commands concerning general purpose (get ID, FW version, ...)
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>

#include "kns_types.h"
#include "mcu_spi_driver.h"
#include "mgr_spi_cmd_common.h"
#include "mgr_spi_cmd_list.h"
#include "mgr_spi_cmd_list_general.h"
//#include "mgr_spi_cmd_list_user_data.h"
/* @todo PRODEV-69: remove specific flag when HW setting check is implemented on all platforms */
#include "kns_cfg.h"
#include "lpm.h" // used for AT+LPM command all is hardcoded so far
#include "build_info.h"
#include "mgr_log.h"
#include "mgr_at_cmd_list_user_data.h"
#include "lpm.h"
#include "mcu_nvm.h"
#include "mcu_misc.h"


/* Functions -----------------------------------------------------------------*/

bool bMGR_SPI_CMD_READ_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	uint8_t ret = HAL_OK;
	tx->data[0] = 1;
	tx->next_req = 1;
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_PING_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{

	HAL_StatusTypeDef ret = HAL_OK;
	tx->data[0] = 1;
	tx->next_req = 1;
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_MACSTATUS_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;

	tx->data[0] = macStatus;
	tx->next_req = 1;
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		// reset Mac status after read if
		//macStatus = MAC_OK;
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_SPISTATE_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;

	// Already written in MGR_SPI_CMD_parseStreamCB to read value before to update it (proces required)
	//tx->data[0] = spiState; 
	tx->next_req = 1;
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_READSPIVERSION_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;

	tx->data[0] = spicmd_version;
	tx->next_req = 1;
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_READFIRMWARE_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;

	memcpy(&tx->data[0], uc_fw_vers_commit_id, FW_VERSION_LENGTH);  // Copy the entire fixed-length string
	tx->next_req = FW_VERSION_LENGTH;  // Total bytes to send
	ret = bMGR_SPI_DRIVER_writeread();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_READADDRESS_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;
	enum KNS_status_t status;
	uint8_t dev_addr[DEVICE_ADDR_LENGTH];
	status = KNS_CFG_getAddr(dev_addr);
	if (status != KNS_STATUS_OK)
	{
		return bMGR_SPI_CMD_logFailedMsg((enum ERROR_RETURN_T) status, tx);
	}
	memcpy(&tx->data[0], dev_addr, DEVICE_ADDR_LENGTH);
	tx->next_req = DEVICE_ADDR_LENGTH;
	ret = bMGR_SPI_DRIVER_writeread();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}
bool bMGR_SPI_CMD_WRITEADDRESSREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	// Give size
	HAL_StatusTypeDef ret = HAL_OK;
	tx->data[0] = rx->data[0];
	rx->next_req = CMD_WRITEADDRESS_WAIT_LEN;
	ret = bMGR_SPI_DRIVER_read();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_WRITEADDRESS_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;
	if (MCU_NVM_setAddr(&(rx->data[1])) != KNS_STATUS_OK)
	{
		MGR_LOG_DEBUG("Failed to write ADDR=%02x%02x%02x%02x\r\n", rx->data[1], rx->data[2],
								  rx->data[3], rx->data[4]);
        return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	} else {
		MGR_LOG_DEBUG("Set new ADDR=%02x%02x%02x%02x\r\n", rx->data[1], rx->data[2],
								  rx->data[3], rx->data[4]);
	}
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_read();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_READSECKEY_cmd(SPI_Buffer *rx, SPI_Buffer *tx) {
	HAL_StatusTypeDef ret = HAL_OK;

	uint8_t dev_seckey[DSK_BYTE_LENGTH];
	if (MCU_AES_get_device_sec_key(dev_seckey) != KNS_STATUS_OK)
	{
		return bMGR_SPI_CMD_logFailedMsg(ERROR_UNKNOWN, tx);
	}
	memcpy(&tx->data[0], dev_seckey, DSK_BYTE_LENGTH);
	tx->next_req = DSK_BYTE_LENGTH;
	ret = bMGR_SPI_DRIVER_writeread();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}

}

bool bMGR_SPI_CMD_WRITESECKEYREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx) {
	HAL_StatusTypeDef ret = HAL_OK;
	tx->data[0] = rx->data[0];
	rx->next_req = CMD_WRITESECKEY_WAIT_LEN;
	ret = bMGR_SPI_DRIVER_read();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_WRITESECKEY_cmd(SPI_Buffer *rx, SPI_Buffer *tx) {
	HAL_StatusTypeDef ret = HAL_OK;
	char sec_key_str[33];
	for (int i = 0; i < 16; i++) {
		sprintf(&sec_key_str[i * 2], "%02x", rx->data[i+1]);
	}
	sec_key_str[32] = '\0';
	if (MCU_AES_set_device_sec_key(&(rx->data[1])) != KNS_STATUS_OK)
	{
		MGR_LOG_DEBUG("Failed to write SECKEY=%s\r\n",sec_key_str);
        return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	} else {
		MGR_LOG_DEBUG("Set new SECKEY=%s\r\n", sec_key_str);
	}
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_read();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}

}


bool bMGR_SPI_CMD_READSPIMACSTATE_cmd(SPI_Buffer *rx, SPI_Buffer *tx){

	HAL_StatusTypeDef ret = HAL_OK;

	// Already written in MGR_SPI_CMD_parseStreamCB to read value before to update it (proces required)
	//tx->data[0] = spiState; 
	tx->data[1] = macStatus;
	tx->next_req = 2;
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		// reset Mac status after read if
		macStatus = MAC_OK;
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_READID_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;
	enum KNS_status_t status;

	uint32_t dev_id;
	status = KNS_CFG_getId(&dev_id);
	if (status != KNS_STATUS_OK)
	{
		return bMGR_SPI_CMD_logFailedMsg((enum ERROR_RETURN_T) status, tx);
			/* ID is printed as a number, with decimal representation.
			 * ID is stored in memory in little endian format.
			 */
	}
	tx->next_req = sizeof(dev_id);
	memcpy(&tx->data[0], &dev_id, tx->next_req);

	ret = bMGR_SPI_DRIVER_writeread();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_WRITEIDREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	// Give size
	HAL_StatusTypeDef ret = HAL_OK;
	tx->data[0] = rx->data[0];
	rx->next_req = CMD_WRITEID_WAIT_LEN;
	ret = bMGR_SPI_DRIVER_read();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_WRITEID_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;

//	uint32_t dev_id = (rx->data[1] << 24 ) |
//					  (rx->data[2] << 16) |
//					  (rx->data[3] << 8) |
//					  (rx->data[4] << 0);
	uint32_t dev_id = 0;
	memcpy(&dev_id, &(rx->data[1]), sizeof(uint32_t));
	if (MCU_NVM_setID(&dev_id) != KNS_STATUS_OK)
	{
		MGR_LOG_DEBUG("[ERROR] failed to set ID\r\n");
        return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	} else {
		MGR_LOG_DEBUG("New id : %u\r\n", dev_id);
	}
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_read();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_READSN_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;
	enum KNS_status_t status;

	uint8_t dev_sn[DEVICE_SN_LENGTH +1];

	status = KNS_CFG_getSN(dev_sn);
	if (status != KNS_STATUS_OK)
		/* TODO: add a new error code ? */
		return bMGR_SPI_CMD_logFailedMsg((enum ERROR_RETURN_T) status, tx);
	dev_sn[DEVICE_SN_LENGTH] = '\0';
	memcpy(&tx->data[0], dev_sn, sizeof(dev_sn));  // Copy the entire fixed-length string
	tx->next_req = sizeof(dev_sn);  // Total bytes to send

	ret = bMGR_SPI_DRIVER_writeread();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_READRCONF_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;

	struct KNS_CFG_radio_t radio_cfg;

	enum KNS_status_t status;
	status = KNS_CFG_getRadioInfo(&radio_cfg);
	if (status != KNS_STATUS_OK)
		return bMGR_SPI_CMD_logFailedMsg((enum ERROR_RETURN_T) status, tx);

	memcpy(&tx->data[0], &radio_cfg, sizeof(radio_cfg));  // Copy the entire fixed-length string
	tx->next_req = sizeof(radio_cfg);  // Total bytes to send

	ret = bMGR_SPI_DRIVER_writeread();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}


bool bMGR_SPI_CMD_WRITERCONFREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	// Give size
	HAL_StatusTypeDef ret = HAL_OK;
	tx->data[0] = rx->data[0];
	rx->next_req = CMD_WRITERCONF_WAIT_LEN;
	ret = bMGR_SPI_DRIVER_read();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_WRITERCONF_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;
	enum KNS_status_t status;
	char rconf_str[33];
	for (int i = 0; i < 16; i++) {
		sprintf(&rconf_str[i * 2], "%02x", rx->data[i+1]);
	}
	rconf_str[32] = '\0';

	status = KNS_CFG_setRadioInfo(&(rx->data[1]));
	if (status != KNS_STATUS_OK) {
		MGR_LOG_DEBUG("Failed to write RCONF=%s\r\n",rconf_str);
		return bMGR_SPI_CMD_logFailedMsg((enum ERROR_RETURN_T) status, tx);
	} else {
		MGR_LOG_DEBUG("Set new RCONF=%s\r\n", rconf_str);
	}

	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_read();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_SAVERCONF_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;
	enum KNS_status_t status;

	status = KNS_CFG_saveRadioInfo();
	if (status != KNS_STATUS_OK)
		return bMGR_SPI_CMD_logFailedMsg((enum ERROR_RETURN_T) status, tx);
	tx->data[0] = 1;
	tx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_READLPM_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;

	tx->data[0] = lpm_config.allowedLPMbitmap;
	tx->next_req = 1;
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_WRITELPMREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;
	txBuf.data[0] = rx->data[0];
	rxBuf.next_req = CMD_WRITELPM_WAIT_LEN; // Only waiting profile id for the moment
	ret = bMGR_SPI_DRIVER_read();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}
bool bMGR_SPI_CMD_WRITELPM_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;

	uint8_t lpm = rx->data[1];  // Extract LPM value from rxBuf.data[1]

	// Define allowed low-power modes
	const uint8_t allowedModes = LOW_POWER_MODE_NONE |
								 LOW_POWER_MODE_SLEEP |
								 LOW_POWER_MODE_STOP |
								 LOW_POWER_MODE_STANDBY |
								 LOW_POWER_MODE_SHUTDOWN;

	// Validate the received LPM value
	if ((lpm & ~allowedModes) != 0) {
		// Invalid value: contains bits outside the allowed set
		return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	}
	lpm_config.allowedLPMbitmap = lpm;
	tx->data[0] = rx->data[0];
	rx->next_req = 1; // Only waiting profile id for the moment
	ret = bMGR_SPI_DRIVER_read();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_READTCXO_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;

	uint32_t tcxo_ms;
	MCU_MISC_TCXO_get_warmup(&tcxo_ms);
	tx->next_req = sizeof(tcxo_ms);
	memcpy(&tx->data[0], &tcxo_ms, tx->next_req);

	ret = bMGR_SPI_DRIVER_writeread();
	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_WRITETCXOREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	// Give size
	HAL_StatusTypeDef ret = HAL_OK;
	tx->data[0] = rx->data[0];
	rx->next_req = CMD_WRITETCXO_WAIT_LEN;
	ret = bMGR_SPI_DRIVER_read();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_WRITETCXO_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	HAL_StatusTypeDef ret = HAL_OK;

	uint32_t tcxo_ms = 0;
	memcpy(&tcxo_ms, &(rx->data[1]), sizeof(uint32_t));
	if (tcxo_ms >30000) {
		// Invalid value: outside the allowed range
		MGR_LOG_DEBUG("[ERROR] TCXO Warmup time in ms should be between 0 to 30 000\r\n");
        return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	}
	MCU_MISC_TCXO_set_warmup(tcxo_ms);
	MGR_LOG_DEBUG("Set TCXO warmup ms to %u\r\n", tcxo_ms);
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_read();

	//Reset tx/rx state if MAC_OK
	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}
