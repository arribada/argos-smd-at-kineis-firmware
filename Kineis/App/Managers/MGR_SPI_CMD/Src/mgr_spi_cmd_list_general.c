// SPDX-License-Identifier: no SPDX license
/**
 * @file mgr_spi_cmd_list_general.c
 * @author Kineis
 * @brief Subset of SPI commands for general purpose operations (get ID, FW version, etc.)
 *
 * This module implements SPI command handlers for device management operations including:
 * - Device identification (ID, address, serial number)
 * - Firmware version information
 * - Configuration read/write (radio config, LPM, TCXO)
 * - Security key management
 * - DFU mode entry
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
#include "mcu_spi_driver.h"
#include "mgr_spi_protocol.h"
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
#include "mcu_flash.h"
#include "stm32wlxx_hal.h"

/* DFU request function (defined in main.c) */
#include "main.h"

/* TCXO warmup limits - MEDIUM FIX: Replace magic number with constant */
#define TCXO_MAX_WARMUP_MS      30000UL

/* External variable for MAC status acknowledgment tracking */
extern bool macStatusAcknowledged;

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

	/* HIGH FIX: Initialize tx->data[0] - was sending uninitialized data */
	tx->data[0] = (uint8_t)spiState;
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

	uint16_t ver_len = (uint16_t)strlen(spicmd_version);
	if (ver_len > (SPI_TRANSACTION_SIZE - SPI_FRAME_MIN_SIZE))
		ver_len = (SPI_TRANSACTION_SIZE - SPI_FRAME_MIN_SIZE);
	memcpy(&tx->data[0], spicmd_version, ver_len);
	tx->next_req = ver_len;
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
	(void)rx;  /* Unused parameter - command only returns data */
	HAL_StatusTypeDef ret = HAL_OK;

	uint16_t fw_len = (uint16_t)strlen(uc_fw_vers_commit_id);
	if (fw_len > (SPI_TRANSACTION_SIZE - SPI_FRAME_MIN_SIZE))
		fw_len = (SPI_TRANSACTION_SIZE - SPI_FRAME_MIN_SIZE);
	memcpy(&tx->data[0], uc_fw_vers_commit_id, fw_len);
	tx->next_req = fw_len;
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
	(void)rx;  /* Unused parameter - command only returns data */
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
	(void)rx;  /* Unused in pipelined protocol - command has no payload */
	HAL_StatusTypeDef ret = HAL_OK;

	/* Send ACK response (OK with no data) for pipelined protocol
	 * This tells master that slave is ready to receive data in next phase */
	tx->data[0] = 1;  /* OK indicator */
	tx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

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

	/* Validate received data size: cmd (1) + addr (4) = 5 bytes minimum */
	if (rx->size < CMD_WRITEADDRESS_WAIT_LEN) {
		MGR_LOG_DEBUG("[ERROR] ADDR size invalid: received %u, expected %u\r\n",
		              rx->size, CMD_WRITEADDRESS_WAIT_LEN);
		return bMGR_SPI_CMD_logFailedMsg(ERROR_MISSING_PARAMETERS, tx);
	}

	/* Validate data is not garbage (all zeros or all 0xFF - typical SPI desync patterns) */
	bool all_zeros = true;
	bool all_ff = true;
	for (int i = 0; i < DEVICE_ADDR_LENGTH; i++) {
		if (rx->data[i + 1] != 0x00) all_zeros = false;
		if (rx->data[i + 1] != 0xFF) all_ff = false;
	}
	if (all_zeros || all_ff) {
		MGR_LOG_DEBUG("[ERROR] ADDR data invalid: all %s (SPI desync?)\r\n",
		              all_zeros ? "zeros" : "0xFF");
		return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	}

	if (MCU_NVM_setAddr(&(rx->data[1])) != KNS_STATUS_OK)
	{
		MGR_LOG_DEBUG("Failed to write ADDR=%02x%02x%02x%02x\r\n", rx->data[1], rx->data[2],
								  rx->data[3], rx->data[4]);
        return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	} else {
		MGR_LOG_DEBUG("Set new ADDR=%02x%02x%02x%02x\r\n", rx->data[1], rx->data[2],
								  rx->data[3], rx->data[4]);
	}

	/* Send success response */
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

bool bMGR_SPI_CMD_READSECKEY_cmd(SPI_Buffer *rx, SPI_Buffer *tx) {
	(void)rx;  /* Unused parameter - command only returns data */
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
	(void)rx;  /* Unused in pipelined protocol */
	HAL_StatusTypeDef ret = HAL_OK;

	/* Send ACK response for pipelined protocol */
	tx->data[0] = 1;  /* OK indicator */
	tx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

	if (ret == HAL_OK)
	{
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_WRITESECKEY_cmd(SPI_Buffer *rx, SPI_Buffer *tx) {
	HAL_StatusTypeDef ret = HAL_OK;

	/* Validate received data size: cmd (1) + seckey (16) = 17 bytes minimum */
	if (rx->size < CMD_WRITESECKEY_WAIT_LEN) {
		MGR_LOG_DEBUG("[ERROR] SECKEY size invalid: received %u, expected %u\r\n",
		              rx->size, CMD_WRITESECKEY_WAIT_LEN);
		return bMGR_SPI_CMD_logFailedMsg(ERROR_MISSING_PARAMETERS, tx);
	}

	/* Validate data is not garbage (all zeros or all 0xFF - typical SPI desync patterns) */
	bool all_zeros = true;
	bool all_ff = true;
	for (int i = 0; i < DSK_BYTE_LENGTH; i++) {
		if (rx->data[i + 1] != 0x00) all_zeros = false;
		if (rx->data[i + 1] != 0xFF) all_ff = false;
	}
	if (all_zeros || all_ff) {
		MGR_LOG_DEBUG("[ERROR] SECKEY data invalid: all %s (SPI desync?)\r\n",
		              all_zeros ? "zeros" : "0xFF");
		return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	}

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

	/* Send success response */
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


bool bMGR_SPI_CMD_READSPIMACSTATE_cmd(SPI_Buffer *rx, SPI_Buffer *tx){

	HAL_StatusTypeDef ret = HAL_OK;

	/* HIGH FIX: Initialize tx->data[0] - was sending uninitialized data */
	tx->data[0] = (uint8_t)spiState;
	tx->data[1] = (uint8_t)macStatus;
	tx->next_req = 2;
	rx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

	if (ret == HAL_OK)
	{
		/* Only reset non-terminal statuses immediately.
		 * Terminal statuses (TX_DONE, TX_TIMEOUT, etc.) are kept until:
		 * - A new TX command is initiated
		 * - This prevents losing TX_DONE if SPI desync causes missed responses
		 */
		switch (macStatus) {
		case MAC_TX_DONE:
		case MAC_TXACK_DONE:
		case MAC_TX_TIMEOUT:
		case MAC_TXACK_TIMEOUT:
		case MAC_ERROR:
			/* Terminal status - mark as acknowledged but don't reset yet */
			macStatusAcknowledged = true;
			/* Status will be reset when new TX is initiated */
			break;
		case MAC_TX_IN_PROGRESS:
			/* TX in progress - don't reset, keep polling */
			break;
		default:
			/* Non-terminal status - reset immediately */
			macStatus = MAC_OK;
			break;
		}
		return true;
	} else {
		return false;
	}
}

bool bMGR_SPI_CMD_READID_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	(void)rx;  /* Unused parameter - command only returns data */
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
	(void)rx;  /* Unused in pipelined protocol */
	HAL_StatusTypeDef ret = HAL_OK;

	/* Send ACK response for pipelined protocol */
	tx->data[0] = 1;  /* OK indicator */
	tx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

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

	/* Validate received data size: cmd (1) + id (4) = 5 bytes minimum */
	if (rx->size < CMD_WRITEID_WAIT_LEN) {
		MGR_LOG_DEBUG("[ERROR] ID size invalid: received %u, expected %u\r\n",
		              rx->size, CMD_WRITEID_WAIT_LEN);
		return bMGR_SPI_CMD_logFailedMsg(ERROR_MISSING_PARAMETERS, tx);
	}

	uint32_t dev_id = 0;
	memcpy(&dev_id, &(rx->data[1]), sizeof(uint32_t));

	/* Validate data is not garbage (all zeros or all 0xFF - typical SPI desync patterns) */
	if (dev_id == 0x00000000 || dev_id == 0xFFFFFFFF) {
		MGR_LOG_DEBUG("[ERROR] ID data invalid: 0x%08lX (SPI desync?)\r\n", dev_id);
		return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	}

	if (MCU_NVM_setID(&dev_id) != KNS_STATUS_OK)
	{
		MGR_LOG_DEBUG("[ERROR] failed to set ID\r\n");
        return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	} else {
		MGR_LOG_DEBUG("New id : %u\r\n", dev_id);
	}

	/* Send success response */
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

bool bMGR_SPI_CMD_READSN_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	(void)rx;  /* Unused parameter - command only returns data */
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
	(void)rx;  /* Unused parameter - command only returns data */
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
	(void)rx;  /* Unused in pipelined protocol */
	HAL_StatusTypeDef ret = HAL_OK;

	/* Send ACK response for pipelined protocol */
	tx->data[0] = 1;  /* OK indicator */
	tx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

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

	/* Validate received data size: cmd (1) + rconf (16) = 17 bytes minimum */
	if (rx->size < CMD_WRITERCONF_WAIT_LEN) {
		MGR_LOG_DEBUG("[ERROR] RCONF size invalid: received %u, expected %u\r\n",
		              rx->size, CMD_WRITERCONF_WAIT_LEN);
		return bMGR_SPI_CMD_logFailedMsg(ERROR_MISSING_PARAMETERS, tx);
	}

	/* Validate data is not garbage (all zeros or all 0xFF - typical SPI desync patterns) */
	bool all_zeros = true;
	bool all_ff = true;
	for (int i = 0; i < 16; i++) {
		if (rx->data[i + 1] != 0x00) all_zeros = false;
		if (rx->data[i + 1] != 0xFF) all_ff = false;
	}
	if (all_zeros || all_ff) {
		MGR_LOG_DEBUG("[ERROR] RCONF data invalid: all %s (SPI desync?)\r\n",
		              all_zeros ? "zeros" : "0xFF");
		return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	}

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

	/* Send success response */
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

bool bMGR_SPI_CMD_SAVERCONF_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	(void)rx;  /* Unused parameter - command only returns status */
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
	(void)rx;  /* Unused in pipelined protocol */
	HAL_StatusTypeDef ret = HAL_OK;

	/* Send ACK response for pipelined protocol */
	tx->data[0] = 1;  /* OK indicator */
	tx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

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

	/* Send success response */
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

bool bMGR_SPI_CMD_READTCXO_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	(void)rx;  /* Unused parameter - command only returns data */
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
	(void)rx;  /* Unused in pipelined protocol */
	HAL_StatusTypeDef ret = HAL_OK;

	/* Send ACK response for pipelined protocol */
	tx->data[0] = 1;  /* OK indicator */
	tx->next_req = 1;
	ret = bMGR_SPI_DRIVER_writeread();

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
	if (tcxo_ms > TCXO_MAX_WARMUP_MS) {
		/* Invalid value: outside the allowed range */
		MGR_LOG_DEBUG("[ERROR] TCXO Warmup time in ms should be between 0 to %lu\r\n",
		              TCXO_MAX_WARMUP_MS);
        return bMGR_SPI_CMD_logFailedMsg(ERROR_PARAMETER_FORMAT, tx);
	}
	MCU_MISC_TCXO_set_warmup(tcxo_ms);
	MGR_LOG_DEBUG("Set TCXO warmup ms to %u\r\n", tcxo_ms);

	/* Send success response */
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

bool bMGR_SPI_CMD_DFU_ENTER_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	(void)rx;  /* Unused parameter - command only triggers reset */

	MGR_LOG_DEBUG("=== DFU_ENTER (0x3F) RECEIVED! ===\r\n");

	/* Send acknowledgment before reset */
	tx->data[0] = 1;  /* OK response */
	tx->next_req = 1;
	bMGR_SPI_DRIVER_writeread();

	/* Wait for Zephyr to clock out the response (NOP transaction takes ~30-50ms) */
	HAL_Delay(100);

	MGR_LOG_DEBUG("DFU: Calling request_dfu_mode(SPI)...\r\n");

	/* Request DFU mode with SPI protocol forced (skip UART/SPI detection race) */
	request_dfu_mode(DFU_PROTO_SPI);

	/* Should never reach here */
	return true;
}

bool bMGR_SPI_CMD_READRCONFRAW_cmd(SPI_Buffer *rx, SPI_Buffer *tx)
{
	(void)rx;  /* Unused parameter - command only returns data */
	HAL_StatusTypeDef ret = HAL_OK;

	uint8_t raw_rconf[FLASH_RADIOCONF_BYTE_SIZE];

	/* Read raw bytes directly from flash */
	enum KNS_status_t status = MCU_FLASH_read(
		FLASH_USER_START_ADDR + FLASH_RADIOCONF_OFFSET,
		raw_rconf,
		FLASH_RADIOCONF_BYTE_SIZE);

	if (status != KNS_STATUS_OK) {
		return bMGR_SPI_CMD_logFailedMsg((enum ERROR_RETURN_T)status, tx);
	}

	/* Copy raw bytes to TX buffer */
	memcpy(&tx->data[0], raw_rconf, FLASH_RADIOCONF_BYTE_SIZE);
	tx->next_req = FLASH_RADIOCONF_BYTE_SIZE;

	ret = bMGR_SPI_DRIVER_writeread();

	if (ret == HAL_OK) {
		return true;
	} else {
		return false;
	}
}
