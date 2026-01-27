// SPDX-License-Identifier: no SPDX license
/**
 * @file    mcu_nvm.c
 * @brief   MCU wrapper for any access to the non volatile memory accesses
 *
 * TODO: move Confradio into flash .
 *
 * @author  shamard
 * @date    Creation 2022/11/18
 * @version 1.0
 * @note
 */

/**
 * @addtogroup MCU_WRAPPERS
 * @{
 */

/* Includes ------------------------------------------------------------------*/

#include "kns_types.h"
#include "aes.h"
#include "mcu_aes.h"
#include "mcu_nvm.h"
#include <stdbool.h>
#include "mcu_flash.h"
#include "stm32wlxx_hal.h"
#include "mgr_log.h"
#include <string.h>
/* Variables ---------------------------------------------------------*/

/** @note The message counter be stored in a non volatile memory (flash, RTC backup reg, etc.).
 * Proposed implementation below is mapping the message_counter variable in a memory section:
 * * Ensure .msgCntSectionData section is mapped at the correct place
 * * or fill free to change this proposal as per your needs.
 *
 * It could remain in RAM as long as the device is not POWER OFF and is not entering LPM.
 * The message counter value should be recovered after each device power off/on and low power wakeup
 *
 * If you use a flash memory, be careful about the storage of this data. The message counter is
 * updated at each message transmission, so this implies a lot of erase/write cycles.
 */
__attribute__((__section__(".msgCntSectionData")))
static uint16_t message_counter = 0;

static uint16_t wakeup_counter = 0;

/** The device identifier may be stored in a secured way (encryption, etc.) */
const uint32_t test_device_id = 123456; // stored in flash


/** The device address may be stored in a secured way (encryption, etc.) */
const uint8_t test_device_addr[4] = { 0x11, 0x22, 0x33, 0x44 };

/** Radio configuration : Ensure radio configuration is valid when Kineis stack is called.
 
 * Depending on the application, before calling the stack, you can:
 * * set it in your application through MCU_NVM_setRadioConfZone()
 * * load it in RAM at startup
 * * point directly on it in flash memory...
 *
 * Several radio configurations are listed in table below. Uncomment/comment the one you want to
 * use/not use.
 */
static uint8_t radioConfZone[16] = {

#ifdef KRD_FW_LP
	/** ---- radio configuration at 22 dBm instead of 27 dBm, needed for KRD_LP platform ---- */

	/** ---- ESS4 402895000,402990000,22,LDA2 ---- */
//	0x9c, 0xf7, 0xf8, 0xde, 0xd7, 0x87, 0x44, 0x43,
//	0xe3, 0xd2, 0x73, 0x90, 0x19, 0x21, 0x47, 0xbc,
	/** ---- ESS4 402895000,402990000,22,LDK  ---- */
//	0xf5, 0xf2, 0xb0, 0xfd, 0x81, 0x85, 0x23, 0x50,
//	0x4e, 0x9e, 0x23, 0x0b, 0xb1, 0x01, 0x12, 0x24,
	/** ---- MSS 3999910000,3999960000,22,LDK  ---- */
	0x92, 0x66, 0x22, 0x00, 0xd6, 0x7d, 0xd3, 0x31,
	0x8e, 0x4d, 0xa6, 0x77, 0x25, 0x8e, 0x54, 0xc4,
#else
	/** ----ESS4 401620000,401680000,27,LDA2 ---- */
	0x3d, 0x67, 0x8a, 0xf1, 0x6b, 0x5a, 0x57, 0x20,
	0x78, 0xf3, 0xdb, 0xc9, 0x5a, 0x11, 0x04, 0xe7,

	/** ---- 401625000,401635000,27,LDA2L ---- */
//	0xbd, 0x17, 0x65, 0x35, 0xb3, 0x94, 0xa6, 0x65,
//	0xbd, 0x86, 0xf3, 0x54, 0xc5, 0xf4, 0x24, 0xfb

	/** ----ESS4 402895000,402990000,27,LDK ---- */
//	0x03, 0x92, 0x1f, 0xb1, 0x04, 0xb9, 0x28, 0x59,
//	0x20, 0x9b, 0x18, 0xab, 0xd0, 0x09, 0xde, 0x96,
	
	/** ---- 401625000,401635000,27,VLDA4 ---- */
//	0x82, 0xd0, 0x7f, 0x9d, 0x9c, 0xe0, 0x81, 0xee,
//	0x44, 0x92, 0x98, 0x36, 0x72, 0xd7, 0x54, 0x93

	/** ----MSS 402890000,402990000,22,VLDA4 ---- */
//	0x55, 0x0b, 0x4b, 0xec, 0x21, 0x00, 0x9c, 0x7a,
//	0x7b, 0x5b, 0xeb, 0xaa, 0x93, 0x7c, 0xdb, 0x41,

#endif

};

/* Device serial number */
static const uint8_t device_sn[DEVICE_SN_LENGTH] = { 'S', 'M', 'D', '_', '1', '1', '_', \
					      '_', '0', '0', '0', '0', '0', '1' };
/* Functions -------------------------------------------------------------*/

//enum KNS_status_t MCU_NVM_getMC(uint16_t *mc_ptr)
//{
//	if (!mc_ptr)
//		return KNS_STATUS_ERROR;
//
//	uint64_t full_counter = MCU_FLASH_read_msg_counter();
//    *mc_ptr = (uint16_t)(full_counter & 0xFFFF);
//    message_counter = *mc_ptr;
//    return KNS_STATUS_OK;
//
//}
//
//enum KNS_status_t MCU_NVM_setMC(uint16_t mcTmp)
//{
//	message_counter = mcTmp;
//
//	return MCU_FLASH_set_msg_counter((uint64_t)mcTmp);
//
//}
enum KNS_status_t MCU_NVM_getMC(uint16_t *mc_ptr)
{
	if (!mc_ptr)
		return KNS_STATUS_ERROR;

	uint64_t full_counter = MCU_FLASH_read_msg_counter();
	*mc_ptr = (uint16_t)(full_counter & 0xFFFF);
	message_counter = *mc_ptr;
	return KNS_STATUS_OK;
}

enum KNS_status_t MCU_NVM_setMC(uint16_t mcTmp)
{
	message_counter = mcTmp;

	return MCU_FLASH_set_msg_counter((uint64_t)mcTmp);
}


enum KNS_status_t MCU_NVM_getWUC(uint16_t *wuc_ptr)
{
	if (!wuc_ptr) 
		return KNS_STATUS_ERROR;

	uint64_t full_counter = MCU_FLASH_read_wku_counter();
    *wuc_ptr = (uint16_t)(full_counter & 0xFFFF);
    return KNS_STATUS_OK;

}

enum KNS_status_t MCU_NVM_setWUC(uint16_t wucTmp)
{
	wakeup_counter = wucTmp;
	return MCU_FLASH_set_wku_counter((uint64_t)wucTmp);

}
enum KNS_status_t MCU_NVM_getRadioConfZonePtr(void **ConfZonePtr)
{
	if (ConfZonePtr == NULL) {
		return KNS_STATUS_ERROR;
	}

	uint8_t flash_radio_conf[FLASH_RADIOCONF_BYTE_SIZE];
	enum KNS_status_t status = MCU_FLASH_read(FLASH_USER_START_ADDR + FLASH_RADIOCONF_OFFSET, flash_radio_conf, FLASH_RADIOCONF_BYTE_SIZE);

	if (status != KNS_STATUS_OK) {
		return status;
	}

	bool flash_empty = true;
	for (int i = 0; i < FLASH_RADIOCONF_BYTE_SIZE; i++) {
		if (flash_radio_conf[i] != 0xFF) {
			flash_empty = false;
			break;
		}
	}

	if (!flash_empty) {
		memcpy(radioConfZone, flash_radio_conf, FLASH_RADIOCONF_BYTE_SIZE);
	}

	*ConfZonePtr = radioConfZone;
	return KNS_STATUS_OK;
}

enum KNS_status_t MCU_NVM_setRadioConfZone(void *ConfZonePtr, uint16_t ConfZoneSize)
{
	if (ConfZonePtr == NULL || ConfZoneSize != FLASH_RADIOCONF_BYTE_SIZE) {
		return KNS_STATUS_MCU_NVM_ERR;
	}

	// Write to flash
	enum KNS_status_t status = MCU_FLASH_write(FLASH_USER_START_ADDR + FLASH_RADIOCONF_OFFSET, ConfZonePtr, FLASH_RADIOCONF_BYTE_SIZE);

	if (status != KNS_STATUS_OK) {
		return status;
	}

	return KNS_STATUS_OK;
}

enum KNS_status_t MCU_NVM_saveRadioConfZone(void)
{
	return KNS_STATUS_OK;
}


enum KNS_status_t MCU_NVM_getID(uint32_t *id)
{
    if (id == NULL) {
        return KNS_STATUS_ERROR;
    }
    enum KNS_status_t status = KNS_STATUS_OK;

    status = MCU_FLASH_read(FLASH_USER_START_ADDR + FLASH_ID_OFFSET, id, FLASH_ID_BYTE_SIZE);

    if (*id == 0xFFFFFFFF) {
		*id = test_device_id;
	}

    return (status);
}

enum KNS_status_t MCU_NVM_setID(uint32_t *id)
{
    if (id == NULL) {
        return KNS_STATUS_MCU_NVM_ERR;
    }

	return(MCU_FLASH_write(FLASH_USER_START_ADDR + FLASH_ID_OFFSET, id, FLASH_ID_BYTE_SIZE));
}


/**
 * @brief Retrieves the stored address into a byte array.
 *
 * @param addr Pointer to a 4-byte array where the address will be stored.
 * @return KNS_status_t Status of the operation.
 */


enum KNS_status_t MCU_NVM_getAddr(uint8_t addr[])
{
    if (!addr) return KNS_STATUS_ERROR; // Vérification des pointeurs

    enum KNS_status_t status = KNS_STATUS_OK;
    status = MCU_FLASH_read(FLASH_USER_START_ADDR + FLASH_ADDR_OFFSET, addr, FLASH_ADDR_BYTE_SIZE);

    if (addr[0] == 0xFF && addr[1] == 0xFF && addr[2] == 0xFF && addr[3] == 0xFF) {
		memcpy(addr, test_device_addr, 4);
	}

	return status;
}

enum KNS_status_t MCU_NVM_setAddr(uint8_t addr[])
{
    return MCU_FLASH_write(FLASH_USER_START_ADDR + FLASH_ADDR_OFFSET, addr, FLASH_ADDR_BYTE_SIZE);
}
enum KNS_status_t MCU_NVM_getSN(uint8_t sn[])
{
    uint16_t i;

    for (i = 0 ; i < DEVICE_SN_LENGTH ; i++)
        sn[i] = device_sn[i];

    return KNS_STATUS_OK;
}

/**
 * @}
 */
