// SPDX-License-Identifier: no SPDX license
/**
 * @file    mcu_aes.c
 * @brief   MCU wrapper for encryption/decryption wrappers utilities
 *
 * @author  Kineis
 * @date    Creation 2022/10/17
 * @version 1.0
 * @note
 */

/**
 * @addtogroup MCU_WRAPPERS
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "kns_types.h"
#include "aes.h"
#include "mcu_aes.h"
#include "mcu_flash.h"

/* Variables ---------------------------------------------------------*/
static aes_context ctx;

/** For security reason, the device secret key cannot appear clearly inside the
 * device memory. This key should be hidden as much as possible and be hardly
 * accessible from the outside. It is recommended to use an encryption mechanism
 * (AES, etc.) to store it.
 *
 * It is up to the integrator to store it as per its own security strategy.
 *
 * @attention Credentials are delivered by Kineis after certification of your device.
 *
 * @attention Once credentials are delivered by Kineis, it becomes your responsability.
 */
static const uint8_t test_device_secret_key[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
 					0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };

/* Functions -------------------------------------------------------------*/

enum KNS_status_t MCU_AES_128_init(uint8_t key[])
{
	if (key == NULL) {
		/* Set the AES key with the Device Secret Key */
		uint8_t device_secret_key[DSK_BYTE_LENGTH]; 
		if (MCU_AES_get_device_sec_key(device_secret_key) != KNS_STATUS_OK) {
			return KNS_STATUS_MCU_AES_ERR;
        }
		if (aes_set_key(device_secret_key, DSK_BYTE_LENGTH, &ctx) != 0)
			return KNS_STATUS_MCU_AES_ERR;
	} else
		/* Set the AES key with the given key */
		if (aes_set_key(key, DSK_BYTE_LENGTH, &ctx) != 0)
			return KNS_STATUS_MCU_AES_ERR;

	return KNS_STATUS_OK;
}

enum KNS_status_t MCU_AES_128_cbc_encrypt(const uint8_t *in, uint8_t *out, int32_t nb_block, uint8_t *iv)
{
	/** Call the AES CBC encrypt */
	if (aes_cbc_encrypt(in, out, nb_block, iv, &ctx) != EXIT_SUCCESS)
		return KNS_STATUS_MCU_AES_ERR;

	return KNS_STATUS_OK;
}

enum KNS_status_t MCU_AES_128_cbc_decrypt(const uint8_t *in, uint8_t *out, int32_t nb_block, uint8_t *iv)
{
	/** Call the AES CBC decrypt */
	if (aes_cbc_decrypt(in, out, nb_block, iv, &ctx) != EXIT_SUCCESS)
		return KNS_STATUS_MCU_AES_ERR;

	return KNS_STATUS_OK;
}

/* Provisioning-intent hook — weak default lives in mcu_nvm.c, strong impl in
 * mgr_cred.c. Declared here to mark the SECKEY write as deliberate. */
extern void MGR_CRED_noteProvisioningWrite(void);

enum KNS_status_t MCU_AES_set_device_sec_key(const uint8_t *key) {
    if (!key) return KNS_STATUS_ERROR;

    enum KNS_status_t status =
        MCU_FLASH_write(FLASH_USER_START_ADDR + FLASH_SECKEY_OFFSET, key, FLASH_SECKEY_BYTE_SIZE);
    if (status == KNS_STATUS_OK)
        MGR_CRED_noteProvisioningWrite();  /* deliberate cred write — see hook */
    return status;
}

enum KNS_status_t MCU_AES_get_device_sec_key(uint8_t *key) {
    if (!key) return KNS_STATUS_ERROR;

    enum KNS_status_t status = MCU_FLASH_read(FLASH_USER_START_ADDR + FLASH_SECKEY_OFFSET, key, FLASH_SECKEY_BYTE_SIZE);
    if (status != KNS_STATUS_OK) return status;

    bool flash_empty = true;
    for (int i = 0; i < FLASH_SECKEY_BYTE_SIZE; i++) {
        if (key[i] != 0xFF) {
            flash_empty = false;
            break;
        }
    }

    if (flash_empty) {
        memcpy(key, test_device_secret_key, FLASH_SECKEY_BYTE_SIZE);
    }

    return KNS_STATUS_OK;
}

/**
 * @}
 */
