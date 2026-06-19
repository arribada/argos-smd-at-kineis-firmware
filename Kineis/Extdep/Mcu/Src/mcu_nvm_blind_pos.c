// SPDX-License-Identifier: no SPDX license
/**
 * @file    mcu_nvm_blind_pos.c
 * @brief   Stub wrappers for the BLIND_POS MAC profile NVM resources.
 * @author  Arribada (gfo974)
 *
 * BLIND_POS is not validated on any of our SMD boards (UW_DOPPLER hardcodes
 * BASIC; the other APP/BOARD combos don't init BLIND_POS either). The lib
 * v11.1.0 only pulls these symbols in if the MAC is actually configured for
 * BLIND_POS, but the krd_fw_package_v11.1.0 ships an example
 * implementation that depends on a private `kns_utils_gnss.h` not exposed
 * in the public headers — so we ship a minimal stub instead.
 *
 * If a future board wires BLIND_POS, replace this with a proper
 * implementation backed by FLASH_USER (mind the existing flash layout
 * — see Kineis/Extdep/Mcu/Inc/mcu_flash.h).
 */

#include <stddef.h>
#include "kns_types.h"
#include "mcu_nvm_blind_pos.h"

enum KNS_status_t MCU_NVM_setPerMap(void *ptr, uint32_t byte_size)
{
	(void)ptr;
	(void)byte_size;
	return KNS_STATUS_DISABLED;
}

enum KNS_status_t MCU_NVM_getPerMap(void **ptr)
{
	if (ptr)
		*ptr = NULL;
	return KNS_STATUS_DISABLED;
}

enum KNS_status_t MCU_NVM_setPerToBlindAbacus(void *ptr, uint32_t byte_size)
{
	(void)ptr;
	(void)byte_size;
	return KNS_STATUS_DISABLED;
}

enum KNS_status_t MCU_NVM_getPerToBlindAbacus(void **ptr)
{
	if (ptr)
		*ptr = NULL;
	return KNS_STATUS_DISABLED;
}

enum KNS_status_t MCU_NVM_setUlPerCalOffset(float ulPerCal)
{
	(void)ulPerCal;
	return KNS_STATUS_DISABLED;
}

enum KNS_status_t MCU_NVM_getUlPerCalOffset(float *ulPerCal)
{
	if (ulPerCal)
		*ulPerCal = 0.0f;
	return KNS_STATUS_DISABLED;
}
