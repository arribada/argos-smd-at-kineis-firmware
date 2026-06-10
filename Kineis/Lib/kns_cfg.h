/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    kns_cfg.h
 * @brief   Device communication driver for Kineis radio frequence
 * @author  Kineis
 *
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Kineis. All rights reserved.</center></h2>
 *
 * This software component is licensed by Kineis under Ultimate Liberty license, the "License";
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * * kineis.com
 *
 */

/**
 * @page kns_cfg_page KNS_CFG: Kineis cofiguration library
 *
 * This page is presenting some Kineis stack configuration services (\ref KNS_CFG)
 *
 * Its purpose is to:
 * * Provide information such as ID, address, serail number, radio configuration
 *
 * @note As those services reply on data stored in non volatile memory, there may be a dependency on
 * MCU wrappers implemented by the integrator.
 *
 * @section kns_app_subpages Sub-pages
 *
 * * @subpage mcu_nvm_page
 */

/**
 * @addtogroup KNS_CFG
 * @brief Kineis RF Transceiver module. (refer to @ref kns_cfg_page for general description).
 * @{
 */

#ifndef KNS_CFG_H
#define KNS_CFG_H

/* Includes ------------------------------------------------------------------------------------ */

#include <stdbool.h>
#include <stdint.h>
#include "kns_types.h"

#pragma GCC visibility push(default)

/* Global defines -------------------------------------------------------------------------------*/
#define DEVICE_ADDR_LENGTH         4
#define DEVICE_SN_LENGTH          14

/* Types --------------------------------------------------------------------------------------- */

/** @brief Bitmap listing stack configuration bits
 */
union KNS_CFG_bitmap_t {
	struct {
		uint32_t suspend:1; // suspend L1 timer. Note exclusive with resume
		uint32_t resume:1;  // resume L1 timer. Note, exclusive with suspend
	};
	uint32_t raw;
};

/* Structures ---------------------------------------------------------------------------------- */

/**
 * @struct KNS_CFG_radio_t
 * @brief radio configuration parameters of Kineis services, may depend on platform needs
 */
struct KNS_CFG_radio_t {
	uint32_t min_frequency;
	uint32_t max_frequency;
	int8_t rf_level;
	enum KNS_tx_mod_t modulation;
};

/* Function prototypes ------------------------------------------------------------------------- */

/**
 * @brief Get the value of the Message Counter
 *
 * @param[out] mc_ptr : pointer where the Message Counter value is copied
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t KNS_CFG_getMC(uint16_t *mc_ptr);

/**
 * @brief Set the value of the Message Counter
 *
 * @param[in] mcTmp : value to store in non-volatile memory
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t KNS_CFG_setMC(uint16_t mcTmp);

/**
 * @brief Get the Kineis radio configuration
 *
 * @param[out] radio_cfg : pointer to a radio configuration
 * The radio configuration is an encrypted memory bloc containing data
 * about radio settings of the device. This data bloc is provided by Kineis.
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t KNS_CFG_getRadioInfo(struct KNS_CFG_radio_t *radio_cfg);

/**
 * @brief Set the Kineis radio configuration
 *
 * @param[in] radio_cfg : pointer to a radio configuration
 * The radio configuration is an encrypted memory bloc containing data
 * about radio settings of the device. This data bloc is provided by Kineis.
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t KNS_CFG_setRadioInfo(void *radio_cfg);

/**
 * @brief Save the Kineis radio configuration into Flash
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t KNS_CFG_saveRadioInfo(void);

/**
 * @brief Get the Kineis device identifier
 *
 * @param[out] dev_id : device identifier
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t KNS_CFG_getId(uint32_t *dev_id);

/**
 * @brief Get the Kineis device address
 *
 * @param[out] dev_add : device address (4-bytes binary array)
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t KNS_CFG_getAddr(uint8_t dev_add[]);

/**
 * @brief Get the device serial number
 *
 * @param[out] sn : serial number (14-bytes null terminated ASCII string)
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t KNS_CFG_getSN(uint8_t sn[]);

/**
 * @brief Set some Kineis stack configuration
 *
 * @param[in] cfg : configuration bitmap
 *
 * @return Status @ref KNS_status_t
 */
enum KNS_status_t KNS_CFG_setCfg(union KNS_CFG_bitmap_t cfg);

/**
 * @brief Get the stack config
 *
 * @return cfg stack configuration bitmap
 */
union KNS_CFG_bitmap_t KNS_CFG_getCfg(void);

#pragma GCC visibility pop

#endif /* KNS_CFG_H */
/**
 * @}
 */
