/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    kns_mac_prfl_cfg.h
 * @brief   Header file describing the ser settings required to configure a MAC protocol
 * @author  Kinéis
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
 * @addtogroup KNS_MAC_PRFL
 * @brief MAC Protocol configuration.
 * @{
 */

#ifndef KNS_MAC_PRFL_CFG_H
#define KNS_MAC_PRFL_CFG_H

/* Includes ------------------------------------------------------------------------------------ */

#include <stdint.h>
#include "kns_types.h"

#pragma GCC visibility push(default)

/* Defines ------------------------------------------------------------------------------------- */

#define NB_PARRALLEL_MSG_MIN 1
#define RETX_NB_MIN 1
#define RETX_PERIOD_MIN_S 60
#define RETX_PERIOD_MAX_S 65535 // 18 hours
#define SATDET_NB_MIN 1
#define SATDET_PERIOD_MIN_S 60
#define SATDET_PERIOD_MAX_S 65535 // 18 hours

/* Enums --------------------------------------------------------------------------------------- */

/**
 * @enum KNS_MAC_prflId_t
 * @brief MAC profile list
 */
enum KNS_MAC_prflId_t {
	KNS_MAC_PRFL_NONE,  /**< no profile currently configured */
	KNS_MAC_PRFL_BASIC,  /**< basic profile */
	KNS_MAC_PRFL_BLIND,  /**< blind profile */
	KNS_MAC_PRFL_SATDET,  /**< satdet profile */
	KNS_MAC_PRFL_BLIND_POS,  /**< blind_pos profile */
	KNS_MAC_PRFL_MAX
};

/* Structures ---------------------------------------------------------------------------------- */

/**
 * @struct KNS_MAC_send_data_ctxt_t
 * @brief Event context structure containing usefull information for the APPlication to send data
 * to Kineis
 *
 * @attention Need to pack the struct to ensure memory mapping whatever the uC/compiler used to
 * compile the kineis stack library
 */
struct __attribute__((packed)) KNS_MAC_BLIND_usrCfg_t {
	int8_t retx_nb;
	uint8_t nb_parrallel_msg;
	uint32_t retx_period_s;
	int8_t per_offset; // device's PER offset compared to reference one used in PER map
};

/**
 * @struct KNS_MAC_send_data_ctxt_t
 * @brief Event context structure containing usefull information for the APPlication to send data
 * to Kineis
 *
 * @attention Need to pack the struct to ensure memory mapping whatever the uC/compiler used to
 * compile the kineis stack library
 */
struct __attribute__((packed)) KNS_MAC_SATDET_usrCfg_t {
	uint8_t nb_parrallel_msg;
	int8_t satdet_nb;
	int8_t retx_nb;
	uint32_t satdet_period_s;
	uint32_t retx_period_s;
};

/**
 * @struct KNS_MAC_prflInfo_t
 * @brief structure describing current profil used by kineis stack MAC mayer
 */
struct __attribute__((packed)) KNS_MAC_prflInfo_t {
	enum KNS_MAC_prflId_t id;
	union { // empty for BASIC and NONE profile
		uint8_t prflCfgPtr; // dummy pointer to to profile context
		struct KNS_MAC_BLIND_usrCfg_t blindCfg; // config for BLIND profile
		struct KNS_MAC_SATDET_usrCfg_t satdetCfg; // config for SATDET profile
	};
};

#pragma GCC visibility pop

#endif /* KNS_MAC_PRFL_CFG_H */
/**
 * @}
 */
