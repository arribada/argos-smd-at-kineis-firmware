/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    kineis_sw_conf.h
 * @brief   Kineis stack SW configurations depending on platform used
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
 * @addtogroup KINEIS_SW_CONF
 * @brief Kineis SW configuration. This module can be adapted by external user depending on project
 * @{
 */

#include "kns_types.h"

#ifndef KINEIS_SW_CONF_H
#define KINEIS_SW_CONF_H

/* Defines ------------------------------------------------------------------------------------- */

/**
 * @brief define include file concerning assert used by Kineis stack
 *
 * @attention DO NOT change this as long as you build with Kinés stack. Must keep it the same way
 * it was when generating the library (libkineis.a).
 */
#define KINEIS_SW_ASSERT_H	"kns_assert.h"

/**
 * @brief define include file concerning critical sections used by Kineis stack
 *
 * @attention DO NOT change this as long as you build with Kinés stack. Must keep it the same way
 * it was when generating the library (libkineis.a).
 */
#define KINEIS_CS_H		"kns_cs.h"

/* Enums --------------------------------------------------------------------------------------- */

/**
 * @brief Upper error code for KIM device. So far, this is used as return code of AT commands.
 */
enum ERROR_RETURN_T {
	// General errors
	ERROR_UNKNOWN = KNS_STATUS_MAX, /**< Start from Max limit of kineis status enum */
	ERROR_INCOMPATIBLE_VALUE,
	/* Hardware feature not present on this board (Arribada). The GUI can
	 * probe AT+DL=? / AT+GNSS=? at connect time and grey-out the relevant
	 * UI panels when this code comes back. Slotted right after
	 * ERROR_INCOMPATIBLE_VALUE = 1001 so the value is 1002 — stable for
	 * the GUI's parsing table. */
	ERROR_FEATURE_NOT_AVAILABLE,

	// User data errors
	ERROR_INVALID_USER_DATA_LENGTH  = ERROR_UNKNOWN + 100,
	ERROR_DATA_QUEUE_FULL,
	ERROR_DATA_QUEUE_EMPTY,

	// AT commands
	ERROR_PARAMETER_FORMAT          = ERROR_UNKNOWN + 200,
	ERROR_MISSING_PARAMETERS,
	ERROR_TOO_MANY_PARAMETERS,
	ERROR_UNKNOWN_AT_CMD
};

#endif // end KINEIS_SW_CONF_H

/**
 * @}
 */
