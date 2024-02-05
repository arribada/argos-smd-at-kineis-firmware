// SPDX-License-Identifier: no SPDX license
/**
 * @file    kns_q_conf.c
 * @brief   Configuration of queues used in kineis_swr
 * @author  Kineis
 */

/**
 * @addtogroup KNS_Q
 * @brief Kineis SW queues utilities in baremetal environement
 * @{
 */

#ifndef KNS_Q_CONF_C
#define KNS_Q_CONF_C

/* Includes ------------------------------------------------------------------------------------ */

#include "kns_q_conf.h"
#include "kns_q.h"

#pragma GCC visibility push(default)

/* Variables ----------------------------------------------------------------------------------- */

#ifdef USE_BAREMETAL

static uint8_t qDataApp2Mac[KNS_Q_DL_APP2MAC_LEN][KNS_Q_DL_APP2MAC_ITEM_BYTESIZE];
static struct q_desc_t qApp2Mac = {
	.mutex = false,
	.rIdx = 0,
	.wIdx = 0,
	.nbElt = KNS_Q_DL_APP2MAC_LEN,
	.eltSize = KNS_Q_DL_APP2MAC_ITEM_BYTESIZE,
	.data = (uint8_t *)qDataApp2Mac
};

static uint8_t qDataMac2App[KNS_Q_UL_MAC2APP_LEN][KNS_Q_UL_MAC2APP_ITEM_BYTESIZE];
static struct q_desc_t qMac2App = {
	.mutex = false,
	.rIdx = 0,
	.wIdx = 0,
	.nbElt = KNS_Q_UL_MAC2APP_LEN,
	.eltSize = KNS_Q_UL_MAC2APP_ITEM_BYTESIZE,
	.data = (uint8_t *)qDataMac2App
};

static uint8_t qDataSrvc2Mac[KNS_Q_UL_SRVC2MAC_LEN][KNS_Q_UL_SRVC2MAC_ITEM_BYTESIZE];
static struct q_desc_t qSrvc2Mac = {
	.mutex = false,
	.rIdx = 0,
	.wIdx = 0,
	.nbElt = KNS_Q_UL_SRVC2MAC_LEN,
	.eltSize = KNS_Q_UL_SRVC2MAC_ITEM_BYTESIZE,
	.data = (uint8_t *)qDataSrvc2Mac
};

struct q_desc_t *qPool[KNS_Q_MAX] = {
	/** @todo APP can add its own queues below */
	//&qSensor2App_dummy,   /**< example for adding one appplication queue here */
	/** @attention keep below queue handlers for proper bahaviour of kineis stack
	 * @attention align this enum wit hcontent of qPool declared in kns_q_conf.c
	 */
	&qApp2Mac,
	&qMac2App,
	&qSrvc2Mac
};

#if (defined(DEBUG) || defined(VERBOSE))
const char *qIdx2Str[KNS_Q_MAX] = {
	/** @todo APP can add its own queues below */
	// "KNS_Q_DL_SENSOR2APP", /**< example for adding one appplication queue here */
	/** @attention keep below queue handlers for proper bahaviour of kineis stack
	 * @attention align this enum wit hcontent of qPool declared in kns_q_conf.c
	 */
	"KNS_Q_DL_APP2MAC",
	"KNS_Q_UL_MAC2APP",
	"KNS_Q_UL_SRVC2MAC"
};
#endif

#endif

#pragma GCC visibility pop

#endif /* KNS_Q_CONF_C */

/**
 * @}
 */
