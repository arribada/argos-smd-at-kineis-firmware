/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    kns_q_conf.h
 * @brief   Configuration of queues used in kineis_sw
 * @author  Kineis
 */

/**
 * @page kns_q_conf_page Kineis OS queue configuration
 *
 * This page is presenting the configuration of OS queues for Kineis Oq abtstraction layer
 * (\ref KNS_Q). It should be fixed as per software architecture.
 *
 * Typical parameters of a queue are:
 * * size of each queue (maximum number of items in queue)
 * * size of each element of the queue.
 *
 * So far, the queues are listed in a simple enum (\ref KNS_Q_handle_t) from lower-priority to
 * higher-priority.
 */

/**
 * @addtogroup KNS_Q
 * @brief Kineis SW queues. (refer to @ref kns_q_page for general description).
 * @{
 */

#ifndef KNS_Q_CONF_H
#define KNS_Q_CONF_H

/* Includes ------------------------------------------------------------------------------------ */

#include <stdbool.h>
#include <stdint.h>

#include "kns_mac_evt.h"
/** @todo remove once SRVC2MAC event and kns queues dependencies are correctly clean-up */
#include "kns_srvc_common.h"

#pragma GCC visibility push(default)

/* Defines ------------------------------------------------------------------------------------- */

/**< @attention queue length should be one more than expected in case of baremetal KNS OS */
#ifdef USE_BAREMETAL
#define KNS_Q_DL_APP2MAC_LEN           3
#else // else of USE_BAREMETAL
#define KNS_Q_DL_APP2MAC_LEN           2
#endif // end of USE_BAREMETAL
#define KNS_Q_DL_APP2MAC_ITEM_BYTESIZE sizeof(struct KNS_MAC_appEvt_t)

/**< @attention queue length should be one more than expected in case of baremetal KNS OS */
#ifdef USE_BAREMETAL
#define KNS_Q_UL_MAC2APP_LEN           6
#else // else of USE_BAREMETAL
#define KNS_Q_UL_MAC2APP_LEN           5
#endif // end of USE_BAREMETAL
#define KNS_Q_UL_MAC2APP_EVT_LIST_LEN  20
#define KNS_Q_UL_MAC2APP_ITEM_BYTESIZE sizeof(struct KNS_MAC_srvcEvt_t)
#define KNS_Q_UL_MAC2APP_EVT_LIST_ITEM_BYTESIZE KNS_Q_UL_MAC2APP_ITEM_BYTESIZE

/**< @attention queue length should be one more than expected in case of baremetal KNS OS */
#ifdef USE_BAREMETAL
#define KNS_Q_UL_SRVC2MAC_LEN           4
#else // else of USE_BAREMETAL
#define KNS_Q_UL_SRVC2MAC_LEN           3
#endif // end of USE_BAREMETAL
/**< @todo set item size high enough to contain TX bitstream and/or
 * RX bitstream (if DL RX is supported)
 */
#define KNS_Q_UL_SRVC2MAC_ITEM_BYTESIZE sizeof(struct KNS_SRVC_evt_t)

/**< @attention queue length should be one more than expected in case of baremetal KNS OS */
#ifdef USE_BAREMETAL
#define KNS_Q_UL_INFRA2MAC_LEN           2
#else // else of USE_BAREMETAL
#define KNS_Q_UL_INFRA2MAC_LEN           1
#endif // end of USE_BAREMETAL
#define KNS_Q_UL_INFRA2MAC_ITEM_BYTESIZE sizeof(struct KNS_MAC_infraEvt_t)

/* Enums --------------------------------------------------------------------------------------- */

/**
 * @enum KNS_Q_handle_t
 * @brief Queue handlers
 * @attention Mandatory enum used in KNS_Q APIs whatever OS coded behind (baremetal, FREERTOS, ...)
 * @attention Sort handlers by priority from lower to higher. E.G. SERVICE-to-MAC queue is higher
 * priority than MAC-to-APP queue. This will be important in case of baremetal "OS".
 */
enum KNS_Q_handle_t {
	/** @todo APP can add its own queues below */
	//KNS_Q_DL_SENSOR2APP_DUMMY = -1,   /**< DRIVER to PROFILE queue, @todo add when supported */
	/** @attention keep below queue handlers for proper bahaviour of kineis stack
	 * @attention align this enum with content of qPool declared in kns_q_conf.c
	 */
	KNS_Q_UL_MAC2APP_EVT_LIST   = 0,  /**< MAC to APP event list to report (+KEVT) queue */
	KNS_Q_DL_APP2MAC            = 1,  /**< APP to MAC queue */
	KNS_Q_UL_MAC2APP            = 2,  /**< MAC to APP queue */
	KNS_Q_UL_INFRA2MAC          = 3,  /**< INFRA to MAC queue, used by profile timer */
	KNS_Q_UL_SRVC2MAC           = 4,  /**< SERVICE to MAC queue */
	KNS_Q_MAX                         /**< number of queues */
};


#ifdef USE_BAREMETAL

/* Structures ---------------------------------------------------------------------------------- */

/**
 * @struct KNS_Q_desc_t
 * @brief queue description queue
 * @attention keep below struct descriptio, as is, in case of Kineis's baremetal OS
 *
 * All useful parameters needed to use a queue
 */
struct q_desc_t {
	bool mutex;
	uint8_t rIdx;
	uint8_t wIdx;
	uint8_t nbElt;
	uint8_t eltSize;
	uint8_t *data;
	bool isLpmBlocker; // Baremetal OS case: avoid entering LPM if some event present in Queue.
};

/* Extern -------------------------------------------------------------------------------------- */

extern struct q_desc_t *qPool[KNS_Q_MAX];

#if (defined(DEBUG) || defined(VERBOSE))
extern const char *qIdx2Str[KNS_Q_MAX];
#endif /* (defined (DEBUG) || defined (VERBOSE)) */

#endif /* USE_BAREMETAL */

#pragma GCC visibility pop

#endif /* KNS_Q_CONF_H */

/**
 * @}
 */
