// SPDX-License-Identifier: no SPDX license
/**
 * @file    kns_q_cmsis_os2.c
 * @brief   Configuration of queues used in kineis_swr
 * @author  William Begoc
 * @date    creation 21/07/2022
 * @version 1.0
 */

/**
 * @addtogroup KNS_Q
 * @brief Kineis SW queue utilities based on CMSIS-RTOS2
 * @{
 */

#ifndef KNS_Q_CMSIS_OS2_C
#define KNS_Q_CMSIS_OS2_C

/* Includes ------------------------------------------------------------------------------------ */

#include "kns_q_conf.h"
#include "kns_q.h"
#include "mgr_log.h"
#include "cmsis_os.h"

#pragma GCC visibility push(default)

/* Defines ------------------------------------------------------------------------------------- */

/* Structures ---------------------------------------------------------------------------------- */

static void *qPoolHandle[KNS_Q_MAX];

/* Function prototypes ------------------------------------------------------------------------- */

#ifndef UNIT_TEST
__attribute((__weak__))
#endif
enum KNS_status_t KNS_Q_create(enum KNS_Q_handle_t qHandle, uint8_t qLength, uint16_t qEltByteSize)
{

	osMessageQueueId_t osMessageQueueId;

	osMessageQueueId = osMessageQueueNew(qLength, qEltByteSize, NULL);
	if (osMessageQueueId != NULL) {
		qPoolHandle[qHandle] = (void *)osMessageQueueId;
		MGR_LOG_VERBOSE("[KNS_Q] create q %d, len %d, elt size %d, RTOS hdl 0x%p\r\n",
			qHandle, qLength, qEltByteSize, (void *)osMessageQueueId);
		return KNS_STATUS_OK;
	} else
		return KNS_STATUS_ERROR;
}

#ifndef UNIT_TEST
__attribute((__weak__))
#endif
enum KNS_status_t KNS_Q_push(enum KNS_Q_handle_t qHandle, void *qItem)
{
	/** Prio parameter is internally ignored
	 * No timeout
	 */
	if (osMessageQueuePut(qPoolHandle[qHandle], (void *)qItem, 0, 0) == osOK) {
		MGR_LOG_VERBOSE("[KNS_Q] push q %d, elt 0x%p\r\n", qHandle, (void *)qItem);
		return KNS_STATUS_OK;
	}
	MGR_LOG_DEBUG("[KNS_Q][ERROR] push q %d FAILED, elt 0x%p\r\n", qHandle, (void *)qItem);
	return KNS_STATUS_ERROR;
}

#ifndef UNIT_TEST
__attribute((__weak__))
#endif
enum KNS_status_t KNS_Q_pop(enum KNS_Q_handle_t qHandle, void *qItem)
{
	/** @todo handle empty queue should return KNS_STATUS_QEMPTY,
	 * KNS_STATUS_OK means some valid element received,
	 * KNS_STATUS_ERROR for any other reason
	 */
	/** Prio parameter is internally ignored
	 * No timeout
	 */
	if (osMessageQueueGet(qPoolHandle[qHandle], (void *)qItem, 0, 0) == osOK) {
		MGR_LOG_VERBOSE("[KNS_Q] pop  q %d\r\n", qHandle);
		return KNS_STATUS_OK;
	}
	MGR_LOG_DEBUG("[KNS_Q][ERROR] pop q %d FAILED\r\n", qHandle);
	return KNS_STATUS_ERROR;
}

#pragma GCC visibility pop

#endif /* KNS_Q_CMSIS_OS2_C */

/**
 * @}
 */
