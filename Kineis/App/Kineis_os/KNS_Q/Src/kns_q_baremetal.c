// SPDX-License-Identifier: no SPDX license
/**
 * @file    kns_q_baremetal.c
 * @brief   Configuration of queues used in kineis_swr
 * @author  William Begoc
 * @date    creation 21/07/2022
 * @version 1.0
 * @note
 */

/**
 * @addtogroup KNS_Q
 * @brief Kineis SW queues utilities in baremetal environement
 * @{
 */

#ifndef KNS_Q_BAREMETAL_C
#define KNS_Q_BAREMETAL_C

/* Includes ------------------------------------------------------------------------------------ */

#include "kns_cs.h"
#include "kns_q_conf.h"
#include "kns_q.h"

//#undef VERBOSE
#include "mgr_log.h"

#pragma GCC visibility push(default)


/* Variables ----------------------------------------------------------------------------------- */

extern struct q_desc_t *qPool[KNS_Q_MAX];

/* Local functions ----------------------------------------------------------------------------- */

/**
 * @brief  Log array of uint8_t
 * @param[in] data: pointer to table
 * @param[in] len: number of byte to log from the table
 */
static void MGR_LOG_VERBOSE_array(__attribute__((unused)) uint8_t *data, uint16_t len)
{
	uint16_t i;

	for (i = 0; i < len; i++)
		MGR_LOG_VERBOSE_RAW("%02X", data[i]);
	MGR_LOG_VERBOSE_RAW("\r\n");
}

/* Function prototypes ------------------------------------------------------------------------- */

#ifndef UNIT_TEST
__attribute((__weak__))
#endif
enum KNS_status_t KNS_Q_create(enum KNS_Q_handle_t qHandle, uint8_t qLength, uint16_t qEltByteSize)
{
	struct q_desc_t *q = qPool[qHandle];

	if (qLength != q->nbElt)
		return KNS_STATUS_BAD_SETTING;
	if (qEltByteSize != q->eltSize)
		return KNS_STATUS_BAD_SETTING;
	return KNS_STATUS_OK;
}

#ifndef UNIT_TEST
__attribute((__weak__))
#endif
enum KNS_status_t KNS_Q_push(enum KNS_Q_handle_t qHandle, void *qItem)
{
	uint16_t eltIdx;
	uint8_t wIdxNext;
	uint8_t *qEltPtr, *qItemPtr;
	struct q_desc_t *q = qPool[qHandle];
#ifdef DEBUG
	uint8_t wIdxPrev = q->wIdx;
#endif

	/** Get queue's mutex to write into FIFO
	 */
	KNS_CS_enter();

	while (q->mutex == true)
		;
	q->mutex = true;

	/** Check FIFO full, @note, with this mechanism, there is always one empty element before
	 *  FIFO full
	 */
	wIdxNext = (q->wIdx  + 1) % q->nbElt;
	if (wIdxNext == q->rIdx) {
		q->mutex = false;
		KNS_CS_exit();
		MGR_LOG_DEBUG("[KNS_Q] push QFULL %s, idx %d, evt=0x%x\r\n", qIdx2Str[qHandle],
			wIdxPrev, ((uint8_t *)qItem)[0]);
		return KNS_STATUS_QFULL;
	}

	qEltPtr = q->data + q->eltSize * q->wIdx;
	qItemPtr = (uint8_t *)qItem;
	for (eltIdx = 0; eltIdx < q->eltSize; eltIdx++)
		qEltPtr[eltIdx] = qItemPtr[eltIdx];

	q->wIdx = wIdxNext;

	q->mutex = false;
	KNS_CS_exit();

	MGR_LOG_VERBOSE("[KNS_Q] push q %s, idx %d, size %d: ", qIdx2Str[qHandle],
		wIdxPrev, q->eltSize);
	MGR_LOG_VERBOSE_array(qEltPtr, q->eltSize);

	return KNS_STATUS_OK;
}

#ifndef UNIT_TEST
__attribute((__weak__))
#endif
enum KNS_status_t KNS_Q_pop(enum KNS_Q_handle_t qHandle, void *qItem)
{
	uint16_t eltIdx;
	uint8_t *qEltPtr, *qItemPtr;
	struct q_desc_t *q = qPool[qHandle];
#if (defined(VERBOSE))
	uint8_t rIdxPrev = q->rIdx;
#endif

	/** Report current is empty if some event prensent in some higher-priority queues.
	 * This way, Kineis OS main loop will let corresponding task to process thoses events.
	 */
	if (KNS_Q_isEvtInHigherPrioQ(qHandle))
		return KNS_STATUS_QEMPTY;

	/** Get queue's mutex to update read pointer of the FIFO
	 */
	KNS_CS_enter();
	while (q->mutex == true)
		;
	q->mutex = true;

	/** Check FIFO empty, @note, with this mechanism, there is always one empty element before
	 *  FIFO full
	 */
	if (q->rIdx == q->wIdx) {
		q->mutex = false;
		KNS_CS_exit();
		//MGR_LOG_VERBOSE("[KNS_Q] pop QEMPTY %s\r\n", qIdx2Str[qHandle]);
		return KNS_STATUS_QEMPTY;
	}

	qEltPtr = q->data + q->eltSize * q->rIdx;
	qItemPtr = (uint8_t *)qItem;
	for (eltIdx = 0; eltIdx < q->eltSize; eltIdx++)
		qItemPtr[eltIdx] = qEltPtr[eltIdx];

	q->rIdx = (q->rIdx  + 1) % q->nbElt;

	q->mutex = false;
	KNS_CS_exit();

	MGR_LOG_VERBOSE("[KNS_Q] pop  q %s, idx %d\r\n", qIdx2Str[qHandle], rIdxPrev);

	return KNS_STATUS_OK;
}

bool KNS_Q_isEvtInHigherPrioQ(enum KNS_Q_handle_t qHandle)
{
	struct q_desc_t *q;
	enum KNS_Q_handle_t qHandleTmp;

	for (qHandleTmp = qHandle + 1; qHandleTmp <  KNS_Q_MAX; qHandleTmp++) {
		q = qPool[qHandleTmp];
		if (q->rIdx != q->wIdx) {
			MGR_LOG_VERBOSE("[KNS_Q] higher-prio check q %s, evt found in q %s\r\n",
				qIdx2Str[qHandle], qIdx2Str[qHandleTmp]);
			return true;
		}
	}

	return false;
}

bool KNS_Q_isEvtInSomeQ()
{
	struct q_desc_t *q;
	enum KNS_Q_handle_t qHandleTmp;

	for (qHandleTmp = 0; qHandleTmp <  KNS_Q_MAX; qHandleTmp++) {
		q = qPool[qHandleTmp];
		if ((q->isLpmBlocker) && (q->rIdx != q->wIdx)) {
			MGR_LOG_VERBOSE("[KNS_Q] some-Q check, evt found in q %s\r\n",
				qIdx2Str[qHandleTmp]);
			return true;
		}
	}

	return false;
}

#pragma GCC visibility pop

#endif /* KNS_Q_BAREMETAL_C */

/**
 * @}
 */
