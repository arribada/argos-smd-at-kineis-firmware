/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    kns_q.h
 * @brief   Configuration of queues used in kineis_swr
 * @author  Kineis
 * @date    creation 21/07/2022
 */

/**
 * @page kns_q_page OS queue utilities
 *
 * This page is presenting the utilities to manipulate queues used in kineis_sw.
 *
 * @section kns_q_intro Introduction
 *
 * As on lots of OS, queues are used by kineis SW environement (kineis stack + application) to
 * share events and data between different tasks/ISR contexts. Source SW entity (task) pushes
 * event+data into the queue from one context (e.g. ISR) and the destination layer (task) will pop
 * it from its own context.
 *
 * @section kns_q_list List of queues
 *
 * Kineis OS does not assume any priorities between tasks directly. But it is done through the side
 * effect of priorities declared on queues. An event delivered in a high-priority queues shall be
 * consumed as soon as possible by the dedicated task.
 *
 * From global kineis SW architecture, below is some example of queues used between ISR/TASK
 * contexts. It is sorted from lowest to highest priority order:
 * * 4: KNS_Q_UL_MAC2APP_EVT_LIST used by the Kinéis application to report events to the user (mgr_at_cmd).
 * * 3: KNS_Q_DL_APP2MAC used from application to Kinéis stack (MAC internal component)
 * * 2: KNS_Q_UL_MAC2APP used from Kinéis stack to application (MAC internal component)
 * * 1: KNS_Q_UL_SRVC2MAC internally used internally in Kineis stack, to send end-of-processing
 *      events from RF DRIVER ISRs to task context layer (MAC internal component)
 *
 * @section kns_q_design Design
 *
 * As kineis_sw is fully known, queues above are hardcoded and statically defined as arrays.
 *
 * The size  and content of each queue shall be defined through kns_q.h and kns_q.c. Refer to
 * @ref kns_q_conf_page for some extra details
 *
 * There is write and read pointer to the cells of the array. Roll-over is handled. Check on FIFO
 * full and FIFO empty is done.
 *
 * @attention due to simple FIFO-full mechanism implemented so far, queues needs to be one element
 * longer than expected by the system, in case of baremetaol Kineis OS.
 *
 * A priority is assigned to those queues, meaning some event in higher-priority queue should be
 * executed before lower-priority queue's events (refer to kineis_sw architecture for details).
 *
 */

/**
 * @addtogroup KNS_Q
 * @brief Kineis SW queues. (refer to @ref kns_q_page for general description).
 * @{
 */

#ifndef KNS_Q_H
#define KNS_Q_H

/* Includes ------------------------------------------------------------------------------------ */

#include <stdbool.h>
#include <stdint.h>
#include "kns_types.h"
#include "kns_q_conf.h"

#pragma GCC visibility push(default)

/* Structures ---------------------------------------------------------------------------------- */

/* Function prototypes ------------------------------------------------------------------------- */

/**
 * @brief This function is used to create queue
 *
 * This function is aimed at creating the queue used in KNS libraries.
 * Size and length of the queues are statically defined KNS_Q_conf.h.
 *
 * This function also handles the correspondence between kineis queue handlers (KNS_Q_handle_t) and
 * RTOS handlers, so that KNS_Q_push/pop can still be used with kineis queue handler references.
 *
 * @note This routine is only useful in case of RTOS. In internal baremetal kineis OS, those queues
 * are directly defined as uint8t arrays during compilation. In case of external RTOS, we need to
 * invoke queue creation routine instead.
 *
 * @param[in] qHandle: kineis queue handle
 * @param[in] qLength: length of the queue in number of element
 * @param[in] qEltByteSize: length of each element of the queue (in bytes).
 * @retval KNS_STATUS_OK if creation was ok, KNS_STATUS_ERROR otherwise.
 */
enum KNS_status_t KNS_Q_create(enum KNS_Q_handle_t qHandle, uint8_t qLength, uint16_t qEltByteSize);

/**
 * @brief This function is used to push an element into a queue
 *
 * As this routine may be invoked from different ISR/TASK context, processing is under critical
 * section.
 *
 * @attention In case of FreeRTOS, This routine does not handle being call from ISR context so far.
 * @todo Check caller is from ISR context or not, and call corresponding xQueueSendToBack(FomISR)
 * fct in consequence.
 *
 * @param[in] qHandle: queue handle
 * @param[in] qItem: pointer to queue element
 * @retval KNS_status_t status: KNS_STATUS_OK if element was added successfully,
 * KNS_STATUS_FULL if no more place in queue, KNS_STATUS_ERROR otherwise.
 */
enum KNS_status_t KNS_Q_push(enum KNS_Q_handle_t qHandle, void *qItem);

/**
 * @brief This function is used to pop next element from a queue
 *
 * As this routine may be invoked from different ISR/TASK context, processing is under critical
 * section.
 *
 * @attention In case of FreeRTOS, This routine does not handle being call from ISR context so far.
 * @todo Check caller is from ISR context or not, and call corresponding xQueueReceive(FomISR)
 * fct in consequence.
 *
 * @todo In case of FREERTOS and CMSIS2, fix behaviour, it should return:
 * * KNS_STATUS_QEMPTY when queue was empty,
 * * KNS_STATUS_OK means some valid element received,
 * * KNS_STATUS_ERROR for any other reason
 *
 * @param[in] qHandle: queue handle
 * @param[in] qItem: pointer to queue element
 * @retval KNS_status_t status: KNS_STATUS_OK if some element was found,
 * KNS_STATUS_EMPTY if no element present in queue, KNS_STATUS_ERROR otherwise
 */
enum KNS_status_t KNS_Q_pop(enum KNS_Q_handle_t qHandle, void *qItem);

#ifdef USE_BAREMETAL
/**
 * @brief This function is used to check some higher-priority queue contains elements, meaning
 * preemption is required.
 *
 * This function is aimed to be used inside some lower-priority event scheduler to check exit is
 * required because higher-priority event scheduler needs to preempt.
 *
 * @param[in] qHandle: queue handle
 * @retval true if some element present in higher priority queue, false otherwise.
 */
bool KNS_Q_isEvtInHigherPrioQ(enum KNS_Q_handle_t qHandle);

/**
 * @brief This function is used to check some queue contains some elements
 *
 * This function is aimed to be used inside some IDLE task to ensure it is possible to go into low
 * power
 *
 * @retval true if some element present in higher priority queue, false otherwise.
 */
bool KNS_Q_isEvtInSomeQ(void);
#endif

#pragma GCC visibility pop

#endif /* KNS_Q_H */
/**
 * @}
 */
