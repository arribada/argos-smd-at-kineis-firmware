/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    kns_os_conf.h
 * @brief   Configuration of task used with/in kineis_sw
 * @author  Kineis
 * @date    creation 23/06/2023
 */

/**
 * @page kns_os_conf_page Kineis OS configuration
 *
 * This page is presenting the configuration of Kineis OS abstraction layer (\ref KNS_OS).
 *
 * Basically, an OS needs the list of all tasks to be registerd. In kineis OS abstraction layer,
 * this is a simple enumerate containing all handlers (\ref KNS_OS_taskHdlr_t).
 *
 * For information, tasks are roder from lower priority to upper priority.
 *
 * \note In can you use the Kineis baremetal OS (USE_BAREMETAL compilation flag), there is actually
 * no notion of priorities on tasks. Priorities are rather handled at queue level. Before popping
 * an event from a queue the baremetal OS checks ther is no event present in higher-priority queues
 * before. Refer to \ref kns_q_page for extra details on this.
 */

/**
 * @addtogroup KNS_OS
 * @{
 */

#ifndef KNS_OS_CONF_H
#define KNS_OS_CONF_H

/* Includes ------------------------------------------------------------------------------------ */

#pragma GCC visibility push(default)

/* Enums --------------------------------------------------------------------------------------- */

/**
 * @enum KNS_OS_taskHdlr_t
 * @brief task handlers
 */
enum KNS_OS_taskHdlr_t {
	KNS_OS_TASK_APP,  /**< APPlication task */
	KNS_OS_TASK_MAC,  /**< Kineis stack(MAC) task */
	KNS_OS_TASK_IDLE, /**< IDLE task, falling down there when no more things to do */
	KNS_OS_TASK_MAX   /**< number of tasks */
};

/* Note: TRACKER mode uses KNS_OS_TASK_APP for the tracker loop.
 * SWS measurement is called directly from the tracker loop, not as a separate OS task.
 */

#pragma GCC visibility pop

#endif /* KNS_OS_CONF_H */

/**
 * @}
 */
