/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    kns_os.h
 * @brief   Minimalist baremetal OS to be used with Kineissoftware stack
 * @author  Kineis
 * @date    creation 23/06/2023
 */

/**
 * @page kns_os_page OS abstraction layer for Kinéis
 *
 * This page is presenting the Kineis OS design (\ref KNS_OS, \ref KNS_Q)
 *
 * @section kns_os_intro Introduction
 *
 * As on lots of OS, queues are used by kineis SW environement (kineis stack + application) to
 * share events and data between different tasks/ISR contexts. Source SW entity (task) pushes
 * event+data into the queue from one context (e.g. ISR) and the destination layer (task) will pop
 * it from its own context.
 *
 * @section kns_os_list List of tasks
 *
 * Kineis OS does not assume any priorities between tasks directly. But it is done through the side
 * effect of priorities declared on queues. An event delivered in a high-priority queues shall be
 * consumed as soon as possible by the dedicated task.
 *
 * Below is typical list of tasks involved in Kineis FW environment (kineis stack + application):
 * * KNS_OS_TASK_APP one application task defined by default. User can decide to add an extra
 *   application task through (\ref kns_os_conf.h) configuration file. Refer also to
 *   \ref kns_os_conf_page for details.
 * * KNS_OS_TASK_MAC Kineis stack main task
 * * KNS_OS_TASK_IDLE idle task entering LPM when other tasks are not active
 *
 * @section kns_os_subpages Sub-pages
 *
 * * @subpage kns_os_conf_page
 * * @subpage kns_q_page
 * * @subpage kns_q_conf_page
 */

/**
 * @addtogroup KNS_OS
 * @brief Kineis SW queues. (refer to @ref kns_os_page for general description).
 * @{
 */

#ifndef KNS_OS_H
#define KNS_OS_H

/* Includes ------------------------------------------------------------------------------------ */

#include <stdbool.h>
#include <stdint.h>
#include "kns_types.h"
#include "kns_os_conf.h"

#pragma GCC visibility push(default)

/* Defines ------------------------------------------------------------------------------------- */

/* Structures ---------------------------------------------------------------------------------- */

/* Function prototypes ------------------------------------------------------------------------- */

/**
 * @brief This function is used to register task to be handled by baremetal Kineis OS
 *
 * In a way to get minimalist footprint, the Kinéis OS is very limited and lot of decalartions are
 * static. For instance, a task handler needs to be defined for the task being registered. Refer to
 * \ref kns_os_conf_page for extra explanantions. task prototype is without any parameter and return
 * value.
 *
 * @param[in] tskHdlr: task handler concerned by the registering (all handlers should be defined in
 * kns_os_conf.h
 * @param[in] taskFctPtr: pointer to task function
 * @retval KNS_STATUS_OK if registration was ok, KNS_STATUS_ERROR otherwise.
 */
enum KNS_status_t KNS_OS_registerTask(enum KNS_OS_taskHdlr_t tskHdlr, void (*taskFctPtr)(void));

/**
 * @brief This function is the main scheduler of Kineis baremetal OS.
 *
 * It is in charge to schedule all task registerd before. this main OS task shall be called in main
 * function of the firmaware.
 *
 * @note This function contains infinite loop and will never exit.
 */
void KNS_OS_main(void);

#pragma GCC visibility pop

#endif /* KNS_OS_H */
/**
 * @}
 */
