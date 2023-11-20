// SPDX-License-Identifier: no SPDX license
/**
 * @file    kns_os.c
 * @brief   Minimalist baremetal OS to be used with Kineis software stack
 * @author  Kineis
 */

/**
 * @addtogroup KNS_OS
 * @brief Kineis SW queues utilities in baremetal environement
 * @{
 */

/* Includes ------------------------------------------------------------------------------------ */

#include <stddef.h>

#include "kns_os.h"

#pragma GCC visibility push(default)

/* Defines ------------------------------------------------------------------------------------- */

/* Structures ---------------------------------------------------------------------------------- */

/* Variables ----------------------------------------------------------------------------------- */

void (*taskPool[KNS_OS_TASK_MAX])(void) = {NULL};

/* Local functions ----------------------------------------------------------------------------- */

/* Function prototypes ------------------------------------------------------------------------- */

enum KNS_status_t KNS_OS_registerTask(enum KNS_OS_taskHdlr_t tskHdlr, void (*taskFctPtr)(void))
{
	if (tskHdlr < KNS_OS_TASK_MAX) {
		taskPool[tskHdlr] = taskFctPtr;
		return KNS_STATUS_OK;
	}
	return KNS_STATUS_BAD_SETTING;
}


void KNS_OS_main(void)
{
	uint8_t idx;

	while (1) {
		for (idx = 0; idx < KNS_OS_TASK_MAX; idx++) {
			if (taskPool[idx] != NULL)
				taskPool[idx]();
		}
	}
}

/**
 * @}
 */
