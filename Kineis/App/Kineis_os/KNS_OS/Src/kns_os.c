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
#include "mgr_log.h"  /* For MGR_LOG_flush() */
#include "mcu_spi_driver.h"  /* For spiState - to avoid flushing logs during SPI transactions */

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

		/* Flush debug log buffer to UART.
		 * With pipelined protocol, we can flush anytime except during
		 * PROCESS_CMD (need to prepare response quickly).
		 * UART TX at 115200 baud sends 1KB in ~90ms, but MGR_LOG_flush
		 * limits to 50ms max per call. */
#ifdef DEBUG
		if (spiState != SPICMD_PROCESS_CMD) {
			MGR_LOG_flush();
		}
#endif
	}
}

#pragma GCC visibility pop

/**
 * @}
 */
