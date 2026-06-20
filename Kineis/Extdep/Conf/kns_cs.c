// SPDX-License-Identifier: no SPDX license
/**
 * @file    kns_cs.c
 * @brief   critical section handling for kineis stack.
 * @author  Kinéis
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
 * @brief Kineis SW configuration regarding critical sections
 * @{
 */

/* Includes ------------------------------------------------------------------------------------ */

#include <stdint.h>

#include "main.h"

#pragma GCC visibility push(default)

/* Defines ------------------------------------------------------------------------------------- */
#define PRIM_ARRAY_SIZE 8 // define the depth of inner critical sections. MUST be 4 at minimum


/* Private variables --------------------------------------------------------------------------- */

static uint32_t prim[PRIM_ARRAY_SIZE];
volatile uint8_t idx = 0;  /* shared main/ISR nesting depth — volatile */

/* Function ------------------------------------------------------------------------------------ */

void KNS_CS_enter()
{

	prim[idx % PRIM_ARRAY_SIZE] = __get_PRIMASK();
	__disable_irq();
	idx++; // make prim pointing on next empty plac in array
}

void KNS_CS_exit()
{
	/* Guard against an unbalanced exit: without this, idx-- at idx==0 wraps to
	 * 255, reads a stale prim[7] and can spuriously __enable_irq inside an outer
	 * critical section (torn flash write / missed wake-arm). */
	if (idx == 0) {
		return;
	}
	idx--; // decrement idx in a way to point on last valid data
	if (!prim[idx % PRIM_ARRAY_SIZE]) {
		__enable_irq();
	}
}

#pragma GCC visibility pop

/**
 * @}
 */
