/**
 ******************************************************************************
 * @file    mgr_log_rtc.c
 * @brief   This file contains log routine concerning RTC. There is a dependency on HAL RTC.
 *
 * @date     2020/04/21
 * @author   creation : William BEGOC
 * @version  1.0
 * @note
 ******************************************************************************
 */

/**
 ******************************************************************************
 * @addtogroup MGR_LOG
 * @{
 ******************************************************************************
 */

/*******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include <stdio.h>
#include "mgr_log.h"
#include "mgr_log_rtc.h"
#include "kns_app_conf.h" // for STM32 HAL include
#include STM32_HAL_H
#include STM32_HAL_RTC_H

/*******************************************************************************
 * EXTERNS
 ******************************************************************************/
extern RTC_HandleTypeDef hrtc;

/*******************************************************************************
 * FUNCTIONS
 ******************************************************************************/
void MGR_LOG_RtcDateTime(void)
{
	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructureget;

	/* Single buffer for the entire timestamp - avoids interleaving issues */
	char timestamp[32];

	HAL_RTC_WaitForSynchro(&hrtc);
	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);

	/* Format complete timestamp in single buffer: "YYYY/MM/DD HH:MM:SS" */
	sprintf(timestamp, "%04d/%02d/%02d %02d:%02d:%02d",
			2000 + sdatestructureget.Year,
			sdatestructureget.Month,
			sdatestructureget.Date,
			stimestructureget.Hours,
			stimestructureget.Minutes,
			stimestructureget.Seconds);

	/* Single atomic call to add timestamp to ring buffer */
	vMGR_LOG_printf("%s", timestamp);
}

/**
 * @}
 */
