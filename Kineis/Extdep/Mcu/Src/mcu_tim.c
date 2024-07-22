// SPDX-License-Identifier: no SPDX license
/**
 * @file    mcu_tim.c
 * @brief   MCU wrappers for timer or non-blocking delays used by Kineis stack
 * @author  kineis
 * @date    Creation 2023/01/17
 */

/**
 * @addtogroup MCU_WRAPPERS
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include "kns_app_conf.h" // for STM32 HAL include
#include  STM32_HAL_H

#include "mcu_tim.h"
#include "tim.h"
#include "rtc.h"

//#undef VERBOSE // TIM verbose log disabled by default as too verbose.
#include "mgr_log.h"


/* Extern ------------------------------------------------------------*/

/* Defines ------------------------------------------------------------*/

/* Types -----------------------------------------------------------*/

/* Macro -------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/

/* Table of callback function pointer
 */
typedef enum KNS_status_t (*timeout_isr_cb_t)(void);

/** Timeer callbacks
 * @attention As some timer wmay exit from LPM (standby mode, ensure this table remains
 * avaible at LPM wakeup
 */
__attribute__((__section__(".lpmSection")))
static timeout_isr_cb_t timeout_isr_cb[MCU_TIM_HDLR_MAX] = {NULL};

/* Static function declaration -------------------------------------------------------------*/

/* Functions -------------------------------------------------------------*/

/**
 * @brief  Tx Timeout ISR override
 *
 * Depending on timer concerned, call the callback of the corresponding handler
 *
 * @note So far in system, all used timers are from mcu_tim, thus the override function can be done
 * here.
 *
 * @param[in] htim TIM handle
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim16) {
		MGR_LOG_VERBOSE("%d: %s %d\r\n", MCU_TIM_HDLR_TX_TIMEOUT, __FUNCTION__,
			__LINE__);
		if (timeout_isr_cb[MCU_TIM_HDLR_TX_TIMEOUT] != NULL)
			timeout_isr_cb[MCU_TIM_HDLR_TX_TIMEOUT]();
	}
}

/**
  * @brief  Wake Up Timer callback.
  * @param[in] hrtc_local: RTC handle
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc_local)
{
	if (hrtc_local == &hrtc) {
		MGR_LOG_VERBOSE("%d: %s %d\r\n", MCU_TIM_HDLR_TX_PERIOD, __FUNCTION__, __LINE__);
		if (timeout_isr_cb[MCU_TIM_HDLR_TX_PERIOD] != NULL)
			timeout_isr_cb[MCU_TIM_HDLR_TX_PERIOD]();

	}
}


enum mcu_tim_status_t MCU_TIM_init(enum mcu_tim_hdlr hdlr, enum KNS_status_t (*eop_isr_cb)(void))
{
	MGR_LOG_VERBOSE("%d: %s %d\r\n", hdlr, __FUNCTION__, __LINE__);
	/** @todo Reserve timer HW peripheral if needed, set params such as clock.
	 * Ensure timer counter is in ms.
	 *
	 * MX_TIMx_Init() is generated automatically by CubeMX It is located in main.c
	 * This function is internally calling HAL_TIM_Base_Init and HAL_TIM_OnePulse_Init
	 *
	 * @note To avoid MX_TIMx_Init() to be defined as static in main.c, uncheck 'Visibility
	 * (static)' in Project Manager -> Advanced Settings
	 */
	switch (hdlr) {
	case MCU_TIM_HDLR_TX_TIMEOUT:
		/* uncomment below to handle real init of peripheral HW */
		MX_TIM16_Init();
		timeout_isr_cb[MCU_TIM_HDLR_TX_TIMEOUT] = eop_isr_cb;
		return MCU_TIM_STATUS_OK;
	break;
	case MCU_TIM_HDLR_TX_PERIOD:
		/** @attention Init/DeInit of RTC in timer wrappers may conflict with RTC alarms and
		 * RTC clock logging */
		/* uncomment below to handle real init of peripheral HW */
//		MX_RTC_Init();
		timeout_isr_cb[MCU_TIM_HDLR_TX_PERIOD] = eop_isr_cb;
		return MCU_TIM_STATUS_OK;
	break;
	default:
		return MCU_TIM_STATUS_ERROR;
	break;
	}
}

enum mcu_tim_status_t MCU_TIM_deinit(enum mcu_tim_hdlr hdlr)
{
	MGR_LOG_VERBOSE("%d: %s %d\r\n", hdlr, __FUNCTION__, __LINE__);

	/* @todo Free timer HW peripheral if needed
	 */
	switch (hdlr) {
	case MCU_TIM_HDLR_TX_TIMEOUT:
		/* uncomment below to handle real de-init of peripheral HW */
		if (HAL_TIM_Base_DeInit(&htim16) != HAL_OK)
			return MCU_TIM_STATUS_ERROR;
		return MCU_TIM_STATUS_OK;
	break;
	case MCU_TIM_HDLR_TX_PERIOD:
		/** @attention Init/DeInit of RTC in timer wrappers may conflict with RTC alarms and
		 * RTC clock logging */
		/* uncomment below to handle real de-init of peripheral HW */
//		if (HAL_RTC_DeInit(&hrtc) != HAL_OK)
//			return MCU_TIM_STATUS_ERROR;
		return MCU_TIM_STATUS_OK;
	break;
	default:
		return MCU_TIM_STATUS_ERROR;
	break;
	}
}

enum mcu_tim_status_t MCU_TIM_start(enum mcu_tim_hdlr hdlr, uint32_t timeout_ms)
{
	TIM_HandleTypeDef *htim = &htim16;
	RTC_HandleTypeDef *hrtc_local;
	uint32_t cnt_val, cnt_val_max;

	MGR_LOG_VERBOSE("%d: %s %d\r\n", hdlr, __FUNCTION__, __LINE__);

	/** Configure timer handler */
	switch (hdlr) {
	case MCU_TIM_HDLR_TX_TIMEOUT:
		htim = &htim16;
	break;
	case MCU_TIM_HDLR_TX_PERIOD:
		hrtc_local = &hrtc;
	break;
	default:
		return MCU_TIM_STATUS_ERROR;
	break;
	}

	/** Configure timer settings */
	switch (hdlr) {
	case MCU_TIM_HDLR_TX_TIMEOUT:
		/** As htim16 ARR are 16 long, check delay is not too big.
		 */
		cnt_val = timeout_ms * 2 - 1;
		cnt_val_max = (1 << 16) - 1;
		if (cnt_val > cnt_val_max)
			return MCU_TIM_STATUS_ERROR;
		MGR_LOG_VERBOSE("start timer %d for %d ms, cnt=%d, cnt_max=%d\r\n",
				hdlr, timeout_ms, cnt_val, cnt_val_max);

		__HAL_TIM_CLEAR_FLAG(htim, TIM_IT_UPDATE);
		__HAL_TIM_SET_COUNTER(htim, 0);
		__HAL_TIM_SET_AUTORELOAD(htim, cnt_val);
		HAL_TIM_Base_Start_IT(htim);
	break;
	case MCU_TIM_HDLR_TX_PERIOD:
		cnt_val = timeout_ms / 1000;
		cnt_val--; /** With this config of RTC timer (1s tick), need to reduce count down
			    * by one
			    */
		cnt_val_max = (1 << 16) - 1;
		if (cnt_val > cnt_val_max)
			return MCU_TIM_STATUS_ERROR;
		MGR_LOG_VERBOSE("start timer %d for %d ms, cnt=%d, cnt_max=%d\r\n",
				hdlr, timeout_ms, cnt_val, cnt_val_max);
		if (HAL_RTCEx_SetWakeUpTimer_IT(hrtc_local, cnt_val,
		    RTC_WAKEUPCLOCK_CK_SPRE_16BITS, 0) != HAL_OK)
			Error_Handler();
	break;
	default:
		return MCU_TIM_STATUS_ERROR;
	break;
	}

	return MCU_TIM_STATUS_OK;
}

enum mcu_tim_status_t MCU_TIM_getCount(enum mcu_tim_hdlr hdlr, uint32_t *elapsed_time_ms)
{
	TIM_HandleTypeDef *htim;
	RTC_HandleTypeDef *hrtc_local;

	switch (hdlr) {
	case MCU_TIM_HDLR_TX_TIMEOUT:
		htim = &htim16;
		/** Get counter value and divide it by 2 as it is currently a 500us-step counter */
		*elapsed_time_ms = __HAL_TIM_GET_COUNTER(htim) / 2;
	break;
	case MCU_TIM_HDLR_TX_PERIOD:
		hrtc_local = &hrtc;
		/** Get counter value and divide it by 2 as it is currently a 500us-step counter */
		*elapsed_time_ms = 1000 * HAL_RTCEx_GetWakeUpTimer(hrtc_local);
	break;
	default:
		return MCU_TIM_STATUS_ERROR;
	break;
	}

	MGR_LOG_VERBOSE("%d: %s %d: %d ms\r\n", hdlr, __FUNCTION__, __LINE__, *elapsed_time_ms);

	return MCU_TIM_STATUS_OK;
}

enum mcu_tim_status_t MCU_TIM_stop(enum mcu_tim_hdlr hdlr)
{
	TIM_HandleTypeDef *htim;
	RTC_HandleTypeDef *hrtc_local;

	MGR_LOG_VERBOSE("%d: %s %d\r\n", hdlr, __FUNCTION__, __LINE__);

	switch (hdlr) {
	case MCU_TIM_HDLR_TX_TIMEOUT:
		htim = &htim16;
		HAL_TIM_Base_Stop_IT(htim);
	break;
	case MCU_TIM_HDLR_TX_PERIOD:
		hrtc_local = &hrtc;
		HAL_RTCEx_DeactivateWakeUpTimer(hrtc_local);
	break;
	default:
		return MCU_TIM_STATUS_ERROR;
	break;
	}

	return MCU_TIM_STATUS_OK;
}

/**
 * @}
 */
