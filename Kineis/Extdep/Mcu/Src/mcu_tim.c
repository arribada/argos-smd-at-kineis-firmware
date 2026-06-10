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
/**
 * @struct timer description
 *
 * Contains:
 * * callback function (NULL) if timer is not active)
 * * timer period (valid when timer is active)
 */
struct timer_desc_t {
	timeout_isr_cb_t isr_cb;
	uint32_t timeout_ms;
};

/** @brief default values of TX fifo elements (all cleared) */
static const struct timer_desc_t timerDflt = {
		.isr_cb = NULL,
		.timeout_ms = 0
};

/**
 * @struct timer description
 *
 * @attention:
 * * As some timer should exit from LPM (standby mode), ensure table remains available at LPM wakeup
 * * So far, it is mapped in RTC BACKUP register to support both SHUTDOSN and STANDBY. In case only
 * need to support STANDBY (no shutdown), it shuold be fine to mapp this in retentionRamData section
 */
__attribute__((__section__(".lpmSection")))
static struct timer_desc_t timer[MCU_TIM_HDLR_MAX] = {timerDflt};

/** @brief Force-reset the timer[] callback table to a known-safe state.
 *
 * timer[] lives in `.lpmSection`, which the linker maps to the RTC backup
 * registers. That section is NOT loaded from flash by the C runtime, so the
 * static initialiser `= {timerDflt}` is effectively a no-op: the array
 * survives across resets, including a flash erase + reflash. If the
 * previous firmware left a valid `isr_cb` value pointing at code that has
 * since been overwritten, the next HAL_TIM_PeriodElapsedCallback /
 * HAL_RTCEx_WakeUpTimerEventCallback dispatch lands on a bogus address and
 * the chip HardFaults.
 *
 * Call this once from main.c at boot, BEFORE the lib has a chance to arm
 * any HW timer or expect a callback. Idempotent and safe — just zeroes the
 * RAM-equivalent backup cells.
 */
void MCU_TIM_resetState(void)
{
	for (uint32_t i = 0; i < (uint32_t)MCU_TIM_HDLR_MAX; i++) {
		timer[i].isr_cb     = NULL;
		timer[i].timeout_ms = 0;
	}
}

/* Static function declaration -------------------------------------------------------------*/

/* Functions -------------------------------------------------------------*/

/* Flash range for the application image (cf. STM32WL55XX_FLASH_APP.ld).
 * Used to validate that a stored isr_cb actually points at a callable
 * function before we BLX through it.
 */
#define MCU_TIM_FLASH_START  0x08000000UL
#define MCU_TIM_FLASH_END    0x08040000UL

static inline bool mcu_tim_cb_is_valid(timeout_isr_cb_t cb)
{
	uintptr_t a = (uintptr_t)cb;
	/* NULL → skip cleanly (never armed) ; out of flash → stale value left
	 * by a previous firmware image in the RTC backup register. */
	if (a == 0u)
		return false;
	if (a < MCU_TIM_FLASH_START || a >= MCU_TIM_FLASH_END)
		return false;
	return true;
}

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
		timeout_isr_cb_t cb = timer[MCU_TIM_HDLR_TX_TIMEOUT].isr_cb;
		if (mcu_tim_cb_is_valid(cb))
			cb();
	}
}

/**
  * @brief  Wake Up Timer callback.
  * @param[in] hrtc_local: RTC handle
  *
  * @note The isr_cb pointer lives in RTC backup (`.lpmSection`) which
  *       survives flash erase + reflash. If the previous firmware
  *       left a value pointing at code we have since overwritten, the
  *       naive `!= NULL` check passes but the dispatch HardFaults.
  *       mcu_tim_cb_is_valid() also rejects out-of-flash pointers so
  *       a stale entry just gets skipped.
  */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc_local)
{
	if (hrtc_local == &hrtc) {
		MGR_LOG_VERBOSE("%d: %s %d\r\n", MCU_TIM_HDLR_TX_PERIOD, __FUNCTION__, __LINE__);
		timeout_isr_cb_t cb = timer[MCU_TIM_HDLR_TX_PERIOD].isr_cb;
		if (mcu_tim_cb_is_valid(cb)) {
			cb();
		} else {
			/* Defensive: if a stale wake-up keeps firing, also
			 * disarm it so we don't burn cycles on the IRQ
			 * loop. Cheaper than a HardFault. */
			(void)HAL_RTCEx_DeactivateWakeUpTimer(hrtc_local);
			__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc_local, RTC_FLAG_WUTF);
			timer[MCU_TIM_HDLR_TX_PERIOD].isr_cb = NULL;
		}
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
	break;
	case MCU_TIM_HDLR_TX_PERIOD:
		/** @attention Init/DeInit of RTC in timer wrappers may conflict with RTC alarms and
		 * RTC clock logging */
		/* uncomment below to handle real init of peripheral HW */
//		MX_RTC_Init();
	break;
	default:
		return MCU_TIM_STATUS_ERROR;
	break;
	}

	timer[hdlr].isr_cb = eop_isr_cb;
	return MCU_TIM_STATUS_OK;
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
	break;
	case MCU_TIM_HDLR_TX_PERIOD:
		/** @attention Init/DeInit of RTC in timer wrappers may conflict with RTC alarms and
		 * RTC clock logging */
		/* uncomment below to handle real de-init of peripheral HW */
//		if (HAL_RTC_DeInit(&hrtc) != HAL_OK)
//			return MCU_TIM_STATUS_ERROR;
	break;
	default:
		return MCU_TIM_STATUS_ERROR;
	break;
	}

	timer[hdlr].isr_cb = NULL;
	return MCU_TIM_STATUS_OK;
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

		timer[hdlr].timeout_ms = timeout_ms;
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
		timer[hdlr].timeout_ms = timeout_ms;
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

enum mcu_tim_status_t MCU_TIM_suspend(enum mcu_tim_hdlr hdlr, __attribute__((unused)) void *ctxt)
{
	MGR_LOG_VERBOSE("%d: %s %d\r\n", hdlr, __FUNCTION__, __LINE__);

	/* So far, stop timer */
	return MCU_TIM_stop(hdlr);
}

enum mcu_tim_status_t MCU_TIM_resume(enum mcu_tim_hdlr hdlr, __attribute__((unused)) void *ctxt)
{
	MGR_LOG_VERBOSE("%d: %s %d\r\n", hdlr, __FUNCTION__, __LINE__);

	/* So far, restart timer with same period as before */
	return MCU_TIM_start(hdlr, timer[hdlr].timeout_ms);
}

/**
 * @}
 */
