/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    mcu_tim.h
 * @brief   MCU wrappers for timer or non-blocking delays used by Kineis stack
 * @author  kineis
 * @date    Creation 2023/01/17
 */

/**
 * @page mcu_tim_page MCU wrappers: TIMers
 *
 * The Kineis SW library requires some timer for its protocols
 */

/**
 * @addtogroup MCU_WRAPPERS
 * @brief MCU wrapper used by Kineis SW stack
 * One has to implement API as per its microcontroller and its platform ressources.
 * @{
 */

#ifndef MCU_TIM_H_
#define MCU_TIM_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "kns_types.h"

/* Types -----------------------------------------------------------*/
enum mcu_tim_status_t {
	MCU_TIM_STATUS_OK,
	MCU_TIM_STATUS_ERROR,
	MCU_TIM_STATUS_MAX_VALUE
};


/**
 * @brief handler for timers used in Kineis SW stack
 *
 * Each value of this enum refers to a feature which requires some timer or non-blocking delay.
 * Behind each handler, there is a static configuration such as:
 * * timer HW peripheral to be used
 * * automatic reload of counter when overflow/underflow is reached
 * * interrupt enable when overflow/underflow is reached
 *
 * @note Duration is not part of the static configuration. It is dynamically configured in
 * MCU_TIM_Start API.
 */
enum mcu_tim_hdlr {
	MCU_TIM_HDLR_TX_TIMEOUT, // not use so far, directly set in RFLL_WL
	MCU_TIM_HDLR_TX_PERIOD,
	MCU_TIM_HDLR_MAX
};

/* Function declaration ------------------------------------------------------------*/

/**
 * @brief Function used to register and initialize a timer
 *
 * This function is actually used to declare and configure general settings of the timer (HW
 * peripheral reservation, clock, ...). It should be called before MCU_TIM_start
 *
 * As long as timer configuration is said to be static (one single config per feature), there is no
 * need to provide some configuration settings at this level. Only one handler is needed.
 *
 * @param[in] hdlr timer handler
 * @param[in] eop_isr_cb callback function to be called when timeout delay is reached
 *
 * @return  MCU_TIM_STATUS_OK if success. Error status otherwise.
 */
enum mcu_tim_status_t MCU_TIM_init(enum mcu_tim_hdlr hdlr, enum KNS_status_t (*eop_isr_cb)(void));

/**
 * @brief Function used to un-register the timer
 *
 * This function is used to free the timer ressource as no more needed.
 *
 * @param[in] hdlr timer handler
 *
 * @return  MCU_TIM_STATUS_OK if success. Error status otherwise.
 */
enum mcu_tim_status_t MCU_TIM_deinit(enum mcu_tim_hdlr hdlr);

/**
 * @brief Function used to start the timer for a specific delay.
 *
 * Most of the timer, it is assumed there is actually a non-blocking mechanism starting behing this
 * delay. It means it assumed there will be an interrupt occuring once delay is reached.
 *
 * @param[in] hdlr timer handler
 * @param[in] timeout_ms timeout duration in milliseconds
 *
 * @return  MCU_TIM_STATUS_OK if success. Error status otherwise.
 */
enum mcu_tim_status_t MCU_TIM_start(enum mcu_tim_hdlr hdlr, uint32_t timeout_ms);

/**
 * @brief Function used to get the timers counter
 *
 * Depending of the feature, this counter vallue is a decrementing or an incrementing timing in ms.
 *
 * @param[in] hdlr timer handler
 * @param[out] elapsed_time_ms from start of the timer, in milliseconds
 *
 * @return  MCU_TIM_STATUS_OK if success. Error status otherwise.
 */
enum mcu_tim_status_t MCU_TIM_getCount(enum mcu_tim_hdlr hdlr, uint32_t *elapsed_time_ms);

/**
 * @brief Function used to stop the counter of the timer.
 *
 * @param[in] hdlr timer handler
 *
 * @return  MCU_TIM_STATUS_OK if success. Error status otherwise.
 */
enum mcu_tim_status_t MCU_TIM_stop(enum mcu_tim_hdlr hdlr);

/**
 * @brief Function used to suspend timer.
 *
 * Depending on the application, the suspend/resume mechanism may entirely disable/re-enable the
 * timer or simply just postpone its internal tick
 *
 * @param[in] hdlr timer handler
 * @param[in] ctxt pointer to some potentiel context or configuration which may impact the behaviour
 * of suspend resume
 *
 * @return  MCU_TIM_STATUS_OK if success. Error status otherwise.
 */
enum mcu_tim_status_t MCU_TIM_suspend(enum mcu_tim_hdlr hdlr, void *ctxt);

/**
 * @brief Function used to restart the timer.
 *
 * Depending on the application, the suspend/resume mechanism may entirely disable/re-enable the
 * timer or simply just postpone its internal tick
 *
 * @param[in] hdlr timer handler
 * @param[in] ctxt pointer to some potentiel context or configuration which may impact the behaviour
 * of suspend resume
 *
 * @return  MCU_TIM_STATUS_OK if success. Error status otherwise.
 */
enum mcu_tim_status_t MCU_TIM_resume(enum mcu_tim_hdlr hdlr, void *ctxt);

#endif /* MCU_TIM_H_ */

/**
 * @}
 */
