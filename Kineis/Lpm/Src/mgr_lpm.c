/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    mgr_lpm.c
 * @brief   low power mode manager is in charge to collect all low power needs from 'clients' and
 *          setup the corresponding low power mode.
 * @author  Kineis
 */

/**
 * @addtogroup MGR_LPM
 * @{
 */

/* Includes ------------------------------------------------------------------------------------ */
#include <stddef.h>
#include "mgr_lpm.h"
#include "kineis_sw_conf.h"  // for assert include
#include KINEIS_SW_ASSERT_H
#include "kns_app_conf.h"  // for STM32 LPM definitions
#include STM32_HAL_H
#include "mgr_log.h"
#ifdef USE_BAREMETAL
#include "kns_q.h"
#endif


/* Defines ------------------------------------------------------------------------------------- */
#ifndef MGR_LPM_CLIENT_NBR_MAX
#define MGR_LPM_CLIENT_NBR_MAX 5
#endif


/* Enums --------------------------------------------------------------------------------------- */


/* Variables ----------------------------------------------------------------------------------- */
/**< table containing LPM manager client callbacks */
struct MgrLpmClientCb_t mgrLpmClientTab[MGR_LPM_CLIENT_NBR_MAX];
uint8_t mgrLpmClientNbr = 0;


/* Local functions ----------------------------------------------------------------------------- */
static enum MgrLpm_LPM_t eMGR_LPM_clientRequest(void);
static void vMGR_LPM_clientNotifyEnter(enum MgrLpm_LPM_t deepest_lpm);
static void vMGR_LPM_clientNotifyExit (enum MgrLpm_LPM_t deepest_lpm);
static void vMGR_LPM_enterSleep       (struct MgrLpm_EnvConfig_t env_config);
static void vMGR_LPM_enterStop        (struct MgrLpm_EnvConfig_t env_config);
static void vMGR_LPM_enterStandBy     (struct MgrLpm_EnvConfig_t env_config);
static void vMGR_LPM_enterShutdown    (struct MgrLpm_EnvConfig_t env_config);


/* Functions ----------------------------------------------------------------------------------- */
enum KNS_status_t MGR_LPM_init(struct MgrLpm_EnvConfig_t env_config)
{
	//> Configuration : allowedLPMbitmap should not be equal to 0. It means no LPM is enable.
	if (env_config.allowedLPMbitmap == 0)
		return KNS_STATUS_ERROR;

	for (uint8_t i = 0; i < MGR_LPM_CLIENT_NBR_MAX; i++) {
		mgrLpmClientTab[i].fpMGR_LPM_LpmReqCb        = NULL;
		mgrLpmClientTab[i].fpMGR_LPM_LpmNotifEnterCb = NULL;
		mgrLpmClientTab[i].fpMGR_LPM_LpmNotifExitCb  = NULL;
	}
	mgrLpmClientNbr = 0;
	return KNS_STATUS_OK;
}

enum KNS_status_t MGR_LPM_registerClient(struct MgrLpmClientCb_t mgrLpmClient)
{
	for (uint8_t i = 0; i < MGR_LPM_CLIENT_NBR_MAX; i++)
		if (mgrLpmClientTab[i].fpMGR_LPM_LpmReqCb == mgrLpmClient.fpMGR_LPM_LpmReqCb)
			return KNS_STATUS_ERROR;

	for (uint8_t i = 0; i < MGR_LPM_CLIENT_NBR_MAX; i++) {
		if (mgrLpmClientTab[i].fpMGR_LPM_LpmReqCb == NULL) {
			mgrLpmClientTab[i].fpMGR_LPM_LpmReqCb        = mgrLpmClient.fpMGR_LPM_LpmReqCb;
			mgrLpmClientTab[i].fpMGR_LPM_LpmNotifEnterCb = mgrLpmClient.fpMGR_LPM_LpmNotifEnterCb;
			mgrLpmClientTab[i].fpMGR_LPM_LpmNotifExitCb  = mgrLpmClient.fpMGR_LPM_LpmNotifExitCb;
			mgrLpmClientNbr++;
			return KNS_STATUS_OK;
		}
	}
	return KNS_STATUS_ERROR;
}

enum KNS_status_t MGR_LPM_unregisterClient(struct MgrLpmClientCb_t mgrLpmClient)
{
	for (uint8_t i = 0; i < MGR_LPM_CLIENT_NBR_MAX; i++) {
		if (mgrLpmClientTab[i].fpMGR_LPM_LpmReqCb == mgrLpmClient.fpMGR_LPM_LpmReqCb) {
			mgrLpmClientTab[i].fpMGR_LPM_LpmReqCb        = NULL;
			mgrLpmClientTab[i].fpMGR_LPM_LpmNotifEnterCb = NULL;
			mgrLpmClientTab[i].fpMGR_LPM_LpmNotifExitCb  = NULL;
			mgrLpmClientNbr--;
			return KNS_STATUS_OK;
		}
	}
	return KNS_STATUS_ERROR;
}

enum KNS_status_t MGR_LPM_enter(struct MgrLpm_EnvConfig_t env_config,
	struct MgrLpm_ctxt_t *mgr_lpm_ctxt)
{
	enum MgrLpm_LPM_t deepest_lpm;

	//> Configuration : allowedLPMbitmap should not be equal to 0. It means no LPM is enable.
	if (env_config.allowedLPMbitmap == 0)
		return KNS_STATUS_ERROR;

	//> Get the deepest LPM which clients can accept
	deepest_lpm = eMGR_LPM_clientRequest();
	
	//> Check if the deepest LPM exists
	while ((deepest_lpm & env_config.allowedLPMbitmap) == 0) {
		deepest_lpm = deepest_lpm >> 1;
		if (deepest_lpm == 0)
			break;
	}

	//> Notify each client with the deepest chosen LPM
	vMGR_LPM_clientNotifyEnter(deepest_lpm);

	/* Silence the "mode=NONE" spam: it fires every main-loop iteration
	 * (~10/s) when STOP is disabled, drowning every other trace. Only log
	 * mode transitions away from NONE which is the interesting case. */
	if (deepest_lpm != LOW_POWER_MODE_NONE)
		MGR_LOG_DEBUG("LPM enter: mode=0x%02X (allowed=0x%02X)\r\n",
			(unsigned int)deepest_lpm, (unsigned int)env_config.allowedLPMbitmap);

	switch (deepest_lpm) {
	case LOW_POWER_MODE_NONE:
		mgr_lpm_ctxt->low_power_mode = deepest_lpm;
		break;
	case LOW_POWER_MODE_SLEEP:
		mgr_lpm_ctxt->low_power_mode = deepest_lpm;
		vMGR_LPM_enterSleep(env_config);
		break;
	case LOW_POWER_MODE_STOP:
		mgr_lpm_ctxt->low_power_mode = deepest_lpm;
		vMGR_LPM_enterStop(env_config);
		break;
	case LOW_POWER_MODE_STANDBY:
		mgr_lpm_ctxt->low_power_mode = deepest_lpm;
		vMGR_LPM_enterStandBy(env_config);
		break;
	case LOW_POWER_MODE_SHUTDOWN:
		mgr_lpm_ctxt->low_power_mode = deepest_lpm;
		vMGR_LPM_enterShutdown(env_config);
		break;
	default:
		return KNS_STATUS_ERROR;
	}

	//> Notify each client after exiting LPM
	vMGR_LPM_clientNotifyExit(deepest_lpm);

	return KNS_STATUS_OK;
}


/* Local functions ----------------------------------------------------------------------------- */
/**
 * @brief internal API used to request client deepest LPM.
 *
 * @return The deepest LPM allowed by clients
 */
static enum MgrLpm_LPM_t eMGR_LPM_clientRequest(void)
{
	uint8_t index;
	
	enum MgrLpm_LPM_t client_deepest_lpm;
	enum MgrLpm_LPM_t deepest_lpm;

	deepest_lpm = LOW_POWER_MODE_SHUTDOWN;

	if (mgrLpmClientNbr == 0)
		return LOW_POWER_MODE_NONE;

	//> Request the deepest LPM allowed by clients
	for (index = 0; index < MGR_LPM_CLIENT_NBR_MAX; index++) {
		if (mgrLpmClientTab[index].fpMGR_LPM_LpmReqCb != NULL) {
			client_deepest_lpm = mgrLpmClientTab[index].fpMGR_LPM_LpmReqCb();
			if (client_deepest_lpm < deepest_lpm)
				deepest_lpm = client_deepest_lpm;
		}
	}

	return deepest_lpm;
}

/**
 * @brief internal API used to notify clients when entering into LPM
 *
 * @note Clients are notified from the client [0] to client [MGR_LPM_CLIENT_NBR_MAX-1]
 *
 * @param[in] deepest_lpm Low power mode which was chosen
 *
 * @return void
 */
static void vMGR_LPM_clientNotifyEnter(enum MgrLpm_LPM_t deepest_lpm)
{
	uint8_t index;

	//> Notify clients with the chosen LPM
	for (index = 0; index < MGR_LPM_CLIENT_NBR_MAX; index++) {
		if (mgrLpmClientTab[index].fpMGR_LPM_LpmNotifEnterCb != NULL) {
			if (!mgrLpmClientTab[index].fpMGR_LPM_LpmNotifEnterCb(deepest_lpm))
				kns_assert(0);
		}
	}
}

/**
 * @brief internal API used to notify clients when exiting LPM
 *
 * @note Clients are notified from the client [MGR_LPM_CLIENT_NBR_MAX-1] to client [0]
 *
 * @param[in] deepest_lpm Low power mode which was chosen
 *
 * @return void
 */
static void vMGR_LPM_clientNotifyExit(enum MgrLpm_LPM_t deepest_lpm)
{
	int8_t index;

	//> Notify clients with LPM used (reverse order, including client[0])
	for (index = MGR_LPM_CLIENT_NBR_MAX - 1; index >= 0; index--) {
		if (mgrLpmClientTab[index].fpMGR_LPM_LpmNotifExitCb != NULL) {
			if (!mgrLpmClientTab[index].fpMGR_LPM_LpmNotifExitCb(deepest_lpm))
				kns_assert(0);
		}
	}
}

/**
 * @brief This is the function to enter in sleep mode
 */
static void vMGR_LPM_enterSleep(struct MgrLpm_EnvConfig_t env_config)
{
	if (env_config.fp_sleep_enter != NULL)
		env_config.fp_sleep_enter();
	
	HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

	if (env_config.fp_sleep_exit != NULL)
		env_config.fp_sleep_exit();
}

/**
 * @brief This is the function to enter in stop mode
 */
static void vMGR_LPM_enterStop(struct MgrLpm_EnvConfig_t env_config)
{
	if (env_config.fp_stop_enter != NULL)
		env_config.fp_stop_enter();

	/* Preventive hardening (dormant — STOP is disabled in the production
	 * MONITORING profile via uw_doppler_lpmReq() returning NONE).
	 *
	 * Without these clears, a stale PWR_FLAG_STOPF / WUFI sticky from a
	 * previous wake makes WFI return immediately on the second STOP entry,
	 * which is the symptom captured in memory/lpm_stop_known_issue.md
	 * ("works once, never again on subsequent boots").
	 *
	 * STANDBY/SHUTDOWN paths below already do this clear. Adding it here
	 * means the moment someone re-enables STOP in MONITORING (by flipping
	 * uw_doppler_lpmReq()) the trap doesn't re-arm. */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);

#if defined(STM32WL55xx)
	/* PWR_C2CR1.LPMS caps the SYSTEM low-power mode (effective mode =
	 * shallowest of CR1/C2CR1, RM0453 6.5.3) and survives every reset
	 * except POR. Seen polluted to Stop0 on the bench: every deep-sleep
	 * request silently degraded (WKUP pins dead, IWDG kept counting).
	 * CPU2 never boots on this product — force "no floor" before entry. */
	MODIFY_REG(PWR->C2CR1, PWR_C2CR1_LPMS, PWR_LOWPOWERMODE_SHUTDOWN);
#endif

#if defined(STM32L476xx) || defined(STM32WLE5xx) || defined(STM32G491xx)
#ifdef USE_BAREMETAL
	if (!KNS_Q_isEvtInSomeQ()) // recheck all queues empty before LPM
#endif
		HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
#elif defined(STM32L071xx)
#ifdef USE_BAREMETAL
	if (!KNS_Q_isEvtInSomeQ()) // recheck all queues empty before LPM
#endif
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
#elif defined(STM32WL55xx)
#ifdef USE_BAREMETAL
	if (!KNS_Q_isEvtInSomeQ()) // recheck all queues empty before LPM
#endif
//		HAL_PWREx_EnterSTOP1Mode(PWR_STOPENTRY_WFI);
		HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
#else
#warning "[LPM MANAGER] Stop mode is not defined for this target."
#endif

	if (env_config.fp_stop_exit != NULL)
		env_config.fp_stop_exit();
}

/**
 * @brief This is the function to enter in standby mode
 */
static void vMGR_LPM_enterStandBy(struct MgrLpm_EnvConfig_t env_config)
{
	if (env_config.fp_standby_enter != NULL)
		env_config.fp_standby_enter();

	/* Clear all wakeup flags in status register. done just before entering sleep
	 * to ensure rising edge on wakeup will be occur during the sleep period
	 */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);
#if defined(STM32WL55xx)
	/* See vMGR_LPM_enterStop: CPU2 LPMS floor must be opened. */
	MODIFY_REG(PWR->C2CR1, PWR_C2CR1_LPMS, PWR_LOWPOWERMODE_SHUTDOWN);
#endif
	HAL_PWR_EnterSTANDBYMode();

	/* We should never reach this point as standby exit performs a reset of the uC */
	NVIC_SystemReset();
	kns_assert(0);
}

/**
 * @brief This is the function to enter in shutdown mode
 */
static void vMGR_LPM_enterShutdown(struct MgrLpm_EnvConfig_t env_config)
{
	if (env_config.fp_shutdown_enter != NULL)
		env_config.fp_shutdown_enter();

	/* Clear all wakeup flags in status register. done just before entering sleep
	 * to ensure rising edge on wakeup will be occur during the sleep period
	 */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);
#if defined(STM32WL55xx)
	/* See vMGR_LPM_enterStop: CPU2 LPMS floor must be opened. */
	MODIFY_REG(PWR->C2CR1, PWR_C2CR1_LPMS, PWR_LOWPOWERMODE_SHUTDOWN);
#endif
	HAL_PWREx_EnterSHUTDOWNMode();
	
	/* We should never reach this point as standby exit performs a reset of the uC */
	NVIC_SystemReset();
	kns_assert(0);
}
/**
 * @}
 */
