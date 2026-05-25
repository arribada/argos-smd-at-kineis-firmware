/**
 * @file    mgr_err.c
 * @brief   Error tracker using TAMP backup registers
 *
 * Uses STM32WL TAMP backup registers BKP2R-BKP7R to persist
 * error context across system resets. These registers survive
 * all resets (NRST, software, watchdog, brown-out) and are only
 * cleared on VBAT removal.
 *
 * Register map:
 *   BKP2R = reset counter (incremented each boot)
 *   BKP3R = last RCC_CSR flags (reset cause snapshot)
 *   BKP4R = last error code (MGR_ERR_Code_t)
 *   BKP5R = last UW_DOPPLER state at time of error
 *   BKP6R = last HAL tick at time of error
 *   BKP7R = reserved
 *
 * Init sequence: read previous session data, log it, then clear
 * error code for this session and increment boot counter.
 */

/**
 * @addtogroup MGR_ERR
 * @{
 */

#include "mgr_err.h"
#include "stm32wlxx.h"
#include "stm32wlxx_hal.h"
#include "mgr_log.h"

/* Use TAMP peripheral struct for backup register access */
#define ERR_BKP_COUNT   (TAMP->BKP2R)   /* Reset counter */
#define ERR_BKP_CSR     (TAMP->BKP3R)   /* Last RCC_CSR */
#define ERR_BKP_CODE    (TAMP->BKP4R)   /* Last error code */
#define ERR_BKP_STATE   (TAMP->BKP5R)   /* Last state */
#define ERR_BKP_TICK    (TAMP->BKP6R)   /* Last tick */
#define ERR_BKP_CRASH   (TAMP->BKP7R)   /* Consecutive crash counter */

__attribute__((unused)) static const char *err_code_str(MGR_ERR_Code_t code)
{
	switch (code) {
	case ERR_NONE:           return "NONE";
	case ERR_HARDFAULT:      return "HARDFAULT";
	case ERR_BUSFAULT:       return "BUSFAULT";
	case ERR_MEMMANAGE:      return "MEMMANAGE";
	case ERR_USAGEFAULT:     return "USAGEFAULT";
	case ERR_NMI:            return "NMI";
	case ERR_ASSERT:         return "ASSERT";
	case ERR_MAC_TIMEOUT:    return "MAC_TIMEOUT";
	case ERR_MAC_INIT_FAIL:  return "MAC_INIT_FAIL";
	case ERR_TX_TIMEOUT:     return "TX_TIMEOUT";
	case ERR_STACK_OVERFLOW: return "STACK_OVERFLOW";
	case ERR_WDG_RESET:      return "WDG_RESET";
	default:                 return "UNKNOWN";
	}
}

__attribute__((unused)) static const char *reset_cause_str(uint32_t csr)
{
	if (csr & RCC_CSR_IWDGRSTF) return "IWDG";
	if (csr & RCC_CSR_SFTRSTF)  return "SOFTWARE";
	if (csr & RCC_CSR_PINRSTF)  return "PIN";
	if (csr & RCC_CSR_BORRSTF)  return "BROWNOUT";
	return "POWER_ON";
}

void MGR_ERR_init(void)
{
	/* Backup access must already be enabled (enable_backup_access() in main.c) */

	/* Read previous session info before modifying. The `unused` attribute
	 * silences -Werror=unused-variable in DEBUG=0 builds where all
	 * MGR_LOG_DEBUG calls below collapse to do{}while(0). */
	uint32_t prev_count = ERR_BKP_COUNT;
	uint32_t prev_csr   __attribute__((unused)) = ERR_BKP_CSR;
	MGR_ERR_Code_t prev_err = (MGR_ERR_Code_t)ERR_BKP_CODE;
	uint32_t prev_state __attribute__((unused)) = ERR_BKP_STATE;
	uint32_t prev_tick  = ERR_BKP_TICK;

	/* Read current reset cause from RCC_CSR */
	uint32_t csr = RCC->CSR;

	/* If THIS boot was caused by IWDG but the previous session never logged
	 * an error code, the previous boot hung silently — surface that as
	 * WDG_RESET so the user sees it instead of last_err=NONE. tick stays 0
	 * because the previous boot never got far enough to record one. */
	if ((csr & RCC_CSR_IWDGRSTF) && prev_err == ERR_NONE) {
		prev_err = ERR_WDG_RESET;
	}

	/* Log the reset cause for THIS boot (most useful diagnostic).
	 * The previous-session summary follows for context. */
	MGR_LOG_DEBUG("[ERR] Boot #%lu reset_cause=%s csr=0x%08lx\r\n",
		prev_count + 1, reset_cause_str(csr), (unsigned long)csr);

	/* Log previous session info */
	MGR_LOG_DEBUG("[ERR] Boot #%lu: prev_reset=%s last_err=%s last_state=%lu tick=%lu\r\n",
		prev_count + 1,
		reset_cause_str(prev_csr),
		err_code_str(prev_err),
		prev_state,
		prev_tick);

	/* Crash loop detection: check if previous boot was short-lived */
	uint32_t prev_crash = ERR_BKP_CRASH;
	if (prev_tick > 0 && prev_tick < MGR_ERR_CRASH_LOOP_MIN_UP_MS && prev_err != ERR_NONE) {
		prev_crash++;
	} else {
		prev_crash = 0;  /* Previous boot was stable, reset counter */
	}

	/* Update registers for this session */
	ERR_BKP_COUNT = prev_count + 1;  /* Increment reset counter */
	ERR_BKP_CSR   = csr;             /* Store current reset cause */
	ERR_BKP_CODE  = ERR_NONE;        /* Clear error code */
	ERR_BKP_STATE = 0;
	ERR_BKP_TICK  = 0;
	ERR_BKP_CRASH = prev_crash;

	if (prev_crash > 0) {
		MGR_LOG_DEBUG("[ERR] Consecutive crashes: %lu\r\n", prev_crash);
	}

	/* Clear RCC reset flags for next reset detection */
	RCC->CSR |= RCC_CSR_RMVF;
}

void MGR_ERR_log(MGR_ERR_Code_t code)
{
	extern volatile uint32_t g_uw_doppler_state_for_err;
	ERR_BKP_CODE  = (uint32_t)code;
	ERR_BKP_STATE = g_uw_doppler_state_for_err;
	ERR_BKP_TICK  = HAL_GetTick();

	MGR_LOG_DEBUG("[ERR] Error logged: %s (tick=%lu)\r\n",
		err_code_str(code), HAL_GetTick());
}

void MGR_ERR_logAndReset(MGR_ERR_Code_t code)
{
	MGR_ERR_log(code);

	/* Drain the log ring buffer synchronously so the assert message reaches
	 * the host UART before reset. Without this the user only sees the next
	 * boot's banner with no clue what failed. Bounded loop: cap iterations
	 * to avoid wedging here if the UART itself is the failure cause. */
	for (int i = 0; i < 32 && MGR_LOG_has_pending(); i++)
		(void)MGR_LOG_flush();

	__disable_irq();
	NVIC_SystemReset();
	/* Never reaches here */
	while (1) {}
}

uint32_t MGR_ERR_getResetCount(void)
{
	return ERR_BKP_COUNT;
}

MGR_ERR_Code_t MGR_ERR_getLastError(void)
{
	return (MGR_ERR_Code_t)ERR_BKP_CODE;
}

uint32_t MGR_ERR_getCrashCount(void)
{
	return ERR_BKP_CRASH;
}

bool MGR_ERR_checkCrashLoop(void)
{
	if (ERR_BKP_CRASH < MGR_ERR_CRASH_LOOP_MAX)
		return false;

	MGR_LOG_DEBUG("[ERR] CRASH LOOP detected (%lu consecutive), safe sleep %us\r\n",
		ERR_BKP_CRASH, MGR_ERR_CRASH_LOOP_SLEEP_S);

	/* Configure RTC wakeup timer to wake after CRASH_LOOP_SLEEP_S seconds.
	 * RTC clock = LSE (32768 Hz), wakeup clock = CK_SPRE (1 Hz).
	 */
	__HAL_RCC_RTCAPB_CLK_ENABLE();

	/* Disable wakeup timer to modify it */
	RTC->CR &= ~RTC_CR_WUTE;
	{
		uint32_t timeout = 100000U;
		while ((RTC->ICSR & RTC_ICSR_WUTWF) == 0 && --timeout > 0)
			;
		if (timeout == 0)
			return false;  /* RTC not responding, skip safe sleep */
	}

	/* Select 1 Hz clock source (ck_spre) and set countdown */
	RTC->CR &= ~RTC_CR_WUCKSEL;
	RTC->CR |= RTC_CR_WUCKSEL_2;  /* WUCKSEL = 0b100 = ck_spre */
	RTC->WUTR = MGR_ERR_CRASH_LOOP_SLEEP_S - 1;

	/* Clear wakeup flag and enable wakeup timer + interrupt */
	RTC->SCR = RTC_SCR_CWUTF;
	RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;

	/* Enable RTC wakeup as EXTI line 17 (internal, rising edge only) */
	EXTI->IMR1 |= EXTI_IMR1_IM17;
	EXTI->RTSR1 |= (1UL << 17);
	EXTI->FTSR1 &= ~(1UL << 17);

	/* Enter STOP2 mode (lowest power with SRAM retention) */
	__disable_irq();
	HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
	__enable_irq();

	/* Woke up - disable wakeup timer */
	RTC->CR &= ~(RTC_CR_WUTE | RTC_CR_WUTIE);
	RTC->SCR = RTC_SCR_CWUTF;

	/* Reset crash counter to give the device another chance */
	ERR_BKP_CRASH = 0;

	MGR_LOG_DEBUG("[ERR] Woke from safe sleep, retrying\r\n");
	return true;
}

/**
 * @}
 */
