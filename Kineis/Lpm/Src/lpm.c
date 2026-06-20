// SPDX-License-Identifier: no SPDX license
/**
 * @file    lpm.c
 * @brief   This file contains some utilities and functions needed for a correct handling of LPM
 *          on the STM32WL55xx cortex M4 core.
 *
 *          This code is provided as an example by Kineis and is free to modifications as per
 *          integrator, application needs.
 * @author  Kineis
 */

/**
 * @addtogroup MGR_LPM
 * @{
 */

/* Includes ------------------------------------------------------------------------------------ */
#include <stdbool.h>

#include "main.h"
#include "usart.h"

#include "lpm.h"
#include "mgr_lpm.h"
#include "lpm_cli_kstk.h"
#include "mgr_log.h"
#include "rtc.h"
/* NB: lpm.c is shared by ALL apps; MGR_ERR is UW_DOPPLER-only on the include
 * path, so the RTC-liveness gate below resets via plain NVIC_SystemReset (no
 * MGR_ERR dependency). The forensic ERR_RTC_DEAD marker is logged on the
 * UW_DOPPLER duty path (mgr_lpm_uw.c); here the recovery boot's g_rtc_use_lsi=1
 * is the indicator. */

#if defined(USE_SPI_DRIVER)
#include "spi.h"
#include "mcu_spi_driver.h"
#endif
#if defined(USE_UW_DOPPLER_APP)
#include "adc.h"
#include "subghz.h"  /* HAL_SUBGHZ_DeInit before SHUTDOWN to drop radio ~500 µA */
#endif
#if defined(BSP_HAS_LED_RGB)
#include "mgr_led.h"
#endif
#if defined(BSP_HAS_REED_SWITCH)
#include "mgr_reed.h"
#endif

#pragma GCC visibility push(default)

/* Defines -------------------------------------------------------------------*/
#if defined(STM32WLE5xx) || defined(STM32WL55xx)
#define USART_ISR_RXNE USART_ISR_RXNE_RXFNE
#endif

/* Types --------------------------------------------------------------------------------------- */

/**
 * @brief structure containing context to backup during LPM.
 *
 * So far it is used to store the current LPM mode.
 *
 * @attention the 32-bit word storing the mow power mode must fitt the low power manager's structure
 * regarding low_power_mode parameter (\ref MgrLpm_ctxt_t)
 */
struct LPM_retentionReg_t {
	/** BKP0R register: 4 lower bits must follow MgrLpm_ctxt_t struct */
	__IO uint32_t reserved             : 28;
	__IO uint32_t low_power_mode       :  4; /**< current low power mode
						   * should follow MgrLpm_LPM_t enum
						   */
	/** BKP1R register */
	// \todo uncomment following lines to add data in next register
	//__IO uint32_t reserved    : 32;
};

/* Private functions prototypes ---------------------------------------------------------------- */

/** Function prototypes defined before variable as referenced in lpm_config variable */
static void LPM_sleep_enter();
static void LPM_sleep_exit();
static void LPM_stop_enter();
static void LPM_stop_exit();
static void LPM_standby_enter();
#ifdef LPM_SHUTDOWN_ENABLED
static void LPM_shutdown_enter();
#endif

/* Variables ----------------------------------------------------------------------------------- */

__attribute__((__section__(".retentionRamData")))
struct MgrLpm_EnvConfig_t lpm_config = {
	.allowedLPMbitmap  = LOW_POWER_MODE_NONE
#if defined(LPM_SLEEP_ENABLED)
			     | LOW_POWER_MODE_SLEEP
#elif defined(LPM_STOP_ENABLED)
			     | LOW_POWER_MODE_SLEEP | LOW_POWER_MODE_STOP
#elif defined(LPM_STANDBY_ENABLED)
			     | LOW_POWER_MODE_SLEEP | LOW_POWER_MODE_STOP | LOW_POWER_MODE_STANDBY
#elif defined(LPM_SHUTDOWN_ENABLED)
			     | LOW_POWER_MODE_SLEEP | LOW_POWER_MODE_STOP | LOW_POWER_MODE_STANDBY
			     | LOW_POWER_MODE_SHUTDOWN
#endif
			     ,
	.fp_sleep_enter    = LPM_sleep_enter,
	.fp_sleep_exit     = LPM_sleep_exit,
	.fp_stop_enter     = LPM_stop_enter,
	.fp_stop_exit      = LPM_stop_exit,
	.fp_standby_enter  = LPM_standby_enter,
#ifdef LPM_SHUTDOWN_ENABLED
	.fp_shutdown_enter = LPM_shutdown_enter
#else
	.fp_shutdown_enter = NULL
#endif
};

__attribute__((__section__(".lpmSection")))
struct LPM_retentionReg_t lpm_ctxt = {
	.low_power_mode = LOW_POWER_MODE_NONE
};

/** Forced LPM mode override consulted by KSTK_lpmReq. LOW_POWER_MODE_NONE
 * (0) means no override active.
 */
static enum MgrLpm_LPM_t lpm_forced_mode = LOW_POWER_MODE_NONE;

/* Private functions --------------------------------------------------------------------------- */

static bool LPM_configWakeUpUart(void)
{
	/* make sure that no UART transfer is on-going */
	/* make sure that UART is ready to receive
	 * (test carried out again later in HAL_UARTEx_StopModeWakeUpSourceConfig)
	 */
	uint32_t uart_timeout = 50000U; /* ~50ms at 48MHz */
	while ((__HAL_UART_GET_FLAG(&hlpuart1, USART_ISR_BUSY) == SET) ||
		(__HAL_UART_GET_FLAG(&hlpuart1, USART_ISR_RXNE) == SET) ||
		(__HAL_UART_GET_FLAG(&hlpuart1, USART_ISR_REACK) == RESET)) {
		if (--uart_timeout == 0U)
			return false;
	}

	/* set the UART wake-up event:
	 * specify wake-up on start-bit detection
	 */
	UART_WakeUpTypeDef WakeUpSelection;

	WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_READDATA_NONEMPTY;
	if (HAL_UARTEx_StopModeWakeUpSourceConfig(&hlpuart1, WakeUpSelection)
			!= HAL_OK)
		return false;

	/* Enable the UART Wake UP from STOP mode Interrupt */
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_WUF);

	/* enable MCU wake-up by UART */
	HAL_UARTEx_EnableStopMode(&hlpuart1);

	return true;
}


/** @brief System Clock Configuration when exit from stop mode
 *
 * @note This function is inspired from SystemClock_Config Core function generated by STM32CubeMX
 *       in main.c. This version is trying to retrieve original configuration instead of direct
 *       duplication.
 */
static void LPM_SystemClock_Config_RestoreFromStop(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  uint32_t pFLatency = 0;

  /* Enable Power Control clock */
#if defined(STM32WLE5xx) || defined(STM32WL55xx)
  HAL_PWR_EnableBkUpAccess();
#else
  __HAL_RCC_PWR_CLK_ENABLE();
#endif

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* After wake-up from Stop reconfigure the system clock: Enable HSI and PLL */
  RCC_OscInitStruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSIState        = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState    = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Get the Clocks configuration according to the internal RCC registers */
  HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &pFLatency);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
   * clocks dividers
   */
  RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, pFLatency) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief Function used to configure the external wakeup pins to exit low power mode (standby and
 *        shutdown only)
 *
 * @note So far, it is coded to exit standby or shutdown only.
 * @note So far, the wakeup pin is directly hardcoded in the core fo this function:
 *       * WKUP2 (PC13, blue user button) falling edge.
 *
 * @todo Need to add input parameter to make it generic regarding the wakeup pins configuration.
 */
static void LPM_configWakeUpPins(void)
{
	/* Disable all wakeup first */
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
	HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN3);

	/* clear all wakeup flags in status register */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

	/* But enable wakeup with:
	 * * falling edge on wakeup pin 2, i.e. PC13,  user blue button
	 * Pressing the user button will exit from shutdown mode.
	 */
	HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN3_HIGH);
}

/**
 * @brief Function used to configure the internal wakeup line to exit low power mode
 *
 * @note So far, our internal line needs is RTC alarms and wakeup timer
 */
static void LPM_configWakeUpRtc(void)
{

#if defined(STM32L476xx) || defined(STM32WLE5xx) || defined(STM32WL55xx)
	HAL_PWREx_DisableInternalWakeUpLine();

	/* clear wakeup flags in status register */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);

	/* But enable wakeup with:
	 * * internal line (RTC alarm, wakup timer)
	 */
	HAL_PWREx_EnableInternalWakeUpLine();
#endif
}

/** @brief System callback invoked by MGR_LPM at SLEEP mode entering */
static void LPM_sleep_enter() {
//	MGR_LOG_DEBUG("==== SLEEP enter ====\r\n");
	HAL_SuspendTick();
	/** force renabling interrupt as wakeup from UART is needed */
	__enable_fault_irq();
	__enable_irq();
}

/** @brief System callback invoked by MGR_LPM at SLEEP mode exit */
static void LPM_sleep_exit() {
	HAL_ResumeTick();
//	MGR_LOG_DEBUG("==== SLEEP exit ====\r\n");
}

/* RTC tick compensation: save RTC time before STOP to compensate HAL_GetTick() on exit.
 * Exported (lpm.h): the MGR_LPM_UW STOP2/STOP1 paths use the same pair so every
 * software timer (TX schedule, seq-restart, SWS dive watchdog, AT grace) keeps
 * counting WALL-CLOCK time across sleep instead of CPU-active time. */
static uint32_t lpm_rtc_subsec_before_stop = 0;
static uint32_t lpm_rtc_sec_before_stop = 0;

void LPM_saveRtcTime(void)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); /* Must read date after time per RM */
	lpm_rtc_subsec_before_stop = sTime.SubSeconds;
	lpm_rtc_sec_before_stop = sTime.Hours * 3600 + sTime.Minutes * 60 + sTime.Seconds;
}

void LPM_compensateTick(void)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	uint32_t sec_after = sTime.Hours * 3600 + sTime.Minutes * 60 + sTime.Seconds;

	/* Compute elapsed seconds (handle midnight wrap) */
	uint32_t elapsed_s;
	if (sec_after >= lpm_rtc_sec_before_stop)
		elapsed_s = sec_after - lpm_rtc_sec_before_stop;
	else
		elapsed_s = (86400 - lpm_rtc_sec_before_stop) + sec_after;

	/* Compute elapsed sub-seconds (RTC sub-second is a down-counter) */
	uint32_t prediv_s = hrtc.Init.SynchPrediv;
	uint32_t subsec_before_ms = ((prediv_s - lpm_rtc_subsec_before_stop) * 1000) / (prediv_s + 1);
	uint32_t subsec_after_ms = ((prediv_s - sTime.SubSeconds) * 1000) / (prediv_s + 1);

	/* Total elapsed ms */
	uint32_t elapsed_ms;
	if (elapsed_s > 0) {
		/* Cap to avoid uint32 overflow (49+ days in STOP) */
		if (elapsed_s > 4000000UL)
			elapsed_s = 4000000UL;
		elapsed_ms = (elapsed_s - 1) * 1000 + (1000 - subsec_before_ms) + subsec_after_ms;
	} else {
		if (subsec_after_ms >= subsec_before_ms)
			elapsed_ms = subsec_after_ms - subsec_before_ms;
		else
			elapsed_ms = 0;
	}

	/* Guard against bogus elapsed values (RTC shadow stale, subsec under-
	 * flow, etc.) that would corrupt uwTick. A single STOP cycle inside the
	 * main app loop is at most a few seconds; the worst legitimate case is
	 * a few hours of inactivity. Anything past 24 h is garbage. */
	if (elapsed_ms > (24UL * 3600UL * 1000UL))
		return;

	extern __IO uint32_t uwTick;
	uwTick += elapsed_ms;
}

/** @brief System callback invoked by MGR_LPM at STOP mode entering */
static void LPM_stop_enter() {
//	MGR_LOG_DEBUG("==== STOP enter ====\r\n");
	HAL_SuspendTick();
	LPM_saveRtcTime();

#if defined(USE_UW_DOPPLER_APP)
	/* De-init ADC for power savings during STOP */
	MX_ADC_DeInit();
#endif

	GPIO_DisableAllToAnalogInput();
	/** Configure and re-enable interrupt as wakeup from UART is needed in case of GUI APP.
	 *
	 * @attention, This should not be done in case of standalone APP, as long as UART reception
	 * is not fully configured (need to call UART_Start_Receive_IT or KINEIS_UART_StartRx_IT
	 * at init)
	 *
	 * */
	LPM_configWakeUpUart();

#if defined(USE_SPI_DRIVER)
	/* Configure SPI NSS (PA15) as EXTI falling edge to wake from STOP.
	 * The SPI peripheral is stopped during STOP mode, so the first NSS
	 * assertion will only wake the MCU — the actual SPI transaction is lost.
	 * The host must retry after the wakeup. */
	{
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		HAL_SPI_DeInit(&hspi1);
		__HAL_RCC_GPIOA_CLK_ENABLE();
		GPIO_InitStruct.Pin = GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
		HAL_NVIC_SetPriority(EXTI15_10_IRQn, 2, 0);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	}
#endif

	__enable_fault_irq();
	__enable_irq();
}

/** @brief System callback invoked by MGR_LPM at STOP mode exit */
static void LPM_stop_exit() {
	/* Wake Up on start bit detection successful */
	LPM_SystemClock_Config_RestoreFromStop();

	/* Resume SysTick before compensating */
	HAL_ResumeTick();

	/* Compensate HAL tick for time spent in STOP mode */
	LPM_compensateTick();

#if defined(USE_SPI_DRIVER)
	/* Re-arm SPI BEFORE the UART-recovery delay below, so the slave is
	 * listening as soon as possible after the NSS-EXTI wake — this is what
	 * minimizes the window in which the host's retry frame is dropped. NSS was
	 * reconfigured as EXTI in stop_enter; restore it as SPI AF and re-arm the
	 * slave DMA so the next master transaction is captured (MX_SPI1_Init alone
	 * leaves the peripheral idle with no DMA armed). The frame that triggered
	 * the wake is inherently lost — the host must retry after wakeup. */
	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	MX_SPI1_Init();
	MCU_SPI_DRIVER_read();
#endif

	HAL_UARTEx_DisableStopMode(&hlpuart1);
	HAL_Delay(100); /** So far need to add some delay at exit before being able to receive a new
			 * AT command from UART link.
			 * @note same delay used in STM32 examples
			 */

#if defined(USE_UW_DOPPLER_APP)
	/* Re-init ADC after STOP mode */
	MX_ADC_Init();
#endif

	/* Re-init LED and REED GPIOs after STOP mode (GPIO_DisableAllToAnalogInput
	 * should have preserved them, but re-init for robustness) */
#if defined(BSP_HAS_LED_RGB)
	MGR_LED_init();
#endif
#if defined(BSP_HAS_REED_SWITCH)
	MGR_REED_init();
#endif

//	MGR_LOG_DEBUG("==== STOP exit ====\r\n");
}

/** @brief System callback invoked by MGR_LPM at STANDBY mode entering
 *
 * @attention Wake-up source is WKUP3 (PB3) with rising-edge polarity. On
 * STM32WL55, enabling EWUP3 **automatically disables the internal pull on PB3**
 * (RM0453 §5.4). PB3 is therefore floating during STANDBY unless an external
 * pull-down resistor (10 kΩ to GND) is wired, or the master holds PB3 LOW
 * actively between wake-up events. Otherwise noise on PB3 can either:
 *   - Trigger a spurious immediate wake-up (false wake)
 *   - Fail to register a clean rising edge from the master (no wake)
 */
static void LPM_standby_enter() {
	/* Log PB3 state and SR1 right before entry so user can verify pin is
	 * LOW (mandatory for rising-edge wake) and no WUFx is pending */
	GPIO_PinState pb3_state __attribute__((unused)) = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	uint32_t pwr_sr1 __attribute__((unused)) = PWR->SR1;
	MGR_LOG_DEBUG("==== STANDBY enter ==== PB3=%u PWR_SR1=0x%08lX\r\n",
		(unsigned int)pb3_state, (unsigned long)pwr_sr1);

#if defined(USE_UW_DOPPLER_APP)
	/* Tear down ADC + clock so the peripheral is in a known state on wake.
	 * Without this, ADC stays active during STANDBY/reset and the next
	 * MX_ADC_Init runs on a half-initialized peripheral → calibration
	 * failure → erratic readings → bad MAC decisions → boot loop. */
	MX_ADC_DeInit();
#else
	/* GUI/STDLN/DOPPLER reach STANDBY (not SHUTDOWN) at rest, and the shared
	 * STANDBY path never tore down the SubGHz radio — its bias network keeps
	 * ~500 µA flowing, which is the bulk of the observed STANDBY floor. STANDBY
	 * exit cold-resets the MCU so MX_SUBGHZ_Init re-arms the radio at boot — no
	 * re-init pairing needed. (UW_DOPPLER does this in its own MGR_LPM_UW path.) */
	{
		extern SUBGHZ_HandleTypeDef hsubghz;
		(void)HAL_SUBGHZ_DeInit(&hsubghz);
	}
#endif

	GPIO_DisableAllToAnalogInput();
	HAL_PWREx_EnablePullUpPullDownConfig();
	/** Force pull down on wakeup pin: PB3, or PC13 or PA0
	 *  NOTE: this is overridden by HAL_PWR_EnableWakeUpPin() below — see
	 *  function docstring. Kept for non-WKUP wake configurations. */
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_3);

	// 2) Program the desired pulls for Standby via PWR (per port/bit)
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PA_PSU_EN_Pin);
	/* PA_PSU_SEL / VSEL must stay HIGH during STANDBY/SHUTDOWN so the
	 * TPS63901 stays in 3V3 mode for the next wake. On STDALONE the
	 * external R11 (10M to VBAT) is too weak alone; enable the PWR
	 * controller pull-up to keep PC1 anchored HIGH while the GPIO is off. */
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_C, PA_PSU_SEL_Pin);
#if defined(STM32WL55xx)
	HAL_PWREx_EnableSRAMRetention();
#else
	HAL_PWREx_EnableSRAM2ContentRetention();
#endif
	LPM_configWakeUpRtc();
	LPM_configWakeUpPins();
	__HAL_RCC_CLEAR_RESET_FLAGS();

	/** Disable all peripherals before going to STANDBY LPM. As STDBY exit leads to reset of the
	 * uC, all peripherals will be restarted through normal wake-up sequence (cf main fct).
	 *
	 * @note RTC peripheral must remain ON in LPM, as it is in charge to exit LPM for
	 * periodic transmission or at SAT pass start and end.
	 *
	 * @note Only RX line of UART is disabled. Disabling UART peripheral entirely may leads to
	 * instabilities if not correctly disabled on host side as well. Refer to HAL_UART_MspDeInit
	 * for details about DeInit sequence
	 */
	// disable UART interrupt and RX GPIO
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);
	HAL_NVIC_DisableIRQ(LPUART1_IRQn);
}

#ifdef LPM_SHUTDOWN_ENABLED
/** @brief System callback invoked by MGR_LPM at SHUTDOWN mode entering */
static void LPM_shutdown_enter() {
	MGR_LOG_DEBUG("==== SHUTDOWN enter ====\r\n");

#if defined(USE_UW_DOPPLER_APP)
	/* Same reason as STANDBY entry: leave ADC peripheral in known state
	 * so the next boot's MX_ADC_Init starts from a clean slate. */
	MX_ADC_DeInit();

	/* SubGHz peripheral keeps ~500 µA flowing through its bias network
	 * even when idle. STOP2 path already does this teardown via
	 * MGR_LPM_UW_enterStop2Timed; the SHUTDOWN path was missing it which
	 * is why post-gesture power-down stayed at ~135 µA instead of the
	 * ~5 µA target seen on the older firmware. HAL_SUBGHZ_DeInit gates
	 * the peripheral clock and puts the radio in reset — equivalent of
	 * what the chip itself does cold-booting from SHUTDOWN. */
	extern SUBGHZ_HandleTypeDef hsubghz;
	(void)HAL_SUBGHZ_DeInit(&hsubghz);

	/* VBAT_EN HIGH leaks ~30 µA through the 120 k + 300 k divider chain.
	 * Drive it LOW before the GPIO goes analog so the divider is broken
	 * for the duration of SHUTDOWN. */
#if defined(BSP_HAS_VBAT_ADC)
	HAL_GPIO_WritePin(VBAT_EN_GPIO_Port, VBAT_EN_Pin, GPIO_PIN_RESET);
#endif
#else
	/* Non-UW apps that are host-forced into SHUTDOWN must also drop the SubGHz
	 * radio (~500 µA) — same rationale as the STANDBY path. SHUTDOWN exit cold-
	 * boots so MX_SUBGHZ_Init re-arms the radio. */
	{
		extern SUBGHZ_HandleTypeDef hsubghz;
		(void)HAL_SUBGHZ_DeInit(&hsubghz);
	}
#endif

	GPIO_DisableAllToAnalogInput();
	HAL_PWREx_EnablePullUpPullDownConfig();
	/** Force pull down on wakeup pin: PB3, or PC13 or PA0 */
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_3);

	// 2) Program the desired pulls for Standby via PWR (per port/bit)
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PA_PSU_EN_Pin);
	/* PA_PSU_SEL / VSEL must stay HIGH during STANDBY/SHUTDOWN so the
	 * TPS63901 stays in 3V3 mode for the next wake. On STDALONE the
	 * external R11 (10M to VBAT) is too weak alone; enable the PWR
	 * controller pull-up to keep PC1 anchored HIGH while the GPIO is off. */
	HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_C, PA_PSU_SEL_Pin);

#if defined(BSP_HAS_PWR_LATCH)
	/* Pull PWR_LATCH (PB7) LOW to cut power on STDALONE board */
	HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_7);
#endif

	LPM_configWakeUpRtc();
	LPM_configWakeUpPins();
	__HAL_RCC_CLEAR_RESET_FLAGS();
}
#endif

/* Functions ----------------------------------------------------------------------------------- */

void LPM_SystemClockConfig(void)
{
	/* =================== SHUTDOWN/STANDBY support ============================= */
	/* Enable Power Clock */
#if defined(STM32WLE5xx) || defined(STM32WL55xx)
	HAL_PWR_EnableBkUpAccess();
#else
	__HAL_RCC_PWR_CLK_ENABLE();
#endif

	/* =================== SLEEP/STOP support ============================= */
	/* Disable Prefetch Buffer */
	__HAL_FLASH_PREFETCH_BUFFER_DISABLE();
	/* Reset all RCC Clock-enable in Sleep and Stop modes but LPTIM1 and LPUART in a way to
	 * improve current drain.
	 */
	/* \note: sounds like no official HAL API exists to configure clock sleep */
	RCC->AHB1SMENR  = 0x0;
	RCC->AHB2SMENR  = 0x0;
	RCC->AHB3SMENR  = 0x0;
	RCC->APB1SMENR1 = 0x0;
	RCC->APB1SMENR2 = 0x0;
	RCC->APB2SMENR  = 0x0;
//	__HAL_RCC_LPTIM1_CLK_SLEEP_ENABLE();
	__HAL_RCC_LPUART1_CLK_SLEEP_ENABLE();
	__HAL_RCC_RTCAPB_CLK_SLEEP_ENABLE();

#if defined(USE_SPI_DRIVER)
	/* Keep SPI1, DMA1/DMAMUX1 and SPI GPIO banks clocked in SLEEP mode,
	 * otherwise the slave cannot shift data and no DMA IT fires to wake us. */
	__HAL_RCC_SPI1_CLK_SLEEP_ENABLE();
	__HAL_RCC_DMA1_CLK_SLEEP_ENABLE();
	__HAL_RCC_DMAMUX1_CLK_SLEEP_ENABLE();
	__HAL_RCC_GPIOA_CLK_SLEEP_ENABLE();   /* NSS PA15, SCK PA1 */
	__HAL_RCC_GPIOB_CLK_SLEEP_ENABLE();   /* MISO PB4, MOSI PB5 */
#endif

#if defined(USE_UW_DOPPLER_APP)
	/* SLEEP-mode peripheral enables for the Kineis MAC + UW_DOPPLER app.
	 *
	 * The aggregator zeroed all APBxSMENR above. Re-enable the bits the
	 * MAC stack and app need to function in SLEEP so the chip can drop
	 * to WFI between events without breaking scheduling:
	 *
	 *  - TIM16     : MAC TX timeout timer. Per lpm_cli_kstk.c:67 the MAC
	 *                explicitly returns SLEEP (not STOP) during TX so this
	 *                timer can fire. Without its sleep clock the timer
	 *                stops, TX times out, MAC state corrupts → IWDG fires.
	 *                This is the proximate cause of the 2026-06-06 SLEEP
	 *                regression (see memory/lpm_sleep_attempt.md).
	 *  - SUBGHZSPI : radio bus, needed for any RF activity in flight.
	 *  - GPIOA/B/C : reed EXTI (PB6), LED outputs, reed PWR_LATCH (PB7).
	 *                Without sleep clock, EXTI edge detection still works
	 *                but writing GPIO state would fail until exit.
	 *  - DMA1 / DMAMUX1 : in case the MAC uses DMA for radio transfers.
	 *  - PWR     : keep PWR controller alive so we can manipulate flags.
	 *
	 * Cost: each enabled peripheral keeps its low-power clock running
	 * (~few hundred nA each). Total budget impact ≪ 100 µA, dwarfed by
	 * the multi-mA win from putting Cortex into WFI most of the time. */
	/* CPU-execution prerequisites in SLEEP — without these the ISR
	 * vector fetch fails on the very first SysTick wake → tick stops
	 * incrementing → state machine appears frozen + IWDG fires.
	 * Root-caused 2026-06-07. */
	__HAL_RCC_FLASH_CLK_SLEEP_ENABLE();   /* instruction fetch */
	__HAL_RCC_SRAM1_CLK_SLEEP_ENABLE();   /* .data / .bss / stack */
	__HAL_RCC_SRAM2_CLK_SLEEP_ENABLE();   /* Kineis ctxt + retention */

	/* Peripherals the MAC stack + UW_DOPPLER need awake during SLEEP:
	 *  - TIM16     : MAC TX timeout timer (lpm_cli_kstk.c:67 — MUST
	 *                run during SLEEP, MAC explicitly only allows
	 *                SLEEP not STOP during TX for this reason).
	 *  - SUBGHZSPI : radio bus.
	 *  - GPIOA/B/C : LED outputs, reed EXTI, PWR_LATCH.
	 *  - DMA1 / DMAMUX1 : any radio DMA transfers. */
	__HAL_RCC_TIM16_CLK_SLEEP_ENABLE();
	__HAL_RCC_SUBGHZSPI_CLK_SLEEP_ENABLE();
	__HAL_RCC_GPIOA_CLK_SLEEP_ENABLE();
	__HAL_RCC_GPIOB_CLK_SLEEP_ENABLE();
	__HAL_RCC_GPIOC_CLK_SLEEP_ENABLE();
	__HAL_RCC_DMA1_CLK_SLEEP_ENABLE();
	__HAL_RCC_DMAMUX1_CLK_SLEEP_ENABLE();
#endif


	/* =================== STOP support ============================= */
	/* Configure the wake up from stop clock, back to full speed HSI. From System clock MUX,
	 * full speed should be back as well then.
	 */
//	__HAL_RCC_WAKEUPSTOP_CLK_CONFIG(RCC_STOP_WAKEUPCLOCK_HSI);
	HAL_RCCEx_WakeUpStopCLKConfig(RCC_STOP_WAKEUPCLOCK_HSI);
	/* =============================================================== */
}

void GPIO_DisableAllToAnalogInput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Enable all GPIO clocks for configuration */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();

	/* ---- GPIOA: Set all unused pins to analog (high impedance) ----
	 * Active pins NOT set to analog:
	 *   PA2  = LPUART1_TX
	 *   PA3  = LPUART1_RX
	 *   PA13 = SWDIO
	 *   PA14 = SWCLK
	 *   SPI mode only: PA1 = SCK, PA15 = NSS
	 *   UW_DOPPLER mode: PA11 = ADC_IN7 (SWS input), PA12 = SWS power control
	 */
#if defined(USE_SPI_DRIVER)
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
	                      GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
#if !defined(USE_UW_DOPPLER_APP)
	                      | GPIO_PIN_11 | GPIO_PIN_12
#endif
	                      ;
#elif defined(USE_UART_DRIVER)
	GPIO_InitStruct.Pin = GPIO_PIN_0
#if !defined(SMD_STDALONE)
	                      | GPIO_PIN_1    /* PA1 = LED_RED on STDALONE */
#endif
	                      | GPIO_PIN_4 | GPIO_PIN_5 |
	                      GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8
#if !defined(SMD_STDALONE)
	                      | GPIO_PIN_9 | GPIO_PIN_10   /* PA9/PA10 = I2C on STDALONE */
#endif
#if !defined(USE_UW_DOPPLER_APP)
	                      | GPIO_PIN_11 | GPIO_PIN_12
#endif
	                      | GPIO_PIN_15;
#endif
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* ---- GPIOB: Set all unused pins to analog ----
	 * Active pins NOT set to analog:
	 *   PB3 = EXT_WKUP_BUTTON / SWO
	 *   SPI mode only: PB4 = MISO, PB5 = MOSI
	 *   SMD_STDALONE: PB4=LED_GREEN, PB5=LED_BLUE, PB6=REED, PB7=PWR_LATCH, PB9=VBAT_EN
	 */
#if defined(USE_SPI_DRIVER)
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
	                      GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
	                      GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
	                      GPIO_PIN_14 | GPIO_PIN_15;
#elif defined(USE_UART_DRIVER)
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
#if !defined(SMD_STDALONE)
	                      GPIO_PIN_4 | GPIO_PIN_5 |
	                      GPIO_PIN_6 | GPIO_PIN_7 |
#endif
	                      GPIO_PIN_8 |
#if !defined(SMD_STDALONE)
	                      GPIO_PIN_9 |
#endif
	                      GPIO_PIN_10 | GPIO_PIN_11 |
	                      GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
#endif
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* ---- GPIOC: Set all unused pins to analog ----
	 * Active pins NOT set to analog: PC0 = PA_PSU_EN, PC1 = PA_PSU_SEL
	 * On STDALONE, PC1 also goes to analog because it's wired to TPS63901 SEL
	 * and must stay high-Z (R11 10M pull-up holds it HIGH externally).
	 */
	GPIO_InitStruct.Pin =
#if defined(SMD_STDALONE)
	                      GPIO_PIN_1 |
#endif
	                      GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 |
	                      GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 |
	                      GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 |
	                      GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* ---- GPIOH: Set PH3 to analog ---- */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/* Ensure PSU pins maintain correct output state */
	GPIO_InitStruct.Pin = PA_PSU_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PA_PSU_EN_GPIO_Port, &GPIO_InitStruct);

#if !defined(SMD_STDALONE)
	GPIO_InitStruct.Pin = PA_PSU_SEL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(PA_PSU_SEL_GPIO_Port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(PA_PSU_SEL_GPIO_Port, PA_PSU_SEL_Pin, GPIO_PIN_SET);
#endif
	HAL_GPIO_WritePin(PA_PSU_EN_GPIO_Port, PA_PSU_EN_Pin, GPIO_PIN_RESET);
}

void LPM_init(void)
{
	MGR_LPM_init(lpm_config);
	MGR_LPM_registerClient(mgrLpmCliKstk);
}

void LPM_enter(void)
{
	MGR_LPM_enter(lpm_config, (struct MgrLpm_ctxt_t *)&lpm_ctxt);

}
void LPM_forceMode(enum MgrLpm_LPM_t low_power_mode)
{
	lpm_ctxt.low_power_mode = low_power_mode;
}

inline enum MgrLpm_LPM_t LPM_getMode(void)
{
	return (enum MgrLpm_LPM_t) lpm_ctxt.low_power_mode;
}

void LPM_setForcedMode(enum MgrLpm_LPM_t mode)
{
	switch (mode) {
	case LOW_POWER_MODE_NONE:
	case LOW_POWER_MODE_SLEEP:
	case LOW_POWER_MODE_STOP:
	case LOW_POWER_MODE_STANDBY:
	case LOW_POWER_MODE_SHUTDOWN:
		lpm_forced_mode = mode;
		break;
	default:
		/* Invalid value silently ignored to keep the API misuse-safe */
		break;
	}
}

enum MgrLpm_LPM_t LPM_getForcedMode(void)
{
	return lpm_forced_mode;
}

void LPM_shutdownNow(void)
{
	LPM_shutdownWithAutoWake(0);
}

void LPM_shutdownWithAutoWake(uint32_t wakeup_seconds)
{
#ifdef LPM_SHUTDOWN_ENABLED
	/* RTC-liveness gate (runtime LSE-death recovery): an auto-wake SHUTDOWN
	 * relies on the RTC to cold-boot the unit at the deadline. If the LSE
	 * crystal died mid-mission the RTC isn't ticking and the wake would NEVER
	 * fire -> permanent brick of a sealed unit. HAL_RTC_WaitForSynchro times
	 * out only when RTCCLK is dead; reset so the boot LSE->LSI fallback
	 * (brick #2) re-inits the RTC on LSI. The wakeup_seconds==0 magnet-only
	 * true-off path uses the HW reed latch (no RTC) and is exempt. */
	if (wakeup_seconds > 0 && HAL_RTC_WaitForSynchro(&hrtc) != HAL_OK) {
		extern void SystemClock_armLsiFallback(void);
		SystemClock_armLsiFallback();   /* force LSI on the recovery boot */
		NVIC_SystemReset();             /* boot re-inits the RTC on LSI (brick #2) */
		/* never returns */
	}

	/* Run the same teardown that the MGR_LPM aggregator would: ADC deinit,
	 * GPIO to analog, pull-up/down config, wake-up RTC + pins armed. */
	LPM_shutdown_enter();

	/* Always disarm any leftover RTC wake-up timer before SHUTDOWN. If the
	 * lib (or anyone else) armed the WUT during operation — e.g. the MAC
	 * L1 timer via MCU_TIM_HDLR_TX_PERIOD — and we enter SHUTDOWN without
	 * stopping it, the WUT fires within seconds and immediately cold-boots
	 * the chip. That defeats the entire point of the sealed end-of-mission
	 * path. Observed on the bench: enter_shutdown() → SHUTDOWN entered →
	 * woke ~0.2 s later with PWR_SR1.WUFI set. */
	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);

	/* For the no-auto-wake case (wakeup_seconds == 0, e.g. magnet-only
	 * end-of-mission), kill EVERY wake source the chip exposes so the
	 * SHUTDOWN actually sticks. The reed-magnet wake on STDALONE is a
	 * HW path (regulator latch via PB7) and doesn't go through any
	 * of these — it brings the board back by re-energising VDD, which
	 * triggers a cold POR reset, not a wake-from-SHUTDOWN.
	 *
	 * Observed pre-fix: with USB power propping VDD up during the
	 * "shutdown" (debug bench scenario), the chip would enter SHUTDOWN
	 * then wake within milliseconds via WUFI → SHUTDOWN → wake … in a
	 * tight loop averaging ~440 µA. */
	if (wakeup_seconds == 0) {
		/* RTC alarms + WUT (in case anything other than the WUT we
		 * just disarmed is sitting armed). */
		(void)HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
		(void)HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B);
		__HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRAF | RTC_FLAG_ALRBF);

		/* Internal wake-up line: gate all PWR-side internal sources. */
		HAL_PWREx_DisableInternalWakeUpLine();
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);

		/* WKUP pins: even though LPM_configWakeUpPins() arms WKUP3
		 * for "operator-pressed PB3 wakes the board" semantics on
		 * SMD_PA/NOPA/OP, the reed-magnet path on STDALONE doesn't
		 * use it (reed = PB6 ≠ WKUP pin) and a noisy floating PB3
		 * would burn the SHUTDOWN attempt. Disable all three. */
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2);
		HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN3);
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	}

	/* Optional RTC wake-up timer for sealed-deployment auto-recovery.
	 * Mirrors the pattern in kns_app_doppler.c:enter_shutdown(). */
	if (wakeup_seconds > 0) {
		HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

		/* CK_SPRE is 1 Hz: 16-bit counter covers 1..65 536 s; 17-bit
		 * adds 65 537..131 072. Clamp at 17-bit max to avoid silent
		 * truncation in the (counter-1) arithmetic. */
		if (wakeup_seconds > 0x20000u)
			wakeup_seconds = 0x20000u;

		uint32_t clk_src = (wakeup_seconds > 0x10000u)
			? RTC_WAKEUPCLOCK_CK_SPRE_17BITS
			: RTC_WAKEUPCLOCK_CK_SPRE_16BITS;
		uint16_t counter = (uint16_t)((wakeup_seconds - 1u) & 0xFFFFu);

		(void)HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, counter, clk_src, 0);

		/* Enable the internal wake-up line so RTC wake reaches the PWR
		 * controller. Without this the timer fires but the chip doesn't
		 * exit SHUTDOWN. */
		HAL_PWREx_DisableInternalWakeUpLine();
		HAL_PWREx_EnableInternalWakeUpLine();

#if defined(BSP_HAS_PWR_LATCH)
		/* Auto-wake SHUTDOWN MUST keep the board powered: LPM_shutdown_enter()
		 * above set a PB7 pull-DOWN (the true-power-off default), which on
		 * STDALONE opens the regulator latch, drops VDD, kills the RTC backup
		 * domain, and makes the wake-up timer above unable to ever fire ->
		 * permanent brick. Override with a pull-UP so PB7 stays HIGH and the
		 * latch stays CLOSED through SHUTDOWN; the RTC then cold-boots the chip
		 * at the deadline. (No effect on the wakeup_seconds==0 true-off path.) */
		HAL_PWREx_DisableGPIOPullDown(PWR_GPIO_B, PWR_GPIO_BIT_7);
		HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_B, PWR_GPIO_BIT_7);
#endif
	}

	/* Mirror what vMGR_LPM_enterShutdown() in mgr_lpm.c does: clear sticky
	 * wake-up flags so the rising edge on the wake source is detected
	 * cleanly during the sleep period. */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUFI);

	/* PWR_C2CR1.LPMS caps the SYSTEM mode (effective = shallowest of
	 * CR1/C2CR1) and survives every reset except POR. Seen polluted to
	 * Stop0 on the bench: "SHUTDOWN" degraded to a mode where IWDG kept
	 * counting (16 s reboot) and WKUP pins never fired. CPU2 never boots
	 * on this product — force "no floor" before entry. */
	MODIFY_REG(PWR->C2CR1, PWR_C2CR1_LPMS, PWR_LOWPOWERMODE_SHUTDOWN);

	HAL_PWREx_EnterSHUTDOWNMode();
	/* Never returns — chip cold-boots on next wake. */
#else
	/* SHUTDOWN not enabled in this build (LPM_SHUTDOWN_ENABLED off): fall
	 * back to NVIC reset so the caller doesn't return into undefined
	 * post-shutdown code paths. */
	(void)wakeup_seconds;
	NVIC_SystemReset();
#endif
	for (;;) { /* unreachable */ }
}
#pragma GCC visibility pop

/**
 * @}
 */
