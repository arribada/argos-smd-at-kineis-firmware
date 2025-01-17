/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/**
 * @page main_page Entry point: main
 *
 * As the kineis stack is meant to be integrated in an OS-like environment supporting LPM, the
 * \ref main function is basically:
 * * Initializing the HW clocks and peripherals according to LPM wakeup sequence
 * * Registering all tasks:
 *     * APP: \ref kns_app_page
 *     * MAC: \ref kns_mac_page,
 *            [libkineis.a doc link](../../../../../Doc/libkineis/html/index.html),
 *            [libknsrf_wl.a doc link](../../../../../Doc/libknsrf_wl/html/index.html)
 *     * IDLE: \ref idle_task_page
 * * Creating all the queues used for communication between tasks
 * * Starting the OS then \ref kns_os_page
 *
 * @section kns_main_subpages Sub-pages
 *
 * * @subpage kns_app_page
 * * @subpage kns_mac_page
 * * @subpage idle_task_page
 * * @subpage kns_os_page
 */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "rtc.h"
#include "subghz.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/** Uncomment below if you want to use Kineis standalone APP or GUI APP.
 * GUI APP is about AT commands support sent to device from GUI or SERIAL hyperterminal).
 * @note both flags are exclusive as standalone app will send periodic frame indefinitively
 */
#if !defined(USE_STDALONE_APP) && !defined(USE_GUI_APP)
//#define USE_STDALONE_APP
#define USE_GUI_APP
#endif

#if defined(USE_STDALONE_APP) && defined(USE_GUI_APP)
#error "USE_STDALONE_APP/USE_GUI_APP: Cannot build FW with both APPs, select only one."
#endif

#if defined(USE_GUI_APP) && (defined(LPM_STANDBY_ENABLED) || defined (LPM_SHUTDOWN_ENABLED))
#warning "GUI_APP compiled with STANDBY/SHUTDOWN LPM: Ensure a wakeup pin (PB3, PC13) is available to exit LPM before sending a new AT command to the device."
#endif

#include "stm32wlxx_it.h"
#include "build_info.h"
#include "kns_q.h"
#include "kns_os.h"
#include "mcu_tim.h"
#include "kns_mac.h"
#include "kns_app.h"
#ifdef USE_GUI_APP
#ifdef USE_BAREMETAL
#include "mgr_at_cmd.h" /* Needed for MGR_AT_CMD_isPendingAt()) in case of BAREMETAL OS to check no
                         * there is no pending AT cmd before entring low power mode
                         */
#endif
#include "mcu_at_console.h"
#endif
#include "lpm.h"
#include "mgr_log.h"

/** Assembly function used to initialize SRAM2 .bss and .data sections. It is based on the same
 * model as the Reset_Handler (cf startup_*.s file) regarding the whole RAM memory
 *
 * This function is called only when waking up from SHUTDOWN, RESET, POWER OFF mode. In any other
 * low power mode, the SRAM2 remains active with current FW.
 *
 * @note By default, the autogenerated code from STM32CubeMx does not handle this.
 */
extern int Sram2_Init();

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

__attribute((__weak__))
const char libkineis_info[] = "N/A";
__attribute((__weak__))
const char libknsrf_wl_info[] = "N/A";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief IDLE task to be registered in OS
 *
 * @page idle_task_page IDLE task
 *
 * This task is called at last position when there is absolutely nothing else to do in other tasks.
 * Thus, its main purpose is to go into low power mode.
 *  
 * @note When Standby LPM is supported and GUI application is runnin, a 10s delay is added before
 * going to STANDBY in a way to let user to enter a new AT cmd from UART if wanted.
 *
 * @note In case of Kineis baremetal OS, recheck all queues are empty before LPM, under critical
 * section:
 * * STANDBY/SHUTDOWN: can disable all interrupts, uC will re-enable it when entering LPM.
 * * SLEEP/STOP: disable all interrupts here, but need to restore it at the very last moment just
 * before entering WFI
 *
 * @section idle_task_subpages Sub-pages
 *
 * @subpage lpm_page
 *
 */
static void IDLE_task(void)
{

  /* ---- KINEIS GUI APP ------------------------------------------------------------------------ */

#ifdef USE_GUI_APP
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = EXT_WKUP_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(EXT_WKUP_BUTTON_GPIO_Port, &GPIO_InitStruct);

#ifdef USE_BAREMETAL
  uint32_t prim;

  if (lpm_config.allowedLPMbitmap & (LOW_POWER_MODE_STANDBY | LOW_POWER_MODE_SHUTDOWN))
  {
    /** wait wakeup pin to turn low (unplug debugger or by user) */
    while(HAL_GPIO_ReadPin(EXT_WKUP_BUTTON_GPIO_Port, EXT_WKUP_BUTTON_Pin) == GPIO_PIN_SET) {

	  //MGR_LOG_DEBUG("==== WAKEUP BUTTON SET ====\r\n");
      if (KNS_Q_isEvtInSomeQ() || MGR_AT_CMD_isPendingAt())
        return;
    }
    /** Do the last check on event out of for loop. */
    HAL_Delay(500);
  }

  /** Disable interrupt for last occurence to avoid any interrup to be skipped */
  prim = __get_PRIMASK();
  __disable_irq();
  __disable_fault_irq();
  if (KNS_Q_isEvtInSomeQ() || MGR_AT_CMD_isPendingAt()) {
    if (!prim){
      __enable_fault_irq();
      __enable_irq();
    }
    return;
  }

  /** Enter low power mode has there is no event preempting */
  LPM_enter();
  if (!prim){
    __enable_fault_irq();
    __enable_irq();
  }
#else // end of USE_BAREMETAL
  if (lpm_config.allowedLPMbitmap & (LOW_POWER_MODE_STANDBY | LOW_POWER_MODE_SHUTDOWN))
    while(HAL_GPIO_ReadPin(EXT_WKUP_BUTTON_GPIO_Port, EXT_WKUP_BUTTON_Pin) == GPIO_PIN_SET);

  /** Enter low power mode has there is no event preempting */
  LPM_enter();
#endif
#endif

  /* ---- KINEIS STDLN APP ---------------------------------------------------------------------- */

#ifdef USE_STDALONE_APP
#ifdef USE_BAREMETAL
  uint32_t prim;

  prim = __get_PRIMASK();
  __disable_irq();
  __disable_fault_irq();
  if (KNS_Q_isEvtInSomeQ()) {
    if (!prim){
      __enable_fault_irq();
      __enable_irq();
    }
    return;
  }
  /** Enter low power mode has there is no event preempting */
  LPM_enter();
  if (!prim){
    __enable_fault_irq();
    __enable_irq();
  }
#else // end of USE_BAREMETAL
  /** Enter low power mode has there is no event preempting */
  LPM_enter();
#endif
#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  bool bIsWakeUpFromReset = false;

  /** Check if reset was triggered by nRST external pin
   *
   * @note This pin is also set when POWERing ON on the device or when debugger is plugged in
   * */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) == SET) {
    __HAL_RCC_CLEAR_RESET_FLAGS();
    bIsWakeUpFromReset = true;
  }
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /** LPM_getMode and LPM_forceMode uses the TAMP/RTC backup registers (BKPRx), cf
   * lpm_ctxt.low_power_mode variable mapped in __section__(".lpmSection").
   *
   * For a proper usage of LPM_get/force_mode functions after reset:
   * * unprotected the backup domain from unwanted write accesses (HAL_PWR_EnableBkUpAccess() called
   * in SystemClock_Config() above
   * * enable its clock through __HAL_RCC_RTCAPB_CLK_ENABLE() below
   */
  __HAL_RCC_RTCAPB_CLK_ENABLE();

//  if (bIsWakeUpFromReset)
//    LPM_forceMode(LOW_POWER_MODE_NONE);

#ifndef LPM_SHUTDOWN_ENABLED
  if (LPM_getMode() > LOW_POWER_MODE_STANDBY) {
    /** @todo remove assert for final-user build, can keep it during dev/integration */
    assert_param(0);
    /** recovery for production build: try wakeup from power OFF leading to reinit everything */
    LPM_forceMode(LOW_POWER_MODE_NONE);
  }
#else
  if (LPM_getMode() > LOW_POWER_MODE_SHUTDOWN) {
    /** @todo remove assert for final-user build, can keep it during dev/integration */
    assert_param(0);
    /** recovery for production build: try wakeup from power OFF leading to reinit everything */
    LPM_forceMode(LOW_POWER_MODE_NONE);
  }
#endif

  /** Specific clock configuration regarding our LPM strategy (want to support all SLEEP, STOP,
   * STANDBY, SHUTDOWN modes on this uC */
  LPM_SystemClockConfig();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_SUBGHZ_Init();
  MX_TIM16_Init();
  MX_RTC_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

#ifdef DEBUG
  /** When core enters debug mode (core halted), freeze RTC */
  __HAL_DBGMCU_FREEZE_RTC();
#endif

  /** As we just woke up, most of GPIOs are useless so far. Limit their current drain */
  GPIO_DisableAllToAnalogInput();

  /** Do specific Init sequence as per wake up mode. The low power mode was set before entering.
   * Some of them (typically standby, shutdown) makes the uC to reset
   *
   * @note Regarding RTC, specific things are tuned directly in the MX_RTC_Init function (/ref rtc.c
   * file)
   */
  switch (LPM_getMode()) {
  case LOW_POWER_MODE_SHUTDOWN:
    /** Initialize retention RAM2 as not done by default in the Reset_Handler */
    Sram2_Init();
    break;
  case LOW_POWER_MODE_STANDBY:
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    break;
  case LOW_POWER_MODE_STOP:
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOP);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOP2);
    break;
  case LOW_POWER_MODE_SLEEP:
    break;
  default: /** Start from reset or power off */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOP);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_STOP2);
    /** Initialize retention RAM2 as not done by default in the Reset_Handler */
    Sram2_Init();
    /** Deinit all unused timers including RTC timer to save current drain (was automatically
     * enabled by STM32Cube generated code earlier
     */
    MCU_TIM_deinit(MCU_TIM_HDLR_TX_TIMEOUT);
    MCU_TIM_deinit(MCU_TIM_HDLR_TX_PERIOD);
    break;
  }

  /** Logging purpose only, mention which LPM exited */
  switch (LPM_getMode()) {
  case LOW_POWER_MODE_SHUTDOWN:
    MGR_LOG_DEBUG("==== WAKEUP from SHUTDOWN ====\r\n");
    break;
  case LOW_POWER_MODE_STANDBY:
    MGR_LOG_DEBUG("==== WAKEUP from STANDBY ====\r\n");
    break;
  case LOW_POWER_MODE_STOP:
    MGR_LOG_DEBUG("==== WAKEUP from STOP MODE ====\r\n");
    break;
  case LOW_POWER_MODE_SLEEP:
    MGR_LOG_DEBUG("==== WAKEUP from SLEEP ====\r\n");
    break;
  default:
    if (bIsWakeUpFromReset)
      MGR_LOG_DEBUG("==== WAKEUP from RESET ====\r\n");
    else
      MGR_LOG_DEBUG("==== WAKEUP from POWER OFF ====\r\n");
    MGR_LOG_DEBUG("Running build done at %s %s, versions:\r\n", __DATE__, __TIME__);
    MGR_LOG_DEBUG("- FW            %s\r\n", uc_fw_vers_commit_id);
    MGR_LOG_DEBUG("- libkineis.a   %s\r\n", libkineis_info);
    MGR_LOG_DEBUG("- libknsrf_wl.a %s\r\n", libknsrf_wl_info);
    MGR_LOG_DEBUG("\r\n");
    break;
  }

  /** LPM managment: Initialize and register Kineis stack client */
  LPM_init();

  /** ---------------------------------------------------------------------------------------------
   * ---- KINEIS STACK MANDATORY RESSOURCES ---- START---------------------------------------------
   * ----------------------------------------------------------------------------------------------
   */
  /** Create OS queues and register task needed by Kineis stack */
  assert_param(KNS_Q_create(KNS_Q_DL_APP2MAC, KNS_Q_DL_APP2MAC_LEN, KNS_Q_DL_APP2MAC_ITEM_BYTESIZE) == KNS_STATUS_OK);
  assert_param(KNS_Q_create(KNS_Q_UL_MAC2APP, KNS_Q_UL_MAC2APP_LEN, KNS_Q_UL_MAC2APP_ITEM_BYTESIZE) == KNS_STATUS_OK);
  assert_param(KNS_Q_create(KNS_Q_UL_INFRA2MAC, KNS_Q_UL_INFRA2MAC_LEN, KNS_Q_UL_INFRA2MAC_ITEM_BYTESIZE) == KNS_STATUS_OK);
  assert_param(KNS_Q_create(KNS_Q_UL_SRVC2MAC, KNS_Q_UL_SRVC2MAC_LEN, KNS_Q_UL_SRVC2MAC_ITEM_BYTESIZE) == KNS_STATUS_OK);
  assert_param(KNS_OS_registerTask(KNS_OS_TASK_MAC, KNS_MAC_task) == KNS_STATUS_OK);
  /** ---------------------------------------------------------------------------------------------
   * ---- KINEIS STACK MANDATORY RESSOURCES ---- END  ---------------------------------------------
   * ----------------------------------------------------------------------------------------------
   */

  /** Register APPlication task and IDLE task
   *
   * @note The Idle task is required to call the low power mode managment only
   * */
#if defined(USE_STDALONE_APP)
  KNS_APP_stdln_init(NULL);
  assert_param(KNS_OS_registerTask(KNS_OS_TASK_APP, KNS_APP_stdln_loop) == KNS_STATUS_OK);
  //assert_param(KNS_OS_registerTask(KNS_OS_TASK_APP, KNS_APP_stdalone_stressTest) == KNS_STATUS_OK);
#elif defined (USE_GUI_APP)
  KNS_APP_gui_init(&hlpuart1);
  assert_param(KNS_OS_registerTask(KNS_OS_TASK_APP, KNS_APP_gui_loop) == KNS_STATUS_OK);
#endif
  assert_param(KNS_OS_registerTask(KNS_OS_TASK_IDLE, IDLE_task) == KNS_STATUS_OK);

  /** As all tasks, queues and OS are ready to start, last LPM-wakeup-specific init sequence */
  switch (LPM_getMode()) {
  case LOW_POWER_MODE_SHUTDOWN:
  case LOW_POWER_MODE_STANDBY:
    /** Simulate potential RTC interrupts, as we can exit shutdown/standby mode from:
     * - RTC alarm
     * - RTC wakeuptimer
     * The handlers check RTC registers in a way to identify RTC interrupt if any.
     * In case shutdown exit is due to the wakeup pin (not RTC), no functional processing will be
     * by those RTC handlers
     */
//    RTC_Alarm_IRQHandler();
    RTC_WKUP_IRQHandler();
    break;
  case LOW_POWER_MODE_STOP:
  case LOW_POWER_MODE_SLEEP:
    break;
  default:
  /** Start from RESET or POWER OFF, log few indications (FW version, Kineis MAc protocol info) */
#if defined (USE_GUI_APP)
    MCU_AT_CONSOLE_send("+FW=%s,%s_%s\r\n", uc_fw_vers_commit_id, __DATE__, __TIME__);
#endif
    break;
  }

  /** End of init sequence, set LPM as runtime mode before starting all tasks */
  LPM_forceMode(LOW_POWER_MODE_NONE);

  /** Start all tasks */
  KNS_OS_main();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Error_Handler();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  MGR_LOG_DEBUG("Error_Handler\r\n");
#ifdef DEBUG
#ifdef USE_GUI_APP
  MCU_AT_CONSOLE_send("+ASSERT=\r\n");
  HAL_Delay(500);
#endif
  __disable_irq();
  while (1)
  {
  }
#else // end of DEBUG
#ifdef USE_GUI_APP
  MCU_AT_CONSOLE_send("+RST=\r\n");
  HAL_Delay(500);
#endif
  __disable_irq();
  /* reset uC */
  NVIC_SystemReset();
#endif /* #ifdef DEBUG */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  MGR_LOG_DEBUG("ASSERT FAIL: %lu %s\r\n", line, file);
  Error_Handler();
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
