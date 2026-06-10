/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32wlxx_it.c
  * @brief   Interrupt Service Routines.
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32wlxx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>  /* snprintf in HardFault_Handler */
#if defined(USE_UW_DOPPLER_APP) || defined(USE_DOPPLER_APP)
#include "mgr_err.h"
#include "mgr_log.h"
extern volatile uint32_t g_uw_doppler_state_for_err;
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  This function is executed in case of core and exception interrupts
  * @retval None
  */
static void Core_Error_Handler(void)
{
#if defined(USE_UW_DOPPLER_APP)
  /* Tracker must always reset — never stay stuck */
  __disable_irq();
  NVIC_SystemReset();
#elif defined(DEBUG)
  __disable_irq();
  while (1)
  {
  }
#else
  __disable_irq();
  NVIC_SystemReset();
#endif
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef hlpuart1;
extern RTC_HandleTypeDef hrtc;
#if defined(USE_SPI_DRIVER)
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
#endif
extern SUBGHZ_HandleTypeDef hsubghz;
extern TIM_HandleTypeDef htim16;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
    Core_Error_Handler();
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/* USER CODE BEGIN FaultHandlers */
/* Forensics-aware fault handlers.
 *
 * Each fault has a naked entry point that captures the stacked exception
 * frame address (SP) and dispatches to a regular C function. The C function
 * (a) prints UART forensics for live debugging, (b) saves the full crash
 * context (R0..R3, R12, LR_before, PC_before, xPSR, fault SCB regs) into
 * MGR_ERR's retention-RAM crash_info struct so the next boot can emit a
 * post-mortem trace, (c) triggers NVIC_SystemReset.
 *
 * The naked wrapper choice between MSP and PSP follows the EXC_RETURN bit
 * pattern in LR: bit 2 = 1 → PSP was active, 0 → MSP. Bare-metal builds
 * almost always use MSP, but the runtime check is cheap. */

#if defined(USE_UW_DOPPLER_APP) || defined(USE_DOPPLER_APP)

static void fault_handler_c(uint32_t *frame, uint8_t fault_type, const char *tag)
{
  /* CPU fault forensic — ERROR-grade. Gated at LOGLVL=NONE only so a
   * GUI parser sees a clean stream even if the device crashes during
   * its session. The frame is always persisted to TAMP/SRAM2 below,
   * so the next boot replay will surface it regardless. */
  if (MGR_LOG_passes(MGR_LOG_LVL_ERROR)) {
    extern UART_HandleTypeDef hlpuart1;
    static char buf[168];
    int n = snprintf(buf, sizeof(buf),
      "\r\n%s!!! %s !!!\r\nHFSR=%08lx CFSR=%08lx BFAR=%08lx MMFAR=%08lx\r\n"
      "PC=%08lx LR=%08lx R0=%08lx R12=%08lx XPSR=%08lx\r\n",
      MGR_LOG_levelTag(MGR_LOG_LVL_ERROR),
      tag,
      (unsigned long)SCB->HFSR, (unsigned long)SCB->CFSR,
      (unsigned long)SCB->BFAR, (unsigned long)SCB->MMFAR,
      (unsigned long)(frame ? frame[6] : 0),
      (unsigned long)(frame ? frame[5] : 0),
      (unsigned long)(frame ? frame[0] : 0),
      (unsigned long)(frame ? frame[4] : 0),
      (unsigned long)(frame ? frame[7] : 0));
    if (n > 0)
      HAL_UART_Transmit(&hlpuart1, (uint8_t *)buf, (uint16_t)n, 100);
  }

  MGR_ERR_captureFault(frame, fault_type, (uint8_t)g_uw_doppler_state_for_err);
  MGR_ERR_LOG_FAULT(fault_type, g_uw_doppler_state_for_err);
  NVIC_SystemReset();
  while (1) { /* unreachable */ }
}

void HardFault_C(uint32_t *frame)   { fault_handler_c(frame, ERR_HARDFAULT,  "HARDFAULT");  }
void MemManage_C(uint32_t *frame)   { fault_handler_c(frame, ERR_MEMMANAGE,  "MEMMANAGE");  }
void BusFault_C(uint32_t *frame)    { fault_handler_c(frame, ERR_BUSFAULT,   "BUSFAULT");   }
void UsageFault_C(uint32_t *frame)  { fault_handler_c(frame, ERR_USAGEFAULT, "USAGEFAULT"); }

#define NAKED_FAULT_TRAMPOLINE(name, c_fn)                                 \
  __attribute__((naked, used)) void name(void)                             \
  {                                                                        \
    __asm volatile (                                                       \
      "tst lr, #4 \n"                                                      \
      "ite eq     \n"                                                      \
      "mrseq r0, msp \n"                                                   \
      "mrsne r0, psp \n"                                                   \
      "b " #c_fn "\n"                                                      \
    );                                                                     \
  }

NAKED_FAULT_TRAMPOLINE(HardFault_Handler,  HardFault_C)
NAKED_FAULT_TRAMPOLINE(MemManage_Handler,  MemManage_C)
NAKED_FAULT_TRAMPOLINE(BusFault_Handler,   BusFault_C)
NAKED_FAULT_TRAMPOLINE(UsageFault_Handler, UsageFault_C)

#else  /* No forensics in GUI/STDLN builds — minimal handlers */

void HardFault_Handler(void)  { NVIC_SystemReset(); while (1) {} }
void MemManage_Handler(void)  { NVIC_SystemReset(); while (1) {} }
void BusFault_Handler(void)   { NVIC_SystemReset(); while (1) {} }
void UsageFault_Handler(void) { NVIC_SystemReset(); while (1) {} }

#endif
/* USER CODE END FaultHandlers */

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
#if defined(BSP_HAS_LED_RGB)
  /* Drive the composite-colour PWM (WHITE/VIOLET/CYAN/YELLOW). Cheap:
   * a single load, switch and one BSRR write per ms when active; no-op
   * the rest of the time. Required on SMD_STDALONE because the RGB LED
   * shares a single anode current-limit resistor and cannot light more
   * than one cathode simultaneously — see MGR_LED_softTick() comment. */
  extern void MGR_LED_softTick(void);
  MGR_LED_softTick();
#endif
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32WLxx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32wlxx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RTC Wakeup Interrupt.
  */
void RTC_WKUP_IRQHandler(void)
{
  /* USER CODE BEGIN RTC_WKUP_IRQn 0 */

  /* USER CODE END RTC_WKUP_IRQn 0 */
  HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
  /* USER CODE BEGIN RTC_WKUP_IRQn 1 */

  /* USER CODE END RTC_WKUP_IRQn 1 */
}

/**
  * @brief This function handles TIM16 Global Interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

#if defined(USE_SPI_DRIVER)
/* Debug counter for SPI interrupts */
volatile uint32_t spi_irq_count = 0;

/**
  * @brief This function handles DMA1 Channel 1 Interrupt (SPI1 RX).
  */
void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
}

/**
  * @brief This function handles DMA1 Channel 2 Interrupt (SPI1 TX).
  */
void DMA1_Channel2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi1_tx);
}

/**
  * @brief This function handles SPI1 Interrupt.
  */
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */
  spi_irq_count++;
  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */

  /* USER CODE END SPI1_IRQn 1 */
}
#endif
/**
  * @brief This function handles LPUART1 Interrupt.
  */
void LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN LPUART1_IRQn 0 */

  /* USER CODE END LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN LPUART1_IRQn 1 */

  /* USER CODE END LPUART1_IRQn 1 */
}

/**
  * @brief This function handles SUBGHZ Radio Interrupt.
  */
volatile uint32_t subghz_irq_count = 0;

void SUBGHZ_Radio_IRQHandler(void)
{
  /* USER CODE BEGIN SUBGHZ_Radio_IRQn 0 */
  subghz_irq_count++;
  /* USER CODE END SUBGHZ_Radio_IRQn 0 */
  HAL_SUBGHZ_IRQHandler(&hsubghz);
  /* USER CODE BEGIN SUBGHZ_Radio_IRQn 1 */

  /* USER CODE END SUBGHZ_Radio_IRQn 1 */
}

/* USER CODE BEGIN 1 */

#if defined(USE_SPI_DRIVER)
/**
 * @brief EXTI15_10 interrupt handler (SPI NSS wakeup from STOP mode)
 *
 * PA15 (NSS) is configured as EXTI falling edge before entering STOP.
 * This handler only clears the flag — the actual SPI re-init happens
 * in LPM_stop_exit(). The first SPI transaction that triggered wakeup
 * is lost; the host must retry.
 */
void EXTI15_10_IRQHandler(void)
{
	if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15) != RESET) {
		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_15);
	}
}
#endif

/* USER CODE END 1 */
