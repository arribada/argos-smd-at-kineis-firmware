/**
  ******************************************************************************
  * @file      startup_bl.s
  * @brief     STM32WL55xx Bootloader startup file for GCC toolchain.
  *            Minimal startup for bootloader (16KB max)
  ******************************************************************************
  */

.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler

/* Linker script symbols */
.word _sidata
.word _sdata
.word _edata
.word _sbss
.word _ebss

/**
 * @brief  Reset handler - bootloader entry point
 */
  .section .text.Reset_Handler
  .weak Reset_Handler
  .type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Call the clock system initialization function */
  bl  SystemInit

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Skip __libc_init_array - not needed for bare-metal bootloader
 * and can cause issues without proper heap/syscalls setup */

/* Call the bootloader main */
  bl main

LoopForever:
    b LoopForever

  .size Reset_Handler, .-Reset_Handler

/**
 * @brief  Default interrupt handler
 */
  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

/******************************************************************************
* Vector table - minimal set for bootloader
******************************************************************************/
  .section .isr_vector,"a",%progbits
  .type g_pfnVectors, %object
  .size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  /* External Interrupts */
  .word WWDG_IRQHandler                   /* 0  */
  .word PVD_PVM_IRQHandler                /* 1  */
  .word TAMP_STAMP_LSECSS_SSRU_IRQHandler /* 2  */
  .word RTC_WKUP_IRQHandler               /* 3  */
  .word FLASH_IRQHandler                  /* 4  */
  .word RCC_IRQHandler                    /* 5  */
  .word EXTI0_IRQHandler                  /* 6  */
  .word EXTI1_IRQHandler                  /* 7  */
  .word EXTI2_IRQHandler                  /* 8  */
  .word EXTI3_IRQHandler                  /* 9  */
  .word EXTI4_IRQHandler                  /* 10 */
  .word DMA1_Channel1_IRQHandler          /* 11 */
  .word DMA1_Channel2_IRQHandler          /* 12 */
  .word DMA1_Channel3_IRQHandler          /* 13 */
  .word DMA1_Channel4_IRQHandler          /* 14 */
  .word DMA1_Channel5_IRQHandler          /* 15 */
  .word DMA1_Channel6_IRQHandler          /* 16 */
  .word DMA1_Channel7_IRQHandler          /* 17 */
  .word ADC_IRQHandler                    /* 18 */
  .word DAC_IRQHandler                    /* 19 */
  .word C2SEV_PWR_C2H_IRQHandler          /* 20 */
  .word COMP_IRQHandler                   /* 21 */
  .word EXTI9_5_IRQHandler                /* 22 */
  .word TIM1_BRK_IRQHandler               /* 23 */
  .word TIM1_UP_IRQHandler                /* 24 */
  .word TIM1_TRG_COM_IRQHandler           /* 25 */
  .word TIM1_CC_IRQHandler                /* 26 */
  .word TIM2_IRQHandler                   /* 27 */
  .word TIM16_IRQHandler                  /* 28 */
  .word TIM17_IRQHandler                  /* 29 */
  .word I2C1_EV_IRQHandler                /* 30 */
  .word I2C1_ER_IRQHandler                /* 31 */
  .word I2C2_EV_IRQHandler                /* 32 */
  .word I2C2_ER_IRQHandler                /* 33 */
  .word SPI1_IRQHandler                   /* 34 */
  .word SPI2_IRQHandler                   /* 35 */
  .word USART1_IRQHandler                 /* 36 */
  .word USART2_IRQHandler                 /* 37 */
  .word LPUART1_IRQHandler                /* 38 */
  .word LPTIM1_IRQHandler                 /* 39 */
  .word LPTIM2_IRQHandler                 /* 40 */
  .word EXTI15_10_IRQHandler              /* 41 */
  .word RTC_Alarm_IRQHandler              /* 42 */
  .word LPTIM3_IRQHandler                 /* 43 */
  .word SUBGHZSPI_IRQHandler              /* 44 */
  .word IPCC_C1_RX_IRQHandler             /* 45 */
  .word IPCC_C1_TX_IRQHandler             /* 46 */
  .word HSEM_IRQHandler                   /* 47 */
  .word I2C3_EV_IRQHandler                /* 48 */
  .word I2C3_ER_IRQHandler                /* 49 */
  .word SUBGHZ_Radio_IRQHandler           /* 50 */
  .word AES_IRQHandler                    /* 51 */
  .word RNG_IRQHandler                    /* 52 */
  .word PKA_IRQHandler                    /* 53 */
  .word DMA2_Channel1_IRQHandler          /* 54 */
  .word DMA2_Channel2_IRQHandler          /* 55 */
  .word DMA2_Channel3_IRQHandler          /* 56 */
  .word DMA2_Channel4_IRQHandler          /* 57 */
  .word DMA2_Channel5_IRQHandler          /* 58 */
  .word DMA2_Channel6_IRQHandler          /* 59 */
  .word DMA2_Channel7_IRQHandler          /* 60 */
  .word DMAMUX1_OVR_IRQHandler            /* 61 */

/*******************************************************************************
* Weak aliases for exception handlers
*******************************************************************************/
  .weak NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler

  .weak BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler

  .weak UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler

  .weak SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  .weak WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,Default_Handler

  .weak PVD_PVM_IRQHandler
  .thumb_set PVD_PVM_IRQHandler,Default_Handler

  .weak TAMP_STAMP_LSECSS_SSRU_IRQHandler
  .thumb_set TAMP_STAMP_LSECSS_SSRU_IRQHandler,Default_Handler

  .weak RTC_WKUP_IRQHandler
  .thumb_set RTC_WKUP_IRQHandler,Default_Handler

  .weak FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler,Default_Handler

  .weak RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler,Default_Handler

  .weak EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler,Default_Handler

  .weak EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler,Default_Handler

  .weak EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler,Default_Handler

  .weak EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler,Default_Handler

  .weak DMA1_Channel1_IRQHandler
  .thumb_set DMA1_Channel1_IRQHandler,Default_Handler

  .weak DMA1_Channel2_IRQHandler
  .thumb_set DMA1_Channel2_IRQHandler,Default_Handler

  .weak DMA1_Channel3_IRQHandler
  .thumb_set DMA1_Channel3_IRQHandler,Default_Handler

  .weak DMA1_Channel4_IRQHandler
  .thumb_set DMA1_Channel4_IRQHandler,Default_Handler

  .weak DMA1_Channel5_IRQHandler
  .thumb_set DMA1_Channel5_IRQHandler,Default_Handler

  .weak DMA1_Channel6_IRQHandler
  .thumb_set DMA1_Channel6_IRQHandler,Default_Handler

  .weak DMA1_Channel7_IRQHandler
  .thumb_set DMA1_Channel7_IRQHandler,Default_Handler

  .weak ADC_IRQHandler
  .thumb_set ADC_IRQHandler,Default_Handler

  .weak DAC_IRQHandler
  .thumb_set DAC_IRQHandler,Default_Handler

  .weak C2SEV_PWR_C2H_IRQHandler
  .thumb_set C2SEV_PWR_C2H_IRQHandler,Default_Handler

  .weak COMP_IRQHandler
  .thumb_set COMP_IRQHandler,Default_Handler

  .weak EXTI9_5_IRQHandler
  .thumb_set EXTI9_5_IRQHandler,Default_Handler

  .weak TIM1_BRK_IRQHandler
  .thumb_set TIM1_BRK_IRQHandler,Default_Handler

  .weak TIM1_UP_IRQHandler
  .thumb_set TIM1_UP_IRQHandler,Default_Handler

  .weak TIM1_TRG_COM_IRQHandler
  .thumb_set TIM1_TRG_COM_IRQHandler,Default_Handler

  .weak TIM1_CC_IRQHandler
  .thumb_set TIM1_CC_IRQHandler,Default_Handler

  .weak TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Default_Handler

  .weak TIM16_IRQHandler
  .thumb_set TIM16_IRQHandler,Default_Handler

  .weak TIM17_IRQHandler
  .thumb_set TIM17_IRQHandler,Default_Handler

  .weak I2C1_EV_IRQHandler
  .thumb_set I2C1_EV_IRQHandler,Default_Handler

  .weak I2C1_ER_IRQHandler
  .thumb_set I2C1_ER_IRQHandler,Default_Handler

  .weak I2C2_EV_IRQHandler
  .thumb_set I2C2_EV_IRQHandler,Default_Handler

  .weak I2C2_ER_IRQHandler
  .thumb_set I2C2_ER_IRQHandler,Default_Handler

  .weak SPI1_IRQHandler
  .thumb_set SPI1_IRQHandler,Default_Handler

  .weak SPI2_IRQHandler
  .thumb_set SPI2_IRQHandler,Default_Handler

  .weak USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak LPUART1_IRQHandler
  .thumb_set LPUART1_IRQHandler,Default_Handler

  .weak LPTIM1_IRQHandler
  .thumb_set LPTIM1_IRQHandler,Default_Handler

  .weak LPTIM2_IRQHandler
  .thumb_set LPTIM2_IRQHandler,Default_Handler

  .weak EXTI15_10_IRQHandler
  .thumb_set EXTI15_10_IRQHandler,Default_Handler

  .weak RTC_Alarm_IRQHandler
  .thumb_set RTC_Alarm_IRQHandler,Default_Handler

  .weak LPTIM3_IRQHandler
  .thumb_set LPTIM3_IRQHandler,Default_Handler

  .weak SUBGHZSPI_IRQHandler
  .thumb_set SUBGHZSPI_IRQHandler,Default_Handler

  .weak IPCC_C1_RX_IRQHandler
  .thumb_set IPCC_C1_RX_IRQHandler,Default_Handler

  .weak IPCC_C1_TX_IRQHandler
  .thumb_set IPCC_C1_TX_IRQHandler,Default_Handler

  .weak HSEM_IRQHandler
  .thumb_set HSEM_IRQHandler,Default_Handler

  .weak I2C3_EV_IRQHandler
  .thumb_set I2C3_EV_IRQHandler,Default_Handler

  .weak I2C3_ER_IRQHandler
  .thumb_set I2C3_ER_IRQHandler,Default_Handler

  .weak SUBGHZ_Radio_IRQHandler
  .thumb_set SUBGHZ_Radio_IRQHandler,Default_Handler

  .weak AES_IRQHandler
  .thumb_set AES_IRQHandler,Default_Handler

  .weak RNG_IRQHandler
  .thumb_set RNG_IRQHandler,Default_Handler

  .weak PKA_IRQHandler
  .thumb_set PKA_IRQHandler,Default_Handler

  .weak DMA2_Channel1_IRQHandler
  .thumb_set DMA2_Channel1_IRQHandler,Default_Handler

  .weak DMA2_Channel2_IRQHandler
  .thumb_set DMA2_Channel2_IRQHandler,Default_Handler

  .weak DMA2_Channel3_IRQHandler
  .thumb_set DMA2_Channel3_IRQHandler,Default_Handler

  .weak DMA2_Channel4_IRQHandler
  .thumb_set DMA2_Channel4_IRQHandler,Default_Handler

  .weak DMA2_Channel5_IRQHandler
  .thumb_set DMA2_Channel5_IRQHandler,Default_Handler

  .weak DMA2_Channel6_IRQHandler
  .thumb_set DMA2_Channel6_IRQHandler,Default_Handler

  .weak DMA2_Channel7_IRQHandler
  .thumb_set DMA2_Channel7_IRQHandler,Default_Handler

  .weak DMAMUX1_OVR_IRQHandler
  .thumb_set DMAMUX1_OVR_IRQHandler,Default_Handler

  .weak SystemInit
