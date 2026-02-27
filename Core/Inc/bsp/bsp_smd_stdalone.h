/**
 * @file    bsp_smd_stdalone.h
 * @brief   BSP pin definitions for SMD_STDALONE board (standalone underwater tracker)
 *
 * Features: External PA, LED RGB, Reed switch, Power latch, VBAT ADC, SWS
 * Comm: UART only
 */

#ifndef BSP_SMD_STDALONE_H
#define BSP_SMD_STDALONE_H

/* ---- Board capabilities ---- */
#define BSP_BOARD_NAME          "SMD_STDALONE"
#define BSP_HAS_EXTERNAL_PA     1
#define BSP_HAS_LED_RGB         1
#define BSP_HAS_REED_SWITCH     1
#define BSP_HAS_PWR_LATCH       1
#define BSP_HAS_VBAT_ADC        1

/* ---- Debug pins ---- */
#define JTMS_SWCLK_Pin          GPIO_PIN_14
#define JTMS_SWCLK_GPIO_Port    GPIOA
#define JTMS_SWDIO_Pin          GPIO_PIN_13
#define JTMS_SWDIO_GPIO_Port    GPIOA
#define SWO_Pin                 GPIO_PIN_3
#define SWO_GPIO_Port           GPIOB

/* ---- External PA control ---- */
#define PA_PSU_EN_Pin           GPIO_PIN_0   /* VPA_EN */
#define PA_PSU_EN_GPIO_Port     GPIOC
#define PA_PSU_SEL_Pin          GPIO_PIN_1   /* VSEL (always HIGH for 3V3) */
#define PA_PSU_SEL_GPIO_Port    GPIOC

/* ---- Salt Water Switch ---- */
#define SWS_OUT_Pin             GPIO_PIN_12
#define SWS_OUT_GPIO_Port       GPIOA
#define SWS_IN_Pin              GPIO_PIN_11  /* ADC_IN7 */
#define SWS_IN_GPIO_Port        GPIOA

/* ---- I2C (reserved, not used) ---- */
#define I2C_SCL_Pin             GPIO_PIN_9
#define I2C_SCL_GPIO_Port       GPIOA
#define I2C_SDA_Pin             GPIO_PIN_10
#define I2C_SDA_GPIO_Port       GPIOA

/* ---- Battery monitoring ---- */
#define VBAT_EN_Pin             GPIO_PIN_9
#define VBAT_EN_GPIO_Port       GPIOB
#define VBAT_ADC_Pin            GPIO_PIN_13
#define VBAT_ADC_GPIO_Port      GPIOB

/* ---- LED RGB (active LOW: GPIO_PIN_RESET = on) ---- */
#define LED_RED_Pin             GPIO_PIN_1
#define LED_RED_GPIO_Port       GPIOA
#define LED_GREEN_Pin           GPIO_PIN_4
#define LED_GREEN_GPIO_Port     GPIOB
#define LED_BLUE_Pin            GPIO_PIN_5
#define LED_BLUE_GPIO_Port      GPIOB

/* ---- Reed switch (active HIGH: HIGH = magnet present) ---- */
#define REED_MCU_Pin            GPIO_PIN_6
#define REED_MCU_GPIO_Port      GPIOB

/* ---- Power latch (HIGH = keep board powered) ---- */
#define PWR_LATCH_Pin           GPIO_PIN_7
#define PWR_LATCH_GPIO_Port     GPIOB

/* ---- MCU_DONE (TPL5111, not implemented) ---- */
#define MCU_DONE_Pin            GPIO_PIN_15
#define MCU_DONE_GPIO_Port      GPIOA

/* ---- Wakeup button: alias to REED for compatibility ---- */
#define EXT_WKUP_BUTTON_Pin         REED_MCU_Pin
#define EXT_WKUP_BUTTON_GPIO_Port   REED_MCU_GPIO_Port

#endif /* BSP_SMD_STDALONE_H */
