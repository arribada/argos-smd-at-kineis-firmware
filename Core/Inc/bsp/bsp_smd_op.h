/**
 * @file    bsp_smd_op.h
 * @brief   BSP pin definitions for SMD_OP board (placeholder, same as SMD_PA for now)
 */

#ifndef BSP_SMD_OP_H
#define BSP_SMD_OP_H

/* ---- Board capabilities ---- */
#define BSP_BOARD_NAME          "SMD_OP"
#define BSP_HAS_EXTERNAL_PA     1

/* ---- Debug pins ---- */
#define JTMS_SWCLK_Pin          GPIO_PIN_14
#define JTMS_SWCLK_GPIO_Port    GPIOA
#define JTMS_SWDIO_Pin          GPIO_PIN_13
#define JTMS_SWDIO_GPIO_Port    GPIOA

/* ---- External PA control ---- */
#define PA_PSU_EN_Pin           GPIO_PIN_0
#define PA_PSU_EN_GPIO_Port     GPIOC
#define PA_PSU_SEL_Pin          GPIO_PIN_1
#define PA_PSU_SEL_GPIO_Port    GPIOC

/* ---- Wakeup button ---- */
#define EXT_WKUP_BUTTON_Pin         GPIO_PIN_3
#define EXT_WKUP_BUTTON_GPIO_Port   GPIOB

#endif /* BSP_SMD_OP_H */
