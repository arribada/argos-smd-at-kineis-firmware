/**
 * @file    bsp_smd_nopa.h
 * @brief   BSP pin definitions for SMD_NOPA board (no external PA)
 */

#ifndef BSP_SMD_NOPA_H
#define BSP_SMD_NOPA_H

/* ---- Board capabilities ---- */
#define BSP_BOARD_NAME          "SMD_NOPA"
#define BSP_HAS_EXTERNAL_PA     0
/* Krd v11.1.0 features — gated by AT handlers, GUI probes at connect */
#define BSP_HAS_DOWNLINK        0
#define BSP_HAS_GNSS            0
#define BSP_HAS_SATDET          0

/* ---- Debug pins ---- */
#define JTMS_SWCLK_Pin          GPIO_PIN_14
#define JTMS_SWCLK_GPIO_Port    GPIOA
#define JTMS_SWDIO_Pin          GPIO_PIN_13
#define JTMS_SWDIO_GPIO_Port    GPIOA

/* ---- PA control pins (defined but not used - no external PA) ---- */
#define PA_PSU_EN_Pin           GPIO_PIN_0
#define PA_PSU_EN_GPIO_Port     GPIOC
#define PA_PSU_SEL_Pin          GPIO_PIN_1
#define PA_PSU_SEL_GPIO_Port    GPIOC

/* ---- Wakeup button ---- */
#define EXT_WKUP_BUTTON_Pin         GPIO_PIN_3
#define EXT_WKUP_BUTTON_GPIO_Port   GPIOB

/* ---- MCU_DONE (TPL5111, uncomment if wired) ---- */
// #define MCU_DONE_Pin            GPIO_PIN_9
// #define MCU_DONE_GPIO_Port      GPIOB

#endif /* BSP_SMD_NOPA_H */
