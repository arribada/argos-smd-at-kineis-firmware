/* SPDX-License-Identifier: no SPDX license */
/**
 * @file    lpm.h
 * @brief   This file contains some utilities and functions needed for a correct handling of LPM
 *          on the STM32WL55xx cortex M4 core
 *
 *          It is a layer overlapping to the generic LPM manager (MRG_LPM):
 *          * MGRM_LPM is a skeleton working on most of the STM32 uC
 *          * LPM is adding specifities as per the current platform, i.e. STM32WL55xx uC
 *
 *          This code is provided as an example by Kineis and is free to modifications as per
 *          integrator's/application's needs
 *
 * @author  Kineis
 */

/**
 * @page lpm_page LPM
 *
 * This page is presenting the low-power-managment code specific to the Kineis reference design
 * example based on STM32WL55xx uC (\ref MGR_LPM).
 *
 * It is in charge to configure the generic low power manager (\ref mgr_lpm_page) to have its LPM
 * (\ref MgrLpm_LPM_t) working on STM32WL55xx uC.
 *
 * It is also in charge to register the Kineis stack client so that the LPM manager knows which LPM
 * to enter at any time of the Kineis protocol (idle phase, TX preparation, TX active, TX done).
 *
 * lpm.c file contains overlaying functions to the mgr_lpm.c.
 *
 * lpm_cli_kstk.c contains how to deduce the allowed LPM (\ref MgrLpm_LPM_t) from Kineis stack
 * ressources status (KNS_MAC_rsrcStatus_t).
 *
 * @section lpm_subpages Sub-pages
 *
 * * @subpage mgr_lpm_page
 */

/**
 * @addtogroup MGR_LPM
 * @brief low power mode specifics to STM32WL55xx, overlay to generic low power manager (refer to
 * @ref lpm_page page for details).
 * @{
 */

#ifndef LPM_H
#define LPM_H

/* Includes ------------------------------------------------------------------------------------ */
#include <stdbool.h>
#include "mgr_lpm.h"

/* Enums --------------------------------------------------------------------------------------- */

extern struct MgrLpm_EnvConfig_t lpm_config; /** @todo can be removed if IDLE task wo/ BAREMETAL OS */

/* Functions ----------------------------------------------------------------------------------- */

/** @brief Specific system function to configure some clock behaviours during LPM (e.g. UART) */
void LPM_SystemClockConfig(void);

/**
 * @brief Disable all GPIO as not used for the moment
 *
 * @todo Use this fct for LPM optimization (especially when LPM standby)
 *
 * This function disables GPIOs by:
 * * clocking all ports
 * * Configure all GPIO port pins in Analog Input mode (floating input trigger OFF)
 * * unclocking all ports
 *
 * It is done in a way to save some current drain while GPIOs are not needed.
 *
 * So far, GPIOs are mainly connected to SX1255 transceiver. Thus it is needed when powering ON,
 * TX, RX, SAT detection and so on. transceiver driver will need to set GPIOs to correct state
 * when needed.
 *
 * \note This GPIO could be disabled back at end of RF any action (RX/TX/SAT_DETECT). So far,
 *       it will only go back disabled when entering LPM shutdown mode.
 *
 * \note GPIOs used by peripherals, such as SPI, SWD.JTAG, LPTIM are not disabled here.
 *
 * \todo Disable SWD GPIOs (used for ST-Link debugging) in release build. Note, this will:
 *       * disable SWD attach/debug on a running build
 *       * but it remains possible to reflash by pressing RESET pin/button of the uC at the start
 *       of flashing procedure
 */
void GPIO_DisableAllToAnalogInput(void);

/**
 * @brief Initializes the LPM management and register clients such as the Kineis stack client
 */
void LPM_init(void);

/**
 * @brief Try to enter low power mode depending on clients capabilities
 *
 * This function will invoke the main entry point of the low power manager and backup set mode.
 * Backing up the low power mode will be used at wake, because the wake upo sequence depends on
 * what remained active during LPM
 *
 * @return MgrLpm_LowPowerMode_t low power mode
 */
void LPM_enter(void);

/**
 * @brief Force the current low power mode manually
 *
 * param[in] MgrLpm_LPM_t low power mode
 */
void LPM_forceMode(enum MgrLpm_LPM_t low_power_mode);

/**
 * @brief Get the current low power mode
 *
 * @return MgrLpm_LowPowerMode_t low power mode
 */
enum MgrLpm_LPM_t LPM_getMode(void);

/**
 * @brief Override the LPM mode requested by clients on next LPM_enter
 *
 * When set to a value other than LOW_POWER_MODE_NONE, the Kineis stack LPM
 * client returns this mode instead of computing it from rsrcStatus. The chosen
 * mode is still masked by lpm_config.allowedLPMbitmap in MGR_LPM_enter, so the
 * mode must also be enabled in the bitmap to actually be entered.
 *
 * @param[in] mode forced mode (LOW_POWER_MODE_NONE clears the override)
 */
void LPM_setForcedMode(enum MgrLpm_LPM_t mode);

/**
 * @brief Get the current forced mode override
 *
 * @return Currently forced mode, or LOW_POWER_MODE_NONE if no override active
 */
enum MgrLpm_LPM_t LPM_getForcedMode(void);

#endif /* LPM_KSTK_H */

/**
 * @}
 */
