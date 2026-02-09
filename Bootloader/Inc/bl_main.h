/**
 * @file    bl_main.h
 * @brief   Main bootloader header - initialization, state machine, and jump-to-app
 * @date    2025
 *
 * @defgroup BL_MAIN Bootloader Main
 * @ingroup  BOOTLOADER
 * @brief    Bootloader entry point, state machine, and application jump
 *
 * Handles bootloader initialization, protocol detection, DFU state machine,
 * and validated jump to the application at @ref APP_FLASH_BASE.
 * @{
 */

#ifndef BL_MAIN_H
#define BL_MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "bl_config.h"
#include "bl_app_header.h"

/*******************************************************************************
 * DEBUG OUTPUT
 ******************************************************************************/

/**
 * @brief Low-level debug print via LPUART1 (available before HAL init)
 * @param str Null-terminated string to print
 */
void early_debug_print(const char *str);

/** @brief Bootloader debug print - disabled when BL_DEBUG is not defined */
#ifdef BL_DEBUG
#define BL_DBG(str) early_debug_print(str)
#else
#define BL_DBG(str) ((void)0)
#endif

/**
 * @brief Initialize the bootloader
 */
void bl_init(void);

/**
 * @brief Run the bootloader main loop
 */
void bl_run(void);

/**
 * @brief Get current bootloader state
 * @return Current state
 */
bl_state_t bl_get_state(void);

/**
 * @brief Set bootloader state
 * @param state New state
 */
void bl_set_state(bl_state_t state);

/**
 * @brief Get detected protocol
 * @return Detected protocol type
 */
bl_protocol_t bl_get_protocol(void);

/**
 * @brief Jump to application
 * @note This function does not return if successful
 */
void bl_jump_to_app(void);

/**
 * @brief Check if application is valid (header + CRC)
 * @return true if application is valid
 */
bool bl_check_app_valid(void);

/**
 * @brief Perform system reset
 */
void bl_system_reset(void);

/**
 * @brief Get bootloader version
 * @return Version as packed uint32_t
 */
uint32_t bl_get_version(void);

/**
 * @brief Get bootloader version string
 * @return Version string (e.g., "BL_V1.0.0")
 */
const char* bl_get_version_string(void);

/** @} */ /* end of BL_MAIN group */

#endif /* BL_MAIN_H */
