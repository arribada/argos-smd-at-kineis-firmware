/**
 * @file    bl_flash.h
 * @brief   Flash memory operations for bootloader
 * @date    2025
 */

#ifndef BL_FLASH_H
#define BL_FLASH_H

#include <stdint.h>
#include <stdbool.h>
#include "bl_config.h"

/**
 * @brief Flash operation status codes
 */
typedef enum {
    BL_FLASH_OK = 0,
    BL_FLASH_ERROR,
    BL_FLASH_BUSY,
    BL_FLASH_TIMEOUT,
    BL_FLASH_ADDR_ERROR,
    BL_FLASH_SIZE_ERROR,
    BL_FLASH_PROTECTED
} bl_flash_status_t;

/**
 * @brief Initialize flash operations
 * @return true if successful
 */
bool bl_flash_init(void);

/**
 * @brief Unlock flash for write/erase operations
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_unlock(void);

/**
 * @brief Lock flash after write/erase operations
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_lock(void);

/**
 * @brief Erase a flash page
 * @param page_addr Address of page to erase (must be page-aligned)
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_erase_page(uint32_t page_addr);

/**
 * @brief Erase multiple flash pages
 * @param start_addr Start address (page-aligned)
 * @param num_pages Number of pages to erase
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_erase_pages(uint32_t start_addr, uint32_t num_pages);

/**
 * @brief Erase the application flash region
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_erase_app(void);

/**
 * @brief Write data to flash (64-bit aligned)
 * @param address Destination address (must be 64-bit aligned)
 * @param data Pointer to data buffer
 * @param length Data length in bytes (must be multiple of 8)
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_write(uint32_t address, const void* data, uint32_t length);

/**
 * @brief Write a single 64-bit doubleword to flash
 * @param address Destination address (must be 64-bit aligned)
 * @param data 64-bit value to write
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_write_doubleword(uint32_t address, uint64_t data);

/**
 * @brief Read data from flash
 * @param address Source address
 * @param buffer Destination buffer
 * @param length Length to read in bytes
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_read(uint32_t address, void* buffer, uint32_t length);

/**
 * @brief Verify written data matches source
 * @param address Flash address
 * @param data Expected data
 * @param length Length in bytes
 * @return true if data matches
 */
bool bl_flash_verify(uint32_t address, const void* data, uint32_t length);

/**
 * @brief Check if address is within application region
 * @param address Address to check
 * @return true if address is in application region
 */
bool bl_flash_addr_in_app(uint32_t address);

/**
 * @brief Check if address is within bootloader region
 * @param address Address to check
 * @return true if address is in bootloader region
 */
bool bl_flash_addr_in_bootloader(uint32_t address);

/**
 * @brief Get page number from address
 * @param address Flash address
 * @return Page number
 */
uint32_t bl_flash_get_page(uint32_t address);

/**
 * @brief Read bootloader state from flash
 * @param state Pointer to state structure to fill
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_read_bl_state(bl_state_flash_t* state);

/**
 * @brief Write bootloader state to flash
 * @param state Pointer to state structure
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_write_bl_state(const bl_state_flash_t* state);

/**
 * @brief Set DFU request flag in bootloader state
 * @param request true to request DFU mode, false to clear
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_set_dfu_request(bool request);

/**
 * @brief Check if DFU mode was requested
 * @return true if DFU was requested
 */
bool bl_flash_is_dfu_requested(void);

/**
 * @brief Clear DFU request flag
 * @return BL_FLASH_OK if successful
 */
bl_flash_status_t bl_flash_clear_dfu_request(void);

#endif /* BL_FLASH_H */
