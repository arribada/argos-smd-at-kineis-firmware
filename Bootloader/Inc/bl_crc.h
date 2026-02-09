/**
 * @file    bl_crc.h
 * @brief   CRC-32/MPEG-2 implementation for firmware integrity verification
 * @date    2025
 *
 * @defgroup BL_CRC CRC Calculation
 * @ingroup  BOOTLOADER
 * @brief    CRC-32/MPEG-2 for DFU firmware integrity checks
 *
 * Software CRC-32 implementation using MPEG-2 parameters:
 * - Polynomial: 0x04C11DB7
 * - Initial value: 0xFFFFFFFF
 * - No final XOR
 *
 * Supports both single-shot and accumulative (chunked) calculation.
 * Used during DFU WRITE to accumulate CRC, then compared in VERIFY command.
 * @{
 */

#ifndef BL_CRC_H
#define BL_CRC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the CRC hardware peripheral
 * @return true if initialization successful, false otherwise
 */
bool bl_crc_init(void);

/**
 * @brief Deinitialize the CRC hardware peripheral
 */
void bl_crc_deinit(void);

/**
 * @brief Reset the CRC calculation (set to initial value)
 */
void bl_crc_reset(void);

/**
 * @brief Calculate CRC32 of a data buffer
 * @param data Pointer to data buffer (must be 32-bit aligned for best performance)
 * @param length_bytes Length of data in bytes
 * @return Calculated CRC32 value
 */
uint32_t bl_crc32_calculate(const void* data, uint32_t length_bytes);

/**
 * @brief Accumulate CRC32 calculation (for large data processed in chunks)
 * @param data Pointer to data buffer
 * @param length_bytes Length of data in bytes
 * @return Current accumulated CRC32 value
 */
uint32_t bl_crc32_accumulate(const void* data, uint32_t length_bytes);

/**
 * @brief Get the current accumulated CRC32 value
 * @return Current CRC32 value
 */
uint32_t bl_crc32_get(void);

/**
 * @brief Calculate CRC32 of flash memory region
 * @param start_addr Start address in flash
 * @param length_bytes Length of data in bytes
 * @return Calculated CRC32 value
 */
uint32_t bl_crc32_flash(uint32_t start_addr, uint32_t length_bytes);

/** @} */ /* end of BL_CRC group */

#endif /* BL_CRC_H */
