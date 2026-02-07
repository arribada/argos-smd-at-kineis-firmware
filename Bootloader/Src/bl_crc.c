/**
 * @file    bl_crc.c
 * @brief   Software CRC32 implementation for bootloader
 * @date    2025
 *
 * Uses CRC-32/MPEG-2 (same as STM32 hardware CRC unit)
 * Polynomial: 0x04C11DB7 (MSB-first, non-reflected)
 * Initial value: 0xFFFFFFFF
 * No final XOR, no reflection
 *
 * This matches the Zephyr argos_dfu_crc32() implementation.
 */

#include "bl_crc.h"
#include <stddef.h>

/* CRC-32/MPEG-2 polynomial (MSB-first) */
#define CRC32_POLYNOMIAL    0x04C11DB7UL
#define CRC32_INIT_VALUE    0xFFFFFFFFUL

/* Current CRC state for accumulate operations */
static uint32_t crc_state = CRC32_INIT_VALUE;
static bool crc_initialized = false;

bool bl_crc_init(void)
{
    crc_state = CRC32_INIT_VALUE;
    crc_initialized = true;
    return true;
}

void bl_crc_deinit(void)
{
    crc_initialized = false;
}

void bl_crc_reset(void)
{
    crc_state = CRC32_INIT_VALUE;
}

/**
 * @brief Calculate CRC32 for a single byte (internal)
 * CRC-32/MPEG-2: MSB-first, no reflection
 */
static uint32_t crc32_byte(uint32_t crc, uint8_t byte)
{
    crc ^= (uint32_t)byte << 24;

    for (int i = 0; i < 8; i++) {
        if (crc & 0x80000000) {
            crc = (crc << 1) ^ CRC32_POLYNOMIAL;
        } else {
            crc <<= 1;
        }
    }

    return crc;
}

uint32_t bl_crc32_calculate(const void* data, uint32_t length_bytes)
{
    if (data == NULL || length_bytes == 0) {
        return 0;
    }

    uint32_t crc = CRC32_INIT_VALUE;
    const uint8_t* ptr = (const uint8_t*)data;

    for (uint32_t i = 0; i < length_bytes; i++) {
        crc = crc32_byte(crc, ptr[i]);
    }

    return crc;  /* No final XOR for MPEG-2 */
}

uint32_t bl_crc32_accumulate(const void* data, uint32_t length_bytes)
{
    if (data == NULL || length_bytes == 0) {
        return crc_state;
    }

    const uint8_t* ptr = (const uint8_t*)data;

    for (uint32_t i = 0; i < length_bytes; i++) {
        crc_state = crc32_byte(crc_state, ptr[i]);
    }

    return crc_state;  /* No final XOR for MPEG-2 */
}

uint32_t bl_crc32_get(void)
{
    return crc_state;  /* No final XOR for MPEG-2 */
}

uint32_t bl_crc32_flash(uint32_t start_addr, uint32_t length_bytes)
{
    if (length_bytes == 0) {
        return 0;
    }

    /* Validate address is in flash range */
    if (start_addr < 0x08000000UL || start_addr >= 0x08040000UL) {
        return 0;
    }

    /* Calculate CRC directly from flash memory */
    return bl_crc32_calculate((const void*)start_addr, length_bytes);
}
