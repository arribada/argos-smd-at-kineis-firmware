/**
 * @file    app_header.c
 * @brief   Application header for bootloader validation
 * @date    2025
 *
 * This structure is placed at 0x08000200 (after ISR vector table) and contains
 * metadata about the application for the bootloader to validate before jumping.
 *
 * Note: The CRC fields are filled by the create_dfu.py post-processing script.
 */

#include <stdint.h>

/*******************************************************************************
 * CONFIGURATION - Must match bootloader bl_config.h
 ******************************************************************************/
#define APP_HEADER_MAGIC        0x4B494E45UL    /* "KINE" in ASCII */
#define APP_HEADER_VERSION      0x0001
#define APP_HEADER_SIZE         256
#define APP_FLASH_BASE          0x08000000UL    /* ISR vector at start of flash */

/* Application version - update for each release */
#define APP_VERSION_MAJOR       1
#define APP_VERSION_MINOR       0
#define APP_VERSION_PATCH       0
#define APP_VERSION             ((APP_VERSION_MAJOR << 16) | (APP_VERSION_MINOR << 8) | APP_VERSION_PATCH)

/* Minimum bootloader version required */
#define MIN_BL_VERSION          0x00010000UL    /* 1.0.0 */

/* Hardware compatibility flags */
#define HW_COMPAT_STM32WL55     0x00000001
#define HW_COMPAT_KINEIS_V10    0x00000002
#define HW_COMPAT_SPI           0x00000020

/*******************************************************************************
 * APPLICATION HEADER STRUCTURE
 ******************************************************************************/

typedef struct __attribute__((packed)) {
    /* Identification block (16 bytes) */
    uint32_t magic;
    uint16_t header_version;
    uint16_t header_size;
    uint32_t app_version;
    uint32_t reserved1;

    /* Flash layout block (16 bytes) */
    uint32_t app_start_addr;
    uint32_t app_size;           /* Filled by create_dfu.py */
    uint32_t app_entry_point;    /* Filled by create_dfu.py */
    uint32_t vector_table_addr;

    /* Integrity block (16 bytes) */
    uint32_t app_crc32;          /* Filled by create_dfu.py */
    uint32_t header_crc32;       /* Filled by create_dfu.py */
    uint64_t reserved2;

    /* Build info block (32 bytes) */
    char build_date[16];
    char git_commit[16];

    /* Compatibility block (16 bytes) */
    uint32_t min_bl_version;
    uint32_t hw_compatibility;
    uint64_t reserved3;

    /* Security block (16 bytes) */
    uint8_t signature[16];

    /* Padding to reach 256 bytes */
    uint8_t reserved[144];
} app_header_t;

/*******************************************************************************
 * APPLICATION HEADER INSTANCE
 * Placed in .app_header section at 0x08000200 by linker script
 ******************************************************************************/

__attribute__((section(".app_header"), used))
const app_header_t application_header = {
    /* Identification block */
    .magic = APP_HEADER_MAGIC,
    .header_version = APP_HEADER_VERSION,
    .header_size = APP_HEADER_SIZE,
    .app_version = APP_VERSION,
    .reserved1 = 0,

    /* Flash layout block - some values filled by create_dfu.py */
    .app_start_addr = APP_FLASH_BASE,
    .app_size = 0xFFFFFFFF,         /* Placeholder - filled by create_dfu.py */
    .app_entry_point = 0xFFFFFFFF,  /* Placeholder - filled by create_dfu.py */
    .vector_table_addr = APP_FLASH_BASE,

    /* Integrity block - filled by create_dfu.py */
    .app_crc32 = 0xFFFFFFFF,        /* Placeholder */
    .header_crc32 = 0xFFFFFFFF,     /* Placeholder */
    .reserved2 = 0,

    /* Build info - will be overwritten by create_dfu.py with actual values */
    .build_date = __DATE__,  /* Max 16 chars, __DATE__ is 11 chars */
    .git_commit = "unknown",

    /* Compatibility block */
    .min_bl_version = MIN_BL_VERSION,
    .hw_compatibility = HW_COMPAT_STM32WL55 | HW_COMPAT_KINEIS_V10 | HW_COMPAT_SPI,
    .reserved3 = 0,

    /* Security block - reserved for future use */
    .signature = {0},

    /* Padding */
    .reserved = {0}
};
