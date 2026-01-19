/**
 * @file    bl_app_header.h
 * @brief   Application header structure for bootloader validation
 * @date    2025
 */

#ifndef BL_APP_HEADER_H
#define BL_APP_HEADER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "bl_config.h"

/*******************************************************************************
 * APPLICATION HEADER STRUCTURE
 *
 * This structure is placed at APP_HEADER_ADDR (0x08004000) and contains
 * metadata about the application firmware for validation by the bootloader.
 *
 * Total size: 256 bytes (64-bit aligned)
 ******************************************************************************/

typedef struct __attribute__((packed)) {
    /* Identification block (16 bytes) */
    uint32_t magic;              /**< Magic number: APP_HEADER_MAGIC (0x4B494E45 = "KINE") */
    uint16_t header_version;     /**< Header format version */
    uint16_t header_size;        /**< Size of this header (256 bytes) */
    uint32_t app_version;        /**< Application version (major.minor.patch packed) */
    uint32_t reserved1;          /**< Reserved for future use */

    /* Flash layout block (16 bytes) */
    uint32_t app_start_addr;     /**< Start address of application code (0x08004100) */
    uint32_t app_size;           /**< Size of application in bytes */
    uint32_t app_entry_point;    /**< Entry point address (Reset_Handler) */
    uint32_t vector_table_addr;  /**< Vector table address */

    /* Integrity block (16 bytes) */
    uint32_t app_crc32;          /**< CRC32 of application code */
    uint32_t header_crc32;       /**< CRC32 of header (excluding this field and app_crc32) */
    uint64_t reserved2;          /**< Reserved */

    /* Build info block (32 bytes) */
    char build_date[16];         /**< Build date "YYYY-MM-DD HH:MM" */
    char git_commit[16];         /**< Git commit hash (truncated) */

    /* Compatibility block (16 bytes) */
    uint32_t min_bl_version;     /**< Minimum bootloader version required */
    uint32_t hw_compatibility;   /**< Hardware compatibility flags */
    uint64_t reserved3;          /**< Reserved */

    /* Security block (16 bytes) - Reserved for future use */
    uint8_t signature[16];       /**< Reserved for digital signature */

    /* Padding to reach 256 bytes */
    uint8_t reserved[144];       /**< Padding */

} app_header_t;

/* Verify structure size at compile time */
_Static_assert(sizeof(app_header_t) == 256, "app_header_t must be 256 bytes");

/*******************************************************************************
 * VERSION MACROS
 ******************************************************************************/

/**
 * @brief Pack version numbers into a single uint32_t
 * Format: 0x00MMNNPP (Major.Minor.Patch)
 */
#define APP_VERSION_PACK(major, minor, patch) \
    (((uint32_t)(major) << 16) | ((uint32_t)(minor) << 8) | (uint32_t)(patch))

/**
 * @brief Extract version components from packed version
 */
#define APP_VERSION_MAJOR(v)    (((v) >> 16) & 0xFF)
#define APP_VERSION_MINOR(v)    (((v) >> 8) & 0xFF)
#define APP_VERSION_PATCH(v)    ((v) & 0xFF)

/*******************************************************************************
 * HARDWARE COMPATIBILITY FLAGS
 ******************************************************************************/
#define HW_COMPAT_STM32WL55     0x00000001  /**< STM32WL55 compatible */
#define HW_COMPAT_KINEIS_V10    0x00000002  /**< Kineis stack V10 compatible */
#define HW_COMPAT_UART          0x00000010  /**< UART communication supported */
#define HW_COMPAT_SPI           0x00000020  /**< SPI communication supported */

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/

/**
 * @brief Get pointer to application header in flash
 * @return Pointer to app_header_t structure
 */
static inline const app_header_t* bl_get_app_header(void) {
    return (const app_header_t*)APP_HEADER_ADDR;
}

/**
 * @brief Check if application header has valid magic number
 * @param header Pointer to application header
 * @return true if magic is valid, false otherwise
 */
static inline bool bl_header_magic_valid(const app_header_t* header) {
    return (header != NULL) && (header->magic == APP_HEADER_MAGIC);
}

/**
 * @brief Check if application header version is compatible
 * @param header Pointer to application header
 * @return true if version is compatible, false otherwise
 */
static inline bool bl_header_version_valid(const app_header_t* header) {
    return (header != NULL) && (header->header_version <= APP_HEADER_VERSION);
}

/**
 * @brief Check if application is compatible with this bootloader
 * @param header Pointer to application header
 * @return true if compatible, false otherwise
 */
static inline bool bl_app_compatible(const app_header_t* header) {
    if (header == NULL) return false;
    return (header->min_bl_version <= BL_VERSION);
}

/**
 * @brief Validate application header (magic, version, addresses)
 * @param header Pointer to application header
 * @return true if header is valid, false otherwise
 */
bool bl_validate_app_header(const app_header_t* header);

/**
 * @brief Validate application CRC32
 * @param header Pointer to application header
 * @return true if CRC matches, false otherwise
 */
bool bl_validate_app_crc(const app_header_t* header);

/**
 * @brief Full application validation (header + CRC)
 * @return true if application is valid, false otherwise
 */
bool bl_app_is_valid(void);

#endif /* BL_APP_HEADER_H */
