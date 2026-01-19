/**
 * @file    test_app_header.c
 * @brief   Unit tests for application header validation
 * @date    2025
 *
 * Tests the application header structure and validation logic.
 */

#include "test_framework.h"

/*******************************************************************************
 * APP HEADER DEFINITIONS
 ******************************************************************************/

#define APP_HEADER_MAGIC        0x4B494E45UL    /* "KINE" */
#define APP_HEADER_VERSION      0x0001
#define APP_HEADER_SIZE         256
#define APP_FLASH_BASE          0x08008100UL
#define APP_MAX_SIZE            0x32F00UL       /* ~183 KB */
#define BL_VERSION              0x00010000UL    /* 1.0.0 */

typedef struct __attribute__((packed)) {
    /* Identification block (16 bytes) */
    uint32_t magic;
    uint16_t header_version;
    uint16_t header_size;
    uint32_t app_version;
    uint32_t reserved1;

    /* Flash layout block (16 bytes) */
    uint32_t app_start_addr;
    uint32_t app_size;
    uint32_t app_entry_point;
    uint32_t vector_table_addr;

    /* Integrity block (16 bytes) */
    uint32_t app_crc32;
    uint32_t header_crc32;
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

    /* Padding */
    uint8_t reserved[144];
} app_header_t;

/*******************************************************************************
 * VALIDATION FUNCTIONS (simplified for testing)
 ******************************************************************************/

static bool validate_header_magic(const app_header_t* header)
{
    return (header != NULL) && (header->magic == APP_HEADER_MAGIC);
}

static bool validate_header_version(const app_header_t* header)
{
    return (header != NULL) && (header->header_version <= APP_HEADER_VERSION);
}

static bool validate_header_size(const app_header_t* header)
{
    return (header != NULL) && (header->header_size == APP_HEADER_SIZE);
}

static bool validate_app_address(const app_header_t* header)
{
    return (header != NULL) && (header->app_start_addr == APP_FLASH_BASE);
}

static bool validate_app_size(const app_header_t* header)
{
    return (header != NULL) &&
           (header->app_size > 0) &&
           (header->app_size <= APP_MAX_SIZE);
}

static bool validate_bl_compatibility(const app_header_t* header)
{
    return (header != NULL) && (header->min_bl_version <= BL_VERSION);
}

static bool validate_header_complete(const app_header_t* header)
{
    return validate_header_magic(header) &&
           validate_header_version(header) &&
           validate_header_size(header) &&
           validate_app_address(header) &&
           validate_app_size(header) &&
           validate_bl_compatibility(header);
}

/*******************************************************************************
 * HELPER FUNCTIONS
 ******************************************************************************/

static void create_valid_header(app_header_t* header)
{
    memset(header, 0, sizeof(app_header_t));
    header->magic = APP_HEADER_MAGIC;
    header->header_version = APP_HEADER_VERSION;
    header->header_size = APP_HEADER_SIZE;
    header->app_version = 0x00010000;  /* 1.0.0 */
    header->app_start_addr = APP_FLASH_BASE;
    header->app_size = 0x10000;  /* 64KB */
    header->app_entry_point = APP_FLASH_BASE + 1;
    header->vector_table_addr = APP_FLASH_BASE;
    header->min_bl_version = 0x00010000;  /* 1.0.0 */
}

/*******************************************************************************
 * TEST CASES - STRUCTURE SIZE
 ******************************************************************************/

void test_header_size(void)
{
    ASSERT_EQ(256, sizeof(app_header_t));
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - MAGIC VALIDATION
 ******************************************************************************/

void test_magic_valid(void)
{
    app_header_t header;
    create_valid_header(&header);
    ASSERT_TRUE(validate_header_magic(&header));
    TEST_PASS();
}

void test_magic_invalid(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.magic = 0xDEADBEEF;
    ASSERT_FALSE(validate_header_magic(&header));
    TEST_PASS();
}

void test_magic_zero(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.magic = 0x00000000;
    ASSERT_FALSE(validate_header_magic(&header));
    TEST_PASS();
}

void test_magic_erased_flash(void)
{
    app_header_t header;
    memset(&header, 0xFF, sizeof(header));
    ASSERT_FALSE(validate_header_magic(&header));
    TEST_PASS();
}

void test_magic_null_header(void)
{
    ASSERT_FALSE(validate_header_magic(NULL));
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - VERSION VALIDATION
 ******************************************************************************/

void test_version_valid(void)
{
    app_header_t header;
    create_valid_header(&header);
    ASSERT_TRUE(validate_header_version(&header));
    TEST_PASS();
}

void test_version_older(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.header_version = 0x0000;  /* Older version */
    ASSERT_TRUE(validate_header_version(&header));
    TEST_PASS();
}

void test_version_newer(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.header_version = 0x0002;  /* Newer version */
    ASSERT_FALSE(validate_header_version(&header));
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - SIZE VALIDATION
 ******************************************************************************/

void test_header_size_valid(void)
{
    app_header_t header;
    create_valid_header(&header);
    ASSERT_TRUE(validate_header_size(&header));
    TEST_PASS();
}

void test_header_size_invalid(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.header_size = 128;
    ASSERT_FALSE(validate_header_size(&header));
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - ADDRESS VALIDATION
 ******************************************************************************/

void test_app_address_valid(void)
{
    app_header_t header;
    create_valid_header(&header);
    ASSERT_TRUE(validate_app_address(&header));
    TEST_PASS();
}

void test_app_address_wrong(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.app_start_addr = 0x08004100;  /* Old address */
    ASSERT_FALSE(validate_app_address(&header));
    TEST_PASS();
}

void test_app_address_bootloader(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.app_start_addr = 0x08000000;  /* Bootloader region */
    ASSERT_FALSE(validate_app_address(&header));
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - APP SIZE VALIDATION
 ******************************************************************************/

void test_app_size_valid(void)
{
    app_header_t header;
    create_valid_header(&header);
    ASSERT_TRUE(validate_app_size(&header));
    TEST_PASS();
}

void test_app_size_zero(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.app_size = 0;
    ASSERT_FALSE(validate_app_size(&header));
    TEST_PASS();
}

void test_app_size_too_large(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.app_size = APP_MAX_SIZE + 1;
    ASSERT_FALSE(validate_app_size(&header));
    TEST_PASS();
}

void test_app_size_max(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.app_size = APP_MAX_SIZE;
    ASSERT_TRUE(validate_app_size(&header));
    TEST_PASS();
}

void test_app_size_min(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.app_size = 1;
    ASSERT_TRUE(validate_app_size(&header));
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - BOOTLOADER COMPATIBILITY
 ******************************************************************************/

void test_bl_compat_equal(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.min_bl_version = BL_VERSION;
    ASSERT_TRUE(validate_bl_compatibility(&header));
    TEST_PASS();
}

void test_bl_compat_older(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.min_bl_version = 0x00000100;  /* 0.1.0 */
    ASSERT_TRUE(validate_bl_compatibility(&header));
    TEST_PASS();
}

void test_bl_compat_newer_required(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.min_bl_version = 0x00020000;  /* 2.0.0 */
    ASSERT_FALSE(validate_bl_compatibility(&header));
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - COMPLETE VALIDATION
 ******************************************************************************/

void test_complete_validation_pass(void)
{
    app_header_t header;
    create_valid_header(&header);
    ASSERT_TRUE(validate_header_complete(&header));
    TEST_PASS();
}

void test_complete_validation_bad_magic(void)
{
    app_header_t header;
    create_valid_header(&header);
    header.magic = 0x00000000;
    ASSERT_FALSE(validate_header_complete(&header));
    TEST_PASS();
}

void test_complete_validation_erased_flash(void)
{
    app_header_t header;
    memset(&header, 0xFF, sizeof(header));
    ASSERT_FALSE(validate_header_complete(&header));
    TEST_PASS();
}

void test_complete_validation_zeroed(void)
{
    app_header_t header;
    memset(&header, 0x00, sizeof(header));
    ASSERT_FALSE(validate_header_complete(&header));
    TEST_PASS();
}

/*******************************************************************************
 * TEST RUNNER
 ******************************************************************************/

int main(void)
{
    TEST_SUITE_START("App Header Unit Tests");

    printf("\n-- Structure Size --\n");
    RUN_TEST(test_header_size);

    printf("\n-- Magic Validation --\n");
    RUN_TEST(test_magic_valid);
    RUN_TEST(test_magic_invalid);
    RUN_TEST(test_magic_zero);
    RUN_TEST(test_magic_erased_flash);
    RUN_TEST(test_magic_null_header);

    printf("\n-- Version Validation --\n");
    RUN_TEST(test_version_valid);
    RUN_TEST(test_version_older);
    RUN_TEST(test_version_newer);

    printf("\n-- Header Size Validation --\n");
    RUN_TEST(test_header_size_valid);
    RUN_TEST(test_header_size_invalid);

    printf("\n-- Address Validation --\n");
    RUN_TEST(test_app_address_valid);
    RUN_TEST(test_app_address_wrong);
    RUN_TEST(test_app_address_bootloader);

    printf("\n-- App Size Validation --\n");
    RUN_TEST(test_app_size_valid);
    RUN_TEST(test_app_size_zero);
    RUN_TEST(test_app_size_too_large);
    RUN_TEST(test_app_size_max);
    RUN_TEST(test_app_size_min);

    printf("\n-- Bootloader Compatibility --\n");
    RUN_TEST(test_bl_compat_equal);
    RUN_TEST(test_bl_compat_older);
    RUN_TEST(test_bl_compat_newer_required);

    printf("\n-- Complete Validation --\n");
    RUN_TEST(test_complete_validation_pass);
    RUN_TEST(test_complete_validation_bad_magic);
    RUN_TEST(test_complete_validation_erased_flash);
    RUN_TEST(test_complete_validation_zeroed);

    TEST_SUITE_END();
    TEST_SUMMARY();
}
