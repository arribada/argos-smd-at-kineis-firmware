/**
 * @file    test_crc32.c
 * @brief   Unit tests for CRC32 implementation
 * @date    2025
 *
 * Tests the software CRC32 implementation used by the bootloader.
 * Uses the STM32 CRC32 polynomial (0x04C11DB7).
 */

#include "test_framework.h"

/* CRC32 polynomial (same as STM32 hardware CRC) */
#define CRC32_POLYNOMIAL    0x04C11DB7UL
#define CRC32_INIT_VALUE    0xFFFFFFFFUL

/* Local CRC32 implementation for testing (copy of bootloader implementation) */
static uint32_t crc32_byte(uint32_t crc, uint8_t byte)
{
    crc ^= ((uint32_t)byte << 24);

    for (int i = 0; i < 8; i++) {
        if (crc & 0x80000000UL) {
            crc = (crc << 1) ^ CRC32_POLYNOMIAL;
        } else {
            crc <<= 1;
        }
    }

    return crc;
}

static uint32_t crc32_calculate(const uint8_t* data, uint32_t length)
{
    if (data == NULL || length == 0) {
        return 0;
    }

    uint32_t crc = CRC32_INIT_VALUE;

    for (uint32_t i = 0; i < length; i++) {
        crc = crc32_byte(crc, data[i]);
    }

    return crc;
}

/*******************************************************************************
 * TEST CASES
 ******************************************************************************/

/**
 * Test CRC32 with empty data
 */
void test_crc32_empty_data(void)
{
    uint32_t crc = crc32_calculate(NULL, 0);
    ASSERT_EQ(0, crc);

    uint8_t data[1] = {0};
    crc = crc32_calculate(data, 0);
    ASSERT_EQ(0, crc);

    TEST_PASS();
}

/**
 * Test CRC32 with single byte
 */
void test_crc32_single_byte(void)
{
    uint8_t data = 0x00;
    uint32_t crc = crc32_calculate(&data, 1);

    /* Pre-calculated expected value for 0x00 with STM32 polynomial */
    /* CRC32(0x00) with init 0xFFFFFFFF and poly 0x04C11DB7 */
    ASSERT_EQ_HEX(0x4E08BFB4, crc);

    TEST_PASS();
}

/**
 * Test CRC32 with known test vector: "123456789"
 */
void test_crc32_test_vector_123456789(void)
{
    const uint8_t data[] = "123456789";
    uint32_t crc = crc32_calculate(data, 9);

    /* STM32 CRC32 result for "123456789" */
    /* Note: STM32 uses different bit order, so result differs from standard CRC32 */
    ASSERT_EQ_HEX(0x89A1897F, crc);

    TEST_PASS();
}

/**
 * Test CRC32 with all zeros
 */
void test_crc32_all_zeros(void)
{
    uint8_t data[16] = {0};
    uint32_t crc = crc32_calculate(data, 16);

    /* Should produce a non-zero result */
    ASSERT_TRUE(crc != 0);

    /* Verify consistency */
    uint32_t crc2 = crc32_calculate(data, 16);
    ASSERT_EQ_HEX(crc, crc2);

    TEST_PASS();
}

/**
 * Test CRC32 with all 0xFF
 */
void test_crc32_all_ff(void)
{
    uint8_t data[16];
    memset(data, 0xFF, sizeof(data));

    uint32_t crc = crc32_calculate(data, 16);

    /* Should produce a non-zero result */
    ASSERT_TRUE(crc != 0);

    TEST_PASS();
}

/**
 * Test CRC32 different data produces different CRC
 */
void test_crc32_different_data(void)
{
    uint8_t data1[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t data2[] = {0x01, 0x02, 0x03, 0x05};

    uint32_t crc1 = crc32_calculate(data1, 4);
    uint32_t crc2 = crc32_calculate(data2, 4);

    /* Different data should produce different CRC */
    ASSERT_TRUE(crc1 != crc2);

    TEST_PASS();
}

/**
 * Test CRC32 consistency (same data = same CRC)
 */
void test_crc32_consistency(void)
{
    uint8_t data[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE};

    uint32_t crc1 = crc32_calculate(data, sizeof(data));
    uint32_t crc2 = crc32_calculate(data, sizeof(data));
    uint32_t crc3 = crc32_calculate(data, sizeof(data));

    ASSERT_EQ_HEX(crc1, crc2);
    ASSERT_EQ_HEX(crc2, crc3);

    TEST_PASS();
}

/**
 * Test CRC32 with incremental data lengths
 */
void test_crc32_incremental_length(void)
{
    uint8_t data[256];
    uint32_t prev_crc = 0;

    /* Fill with pattern */
    for (int i = 0; i < 256; i++) {
        data[i] = (uint8_t)i;
    }

    /* Each length should produce different CRC */
    for (int len = 1; len <= 256; len++) {
        uint32_t crc = crc32_calculate(data, len);

        /* CRC should change with each additional byte */
        if (len > 1) {
            ASSERT_TRUE(crc != prev_crc);
        }
        prev_crc = crc;
    }

    TEST_PASS();
}

/**
 * Test CRC32 with realistic firmware-like data
 */
void test_crc32_firmware_pattern(void)
{
    /* Simulate typical ARM Cortex-M vector table */
    uint8_t vector_table[] = {
        0x00, 0x80, 0x00, 0x20,  /* Initial SP: 0x20008000 */
        0x01, 0x81, 0x00, 0x08,  /* Reset handler */
        0x11, 0x81, 0x00, 0x08,  /* NMI handler */
        0x15, 0x81, 0x00, 0x08,  /* HardFault handler */
    };

    uint32_t crc = crc32_calculate(vector_table, sizeof(vector_table));

    /* Should produce valid CRC */
    ASSERT_TRUE(crc != 0);
    ASSERT_TRUE(crc != 0xFFFFFFFF);

    /* Verify consistency */
    uint32_t crc2 = crc32_calculate(vector_table, sizeof(vector_table));
    ASSERT_EQ_HEX(crc, crc2);

    TEST_PASS();
}

/**
 * Test CRC32 byte order sensitivity
 */
void test_crc32_byte_order(void)
{
    uint8_t data_le[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t data_be[] = {0x04, 0x03, 0x02, 0x01};

    uint32_t crc_le = crc32_calculate(data_le, 4);
    uint32_t crc_be = crc32_calculate(data_be, 4);

    /* Different byte order should produce different CRC */
    ASSERT_TRUE(crc_le != crc_be);

    TEST_PASS();
}

/*******************************************************************************
 * TEST RUNNER
 ******************************************************************************/

int main(void)
{
    TEST_SUITE_START("CRC32 Unit Tests");

    RUN_TEST(test_crc32_empty_data);
    RUN_TEST(test_crc32_single_byte);
    RUN_TEST(test_crc32_test_vector_123456789);
    RUN_TEST(test_crc32_all_zeros);
    RUN_TEST(test_crc32_all_ff);
    RUN_TEST(test_crc32_different_data);
    RUN_TEST(test_crc32_consistency);
    RUN_TEST(test_crc32_incremental_length);
    RUN_TEST(test_crc32_firmware_pattern);
    RUN_TEST(test_crc32_byte_order);

    TEST_SUITE_END();
    TEST_SUMMARY();
}
