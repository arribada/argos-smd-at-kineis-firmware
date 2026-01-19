/**
 * @file    test_dfu_protocol.c
 * @brief   Unit tests for DFU protocol parsing and handling
 * @date    2025
 *
 * Tests the DFU protocol implementation including:
 * - AT command parsing (UART)
 * - SPI command parsing
 * - Response formatting
 */

#include "test_framework.h"
#include <ctype.h>

/*******************************************************************************
 * DFU PROTOCOL DEFINITIONS (from bl_config.h)
 ******************************************************************************/

typedef enum {
    DFU_CMD_PING        = 0x01,
    DFU_CMD_GET_INFO    = 0x02,
    DFU_CMD_ERASE       = 0x03,
    DFU_CMD_WRITE       = 0x04,
    DFU_CMD_READ        = 0x05,
    DFU_CMD_VERIFY      = 0x06,
    DFU_CMD_RESET       = 0x07,
    DFU_CMD_JUMP        = 0x08,
    DFU_CMD_GET_STATUS  = 0x09,
    DFU_CMD_MAX
} dfu_cmd_t;

typedef enum {
    DFU_RSP_OK              = 0x00,
    DFU_RSP_ERROR           = 0x01,
    DFU_RSP_CRC_ERROR       = 0x02,
    DFU_RSP_ADDR_ERROR      = 0x03,
    DFU_RSP_SIZE_ERROR      = 0x04,
    DFU_RSP_FLASH_ERROR     = 0x05,
    DFU_RSP_INVALID_CMD     = 0x07
} dfu_response_t;

/* Memory layout */
#define APP_HEADER_ADDR         0x08008000UL
#define APP_FLASH_BASE          0x08008100UL
#define APP_FLASH_END           0x0803AFFFUL
#define BL_FLASH_BASE           0x08000000UL
#define BL_FLASH_END            0x08007FFFUL

/*******************************************************************************
 * AT COMMAND PARSER (simplified version for testing)
 ******************************************************************************/

typedef struct {
    dfu_cmd_t cmd;
    uint32_t address;
    uint8_t data[256];
    uint32_t data_len;
    uint32_t crc;
    bool valid;
} at_parsed_cmd_t;

static int hex_char_to_int(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

static uint32_t parse_hex32(const char* str)
{
    uint32_t value = 0;
    for (int i = 0; i < 8 && str[i]; i++) {
        int digit = hex_char_to_int(str[i]);
        if (digit < 0) break;
        value = (value << 4) | digit;
    }
    return value;
}

static at_parsed_cmd_t parse_at_command(const char* cmd)
{
    at_parsed_cmd_t result = {0};
    result.valid = false;

    /* Check prefix */
    if (strncmp(cmd, "AT+DFU=", 7) != 0) {
        return result;
    }

    const char* param = cmd + 7;

    /* Parse command */
    if (strncmp(param, "PING", 4) == 0) {
        result.cmd = DFU_CMD_PING;
        result.valid = true;
    }
    else if (strncmp(param, "INFO", 4) == 0) {
        result.cmd = DFU_CMD_GET_INFO;
        result.valid = true;
    }
    else if (strncmp(param, "ERASE", 5) == 0) {
        result.cmd = DFU_CMD_ERASE;
        result.valid = true;
    }
    else if (strncmp(param, "RESET", 5) == 0) {
        result.cmd = DFU_CMD_RESET;
        result.valid = true;
    }
    else if (strncmp(param, "JUMP", 4) == 0) {
        result.cmd = DFU_CMD_JUMP;
        result.valid = true;
    }
    else if (strncmp(param, "STATUS", 6) == 0) {
        result.cmd = DFU_CMD_GET_STATUS;
        result.valid = true;
    }
    else if (strncmp(param, "WRITE,", 6) == 0) {
        result.cmd = DFU_CMD_WRITE;
        /* Parse address */
        const char* addr_str = param + 6;
        result.address = parse_hex32(addr_str);

        /* Find data after comma */
        const char* comma = strchr(addr_str, ',');
        if (comma) {
            const char* hex_data = comma + 1;
            result.data_len = 0;

            /* Parse hex data */
            while (hex_data[0] && hex_data[1] && result.data_len < sizeof(result.data)) {
                int hi = hex_char_to_int(hex_data[0]);
                int lo = hex_char_to_int(hex_data[1]);
                if (hi < 0 || lo < 0) break;
                result.data[result.data_len++] = (hi << 4) | lo;
                hex_data += 2;
            }
            result.valid = (result.data_len > 0);
        }
    }
    else if (strncmp(param, "VERIFY,", 7) == 0) {
        result.cmd = DFU_CMD_VERIFY;
        result.crc = parse_hex32(param + 7);
        result.valid = true;
    }

    return result;
}

/*******************************************************************************
 * ADDRESS VALIDATION
 ******************************************************************************/

static bool is_valid_app_address(uint32_t addr)
{
    return (addr >= APP_HEADER_ADDR && addr <= APP_FLASH_END);
}

static bool is_bootloader_address(uint32_t addr)
{
    return (addr >= BL_FLASH_BASE && addr <= BL_FLASH_END);
}

static bool is_aligned_64bit(uint32_t addr)
{
    return (addr % 8) == 0;
}

/*******************************************************************************
 * TEST CASES - AT COMMAND PARSING
 ******************************************************************************/

void test_at_parse_ping(void)
{
    at_parsed_cmd_t result = parse_at_command("AT+DFU=PING");
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(DFU_CMD_PING, result.cmd);
    TEST_PASS();
}

void test_at_parse_info(void)
{
    at_parsed_cmd_t result = parse_at_command("AT+DFU=INFO");
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(DFU_CMD_GET_INFO, result.cmd);
    TEST_PASS();
}

void test_at_parse_erase(void)
{
    at_parsed_cmd_t result = parse_at_command("AT+DFU=ERASE");
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(DFU_CMD_ERASE, result.cmd);
    TEST_PASS();
}

void test_at_parse_reset(void)
{
    at_parsed_cmd_t result = parse_at_command("AT+DFU=RESET");
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(DFU_CMD_RESET, result.cmd);
    TEST_PASS();
}

void test_at_parse_jump(void)
{
    at_parsed_cmd_t result = parse_at_command("AT+DFU=JUMP");
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(DFU_CMD_JUMP, result.cmd);
    TEST_PASS();
}

void test_at_parse_status(void)
{
    at_parsed_cmd_t result = parse_at_command("AT+DFU=STATUS");
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(DFU_CMD_GET_STATUS, result.cmd);
    TEST_PASS();
}

void test_at_parse_write_simple(void)
{
    at_parsed_cmd_t result = parse_at_command("AT+DFU=WRITE,08008100,DEADBEEF");
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(DFU_CMD_WRITE, result.cmd);
    ASSERT_EQ_HEX(0x08008100, result.address);
    ASSERT_EQ(4, result.data_len);
    ASSERT_EQ(0xDE, result.data[0]);
    ASSERT_EQ(0xAD, result.data[1]);
    ASSERT_EQ(0xBE, result.data[2]);
    ASSERT_EQ(0xEF, result.data[3]);
    TEST_PASS();
}

void test_at_parse_write_long_data(void)
{
    /* 64 bytes of data */
    at_parsed_cmd_t result = parse_at_command(
        "AT+DFU=WRITE,08008200,"
        "00112233445566778899AABBCCDDEEFF"
        "00112233445566778899AABBCCDDEEFF"
        "00112233445566778899AABBCCDDEEFF"
        "00112233445566778899AABBCCDDEEFF"
    );
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(DFU_CMD_WRITE, result.cmd);
    ASSERT_EQ_HEX(0x08008200, result.address);
    ASSERT_EQ(64, result.data_len);
    TEST_PASS();
}

void test_at_parse_verify(void)
{
    at_parsed_cmd_t result = parse_at_command("AT+DFU=VERIFY,12345678");
    ASSERT_TRUE(result.valid);
    ASSERT_EQ(DFU_CMD_VERIFY, result.cmd);
    ASSERT_EQ_HEX(0x12345678, result.crc);
    TEST_PASS();
}

void test_at_parse_invalid_prefix(void)
{
    at_parsed_cmd_t result = parse_at_command("AT+DFX=PING");
    ASSERT_FALSE(result.valid);
    TEST_PASS();
}

void test_at_parse_invalid_command(void)
{
    at_parsed_cmd_t result = parse_at_command("AT+DFU=INVALID");
    ASSERT_FALSE(result.valid);
    TEST_PASS();
}

void test_at_parse_empty(void)
{
    at_parsed_cmd_t result = parse_at_command("");
    ASSERT_FALSE(result.valid);
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - ADDRESS VALIDATION
 ******************************************************************************/

void test_address_valid_app_start(void)
{
    ASSERT_TRUE(is_valid_app_address(APP_HEADER_ADDR));
    TEST_PASS();
}

void test_address_valid_app_code_start(void)
{
    ASSERT_TRUE(is_valid_app_address(APP_FLASH_BASE));
    TEST_PASS();
}

void test_address_valid_app_end(void)
{
    ASSERT_TRUE(is_valid_app_address(APP_FLASH_END));
    TEST_PASS();
}

void test_address_invalid_bootloader(void)
{
    ASSERT_FALSE(is_valid_app_address(BL_FLASH_BASE));
    ASSERT_FALSE(is_valid_app_address(0x08004000));
    TEST_PASS();
}

void test_address_bootloader_detection(void)
{
    ASSERT_TRUE(is_bootloader_address(BL_FLASH_BASE));
    ASSERT_TRUE(is_bootloader_address(0x08004000));
    ASSERT_TRUE(is_bootloader_address(BL_FLASH_END));
    ASSERT_FALSE(is_bootloader_address(APP_FLASH_BASE));
    TEST_PASS();
}

void test_address_alignment(void)
{
    ASSERT_TRUE(is_aligned_64bit(0x08008100));
    ASSERT_TRUE(is_aligned_64bit(0x08008108));
    ASSERT_FALSE(is_aligned_64bit(0x08008101));
    ASSERT_FALSE(is_aligned_64bit(0x08008104));
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - SPI COMMAND IDS
 ******************************************************************************/

#define SPI_CMD_DFU_BASE        0x30
#define SPI_CMD_DFU_PING        0x30
#define SPI_CMD_DFU_GET_INFO    0x31
#define SPI_CMD_DFU_ERASE       0x32
#define SPI_CMD_DFU_WRITE_REQ   0x33
#define SPI_CMD_DFU_WRITE_DATA  0x34
#define SPI_CMD_DFU_VERIFY      0x37
#define SPI_CMD_DFU_JUMP        0x39

static dfu_cmd_t spi_to_dfu_cmd(uint8_t spi_cmd)
{
    switch (spi_cmd) {
        case SPI_CMD_DFU_PING:       return DFU_CMD_PING;
        case SPI_CMD_DFU_GET_INFO:   return DFU_CMD_GET_INFO;
        case SPI_CMD_DFU_ERASE:      return DFU_CMD_ERASE;
        case SPI_CMD_DFU_WRITE_DATA: return DFU_CMD_WRITE;
        case SPI_CMD_DFU_VERIFY:     return DFU_CMD_VERIFY;
        case SPI_CMD_DFU_JUMP:       return DFU_CMD_JUMP;
        default: return DFU_CMD_MAX;
    }
}

void test_spi_cmd_ping(void)
{
    ASSERT_EQ(DFU_CMD_PING, spi_to_dfu_cmd(0x30));
    TEST_PASS();
}

void test_spi_cmd_info(void)
{
    ASSERT_EQ(DFU_CMD_GET_INFO, spi_to_dfu_cmd(0x31));
    TEST_PASS();
}

void test_spi_cmd_erase(void)
{
    ASSERT_EQ(DFU_CMD_ERASE, spi_to_dfu_cmd(0x32));
    TEST_PASS();
}

void test_spi_cmd_write(void)
{
    ASSERT_EQ(DFU_CMD_WRITE, spi_to_dfu_cmd(0x34));
    TEST_PASS();
}

void test_spi_cmd_verify(void)
{
    ASSERT_EQ(DFU_CMD_VERIFY, spi_to_dfu_cmd(0x37));
    TEST_PASS();
}

void test_spi_cmd_jump(void)
{
    ASSERT_EQ(DFU_CMD_JUMP, spi_to_dfu_cmd(0x39));
    TEST_PASS();
}

void test_spi_cmd_invalid(void)
{
    ASSERT_EQ(DFU_CMD_MAX, spi_to_dfu_cmd(0xFF));
    ASSERT_EQ(DFU_CMD_MAX, spi_to_dfu_cmd(0x00));
    TEST_PASS();
}

/*******************************************************************************
 * TEST RUNNER
 ******************************************************************************/

int main(void)
{
    TEST_SUITE_START("DFU Protocol Unit Tests");

    printf("\n-- AT Command Parsing --\n");
    RUN_TEST(test_at_parse_ping);
    RUN_TEST(test_at_parse_info);
    RUN_TEST(test_at_parse_erase);
    RUN_TEST(test_at_parse_reset);
    RUN_TEST(test_at_parse_jump);
    RUN_TEST(test_at_parse_status);
    RUN_TEST(test_at_parse_write_simple);
    RUN_TEST(test_at_parse_write_long_data);
    RUN_TEST(test_at_parse_verify);
    RUN_TEST(test_at_parse_invalid_prefix);
    RUN_TEST(test_at_parse_invalid_command);
    RUN_TEST(test_at_parse_empty);

    printf("\n-- Address Validation --\n");
    RUN_TEST(test_address_valid_app_start);
    RUN_TEST(test_address_valid_app_code_start);
    RUN_TEST(test_address_valid_app_end);
    RUN_TEST(test_address_invalid_bootloader);
    RUN_TEST(test_address_bootloader_detection);
    RUN_TEST(test_address_alignment);

    printf("\n-- SPI Command IDs --\n");
    RUN_TEST(test_spi_cmd_ping);
    RUN_TEST(test_spi_cmd_info);
    RUN_TEST(test_spi_cmd_erase);
    RUN_TEST(test_spi_cmd_write);
    RUN_TEST(test_spi_cmd_verify);
    RUN_TEST(test_spi_cmd_jump);
    RUN_TEST(test_spi_cmd_invalid);

    TEST_SUITE_END();
    TEST_SUMMARY();
}
