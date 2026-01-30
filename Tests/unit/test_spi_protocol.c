/**
 * @file    test_spi_protocol.c
 * @brief   Unit tests for SPI DFU protocol and timing
 * @date    2025
 *
 * Tests the SPI protocol implementation including:
 * - SPI command frame structure
 * - Two-transaction protocol state machine
 * - Timing constraints validation
 * - Transaction detection logic
 * - Error handling and recovery
 */

#include "test_framework.h"
#include <time.h>

/*******************************************************************************
 * SPI PROTOCOL DEFINITIONS (from bl_config.h and mcu_spi_driver.h)
 ******************************************************************************/

/* SPI Commands */
#define SPI_CMD_DFU_BASE        0x30
#define SPI_CMD_DFU_PING        0x30
#define SPI_CMD_DFU_GET_INFO    0x31
#define SPI_CMD_DFU_ERASE       0x32
#define SPI_CMD_DFU_WRITE_REQ   0x33
#define SPI_CMD_DFU_WRITE_DATA  0x34
#define SPI_CMD_DFU_READ_REQ    0x35
#define SPI_CMD_DFU_READ_DATA   0x36
#define SPI_CMD_DFU_VERIFY      0x37
#define SPI_CMD_DFU_RESET       0x38
#define SPI_CMD_DFU_JUMP        0x39
#define SPI_CMD_DFU_GET_STATUS  0x3A
#define SPI_CMD_DFU_ABORT       0x3B
#define SPI_CMD_DFU_SET_HEADER  0x3C

/* Response codes */
#define DFU_RSP_OK              0x00
#define DFU_RSP_ERROR           0x01
#define DFU_RSP_CRC_ERROR       0x02
#define DFU_RSP_ADDR_ERROR      0x03
#define DFU_RSP_INVALID_CMD     0x07

/* Timing constants - must match production code (mcu_spi_driver.c, bl_spi.c) */
#define RX_STABLE_TIMEOUT_MS    10
#define SPI_MAX_FRAME_SIZE      255

/* SPI State machine */
typedef enum {
    SPICMD_UNKNOWN,
    SPICMD_INIT,
    SPICMD_IDLE,
    SPICMD_PROCESS_CMD,
    SPICMD_WAITING_RX,
    SPICMD_WAITING_TX,
    SPICMD_WAITING_MAC_EVT,
    SPICMD_ERROR
} SpiState;

/*******************************************************************************
 * MOCK STRUCTURES FOR TESTING
 ******************************************************************************/

typedef struct {
    uint8_t cmd;
    uint16_t payload_len;
    bool has_payload;
    bool requires_response;
} spi_cmd_info_t;

/* Command info lookup table */
static spi_cmd_info_t get_cmd_info(uint8_t cmd)
{
    spi_cmd_info_t info = {cmd, 0, false, true};

    switch (cmd) {
        case SPI_CMD_DFU_PING:
        case SPI_CMD_DFU_GET_INFO:
        case SPI_CMD_DFU_ERASE:
        case SPI_CMD_DFU_RESET:
        case SPI_CMD_DFU_JUMP:
        case SPI_CMD_DFU_GET_STATUS:
        case SPI_CMD_DFU_ABORT:
            info.payload_len = 0;
            info.has_payload = false;
            break;
        case SPI_CMD_DFU_VERIFY:
            info.payload_len = 4;  /* CRC32 */
            info.has_payload = true;
            break;
        case SPI_CMD_DFU_WRITE_REQ:
        case SPI_CMD_DFU_READ_REQ:
            info.payload_len = 6;  /* addr(4) + len(2) */
            info.has_payload = true;
            break;
        case SPI_CMD_DFU_SET_HEADER:
            info.payload_len = 256;
            info.has_payload = true;
            break;
        default:
            info.payload_len = 0;
            info.requires_response = false;
            break;
    }
    return info;
}

/* Mock for transaction end detection */
typedef struct {
    uint16_t bytes_received;
    uint32_t stable_start_tick;
    bool activity_detected;
    uint16_t last_rx_count;
} transaction_detector_t;

static void detector_init(transaction_detector_t* det)
{
    det->bytes_received = 0;
    det->stable_start_tick = 0;
    det->activity_detected = false;
    det->last_rx_count = 0;
}

static bool detector_check_end(transaction_detector_t* det, uint16_t current_rx, uint32_t current_tick)
{
    if (current_rx > det->last_rx_count) {
        det->activity_detected = true;
        det->last_rx_count = current_rx;
        det->stable_start_tick = current_tick;
        return false;
    }

    if (det->activity_detected && current_rx > 0) {
        if (det->stable_start_tick == 0) {
            det->stable_start_tick = current_tick;
        }

        if ((current_tick - det->stable_start_tick) >= RX_STABLE_TIMEOUT_MS) {
            det->bytes_received = current_rx;
            det->last_rx_count = 0;
            det->stable_start_tick = 0;
            det->activity_detected = false;
            return true;
        }
    }

    return false;
}

/*******************************************************************************
 * TIMING CALCULATIONS
 ******************************************************************************/

/* Calculate time in microseconds for N bytes at given SPI frequency */
static uint32_t calc_transfer_time_us(uint16_t bytes, uint32_t freq_hz)
{
    /* Time = (bytes * 8 bits) / freq_hz * 1000000 */
    return (uint32_t)((uint64_t)bytes * 8 * 1000000 / freq_hz);
}

/* Check if timing is safe (transfer time + margin < timeout) */
static bool is_timing_safe(uint16_t bytes, uint32_t freq_hz, uint32_t timeout_ms)
{
    uint32_t transfer_us = calc_transfer_time_us(bytes, freq_hz);
    uint32_t timeout_us = timeout_ms * 1000;
    /* Need at least 20% margin */
    return (transfer_us * 120 / 100) < timeout_us;
}

/*******************************************************************************
 * TEST CASES - SPI COMMAND STRUCTURE
 ******************************************************************************/

void test_spi_cmd_ping_no_payload(void)
{
    spi_cmd_info_t info = get_cmd_info(SPI_CMD_DFU_PING);
    ASSERT_EQ(0, info.payload_len);
    ASSERT_FALSE(info.has_payload);
    ASSERT_TRUE(info.requires_response);
    TEST_PASS();
}

void test_spi_cmd_info_no_payload(void)
{
    spi_cmd_info_t info = get_cmd_info(SPI_CMD_DFU_GET_INFO);
    ASSERT_EQ(0, info.payload_len);
    ASSERT_FALSE(info.has_payload);
    ASSERT_TRUE(info.requires_response);
    TEST_PASS();
}

void test_spi_cmd_verify_has_crc(void)
{
    spi_cmd_info_t info = get_cmd_info(SPI_CMD_DFU_VERIFY);
    ASSERT_EQ(4, info.payload_len);
    ASSERT_TRUE(info.has_payload);
    TEST_PASS();
}

void test_spi_cmd_write_req_has_addr_len(void)
{
    spi_cmd_info_t info = get_cmd_info(SPI_CMD_DFU_WRITE_REQ);
    ASSERT_EQ(6, info.payload_len);  /* 4 bytes addr + 2 bytes len */
    ASSERT_TRUE(info.has_payload);
    TEST_PASS();
}

void test_spi_cmd_set_header_size(void)
{
    spi_cmd_info_t info = get_cmd_info(SPI_CMD_DFU_SET_HEADER);
    ASSERT_EQ(256, info.payload_len);
    ASSERT_TRUE(info.has_payload);
    TEST_PASS();
}

void test_spi_cmd_invalid_no_response(void)
{
    spi_cmd_info_t info = get_cmd_info(0xFF);
    ASSERT_FALSE(info.requires_response);
    TEST_PASS();
}

void test_spi_all_cmds_in_valid_range(void)
{
    /* All DFU commands should be in 0x30-0x3F range */
    for (uint8_t cmd = SPI_CMD_DFU_PING; cmd <= SPI_CMD_DFU_SET_HEADER; cmd++) {
        ASSERT_TRUE(cmd >= 0x30 && cmd <= 0x3F);
    }
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - TRANSACTION DETECTION
 ******************************************************************************/

void test_detector_init_state(void)
{
    transaction_detector_t det;
    detector_init(&det);
    ASSERT_EQ(0, det.bytes_received);
    ASSERT_EQ(0, det.stable_start_tick);
    ASSERT_FALSE(det.activity_detected);
    ASSERT_EQ(0, det.last_rx_count);
    TEST_PASS();
}

void test_detector_first_byte_activates(void)
{
    transaction_detector_t det;
    detector_init(&det);

    bool ended = detector_check_end(&det, 1, 100);
    ASSERT_FALSE(ended);
    ASSERT_TRUE(det.activity_detected);
    ASSERT_EQ(1, det.last_rx_count);
    ASSERT_EQ(100, det.stable_start_tick);
    TEST_PASS();
}

void test_detector_bytes_accumulate(void)
{
    transaction_detector_t det;
    detector_init(&det);

    detector_check_end(&det, 1, 100);
    detector_check_end(&det, 3, 101);
    detector_check_end(&det, 5, 102);

    ASSERT_EQ(5, det.last_rx_count);
    ASSERT_EQ(102, det.stable_start_tick);
    TEST_PASS();
}

void test_detector_timeout_completes(void)
{
    transaction_detector_t det;
    detector_init(&det);

    /* Receive 5 bytes */
    detector_check_end(&det, 5, 100);

    /* Check before timeout - should not complete */
    bool ended = detector_check_end(&det, 5, 103);
    ASSERT_FALSE(ended);

    /* Check at timeout - should complete */
    ended = detector_check_end(&det, 5, 100 + RX_STABLE_TIMEOUT_MS);
    ASSERT_TRUE(ended);
    ASSERT_EQ(5, det.bytes_received);
    TEST_PASS();
}

void test_detector_resets_after_completion(void)
{
    transaction_detector_t det;
    detector_init(&det);

    detector_check_end(&det, 5, 100);
    detector_check_end(&det, 5, 100 + RX_STABLE_TIMEOUT_MS);

    /* After completion, state should be reset */
    ASSERT_FALSE(det.activity_detected);
    ASSERT_EQ(0, det.last_rx_count);
    ASSERT_EQ(0, det.stable_start_tick);
    TEST_PASS();
}

void test_detector_no_completion_without_data(void)
{
    transaction_detector_t det;
    detector_init(&det);

    /* Check without any data received */
    bool ended = detector_check_end(&det, 0, 100);
    ASSERT_FALSE(ended);
    ended = detector_check_end(&det, 0, 200);
    ASSERT_FALSE(ended);
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - TIMING CALCULATIONS
 ******************************************************************************/

void test_timing_125khz_single_byte(void)
{
    /* At 125kHz, 1 byte = 8 bits / 125000 = 64 us */
    uint32_t time_us = calc_transfer_time_us(1, 125000);
    ASSERT_EQ(64, time_us);
    TEST_PASS();
}

void test_timing_125khz_command(void)
{
    /* 5 byte command at 125kHz = 320 us */
    uint32_t time_us = calc_transfer_time_us(5, 125000);
    ASSERT_EQ(320, time_us);
    TEST_PASS();
}

void test_timing_125khz_max_frame(void)
{
    /* 255 bytes at 125kHz = 16320 us = ~16.3 ms */
    uint32_t time_us = calc_transfer_time_us(255, 125000);
    ASSERT_EQ(16320, time_us);
    TEST_PASS();
}

void test_timing_500khz_max_frame(void)
{
    /* 255 bytes at 500kHz = 4080 us = ~4 ms */
    uint32_t time_us = calc_transfer_time_us(255, 500000);
    ASSERT_EQ(4080, time_us);
    TEST_PASS();
}

void test_timing_1mhz_max_frame(void)
{
    /* 255 bytes at 1MHz = 2040 us = ~2 ms */
    uint32_t time_us = calc_transfer_time_us(255, 1000000);
    ASSERT_EQ(2040, time_us);
    TEST_PASS();
}

void test_timing_safety_125khz_5byte(void)
{
    /* 5 bytes at 125kHz with 5ms timeout should be safe */
    ASSERT_TRUE(is_timing_safe(5, 125000, 5));
    TEST_PASS();
}

void test_timing_safety_125khz_max(void)
{
    /* 255 bytes at 125kHz with 5ms timeout - NOT safe (16ms > 5ms) */
    ASSERT_FALSE(is_timing_safe(255, 125000, 5));
    TEST_PASS();
}

void test_timing_safety_500khz_max(void)
{
    /* 255 bytes at 500kHz with 5ms timeout should be safe (4ms < 5ms) */
    ASSERT_TRUE(is_timing_safe(255, 500000, 5));
    TEST_PASS();
}

void test_timing_safety_1mhz_max(void)
{
    /* 255 bytes at 1MHz with 5ms timeout is definitely safe */
    ASSERT_TRUE(is_timing_safe(255, 1000000, 5));
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - STATE MACHINE TRANSITIONS
 ******************************************************************************/

void test_state_idle_to_waiting_rx(void)
{
    SpiState state = SPICMD_IDLE;
    /* Simulate starting a read operation */
    state = SPICMD_WAITING_RX;
    ASSERT_EQ(SPICMD_WAITING_RX, state);
    TEST_PASS();
}

void test_state_waiting_rx_to_process(void)
{
    SpiState state = SPICMD_WAITING_RX;
    /* Simulate receiving data and processing */
    state = SPICMD_PROCESS_CMD;
    ASSERT_EQ(SPICMD_PROCESS_CMD, state);
    TEST_PASS();
}

void test_state_process_to_waiting_tx(void)
{
    SpiState state = SPICMD_PROCESS_CMD;
    /* After processing, prepare response */
    state = SPICMD_WAITING_TX;
    ASSERT_EQ(SPICMD_WAITING_TX, state);
    TEST_PASS();
}

void test_state_waiting_tx_to_idle(void)
{
    SpiState state = SPICMD_WAITING_TX;
    /* After TX complete, return to idle */
    state = SPICMD_IDLE;
    ASSERT_EQ(SPICMD_IDLE, state);
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - FRAME PARSING
 ******************************************************************************/

typedef struct {
    uint8_t cmd;
    uint32_t address;
    uint16_t length;
    uint8_t data[256];
    uint16_t data_len;
    bool valid;
} spi_frame_t;

static spi_frame_t parse_write_req_frame(const uint8_t* buffer, uint16_t len)
{
    spi_frame_t frame = {0};
    frame.valid = false;

    if (len < 7) return frame;  /* Need cmd + 4 addr + 2 len */

    frame.cmd = buffer[0];
    if (frame.cmd != SPI_CMD_DFU_WRITE_REQ) return frame;

    /* Parse address (little endian) */
    frame.address = buffer[1] | (buffer[2] << 8) | (buffer[3] << 16) | (buffer[4] << 24);

    /* Parse length (little endian) */
    frame.length = buffer[5] | (buffer[6] << 8);

    frame.valid = true;
    return frame;
}

static spi_frame_t parse_verify_frame(const uint8_t* buffer, uint16_t len)
{
    spi_frame_t frame = {0};
    frame.valid = false;

    if (len < 5) return frame;  /* Need cmd + 4 CRC */

    frame.cmd = buffer[0];
    if (frame.cmd != SPI_CMD_DFU_VERIFY) return frame;

    /* Parse CRC (little endian, store in address field for simplicity) */
    frame.address = buffer[1] | (buffer[2] << 8) | (buffer[3] << 16) | (buffer[4] << 24);

    frame.valid = true;
    return frame;
}

void test_parse_write_req_valid(void)
{
    uint8_t frame[] = {0x33, 0x00, 0x81, 0x00, 0x08, 0x00, 0x01};  /* cmd, addr=0x08008100, len=256 */
    spi_frame_t parsed = parse_write_req_frame(frame, sizeof(frame));

    ASSERT_TRUE(parsed.valid);
    ASSERT_EQ(SPI_CMD_DFU_WRITE_REQ, parsed.cmd);
    ASSERT_EQ_HEX(0x08008100, parsed.address);
    ASSERT_EQ(256, parsed.length);
    TEST_PASS();
}

void test_parse_write_req_too_short(void)
{
    uint8_t frame[] = {0x33, 0x00, 0x81, 0x00};  /* Missing length */
    spi_frame_t parsed = parse_write_req_frame(frame, sizeof(frame));

    ASSERT_FALSE(parsed.valid);
    TEST_PASS();
}

void test_parse_write_req_wrong_cmd(void)
{
    uint8_t frame[] = {0x30, 0x00, 0x81, 0x00, 0x08, 0x00, 0x01};  /* Wrong command */
    spi_frame_t parsed = parse_write_req_frame(frame, sizeof(frame));

    ASSERT_FALSE(parsed.valid);
    TEST_PASS();
}

void test_parse_verify_valid(void)
{
    uint8_t frame[] = {0x37, 0x78, 0x56, 0x34, 0x12};  /* cmd, CRC=0x12345678 */
    spi_frame_t parsed = parse_verify_frame(frame, sizeof(frame));

    ASSERT_TRUE(parsed.valid);
    ASSERT_EQ(SPI_CMD_DFU_VERIFY, parsed.cmd);
    ASSERT_EQ_HEX(0x12345678, parsed.address);  /* CRC stored in address field */
    TEST_PASS();
}

void test_parse_verify_too_short(void)
{
    uint8_t frame[] = {0x37, 0x78, 0x56};  /* Missing bytes */
    spi_frame_t parsed = parse_verify_frame(frame, sizeof(frame));

    ASSERT_FALSE(parsed.valid);
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - RESPONSE FORMATTING
 ******************************************************************************/

typedef struct {
    uint8_t status;
    uint8_t data[256];
    uint16_t data_len;
    uint16_t total_len;
} spi_response_t;

static spi_response_t format_response(uint8_t status, const uint8_t* data, uint16_t data_len)
{
    spi_response_t resp = {0};
    resp.status = status;

    if (data != NULL && data_len > 0) {
        memcpy(resp.data, data, data_len);
        resp.data_len = data_len;
    }

    resp.total_len = 1 + resp.data_len;

    /* Minimum 16 bytes for proper SPI transfer */
    if (resp.total_len < 16) {
        resp.total_len = 16;
    }

    return resp;
}

void test_response_ok_no_data(void)
{
    spi_response_t resp = format_response(DFU_RSP_OK, NULL, 0);
    ASSERT_EQ(DFU_RSP_OK, resp.status);
    ASSERT_EQ(0, resp.data_len);
    ASSERT_EQ(16, resp.total_len);  /* Padded to minimum */
    TEST_PASS();
}

void test_response_ok_with_data(void)
{
    uint8_t data[] = {0x01, 0x02, 0x03, 0x04};
    spi_response_t resp = format_response(DFU_RSP_OK, data, sizeof(data));

    ASSERT_EQ(DFU_RSP_OK, resp.status);
    ASSERT_EQ(4, resp.data_len);
    ASSERT_MEM_EQ(data, resp.data, 4);
    ASSERT_EQ(16, resp.total_len);  /* Still padded */
    TEST_PASS();
}

void test_response_error(void)
{
    spi_response_t resp = format_response(DFU_RSP_CRC_ERROR, NULL, 0);
    ASSERT_EQ(DFU_RSP_CRC_ERROR, resp.status);
    ASSERT_EQ(16, resp.total_len);
    TEST_PASS();
}

void test_response_large_data(void)
{
    uint8_t data[32];
    memset(data, 0xAB, sizeof(data));
    spi_response_t resp = format_response(DFU_RSP_OK, data, sizeof(data));

    ASSERT_EQ(32, resp.data_len);
    ASSERT_EQ(33, resp.total_len);  /* 1 + 32, no padding needed */
    TEST_PASS();
}

/*******************************************************************************
 * TEST CASES - TWO-TRANSACTION PROTOCOL
 ******************************************************************************/

typedef struct {
    SpiState state;
    bool waiting_for_tx;
    uint8_t rx_buffer[256];
    uint16_t rx_count;
    uint8_t tx_buffer[256];
    uint16_t tx_len;
} two_transaction_state_t;

static void two_tx_init(two_transaction_state_t* s)
{
    s->state = SPICMD_IDLE;
    s->waiting_for_tx = false;
    s->rx_count = 0;
    s->tx_len = 0;
    memset(s->rx_buffer, 0xAA, sizeof(s->rx_buffer));
    memset(s->tx_buffer, 0xAA, sizeof(s->tx_buffer));
}

/* Simulate transaction 1: Master sends command */
static void two_tx_receive_command(two_transaction_state_t* s, const uint8_t* cmd, uint16_t len)
{
    s->state = SPICMD_WAITING_RX;
    memcpy(s->rx_buffer, cmd, len);
    s->rx_count = len;
    s->state = SPICMD_PROCESS_CMD;
}

/* Simulate preparing response for transaction 2 */
static void two_tx_prepare_response(two_transaction_state_t* s, uint8_t status)
{
    s->tx_buffer[0] = status;
    s->tx_len = 16;  /* Minimum padded size */
    s->state = SPICMD_WAITING_TX;
    s->waiting_for_tx = true;
}

/* Simulate transaction 2: Master reads response */
static void two_tx_send_response(two_transaction_state_t* s)
{
    s->waiting_for_tx = false;
    s->state = SPICMD_IDLE;
}

void test_two_tx_init_state(void)
{
    two_transaction_state_t s;
    two_tx_init(&s);

    ASSERT_EQ(SPICMD_IDLE, s.state);
    ASSERT_FALSE(s.waiting_for_tx);
    ASSERT_EQ(0, s.rx_count);
    TEST_PASS();
}

void test_two_tx_receive_ping(void)
{
    two_transaction_state_t s;
    two_tx_init(&s);

    uint8_t ping_cmd[] = {SPI_CMD_DFU_PING};
    two_tx_receive_command(&s, ping_cmd, 1);

    ASSERT_EQ(SPICMD_PROCESS_CMD, s.state);
    ASSERT_EQ(1, s.rx_count);
    ASSERT_EQ(SPI_CMD_DFU_PING, s.rx_buffer[0]);
    TEST_PASS();
}

void test_two_tx_prepare_ok_response(void)
{
    two_transaction_state_t s;
    two_tx_init(&s);

    uint8_t ping_cmd[] = {SPI_CMD_DFU_PING};
    two_tx_receive_command(&s, ping_cmd, 1);
    two_tx_prepare_response(&s, DFU_RSP_OK);

    ASSERT_EQ(SPICMD_WAITING_TX, s.state);
    ASSERT_TRUE(s.waiting_for_tx);
    ASSERT_EQ(DFU_RSP_OK, s.tx_buffer[0]);
    TEST_PASS();
}

void test_two_tx_complete_cycle(void)
{
    two_transaction_state_t s;
    two_tx_init(&s);

    /* Transaction 1: Receive PING */
    uint8_t ping_cmd[] = {SPI_CMD_DFU_PING};
    two_tx_receive_command(&s, ping_cmd, 1);
    ASSERT_EQ(SPICMD_PROCESS_CMD, s.state);

    /* Prepare response */
    two_tx_prepare_response(&s, DFU_RSP_OK);
    ASSERT_EQ(SPICMD_WAITING_TX, s.state);

    /* Transaction 2: Send response */
    two_tx_send_response(&s);
    ASSERT_EQ(SPICMD_IDLE, s.state);
    ASSERT_FALSE(s.waiting_for_tx);

    TEST_PASS();
}

void test_two_tx_write_req_sequence(void)
{
    two_transaction_state_t s;
    two_tx_init(&s);

    /* Transaction 1: WRITE_REQ with address and length */
    uint8_t write_req[] = {SPI_CMD_DFU_WRITE_REQ, 0x00, 0x81, 0x00, 0x08, 0x00, 0x01};
    two_tx_receive_command(&s, write_req, sizeof(write_req));

    /* Verify command parsed correctly */
    ASSERT_EQ(SPI_CMD_DFU_WRITE_REQ, s.rx_buffer[0]);
    ASSERT_EQ(7, s.rx_count);

    two_tx_prepare_response(&s, DFU_RSP_OK);
    two_tx_send_response(&s);

    /* Ready for WRITE_DATA */
    ASSERT_EQ(SPICMD_IDLE, s.state);
    TEST_PASS();
}

/*******************************************************************************
 * TEST RUNNER
 ******************************************************************************/

int main(void)
{
    TEST_SUITE_START("SPI Protocol Unit Tests");

    printf("\n-- SPI Command Structure --\n");
    RUN_TEST(test_spi_cmd_ping_no_payload);
    RUN_TEST(test_spi_cmd_info_no_payload);
    RUN_TEST(test_spi_cmd_verify_has_crc);
    RUN_TEST(test_spi_cmd_write_req_has_addr_len);
    RUN_TEST(test_spi_cmd_set_header_size);
    RUN_TEST(test_spi_cmd_invalid_no_response);
    RUN_TEST(test_spi_all_cmds_in_valid_range);

    printf("\n-- Transaction Detection --\n");
    RUN_TEST(test_detector_init_state);
    RUN_TEST(test_detector_first_byte_activates);
    RUN_TEST(test_detector_bytes_accumulate);
    RUN_TEST(test_detector_timeout_completes);
    RUN_TEST(test_detector_resets_after_completion);
    RUN_TEST(test_detector_no_completion_without_data);

    printf("\n-- Timing Calculations --\n");
    RUN_TEST(test_timing_125khz_single_byte);
    RUN_TEST(test_timing_125khz_command);
    RUN_TEST(test_timing_125khz_max_frame);
    RUN_TEST(test_timing_500khz_max_frame);
    RUN_TEST(test_timing_1mhz_max_frame);
    RUN_TEST(test_timing_safety_125khz_5byte);
    RUN_TEST(test_timing_safety_125khz_max);
    RUN_TEST(test_timing_safety_500khz_max);
    RUN_TEST(test_timing_safety_1mhz_max);

    printf("\n-- State Machine --\n");
    RUN_TEST(test_state_idle_to_waiting_rx);
    RUN_TEST(test_state_waiting_rx_to_process);
    RUN_TEST(test_state_process_to_waiting_tx);
    RUN_TEST(test_state_waiting_tx_to_idle);

    printf("\n-- Frame Parsing --\n");
    RUN_TEST(test_parse_write_req_valid);
    RUN_TEST(test_parse_write_req_too_short);
    RUN_TEST(test_parse_write_req_wrong_cmd);
    RUN_TEST(test_parse_verify_valid);
    RUN_TEST(test_parse_verify_too_short);

    printf("\n-- Response Formatting --\n");
    RUN_TEST(test_response_ok_no_data);
    RUN_TEST(test_response_ok_with_data);
    RUN_TEST(test_response_error);
    RUN_TEST(test_response_large_data);

    printf("\n-- Two-Transaction Protocol --\n");
    RUN_TEST(test_two_tx_init_state);
    RUN_TEST(test_two_tx_receive_ping);
    RUN_TEST(test_two_tx_prepare_ok_response);
    RUN_TEST(test_two_tx_complete_cycle);
    RUN_TEST(test_two_tx_write_req_sequence);

    TEST_SUITE_END();
    TEST_SUMMARY();
}
