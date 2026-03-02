/**
 * @file    bl_spi.c
 * @brief   SPI communication driver for bootloader DFU (slave mode, polling)
 * @date    2025
 */

#include "bl_spi.h"
#include "bl_spi_protocol.h"
#include "bl_config.h"
#include "stm32wlxx_hal.h"
#include <string.h>

_Static_assert(BL_SPI_TRANSACTION_SIZE <= BL_RX_BUFFER_SIZE,
               "SPI transaction size exceeds RX buffer");

/* SPI handle */
SPI_HandleTypeDef hspi1_bl;

/* RX buffer and state */
static uint8_t rx_buffer[BL_RX_BUFFER_SIZE];
static volatile uint16_t rx_count = 0;
static volatile bool rx_complete = false;

/* TX buffer */
static uint8_t tx_buffer[BL_TX_BUFFER_SIZE];
static uint16_t tx_len = 0;

/* Command parsing state */
static uint8_t current_cmd = 0;
static uint8_t payload_buffer[BL_CHUNK_SIZE];
static uint16_t payload_len = 0;
static volatile bool cmd_ready = false;

/* State for multi-transaction commands (WRITE_REQ -> WRITE_DATA) */
static uint32_t pending_write_addr = 0;
static uint16_t pending_write_len = 0;
static bool write_req_pending = false;

/* State for multi-transaction commands (READ_REQ -> READ_DATA) */
static uint8_t pending_read_buffer[BL_CHUNK_SIZE];
static uint16_t pending_read_len = 0;
static bool read_data_pending = false;

/* SPI state machine */
typedef enum {
    BL_SPI_STATE_IDLE,
    BL_SPI_STATE_WAITING_RX,
    BL_SPI_STATE_PROCESSING,
    BL_SPI_STATE_WAITING_TX
} bl_spi_state_t;

static volatile bl_spi_state_t spi_state = BL_SPI_STATE_IDLE;

/* Debug output (declared in bl_main.h) */
#include "bl_main.h"

#ifdef BL_DEBUG
static void spi_debug_hex8(uint8_t val)
{
    char hex[3];
    uint8_t nib = (val >> 4) & 0xF;
    hex[0] = (nib < 10) ? ('0' + nib) : ('A' + nib - 10);
    nib = val & 0xF;
    hex[1] = (nib < 10) ? ('0' + nib) : ('A' + nib - 10);
    hex[2] = '\0';
    early_debug_print(hex);
}
#endif

/**
 * @brief SPI MSP Initialization - Called by HAL_SPI_Init()
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (spiHandle->Instance == SPI1) {
        /* Enable clocks */
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* Configure SPI1 GPIO:
         * PA1  - SPI1_SCK  (AF5)
         * PA15 - SPI1_NSS  (AF5)
         * PB4  - SPI1_MISO (AF5)
         * PB5  - SPI1_MOSI (AF5)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_15 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/**
 * @brief SPI MSP De-Initialization
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
    if (spiHandle->Instance == SPI1) {
        __HAL_RCC_SPI1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15 | GPIO_PIN_1);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4 | GPIO_PIN_5);
    }
}

bool bl_spi_init(void)
{
    BL_DBG("[SPI] Init\r\n");

    /* Reset SPI1 peripheral */
    __HAL_RCC_SPI1_FORCE_RESET();
    BL_SETTLE_DELAY(100);
    __HAL_RCC_SPI1_RELEASE_RESET();

    /* Configure SPI1 as slave */
    hspi1_bl.Instance = SPI1;
    hspi1_bl.Init.Mode = SPI_MODE_SLAVE;
    hspi1_bl.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1_bl.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1_bl.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1_bl.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1_bl.Init.NSS = SPI_NSS_HARD_INPUT;
    hspi1_bl.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1_bl.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1_bl.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1_bl.Init.CRCPolynomial = 7;
    hspi1_bl.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1_bl.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi1_bl) != HAL_OK) {
        early_debug_print("[SPI] Init FAILED\r\n");
        return false;
    }

    /* Initialize protocol layer */
    bl_spi_protocol_init();

    /* Reset state */
    rx_count = 0;
    rx_complete = false;
    current_cmd = 0;
    payload_len = 0;
    cmd_ready = false;
    pending_write_addr = 0;
    pending_write_len = 0;
    write_req_pending = false;
    pending_read_len = 0;
    read_data_pending = false;
    spi_state = BL_SPI_STATE_IDLE;

    BL_DBG("[SPI] Init OK\r\n");
    return true;
}

void bl_spi_deinit(void)
{
    HAL_SPI_DeInit(&hspi1_bl);
}

/**
 * @brief Start SPI reception using polling
 */
static bool bl_spi_start_polling_rx(void)
{
    /* Don't start if NSS is low (transaction in progress) */
    if (!(GPIOA->IDR & GPIO_PIN_15)) {
        return false;
    }

    /* Ensure SPI is ready */
    if (hspi1_bl.State != HAL_SPI_STATE_READY) {
        HAL_SPI_Abort(&hspi1_bl);
        hspi1_bl.State = HAL_SPI_STATE_READY;
    }

    /* Clear errors and flush RX FIFO */
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1_bl);
    hspi1_bl.ErrorCode = HAL_SPI_ERROR_NONE;
    while (SPI1->SR & SPI_SR_RXNE) {
        (void)SPI1->DR;
    }

    /* Reset state */
    memset(rx_buffer, 0x00, BL_RX_BUFFER_SIZE);
    spi_state = BL_SPI_STATE_WAITING_RX;

    /* Enable SPI and pre-fill TX FIFO with idle pattern */
    __HAL_SPI_ENABLE(&hspi1_bl);
    for (int i = 0; i < 4; i++) {
        while (!(SPI1->SR & SPI_SR_TXE));
        *(__IO uint8_t *)&SPI1->DR = BL_SPI_IDLE_PATTERN;
    }

    return true;
}

/**
 * @brief Poll for SPI data
 * @return Number of bytes received when transaction completes
 */
static uint16_t bl_spi_poll_rx(void)
{
    uint16_t count = 0;
    uint32_t timeout_start = HAL_GetTick();
    bool got_data = false;

    /* Wait for NSS to go low (transaction start) */
    while ((GPIOA->IDR & GPIO_PIN_15) && (HAL_GetTick() - timeout_start < 100)) {
    }

    if (GPIOA->IDR & GPIO_PIN_15) {
        return 0;
    }

    /* NSS low - receive bytes */
    timeout_start = HAL_GetTick();

    while (count < BL_SPI_TRANSACTION_SIZE && (HAL_GetTick() - timeout_start < 100)) {
        if (SPI1->SR & SPI_SR_RXNE) {
            rx_buffer[count++] = *(__IO uint8_t *)&SPI1->DR;
            got_data = true;
            timeout_start = HAL_GetTick();
        }

        /* Check if NSS went high (transaction end) */
        if (got_data && (GPIOA->IDR & GPIO_PIN_15)) {
            break;
        }
    }

    return count;
}

void bl_spi_start_rx(void)
{
    rx_count = 0;
    rx_complete = false;

    if (!bl_spi_start_polling_rx()) {
        spi_state = BL_SPI_STATE_IDLE;
    }
}

void bl_spi_stop_rx(void)
{
    HAL_SPI_Abort(&hspi1_bl);
    spi_state = BL_SPI_STATE_IDLE;
}

/**
 * @brief Handle poll transaction in TX state - clock out response
 */
static void bl_spi_handle_tx_poll(void)
{
    uint32_t timeout_start = HAL_GetTick();
    uint16_t tx_index = 4;  /* First 4 bytes already in FIFO */
    uint16_t rx_idx = 0;

    /* Wait for NSS to go low */
    while ((GPIOA->IDR & GPIO_PIN_15) && (HAL_GetTick() - timeout_start < 100)) {
    }

    if (GPIOA->IDR & GPIO_PIN_15) {
        spi_state = BL_SPI_STATE_IDLE;
        return;
    }

    timeout_start = HAL_GetTick();

    while (rx_idx < BL_SPI_TRANSACTION_SIZE && (HAL_GetTick() - timeout_start < 100)) {
        if (SPI1->SR & SPI_SR_RXNE) {
            (void)*(__IO uint8_t *)&SPI1->DR;
            rx_idx++;
            timeout_start = HAL_GetTick();

            if (tx_index < tx_len && (SPI1->SR & SPI_SR_TXE)) {
                *(__IO uint8_t *)&SPI1->DR = tx_buffer[tx_index++];
            } else if (SPI1->SR & SPI_SR_TXE) {
                *(__IO uint8_t *)&SPI1->DR = BL_SPI_IDLE_PATTERN;
            }
        }

        if (rx_idx > 0 && (GPIOA->IDR & GPIO_PIN_15)) {
            break;
        }
    }

    spi_state = BL_SPI_STATE_IDLE;
}

bool bl_spi_has_data(void)
{
    /* Handle response transmission */
    if (spi_state == BL_SPI_STATE_WAITING_TX) {
        bl_spi_handle_tx_poll();
        rx_complete = false;
        rx_count = 0;
        return false;
    }

    if (spi_state == BL_SPI_STATE_IDLE) {
        rx_count = 0;
        rx_complete = false;
        if (!bl_spi_start_polling_rx()) {
            return false;
        }
        return false;
    }

    if (spi_state == BL_SPI_STATE_WAITING_RX) {
        uint16_t bytes = bl_spi_poll_rx();

        if (bytes > 0) {
            /* Validate frame */
            if (bytes >= 5 && rx_buffer[0] == BL_SPI_IDLE_PATTERN) {
                rx_count = bytes;
                rx_complete = true;
                spi_state = BL_SPI_STATE_PROCESSING;
            } else {
                spi_state = BL_SPI_STATE_IDLE;
                return false;
            }
        }
    }

    return rx_complete || (rx_count > 0);
}

uint16_t bl_spi_rx_available(void)
{
    return rx_count;
}

uint16_t bl_spi_read(uint8_t* buffer, uint16_t max_len)
{
    uint16_t len = (rx_count < max_len) ? rx_count : max_len;
    memcpy(buffer, rx_buffer, len);
    return len;
}

bool bl_spi_read_byte(uint8_t* byte)
{
    if (rx_count == 0) return false;
    *byte = rx_buffer[0];
    return true;
}

bool bl_spi_prepare_tx(const uint8_t* data, uint16_t len)
{
    if (len > BL_TX_BUFFER_SIZE) len = BL_TX_BUFFER_SIZE;
    memcpy(tx_buffer, data, len);
    tx_len = len;
    return true;
}

void bl_spi_flush_rx(void)
{
    rx_count = 0;
    rx_complete = false;
}

/**
 * @brief Check if buffer is a poll frame (all 0xAA)
 */
static bool is_poll_frame(const uint8_t *data, uint16_t len)
{
    if (len < 5) return true;
    return (data[0] == BL_SPI_IDLE_PATTERN &&
            data[1] == BL_SPI_IDLE_PATTERN &&
            data[2] == BL_SPI_IDLE_PATTERN &&
            data[3] == BL_SPI_IDLE_PATTERN);
}

bool bl_spi_process(void)
{
    bl_spi_has_data();

    if (cmd_ready) return true;
    if (!rx_complete) return false;

    rx_complete = false;

    if (rx_count < 1) {
        bl_spi_start_rx();
        return false;
    }

    /* Skip poll frames */
    if (is_poll_frame(rx_buffer, rx_count)) {
        bl_spi_start_rx();
        return false;
    }

#ifdef BL_DEBUG
    /* Debug: show command header */
    early_debug_print("[CMD:");
    spi_debug_hex8(rx_buffer[2]);
    early_debug_print("]\r\n");
#endif

    /* Parse with protocol layer */
    if (!bl_spi_protocol_process(rx_buffer, rx_count)) {
        bl_spi_start_rx();
        return false;
    }

    bl_spi_request_t *request = bl_spi_protocol_get_request();
    bl_spi_protocol_ctx_t *prot_ctx = bl_spi_protocol_get_ctx();

    /* Check CRC for A+ protocol */
    if (prot_ctx->mode == BL_PROT_MODE_A_PLUS && !request->crc_valid) {
        bl_spi_send_response(DFU_RSP_CRC_ERROR, NULL, 0);
        bl_spi_start_rx();
        return false;
    }

    /* Extract command and payload */
    current_cmd = request->command;
    payload_len = request->data_len;

    if (payload_len > 0 && payload_len <= BL_CHUNK_SIZE) {
        memcpy(payload_buffer, request->data, payload_len);
    }

    /* Handle WRITE_REQ - store address/length */
    if (current_cmd == SPI_CMD_DFU_WRITE_REQ && payload_len >= 6) {
        memcpy(&pending_write_addr, payload_buffer, 4);
        memcpy(&pending_write_len, payload_buffer + 4, 2);
        write_req_pending = true;
    }

    /* Validate WRITE_DATA has pending request */
    if (current_cmd == SPI_CMD_DFU_WRITE_DATA) {
        if (!write_req_pending || pending_write_len == 0) {
            bl_spi_send_response(DFU_RSP_NOT_READY, NULL, 0);
            bl_spi_start_rx();
            return false;
        }
    }

    cmd_ready = true;
    return cmd_ready;
}

uint8_t bl_spi_get_command(void)
{
    return current_cmd;
}

uint16_t bl_spi_get_payload(uint8_t* buffer, uint16_t max_len)
{
    /* For WRITE_DATA, prepend stored address */
    if (current_cmd == SPI_CMD_DFU_WRITE_DATA && write_req_pending) {
        if (max_len < 4 + payload_len) return 0;
        memcpy(buffer, &pending_write_addr, 4);
        memcpy(buffer + 4, payload_buffer, payload_len);
        return 4 + payload_len;
    }

    uint16_t len = (payload_len < max_len) ? payload_len : max_len;
    memcpy(buffer, payload_buffer, len);
    return len;
}

bool bl_spi_store_read_data(const uint8_t* data, uint16_t len)
{
    if (data == NULL || len == 0 || len > BL_CHUNK_SIZE) return false;
    memcpy(pending_read_buffer, data, len);
    pending_read_len = len;
    read_data_pending = true;
    return true;
}

uint16_t bl_spi_get_read_data(uint8_t* buffer, uint16_t max_len)
{
    if (!read_data_pending || buffer == NULL) return 0;
    uint16_t len = (pending_read_len < max_len) ? pending_read_len : max_len;
    memcpy(buffer, pending_read_buffer, len);
    return len;
}

bool bl_spi_has_read_data(void)
{
    return read_data_pending;
}

void bl_spi_send_response(dfu_response_t status, const uint8_t* data, uint16_t data_len)
{
    bl_spi_protocol_ctx_t *prot_ctx = bl_spi_protocol_get_ctx();

    /* Build response based on protocol mode */
    if (prot_ctx->mode == BL_PROT_MODE_A_PLUS) {
        uint8_t data_len_u8 = (data_len > 255) ? 255 : (uint8_t)data_len;
        tx_len = bl_spi_protocol_build_response(tx_buffer,
                                                 prot_ctx->request.sequence,
                                                 (bl_prot_status_t)status,
                                                 data, data_len_u8);
    } else {
        tx_buffer[0] = (uint8_t)status;
        if (data != NULL && data_len > 0) {
            uint16_t copy_len = (data_len < BL_TX_BUFFER_SIZE - 1) ? data_len : (BL_TX_BUFFER_SIZE - 1);
            memcpy(tx_buffer + 1, data, copy_len);
            tx_len = copy_len + 1;
        } else {
            tx_len = 1;
        }
        if (tx_len < 16) {
            memset(&tx_buffer[tx_len], BL_SPI_IDLE_PATTERN, 16 - tx_len);
            tx_len = 16;
        }
    }

    /* Clear pending states */
    if (current_cmd == SPI_CMD_DFU_WRITE_DATA) {
        write_req_pending = false;
        pending_write_addr = 0;
        pending_write_len = 0;
    }
    if (current_cmd == SPI_CMD_DFU_READ_DATA) {
        read_data_pending = false;
        pending_read_len = 0;
    }

    /* Reset command state */
    current_cmd = 0;
    payload_len = 0;
    cmd_ready = false;
    rx_complete = false;
    rx_count = 0;

    /* Wait for any ongoing transaction to complete */
    uint32_t wait_start = HAL_GetTick();
    while (!(GPIOA->IDR & GPIO_PIN_15) && (HAL_GetTick() - wait_start < 10)) {
    }

    /* Clear errors and flush FIFO */
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1_bl);
    hspi1_bl.ErrorCode = HAL_SPI_ERROR_NONE;
    while (SPI1->SR & SPI_SR_RXNE) {
        (void)SPI1->DR;
    }

    /* Wait for TX FIFO to empty */
    wait_start = HAL_GetTick();
    while ((SPI1->SR & SPI_SR_FTLVL) && (HAL_GetTick() - wait_start < 5)) {
    }

    /* Pre-fill TX FIFO with response data */
    uint16_t prefill = (tx_len > 4) ? 4 : tx_len;
    for (uint16_t i = 0; i < prefill; i++) {
        while (!(SPI1->SR & SPI_SR_TXE));
        *(__IO uint8_t *)&SPI1->DR = tx_buffer[i];
    }

    spi_state = BL_SPI_STATE_WAITING_TX;
}

/**
 * @brief Wait for pending TX response to be clocked out by master
 *
 * Must be called after bl_spi_send_response() and before any state
 * transition (VALIDATE, JUMP, RESET) that would prevent the normal
 * bl_spi_has_data() -> bl_spi_handle_tx_poll() path from running.
 *
 * Without this, only the first 4 pre-filled FIFO bytes are sent;
 * remaining bytes (including CRC) never reach the master.
 */
void bl_spi_wait_tx_done(void)
{
    uint32_t start = HAL_GetTick();
    while (spi_state == BL_SPI_STATE_WAITING_TX &&
           (HAL_GetTick() - start) < 500) {
        bl_spi_handle_tx_poll();
    }
    if (spi_state == BL_SPI_STATE_WAITING_TX) {
        spi_state = BL_SPI_STATE_IDLE;
    }
}

void bl_spi_irq_handler(void)
{
    /* Not used in polling mode */
}
