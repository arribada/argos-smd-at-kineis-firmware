/**
 * @file    bl_spi.c
 * @brief   SPI communication driver for bootloader DFU (slave mode with DMA)
 * @date    2025
 *
 * This driver uses DMA for reliable SPI slave operation and polling-based
 * transaction end detection to handle variable-length commands.
 */

#include "bl_spi.h"
#include "bl_spi_protocol.h"
#include "bl_config.h"
#include "stm32wlxx_hal.h"
#include <string.h>

/* SPI handle */
static SPI_HandleTypeDef hspi;

/* DMA handles */
static DMA_HandleTypeDef hdma_spi_rx;
static DMA_HandleTypeDef hdma_spi_tx;

/* RX buffer and state */
static uint8_t rx_buffer[BL_RX_BUFFER_SIZE];
static volatile uint16_t rx_count = 0;
static volatile bool rx_complete = false;

/* TX buffer - filled with idle pattern for slave MISO */
static uint8_t tx_buffer[BL_TX_BUFFER_SIZE];
static uint8_t tx_idle_buffer[BL_RX_BUFFER_SIZE];  /* Idle TX with 0xAA pattern */
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

/* Transaction detection state (polling-based) */
static volatile uint16_t last_rx_count = 0;
static volatile uint32_t rx_stable_start_tick = 0;
static volatile bool rx_activity_detected = false;

/* Use centralized timeout from bl_config.h */
#define RX_STABLE_TIMEOUT_MS    BL_SPI_RX_STABLE_TIMEOUT_MS

/* Maximum frame size for DMA setup */
#define SPI_MAX_FRAME_SIZE      255

/* SPI state machine */
typedef enum {
    BL_SPI_STATE_IDLE,
    BL_SPI_STATE_WAITING_RX,
    BL_SPI_STATE_PROCESSING,
    BL_SPI_STATE_WAITING_TX,
    BL_SPI_STATE_ERROR
} bl_spi_state_t;

static volatile bl_spi_state_t spi_state = BL_SPI_STATE_IDLE;

/* Forward declarations */
static bool bl_spi_init_dma(void);
static bool bl_spi_start_dma_rx(void);
static bool bl_spi_check_transaction_end(uint16_t *bytes_received);
static void bl_spi_clear_errors(void);

bool bl_spi_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    /* Configure SPI1 GPIO (same pinout as application):
     * PA1  - SPI1_SCK
     * PA15 - SPI1_NSS
     * PB4  - SPI1_MISO
     * PB5  - SPI1_MOSI
     */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_15;
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

    /* Configure SPI1 as slave */
    hspi.Instance = SPI1;
    hspi.Init.Mode = SPI_MODE_SLAVE;
    hspi.Init.Direction = SPI_DIRECTION_2LINES;
    hspi.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi.Init.NSS = SPI_NSS_HARD_INPUT;
    hspi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi.Init.CRCPolynomial = 7;
    hspi.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    if (HAL_SPI_Init(&hspi) != HAL_OK) {
        return false;
    }

    /* Initialize DMA */
    if (!bl_spi_init_dma()) {
        return false;
    }

    /* Configure NVIC - SPI and DMA at HIGH priority (1), higher than UART (6) */
    HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    /* Initialize idle TX buffer with idle pattern for scope debugging */
    memset(tx_idle_buffer, BL_SPI_IDLE_PATTERN, sizeof(tx_idle_buffer));

    /* Initialize A+ protocol layer */
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
    last_rx_count = 0;
    rx_stable_start_tick = 0;
    rx_activity_detected = false;
    spi_state = BL_SPI_STATE_IDLE;

    return true;
}

static bool bl_spi_init_dma(void)
{
    /* SPI1_RX DMA Init - DMA1 Channel 1 */
    hdma_spi_rx.Instance = DMA1_Channel1;
    hdma_spi_rx.Init.Request = DMA_REQUEST_SPI1_RX;
    hdma_spi_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_spi_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi_rx.Init.Mode = DMA_NORMAL;
    hdma_spi_rx.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&hdma_spi_rx) != HAL_OK) {
        return false;
    }
    __HAL_LINKDMA(&hspi, hdmarx, hdma_spi_rx);

    /* SPI1_TX DMA Init - DMA1 Channel 2 */
    hdma_spi_tx.Instance = DMA1_Channel2;
    hdma_spi_tx.Init.Request = DMA_REQUEST_SPI1_TX;
    hdma_spi_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi_tx.Init.Mode = DMA_NORMAL;
    hdma_spi_tx.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&hdma_spi_tx) != HAL_OK) {
        return false;
    }
    __HAL_LINKDMA(&hspi, hdmatx, hdma_spi_tx);

    return true;
}

void bl_spi_deinit(void)
{
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);

    HAL_DMA_DeInit(&hdma_spi_rx);
    HAL_DMA_DeInit(&hdma_spi_tx);
    HAL_SPI_DeInit(&hspi);

    /* Reset GPIO to analog */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_15);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4 | GPIO_PIN_5);
}

void bl_spi_start_rx(void)
{
    /* Reset state for new reception */
    rx_count = 0;
    rx_complete = false;
    last_rx_count = 0;
    rx_stable_start_tick = 0;
    rx_activity_detected = false;

    /* Start DMA reception */
    bl_spi_start_dma_rx();
}

static bool bl_spi_start_dma_rx(void)
{
    /* Clear any pending errors */
    bl_spi_clear_errors();

    /* Ensure SPI is ready */
    if (hspi.State != HAL_SPI_STATE_READY) {
        HAL_SPI_Abort(&hspi);
        hspi.State = HAL_SPI_STATE_READY;
    }

    /* Reset transaction detection */
    last_rx_count = 0;
    rx_stable_start_tick = 0;
    rx_activity_detected = false;

    spi_state = BL_SPI_STATE_WAITING_RX;

    /* Start DMA transfer - receive max frame size, use polling to detect actual end */
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive_DMA(&hspi,
        tx_idle_buffer, rx_buffer, SPI_MAX_FRAME_SIZE);

    return (ret == HAL_OK);
}

static void bl_spi_clear_errors(void)
{
    if (hspi.ErrorCode != HAL_SPI_ERROR_NONE) {
        if (hspi.ErrorCode & HAL_SPI_ERROR_OVR) {
            __HAL_SPI_CLEAR_OVRFLAG(&hspi);
        }
        if (hspi.ErrorCode & HAL_SPI_ERROR_MODF) {
            __HAL_SPI_CLEAR_MODFFLAG(&hspi);
        }
        if (hspi.ErrorCode & HAL_SPI_ERROR_FRE) {
            __HAL_SPI_CLEAR_FREFLAG(&hspi);
        }
        hspi.ErrorCode = HAL_SPI_ERROR_NONE;
    }
}

void bl_spi_stop_rx(void)
{
    HAL_SPI_Abort(&hspi);
    spi_state = BL_SPI_STATE_IDLE;
}

bool bl_spi_has_data(void)
{
    /* Check for transaction end using polling */
    if (spi_state == BL_SPI_STATE_WAITING_RX) {
        uint16_t bytes_received = 0;
        if (bl_spi_check_transaction_end(&bytes_received)) {
            rx_count = bytes_received;
            rx_complete = true;

            /* Abort DMA to stop the transfer */
            HAL_SPI_Abort(&hspi);
            hspi.State = HAL_SPI_STATE_READY;

            spi_state = BL_SPI_STATE_PROCESSING;
        }
    }

    return rx_complete || (rx_count > 0);
}

static bool bl_spi_check_transaction_end(uint16_t *bytes_received)
{
    /* Calculate bytes received from DMA NDTR register */
    uint16_t total_requested = hspi.RxXferSize;
    uint16_t remaining;

    /* Defensive null check for DMA handle */
    if (hspi.hdmarx != NULL && hspi.hdmarx->Instance != NULL) {
        remaining = __HAL_DMA_GET_COUNTER(hspi.hdmarx);
    } else {
        /* Fallback: no DMA, assume no data */
        return false;
    }

    /* Protect against underflow */
    uint16_t current_rx_count = (remaining <= total_requested) ?
        (total_requested - remaining) : 0;

    /* Check if new bytes received */
    if (current_rx_count > last_rx_count) {
        rx_activity_detected = true;
        last_rx_count = current_rx_count;
        rx_stable_start_tick = HAL_GetTick();
        return false;
    }

    /* No new bytes - check if transaction complete */
    if (rx_activity_detected && current_rx_count > 0) {
        if (rx_stable_start_tick == 0) {
            rx_stable_start_tick = HAL_GetTick();
        }

        /* Check stability timeout */
        if ((HAL_GetTick() - rx_stable_start_tick) >= RX_STABLE_TIMEOUT_MS) {
            *bytes_received = current_rx_count;

            /* Reset for next transaction */
            last_rx_count = 0;
            rx_stable_start_tick = 0;
            rx_activity_detected = false;

            return true;
        }
    }

    return false;
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
    if (rx_count == 0) {
        return false;
    }
    *byte = rx_buffer[0];
    return true;
}

bool bl_spi_prepare_tx(const uint8_t* data, uint16_t len)
{
    if (len > BL_TX_BUFFER_SIZE) {
        len = BL_TX_BUFFER_SIZE;
    }
    memcpy(tx_buffer, data, len);
    tx_len = len;
    return true;
}

void bl_spi_flush_rx(void)
{
    rx_count = 0;
    rx_complete = false;
}

bool bl_spi_process(void)
{
    /* First check for new data via polling */
    bl_spi_has_data();

    if (cmd_ready) {
        return true;
    }

    if (!rx_complete) {
        return false;
    }

    /* Process received data */
    rx_complete = false;

    if (rx_count < 1) {
        /* No data, restart reception */
        bl_spi_start_rx();
        return false;
    }

    /* Use protocol layer to parse frame (detects A+ vs legacy) */
    if (!bl_spi_protocol_process(rx_buffer, rx_count)) {
        /* Invalid frame or sync issue - restart */
        bl_spi_start_rx();
        return false;
    }

    bl_spi_request_t *request = bl_spi_protocol_get_request();
    bl_spi_protocol_ctx_t *prot_ctx = bl_spi_protocol_get_ctx();

    /* Check CRC for A+ protocol */
    if (prot_ctx->mode == BL_PROT_MODE_A_PLUS && !request->crc_valid) {
        /* CRC error - send error response and restart */
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

    /* Handle WRITE_REQ - store address/length for WRITE_DATA */
    if (current_cmd == SPI_CMD_DFU_WRITE_REQ && payload_len >= 6) {
        memcpy(&pending_write_addr, payload_buffer, 4);
        memcpy(&pending_write_len, payload_buffer + 4, 2);
        write_req_pending = true;
    }

    /* For WRITE_DATA with pending request, validate we got the expected length */
    if (current_cmd == SPI_CMD_DFU_WRITE_DATA) {
        if (!write_req_pending || pending_write_len == 0) {
            /* No pending WRITE_REQ - error */
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
        if (max_len < 4 + payload_len) {
            return 0;
        }
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
    if (data == NULL || len == 0 || len > BL_CHUNK_SIZE) {
        return false;
    }
    memcpy(pending_read_buffer, data, len);
    pending_read_len = len;
    read_data_pending = true;
    return true;
}

uint16_t bl_spi_get_read_data(uint8_t* buffer, uint16_t max_len)
{
    if (!read_data_pending || buffer == NULL) {
        return 0;
    }
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
        /* A+ protocol: [MAGIC][SEQ][STATUS][LEN][DATA...][CRC] */
        uint8_t data_len_u8 = (data_len > 250) ? 250 : (uint8_t)data_len;
        tx_len = bl_spi_protocol_build_response(tx_buffer,
                                                 prot_ctx->request.sequence,
                                                 (bl_prot_status_t)status,
                                                 data, data_len_u8);
    } else {
        /* Legacy protocol: [STATUS][DATA...] */
        tx_buffer[0] = (uint8_t)status;

        if (data != NULL && data_len > 0) {
            uint16_t copy_len = (data_len < BL_TX_BUFFER_SIZE - 1) ? data_len : (BL_TX_BUFFER_SIZE - 1);
            memcpy(tx_buffer + 1, data, copy_len);
            tx_len = copy_len + 1;
        } else {
            tx_len = 1;
        }

        /* Pad TX buffer with idle pattern for master over-read */
        if (tx_len < 16) {
            memset(&tx_buffer[tx_len], BL_SPI_IDLE_PATTERN, 16 - tx_len);
            tx_len = 16;
        }
    }

    /* Clear write pending state after WRITE_DATA */
    if (current_cmd == SPI_CMD_DFU_WRITE_DATA) {
        write_req_pending = false;
        pending_write_addr = 0;
        pending_write_len = 0;
    }

    /* Clear read pending state after READ_DATA */
    if (current_cmd == SPI_CMD_DFU_READ_DATA) {
        read_data_pending = false;
        pending_read_len = 0;
    }

    /* Reset command state */
    current_cmd = 0;
    payload_len = 0;
    cmd_ready = false;

    /* Clear errors and ensure ready state */
    bl_spi_clear_errors();
    if (hspi.State != HAL_SPI_STATE_READY) {
        HAL_SPI_Abort(&hspi);
        hspi.State = HAL_SPI_STATE_READY;
    }

    /* Setup DMA for response TX + next RX */
    spi_state = BL_SPI_STATE_WAITING_TX;

    /* Reset transaction detection for next command */
    last_rx_count = 0;
    rx_stable_start_tick = 0;
    rx_activity_detected = false;

    /* Start combined TX/RX DMA - response goes out, ready for next command */
    HAL_SPI_TransmitReceive_DMA(&hspi, tx_buffer, rx_buffer, SPI_MAX_FRAME_SIZE);
}

void bl_spi_irq_handler(void)
{
    /* Handled by HAL */
}

/* HAL SPI callbacks - minimal processing, no logging */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi_ptr)
{
    if (hspi_ptr->Instance == SPI1) {
        /* DMA transfer complete - but we use polling for transaction detection */
        if (spi_state == BL_SPI_STATE_WAITING_TX) {
            /* Response sent, go back to idle/waiting */
            spi_state = BL_SPI_STATE_WAITING_RX;
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi_ptr)
{
    if (hspi_ptr->Instance == SPI1) {
        /* In WAITING_RX or WAITING_TX, errors are normal (CS deassert) */
        /* Just clear errors, let polling handle transaction detection */
        if (hspi_ptr->ErrorCode & HAL_SPI_ERROR_OVR) {
            __HAL_SPI_CLEAR_OVRFLAG(hspi_ptr);
        }
        if (hspi_ptr->ErrorCode & HAL_SPI_ERROR_MODF) {
            __HAL_SPI_CLEAR_MODFFLAG(hspi_ptr);
        }
        if (hspi_ptr->ErrorCode & HAL_SPI_ERROR_FRE) {
            __HAL_SPI_CLEAR_FREFLAG(hspi_ptr);
        }
        hspi_ptr->ErrorCode = HAL_SPI_ERROR_NONE;
    }
}

/* IRQ Handlers */
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi);
}

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi_rx);
}

void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi_tx);
}
