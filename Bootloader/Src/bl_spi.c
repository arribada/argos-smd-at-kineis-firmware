/**
 * @file    bl_spi.c
 * @brief   SPI communication driver for bootloader DFU (slave mode)
 * @date    2025
 */

#include "bl_spi.h"
#include "bl_config.h"
#include "stm32wlxx_hal.h"
#include <string.h>

/* SPI handle */
static SPI_HandleTypeDef hspi;

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

/* Expected bytes for current command */
static uint16_t expected_bytes = 1;  /* Start with expecting command byte */

bool bl_spi_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    /* Configure SPI1 GPIO:
     * PA1  - SPI1_SCK
     * PA6  - SPI1_MISO (or PB4)
     * PA7  - SPI1_MOSI (or PB5)
     * PA15 - SPI1_NSS
     */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

    if (HAL_SPI_Init(&hspi) != HAL_OK) {
        return false;
    }

    /* Configure NVIC for SPI1 */
    HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    /* Reset state */
    rx_count = 0;
    rx_complete = false;
    current_cmd = 0;
    payload_len = 0;
    cmd_ready = false;
    expected_bytes = 1;

    return true;
}

void bl_spi_deinit(void)
{
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
    HAL_SPI_DeInit(&hspi);

    /* Reset GPIO to analog */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_15);
}

void bl_spi_start_rx(void)
{
    /* Start reception - prepare for first command byte */
    rx_count = 0;
    expected_bytes = 1;

    /* Prepare default response (0x00 = idle) */
    tx_buffer[0] = 0x00;
    tx_len = 1;

    HAL_SPI_TransmitReceive_IT(&hspi, tx_buffer, rx_buffer, expected_bytes);
}

void bl_spi_stop_rx(void)
{
    HAL_SPI_Abort(&hspi);
}

bool bl_spi_has_data(void)
{
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
    if (cmd_ready) {
        return true;
    }

    if (!rx_complete) {
        return false;
    }

    /* Process received data */
    rx_complete = false;

    /* First byte is command */
    if (current_cmd == 0) {
        current_cmd = rx_buffer[0];
        payload_len = 0;

        /* Determine expected payload length based on command */
        switch (current_cmd) {
            case SPI_CMD_DFU_PING:
            case SPI_CMD_DFU_GET_INFO:
            case SPI_CMD_DFU_ERASE:
            case SPI_CMD_DFU_RESET:
            case SPI_CMD_DFU_JUMP:
            case SPI_CMD_DFU_GET_STATUS:
            case SPI_CMD_DFU_ABORT:
                /* No payload */
                cmd_ready = true;
                break;

            case SPI_CMD_DFU_VERIFY:
                /* 4 bytes CRC */
                expected_bytes = 4;
                HAL_SPI_TransmitReceive_IT(&hspi, tx_buffer, rx_buffer, expected_bytes);
                break;

            case SPI_CMD_DFU_WRITE_REQ:
                /* 4 bytes address + 2 bytes length */
                expected_bytes = 6;
                HAL_SPI_TransmitReceive_IT(&hspi, tx_buffer, rx_buffer, expected_bytes);
                break;

            case SPI_CMD_DFU_WRITE_DATA:
                /* First get the length from WRITE_REQ if we have it */
                if (payload_len >= 6) {
                    uint16_t data_len;
                    memcpy(&data_len, payload_buffer + 4, 2);
                    expected_bytes = data_len;
                    HAL_SPI_TransmitReceive_IT(&hspi, tx_buffer, rx_buffer, expected_bytes);
                } else {
                    cmd_ready = true;  /* Error - no length specified */
                }
                break;

            case SPI_CMD_DFU_READ_REQ:
                /* 4 bytes address + 2 bytes length */
                expected_bytes = 6;
                HAL_SPI_TransmitReceive_IT(&hspi, tx_buffer, rx_buffer, expected_bytes);
                break;

            case SPI_CMD_DFU_SET_HEADER:
                /* 256 bytes header */
                expected_bytes = 256;
                HAL_SPI_TransmitReceive_IT(&hspi, tx_buffer, rx_buffer, expected_bytes);
                break;

            default:
                /* Unknown command */
                cmd_ready = true;
                break;
        }
    } else {
        /* Received payload data */
        if (rx_count <= BL_CHUNK_SIZE - payload_len) {
            memcpy(payload_buffer + payload_len, rx_buffer, rx_count);
            payload_len += rx_count;
        }
        cmd_ready = true;
    }

    return cmd_ready;
}

uint8_t bl_spi_get_command(void)
{
    return current_cmd;
}

uint16_t bl_spi_get_payload(uint8_t* buffer, uint16_t max_len)
{
    uint16_t len = (payload_len < max_len) ? payload_len : max_len;
    memcpy(buffer, payload_buffer, len);
    return len;
}

void bl_spi_send_response(dfu_response_t status, const uint8_t* data, uint16_t data_len)
{
    /* Prepare response for next transaction */
    tx_buffer[0] = (uint8_t)status;

    if (data != NULL && data_len > 0) {
        uint16_t copy_len = (data_len < BL_TX_BUFFER_SIZE - 1) ? data_len : (BL_TX_BUFFER_SIZE - 1);
        memcpy(tx_buffer + 1, data, copy_len);
        tx_len = copy_len + 1;
    } else {
        tx_len = 1;
    }

    /* Reset command state */
    current_cmd = 0;
    payload_len = 0;
    cmd_ready = false;

    /* Start next reception */
    expected_bytes = 1;
    HAL_SPI_TransmitReceive_IT(&hspi, tx_buffer, rx_buffer, expected_bytes);
}

void bl_spi_irq_handler(void)
{
    /* Nothing to do here - handled by HAL callback */
}

/* HAL SPI callbacks */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi_ptr)
{
    if (hspi_ptr->Instance == SPI1) {
        rx_count = expected_bytes;
        rx_complete = true;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi_ptr)
{
    if (hspi_ptr->Instance == SPI1) {
        /* Reset and restart */
        rx_count = 0;
        rx_complete = false;
        current_cmd = 0;
        expected_bytes = 1;
        HAL_SPI_TransmitReceive_IT(&hspi, tx_buffer, rx_buffer, expected_bytes);
    }
}

/* SPI1 IRQ Handler */
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi);
}
