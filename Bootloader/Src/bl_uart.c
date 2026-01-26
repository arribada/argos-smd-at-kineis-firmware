/**
 * @file    bl_uart.c
 * @brief   UART communication driver for bootloader DFU
 * @date    2025
 */

#include "bl_uart.h"
#include "bl_config.h"
#include "stm32wlxx_hal.h"
#include <string.h>
#include <stdio.h>

/* UART handle */
static UART_HandleTypeDef huart;

/* RX buffer and state */
static uint8_t rx_buffer[BL_RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;
static volatile bool rx_overflow = false;

/* TX buffer */
static uint8_t tx_buffer[BL_TX_BUFFER_SIZE];

/* Command parsing state */
static char cmd_buffer[128];
static uint16_t cmd_len = 0;
static volatile bool cmd_ready = false;

/* Single byte for interrupt reception */
static uint8_t rx_byte;

bool bl_uart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Enable clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_LPUART1_CLK_ENABLE();

    /* Configure LPUART1 GPIO: PA2 (TX), PA3 (RX) */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_LPUART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Configure LPUART1 */
    huart.Instance = LPUART1;
    huart.Init.BaudRate = BL_UART_BAUDRATE;
    huart.Init.WordLength = BL_UART_WORDLENGTH;
    huart.Init.StopBits = BL_UART_STOPBITS;
    huart.Init.Parity = BL_UART_PARITY;
    huart.Init.Mode = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart) != HAL_OK) {
        return false;
    }

    /* Configure NVIC for LPUART1 */
    HAL_NVIC_SetPriority(LPUART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);

    /* Reset buffers */
    rx_head = 0;
    rx_tail = 0;
    rx_overflow = false;
    cmd_len = 0;
    cmd_ready = false;

    return true;
}

void bl_uart_deinit(void)
{
    HAL_NVIC_DisableIRQ(LPUART1_IRQn);
    HAL_UART_DeInit(&huart);

    /* Reset GPIO to analog */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
}

void bl_uart_start_rx(void)
{
    /* Start interrupt-based reception */
    HAL_UART_Receive_IT(&huart, &rx_byte, 1);
}

void bl_uart_stop_rx(void)
{
    HAL_UART_AbortReceive(&huart);
}

bool bl_uart_has_data(void)
{
    return (rx_head != rx_tail);
}

uint16_t bl_uart_rx_available(void)
{
    if (rx_head >= rx_tail) {
        return rx_head - rx_tail;
    } else {
        return BL_RX_BUFFER_SIZE - rx_tail + rx_head;
    }
}

uint16_t bl_uart_read(uint8_t* buffer, uint16_t max_len)
{
    uint16_t count = 0;

    while (count < max_len && rx_tail != rx_head) {
        buffer[count++] = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % BL_RX_BUFFER_SIZE;
    }

    return count;
}

bool bl_uart_read_byte(uint8_t* byte)
{
    if (rx_tail == rx_head) {
        return false;
    }

    *byte = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % BL_RX_BUFFER_SIZE;
    return true;
}

bool bl_uart_transmit(const uint8_t* data, uint16_t len)
{
    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(&huart, (uint8_t*)data, len, 500);
    return (status == HAL_OK);
}

bool bl_uart_print(const char* str)
{
    return bl_uart_transmit((const uint8_t*)str, strlen(str));
}

void bl_uart_flush_rx(void)
{
    rx_head = 0;
    rx_tail = 0;
    rx_overflow = false;
}

bool bl_uart_process(void)
{
    /* Check if we already have a complete command */
    if (cmd_ready) {
        return true;
    }

    /* Process received bytes */
    while (rx_tail != rx_head) {
        uint8_t c = rx_buffer[rx_tail];
        rx_tail = (rx_tail + 1) % BL_RX_BUFFER_SIZE;

        /* Add to command buffer if not full */
        if (cmd_len < sizeof(cmd_buffer) - 1) {
            cmd_buffer[cmd_len++] = c;
        }

        /* Check for command terminator */
        if (c == '\n' || c == '\r') {
            /* Check if we have a valid AT+DFU command */
            cmd_buffer[cmd_len] = '\0';

            /* Remove trailing CR/LF */
            while (cmd_len > 0 && (cmd_buffer[cmd_len-1] == '\r' || cmd_buffer[cmd_len-1] == '\n')) {
                cmd_buffer[--cmd_len] = '\0';
            }

            /* Check for AT+DFU prefix */
            if (cmd_len >= 7 && strncmp(cmd_buffer, "AT+DFU", 6) == 0) {
                cmd_ready = true;
                return true;
            }

            /* Invalid command, reset */
            cmd_len = 0;
        }
    }

    return false;
}

uint16_t bl_uart_get_command(char* buffer, uint16_t max_len)
{
    if (!cmd_ready) {
        return 0;
    }

    uint16_t len = cmd_len;
    if (len >= max_len) {
        len = max_len - 1;
    }

    memcpy(buffer, cmd_buffer, len);
    buffer[len] = '\0';

    /* Reset command state */
    cmd_ready = false;
    cmd_len = 0;

    return len;
}

void bl_uart_send_response(dfu_response_t status, const uint8_t* data, uint16_t data_len)
{
    char response[BL_TX_BUFFER_SIZE];
    int len;

    if (status == DFU_RSP_OK) {
        if (data != NULL && data_len > 0) {
            /* Format: +DFU=OK,<data>\r\n */
            len = snprintf(response, sizeof(response), "+DFU=OK,");

            /* Check if data is printable string */
            bool is_string = true;
            for (uint16_t i = 0; i < data_len; i++) {
                if (data[i] < 0x20 || data[i] > 0x7E) {
                    is_string = false;
                    break;
                }
            }

            if (is_string) {
                memcpy(response + len, data, data_len);
                len += data_len;
            } else {
                /* Convert to hex */
                for (uint16_t i = 0; i < data_len && len < (int)sizeof(response) - 3; i++) {
                    len += snprintf(response + len, sizeof(response) - len, "%02X", data[i]);
                }
            }
            len += snprintf(response + len, sizeof(response) - len, "\r\n");
        } else {
            len = snprintf(response, sizeof(response), "+DFU=OK\r\n");
        }
    } else {
        /* Error response */
        const char* err_str;
        switch (status) {
            case DFU_RSP_CRC_ERROR:      err_str = "CRC_ERROR"; break;
            case DFU_RSP_ADDR_ERROR:     err_str = "ADDR_ERROR"; break;
            case DFU_RSP_SIZE_ERROR:     err_str = "SIZE_ERROR"; break;
            case DFU_RSP_FLASH_ERROR:    err_str = "FLASH_ERROR"; break;
            case DFU_RSP_BUSY:           err_str = "BUSY"; break;
            case DFU_RSP_INVALID_CMD:    err_str = "INVALID_CMD"; break;
            case DFU_RSP_TIMEOUT:        err_str = "TIMEOUT"; break;
            case DFU_RSP_NOT_READY:      err_str = "NOT_READY"; break;
            case DFU_RSP_INVALID_HEADER: err_str = "INVALID_HEADER"; break;
            case DFU_RSP_VERIFY_ERROR:   err_str = "VERIFY_ERROR"; break;
            default:                     err_str = "ERROR"; break;
        }
        len = snprintf(response, sizeof(response), "+DFU=ERR,%s\r\n", err_str);
    }

    bl_uart_transmit((uint8_t*)response, len);
}

void bl_uart_rx_irq_handler(void)
{
    /* Store received byte in circular buffer */
    uint16_t next_head = (rx_head + 1) % BL_RX_BUFFER_SIZE;

    if (next_head != rx_tail) {
        rx_buffer[rx_head] = rx_byte;
        rx_head = next_head;
    } else {
        rx_overflow = true;
    }

    /* Continue reception */
    HAL_UART_Receive_IT(&huart, &rx_byte, 1);
}

/* HAL UART callbacks */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart_ptr)
{
    if (huart_ptr->Instance == LPUART1) {
        bl_uart_rx_irq_handler();
    }
}

/* LPUART1 IRQ Handler */
void LPUART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart);
}
