/**
 * @file    bl_uart.h
 * @brief   UART communication driver for bootloader DFU
 * @date    2025
 */

#ifndef BL_UART_H
#define BL_UART_H

#include <stdint.h>
#include <stdbool.h>
#include "bl_config.h"

/**
 * @brief Initialize UART for bootloader
 * @return true if successful
 */
bool bl_uart_init(void);

/**
 * @brief Deinitialize UART
 */
void bl_uart_deinit(void);

/**
 * @brief Start UART reception (enable interrupts)
 */
void bl_uart_start_rx(void);

/**
 * @brief Stop UART reception
 */
void bl_uart_stop_rx(void);

/**
 * @brief Check if UART has received data
 * @return true if data available
 */
bool bl_uart_has_data(void);

/**
 * @brief Get number of bytes available in RX buffer
 * @return Number of bytes
 */
uint16_t bl_uart_rx_available(void);

/**
 * @brief Read data from UART RX buffer
 * @param buffer Destination buffer
 * @param max_len Maximum bytes to read
 * @return Number of bytes read
 */
uint16_t bl_uart_read(uint8_t* buffer, uint16_t max_len);

/**
 * @brief Read a single byte from UART
 * @param byte Pointer to store byte
 * @return true if byte was read
 */
bool bl_uart_read_byte(uint8_t* byte);

/**
 * @brief Transmit data via UART (blocking)
 * @param data Data to send
 * @param len Data length
 * @return true if successful
 */
bool bl_uart_transmit(const uint8_t* data, uint16_t len);

/**
 * @brief Transmit a null-terminated string
 * @param str String to send
 * @return true if successful
 */
bool bl_uart_print(const char* str);

/**
 * @brief Flush UART RX buffer
 */
void bl_uart_flush_rx(void);

/**
 * @brief Process UART data and parse AT commands
 * @return true if a complete command was received
 */
bool bl_uart_process(void);

/**
 * @brief Get the last received AT command
 * @param cmd_buffer Buffer to store command
 * @param max_len Maximum buffer length
 * @return Length of command, 0 if none
 */
uint16_t bl_uart_get_command(char* cmd_buffer, uint16_t max_len);

/**
 * @brief Send DFU response via UART
 * @param status Response status code
 * @param data Optional response data
 * @param data_len Data length
 */
void bl_uart_send_response(dfu_response_t status, const uint8_t* data, uint16_t data_len);

/**
 * @brief UART RX interrupt handler (called from IRQ)
 */
void bl_uart_rx_irq_handler(void);

/**
 * @brief Get RX interrupt count (for debugging)
 * @return Number of RX interrupts triggered
 */
uint32_t bl_uart_get_irq_count(void);

/**
 * @brief Get HAL UART RX state (for debugging)
 * @return HAL RxState value
 */
uint32_t huart_state_debug(void);

#endif /* BL_UART_H */
