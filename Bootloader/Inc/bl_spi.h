/**
 * @file    bl_spi.h
 * @brief   SPI communication driver for bootloader DFU
 * @date    2025
 */

#ifndef BL_SPI_H
#define BL_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "bl_config.h"

/**
 * @brief Initialize SPI for bootloader (slave mode)
 * @return true if successful
 */
bool bl_spi_init(void);

/**
 * @brief Deinitialize SPI
 */
void bl_spi_deinit(void);

/**
 * @brief Start SPI reception (enable interrupts)
 */
void bl_spi_start_rx(void);

/**
 * @brief Stop SPI reception
 */
void bl_spi_stop_rx(void);

/**
 * @brief Check if SPI has received data
 * @return true if data available
 */
bool bl_spi_has_data(void);

/**
 * @brief Get number of bytes available in RX buffer
 * @return Number of bytes
 */
uint16_t bl_spi_rx_available(void);

/**
 * @brief Read data from SPI RX buffer
 * @param buffer Destination buffer
 * @param max_len Maximum bytes to read
 * @return Number of bytes read
 */
uint16_t bl_spi_read(uint8_t* buffer, uint16_t max_len);

/**
 * @brief Read a single byte from SPI
 * @param byte Pointer to store byte
 * @return true if byte was read
 */
bool bl_spi_read_byte(uint8_t* byte);

/**
 * @brief Prepare data for next SPI transaction (slave TX)
 * @param data Data to send
 * @param len Data length
 * @return true if successful
 */
bool bl_spi_prepare_tx(const uint8_t* data, uint16_t len);

/**
 * @brief Flush SPI RX buffer
 */
void bl_spi_flush_rx(void);

/**
 * @brief Process SPI data and handle commands
 * @return true if a command was processed
 */
bool bl_spi_process(void);

/**
 * @brief Get the current SPI command byte
 * @return Command byte, or 0 if none
 */
uint8_t bl_spi_get_command(void);

/**
 * @brief Get command payload data
 * @param buffer Buffer to store payload
 * @param max_len Maximum buffer length
 * @return Length of payload
 */
uint16_t bl_spi_get_payload(uint8_t* buffer, uint16_t max_len);

/**
 * @brief Send DFU response via SPI
 * @param status Response status code
 * @param data Optional response data
 * @param data_len Data length
 */
void bl_spi_send_response(dfu_response_t status, const uint8_t* data, uint16_t data_len);

/**
 * @brief SPI interrupt handler (called from IRQ)
 */
void bl_spi_irq_handler(void);

#endif /* BL_SPI_H */
