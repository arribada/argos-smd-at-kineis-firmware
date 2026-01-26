/**
 * @file    bl_dfu.c
 * @brief   DFU protocol handler implementation
 * @date    2025
 */

#include "bl_dfu.h"
#include "bl_flash.h"
#include "bl_crc.h"
#include "bl_main.h"
#include "bl_app_header.h"
#include <string.h>
#include <stdio.h>

/* DFU context */
static dfu_context_t dfu_ctx;

void bl_dfu_init(void)
{
    bl_dfu_reset();
}

void bl_dfu_reset(void)
{
    memset(&dfu_ctx, 0, sizeof(dfu_ctx));
    dfu_ctx.write_addr = APP_HEADER_ADDR;
    dfu_ctx.last_error = DFU_RSP_OK;
}

dfu_response_t bl_dfu_process_cmd(dfu_cmd_t cmd, const uint8_t* data,
                                   uint16_t data_len, uint8_t* response,
                                   uint16_t* response_len)
{
    dfu_response_t status = DFU_RSP_OK;

    switch (cmd) {
        case DFU_CMD_PING:
            status = bl_dfu_cmd_ping(response, response_len);
            break;

        case DFU_CMD_GET_INFO:
            status = bl_dfu_cmd_get_info(response, response_len);
            break;

        case DFU_CMD_ERASE:
            status = bl_dfu_cmd_erase();
            *response_len = 0;
            break;

        case DFU_CMD_WRITE:
            status = bl_dfu_cmd_write(data, data_len);
            *response_len = 0;
            break;

        case DFU_CMD_READ:
            status = bl_dfu_cmd_read(data, data_len, response, response_len);
            break;

        case DFU_CMD_VERIFY:
            status = bl_dfu_cmd_verify(data, data_len);
            *response_len = 0;
            break;

        case DFU_CMD_GET_STATUS:
            status = bl_dfu_cmd_get_status(response, response_len);
            break;

        case DFU_CMD_SET_HEADER:
            status = bl_dfu_cmd_set_header(data, data_len);
            *response_len = 0;
            break;

        case DFU_CMD_ABORT:
            status = bl_dfu_cmd_abort();
            *response_len = 0;
            break;

        case DFU_CMD_RESET:
        case DFU_CMD_JUMP:
            /* These are handled by the caller */
            *response_len = 0;
            break;

        default:
            status = DFU_RSP_INVALID_CMD;
            *response_len = 0;
            break;
    }

    dfu_ctx.last_error = status;
    return status;
}

dfu_response_t bl_dfu_cmd_ping(uint8_t* response, uint16_t* response_len)
{
    /* Return bootloader version string */
    const char* version = bl_get_version_string();
    uint16_t len = strlen(version);

    if (*response_len < len) {
        return DFU_RSP_SIZE_ERROR;
    }

    memcpy(response, version, len);
    *response_len = len;

    return DFU_RSP_OK;
}

dfu_response_t bl_dfu_cmd_get_info(uint8_t* response, uint16_t* response_len)
{
    /* Build info response:
     * - BL version (4 bytes)
     * - App header valid flag (1 byte)
     * - App version (4 bytes) if valid
     * - App CRC (4 bytes) if valid
     * - Flash size available (4 bytes)
     */
    uint8_t* ptr = response;
    uint32_t bl_ver = bl_get_version();
    const app_header_t* header = bl_get_app_header();
    bool app_valid = bl_check_app_valid();

    if (*response_len < 17) {
        return DFU_RSP_SIZE_ERROR;
    }

    /* BL version */
    memcpy(ptr, &bl_ver, 4);
    ptr += 4;

    /* App valid flag */
    *ptr++ = app_valid ? 1 : 0;

    /* App version */
    if (app_valid && header) {
        memcpy(ptr, &header->app_version, 4);
    } else {
        memset(ptr, 0, 4);
    }
    ptr += 4;

    /* App CRC */
    if (app_valid && header) {
        memcpy(ptr, &header->app_crc32, 4);
    } else {
        memset(ptr, 0, 4);
    }
    ptr += 4;

    /* Flash size available */
    uint32_t flash_avail = APP_FLASH_SIZE;
    memcpy(ptr, &flash_avail, 4);
    ptr += 4;

    *response_len = (uint16_t)(ptr - response);

    return DFU_RSP_OK;
}

dfu_response_t bl_dfu_cmd_erase(void)
{
    bl_flash_status_t status;

    /* Erase application flash region */
    status = bl_flash_erase_app();

    if (status != BL_FLASH_OK) {
        return DFU_RSP_FLASH_ERROR;
    }

    /* Reset DFU context for new transfer */
    bl_dfu_reset();
    dfu_ctx.erase_done = true;

    /* Reset CRC calculation */
    bl_crc_reset();

    return DFU_RSP_OK;
}

dfu_response_t bl_dfu_cmd_write(const uint8_t* data, uint16_t data_len)
{
    bl_flash_status_t status;
    uint32_t address;
    const uint8_t* payload;
    uint16_t payload_len;

    /* Check if erase was done */
    if (!dfu_ctx.erase_done) {
        return DFU_RSP_NOT_READY;
    }

    /* Parse address from data */
    if (data_len < 5) { /* 4 bytes address + at least 1 byte data */
        return DFU_RSP_SIZE_ERROR;
    }

    memcpy(&address, data, 4);
    payload = data + 4;
    payload_len = data_len - 4;

    /* Validate address */
    if (!bl_flash_addr_in_app(address)) {
        return DFU_RSP_ADDR_ERROR;
    }

    /* Check alignment (flash requires 64-bit alignment) */
    if ((address % 8) != 0) {
        return DFU_RSP_ADDR_ERROR;
    }

    /* Pad payload to 8-byte boundary if needed */
    uint8_t aligned_data[BL_CHUNK_SIZE + 8];
    uint16_t aligned_len = payload_len;

    memcpy(aligned_data, payload, payload_len);

    if ((payload_len % 8) != 0) {
        aligned_len = ((payload_len / 8) + 1) * 8;
        memset(aligned_data + payload_len, 0xFF, aligned_len - payload_len);
    }

    /* Write data to flash */
    status = bl_flash_write(address, aligned_data, aligned_len);

    if (status != BL_FLASH_OK) {
        return DFU_RSP_FLASH_ERROR;
    }

    /* Verify write */
    if (!bl_flash_verify(address, aligned_data, aligned_len)) {
        return DFU_RSP_VERIFY_ERROR;
    }

    /* Update context */
    dfu_ctx.received_size += payload_len;
    dfu_ctx.write_addr = address + aligned_len;

    /* Accumulate CRC */
    bl_crc32_accumulate(payload, payload_len);

    return DFU_RSP_OK;
}

dfu_response_t bl_dfu_cmd_read(const uint8_t* data, uint16_t data_len,
                               uint8_t* response, uint16_t* response_len)
{
    uint32_t address;
    uint16_t read_len;

    /* Parse address and length */
    if (data_len < 6) { /* 4 bytes address + 2 bytes length */
        return DFU_RSP_SIZE_ERROR;
    }

    memcpy(&address, data, 4);
    memcpy(&read_len, data + 4, 2);

    /* Validate */
    if (read_len > *response_len) {
        read_len = *response_len;
    }

    /* Read from flash */
    bl_flash_status_t status = bl_flash_read(address, response, read_len);

    if (status != BL_FLASH_OK) {
        return DFU_RSP_FLASH_ERROR;
    }

    *response_len = read_len;

    return DFU_RSP_OK;
}

dfu_response_t bl_dfu_cmd_verify(const uint8_t* data, uint16_t data_len)
{
    uint32_t expected_crc;

    if (data_len < 4) {
        return DFU_RSP_SIZE_ERROR;
    }

    memcpy(&expected_crc, data, 4);
    dfu_ctx.expected_crc = expected_crc;

    /* Get calculated CRC */
    dfu_ctx.calculated_crc = bl_crc32_get();

    /* Compare CRCs */
    if (dfu_ctx.calculated_crc != dfu_ctx.expected_crc) {
        return DFU_RSP_CRC_ERROR;
    }

    return DFU_RSP_OK;
}

dfu_response_t bl_dfu_cmd_get_status(uint8_t* response, uint16_t* response_len)
{
    /* Build status response:
     * - Current state (1 byte)
     * - Bytes received (4 bytes)
     * - Current write address (4 bytes)
     * - Last error (1 byte)
     * - Erase done flag (1 byte)
     */
    uint8_t* ptr = response;

    if (*response_len < 11) {
        return DFU_RSP_SIZE_ERROR;
    }

    /* State */
    *ptr++ = (uint8_t)bl_get_state();

    /* Received size */
    memcpy(ptr, &dfu_ctx.received_size, 4);
    ptr += 4;

    /* Write address */
    memcpy(ptr, &dfu_ctx.write_addr, 4);
    ptr += 4;

    /* Last error */
    *ptr++ = (uint8_t)dfu_ctx.last_error;

    /* Erase done */
    *ptr++ = dfu_ctx.erase_done ? 1 : 0;

    *response_len = (uint16_t)(ptr - response);

    return DFU_RSP_OK;
}

dfu_response_t bl_dfu_cmd_set_header(const uint8_t* data, uint16_t data_len)
{
    bl_flash_status_t status;

    /* Validate header size */
    if (data_len != sizeof(app_header_t)) {
        return DFU_RSP_SIZE_ERROR;
    }

    /* Validate header content */
    const app_header_t* header = (const app_header_t*)data;

    if (!bl_header_magic_valid(header)) {
        return DFU_RSP_INVALID_HEADER;
    }

    /* Check if header page needs erasing (if not already done) */
    if (!dfu_ctx.erase_done) {
        /* Erase just the header page */
        status = bl_flash_unlock();
        if (status != BL_FLASH_OK) {
            return DFU_RSP_FLASH_ERROR;
        }

        status = bl_flash_erase_page(APP_HEADER_ADDR);
        bl_flash_lock();

        if (status != BL_FLASH_OK) {
            return DFU_RSP_FLASH_ERROR;
        }
    }

    /* Write header */
    status = bl_flash_write(APP_HEADER_ADDR, data, data_len);

    if (status != BL_FLASH_OK) {
        return DFU_RSP_FLASH_ERROR;
    }

    /* Verify */
    if (!bl_flash_verify(APP_HEADER_ADDR, data, data_len)) {
        return DFU_RSP_VERIFY_ERROR;
    }

    dfu_ctx.header_written = true;

    return DFU_RSP_OK;
}

dfu_response_t bl_dfu_cmd_abort(void)
{
    /* Reset DFU context */
    bl_dfu_reset();

    return DFU_RSP_OK;
}

const dfu_context_t* bl_dfu_get_context(void)
{
    return &dfu_ctx;
}

bool bl_dfu_is_complete(void)
{
    /* Transfer is complete if:
     * - Erase was done
     * - CRC was verified successfully
     */
    return dfu_ctx.erase_done &&
           (dfu_ctx.calculated_crc == dfu_ctx.expected_crc) &&
           (dfu_ctx.expected_crc != 0);
}
