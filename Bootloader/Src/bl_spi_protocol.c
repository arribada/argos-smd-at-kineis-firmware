/**
 * @file    bl_spi_protocol.c
 * @brief   SPI Protocol A+ implementation for bootloader
 * @date    2025
 */

#include "bl_spi_protocol.h"
#include "bl_dfu.h"
#include "bl_main.h"
#include <string.h>

/*******************************************************************************
 * PRIVATE VARIABLES
 ******************************************************************************/

static bl_spi_protocol_ctx_t protocol_ctx;

/*******************************************************************************
 * PUBLIC FUNCTIONS
 ******************************************************************************/

void bl_spi_protocol_init(void)
{
    memset(&protocol_ctx, 0, sizeof(protocol_ctx));
    protocol_ctx.mode = BL_PROT_MODE_UNKNOWN;
    protocol_ctx.op_state = BL_DFU_STATE_IDLE;
}

void bl_spi_protocol_reset(void)
{
    /* Preserve statistics and mode preference */
    bl_prot_mode_t saved_mode = protocol_ctx.mode;
    uint32_t saved_frames = protocol_ctx.frame_count;
    uint32_t saved_crc_errors = protocol_ctx.crc_error_count;
    uint32_t saved_seq_errors = protocol_ctx.seq_error_count;
    bl_dfu_op_state_t saved_op_state = protocol_ctx.op_state;

    memset(&protocol_ctx.request, 0, sizeof(protocol_ctx.request));

    protocol_ctx.mode = saved_mode;
    protocol_ctx.frame_count = saved_frames;
    protocol_ctx.crc_error_count = saved_crc_errors;
    protocol_ctx.seq_error_count = saved_seq_errors;
    protocol_ctx.op_state = saved_op_state;
}

uint8_t bl_spi_protocol_crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00;  /* Initial value */

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;  /* CRC-8-CCITT polynomial */
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

bool bl_spi_protocol_is_valid_dfu_cmd(uint8_t cmd)
{
    /* DFU commands are in range 0x30-0x3F */
    return (cmd >= SPI_CMD_DFU_BASE && cmd <= SPI_CMD_DFU_ENTER);
}

bool bl_spi_protocol_process(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0) {
        return false;
    }

    /* Reset request state */
    memset(&protocol_ctx.request, 0, sizeof(protocol_ctx.request));

    uint8_t first_byte = data[0];

    /* Check for A+ protocol magic byte */
    if (first_byte == BL_SPI_MAGIC_REQUEST) {
        protocol_ctx.mode = BL_PROT_MODE_A_PLUS;

        /* Need at least header + CRC */
        if (length < BL_SPI_FRAME_MIN_SIZE) {
            return false;
        }

        /* Parse header */
        protocol_ctx.request.sequence = data[BL_SPI_FRAME_SEQ_OFFSET];
        protocol_ctx.request.command = data[BL_SPI_FRAME_CMD_OFFSET];
        protocol_ctx.request.data_len = data[BL_SPI_FRAME_LEN_OFFSET];

        /* Validate data length */
        if (protocol_ctx.request.data_len > BL_SPI_FRAME_MAX_DATA) {
            return false;
        }

        /* Check if we have complete frame */
        uint16_t expected_len = BL_SPI_FRAME_HEADER_SIZE +
                                protocol_ctx.request.data_len +
                                BL_SPI_FRAME_CRC_SIZE;
        if (length < expected_len) {
            return false;
        }

        /* Copy data payload */
        if (protocol_ctx.request.data_len > 0) {
            memcpy(protocol_ctx.request.data,
                   &data[BL_SPI_FRAME_DATA_OFFSET],
                   protocol_ctx.request.data_len);
        }

        /* Get received CRC */
        uint16_t crc_offset = BL_SPI_FRAME_HEADER_SIZE + protocol_ctx.request.data_len;
        protocol_ctx.request.received_crc = data[crc_offset];

        /* Compute CRC over MAGIC + SEQ + CMD + LEN + DATA */
        uint16_t crc_len = BL_SPI_FRAME_HEADER_SIZE + protocol_ctx.request.data_len;
        protocol_ctx.request.computed_crc = bl_spi_protocol_crc8(data, crc_len);

        protocol_ctx.request.crc_valid =
            (protocol_ctx.request.received_crc == protocol_ctx.request.computed_crc);

        if (!protocol_ctx.request.crc_valid) {
            protocol_ctx.crc_error_count++;
        }

        /* Check sequence (should increment or be 0 for first frame) */
        if (protocol_ctx.frame_count > 0) {
            uint8_t expected_seq = (protocol_ctx.last_sequence + 1) & 0xFF;
            if (protocol_ctx.request.sequence != expected_seq &&
                protocol_ctx.request.sequence != 0) {
                protocol_ctx.seq_error_count++;
            }
        }
        protocol_ctx.last_sequence = protocol_ctx.request.sequence;

        protocol_ctx.frame_count++;
        return true;

    } else if (bl_spi_protocol_is_valid_dfu_cmd(first_byte)) {
        /* Legacy protocol - single command byte */
        protocol_ctx.mode = BL_PROT_MODE_LEGACY;
        protocol_ctx.request.command = first_byte;
        protocol_ctx.request.sequence = 0;
        protocol_ctx.request.crc_valid = true;  /* Legacy has no CRC */

        /* For legacy, remaining bytes are payload */
        if (length > 1) {
            protocol_ctx.request.data_len = (length - 1 > BL_SPI_FRAME_MAX_DATA) ?
                                             BL_SPI_FRAME_MAX_DATA : (length - 1);
            memcpy(protocol_ctx.request.data, &data[1], protocol_ctx.request.data_len);
        }

        protocol_ctx.frame_count++;
        return true;

    } else if (first_byte == 0xFF || first_byte == 0x00) {
        /* Sync pattern or empty - not an error, just no command */
        return false;

    } else {
        /* Invalid first byte */
        return false;
    }
}

bool bl_spi_protocol_is_legacy(void)
{
    return (protocol_ctx.mode == BL_PROT_MODE_LEGACY);
}

bl_spi_request_t* bl_spi_protocol_get_request(void)
{
    return &protocol_ctx.request;
}

uint8_t bl_spi_protocol_get_command(void)
{
    return protocol_ctx.request.command;
}

bl_spi_protocol_ctx_t* bl_spi_protocol_get_ctx(void)
{
    return &protocol_ctx;
}

uint16_t bl_spi_protocol_build_response(uint8_t *tx_buf, uint8_t sequence,
                                         bl_prot_status_t status,
                                         const uint8_t *data, uint8_t data_len)
{
    uint16_t index = 0;

    /* Build frame header */
    tx_buf[index++] = BL_SPI_MAGIC_RESPONSE;
    tx_buf[index++] = sequence;
    tx_buf[index++] = (uint8_t)status;
    tx_buf[index++] = data_len;

    /* Copy data if any */
    if (data_len > 0 && data != NULL) {
        memcpy(&tx_buf[index], data, data_len);
        index += data_len;
    }

    /* Compute and append CRC */
    uint8_t crc = bl_spi_protocol_crc8(tx_buf, index);
    tx_buf[index++] = crc;

    /* Pad with idle pattern for master over-read */
    while (index < 16) {
        tx_buf[index++] = BL_SPI_IDLE_PATTERN;
    }

    return index;
}

uint16_t bl_spi_protocol_build_legacy_response(uint8_t *tx_buf,
                                                bl_prot_status_t status,
                                                const uint8_t *data, uint8_t data_len)
{
    uint16_t index = 0;

    /* Status byte */
    tx_buf[index++] = (uint8_t)status;

    /* Data if any */
    if (data_len > 0 && data != NULL) {
        memcpy(&tx_buf[index], data, data_len);
        index += data_len;
    }

    /* Pad with idle pattern */
    while (index < 16) {
        tx_buf[index++] = BL_SPI_IDLE_PATTERN;
    }

    return index;
}

void bl_spi_protocol_set_op_state(bl_dfu_op_state_t state)
{
    protocol_ctx.op_state = state;
}

bl_dfu_op_state_t bl_spi_protocol_get_op_state(void)
{
    return protocol_ctx.op_state;
}

void bl_spi_protocol_build_extended_status(bl_extended_status_t *status)
{
    if (status == NULL) {
        return;
    }

    const dfu_context_t *dfu_ctx = bl_dfu_get_context();

    memset(status, 0, sizeof(bl_extended_status_t));

    /* Protocol info */
    status->protocol_version = 0x01;  /* A+ v1 */
    status->bootloader_state = (uint8_t)bl_get_state();
    status->dfu_op_state = (uint8_t)protocol_ctx.op_state;
    status->last_error = (uint8_t)dfu_ctx->last_error;

    /* DFU session state */
    status->session_active = dfu_ctx->session_active ? 1 : 0;
    status->erase_done = dfu_ctx->erase_done ? 1 : 0;
    status->verify_passed = dfu_ctx->verify_passed ? 1 : 0;

    /* Progress info */
    status->received_bytes = dfu_ctx->received_size;
    status->write_address = dfu_ctx->write_addr;
    status->expected_crc = dfu_ctx->expected_crc;
    status->calculated_crc = dfu_ctx->calculated_crc;

    /* Protocol statistics */
    status->frame_count = protocol_ctx.frame_count;
    status->crc_error_count = protocol_ctx.crc_error_count;
}
