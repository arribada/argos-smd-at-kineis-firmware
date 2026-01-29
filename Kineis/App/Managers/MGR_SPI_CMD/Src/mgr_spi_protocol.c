// SPDX-License-Identifier: no SPDX license
/**
 * @file   mgr_spi_protocol.c
 * @author Arribada
 * @brief  SPI Protocol A+ implementation
 *
 * This module implements frame parsing, CRC checking, and response building
 * for the A+ protocol. It also provides backward compatibility detection
 * for legacy protocol.
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "mgr_spi_protocol.h"
#include "mgr_spi_cmd.h"
#include "mgr_spi_cmd_list.h"
#include "mgr_log.h"

/* Private defines -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
SpiProtocolContext spi_protocol_ctx;

/** @brief Temporary buffer for header parsing */
static uint8_t header_buf[SPI_FRAME_HEADER_SIZE];
static uint8_t header_index = 0;

/* Private function prototypes -----------------------------------------------*/
static void protocol_parse_header(void);
static bool is_valid_legacy_command(uint8_t byte);

/* Functions -----------------------------------------------------------------*/

void MGR_SPI_PROTOCOL_init(void)
{
    memset(&spi_protocol_ctx, 0, sizeof(spi_protocol_ctx));
    spi_protocol_ctx.mode = SPI_PROTOCOL_UNKNOWN;
    spi_protocol_ctx.state = SPI_PROT_IDLE;
    header_index = 0;
    SPI_LOG_VERBOSE("%s:: Protocol initialized\r\n", __func__);
}

uint8_t MGR_SPI_PROTOCOL_crc8(const uint8_t *data, uint16_t length)
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

/**
 * @brief Check if byte is a valid legacy command
 */
static bool is_valid_legacy_command(uint8_t byte)
{
    /* Legacy commands are 0x01 to SPICMD_MAX_COUNT-1 */
    return (byte > CMD_NONE && byte < SPICMD_MAX_COUNT);
}

/**
 * @brief Parse received header bytes
 */
static void protocol_parse_header(void)
{
    spi_protocol_ctx.request.sequence = header_buf[SPI_FRAME_SEQ_OFFSET];
    spi_protocol_ctx.request.command = header_buf[SPI_FRAME_CMD_OFFSET];
    spi_protocol_ctx.request.data_len = header_buf[SPI_FRAME_LEN_OFFSET];

    SPI_LOG_VERBOSE("%s:: Header: SEQ=%u CMD=0x%02X LEN=%u\r\n",
                  __func__,
                  spi_protocol_ctx.request.sequence,
                  spi_protocol_ctx.request.command,
                  spi_protocol_ctx.request.data_len);
}

bool MGR_SPI_PROTOCOL_rx_byte(uint8_t byte)
{
    switch (spi_protocol_ctx.state) {
        case SPI_PROT_IDLE:
            header_index = 0;
            spi_protocol_ctx.rx_index = 0;

            if (byte == SPI_MAGIC_REQUEST) {
                /* A+ protocol detected */
                spi_protocol_ctx.mode = SPI_PROTOCOL_A_PLUS;
                spi_protocol_ctx.state = SPI_PROT_RX_HEADER;
                header_buf[header_index++] = byte;
                SPI_LOG_VERBOSE("%s:: A+ protocol detected\r\n", __func__);
            } else if (is_valid_legacy_command(byte)) {
                /* Legacy protocol detected */
                spi_protocol_ctx.mode = SPI_PROTOCOL_LEGACY;
                spi_protocol_ctx.request.command = byte;
                spi_protocol_ctx.state = SPI_PROT_FRAME_READY;
                spi_protocol_ctx.frame_count++;
                SPI_LOG_VERBOSE("%s:: Legacy cmd=0x%02X\r\n", __func__, byte);
                return true;
            } else {
                /* Invalid first byte */
                MGR_LOG_DEBUG("%s:: Invalid byte 0x%02X\r\n", __func__, byte);
                spi_protocol_ctx.state = SPI_PROT_ERROR;
            }
            break;

        case SPI_PROT_RX_HEADER:
            header_buf[header_index++] = byte;
            if (header_index >= SPI_FRAME_HEADER_SIZE) {
                protocol_parse_header();

                /* Validate data length */
                if (spi_protocol_ctx.request.data_len > SPI_FRAME_MAX_DATA) {
                    MGR_LOG_DEBUG("%s:: Invalid length %u\r\n", __func__,
                                  spi_protocol_ctx.request.data_len);
                    spi_protocol_ctx.state = SPI_PROT_ERROR;
                    return false;
                }

                if (spi_protocol_ctx.request.data_len > 0) {
                    spi_protocol_ctx.state = SPI_PROT_RX_DATA;
                    spi_protocol_ctx.rx_index = 0;
                } else {
                    /* No data, go directly to CRC */
                    spi_protocol_ctx.state = SPI_PROT_RX_CRC;
                }
            }
            break;

        case SPI_PROT_RX_DATA:
            if (spi_protocol_ctx.rx_index < SPI_FRAME_MAX_DATA) {
                spi_protocol_ctx.request.data[spi_protocol_ctx.rx_index++] = byte;
            }
            if (spi_protocol_ctx.rx_index >= spi_protocol_ctx.request.data_len) {
                spi_protocol_ctx.state = SPI_PROT_RX_CRC;
            }
            break;

        case SPI_PROT_RX_CRC:
            {
                spi_protocol_ctx.request.received_crc = byte;

                /* Compute CRC over MAGIC + SEQ + CMD + LEN + DATA (include magic byte) */
                uint16_t crc_len = 4 + spi_protocol_ctx.request.data_len;
                uint8_t crc_buf[SPI_FRAME_MAX_DATA + 4];
                crc_buf[0] = SPI_MAGIC_REQUEST;
                crc_buf[1] = spi_protocol_ctx.request.sequence;
                crc_buf[2] = spi_protocol_ctx.request.command;
                crc_buf[3] = spi_protocol_ctx.request.data_len;
                if (spi_protocol_ctx.request.data_len > 0) {
                    memcpy(&crc_buf[4], spi_protocol_ctx.request.data,
                           spi_protocol_ctx.request.data_len);
                }

                spi_protocol_ctx.request.computed_crc = MGR_SPI_PROTOCOL_crc8(crc_buf, crc_len);
                spi_protocol_ctx.request.crc_valid =
                    (spi_protocol_ctx.request.received_crc == spi_protocol_ctx.request.computed_crc);

                if (!spi_protocol_ctx.request.crc_valid) {
                    MGR_LOG_DEBUG("%s:: CRC error: rx=0x%02X calc=0x%02X\r\n",
                                  __func__,
                                  spi_protocol_ctx.request.received_crc,
                                  spi_protocol_ctx.request.computed_crc);
                    spi_protocol_ctx.crc_error_count++;
                }

                spi_protocol_ctx.state = SPI_PROT_FRAME_READY;
                spi_protocol_ctx.frame_count++;
                return true;
            }
            break;

        case SPI_PROT_FRAME_READY:
        case SPI_PROT_ERROR:
            /* Should not receive bytes in these states */
            MGR_LOG_DEBUG("%s:: Unexpected byte in state %d\r\n",
                          __func__, spi_protocol_ctx.state);
            break;

        default:
            spi_protocol_ctx.state = SPI_PROT_ERROR;
            break;
    }

    return false;
}

bool MGR_SPI_PROTOCOL_is_legacy(void)
{
    return (spi_protocol_ctx.mode == SPI_PROTOCOL_LEGACY);
}

uint8_t MGR_SPI_PROTOCOL_get_legacy_cmd(void)
{
    return spi_protocol_ctx.request.command;
}

SpiRequestFrame* MGR_SPI_PROTOCOL_get_request(void)
{
    return &spi_protocol_ctx.request;
}

uint16_t MGR_SPI_PROTOCOL_build_response(uint8_t *tx_buf, uint8_t sequence,
                                          SpiProtocolStatus status,
                                          const uint8_t *data, uint8_t data_len)
{
    uint16_t index = 0;

    /* Build frame header */
    tx_buf[index++] = SPI_MAGIC_RESPONSE;
    tx_buf[index++] = sequence;
    tx_buf[index++] = (uint8_t)status;
    tx_buf[index++] = data_len;

    /* Copy data if any */
    if (data_len > 0 && data != NULL) {
        memcpy(&tx_buf[index], data, data_len);
        index += data_len;
    }

    /* Compute and append CRC (over MAGIC + SEQ + STATUS + LEN + DATA) */
    uint8_t crc = MGR_SPI_PROTOCOL_crc8(&tx_buf[0], 4 + data_len);
    tx_buf[index++] = crc;

    SPI_LOG_VERBOSE("%s:: Response: SEQ=%u STATUS=%u LEN=%u CRC=0x%02X\r\n",
                  __func__, sequence, status, data_len, crc);

    return index;
}

uint16_t MGR_SPI_PROTOCOL_build_error_response(uint8_t *tx_buf, uint8_t sequence,
                                                SpiProtocolStatus status)
{
    return MGR_SPI_PROTOCOL_build_response(tx_buf, sequence, status, NULL, 0);
}

void MGR_SPI_PROTOCOL_reset(void)
{
    spi_protocol_ctx.mode = SPI_PROTOCOL_UNKNOWN;
    spi_protocol_ctx.state = SPI_PROT_IDLE;
    header_index = 0;
    spi_protocol_ctx.rx_index = 0;
    /* Preserve statistics */
    SPI_LOG_VERBOSE("%s:: Protocol reset\r\n", __func__);
}

int MGR_SPI_PROTOCOL_get_stats(char *buf, uint16_t buf_size)
{
    return snprintf(buf, buf_size,
                    "SPI Protocol Stats:\r\n"
                    "  Frames: %lu\r\n"
                    "  CRC errors: %lu\r\n"
                    "  SEQ errors: %lu\r\n"
                    "  Mode: %s\r\n",
                    (unsigned long)spi_protocol_ctx.frame_count,
                    (unsigned long)spi_protocol_ctx.crc_error_count,
                    (unsigned long)spi_protocol_ctx.seq_error_count,
                    spi_protocol_ctx.mode == SPI_PROTOCOL_A_PLUS ? "A+" :
                    spi_protocol_ctx.mode == SPI_PROTOCOL_LEGACY ? "Legacy" : "Unknown");
}

bool MGR_SPI_PROTOCOL_process_buffer(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0) {
        MGR_LOG_DEBUG("%s:: Invalid input\r\n", __func__);
        return false;
    }

    /* Reset state for new frame */
    spi_protocol_ctx.state = SPI_PROT_IDLE;
    header_index = 0;
    spi_protocol_ctx.rx_index = 0;

    uint8_t first_byte = data[0];

    SPI_LOG_VERBOSE("%s:: Processing %u bytes, first=0x%02X\r\n", __func__, length, first_byte);

    if (first_byte == SPI_MAGIC_REQUEST) {
        /* A+ protocol frame */
        spi_protocol_ctx.mode = SPI_PROTOCOL_A_PLUS;

        /* Need at least header + CRC */
        if (length < SPI_FRAME_MIN_SIZE) {
            MGR_LOG_DEBUG("%s:: A+ frame too short (%u < %u)\r\n",
                          __func__, length, SPI_FRAME_MIN_SIZE);
            spi_protocol_ctx.state = SPI_PROT_ERROR;
            return false;
        }

        /* Parse header: Magic(0) + Seq(1) + Cmd(2) + Len(3) */
        spi_protocol_ctx.request.sequence = data[SPI_FRAME_SEQ_OFFSET];
        spi_protocol_ctx.request.command = data[SPI_FRAME_CMD_OFFSET];
        spi_protocol_ctx.request.data_len = data[SPI_FRAME_LEN_OFFSET];

        SPI_LOG_VERBOSE("A+ S=%u C=0x%02X L=%u\r\n",
                      spi_protocol_ctx.request.sequence,
                      spi_protocol_ctx.request.command,
                      spi_protocol_ctx.request.data_len);

        /* Validate data length */
        if (spi_protocol_ctx.request.data_len > SPI_FRAME_MAX_DATA) {
            MGR_LOG_DEBUG("%s:: Invalid data length %u\r\n",
                          __func__, spi_protocol_ctx.request.data_len);
            spi_protocol_ctx.state = SPI_PROT_ERROR;
            return false;
        }

        /* Check if we have enough bytes for data + CRC */
        uint16_t expected_len = SPI_FRAME_HEADER_SIZE + spi_protocol_ctx.request.data_len + SPI_FRAME_CRC_SIZE;
        if (length < expected_len) {
            MGR_LOG_DEBUG("%s:: A+ frame incomplete (%u < %u)\r\n",
                          __func__, length, expected_len);
            spi_protocol_ctx.state = SPI_PROT_ERROR;
            return false;
        }

        /* Copy data payload if present */
        if (spi_protocol_ctx.request.data_len > 0) {
            memcpy(spi_protocol_ctx.request.data,
                   &data[SPI_FRAME_DATA_OFFSET],
                   spi_protocol_ctx.request.data_len);
        }

        /* Get received CRC (last byte of frame) */
        uint16_t crc_offset = SPI_FRAME_HEADER_SIZE + spi_protocol_ctx.request.data_len;
        spi_protocol_ctx.request.received_crc = data[crc_offset];

        /* Compute CRC over MAGIC + SEQ + CMD + LEN + DATA (include magic byte) */
        uint16_t crc_len = 4 + spi_protocol_ctx.request.data_len;
        spi_protocol_ctx.request.computed_crc = MGR_SPI_PROTOCOL_crc8(
            &data[SPI_FRAME_MAGIC_OFFSET], crc_len);

        spi_protocol_ctx.request.crc_valid =
            (spi_protocol_ctx.request.received_crc == spi_protocol_ctx.request.computed_crc);

        /* Debug: show CRC calculation details */
        SPI_LOG_VERBOSE("CRC: rxd=0x%02X calc=0x%02X bytes=[%02X %02X %02X %02X]\r\n",
                      spi_protocol_ctx.request.received_crc,
                      spi_protocol_ctx.request.computed_crc,
                      data[0], data[1], data[2], data[3]);

        if (!spi_protocol_ctx.request.crc_valid) {
            spi_protocol_ctx.crc_error_count++;
        }

        spi_protocol_ctx.state = SPI_PROT_FRAME_READY;
        spi_protocol_ctx.frame_count++;

        SPI_LOG_VERBOSE("%s:: A+ frame complete, CRC %s\r\n",
                      __func__, spi_protocol_ctx.request.crc_valid ? "OK" : "ERROR");
        return true;

    } else if (is_valid_legacy_command(first_byte)) {
        /* Legacy protocol - single command byte */
        spi_protocol_ctx.mode = SPI_PROTOCOL_LEGACY;
        spi_protocol_ctx.request.command = first_byte;
        spi_protocol_ctx.request.data_len = 0;
        spi_protocol_ctx.request.crc_valid = true; /* Legacy has no CRC */
        spi_protocol_ctx.state = SPI_PROT_FRAME_READY;
        spi_protocol_ctx.frame_count++;

        SPI_LOG_VERBOSE("%s:: Legacy cmd=0x%02X\r\n", __func__, first_byte);
        return true;

    } else {
        /* Invalid first byte */
        MGR_LOG_DEBUG("%s:: Invalid first byte 0x%02X\r\n", __func__, first_byte);
        spi_protocol_ctx.state = SPI_PROT_ERROR;
        return false;
    }
}

/**
 * @}
 */
