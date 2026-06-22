/* SPDX-License-Identifier: no SPDX license */
/**
 * @file mgr_spi_cmd_conf.h
 * @brief SPI commands list header file.
 *
 * This file declares the list of available SPI commands and their associated parameters.
 * It includes definitions for command identifiers, the function pointer type used to process
 * each command, and a descriptor structure that holds the necessary information for handling
 * each SPI command.
 *
 * The available SPI commands allow for reading/writing various parameters such as firmware version,
 * configuration settings, device addresses, IDs, security keys, and more.
 *
 * @author Arribada
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MGR_SPI_CMD_CONF_H
#define __MGR_SPI_CMD_CONF_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "mcu_spi_driver.h"
#include "mcu_aes.h"
#include "mcu_nvm.h"

/* Defines -------------------------------------------------------------------*/
#define CMD_VARIABLE_LEN          0xFF   /**< Indicator for variable length command. */
#define CMD_WRITEID_WAIT_LEN      5      /**< 4 bytes for uint32 size + 1 byte for command. */
#define CMD_WRITETCXO_WAIT_LEN    5      /**< 4 bytes for uint32 size + 1 byte for command. */
#define CMD_WRITEADDRESS_WAIT_LEN (DEVICE_ADDR_LENGTH + 1) /**< Device address length + 1 byte for command. */
#define CMD_WRITESECKEY_WAIT_LEN  (DSK_BYTE_LENGTH + 1)      /**< Secret key length + 1 byte for command. */
#define CMD_WRITERCONF_WAIT_LEN   17     /**< 32 bytes for configuration size + 1 byte for command. */
#define CMD_WRITELPM_WAIT_LEN     2      /**< 1 byte for low power mode + 1 byte for command. */
#define CMD_WRITEKMAC_WAIT_LEN    2      /**< 1 byte for write-only ID (for now) + 1 byte for command. */
#define CMD_WRITETX_WAIT_LEN      3      /**< 1 byte for write-only ID + 2 bytes for data size (uint16). */
#define CMD_WRITEMC_WAIT_LEN      3      /**< 2 bytes for uint16 message counter + 1 byte for command. */

/* Enumerations --------------------------------------------------------------*/

/**
 * @brief Enumeration of SPI command values.
 *
 * This enumeration lists all possible SPI command identifiers used for configuration
 * and management operations.
 */
typedef enum {
    CMD_NONE             = 0x00, /**< Unknown command. */
    CMD_READ             = 0x01, /**< Read command. */
    CMD_PING             = 0x02, /**< Ping command. */
    CMD_MAC_STATUS       = 0x03, /**< Read MAC status. */
    CMD_SPI_STATUS       = 0x04, /**< Read SPI status. */
    CMD_READ_VERSION     = 0x05, /**< Read version. */
    CMD_READ_FIRMWARE    = 0x06, /**< Read firmware. */
    CMD_READ_ADDR        = 0x07, /**< Read address. */
    CMD_READ_ID          = 0x08, /**< Read ID. */
    CMD_READ_SN          = 0x09, /**< Read serial number. */
    CMD_READ_RCONF       = 0x0A, /**< Read configuration. */
    CMD_WRITE_RCONF_REQ  = 0x0B, /**< Write configuration request. */
    CMD_WRITE_RCONF      = 0x0C, /**< Write configuration value. */
    CMD_SAVE_RCONF       = 0x0D, /**< Save configuration. */
    CMD_READ_KMAC        = 0x0E, /**< Read KMAC profile. */
    CMD_WRITE_KMAC_REQ   = 0x0F, /**< Write KMAC profile request. */
    CMD_WRITE_KMAC       = 0x10, /**< Write KMAC profile value. */
    CMD_READ_LPM         = 0x11, /**< Read low power mode. */
    CMD_WRITE_LPM_REQ    = 0x12, /**< Set low power mode request. */
    CMD_WRITE_LPM        = 0x13, /**< Set low power mode value. */
    CMD_WRITE_TX_REQ     = 0x14, /**< TX uplink request. */
    CMD_WRITE_TX_SIZE    = 0x15, /**< Waiting size request to read TX data. */
    CMD_WRITE_TX         = 0x16, /**< Write TX uplink value. */
    CMD_READ_CW          = 0x17, /**< Read Continuous Wave (CW) command (not implemented in SPI). */
    CMD_WRITE_CW_REQ     = 0x18, /**< Write CW request (not implemented in SPI driver). */
    CMD_WRITE_CW         = 0x19, /**< Write CW value (not implemented in SPI driver). */
    CMD_READ_PREPASSEN   = 0x1A, /**< Read prepass enable status. */
    CMD_WRITE_PREPASSEN_REQ = 0x1B, /**< Write prepass enable request. */
    CMD_WRITE_PREPASSEN  = 0x1C, /**< Write prepass enable value. */
    CMD_READ_UDATE       = 0x1D, /**< Read user date. */
    CMD_WRITE_UDATE_REQ  = 0x1E, /**< Write date request. */
    CMD_WRITE_UDATE      = 0x1F, /**< Write date value. */
    CMD_WRITE_ID_REQ     = 0x20, /**< Write ID request. */
    CMD_WRITE_ID         = 0x21, /**< Write ID value. */
    CMD_WRITE_ADDR_REQ   = 0x22, /**< Write address request. */
    CMD_WRITE_ADDR       = 0x23, /**< Write address value. */
    CMD_READ_SECKEY      = 0x24, /**< Read secret key. */
    CMD_WRITE_SECKEY_REQ = 0x25, /**< Write secret key request. */
    CMD_WRITE_SECKEY     = 0x26, /**< Write secret key value. */
    CMD_READ_SPIMAC_STATE = 0x27, /**< Read SPI MAC state. */
    CMD_READ_TCXO_WU     = 0x28, /**< Read TCXO wake-up identifier. */
    CMD_WRITE_TCXOWU_REQ = 0x29, /**< Write TCXO wake-up request. */
    CMD_WRITE_TCXOWU     = 0x2A, /**< Write TCXO wake-up value. */
    CMD_READ_RCONF_RAW   = 0x2B, /**< Read raw radio configuration (16 bytes from flash). */
    CMD_READ_MC          = 0x2C, /**< Read message counter (uint16, 9-bit 0..511 range). */
    CMD_WRITE_MC_REQ     = 0x2D, /**< Write message counter request. */
    CMD_WRITE_MC         = 0x2E, /**< Write message counter value (uint16, folded mod 512). */
    CMD_READ_KCFG        = 0x2F, /**< Read stack config bitmap (uint32, read-only). */
    /* DFU commands - aligned with bootloader (0x30-0x3F range) */
    CMD_DFU_ENTER        = 0x3F, /**< Enter DFU/bootloader mode. */
    SPICMD_MAX_COUNT     = 0x40  /**< Maximum number of SPI commands. */
} CmdValue;

/* Types ---------------------------------------------------------------------*/

/**
 * @brief Function pointer type for processing SPI commands.
 *
 * This type defines a pointer to a function that processes a specific SPI command.
 *
 * @param[in] rx Pointer to the SPI receive buffer containing the command.
 * @param[out] tx Pointer to the SPI transmit buffer where the response will be written.
 *
 * @return true if the command is processed successfully, false otherwise.
 */
typedef bool (*pvSPICMD_cmd_proc_fun_type)(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Descriptor structure for an SPI command.
 *
 * This structure holds the necessary information to process a specific SPI command,
 * including the command identifier, any expected subsequent command, and a pointer
 * to the function that handles the command.
 *
 * SPI commands are typically formatted as "SPI+XXXX=..." or "SPI+XXXX=?".
 */
struct spicmd_desc_t {
    CmdValue cmd;                             /**< SPI command identifier. */
    CmdValue next_cmd;                        /**< Next expected command, if any. */
    pvSPICMD_cmd_proc_fun_type f_ht_cmd_fun_proc; /**< Function to process the command. */
};

/* Extern Variables ----------------------------------------------------------*/
extern const char *spicmd_version;                        /**< SPI command list version. */
extern const struct spicmd_desc_t cas_spicmd_list_array[]; /**< Array of SPI command descriptors. */
extern uint16_t userTxPayloadSize;                       /**< User-defined TX payload size. */

#endif /* __MGR_SPI_CMD_CONF_H */

/**
 * @}
 */
