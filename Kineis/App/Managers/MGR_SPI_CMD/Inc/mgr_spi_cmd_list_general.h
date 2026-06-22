/* SPDX-License-Identifier: no SPDX license */
/**
 * @file mgr_spi_cmd_list_general.h
 * @author Kineis
 * @brief Subset of SPI commands for general purpose operations (e.g., get ID, firmware version, etc.)
 *
 * This header file declares functions that handle SPI commands for general device management such as
 * retrieving version information, reading configurations, and writing parameters like addresses, IDs,
 * and security keys.
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MGR_SPI_CMD_LIST_GENERAL_H
#define __MGR_SPI_CMD_LIST_GENERAL_H

/* Includes ------------------------------------------------------------------*/
#include "mgr_at_cmd_common.h"
#include "mcu_spi_driver.h"

/* Define value --------------------------------------------------------------*/

/* Extern variable -----------------------------------------------------------*/
/* Functions -----------------------------------------------------------------*/

/**
 * @brief Retrieve the firmware version of the device.
 *
 * This function processes the SPI command to obtain the current firmware version of the device.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the version information will be written.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_VERSION_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Process a generic read command.
 *
 * This function handles the SPI command to read data or configuration from the device.
 *
 * @param rx Pointer to the SPI receive buffer containing the command details.
 * @param tx Pointer to the SPI transmit buffer where the requested data will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READ_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Ping the device to verify connectivity.
 *
 * This function processes the SPI command that pings the device, allowing the caller to confirm the device is responsive.
 *
 * @param rx Pointer to the SPI receive buffer containing the ping command.
 * @param tx Pointer to the SPI transmit buffer where the ping response will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_PING_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Retrieve the MAC status of the device.
 *
 * This function processes the SPI command to get the status of the device's MAC (Media Access Control) layer.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the MAC status information will be written.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_MACSTATUS_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Retrieve the SPI state of the device.
 *
 * This function processes the SPI command to obtain the current state of the SPI interface or the device as it relates to SPI communications.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the state information will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_SPISTATE_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Retrieve the SPI driver version.
 *
 * This function processes the SPI command to get the version information of the SPI driver or interface.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the SPI version information will be written.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READSPIVERSION_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Retrieve the firmware details of the device.
 *
 * This function processes the SPI command to obtain detailed firmware information.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the firmware details will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READFIRMWARE_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Read the device address.
 *
 * This function processes the SPI command to read the current address assigned to the device.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the address will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READADDRESS_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Initiate a request to write a new device address.
 *
 * This function processes the SPI command that initiates the process for writing a new address to the device.
 *
 * @param rx Pointer to the SPI receive buffer containing the request command.
 * @param tx Pointer to the SPI transmit buffer where the request acknowledgment will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITEADDRESSREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Write a new device address.
 *
 * This function processes the SPI command to update the device's address following a valid write address request.
 *
 * @param rx Pointer to the SPI receive buffer containing the new address data.
 * @param tx Pointer to the SPI transmit buffer where the result of the write operation will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITEADDRESS_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Read the device identification number.
 *
 * This function processes the SPI command to retrieve the device ID.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the device ID will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READID_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Initiate a request to write a new device ID.
 *
 * This function processes the SPI command that initiates the procedure for updating the device ID.
 *
 * @param rx Pointer to the SPI receive buffer containing the request.
 * @param tx Pointer to the SPI transmit buffer where the acknowledgment of the request will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITEIDREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Write a new device identification number.
 *
 * This function processes the SPI command to update the device's ID after a valid write request.
 *
 * @param rx Pointer to the SPI receive buffer containing the new ID data.
 * @param tx Pointer to the SPI transmit buffer where the result of the write operation will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITEID_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Read the device serial number.
 *
 * This function processes the SPI command to retrieve the device's serial number.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the serial number will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READSN_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Read the radio configuration.
 *
 * This function processes the SPI command to obtain the current radio configuration settings of the device.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the radio configuration data will be written.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READRCONF_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Initiate a request to write new radio configuration settings.
 *
 * This function processes the SPI command that starts the process of updating the device's radio configuration.
 *
 * @param rx Pointer to the SPI receive buffer containing the request command.
 * @param tx Pointer to the SPI transmit buffer where the acknowledgment will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITERCONFREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Write new radio configuration settings.
 *
 * This function processes the SPI command to update the device's radio configuration after a valid write request.
 *
 * @param rx Pointer to the SPI receive buffer containing the new configuration data.
 * @param tx Pointer to the SPI transmit buffer where the result of the write operation will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITERCONF_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Save the current radio configuration to nonvolatile memory.
 *
 * This function processes the SPI command to store the active radio configuration settings,
 * ensuring they persist after a reset.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the operation result will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_SAVERCONF_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Read the low power mode (LPM) configuration.
 *
 * This function processes the SPI command to retrieve the current low power mode settings of the device.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the LPM configuration will be written.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READLPM_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Initiate a request to write new low power mode (LPM) settings.
 *
 * This function processes the SPI command that initiates the procedure for updating the device's low power mode configuration.
 *
 * @param rx Pointer to the SPI receive buffer containing the request.
 * @param tx Pointer to the SPI transmit buffer where the acknowledgment will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITELPMREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Write new low power mode (LPM) settings.
 *
 * This function processes the SPI command to update the device's low power mode configuration after a valid write request.
 *
 * @param rx Pointer to the SPI receive buffer containing the new LPM settings.
 * @param tx Pointer to the SPI transmit buffer where the result of the operation will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITELPM_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Read the security key from the device.
 *
 * This function processes the SPI command to retrieve the current security key used for secure operations.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the security key will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READSECKEY_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Initiate a request to write a new security key.
 *
 * This function processes the SPI command that starts the process of updating the device's security key.
 *
 * @param rx Pointer to the SPI receive buffer containing the request.
 * @param tx Pointer to the SPI transmit buffer where the acknowledgment will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITESECKEYREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Write a new security key to the device.
 *
 * This function processes the SPI command to update the device's security key after a valid write request.
 *
 * @param rx Pointer to the SPI receive buffer containing the new security key data.
 * @param tx Pointer to the SPI transmit buffer where the result of the write operation will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITESECKEY_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Read the state of the SPI MAC configuration.
 *
 * This function processes the SPI command to retrieve the current state or configuration of the SPI MAC.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the MAC state information will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READSPIMACSTATE_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Write TCXO calibration or configuration data.
 *
 * This function processes the SPI command to update the Temperature Compensated Crystal Oscillator (TCXO)
 * calibration or configuration settings.
 *
 * @param rx Pointer to the SPI receive buffer containing the new TCXO data.
 * @param tx Pointer to the SPI transmit buffer where the result of the write operation will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITETCXO_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Initiate a request to write new TCXO configuration data.
 *
 * This function processes the SPI command that starts the procedure for updating the TCXO settings.
 *
 * @param rx Pointer to the SPI receive buffer containing the request.
 * @param tx Pointer to the SPI transmit buffer where the acknowledgment will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITETCXOREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Read the TCXO configuration or calibration settings.
 *
 * This function processes the SPI command to retrieve the current settings of the Temperature Compensated Crystal Oscillator (TCXO).
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the TCXO configuration data will be sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READTCXO_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Enter DFU (Device Firmware Update) bootloader mode.
 *
 * This function processes the SPI command to enter the bootloader for firmware update.
 * It sets the DFU request flag in the bootloader state flash and triggers a system reset.
 * After reset, the bootloader will detect the DFU flag and enter DFU mode.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the acknowledgment will be sent.
 *
 * @return true if the command is correctly received and processed (device will reset).
 */
bool bMGR_SPI_CMD_DFU_ENTER_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Read the raw radio configuration from flash.
 *
 * This function reads the raw 16-byte radio configuration directly from flash memory,
 * without decoding it. This is useful for debugging and verifying flash contents.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the raw data will be sent (16 bytes).
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READRCONFRAW_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Read the message counter (MC).
 *
 * Returns the current 9-bit message counter as a little-endian uint16 (2 bytes).
 * SPI equivalent of the UART AT+MC query.
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the uint16 MC is sent (2 bytes, LE).
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READMC_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Write message counter request (pipelined-protocol ACK phase).
 *
 * @param rx Unused.
 * @param tx Pointer to the SPI transmit buffer where the ACK is sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITEMCREQ_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Write the message counter (MC) value (data phase).
 *
 * Expects a little-endian uint16 in rx->data[1..2]. The host value is folded
 * into the 9-bit protocol range (mod 512) before being stored, mirroring AT+MC.
 *
 * @param rx Pointer to the SPI receive buffer (cmd byte + uint16 MC, LE).
 * @param tx Pointer to the SPI transmit buffer where the status is sent.
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_WRITEMC_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

/**
 * @brief Read the stack configuration bitmap (KCFG).
 *
 * Returns the v11 stack config bitmap as a little-endian uint32 (4 bytes):
 * bit0 = L1 TX-period timer suspended, bit1 = resumed. SPI equivalent of the
 * UART AT+KCFG query. Read-only: there is no SPI write for KCFG because setting
 * it would suspend/resume the RTC wake-up timer owned by the LPM scheduler and
 * silently break auto-sleep (same reason the AT+KCFG set form is refused).
 *
 * @param rx Pointer to the SPI receive buffer containing the command.
 * @param tx Pointer to the SPI transmit buffer where the uint32 bitmap is sent (4 bytes, LE).
 *
 * @return true if the command is correctly received and processed, false otherwise.
 */
bool bMGR_SPI_CMD_READKCFG_cmd(SPI_Buffer *rx, SPI_Buffer *tx);

#endif /* __MGR_SPI_CMD_LIST_GENERAL_H */

/** @} */
