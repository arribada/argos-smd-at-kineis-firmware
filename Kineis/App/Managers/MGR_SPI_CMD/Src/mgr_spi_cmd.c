// SPDX-License-Identifier: no SPDX license
/**
 * @file   mgr_spi_cmd.c
 * @author Arribada
 * @brief  SPI Command Manager - Pipelined Single-Transaction Protocol
 *
 * This module implements a pipelined protocol where:
 * - Each transaction is a fixed 64 bytes
 * - Master sends command, slave sends response to PREVIOUS command
 * - Response is always ready: no timing issues
 *
 * For immediate response, master sends command then NOP (to get response).
 */

/**
 * @addtogroup MGR_SPI_CMD
 * @{
 */

/* Includes -------------------------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>

#include "kns_types.h"
#include "mgr_spi_cmd.h"
#include "kineis_sw_conf.h"
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"
#include "mcu_nvm.h"
#include "mgr_at_cmd_list_user_data.h"
#include "kns_mac.h"
#include "kns_q.h"
#include "user_data.h"
#include "mcu_spi_driver.h"
#include "mgr_spi_cmd_common.h"
#include "mgr_spi_cmd_list_general.h"
#include "mgr_spi_cmd_list.h"
#include "mgr_spi_protocol.h"
#include "kns_cfg.h"
#include "mcu_misc.h"

/* Defines --------------------------------------------------------------------------------------*/
/** @brief Timeout for waiting for transaction (1 second) */
#define TRANSACTION_TIMEOUT_MS  1000

/* Private variables ----------------------------------------------------------------------------*/
volatile SpiState spiState = SPICMD_INIT;
MACStatus macStatus = MAC_OK;
CmdValue cmdInProgress = CMD_NONE;

/* Private functions ----------------------------------------------------------------------------*/

/**
 * @brief Process a command and prepare response
 *
 * @param cmd Command to process
 * @return true if command was processed successfully
 */
static bool MGR_SPI_CMD_process_cmd(uint8_t cmd)
{
    bool ret = true;

    SPI_LOG_VERBOSE("%s:: Processing cmd=%u\r\n", __func__, cmd);

    /* Handle NOP (0x00) - no processing, just used to get previous response */
    if (cmd == CMD_NONE) {
        SPI_LOG_VERBOSE("%s:: NOP command - no processing\r\n", __func__);
        return true;  /* Success, but don't change response buffer */
    }

    if ((cmd > 0) && (cmd < SPICMD_MAX_COUNT)) {
        /* Check if handler function is valid before calling */
        if (cas_spicmd_list_array[cmd].f_ht_cmd_fun_proc != NULL) {
            ret = cas_spicmd_list_array[cmd].f_ht_cmd_fun_proc(&rxBuf, &txBuf);

            /* After processing, set the response in TX buffer for next transaction */
            if (ret && txBuf.next_req > 0) {
                MCU_SPI_DRIVER_set_response(txBuf.data, txBuf.next_req);
            }
        } else {
            MGR_LOG_DEBUG("%s:: NULL handler for cmd %u\r\n", __func__, cmd);
            ret = false;
        }
    } else {
        MGR_LOG_DEBUG("Unknown command %u\r\n", cmd);
        ret = false;
    }

    return ret;
}

/**
 * @brief Process received transaction data
 *
 * @param data Received data buffer
 * @param len Number of bytes received
 */
static void MGR_SPI_CMD_process_transaction(uint8_t *data, uint16_t len)
{
    /* Log received data */
    MGR_LOG_DEBUG("RX %u: %02X %02X %02X %02X %02X\r\n",
                  len, data[0], data[1], data[2], data[3], data[4]);

    /* Try to parse as protocol frame */
    bool frame_ready = MGR_SPI_PROTOCOL_process_buffer(data, len);

    if (frame_ready) {
        if (MGR_SPI_PROTOCOL_is_legacy()) {
            /* Legacy protocol: direct command byte */
            cmdInProgress = MGR_SPI_PROTOCOL_get_legacy_cmd();
            SPI_LOG_VERBOSE("Legacy cmd=0x%02X\r\n", cmdInProgress);
            spiState = SPICMD_PROCESS_CMD;
        } else {
            /* A+ protocol: frame received */
            SpiRequestFrame *req = MGR_SPI_PROTOCOL_get_request();

            if (!req->crc_valid) {
                /* Frame CRC error - prepare error response for next transaction */
                MGR_LOG_DEBUG("Frame CRC error, preparing NACK\r\n");
                uint8_t error_response[8];
                uint16_t resp_len = MGR_SPI_PROTOCOL_build_error_response(
                    error_response, req->sequence, SPI_PROT_STATUS_FRAME_CRC_ERROR);
                MCU_SPI_DRIVER_set_response(error_response, resp_len);
                MGR_SPI_PROTOCOL_reset();
                spiState = SPICMD_IDLE;
            } else {
                /* Valid frame - process command */
                cmdInProgress = req->command;
                SPI_LOG_VERBOSE("A+ cmd=0x%02X seq=%u len=%u\r\n",
                              cmdInProgress, req->sequence, req->data_len);

                /* Copy payload to rxBuf for handler compatibility:
                 * Handlers expect: rxBuf.data[0]=cmd, rxBuf.data[1...]=payload
                 * This maintains backward compatibility with legacy protocol handlers */
                rxBuf.data[0] = req->command;
                if (req->data_len > 0) {
                    memcpy(&rxBuf.data[1], req->data, req->data_len);
                }
                rxBuf.size = req->data_len + 1;  /* cmd byte + payload */

                spiState = SPICMD_PROCESS_CMD;
            }
        }
    } else {
        /* Not a valid frame - could be all 0xFF (dummy read) or garbage */
        if (spi_protocol_ctx.state == SPI_PROT_ERROR) {
            SPI_LOG_VERBOSE("Protocol parse error\r\n");
        }
        /* Reset protocol and stay idle */
        MGR_SPI_PROTOCOL_reset();
        spiState = SPICMD_IDLE;
    }
}

/* Public functions -----------------------------------------------------------------------------*/

bool MGR_SPI_CMD_start(void *context)
{
    /* Initialize protocol layer */
    MGR_SPI_PROTOCOL_init();

    /* Register SPI driver (callback not used in pipelined mode) */
    bool ret = MCU_SPI_DRIVER_register(context, NULL);
    if (!ret) {
        spiState = SPICMD_ERROR;
    }
    return ret;
}

void MGR_SPI_CMD_state_handler(void)
{
    switch (spiState) {
    case SPICMD_INIT:
        MGR_LOG_DEBUG("SPI_CMD_INIT\r\n");
        if (MGR_SPI_CMD_start(NULL)) {
            spiState = SPICMD_IDLE;
        } else {
            spiState = SPICMD_ERROR;
        }
        break;

    case SPICMD_IDLE:
        {
            /* Poll for transaction end */
            uint16_t bytes_received = 0;
            if (MCU_SPI_DRIVER_check_transaction_end(&bytes_received)) {
                /* Transaction complete - abort DMA and process */
                MCU_SPI_DRIVER_abort_transfer();

                if (bytes_received > 0) {
                    rxBuf.size = bytes_received;
                    MGR_SPI_CMD_process_transaction(rxBuf.data, bytes_received);
                }

                /* Start next DMA transfer (response already in TX buffer) */
                if (spiState == SPICMD_IDLE) {
                    MCU_SPI_DRIVER_read();
                }
                /* If state changed to PROCESS_CMD, don't arm DMA yet.
                 * Command will be processed synchronously, then we arm with response. */

                startTickTimeout = 0;
            } else {
                /* Timeout handling - just for logging */
                if (startTickTimeout == 0) {
                    startTickTimeout = HAL_GetTick();
                }

                if ((HAL_GetTick() - startTickTimeout) > TRANSACTION_TIMEOUT_MS) {
                    SPI_LOG_VERBOSE("Waiting for transaction...\r\n");
                    startTickTimeout = HAL_GetTick();
                }
            }
        }
        break;

    case SPICMD_PROCESS_CMD:
        {
            SPI_LOG_VERBOSE("Processing cmd=0x%02X\r\n", cmdInProgress);

            bool ret = MGR_SPI_CMD_process_cmd(cmdInProgress);
            if (!ret) {
                MGR_LOG_DEBUG("Failed to process cmd %u\r\n", cmdInProgress);
            }

            /* Reset protocol for next command */
            MGR_SPI_PROTOCOL_reset();
            cmdInProgress = CMD_NONE;

            /* Start next DMA transfer (response now in TX buffer) */
            MCU_SPI_DRIVER_read();
            spiState = SPICMD_IDLE;
        }
        break;

    case SPICMD_ERROR:
        MGR_LOG_DEBUG("SPI error, resetting...\r\n");
        startTickTimeout = 0;
        MGR_SPI_PROTOCOL_reset();
        cmdInProgress = CMD_NONE;

        if (!MCU_SPI_DRIVER_reset(NULL)) {
            MGR_LOG_DEBUG("Reset failed, retrying...\r\n");
            spi_stats.error_count++;
        }
        spiState = SPICMD_INIT;
        break;

    default:
        MGR_LOG_DEBUG("Unknown state: %u\r\n", spiState);
        spiState = SPICMD_ERROR;
        break;
    }
}

enum KNS_status_t MGR_SPI_CMD_macEvtProcess(void)
{
    enum KNS_status_t cbStatus;
    struct KNS_MAC_srvcEvt_t srvcEvt;
    struct sUserDataTxFifoElt_t *spUserDataMsg = USERDATA_txFifoGetFirst();

    cbStatus = KNS_Q_pop(KNS_Q_UL_MAC2APP, (void *)&srvcEvt);

    if (cbStatus != KNS_STATUS_OK)
        return cbStatus;

    /* Get pointer to user data FIFO element when possible.
     * Note: macStatus is set in the second switch below, not here. */
    switch (srvcEvt.id) {
    case (KNS_MAC_TX_DONE):
    case (KNS_MAC_TXACK_DONE):
    case (KNS_MAC_TX_TIMEOUT):
    case (KNS_MAC_TXACK_TIMEOUT):
    case (KNS_MAC_RX_ERROR):
    case (KNS_MAC_RX_TIMEOUT):
        spUserDataMsg = USERDATA_txFifoFindPayload(srvcEvt.tx_ctxt.data,
            srvcEvt.tx_ctxt.data_bitlen);
        kns_assert(spUserDataMsg != NULL);
        /* macStatus will be set in the processing switch below */
        break;
    case (KNS_MAC_ERROR):
        if (srvcEvt.app_evt == KNS_MAC_SEND_DATA) {
            spUserDataMsg = USERDATA_txFifoFindPayload(srvcEvt.tx_ctxt.data,
                srvcEvt.tx_ctxt.data_bitlen);
            MCU_MISC_TCXO_Force_State(false);
            kns_assert(spUserDataMsg != NULL);
        }
        break;
    default:
        break;
    }

    /* Process event */
    switch (srvcEvt.id) {
    case (KNS_MAC_TX_DONE):
        MGR_LOG_DEBUG("MGR_SPI_CMD TX_DONE callback reached\r\n");
        kns_assert(spUserDataMsg->bIsToBeTransmit);
        if (spUserDataMsg->u8Attr.sf == ATTR_MAIL_REQUEST) {
            /* TODO: no service used for the moment */
        } else {
            USERDATA_txFifoRemoveElt(spUserDataMsg);
        }
        MCU_MISC_TCXO_Force_State(false);
        macStatus = MAC_TX_DONE;
        cbStatus = KNS_STATUS_OK;
        break;

    case (KNS_MAC_TXACK_DONE):
        MGR_LOG_DEBUG("MGR_SPI_CMD TXACK_DONE callback reached\r\n");
        kns_assert(spUserDataMsg->bIsToBeTransmit);
        if (spUserDataMsg->u8Attr.sf == ATTR_MAIL_REQUEST)
            macStatus = MAC_TX_DONE;
        else
            macStatus = MAC_TXACK_DONE;
        MCU_MISC_TCXO_Force_State(false);
        USERDATA_txFifoRemoveElt(spUserDataMsg);
        cbStatus = KNS_STATUS_OK;
        break;

    case (KNS_MAC_TX_TIMEOUT):
        MGR_LOG_DEBUG("MGR_SPI_CMD TX_TIMEOUT callback reached\r\n");
        kns_assert(spUserDataMsg->bIsToBeTransmit);
        MCU_MISC_TCXO_Force_State(false);
        USERDATA_txFifoRemoveElt(spUserDataMsg);
        macStatus = MAC_TX_TIMEOUT;
        cbStatus = KNS_STATUS_TIMEOUT;
        break;

    case (KNS_MAC_TXACK_TIMEOUT):
        MGR_LOG_DEBUG("MGR_SPI_CMD TXACK_TIMEOUT callback reached\r\n");
        MCU_MISC_TCXO_Force_State(false);
        kns_assert(spUserDataMsg->bIsToBeTransmit);
        USERDATA_txFifoRemoveElt(spUserDataMsg);
        macStatus = MAC_TXACK_TIMEOUT;
        cbStatus = KNS_STATUS_TIMEOUT;
        break;

    case (KNS_MAC_RX_ERROR):
        MGR_LOG_DEBUG("MGR_SPI_CMD RX ERROR callback reached\r\n");
        if (spUserDataMsg->bIsToBeTransmit) {
            USERDATA_txFifoRemoveElt(spUserDataMsg);
            macStatus = MAC_RX_ERROR;
            cbStatus = KNS_STATUS_RF_ERR;
        } else {
            MGR_LOG_DEBUG("RX ERROR unexpected event received.\r\n");
            macStatus = MAC_ERROR;
            cbStatus = KNS_STATUS_ERROR;
        }
        break;

    case (KNS_MAC_RX_TIMEOUT):
        MGR_LOG_DEBUG("MGR_SPI_CMD RX timeout callback reached\r\n");
        USERDATA_txFifoRemoveElt(spUserDataMsg);
        macStatus = MAC_RX_TIMEOUT;
        cbStatus = KNS_STATUS_TIMEOUT;
        break;

    case (KNS_MAC_OK):
        MGR_LOG_DEBUG("MGR_SPI_CMD MAC reported OK to previous command.\r\n");
        if (srvcEvt.app_evt == KNS_MAC_STOP_SEND_DATA)
            kns_assert(USERDATA_txFifoFlush() == true);
        macStatus = MAC_OK;
        cbStatus = KNS_STATUS_OK;
        break;

    case (KNS_MAC_ERROR):
        MGR_LOG_DEBUG("MGR_SPI_CMD MAC reported ERROR to previous command.\r\n");
        if (srvcEvt.app_evt == KNS_MAC_SEND_DATA)
            USERDATA_txFifoRemoveElt(spUserDataMsg);
        macStatus = MAC_ERROR;
        cbStatus = KNS_STATUS_ERROR;
        break;

    /* Downlink and satellite detection events - informational, no user data involved */
    case (KNS_MAC_RX_RECEIVED):
        MGR_LOG_DEBUG("MGR_SPI_CMD RX frame received\r\n");
        macStatus = MAC_RX_RECEIVED;
        cbStatus = KNS_STATUS_OK;
        break;

    case (KNS_MAC_DL_BC):
        MGR_LOG_DEBUG("MGR_SPI_CMD Downlink beacon received\r\n");
        macStatus = MAC_RX_RECEIVED;
        cbStatus = KNS_STATUS_OK;
        break;

    case (KNS_MAC_DL_ACK):
        MGR_LOG_DEBUG("MGR_SPI_CMD Downlink ACK received\r\n");
        macStatus = MAC_RX_RECEIVED;
        cbStatus = KNS_STATUS_OK;
        break;

    case (KNS_MAC_SAT_DETECTED):
        MGR_LOG_DEBUG("MGR_SPI_CMD Satellite detected\r\n");
        macStatus = MAC_SAT_DETECTED;
        cbStatus = KNS_STATUS_OK;
        break;

    case (KNS_MAC_SAT_LOST):
        MGR_LOG_DEBUG("MGR_SPI_CMD Satellite lost\r\n");
        macStatus = MAC_SAT_LOST;
        cbStatus = KNS_STATUS_OK;
        break;

    case (KNS_MAC_SAT_DETECT_TIMEOUT):
        MGR_LOG_DEBUG("MGR_SPI_CMD Satellite detection timeout\r\n");
        macStatus = MAC_SAT_LOST;
        cbStatus = KNS_STATUS_TIMEOUT;
        break;

    case (KNS_MAC_RF_ABORTED):
        MGR_LOG_DEBUG("MGR_SPI_CMD RF operation aborted\r\n");
        macStatus = MAC_RF_ABORTED;
        cbStatus = KNS_STATUS_OK;
        break;

    default:
        MGR_LOG_DEBUG("MGR_SPI_CMD Unknown MAC event: %d\r\n", srvcEvt.id);
        macStatus = MAC_ERROR;
        cbStatus = KNS_STATUS_ERROR;
        break;
    }

    return cbStatus;
}

/**
 * @}
 */
