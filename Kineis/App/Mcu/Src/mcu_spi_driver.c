// SPDX-License-Identifier: no SPDX license
/**
 * @file    mcu_spi_driver.c
 * @author  Arribada
 * @brief   MCU wrapper for SPI - Pipelined Single-Transaction Protocol
 *
 * This driver implements a simple pipelined protocol:
 * - Fixed 64-byte transactions
 * - Master sends command, slave sends response to PREVIOUS command
 * - No timing issues: response is always ready
 *
 * Flow:
 * 1. Transaction N: Master sends CMD_N, Slave sends Response_(N-1)
 * 2. STM32 processes CMD_N, prepares Response_N in TX buffer
 * 3. Transaction N+1: Master sends CMD_(N+1), Slave sends Response_N
 *
 * For immediate response, master can send NOP command after real command.
 */

/**
 * @addtogroup MCU_APP_WRAPPERS
 * @{
 */

/* Includes ------------------------------------------------------------------*/
#include <stddef.h>
#include <ctype.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "kineis_sw_conf.h"
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"
#include "mcu_spi_driver.h"
#include "mgr_spi_cmd.h"

/* Variables -----------------------------------------------------------------*/
static SPI_HandleTypeDef *hspi_handle = NULL;

/* SPI buffers */
static uint8_t spiTxBuf[TXBUF_SIZE];
static uint8_t spiRxBuf[RXBUF_SIZE];
SPI_Buffer rxBuf = { .data = spiRxBuf, .size = RXBUF_SIZE, .next_req = SPI_TRANSACTION_SIZE };
SPI_Buffer txBuf = { .data = spiTxBuf, .size = TXBUF_SIZE, .next_req = 0 };

/* SPI statistics */
SPI_Stats_t spi_stats = {0};

/* Response ready flag - indicates TX buffer contains valid response */
volatile bool response_ready = false;

/* Timeout tick */
volatile uint32_t startTickTimeout = 0;

/* Transaction end detection */
static volatile uint16_t last_rx_count = 0;
static volatile uint32_t rx_stable_start_tick = 0;
static volatile bool rx_activity_detected = false;

/* Stability timeout - transaction complete when no new bytes for this duration */
#define RX_STABLE_TIMEOUT_MS 2

/* Private functions ---------------------------------------------------------*/

/**
 * @brief Fill TX buffer with idle pattern
 */
static void fill_idle_pattern(void)
{
    memset(spiTxBuf, SPI_IDLE_PATTERN, SPI_TRANSACTION_SIZE);
    response_ready = false;
}

/**
 * @brief SPI TxRx completion callback
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        startTickTimeout = 0;
        spi_stats.rx_count++;
        spi_stats.tx_count++;
        /* Transaction complete - state machine will process */
    } else {
        kns_assert(0);
    }
}

/**
 * @brief SPI error callback - just clear errors and continue
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    /* Clear all error flags */
    if (hspi->ErrorCode & HAL_SPI_ERROR_OVR) {
        __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }
    if (hspi->ErrorCode & HAL_SPI_ERROR_MODF) {
        __HAL_SPI_CLEAR_MODFFLAG(hspi);
    }
    if (hspi->ErrorCode & HAL_SPI_ERROR_FRE) {
        __HAL_SPI_CLEAR_FREFLAG(hspi);
    }
    hspi->ErrorCode = HAL_SPI_ERROR_NONE;

    /* Errors during transaction are normal (CS released early).
     * Let the polling mechanism detect transaction end. */
    spi_stats.error_count++;
}

/* Public functions ----------------------------------------------------------*/

bool MCU_SPI_DRIVER_set_response(const uint8_t *data, uint16_t len)
{
    if (len > SPI_TRANSACTION_SIZE) {
        MGR_LOG_DEBUG("%s:: Response too large (%u > %u)\r\n",
                      __func__, len, SPI_TRANSACTION_SIZE);
        return false;
    }

    /* Copy response to TX buffer */
    memcpy(spiTxBuf, data, len);

    /* Fill rest with idle pattern */
    if (len < SPI_TRANSACTION_SIZE) {
        memset(&spiTxBuf[len], SPI_IDLE_PATTERN, SPI_TRANSACTION_SIZE - len);
    }

    txBuf.next_req = len;
    response_ready = true;

    SPI_LOG_VERBOSE("Response set: %u bytes [%02X %02X %02X %02X %02X]\r\n",
                  len, spiTxBuf[0], spiTxBuf[1], spiTxBuf[2],
                  spiTxBuf[3], spiTxBuf[4]);

    return true;
}

HAL_StatusTypeDef MCU_SPI_DRIVER_read(void)
{
    /* Reset transaction detection */
    last_rx_count = 0;
    rx_stable_start_tick = 0;
    rx_activity_detected = false;

    /* Ensure SPI is ready */
    if (hspi_handle->State != HAL_SPI_STATE_READY) {
        HAL_SPI_Abort(hspi_handle);
        hspi_handle->State = HAL_SPI_STATE_READY;
    }

    /* Clear any pending errors */
    if (hspi_handle->ErrorCode != HAL_SPI_ERROR_NONE) {
        if (hspi_handle->ErrorCode & HAL_SPI_ERROR_OVR) {
            __HAL_SPI_CLEAR_OVRFLAG(hspi_handle);
        }
        hspi_handle->ErrorCode = HAL_SPI_ERROR_NONE;
    }

    /* If no response ready, fill with idle pattern */
    if (!response_ready) {
        fill_idle_pattern();
    }

    /* Arm DMA for fixed-size transaction */
    HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive_DMA(hspi_handle,
        spiTxBuf, spiRxBuf, SPI_TRANSACTION_SIZE);

    if (ret == HAL_OK) {
        spiState = SPICMD_IDLE;
        /* Response was sent, clear flag for next transaction */
        response_ready = false;
    } else {
        MGR_LOG_DEBUG("%s:: HAL error %d\r\n", __func__, ret);
        spiState = SPICMD_ERROR;
        spi_stats.error_count++;
        spi_stats.last_error = hspi_handle->ErrorCode;
    }

    return ret;
}

bool MCU_SPI_DRIVER_check_transaction_end(uint16_t *bytes_received)
{
    if (hspi_handle == NULL) {
        return false;
    }

    /* Get current bytes received from DMA */
    uint16_t total = hspi_handle->RxXferSize;
    uint16_t remaining = 0;

    if (hspi_handle->hdmarx != NULL && hspi_handle->hdmarx->Instance != NULL) {
        remaining = __HAL_DMA_GET_COUNTER(hspi_handle->hdmarx);
    }

    uint16_t current = (remaining <= total) ? (total - remaining) : 0;

    /* Detect new bytes received */
    if (current > last_rx_count) {
        rx_activity_detected = true;
        last_rx_count = current;
        rx_stable_start_tick = HAL_GetTick();
        return false;
    }

    /* Check if stable long enough */
    if (rx_activity_detected && current > 0) {
        if (rx_stable_start_tick == 0) {
            rx_stable_start_tick = HAL_GetTick();
        }

        if ((HAL_GetTick() - rx_stable_start_tick) >= RX_STABLE_TIMEOUT_MS) {
            *bytes_received = current;

            /* Reset for next transaction */
            last_rx_count = 0;
            rx_stable_start_tick = 0;
            rx_activity_detected = false;

            return true;
        }
    }

    return false;
}

void MCU_SPI_DRIVER_abort_transfer(void)
{
    if (hspi_handle == NULL) {
        return;
    }

    HAL_SPI_Abort(hspi_handle);
    hspi_handle->State = HAL_SPI_STATE_READY;

    /* Reset detection state */
    last_rx_count = 0;
    rx_stable_start_tick = 0;
    rx_activity_detected = false;
}

bool MCU_SPI_DRIVER_register(void *handle, int8_t (*rx_spi_evt_cb)(SPI_Buffer *rx, SPI_Buffer *tx))
{
    (void)rx_spi_evt_cb;  /* Callback not used in pipelined mode */

    if (handle != NULL && hspi_handle == NULL) {
        hspi_handle = (SPI_HandleTypeDef *)handle;

        MGR_LOG_DEBUG("\r\n========================================\r\n");
        MGR_LOG_DEBUG("SPI DRIVER - Pipelined Protocol\r\n");
        MGR_LOG_DEBUG("========================================\r\n");
        MGR_LOG_DEBUG("Transaction size: %u bytes\r\n", SPI_TRANSACTION_SIZE);
        MGR_LOG_DEBUG("Mode: %s\r\n",
                      (hspi_handle->Init.Mode == SPI_MODE_SLAVE) ? "SLAVE" : "MASTER");
        MGR_LOG_DEBUG("========================================\r\n");
    } else if (handle == NULL && hspi_handle == NULL) {
        MGR_LOG_DEBUG("%s:: ERROR: invalid hspi ptr\r\n", __func__);
        kns_assert(0);
    }

    /* Initialize with idle pattern */
    fill_idle_pattern();
    rxBuf.next_req = SPI_TRANSACTION_SIZE;

    /* Start first DMA transfer */
    HAL_StatusTypeDef ret = MCU_SPI_DRIVER_read();

    return (ret == HAL_OK);
}

bool MCU_SPI_DRIVER_reset(SPI_HandleTypeDef *hspi)
{
    spi_stats.reset_count++;
    SPI_LOG_VERBOSE("%s:: reset #%lu\r\n", __func__, (unsigned long)spi_stats.reset_count);

    if (hspi != NULL && hspi_handle == NULL) {
        hspi_handle = hspi;
    } else if (hspi == NULL && hspi_handle == NULL) {
        MGR_LOG_DEBUG("%s:: ERROR: invalid hspi ptr\r\n", __func__);
        kns_assert(0);
    }

    /* Clear errors */
    if (hspi_handle->ErrorCode & HAL_SPI_ERROR_OVR) {
        __HAL_SPI_CLEAR_OVRFLAG(hspi_handle);
    }
    if (hspi_handle->ErrorCode & HAL_SPI_ERROR_MODF) {
        __HAL_SPI_CLEAR_MODFFLAG(hspi_handle);
    }
    if (hspi_handle->ErrorCode & HAL_SPI_ERROR_FRE) {
        __HAL_SPI_CLEAR_FREFLAG(hspi_handle);
    }

    /* Abort and reset SPI */
    HAL_SPI_Abort(hspi_handle);

    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();

    __HAL_SPI_DISABLE_IT(hspi_handle, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);
    HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);

    hspi_handle->State = HAL_SPI_STATE_READY;
    hspi_handle->ErrorCode = HAL_SPI_ERROR_NONE;

    HAL_StatusTypeDef ret = HAL_SPI_DeInit(hspi_handle);
    if (ret != HAL_OK) {
        MGR_LOG_DEBUG("Failed to deinitialize SPI.\r\n");
        return false;
    }

    ret = HAL_SPI_Init(hspi_handle);
    if (ret != HAL_OK) {
        MGR_LOG_DEBUG("Failed to reinitialize SPI.\r\n");
        return false;
    }
    __HAL_SPI_ENABLE(hspi_handle);

    /* Re-register with idle pattern */
    fill_idle_pattern();
    rxBuf.next_req = SPI_TRANSACTION_SIZE;

    return (MCU_SPI_DRIVER_read() == HAL_OK);
}

/**
 * @}
 */
