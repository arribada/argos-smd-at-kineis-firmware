// SPDX-License-Identifier: no SPDX license
/**
 * @file    mcu_at_spi.c
 * @author  Arribada
 * @brief   MCU wrapper for AT SPI
 */

/**
 * @addtogroup MCU_APP_WRAPPERS
 * @brief MCU wrapper used by Kineis Application example.
 *
 * One has to implement API as per its microcontroller and its platform ressources.
 * This version is for STM32 uC such as STM32WLE5xx, STM32WL55xx.
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
#include "kineis_sw_conf.h" // for assert include below
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"
#include "mcu_spi_driver.h"

/* Defines -------------------------------------------------------------------*/


/* Variables -----------------------------------------------------------------*/
static SPI_HandleTypeDef *hspi_handle = NULL;

static uint8_t spiTxBuf[TXBUF_SIZE];
static uint8_t spiRxBuf[RXBUF_SIZE];
static uint8_t spiIdleTxBuf[RXBUF_SIZE];  /* Idle TX buffer filled with 0xFF */
SPI_Buffer rxBuf = { .data = spiRxBuf, .size = RXBUF_SIZE, .next_req = 1};
SPI_Buffer txBuf = { .data = spiTxBuf, .size = TXBUF_SIZE, .next_req = 0};

/* SPI statistics for diagnostics */
SPI_Stats_t spi_stats = {0};

static int8_t (*rxSpiEvtCb)(SPI_Buffer *rx, SPI_Buffer *tx) = NULL;

volatile uint32_t startTickTimeout = 0;

/* Variables for transaction end detection based on RxXferCount stability */
static uint16_t last_rx_count = 0;        /* Last observed RxXferCount */
static uint32_t rx_stable_start_tick = 0; /* When RxXferCount stopped changing */
static bool rx_activity_detected = false; /* True if we've received at least one byte */
#define RX_STABLE_TIMEOUT_MS 2  /* Consider transaction complete after 2ms of no new bytes */
/* Private function prototypes -----------------------------------------------*/


// /* Functions -----------------------------------------------------------------*/
// Callback when a command is received (for Receive_IT only - not used anymore)
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        MGR_LOG_DEBUG("RX completed\r\n");
        spi_stats.rx_count++;
        if (rxSpiEvtCb != NULL)
        {
        	rxSpiEvtCb(&rxBuf, &txBuf);
			startTickTimeout = 0;
        } else {
			MGR_LOG_DEBUG("%s:: rxSpiEvtCb not defined\r\n", __func__);
            spiState = SPICMD_ERROR;
            spi_stats.error_count++;
        }
    } else {
		MGR_LOG_DEBUG("%s::ERROR SPI interrupt from other SPI instance\r\n", __func__);
    	kns_assert(0);
    }
}

// Callback for SPI TxRx completion - used for both read and writeread
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        MGR_LOG_DEBUG("TX-RX completed (state=%d)\r\n", spiState);
		startTickTimeout = 0;

		if (spiState == SPICMD_WAITING_RX) {
			/* This was a "read" operation - process received data */
			spi_stats.rx_count++;
			if (rxSpiEvtCb != NULL) {
				rxSpiEvtCb(&rxBuf, &txBuf);
			} else {
				MGR_LOG_DEBUG("%s:: rxSpiEvtCb not defined\r\n", __func__);
				spiState = SPICMD_ERROR;
				spi_stats.error_count++;
			}
		} else if (spiState == SPICMD_WAITING_TX) {
			/* This was a "writeread" operation - response sent, go idle */
			spi_stats.tx_count++;
			spiState = SPICMD_IDLE;
		} else {
			MGR_LOG_DEBUG("%s:: Unexpected state %d\r\n", __func__, spiState);
			spiState = SPICMD_IDLE;
		}
    } else {
		MGR_LOG_DEBUG("%s::ERROR SPI interrupt from other SPI instance\r\n", __func__);
    	kns_assert(0);
    }
}

// Callback not used
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        MGR_LOG_DEBUG("TX completed\r\n");
		startTickTimeout = 0;
		spi_stats.tx_count++;
    } else {
		MGR_LOG_DEBUG("%s::ERROR SPI interrupt from other SPI instance\r\n", __func__);
    	kns_assert(0);
    }
}

HAL_StatusTypeDef MCU_SPI_DRIVER_writeread()
{
	HAL_StatusTypeDef ret = HAL_OK;

	/* Bounds checking */
	if (txBuf.next_req > TXBUF_SIZE || txBuf.next_req > RXBUF_SIZE) {
		MGR_LOG_DEBUG("%s:: Buffer overflow prevented (req=%u, max=%u)\r\n",
					  __func__, txBuf.next_req, TXBUF_SIZE);
		spiState = SPICMD_ERROR;
		spi_stats.error_count++;
		return HAL_ERROR;
	}

	MGR_LOG_DEBUG("%s:: Sending %u bytes, first=0x%02X\r\n",
				  __func__, txBuf.next_req, txBuf.data[0]);

	/* Reset transaction detection for the new transfer */
	last_rx_count = 0;
	rx_stable_start_tick = 0;
	rx_activity_detected = false;

	// waiting Read request
	ret = HAL_SPI_TransmitReceive_IT(hspi_handle, txBuf.data, rxBuf.data, txBuf.next_req);
	if (ret == HAL_OK){
		spiState = SPICMD_WAITING_TX;
		MGR_LOG_DEBUG("%s:: TX ready, waiting for master clock\r\n", __func__);
	} else {
		MGR_LOG_DEBUG("%s:: HAL error %d\r\n", __func__, ret);
		spiState = SPICMD_ERROR;
		spi_stats.error_count++;
		spi_stats.last_error = hspi_handle->ErrorCode;
	}
	return ret;
}

HAL_StatusTypeDef MCU_SPI_DRIVER_read()
{
	if(rxBuf.next_req < 1) {
		MGR_LOG_DEBUG("%s:: Waiting RX is %u should be greater than 0\r\n",__func__, rxBuf.next_req);
		rxBuf.next_req = 1; // next request forced to 1
	}

	/* Bounds checking */
	if (rxBuf.next_req > RXBUF_SIZE) {
		MGR_LOG_DEBUG("%s:: Buffer overflow prevented (req=%u, max=%u)\r\n",
					  __func__, rxBuf.next_req, RXBUF_SIZE);
		spiState = SPICMD_ERROR;
		spi_stats.error_count++;
		return HAL_ERROR;
	}

	/* Fill entire idle TX buffer with 0xFF so slave sends 0xFF during the whole transaction
	 * This is critical for SPI slave mode - we must pre-load the TX buffer before master clocks */
	memset(spiIdleTxBuf, 0xFF, RXBUF_SIZE);

	/* Reset transaction detection state for new transfer */
	last_rx_count = 0;
	rx_stable_start_tick = 0;
	rx_activity_detected = false;

	/* Use TransmitReceive_IT instead of Receive_IT so slave sends data while receiving */
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive_IT(hspi_handle, spiIdleTxBuf, rxBuf.data, rxBuf.next_req);
	if (ret == HAL_OK) {
		spiState = SPICMD_WAITING_RX;
		MGR_LOG_DEBUG("%s:: SPI ready, RxSize=%u, State=%u, SR=0x%04X\r\n",
					  __func__, hspi_handle->RxXferSize, hspi_handle->State,
					  (unsigned int)hspi_handle->Instance->SR);
	} else {
		MGR_LOG_DEBUG("%s:: HAL error %d, State=%u, ErrorCode=0x%X\r\n",
					  __func__, ret, hspi_handle->State, (unsigned int)hspi_handle->ErrorCode);
		spiState = SPICMD_ERROR;
		spi_stats.error_count++;
		spi_stats.last_error = hspi_handle->ErrorCode;
	}
	return ret;
}

/**
 * @brief Check if SPI transaction has ended by monitoring RxXferCount stability
 *
 * This function detects when a SPI transaction has completed by checking
 * if the number of received bytes has stopped changing. More reliable than
 * BSY flag in slave mode.
 *
 * @param[out] bytes_received Number of bytes that were received
 * @return true if transaction ended and data is available, false otherwise
 */
bool MCU_SPI_DRIVER_check_transaction_end(uint16_t *bytes_received)
{
	if (hspi_handle == NULL) {
		return false;
	}

	/* Calculate current bytes received */
	uint16_t total_requested = hspi_handle->RxXferSize;
	uint16_t remaining = hspi_handle->RxXferCount;
	uint16_t current_rx_count = total_requested - remaining;

	/* Check if we've received new bytes */
	if (current_rx_count > last_rx_count) {
		/* Activity detected - bytes are being received */
		if (!rx_activity_detected) {
			/* First activity - log HAL state for debugging */
			MGR_LOG_DEBUG("%s:: First RX activity: count=%u, Size=%u, Remain=%u, SR=0x%04X\r\n",
						  __func__, current_rx_count, total_requested, remaining,
						  (unsigned int)hspi_handle->Instance->SR);
		}
		rx_activity_detected = true;
		last_rx_count = current_rx_count;
		rx_stable_start_tick = HAL_GetTick();  /* Reset stability timer */
		return false;
	}

	/* No new bytes received - check if we should consider transaction complete */
	if (rx_activity_detected && current_rx_count > 0) {
		/* We've received some bytes and they've stopped coming */
		if (rx_stable_start_tick == 0) {
			rx_stable_start_tick = HAL_GetTick();
		}

		/* Check if RX count has been stable long enough */
		if ((HAL_GetTick() - rx_stable_start_tick) >= RX_STABLE_TIMEOUT_MS) {
			*bytes_received = current_rx_count;

			MGR_LOG_DEBUG("%s:: Transaction ended, received %u bytes (stable for %ums)\r\n",
						  __func__, *bytes_received, RX_STABLE_TIMEOUT_MS);

			/* Reset state for next transaction */
			last_rx_count = 0;
			rx_stable_start_tick = 0;
			rx_activity_detected = false;

			return true;
		}
	}

	return false;
}

/**
 * @brief Abort current SPI transfer and return to idle state
 *
 * This should be called when a transaction has ended (detected by
 * MCU_SPI_DRIVER_check_transaction_end) to properly terminate the HAL transfer.
 */
void MCU_SPI_DRIVER_abort_transfer(void)
{
	if (hspi_handle == NULL) {
		return;
	}

	/* Abort any ongoing transfer */
	HAL_SPI_Abort_IT(hspi_handle);

	/* Reset state */
	hspi_handle->State = HAL_SPI_STATE_READY;

	/* Reset transaction detection variables */
	last_rx_count = 0;
	rx_stable_start_tick = 0;
	rx_activity_detected = false;
}



bool MCU_SPI_DRIVER_register(void *handle, int8_t (*rx_spi_evt_cb)(SPI_Buffer *rx, SPI_Buffer *tx))
{
	HAL_StatusTypeDef ret = HAL_OK;
	MGR_LOG_DEBUG("%s:: called\r\n", __func__);
    // Check if we save spi handle or if we already have a valid value
	if (handle != NULL && hspi_handle == NULL) // Handle is not set yet
	{
		hspi_handle = (SPI_HandleTypeDef *)handle;
    } else if (handle == NULL && hspi_handle == NULL) // Handle is already set
	{
	    MGR_LOG_DEBUG("%s::ERROR failed to register: invalid hspi ptr \r\n", __func__);
        kns_assert(0);
	}

    // Check if rx_spi_evt_cb is not NULL and well defined
	if (rx_spi_evt_cb != NULL && rxSpiEvtCb == NULL) // Handle is not set yet
	{
        rxSpiEvtCb = rx_spi_evt_cb;
    } else if (rx_spi_evt_cb == NULL && rxSpiEvtCb == NULL) // Handle is already set
	{
	    MGR_LOG_DEBUG("%s::ERROR failed to register: invalid rx_spi_evt_cb ptr \r\n", __func__);
        kns_assert(0);
	}

	// Set SPI OK and TX WAITING flags
	// Request max frame size so we use polling-based idle detection
	// instead of HAL callback firing after each byte
	txBuf.next_req = 0;
	rxBuf.next_req = 255;  /* SPI_FRAME_MAX_SIZE - use polling to detect end */
    ret = MCU_SPI_DRIVER_read();
	// Start receiving the command
    if (ret == HAL_OK) // Want to read the command received
    {
    	return true;
    } else {
    	return false;
    }
}


bool MCU_SPI_DRIVER_reset(SPI_HandleTypeDef *hspi)
{
	// Set SPI OK and TX WAITING flags
	MGR_LOG_DEBUG("%s:: called\r\n", __func__);
	spi_stats.reset_count++;

	if (hspi != NULL && hspi_handle == NULL) // Handle is not set yet
	{
		hspi_handle = (SPI_HandleTypeDef *)hspi;
    } else if ((hspi == NULL) && (hspi_handle == NULL)) // Handle is already set
	{
	    MGR_LOG_DEBUG("%s::ERROR invalid hspi ptr \r\n", __func__);
        kns_assert(0);
	}

    // Check for Overrun Error
    if (hspi_handle->ErrorCode & HAL_SPI_ERROR_OVR) {
        // Clear the Overrun flag
		MGR_LOG_DEBUG("%s:: Clear Overrun flag\r\n", __func__);
        __HAL_SPI_CLEAR_OVRFLAG(hspi_handle);
    }

    // Check for Mode Fault Error
    if (hspi_handle->ErrorCode & HAL_SPI_ERROR_MODF) {
        // Clear the Mode Fault flag
		MGR_LOG_DEBUG("%s:: Clear Mode Fault flag\r\n", __func__);
        __HAL_SPI_CLEAR_MODFFLAG(hspi_handle);
    }

    // Check for Frame Error
    if (hspi_handle->ErrorCode & HAL_SPI_ERROR_FRE) {
        // Clear the Frame Error flag
		MGR_LOG_DEBUG("%s:: Clear Frame Error flag\r\n", __func__);
        __HAL_SPI_CLEAR_FREFLAG(hspi_handle);
    }


    // Step 1: Abort all ongoing SPI transfers
    HAL_StatusTypeDef ret = HAL_OK;
	ret = HAL_SPI_Abort(hspi_handle);
	if (ret != HAL_OK)
	{
	  MGR_LOG_DEBUG("Failed to abort SPI transfers.\r\n");
	}

   // Step 2: Disable the SPI peripheral clock (Force hardware reset)
	__HAL_RCC_SPI1_FORCE_RESET(); // Replace SPI1 with your SPI instance
	//HAL_Delay(1);                 // Short delay for hardware reset
	__HAL_RCC_SPI1_RELEASE_RESET();

	 // Step 3: Clear pending interrupts and reset NVIC
	__HAL_SPI_DISABLE_IT(hspi_handle, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);
	HAL_NVIC_ClearPendingIRQ(SPI1_IRQn); // Replace SPI1_IRQn with your SPI IRQ

	// Step 4: Reset the internal state of the SPI handle
	hspi_handle->State = HAL_SPI_STATE_READY;
	hspi_handle->ErrorCode = HAL_SPI_ERROR_NONE;

	// Step 5: Deinitialize the SPI peripheral
	ret = HAL_SPI_DeInit(hspi_handle);
	if (ret != HAL_OK)
	{
		MGR_LOG_DEBUG("Failed to deinitialize SPI.\r\n");
		return false;
	}

	// Step 6: Reinitialize the SPI peripheral with default configuration
	ret = HAL_SPI_Init(hspi_handle);
	if (ret != HAL_OK)
	{
		MGR_LOG_DEBUG("Failed to reinitialize SPI.\r\n");
		return false;
	}
    __HAL_SPI_ENABLE(hspi_handle);

    return (MCU_SPI_DRIVER_register(hspi_handle, NULL));
}

/**
  * @brief  UART error callback. Can raise in case of UART OVERFLOW, DMA RX ERROR, ...
 *
 * This function is highly based on STM32HAL_UART. It overrides the generic defined error callback
 *
 * @note So far, this callback is emptied
 *
  * @param  huart UART handle.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{

	// Set SPI OK and TX WAITING flags
	MGR_LOG_DEBUG("%s:: called\r\n", __func__);
    bool ret = MCU_SPI_DRIVER_reset(hspi);
    if (!ret) {
        // Retry to register
        MGR_LOG_DEBUG("%s::ERROR Failed to reset SPI_driver...\r\n", __func__);
        kns_assert(0);
    }
}

/**
 * @}
 */
