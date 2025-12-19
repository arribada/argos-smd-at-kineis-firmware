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
#include "mgr_spi_cmd.h"  /* For spiState extern */

/* Defines -------------------------------------------------------------------*/
#define SPI_MAX_RETRY_COUNT     3       /* Max retries before hard reset */
#define SPI_BUSY_WAIT_CYCLES    10000   /* Busy-wait cycles for short delays */

/* Variables -----------------------------------------------------------------*/
static SPI_HandleTypeDef *hspi_handle = NULL;
static uint8_t spi_error_count = 0;     /* Track consecutive errors for recovery */

static uint8_t spiTxBuf[TXBUF_SIZE];
static uint8_t spiRxBuf[RXBUF_SIZE];
SPI_Buffer rxBuf = { .data = spiRxBuf, .size = RXBUF_SIZE, .next_req = 1};
SPI_Buffer txBuf = { .data = spiTxBuf, .size = TXBUF_SIZE, .next_req = 0};


static int8_t (*rxSpiEvtCb)(SPI_Buffer *rx, SPI_Buffer *tx) = NULL;

uint32_t startTickTimeout = 0;
/* Private function prototypes -----------------------------------------------*/
static inline void MCU_SPI_DRIVER_onSuccess(void);

/* Functions -----------------------------------------------------------------*/

/**
 * @brief Reset counters on successful SPI transaction
 * @note Inline for fast ISR execution
 */
static inline void MCU_SPI_DRIVER_onSuccess(void)
{
	spi_error_count = 0;
	startTickTimeout = 0;
}

/**
 * @brief RX complete callback
 * @note Called from ISR context
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance != SPI1) {
		return;
	}

	MCU_SPI_DRIVER_onSuccess();

	if (rxSpiEvtCb != NULL) {
		rxSpiEvtCb(&rxBuf, &txBuf);
	} else {
		MGR_LOG_DEBUG("%s:: rxSpiEvtCb not defined\r\n", __func__);
		spiState = SPICMD_ERROR;
	}
}

/**
 * @brief TX-RX complete callback
 * @note Called from ISR context
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance != SPI1) {
		return;
	}

	MCU_SPI_DRIVER_onSuccess();
	spiState = SPICMD_IDLE;
}

/**
 * @brief TX complete callback
 * @note Called from ISR context
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance != SPI1) {
		return;
	}

	MCU_SPI_DRIVER_onSuccess();
}

HAL_StatusTypeDef MCU_SPI_DRIVER_writeread()
{
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t retry = 0;

	/* Validate handle */
	if (hspi_handle == NULL) {
		MGR_LOG_DEBUG("%s:: ERROR hspi_handle is NULL\r\n", __func__);
		spiState = SPICMD_ERROR;
		return HAL_ERROR;
	}

	/* Check if SPI is ready, abort if busy */
	if (hspi_handle->State != HAL_SPI_STATE_READY) {
		MGR_LOG_DEBUG("%s:: SPI busy (state=%d), aborting...\r\n", __func__, hspi_handle->State);
		HAL_SPI_Abort_IT(hspi_handle);
		/* Brief busy-wait for abort to complete */
		for (volatile uint32_t i = 0; i < SPI_BUSY_WAIT_CYCLES; i++) { __NOP(); }
	}

	/* Retry mechanism */
	while (retry < SPI_MAX_RETRY_COUNT) {
		ret = HAL_SPI_TransmitReceive_IT(hspi_handle, txBuf.data, rxBuf.data, txBuf.next_req);
		if (ret == HAL_OK) {
			spiState = SPICMD_WAITING_TX;
			spi_error_count = 0;
			return ret;
		}
		retry++;
		MGR_LOG_DEBUG("%s:: Retry %u/%u\r\n", __func__, retry, SPI_MAX_RETRY_COUNT);
	}

	MGR_LOG_DEBUG("%s:: Failed after %u retries\r\n", __func__, SPI_MAX_RETRY_COUNT);
	spiState = SPICMD_ERROR;
	spi_error_count++;
	return ret;
}

HAL_StatusTypeDef MCU_SPI_DRIVER_read()
{
	HAL_StatusTypeDef ret = HAL_OK;
	uint8_t retry = 0;

	/* Validate handle */
	if (hspi_handle == NULL) {
		MGR_LOG_DEBUG("%s:: ERROR hspi_handle is NULL\r\n", __func__);
		spiState = SPICMD_ERROR;
		return HAL_ERROR;
	}

	/* Validate request size */
	if (rxBuf.next_req < 1) {
		MGR_LOG_DEBUG("%s:: Waiting RX is %u should be greater than 0\r\n", __func__, rxBuf.next_req);
		rxBuf.next_req = 1;
	}

	/* Check if SPI is ready, abort if busy */
	if (hspi_handle->State != HAL_SPI_STATE_READY) {
		MGR_LOG_DEBUG("%s:: SPI busy (state=%d), aborting...\r\n", __func__, hspi_handle->State);
		HAL_SPI_Abort_IT(hspi_handle);
		/* Brief busy-wait for abort to complete */
		for (volatile uint32_t i = 0; i < SPI_BUSY_WAIT_CYCLES; i++) { __NOP(); }
	}

	/* Retry mechanism */
	while (retry < SPI_MAX_RETRY_COUNT) {
		ret = HAL_SPI_Receive_IT(hspi_handle, rxBuf.data, rxBuf.next_req);
		if (ret == HAL_OK) {
			spiState = SPICMD_WAITING_RX;
			spi_error_count = 0;
			return ret;
		}
		retry++;
		MGR_LOG_DEBUG("%s:: Retry %u/%u\r\n", __func__, retry, SPI_MAX_RETRY_COUNT);
	}

	MGR_LOG_DEBUG("%s:: Failed after %u retries\r\n", __func__, SPI_MAX_RETRY_COUNT);
	spiState = SPICMD_ERROR;
	spi_error_count++;
	return ret;
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
	txBuf.next_req = 0;
	rxBuf.next_req = 1;
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
	HAL_StatusTypeDef ret = HAL_OK;

	MGR_LOG_DEBUG("%s:: called (error_count=%u)\r\n", __func__, spi_error_count);

	/* Use provided handle or existing one */
	if (hspi != NULL) {
		hspi_handle = hspi;
	}

	/* Check we have a valid handle */
	if (hspi_handle == NULL) {
		MGR_LOG_DEBUG("%s::ERROR invalid hspi ptr\r\n", __func__);
		return false;
	}

	/* Disable interrupts during reset to prevent race conditions */
	__disable_irq();

	/* Log and clear all error flags */
	if (hspi_handle->ErrorCode != HAL_SPI_ERROR_NONE) {
		MGR_LOG_DEBUG("%s:: ErrorCode=0x%lX\r\n", __func__, hspi_handle->ErrorCode);

		if (hspi_handle->ErrorCode & HAL_SPI_ERROR_OVR) {
			MGR_LOG_DEBUG("%s:: Clear Overrun flag\r\n", __func__);
			__HAL_SPI_CLEAR_OVRFLAG(hspi_handle);
		}
		if (hspi_handle->ErrorCode & HAL_SPI_ERROR_MODF) {
			MGR_LOG_DEBUG("%s:: Clear Mode Fault flag\r\n", __func__);
			__HAL_SPI_CLEAR_MODFFLAG(hspi_handle);
		}
		if (hspi_handle->ErrorCode & HAL_SPI_ERROR_FRE) {
			MGR_LOG_DEBUG("%s:: Clear Frame Error flag\r\n", __func__);
			__HAL_SPI_CLEAR_FREFLAG(hspi_handle);
		}
	}

	/* Step 1: Abort all ongoing SPI transfers */
	ret = HAL_SPI_Abort(hspi_handle);
	if (ret != HAL_OK) {
		MGR_LOG_DEBUG("Failed to abort SPI transfers (ret=%d)\r\n", ret);
	}

	/* Step 2: Force hardware reset via RCC */
	__HAL_RCC_SPI1_FORCE_RESET();
	/* Brief busy-wait delay */
	for (volatile uint32_t i = 0; i < SPI_BUSY_WAIT_CYCLES; i++) { __NOP(); }
	__HAL_RCC_SPI1_RELEASE_RESET();

	/* Step 3: Clear pending interrupts and reset NVIC */
	__HAL_SPI_DISABLE_IT(hspi_handle, SPI_IT_RXNE | SPI_IT_TXE | SPI_IT_ERR);
	HAL_NVIC_ClearPendingIRQ(SPI1_IRQn);

	/* Step 4: Reset the internal state of the SPI handle */
	hspi_handle->State = HAL_SPI_STATE_READY;
	hspi_handle->ErrorCode = HAL_SPI_ERROR_NONE;
	hspi_handle->TxXferCount = 0;
	hspi_handle->RxXferCount = 0;

	/* Step 5: Deinitialize the SPI peripheral */
	ret = HAL_SPI_DeInit(hspi_handle);
	if (ret != HAL_OK) {
		MGR_LOG_DEBUG("Failed to deinitialize SPI (ret=%d)\r\n", ret);
		__enable_irq();
		return false;
	}

	/* Step 6: Reinitialize the SPI peripheral */
	ret = HAL_SPI_Init(hspi_handle);
	if (ret != HAL_OK) {
		MGR_LOG_DEBUG("Failed to reinitialize SPI (ret=%d)\r\n", ret);
		__enable_irq();
		return false;
	}

	__HAL_SPI_ENABLE(hspi_handle);

	/* Re-enable interrupts */
	__enable_irq();

	/* Reset error counter on successful reset */
	spi_error_count = 0;

	/* Reset buffer states */
	txBuf.next_req = 0;
	rxBuf.next_req = 1;

	MGR_LOG_DEBUG("%s:: Reset complete, restarting RX\r\n", __func__);

	/* Restart RX listening */
	return MCU_SPI_DRIVER_register(hspi_handle, NULL);
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
