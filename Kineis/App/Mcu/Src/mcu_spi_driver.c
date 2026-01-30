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
#include "mgr_spi_cmd.h"

/* Defines -------------------------------------------------------------------*/


/* Variables -----------------------------------------------------------------*/
static SPI_HandleTypeDef *hspi_handle = NULL;

/* External SPI interrupt counter from stm32wlxx_it.c */
extern volatile uint32_t spi_irq_count;

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
/* Must be volatile - accessed from both ISR and main context */
static volatile uint16_t last_rx_count = 0;        /* Last observed RxXferCount */
static volatile uint32_t rx_stable_start_tick = 0; /* When RxXferCount stopped changing */
static volatile bool rx_activity_detected = false; /* True if we've received at least one byte */

/* Flag for two-transaction protocol: prevents state change after error recovery */
static volatile bool waiting_for_tx_transaction = false;
/* Consider transaction complete after no new bytes for this duration.
 * At 100kHz SPI clock, one byte takes 80us (8 bits / 100kHz).
 * A 255-byte frame would take ~20ms. But master sends only 5-byte command.
 * After CS goes high, we should detect quickly (within 1 byte time + margin).
 *
 * IMPORTANT: This timeout affects the minimum delay the master must wait
 * before reading the response. Master should wait at least:
 *   RX_STABLE_TIMEOUT_MS + processing_time (~5ms) + margin
 *
 * Increased to 10ms for reliability with slower SPI speeds (125kHz).
 */
#define RX_STABLE_TIMEOUT_MS 10
/* Private function prototypes -----------------------------------------------*/


// /* Functions -----------------------------------------------------------------*/
// Callback when a command is received (for Receive_IT only - not used anymore)
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        /* NOTE: No logging in ISR - would block for 500ms */
        spi_stats.rx_count++;
        if (rxSpiEvtCb != NULL)
        {
        	rxSpiEvtCb(&rxBuf, &txBuf);
			startTickTimeout = 0;
        } else {
            spiState = SPICMD_ERROR;
            spi_stats.error_count++;
        }
    } else {
    	kns_assert(0);
    }
}

// Callback for SPI TxRx completion - used for both read and writeread
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        /* NOTE: No logging in ISR - would block for 500ms */
		startTickTimeout = 0;

		if (spiState == SPICMD_WAITING_RX) {
			/* This was a "read" operation - process received data */
			spi_stats.rx_count++;
			if (rxSpiEvtCb != NULL) {
				rxSpiEvtCb(&rxBuf, &txBuf);
			} else {
				spiState = SPICMD_ERROR;
				spi_stats.error_count++;
			}
		} else if (spiState == SPICMD_WAITING_TX) {
			/* Check if this is a spurious callback from HAL_SPI_Abort during error recovery.
			 * When waiting_for_tx_transaction is true, we're in the middle of re-arming
			 * and should NOT transition to IDLE yet. */
			if (waiting_for_tx_transaction) {
				/* Spurious callback from Abort - stay in WAITING_TX, don't change anything */
				return;
			}
			/* Real completion - response sent, go idle */
			spi_stats.tx_count++;
			spiState = SPICMD_IDLE;
		} else {
			spiState = SPICMD_IDLE;
		}
    } else {
    	kns_assert(0);
    }
}

// Callback not used
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        /* NOTE: No logging in ISR - would block for 500ms */
		startTickTimeout = 0;
		spi_stats.tx_count++;
    } else {
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

	SPI_LOG_VERBOSE("%s:: Sending %u bytes, first=0x%02X\r\n",
				  __func__, txBuf.next_req, txBuf.data[0]);

	/* Reset transaction detection for the new transfer */
	last_rx_count = 0;
	rx_stable_start_tick = 0;
	rx_activity_detected = false;

	/* Clear the two-transaction flag - this is the initial TX setup */
	waiting_for_tx_transaction = false;

	/* Clear any pending error flags from previous transaction to prevent
	 * the error callback from firing immediately after DMA setup */
	if (hspi_handle->ErrorCode != HAL_SPI_ERROR_NONE) {
		if (hspi_handle->ErrorCode & HAL_SPI_ERROR_OVR) {
			__HAL_SPI_CLEAR_OVRFLAG(hspi_handle);
		}
		if (hspi_handle->ErrorCode & HAL_SPI_ERROR_MODF) {
			__HAL_SPI_CLEAR_MODFFLAG(hspi_handle);
		}
		if (hspi_handle->ErrorCode & HAL_SPI_ERROR_FRE) {
			__HAL_SPI_CLEAR_FREFLAG(hspi_handle);
		}
		hspi_handle->ErrorCode = HAL_SPI_ERROR_NONE;
	}

	/* Ensure SPI is in ready state */
	if (hspi_handle->State != HAL_SPI_STATE_READY) {
		HAL_SPI_Abort(hspi_handle);
		hspi_handle->State = HAL_SPI_STATE_READY;
	}

	/* Pad TX buffer with idle pattern after the response data.
	 * The master may read more bytes than the response size.
	 * We configure DMA for a larger size to avoid overrun errors.
	 * Using 0xAA pattern for easy scope identification. */
	uint16_t transfer_size = txBuf.next_req;
	if (transfer_size < 16) {
		/* Minimum 16 bytes to handle master reading extra bytes */
		memset(&txBuf.data[transfer_size], 0xAA, 16 - transfer_size);
		transfer_size = 16;
	} else if (transfer_size < TXBUF_SIZE) {
		/* Pad remaining buffer to prevent sending stale data */
		txBuf.data[transfer_size] = 0xAA;
	}

	/* CRITICAL: Set state BEFORE HAL call to prevent race condition!
	 * If error callback fires during HAL_SPI_TransmitReceive_DMA, it needs
	 * to see WAITING_TX state, otherwise it does a full reset (which sets
	 * up idle pattern 0xAA instead of our response). */
	spiState = SPICMD_WAITING_TX;

	/* Set up response transfer using DMA - master will clock this out */
	ret = HAL_SPI_TransmitReceive_DMA(hspi_handle, txBuf.data, rxBuf.data, transfer_size);
	if (ret == HAL_OK){
		SPI_LOG_VERBOSE("%s:: TX ready at tick=%lu, size=%u, first byte=0x%02X\r\n",
					  __func__, (unsigned long)HAL_GetTick(), txBuf.next_req, txBuf.data[0]);
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

	/* Initialize idle TX buffer with 0xAA pattern only once for efficiency.
	 * 0xAA (10101010) makes it easy to see on a scope and distinguishes from 0x00/0xFF */
	static bool idle_buf_initialized = false;
	if (!idle_buf_initialized) {
		memset(spiIdleTxBuf, 0xAA, RXBUF_SIZE);
		idle_buf_initialized = true;
	}

	/* Reset transaction detection state for new transfer */
	last_rx_count = 0;
	rx_stable_start_tick = 0;
	rx_activity_detected = false;

	/* Ensure SPI is in ready state */
	if (hspi_handle->State != HAL_SPI_STATE_READY) {
		/* Abort any ongoing transfer */
		HAL_SPI_Abort(hspi_handle);
	}

	/* Use DMA for reliable high-speed SPI slave operation.
	 * DMA handles data transfer without CPU intervention, preventing overrun errors
	 * that occur with interrupt-based transfers when master clocks too fast. */
	HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive_DMA(hspi_handle,
		spiIdleTxBuf,   /* TX buffer with 0xAA pattern */
		rxBuf.data,     /* RX buffer */
		rxBuf.next_req);

	if (ret == HAL_OK) {
		spiState = SPICMD_WAITING_RX;
		/* Debug: Check DMA configuration - only log once at startup */
		static bool dma_debug_logged = false;
		if (!dma_debug_logged) {
			MGR_LOG_DEBUG("DMA RX: CNDTR=%lu CPAR=0x%08lX CMAR=0x%08lX CCR=0x%08lX\r\n",
						  (unsigned long)hspi_handle->hdmarx->Instance->CNDTR,
						  (unsigned long)hspi_handle->hdmarx->Instance->CPAR,
						  (unsigned long)hspi_handle->hdmarx->Instance->CMAR,
						  (unsigned long)hspi_handle->hdmarx->Instance->CCR);
			/* Check DMAMUX configuration */
#ifdef DEBUG
			uint32_t dmamux_rx = DMAMUX1_Channel0->CCR;
			uint32_t dmamux_tx = DMAMUX1_Channel1->CCR;
			MGR_LOG_DEBUG("DMAMUX: Ch0(RX)=0x%08lX Ch1(TX)=0x%08lX\r\n",
						  (unsigned long)dmamux_rx, (unsigned long)dmamux_tx);
#endif
			MGR_LOG_DEBUG("SPI: CR1=0x%04X CR2=0x%04X SR=0x%04X\r\n",
						  (unsigned int)hspi_handle->Instance->CR1,
						  (unsigned int)hspi_handle->Instance->CR2,
						  (unsigned int)hspi_handle->Instance->SR);
			dma_debug_logged = true;
		}
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
 * CRITICAL: This function is called in a tight polling loop - NO LOGGING here!
 *
 * @param[out] bytes_received Number of bytes that were received
 * @return true if transaction ended and data is available, false otherwise
 */
bool MCU_SPI_DRIVER_check_transaction_end(uint16_t *bytes_received)
{
	if (hspi_handle == NULL) {
		return false;
	}

	/* Calculate current bytes received from DMA NDTR register.
	 * For DMA mode, RxXferCount is NOT updated during transfer.
	 * The DMA's CNDTR register shows remaining bytes to transfer. */
	uint16_t total_requested = hspi_handle->RxXferSize;
	uint16_t remaining;

	if (hspi_handle->hdmarx != NULL && hspi_handle->hdmarx->Instance != NULL) {
		/* DMA mode: read remaining count from DMA controller */
		remaining = __HAL_DMA_GET_COUNTER(hspi_handle->hdmarx);
	} else {
		/* Interrupt mode fallback or DMA not configured */
		remaining = hspi_handle->RxXferCount;
	}

	/* Protect against underflow if remaining > total_requested (shouldn't happen) */
	uint16_t current_rx_count = (remaining <= total_requested) ?
		(total_requested - remaining) : 0;

	/* Check if we've received new bytes - NO logging in this hot path */
	if (current_rx_count > last_rx_count) {
		/* Activity detected - bytes are being received */
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

	/* Abort any ongoing DMA transfer */
	HAL_SPI_Abort(hspi_handle);

	/* Reset state */
	hspi_handle->State = HAL_SPI_STATE_READY;

	/* Reset transaction detection variables */
	last_rx_count = 0;
	rx_stable_start_tick = 0;
	rx_activity_detected = false;
}

/**
 * @brief Check if TX transaction has completed for two-transaction protocol.
 *
 * In the two-transaction protocol:
 * 1. Master sends command (transaction 1)
 * 2. STM32 processes and prepares response, error callback re-arms TX
 * 3. Master clocks to read response (transaction 2)
 *
 * This function polls for transaction 2 completion by checking if the
 * IT/DMA transfer finished (HAL_SPI_STATE_READY).
 *
 * @return true if TX transaction completed, false if still waiting.
 */
bool MCU_SPI_DRIVER_check_tx_complete(void)
{
	if (hspi_handle == NULL) {
		return false;
	}

	/* Only applicable if we're waiting for transaction 2 */
	if (!waiting_for_tx_transaction) {
		return false;
	}

	/* Check if the SPI transfer completed.
	 * When using IT mode, the HAL state goes to READY when transfer completes.
	 * The TxRxCpltCallback was ignored due to the flag, but the HAL state
	 * still updates. */
	if (hspi_handle->State == HAL_SPI_STATE_READY) {
		/* Transaction 2 completed - response was sent */
		SPI_LOG_VERBOSE("%s:: TX transaction 2 completed\r\n", __func__);
		spi_stats.tx_count++;
		waiting_for_tx_transaction = false;
		return true;
	}

	return false;
}


bool MCU_SPI_DRIVER_register(void *handle, int8_t (*rx_spi_evt_cb)(SPI_Buffer *rx, SPI_Buffer *tx))
{
	HAL_StatusTypeDef ret = HAL_OK;
	MGR_LOG_DEBUG("%s:: called\r\n", __func__);
    // Check if we save spi handle or if we already have a valid value
	if (handle != NULL && hspi_handle == NULL) // Handle is not set yet
	{
		hspi_handle = (SPI_HandleTypeDef *)handle;

		/* Diagnostic: Verify SPI and GPIO configuration */
		MGR_LOG_DEBUG("%s:: SPI1 CR1=0x%04X, CR2=0x%04X, SR=0x%04X\r\n",
					  __func__,
					  (unsigned int)hspi_handle->Instance->CR1,
					  (unsigned int)hspi_handle->Instance->CR2,
					  (unsigned int)hspi_handle->Instance->SR);

		/* Check GPIO mode registers for SPI pins
		 * MODER: 00=Input, 01=Output, 10=Alternate Function, 11=Analog
		 * PB4 (MISO) should be 10 (AF), PB5 (MOSI) should be 10 (AF)
		 * PA1 (SCK) should be 10 (AF), PA15 (NSS) should be 10 (AF) */
#ifdef DEBUG
		uint32_t gpiob_moder = GPIOB->MODER;
		uint32_t gpioa_moder = GPIOA->MODER;
		uint8_t pb4_mode = (gpiob_moder >> (4 * 2)) & 0x3;  /* MISO */
		uint8_t pb5_mode = (gpiob_moder >> (5 * 2)) & 0x3;  /* MOSI */
		uint8_t pa1_mode = (gpioa_moder >> (1 * 2)) & 0x3;  /* SCK */
		uint8_t pa15_mode = (gpioa_moder >> (15 * 2)) & 0x3; /* NSS */

		MGR_LOG_DEBUG("%s:: GPIO: MISO(PB4)=%u, MOSI(PB5)=%u, SCK(PA1)=%u, NSS(PA15)=%u (2=AF)\r\n",
					  __func__, pb4_mode, pb5_mode, pa1_mode, pa15_mode);
#endif
		/* Check GPIO clock status */
#ifdef DEBUG
		uint32_t rcc_ahb2enr = RCC->AHB2ENR;
		MGR_LOG_DEBUG("%s:: RCC_AHB2ENR=0x%08X (GPIOA=%d, GPIOB=%d)\r\n",
					  __func__, (unsigned int)rcc_ahb2enr,
					  (rcc_ahb2enr & RCC_AHB2ENR_GPIOAEN) ? 1 : 0,
					  (rcc_ahb2enr & RCC_AHB2ENR_GPIOBEN) ? 1 : 0);
#endif
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
  * @brief  SPI error callback. Handles overrun, mode fault, and frame errors.
  *
  * For two-transaction protocol, errors during WAITING_RX or WAITING_TX are normal
  * (occur when CS goes HIGH). We clear errors but don't reset to preserve received data.
  *
  * @param  hspi SPI handle.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	SPI_LOG_VERBOSE("%s:: called, state=%d, error=0x%X\r\n", __func__, spiState, (unsigned int)hspi->ErrorCode);

	/* For two-transaction protocol, errors during WAITING_RX or WAITING_TX are NORMAL.
	 * They typically occur when CS goes HIGH (end of transaction 1) while DMA was
	 * expecting more bytes. We must NOT reset - just clear errors and let the
	 * polling mechanism (check_transaction_end) detect the end and process data.
	 *
	 * CRITICAL: A full reset here would LOSE the received data!
	 *
	 * Two-transaction protocol flow:
	 * 1. Master sends command (transaction 1) - state = WAITING_RX
	 * 2. CS goes HIGH → error callback fires (overrun)
	 * 3. Main loop polling detects end, processes command, prepares response
	 * 4. Master reads response (transaction 2) - state = WAITING_TX
	 */
	if (spiState == SPICMD_WAITING_RX || spiState == SPICMD_WAITING_TX) {
		/* Clear error flags but DO NOT reset or abort */
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

		SPI_LOG_VERBOSE("%s:: %s - errors cleared, continuing\r\n", __func__,
			spiState == SPICMD_WAITING_RX ? "WAITING_RX" : "WAITING_TX");
		return;
	}

	/* For other states (IDLE, ERROR, PROCESS_CMD), do a full reset */
	bool ret = MCU_SPI_DRIVER_reset(hspi);
	if (!ret) {
		MGR_LOG_DEBUG("%s::ERROR Failed to reset SPI_driver, attempting recovery...\r\n", __func__);
		/* Instead of fatal assert, try to recover by clearing errors */
		hspi->ErrorCode = HAL_SPI_ERROR_NONE;
		hspi->State = HAL_SPI_STATE_READY;
		spiState = SPICMD_ERROR;
		spi_stats.error_count++;
	}
}

/**
 * @}
 */
