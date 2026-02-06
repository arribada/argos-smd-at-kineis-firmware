// SPDX-License-Identifier: no SPDX license
/**
 * @file    mcu_at_console.c
 * @author  Kinéis
 * @brief   MCU wrapper for AT CMD console
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
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "kns_app_conf.h" // for STM32 HAL include on UART
#include STM32_HAL_H
#include "kineis_sw_conf.h" // for assert include below
#include KINEIS_SW_ASSERT_H
#include "mgr_log.h"  /* For unified UART output through ring buffer */

/* Defines -------------------------------------------------------------------*/
#if defined(STM32WLE5xx) || defined(STM32WL55xx)
#define USART_ISR_RXNE USART_ISR_RXNE_RXFNE
#endif

#ifdef USE_HDA4
#define TXBUF_SIZE 2560
#define RXBUF_SIZE 2560
#else
#define TXBUF_SIZE 256
#define RXBUF_SIZE 256
#endif

/* Variables -----------------------------------------------------------------*/

static UART_HandleTypeDef *huart_handle; /** This AT console needs some UART link */
static char uartTxBuf[TXBUF_SIZE];
static uint8_t uartRxBuf[RXBUF_SIZE];

static bool (*rxEvtCb)(uint8_t *pu8_RxBuffer, int16_t *pi16_nbRxValidChar);

/* Private function prototypes -----------------------------------------------*/

// Functions that could be used to (potentially) remove string.h :
//unsigned int strlen(const char *str)
//{
//    uint16_t nb_char;
//
//    for (nb_char = 0; nb_char < 65535 ; nb_char++)
//        if (str[nb_char] == '\0')
//            break;
//    return nb_char;
//}
//
//void *memset(void *str, int val, unsigned int size)
//{
//    unsigned int i;
//
//    for(i=0 ; i<size ; i++)
//        ((char*)str)[i] = val;
//    return str;
//}

/** @brief RX interrupt handler for 7 or 8 bits data word length .
 *
 * This function is highly based on STM32HAL_UART expect callback is called at each received
 * character. This allow to treat RX received characters as a continuous stream.
 *
 * At each received character, the callback is said to consume data, it updates the buffer pointer
 * and the number-of-valid-char reduced by what was consummed.
 *
 * @note In case of overflow on RX buffer, force write from the beginning of the buffer pRxBuffPtr.
 *       thus, the entire buffer may be lost. Actually, such case may only happen in case:
 *       * user sent too much data in console, this is not an expected scenario
 *       * user has sent some garbage before its several AT cmds, leading accumuation to overflow.
 *       This is not an expected behaviour as well.
 *
 * @todo circular buffer may be implemented instead of overflowing sometimes.
 *
 * STM32HAL_UART base function prototype:
 @verbatim
static void UART_RxISR_8BIT(UART_HandleTypeDef *huart)
 @endverbatim
 *
 * @param huart UART handle.
 * @retval None
 */
static void KINEIS_RxISR_8BIT(UART_HandleTypeDef *huart)
{
	uint16_t uhMask = huart->Mask;
	uint16_t  uhdata;
	uint8_t *pu8_RxBuffer = NULL;
	int16_t i16_nbRxValidChar = 0;

	i16_nbRxValidChar = huart->RxXferSize - huart->RxXferCount;
	pu8_RxBuffer = huart->pRxBuffPtr - i16_nbRxValidChar;
	//< pu8_RxBuffer points to the 1st element of the RX array

	/* Check that a Rx process is ongoing */
	if (huart->RxState == HAL_UART_STATE_BUSY_RX) {
		/* consume all incomming data in same interrupt. RXNE is set by HW and cleared by
		 * reading RDR reg
		 */
		while ((READ_REG(huart->Instance->ISR) & USART_ISR_RXNE) != 0U) {
			uhdata = (uint16_t) READ_REG(huart->Instance->RDR);
			*huart->pRxBuffPtr = (uint8_t)(uhdata & (uint8_t)uhMask);

			/* In case of overflow, force writing from the beginning of the buffer
			 */
			if (huart->RxXferCount == 1U) {
				huart->pRxBuffPtr = huart->pRxBuffPtr - huart->RxXferSize +
							huart->RxXferCount;
				huart->RxXferCount = huart->RxXferSize;
			} else {
				huart->pRxBuffPtr++;
				huart->RxXferCount--;
			}
		}

		/* Should raise an os_event to main task/function */
		/* \todo implement call back mechanism to raise os event instead of using
		 * global variables directly.
		 */

		if (rxEvtCb != NULL) {
			i16_nbRxValidChar = huart->RxXferSize - huart->RxXferCount;
			pu8_RxBuffer = huart->pRxBuffPtr - i16_nbRxValidChar;
			//< pu8_RxBuffer points to the 1st element of the RX array
			while (rxEvtCb(pu8_RxBuffer, &i16_nbRxValidChar) == true) {
				// empty loop
			};
			//< realign huart to the beginning of found AT cmd
			huart->pRxBuffPtr = pu8_RxBuffer + i16_nbRxValidChar;
			huart->RxXferCount = huart->RxXferSize - i16_nbRxValidChar;
		}
	} else {
		/* Clear RXNE interrupt flag */
		__HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
	}
}

/** @brief Enable and start RX interrupt from UART
 *
 * This function is highly based on STM32HAL_UART expect it called internal interrupt handler
 * instead of generic one.
 *
  @verbatim
HAL_StatusTypeDef UART_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
  @endverbatim
 * @param huart UART handle.
 * @param pData Pointer to data buffer (u8 or u16 data elements).
 * @param Size  Amount of data elements (u8 or u16) to be received.
 * @retval HAL status
 */
static HAL_StatusTypeDef KINEIS_UART_StartRx_IT(UART_HandleTypeDef *huart, uint8_t *pData,
						uint16_t Size)
{
	if (huart ==  NULL)
		return HAL_ERROR;

	/* Check that a Rx process is not already ongoing */
	if (huart->RxState == HAL_UART_STATE_READY) {
		if ((pData == NULL) || (Size == 0U))
			return HAL_ERROR;

		__HAL_LOCK(huart);

		huart->pRxBuffPtr  = pData;
		huart->RxXferSize  = Size;
		huart->RxXferCount = Size;
		huart->RxISR       = NULL;

		/* Computation of UART mask to apply to RDR register */
		UART_MASK_COMPUTATION(huart);

		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_BUSY_RX;

		/* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
		SET_BIT(huart->Instance->CR3, USART_CR3_EIE);

		/* Set the Rx ISR function pointer according to the data word length */
		if ((huart->Init.WordLength != UART_WORDLENGTH_9B) ||
			(huart->Init.Parity != UART_PARITY_NONE)) {
			huart->RxISR = KINEIS_RxISR_8BIT;
		}

		__HAL_UNLOCK(huart);

		/* Enable the UART Parity Error interrupt and Data Register Not Empty interrupt */
#if defined(STM32WLE5xx) || defined(STM32WL55xx)
		ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE_RXFNEIE);
#else
		SET_BIT(huart->Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
#endif
		return HAL_OK;
	} else {
		return HAL_BUSY;
	}
}

/* Functions -----------------------------------------------------------------*/

bool MCU_AT_CONSOLE_register(void *handle, bool (*rx_evt_cb)(uint8_t *pu8_RxBuffer,
	int16_t *pi16_nbRxValidChar))
{
	huart_handle = (UART_HandleTypeDef *)handle;

	if (KINEIS_UART_StartRx_IT(huart_handle, uartRxBuf, sizeof(uartRxBuf)) == HAL_OK) {
		rxEvtCb = rx_evt_cb;
		return true;
	} else
		return false;
}

void MCU_AT_CONSOLE_send(const char *format, ...)
{
	va_list args;
	int written;

	/* Reset output buffer */
	memset(uartTxBuf, 0x00, sizeof(uartTxBuf));

	va_start(args, format);
	/* CRITICAL FIX: Use vsnprintf to prevent buffer overflow */
	written = vsnprintf(uartTxBuf, sizeof(uartTxBuf), format, args);
	va_end(args);

	/* Check for truncation or error */
	if (written < 0) {
		/* Encoding error - send error message instead */
		const char *err_msg = "+ERROR=FORMAT\r\n";
		if (huart_handle != NULL) {
			MGR_LOG_pause();  /* Pause debug logs - AT has priority */
			HAL_UART_Transmit(huart_handle, (uint8_t *)err_msg, strlen(err_msg), 500);
			MGR_LOG_resume();
		}
		return;
	}
	if ((size_t)written >= sizeof(uartTxBuf)) {
		/* Output was truncated - ensure null termination and continue */
		uartTxBuf[sizeof(uartTxBuf) - 1] = '\0';
		written = sizeof(uartTxBuf) - 1;
	}

	/* AT UART PRIORITY: Pause debug logs while sending AT response.
	 * This ensures AT responses are never interrupted by debug logs.
	 * Debug logs remain in the ring buffer and will be sent later. */
	MGR_LOG_pause();

	/* Send AT response via UART - has absolute priority */
	if (huart_handle != NULL) {
		HAL_UART_Transmit(huart_handle, (uint8_t *)uartTxBuf, (uint16_t)written, 500);
	}

	MGR_LOG_resume();
	/* Note: Removed assert for NULL handle - silent fail is safer than crash */
}

void MCU_AT_CONSOLE_send_dataBuf(uint8_t *pu8_inDataBuff, uint16_t u16_dataLenBit)
{
	uint16_t u16_remainingBits;
	const uint16_t u16_dataLenByte_trunc = u16_dataLenBit >> 3;
	uint8_t *pu8_hex;

	/*
	 * Convert data hex to ascii per byte then last byte is
	 * treated per nibble
	 */

	/* Pause logs once for the entire data buffer transfer */
	bool was_paused = MGR_LOG_is_paused();
	if (!was_paused) {
		MGR_LOG_pause();
	}

	for (pu8_hex = pu8_inDataBuff;
			(pu8_hex < (pu8_inDataBuff + u16_dataLenByte_trunc));
			pu8_hex++)
		MCU_AT_CONSOLE_send("%02X", *pu8_hex);

	u16_remainingBits = u16_dataLenBit - (u16_dataLenByte_trunc << 3);
	if (u16_remainingBits > 4)
		MCU_AT_CONSOLE_send("%02X", *pu8_hex);
	else if (u16_remainingBits > 0)
		MCU_AT_CONSOLE_send("%01X", *pu8_hex);
	/* else no additional bits */

	/* Resume logs only if we paused them */
	if (!was_paused) {
		MGR_LOG_resume();
	}
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
//void HAL_UART_ErrorCallback(__attribute__((unused)) UART_HandleTypeDef *huart)
//{
//	MCU_AT_CONSOLE_send("\r\n+UART_ERROR (OVR, RX error) => PLEASE RESET MODULE.\r\n\r\n");
//	kns_assert(0);
//}

/**
 * @}
 */
