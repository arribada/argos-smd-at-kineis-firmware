/**
 * @file    bl_spi.c
 * @brief   SPI communication driver for bootloader DFU (slave mode with DMA)
 * @date    2025
 *
 * This driver uses the proper HAL pattern with HAL_SPI_MspInit callback
 * for reliable SPI slave operation. Matches the working APP SPI exactly.
 */

#include "bl_spi.h"
#include "bl_spi_protocol.h"
#include "bl_config.h"
#include "stm32wlxx_hal.h"
#include <string.h>

/*******************************************************************************
 * HANDLES - Must be accessible for IRQ handlers
 ******************************************************************************/

/* SPI handle - global for HAL callbacks */
SPI_HandleTypeDef hspi1_bl;

/* DMA handles - global for HAL callbacks */
DMA_HandleTypeDef hdma_spi1_rx_bl;
DMA_HandleTypeDef hdma_spi1_tx_bl;

/*******************************************************************************
 * PRIVATE VARIABLES
 ******************************************************************************/

/* RX buffer and state */
static uint8_t rx_buffer[BL_RX_BUFFER_SIZE];
static volatile uint16_t rx_count = 0;
static volatile bool rx_complete = false;

/* TX buffers */
static uint8_t tx_buffer[BL_TX_BUFFER_SIZE];
static uint8_t tx_idle_buffer[BL_RX_BUFFER_SIZE];
static uint16_t tx_len = 0;

/* Command parsing state */
static uint8_t current_cmd = 0;
static uint8_t payload_buffer[BL_CHUNK_SIZE];
static uint16_t payload_len = 0;
static volatile bool cmd_ready = false;

/* State for multi-transaction commands (WRITE_REQ -> WRITE_DATA) */
static uint32_t pending_write_addr = 0;
static uint16_t pending_write_len = 0;
static bool write_req_pending = false;

/* State for multi-transaction commands (READ_REQ -> READ_DATA) */
static uint8_t pending_read_buffer[BL_CHUNK_SIZE];
static uint16_t pending_read_len = 0;
static bool read_data_pending = false;

/* Transaction detection state */
static volatile uint16_t last_rx_count = 0;
static volatile uint32_t rx_stable_start_tick = 0;
static volatile bool rx_activity_detected = false;
static volatile uint16_t expected_transfer_size = 0;
static volatile bool dma_transfer_complete = false;

/* Transaction sizes - must match Zephyr driver */
#define SPI_RX_TRANSACTION_SIZE    BL_SPI_TRANSACTION_SIZE  /* 280 bytes */
#define SPI_TX_TRANSACTION_SIZE    64  /* Zephyr NOP poll size */

/* Stable timeout for transaction detection */
#define RX_STABLE_TIMEOUT_MS    BL_SPI_RX_STABLE_TIMEOUT_MS

/* SPI state machine */
typedef enum {
    BL_SPI_STATE_IDLE,
    BL_SPI_STATE_WAITING_RX,
    BL_SPI_STATE_PROCESSING,
    BL_SPI_STATE_WAITING_TX
} bl_spi_state_t;

static volatile bl_spi_state_t spi_state = BL_SPI_STATE_IDLE;

/*******************************************************************************
 * EXTERNAL DEBUG FUNCTIONS
 ******************************************************************************/

extern void early_debug_print(const char *str);

static void spi_debug_hex8(uint8_t val)
{
    char hex[3];
    uint8_t nib = (val >> 4) & 0xF;
    hex[0] = (nib < 10) ? ('0' + nib) : ('A' + nib - 10);
    nib = val & 0xF;
    hex[1] = (nib < 10) ? ('0' + nib) : ('A' + nib - 10);
    hex[2] = '\0';
    early_debug_print(hex);
}

static void spi_print_hex32(uint32_t val)
{
    char hex[9];
    for (int i = 7; i >= 0; i--) {
        uint8_t nib = (val >> (i * 4)) & 0xF;
        hex[7-i] = (nib < 10) ? ('0' + nib) : ('A' + nib - 10);
    }
    hex[8] = '\0';
    early_debug_print(hex);
}

/*******************************************************************************
 * HAL MSP CALLBACKS - Called automatically by HAL_SPI_Init()
 * This is the CRITICAL difference from the old bootloader code.
 * The APP SPI works because it uses this pattern properly.
 ******************************************************************************/

/**
 * @brief SPI MSP Initialization - Called by HAL_SPI_Init()
 *
 * This callback configures:
 * 1. Clocks (SPI, GPIO, DMA, DMAMUX)
 * 2. GPIO pins in Alternate Function mode
 * 3. DMA channels with proper DMAMUX routing
 * 4. NVIC priorities and enables
 *
 * IMPORTANT: This runs BEFORE HAL_SPI_Init() returns, ensuring
 * DMA is properly linked when the first transaction starts.
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (spiHandle->Instance == SPI1) {
        early_debug_print("[SPI] MspInit: Enabling clocks\r\n");

        /* Enable clocks - MUST be done first */
        __HAL_RCC_SPI1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();
        __HAL_RCC_DMAMUX1_CLK_ENABLE();

        /* CRITICAL: Reset DMA1 to clear stale state from APP jump.
         * After direct jump (no system reset), DMA/DMAMUX keep old config.
         * Without reset, HAL_DMA_Init may not properly reconfigure DMAMUX. */
        early_debug_print("[SPI] MspInit: Resetting DMA1\r\n");
        __HAL_RCC_DMA1_FORCE_RESET();
        for (volatile int i = 0; i < 100; i++);
        __HAL_RCC_DMA1_RELEASE_RESET();

        /* Configure SPI1 GPIO:
         * PA1  - SPI1_SCK  (AF5)
         * PA15 - SPI1_NSS  (AF5)
         * PB4  - SPI1_MISO (AF5)
         * PB5  - SPI1_MOSI (AF5)
         */
        GPIO_InitStruct.Pin = GPIO_PIN_15 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        early_debug_print("[SPI] MspInit: Configuring DMA\r\n");

        /* SPI1_RX DMA Init - DMA1 Channel 1 */
        hdma_spi1_rx_bl.Instance = DMA1_Channel1;
        hdma_spi1_rx_bl.Init.Request = DMA_REQUEST_SPI1_RX;
        hdma_spi1_rx_bl.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_spi1_rx_bl.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi1_rx_bl.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi1_rx_bl.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_rx_bl.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi1_rx_bl.Init.Mode = DMA_NORMAL;
        hdma_spi1_rx_bl.Init.Priority = DMA_PRIORITY_HIGH;
        if (HAL_DMA_Init(&hdma_spi1_rx_bl) != HAL_OK) {
            early_debug_print("[SPI] MspInit: DMA RX init FAILED\r\n");
            return;
        }

        /* VERIFY DMAMUX is correctly configured for SPI1_RX (request ID = 10).
         * After direct APP->BL jump, DMAMUX may have stale config. */
        if ((DMAMUX1_Channel0->CCR & DMAMUX_CxCR_DMAREQ_ID) != DMA_REQUEST_SPI1_RX) {
            early_debug_print("[SPI] MspInit: DMAMUX0 wrong! Forcing SPI1_RX\r\n");
            DMAMUX1_Channel0->CCR = DMA_REQUEST_SPI1_RX;
        }

        /* Link DMA to SPI handle - CRITICAL for HAL_SPI_TransmitReceive_DMA */
        __HAL_LINKDMA(spiHandle, hdmarx, hdma_spi1_rx_bl);

        /* SPI1_TX DMA Init - DMA1 Channel 2 */
        hdma_spi1_tx_bl.Instance = DMA1_Channel2;
        hdma_spi1_tx_bl.Init.Request = DMA_REQUEST_SPI1_TX;
        hdma_spi1_tx_bl.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_spi1_tx_bl.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_spi1_tx_bl.Init.MemInc = DMA_MINC_ENABLE;
        hdma_spi1_tx_bl.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_spi1_tx_bl.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_spi1_tx_bl.Init.Mode = DMA_NORMAL;
        hdma_spi1_tx_bl.Init.Priority = DMA_PRIORITY_HIGH;
        if (HAL_DMA_Init(&hdma_spi1_tx_bl) != HAL_OK) {
            early_debug_print("[SPI] MspInit: DMA TX init FAILED\r\n");
            return;
        }

        /* VERIFY DMAMUX is correctly configured for SPI1_TX (request ID = 11) */
        if ((DMAMUX1_Channel1->CCR & DMAMUX_CxCR_DMAREQ_ID) != DMA_REQUEST_SPI1_TX) {
            early_debug_print("[SPI] MspInit: DMAMUX1 wrong! Forcing SPI1_TX\r\n");
            DMAMUX1_Channel1->CCR = DMA_REQUEST_SPI1_TX;
        }

        __HAL_LINKDMA(spiHandle, hdmatx, hdma_spi1_tx_bl);

        early_debug_print("[SPI] MspInit: Configuring NVIC\r\n");

        /* DMA interrupt init - Priority 2 (same as APP) */
        HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
        HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

        /* SPI1 interrupt Init - Priority 2 (same as APP) */
        HAL_NVIC_SetPriority(SPI1_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);

        /* CRITICAL FIX: Force DMAMUX configuration UNCONDITIONALLY.
         * HAL_DMA_Init writes to hdma->DMAmuxChannel which may point to wrong channel.
         * We MUST ensure DMAMUX channels are correctly mapped:
         * - DMAMUX1_Channel0 (for DMA1_Ch1) = SPI1_RX (10)
         * - DMAMUX1_Channel1 (for DMA1_Ch2) = SPI1_TX (11)
         *
         * IMPORTANT: DMA channels must be DISABLED before modifying DMAMUX!
         */
        early_debug_print("[SPI] MspInit: FORCING DMAMUX config\r\n");
        early_debug_print("[SPI] Before: DMAMUX0=");
        spi_debug_hex8(DMAMUX1_Channel0->CCR & 0x7F);
        early_debug_print(" DMAMUX1=");
        spi_debug_hex8(DMAMUX1_Channel1->CCR & 0x7F);
        early_debug_print("\r\n");

        /* STEP 1: Disable DMA channels before modifying DMAMUX */
        DMA1_Channel1->CCR &= ~DMA_CCR_EN;
        DMA1_Channel2->CCR &= ~DMA_CCR_EN;
        __DSB();

        /* STEP 2: Clear any pending flags */
        DMA1->IFCR = DMA_IFCR_CGIF1 | DMA_IFCR_CGIF2;

        /* STEP 3: Force correct DMAMUX routing */
        DMAMUX1_Channel0->CCR = DMA_REQUEST_SPI1_RX;  /* 10 = SPI1_RX */
        DMAMUX1_Channel1->CCR = DMA_REQUEST_SPI1_TX;  /* 11 = SPI1_TX */
        __DSB();

        early_debug_print("[SPI] After:  DMAMUX0=");
        spi_debug_hex8(DMAMUX1_Channel0->CCR & 0x7F);
        early_debug_print(" DMAMUX1=");
        spi_debug_hex8(DMAMUX1_Channel1->CCR & 0x7F);
        early_debug_print("\r\n");

        early_debug_print("[SPI] MspInit: Complete\r\n");
    }
}

/**
 * @brief SPI MSP De-Initialization
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
    if (spiHandle->Instance == SPI1) {
        /* Disable SPI clock */
        __HAL_RCC_SPI1_CLK_DISABLE();

        /* De-init GPIO */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15 | GPIO_PIN_1);
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4 | GPIO_PIN_5);

        /* De-init DMA */
        HAL_DMA_DeInit(spiHandle->hdmarx);
        HAL_DMA_DeInit(spiHandle->hdmatx);

        /* Disable interrupts */
        HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
        HAL_NVIC_DisableIRQ(SPI1_IRQn);
    }
}

/*******************************************************************************
 * PUBLIC API
 ******************************************************************************/

bool bl_spi_init(void)
{
    early_debug_print("\r\n[SPI] === Bootloader SPI Init (HAL pattern) ===\r\n");

    /* Reset SPI1 peripheral to clear any state from application */
    __HAL_RCC_SPI1_FORCE_RESET();
    for (volatile int i = 0; i < 100; i++);
    __HAL_RCC_SPI1_RELEASE_RESET();

    /* Configure SPI1 as slave - EXACTLY like the APP */
    hspi1_bl.Instance = SPI1;
    hspi1_bl.Init.Mode = SPI_MODE_SLAVE;
    hspi1_bl.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1_bl.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1_bl.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1_bl.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1_bl.Init.NSS = SPI_NSS_HARD_INPUT;
    hspi1_bl.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1_bl.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1_bl.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1_bl.Init.CRCPolynomial = 7;
    hspi1_bl.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1_bl.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

    /* HAL_SPI_Init() will call HAL_SPI_MspInit() automatically
     * This configures GPIO, DMA, and NVIC in the correct order */
    early_debug_print("[SPI] Calling HAL_SPI_Init (triggers MspInit)...\r\n");
    if (HAL_SPI_Init(&hspi1_bl) != HAL_OK) {
        early_debug_print("[SPI] HAL_SPI_Init FAILED!\r\n");
        return false;
    }
    early_debug_print("[SPI] HAL_SPI_Init OK\r\n");

    /* Initialize idle TX buffer */
    memset(tx_idle_buffer, BL_SPI_IDLE_PATTERN, sizeof(tx_idle_buffer));

    /* Initialize A+ protocol layer */
    bl_spi_protocol_init();

    /* Reset state */
    rx_count = 0;
    rx_complete = false;
    current_cmd = 0;
    payload_len = 0;
    cmd_ready = false;
    pending_write_addr = 0;
    pending_write_len = 0;
    write_req_pending = false;
    pending_read_len = 0;
    read_data_pending = false;
    last_rx_count = 0;
    rx_stable_start_tick = 0;
    rx_activity_detected = false;
    dma_transfer_complete = false;
    spi_state = BL_SPI_STATE_IDLE;

    /* Debug: Print register state */
    early_debug_print("[SPI] SPI1 CR1=0x");
    spi_print_hex32(SPI1->CR1);
    early_debug_print(" CR2=0x");
    spi_print_hex32(SPI1->CR2);
    early_debug_print("\r\n[SPI] DMAMUX C0=0x");
    spi_print_hex32(DMAMUX1_Channel0->CCR);
    early_debug_print(" (STM32WL: 0x07=SPI1_RX)");
    early_debug_print("\r\n[SPI] DMAMUX C1=0x");
    spi_print_hex32(DMAMUX1_Channel1->CCR);
    early_debug_print(" (STM32WL: 0x08=SPI1_TX)");
    early_debug_print("\r\n[SPI] DMA1_Ch1 CPAR=0x");
    spi_print_hex32(DMA1_Channel1->CPAR);
    early_debug_print(" CMAR=0x");
    spi_print_hex32(DMA1_Channel1->CMAR);
    early_debug_print("\r\n[SPI] Init complete, ready for transactions\r\n");

    return true;
}

void bl_spi_deinit(void)
{
    HAL_SPI_DeInit(&hspi1_bl);  /* This calls HAL_SPI_MspDeInit */
}

/*******************************************************************************
 * DMA MANAGEMENT
 ******************************************************************************/

/**
 * @brief Start DMA reception for next transaction
 * @return true if DMA started successfully
 */
/**
 * @brief Start SPI reception using POLLING (no DMA) for debugging
 * This helps isolate if the problem is DMA-related
 */
static bool bl_spi_start_polling_rx(void)
{
    /* Don't start if NSS is low (transaction in progress) */
    if (!(GPIOA->IDR & GPIO_PIN_15)) {
        return false;
    }

    /* Ensure SPI is ready */
    if (hspi1_bl.State != HAL_SPI_STATE_READY) {
        HAL_SPI_Abort(&hspi1_bl);
        hspi1_bl.State = HAL_SPI_STATE_READY;
    }

    /* Clear error flags and any old data in FIFO */
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1_bl);
    hspi1_bl.ErrorCode = HAL_SPI_ERROR_NONE;

    /* Flush RX FIFO - read until empty */
    while (SPI1->SR & SPI_SR_RXNE) {
        (void)SPI1->DR;
    }

    /* Reset state */
    memset(rx_buffer, 0x00, BL_RX_BUFFER_SIZE);
    rx_activity_detected = false;
    dma_transfer_complete = false;
    spi_state = BL_SPI_STATE_WAITING_RX;

    /* CRITICAL: Pre-fill TX FIFO with idle pattern BEFORE enabling SPI
     * The STM32WL SPI FIFO is 4 bytes. Fill it so master sees 0xAA immediately. */
    __HAL_SPI_ENABLE(&hspi1_bl);

    /* Pre-fill TX FIFO with idle pattern (4 bytes for STM32WL) */
    for (int i = 0; i < 4; i++) {
        while (!(SPI1->SR & SPI_SR_TXE));
        *(__IO uint8_t *)&SPI1->DR = BL_SPI_IDLE_PATTERN;
    }

    /* Debug first time */
    static bool first_start = true;
    if (first_start) {
        first_start = false;
        early_debug_print("[SPI] POLLING mode. CR1=0x");
        spi_print_hex32(SPI1->CR1);
        early_debug_print(" CR2=0x");
        spi_print_hex32(SPI1->CR2);
        early_debug_print("\r\n");
    }

    return true;
}

/**
 * @brief Poll for SPI data (no DMA version)
 * Returns number of bytes received when transaction completes
 *
 * IMPORTANT: Must handle large frames (up to 280 bytes for WRITE_DATA with 256-byte payload)
 */
static uint16_t bl_spi_poll_rx(void)
{
    uint16_t count = 0;
    uint32_t timeout_start = HAL_GetTick();
    bool got_data = false;

    /* Wait for NSS to go low (transaction start) or timeout */
    while ((GPIOA->IDR & GPIO_PIN_15) && (HAL_GetTick() - timeout_start < 100)) {
        /* NSS high, waiting... */
    }

    if (GPIOA->IDR & GPIO_PIN_15) {
        return 0;  /* No transaction started */
    }

    /* NSS is low - transaction in progress. Receive bytes.
     * Max frame size: header(4) + payload(256) + CRC(1) = 261 bytes
     * Use BL_SPI_TRANSACTION_SIZE (280) as limit to be safe */
    timeout_start = HAL_GetTick();

    while (count < BL_SPI_TRANSACTION_SIZE && (HAL_GetTick() - timeout_start < 100)) {
        /* Check if RX FIFO has data */
        if (SPI1->SR & SPI_SR_RXNE) {
            rx_buffer[count++] = *(__IO uint8_t *)&SPI1->DR;
            /* Send idle byte on TX */
            *(__IO uint8_t *)&SPI1->DR = BL_SPI_IDLE_PATTERN;
            got_data = true;
            timeout_start = HAL_GetTick();  /* Reset timeout on each byte */
        }

        /* Check if NSS went high (transaction end) */
        if (got_data && (GPIOA->IDR & GPIO_PIN_15)) {
            break;
        }
    }

    return count;
}

static bool bl_spi_start_dma_rx(void)
{
    /* USE POLLING MODE FOR DEBUGGING */
    return bl_spi_start_polling_rx();
}

void bl_spi_start_rx(void)
{
    rx_count = 0;
    rx_complete = false;
    last_rx_count = 0;
    rx_stable_start_tick = 0;
    rx_activity_detected = false;

    if (!bl_spi_start_dma_rx()) {
        spi_state = BL_SPI_STATE_IDLE;  /* Will retry in bl_spi_has_data() */
    }
}

void bl_spi_stop_rx(void)
{
    HAL_SPI_Abort(&hspi1_bl);
    spi_state = BL_SPI_STATE_IDLE;
}

/*******************************************************************************
 * TRANSACTION DETECTION
 ******************************************************************************/

/**
 * @brief Check if a SPI transaction has completed
 * Uses DMA counter and timeout to detect end of variable-length transfers.
 */
static bool bl_spi_check_transaction_end(uint16_t *bytes_received)
{
    uint16_t total = expected_transfer_size;

    /* Fast path: DMA callback already fired */
    if (dma_transfer_complete) {
        *bytes_received = total;
        last_rx_count = 0;
        rx_stable_start_tick = 0;
        rx_activity_detected = false;
        dma_transfer_complete = false;
        return true;
    }

    /* Check DMA counter */
    if (hspi1_bl.hdmarx == NULL || hspi1_bl.hdmarx->Instance == NULL) {
        return false;
    }

    uint16_t remaining = __HAL_DMA_GET_COUNTER(hspi1_bl.hdmarx);
    uint16_t current = (remaining <= total) ? (total - remaining) : 0;

    /* New bytes received? */
    if (current > last_rx_count) {
        rx_activity_detected = true;
        last_rx_count = current;
        rx_stable_start_tick = HAL_GetTick();
        return false;
    }

    /* No new bytes - check timeout */
    if (rx_activity_detected && current > 0) {
        uint32_t elapsed = HAL_GetTick() - rx_stable_start_tick;

        if (elapsed >= RX_STABLE_TIMEOUT_MS) {
            *bytes_received = current;
            last_rx_count = 0;
            rx_stable_start_tick = 0;
            rx_activity_detected = false;
            dma_transfer_complete = false;
            return true;
        }
    }

    return false;
}

/*******************************************************************************
 * DATA ACCESS
 ******************************************************************************/

/**
 * @brief Handle poll transaction in TX state - clock out response
 * Returns when transaction is complete
 */
static void bl_spi_handle_tx_poll(void)
{
    uint32_t timeout_start = HAL_GetTick();
    uint16_t tx_index = 4;  /* First 4 bytes already in FIFO */
    uint16_t rx_idx = 0;

    /* Wait for NSS to go low (poll transaction start) */
    while ((GPIOA->IDR & GPIO_PIN_15) && (HAL_GetTick() - timeout_start < 100)) {
        /* NSS high, waiting... */
    }

    if (GPIOA->IDR & GPIO_PIN_15) {
        /* Timeout - no poll received, go back to idle */
        spi_state = BL_SPI_STATE_IDLE;
        return;
    }

    /* NSS low - transaction in progress. Clock out response.
     * Use same size limit as RX for consistency */
    timeout_start = HAL_GetTick();

    while (rx_idx < BL_SPI_TRANSACTION_SIZE && (HAL_GetTick() - timeout_start < 100)) {
        /* Check for received byte (means master clocked) */
        if (SPI1->SR & SPI_SR_RXNE) {
            (void)*(__IO uint8_t *)&SPI1->DR;  /* Read and discard RX */
            rx_idx++;
            timeout_start = HAL_GetTick();

            /* Load next TX byte if available */
            if (tx_index < tx_len && (SPI1->SR & SPI_SR_TXE)) {
                *(__IO uint8_t *)&SPI1->DR = tx_buffer[tx_index++];
            } else if (SPI1->SR & SPI_SR_TXE) {
                /* Pad with idle pattern */
                *(__IO uint8_t *)&SPI1->DR = BL_SPI_IDLE_PATTERN;
            }
        }

        /* Check if NSS went high (transaction end) */
        if (rx_idx > 0 && (GPIOA->IDR & GPIO_PIN_15)) {
            break;
        }
    }

    /* Response sent - prepare for next command */
    spi_state = BL_SPI_STATE_IDLE;
}

bool bl_spi_has_data(void)
{
    /* POLLING MODE FOR DEBUGGING */

    /* Handle response transmission state */
    if (spi_state == BL_SPI_STATE_WAITING_TX) {
        bl_spi_handle_tx_poll();
        /* CRITICAL: Reset RX state after TX handling (whether success or timeout)
         * Without this, old rx_complete/rx_count cause re-processing of stale data */
        rx_complete = false;
        rx_count = 0;
        return false;  /* After TX, go to idle and wait for next command */
    }

    if (spi_state == BL_SPI_STATE_IDLE) {
        rx_count = 0;
        rx_complete = false;
        if (!bl_spi_start_dma_rx()) {  /* This now uses polling */
            return false;
        }
        return false;
    }

    if (spi_state == BL_SPI_STATE_WAITING_RX) {
        /* Poll for data */
        uint16_t bytes = bl_spi_poll_rx();

        if (bytes > 0) {
            /* Debug: show what we received (use hex16 for values > 255) */
            early_debug_print("[RX:");
            if (bytes > 255) {
                spi_debug_hex8(bytes >> 8);  /* High byte */
            }
            spi_debug_hex8(bytes & 0xFF);    /* Low byte */
            early_debug_print(" ");
            for (int i = 0; i < (bytes > 8 ? 8 : bytes); i++) {
                spi_debug_hex8(rx_buffer[i]);
            }
            early_debug_print("]\r\n");

            /* Validate frame */
            if (bytes >= 5 && rx_buffer[0] == BL_SPI_IDLE_PATTERN) {
                rx_count = bytes;
                rx_complete = true;
                spi_state = BL_SPI_STATE_PROCESSING;
            } else {
                /* Invalid or incomplete - restart */
                spi_state = BL_SPI_STATE_IDLE;
                return false;
            }
        }
    }

    return rx_complete || (rx_count > 0);
}

uint16_t bl_spi_rx_available(void)
{
    return rx_count;
}

uint16_t bl_spi_read(uint8_t* buffer, uint16_t max_len)
{
    uint16_t len = (rx_count < max_len) ? rx_count : max_len;
    memcpy(buffer, rx_buffer, len);
    return len;
}

bool bl_spi_read_byte(uint8_t* byte)
{
    if (rx_count == 0) return false;
    *byte = rx_buffer[0];
    return true;
}

bool bl_spi_prepare_tx(const uint8_t* data, uint16_t len)
{
    if (len > BL_TX_BUFFER_SIZE) len = BL_TX_BUFFER_SIZE;
    memcpy(tx_buffer, data, len);
    tx_len = len;
    return true;
}

void bl_spi_flush_rx(void)
{
    rx_count = 0;
    rx_complete = false;
}

/*******************************************************************************
 * COMMAND PROCESSING
 ******************************************************************************/

/**
 * @brief Check if buffer is a poll frame (all 0xAA)
 */
static bool is_poll_frame(const uint8_t *data, uint16_t len)
{
    if (len < 5) return true;
    return (data[0] == BL_SPI_IDLE_PATTERN &&
            data[1] == BL_SPI_IDLE_PATTERN &&
            data[2] == BL_SPI_IDLE_PATTERN &&
            data[3] == BL_SPI_IDLE_PATTERN);
}

bool bl_spi_process(void)
{
    bl_spi_has_data();

    if (cmd_ready) return true;
    if (!rx_complete) return false;

    rx_complete = false;

    if (rx_count < 1) {
        bl_spi_start_rx();
        return false;
    }

    /* TIMING CRITICAL: Check for poll frame FIRST, restart DMA IMMEDIATELY
     * Debug prints take ~1ms each at 115200 baud, causing us to miss commands */

    /* Skip poll frames - restart DMA ASAP before any debug prints */
    if (is_poll_frame(rx_buffer, rx_count)) {
        bl_spi_start_rx();  /* Re-arm DMA immediately */
        return false;
    }

    /* Only print for real commands (not poll frames) */
    early_debug_print("[");
    spi_debug_hex8(rx_buffer[0]);
    spi_debug_hex8(rx_buffer[1]);
    spi_debug_hex8(rx_buffer[2]);
    spi_debug_hex8(rx_buffer[3]);
    early_debug_print("]");

    /* Parse with protocol layer */
    if (!bl_spi_protocol_process(rx_buffer, rx_count)) {
        bl_spi_start_rx();
        return false;
    }

    bl_spi_request_t *request = bl_spi_protocol_get_request();
    bl_spi_protocol_ctx_t *prot_ctx = bl_spi_protocol_get_ctx();

    /* Check CRC for A+ protocol */
    if (prot_ctx->mode == BL_PROT_MODE_A_PLUS && !request->crc_valid) {
        bl_spi_send_response(DFU_RSP_CRC_ERROR, NULL, 0);
        bl_spi_start_rx();
        return false;
    }

    /* Extract command and payload */
    current_cmd = request->command;
    payload_len = request->data_len;

    if (payload_len > 0 && payload_len <= BL_CHUNK_SIZE) {
        memcpy(payload_buffer, request->data, payload_len);
    }

    /* Handle WRITE_REQ - store address/length */
    if (current_cmd == SPI_CMD_DFU_WRITE_REQ && payload_len >= 6) {
        memcpy(&pending_write_addr, payload_buffer, 4);
        memcpy(&pending_write_len, payload_buffer + 4, 2);
        write_req_pending = true;
    }

    /* Validate WRITE_DATA has pending request */
    if (current_cmd == SPI_CMD_DFU_WRITE_DATA) {
        if (!write_req_pending || pending_write_len == 0) {
            bl_spi_send_response(DFU_RSP_NOT_READY, NULL, 0);
            bl_spi_start_rx();
            return false;
        }
    }

    cmd_ready = true;
    return cmd_ready;
}

uint8_t bl_spi_get_command(void)
{
    return current_cmd;
}

uint16_t bl_spi_get_payload(uint8_t* buffer, uint16_t max_len)
{
    /* For WRITE_DATA, prepend stored address */
    if (current_cmd == SPI_CMD_DFU_WRITE_DATA && write_req_pending) {
        if (max_len < 4 + payload_len) return 0;
        memcpy(buffer, &pending_write_addr, 4);
        memcpy(buffer + 4, payload_buffer, payload_len);
        return 4 + payload_len;
    }

    uint16_t len = (payload_len < max_len) ? payload_len : max_len;
    memcpy(buffer, payload_buffer, len);
    return len;
}

/*******************************************************************************
 * READ DATA STORAGE (for multi-transaction reads)
 ******************************************************************************/

bool bl_spi_store_read_data(const uint8_t* data, uint16_t len)
{
    if (data == NULL || len == 0 || len > BL_CHUNK_SIZE) return false;
    memcpy(pending_read_buffer, data, len);
    pending_read_len = len;
    read_data_pending = true;
    return true;
}

uint16_t bl_spi_get_read_data(uint8_t* buffer, uint16_t max_len)
{
    if (!read_data_pending || buffer == NULL) return 0;
    uint16_t len = (pending_read_len < max_len) ? pending_read_len : max_len;
    memcpy(buffer, pending_read_buffer, len);
    return len;
}

bool bl_spi_has_read_data(void)
{
    return read_data_pending;
}

/*******************************************************************************
 * RESPONSE TRANSMISSION
 ******************************************************************************/

void bl_spi_send_response(dfu_response_t status, const uint8_t* data, uint16_t data_len)
{
    bl_spi_protocol_ctx_t *prot_ctx = bl_spi_protocol_get_ctx();

    /* Build response based on protocol mode */
    if (prot_ctx->mode == BL_PROT_MODE_A_PLUS) {
        uint8_t data_len_u8 = (data_len > 255) ? 255 : (uint8_t)data_len;
        tx_len = bl_spi_protocol_build_response(tx_buffer,
                                                 prot_ctx->request.sequence,
                                                 (bl_prot_status_t)status,
                                                 data, data_len_u8);
    } else {
        tx_buffer[0] = (uint8_t)status;
        if (data != NULL && data_len > 0) {
            uint16_t copy_len = (data_len < BL_TX_BUFFER_SIZE - 1) ? data_len : (BL_TX_BUFFER_SIZE - 1);
            memcpy(tx_buffer + 1, data, copy_len);
            tx_len = copy_len + 1;
        } else {
            tx_len = 1;
        }
        /* Pad for master over-read */
        if (tx_len < 16) {
            memset(&tx_buffer[tx_len], BL_SPI_IDLE_PATTERN, 16 - tx_len);
            tx_len = 16;
        }
    }

    /* Clear pending states */
    if (current_cmd == SPI_CMD_DFU_WRITE_DATA) {
        write_req_pending = false;
        pending_write_addr = 0;
        pending_write_len = 0;
    }
    if (current_cmd == SPI_CMD_DFU_READ_DATA) {
        read_data_pending = false;
        pending_read_len = 0;
    }

    /* Reset command state */
    current_cmd = 0;
    payload_len = 0;
    cmd_ready = false;
    rx_complete = false;  /* Reset RX state for next command */
    rx_count = 0;

    /* POLLING MODE: Pre-load TX FIFO with response data.
     * The response will be clocked out when master sends the poll transaction.
     *
     * CRITICAL: We must wait for NSS high, then pre-fill TX FIFO before
     * the next transaction starts. */

    /* Clear any errors */
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1_bl);
    hspi1_bl.ErrorCode = HAL_SPI_ERROR_NONE;

    /* Flush RX FIFO */
    while (SPI1->SR & SPI_SR_RXNE) {
        (void)SPI1->DR;
    }

    /* Wait for any ongoing transaction to complete (NSS high) */
    uint32_t wait_start = HAL_GetTick();
    while (!(GPIOA->IDR & GPIO_PIN_15) && (HAL_GetTick() - wait_start < 10)) {
        /* Wait for NSS to go high */
    }

    /* Pre-fill TX FIFO with response data (up to 4 bytes for STM32WL FIFO) */
    uint16_t prefill = (tx_len > 4) ? 4 : tx_len;
    for (uint16_t i = 0; i < prefill; i++) {
        while (!(SPI1->SR & SPI_SR_TXE));
        *(__IO uint8_t *)&SPI1->DR = tx_buffer[i];
    }

    /* Update state - we're now waiting for master to clock out response */
    spi_state = BL_SPI_STATE_WAITING_TX;
    last_rx_count = 0;
    rx_stable_start_tick = 0;
    rx_activity_detected = false;
    dma_transfer_complete = false;
}

/*******************************************************************************
 * IRQ HANDLERS AND CALLBACKS
 ******************************************************************************/

void bl_spi_irq_handler(void)
{
    /* Handled by HAL */
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        dma_transfer_complete = true;

        if (spi_state == BL_SPI_STATE_WAITING_TX) {
            /* Response sent - restart RX */
            spi_state = BL_SPI_STATE_IDLE;
        }
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        /* Clear errors silently - they're normal when NSS deasserts */
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
    }
}

/* IRQ Handlers - Route to HAL */
void SPI1_IRQHandler(void)
{
    HAL_SPI_IRQHandler(&hspi1_bl);
}

void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_rx_bl);
}

void DMA1_Channel2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_spi1_tx_bl);
}
