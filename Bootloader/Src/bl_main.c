/**
 * @file    bl_main.c
 * @brief   Main bootloader implementation
 * @date    2025
 */

#include "bl_main.h"
#include "bl_config.h"
#include "bl_flash.h"
#include "bl_crc.h"
#include "bl_dfu.h"
#include "bl_uart.h"
#include "bl_spi.h"
#include "bl_spi_protocol.h"
#include "bl_app_header.h"
#include "stm32wlxx_hal.h"
#include <string.h>
#include <stdlib.h>

/* Current bootloader state */
static bl_state_t current_state = BL_STATE_INIT;
static bl_protocol_t detected_protocol = BL_PROTO_NONE;

/* Application jump function pointer type */
typedef void (*pFunction)(void);

/* Forward declarations */
static void bl_state_init(void);
static void bl_state_check_app(void);
static void bl_state_detect_protocol(void);
static void bl_state_dfu_uart(void);
static void bl_state_dfu_spi(void);
static void bl_state_validate(void);
static void bl_state_error(void);
static void bl_hw_init(void);
static void bl_hw_deinit(void);

void bl_init(void)
{
    /* Initialize HAL */
    HAL_Init();

    /* Configure system clock */
    bl_hw_init();

    /* Initialize subsystems */
    bl_flash_init();
    bl_crc_init();
    bl_dfu_init();

    current_state = BL_STATE_INIT;
}

void bl_run(void)
{
    while (1) {
        switch (current_state) {
            case BL_STATE_INIT:
                bl_state_init();
                break;

            case BL_STATE_CHECK_APP:
                bl_state_check_app();
                break;

            case BL_STATE_DETECT_PROTOCOL:
                bl_state_detect_protocol();
                break;

            case BL_STATE_DFU_IDLE:
            case BL_STATE_DFU_UART:
                bl_state_dfu_uart();
                break;

            case BL_STATE_DFU_SPI:
                bl_state_dfu_spi();
                break;

            case BL_STATE_VALIDATE:
                bl_state_validate();
                break;

            case BL_STATE_JUMP_APP:
                bl_jump_to_app();
                /* If we return here, jump failed */
                current_state = BL_STATE_ERROR;
                break;

            case BL_STATE_ERROR:
            default:
                bl_state_error();
                break;
        }
    }
}

bl_state_t bl_get_state(void)
{
    return current_state;
}

void bl_set_state(bl_state_t state)
{
    current_state = state;
}

bl_protocol_t bl_get_protocol(void)
{
    return detected_protocol;
}

static void bl_state_init(void)
{
    /* Check for DFU request flag */
    if (bl_flash_is_dfu_requested()) {
        /* Clear the flag */
        bl_flash_clear_dfu_request();
        /* Go directly to DFU mode */
        current_state = BL_STATE_DETECT_PROTOCOL;
    } else {
        /* Check if application is valid */
        current_state = BL_STATE_CHECK_APP;
    }
}

static void bl_state_check_app(void)
{
    if (bl_check_app_valid()) {
        /* Application is valid, jump to it */
        current_state = BL_STATE_JUMP_APP;
    } else {
        /* No valid application, enter DFU mode */
        current_state = BL_STATE_DETECT_PROTOCOL;
    }
}

static void bl_state_detect_protocol(void)
{
    uint32_t start_tick = HAL_GetTick();

    /* Initialize both interfaces */
    bl_uart_init();
    bl_spi_init();

    /* Start reception on both */
    bl_uart_start_rx();
    bl_spi_start_rx();

    /* Wait for activity on either interface */
    while ((HAL_GetTick() - start_tick) < BL_DETECTION_TIMEOUT_MS) {
        if (bl_uart_has_data()) {
            detected_protocol = BL_PROTO_UART;
            bl_spi_stop_rx();
            bl_spi_deinit();
            current_state = BL_STATE_DFU_UART;
            return;
        }

        if (bl_spi_has_data()) {
            detected_protocol = BL_PROTO_SPI;
            bl_uart_stop_rx();
            bl_uart_deinit();
            current_state = BL_STATE_DFU_SPI;
            return;
        }
    }

    /* Timeout - check if we have a valid app to boot */
    if (bl_check_app_valid()) {
        bl_uart_stop_rx();
        bl_spi_stop_rx();
        current_state = BL_STATE_JUMP_APP;
    } else {
        /* Stay in DFU mode with UART as default */
        detected_protocol = BL_PROTO_UART;
        bl_spi_stop_rx();
        bl_spi_deinit();
        current_state = BL_STATE_DFU_UART;
    }
}

static void bl_state_dfu_uart(void)
{
    /* Process UART commands */
    if (bl_uart_process()) {
        char cmd_buffer[128];
        uint16_t cmd_len = bl_uart_get_command(cmd_buffer, sizeof(cmd_buffer));

        if (cmd_len > 0) {
            /* Parse and execute AT+DFU command */
            uint8_t response[BL_TX_BUFFER_SIZE];
            uint16_t response_len = sizeof(response);
            dfu_response_t status;

            /* Parse AT command and get DFU command */
            dfu_cmd_t dfu_cmd = DFU_CMD_MAX;
            uint8_t payload[BL_CHUNK_SIZE];
            uint16_t payload_len = 0;

            /* Simple AT command parsing */
            if (strncmp(cmd_buffer, "AT+DFU=PING", 11) == 0) {
                dfu_cmd = DFU_CMD_PING;
            } else if (strncmp(cmd_buffer, "AT+DFU=INFO", 11) == 0) {
                dfu_cmd = DFU_CMD_GET_INFO;
            } else if (strncmp(cmd_buffer, "AT+DFU=ERASE", 12) == 0) {
                dfu_cmd = DFU_CMD_ERASE;
            } else if (strncmp(cmd_buffer, "AT+DFU=VERIFY", 13) == 0) {
                dfu_cmd = DFU_CMD_VERIFY;
                /* Parse CRC from command: AT+DFU=VERIFY,<crc32_hex> */
                const char* crc_str = cmd_buffer + 14;
                if (*crc_str == ',') crc_str++;
                char* endptr;
                uint32_t crc = strtoul(crc_str, &endptr, 16);
                if (endptr == crc_str) {
                    /* Parse error - no CRC provided */
                    bl_uart_send_response(DFU_RSP_CRC_ERROR, NULL, 0);
                    return;
                }
                memcpy(payload, &crc, 4);
                payload_len = 4;
            } else if (strncmp(cmd_buffer, "AT+DFU=WRITE", 12) == 0) {
                dfu_cmd = DFU_CMD_WRITE;
                /* Parse address and hex data: AT+DFU=WRITE,<addr>,<hex_data> */
                const char* params = cmd_buffer + 13;
                if (*params == ',') params++;
                char* endptr;
                uint32_t addr = strtoul(params, &endptr, 16);
                if (endptr == params) {
                    /* Parse error - no address */
                    bl_uart_send_response(DFU_RSP_ADDR_ERROR, NULL, 0);
                    return;
                }
                params = endptr;
                if (*params == ',') params++;

                memcpy(payload, &addr, 4);
                payload_len = 4;

                /* Convert hex string to binary */
                while (*params && payload_len < BL_CHUNK_SIZE) {
                    /* Validate hex characters */
                    if (!((params[0] >= '0' && params[0] <= '9') ||
                          (params[0] >= 'A' && params[0] <= 'F') ||
                          (params[0] >= 'a' && params[0] <= 'f'))) {
                        break;  /* Stop at first non-hex character */
                    }
                    if (!params[1]) break;  /* Need pairs */
                    char hex[3] = {params[0], params[1], 0};
                    payload[payload_len++] = (uint8_t)strtoul(hex, NULL, 16);
                    params += 2;
                }
                /* Check if we got any data */
                if (payload_len <= 4) {
                    bl_uart_send_response(DFU_RSP_SIZE_ERROR, NULL, 0);
                    return;
                }
            } else if (strncmp(cmd_buffer, "AT+DFU=JUMP", 11) == 0) {
                dfu_cmd = DFU_CMD_JUMP;
            } else if (strncmp(cmd_buffer, "AT+DFU=RESET", 12) == 0) {
                dfu_cmd = DFU_CMD_RESET;
            } else if (strncmp(cmd_buffer, "AT+DFU=STATUS", 13) == 0) {
                dfu_cmd = DFU_CMD_GET_STATUS;
            } else if (strncmp(cmd_buffer, "AT+DFU=ABORT", 12) == 0) {
                dfu_cmd = DFU_CMD_ABORT;
            } else if (strncmp(cmd_buffer, "AT+DFU=READ", 11) == 0) {
                dfu_cmd = DFU_CMD_READ;
                /* Parse address and length: AT+DFU=READ,<addr>,<len> */
                const char* params = cmd_buffer + 12;
                if (*params == ',') params++;
                char* endptr;
                uint32_t addr = strtoul(params, &endptr, 16);
                if (endptr == params) {
                    /* Parse error - no address */
                    bl_uart_send_response(DFU_RSP_ADDR_ERROR, NULL, 0);
                    return;
                }
                params = endptr;
                if (*params == ',') params++;
                uint16_t len = (uint16_t)strtoul(params, &endptr, 10);
                if (endptr == params || len == 0) {
                    /* Parse error - no length */
                    bl_uart_send_response(DFU_RSP_SIZE_ERROR, NULL, 0);
                    return;
                }
                memcpy(payload, &addr, 4);
                memcpy(payload + 4, &len, 2);
                payload_len = 6;
            }

            /* Process command */
            if (dfu_cmd != DFU_CMD_MAX) {
                /* Special handling for JUMP - must verify CRC first */
                if (dfu_cmd == DFU_CMD_JUMP) {
                    if (!bl_dfu_can_jump()) {
                        bl_uart_send_response(DFU_RSP_NOT_READY, NULL, 0);
                        return;
                    }
                }

                status = bl_dfu_process_cmd(dfu_cmd, payload, payload_len,
                                           response, &response_len);
                bl_uart_send_response(status, response, response_len);

                /* Handle special commands */
                if (status == DFU_RSP_OK) {
                    if (dfu_cmd == DFU_CMD_JUMP) {
                        current_state = BL_STATE_JUMP_APP;
                    } else if (dfu_cmd == DFU_CMD_RESET) {
                        bl_system_reset();
                    } else if (dfu_cmd == DFU_CMD_VERIFY && bl_dfu_is_complete()) {
                        current_state = BL_STATE_VALIDATE;
                    }
                }
            } else {
                bl_uart_send_response(DFU_RSP_INVALID_CMD, NULL, 0);
            }
        }
    }
}

static void bl_state_dfu_spi(void)
{
    /* Process SPI commands */
    if (bl_spi_process()) {
        uint8_t spi_cmd = bl_spi_get_command();
        uint8_t payload[BL_CHUNK_SIZE];
        uint16_t payload_len = bl_spi_get_payload(payload, sizeof(payload));

        uint8_t response[BL_TX_BUFFER_SIZE];
        uint16_t response_len = sizeof(response);
        dfu_response_t status;

        /* Map SPI command to DFU command */
        dfu_cmd_t dfu_cmd = DFU_CMD_MAX;

        if (spi_cmd >= SPI_CMD_DFU_BASE) {
            switch (spi_cmd) {
                case SPI_CMD_DFU_PING:      dfu_cmd = DFU_CMD_PING; break;
                case SPI_CMD_DFU_GET_INFO:  dfu_cmd = DFU_CMD_GET_INFO; break;
                case SPI_CMD_DFU_ERASE:     dfu_cmd = DFU_CMD_ERASE; break;
                case SPI_CMD_DFU_WRITE_REQ:
                    /* WRITE_REQ just stores address/length for next WRITE_DATA */
                    /* Send OK response, no DFU processing needed */
                    bl_spi_send_response(DFU_RSP_OK, NULL, 0);
                    return;
                case SPI_CMD_DFU_WRITE_DATA:dfu_cmd = DFU_CMD_WRITE; break;
                case SPI_CMD_DFU_READ_REQ:
                    /* READ_REQ reads data and stores for next READ_DATA */
                    {
                        uint8_t read_buf[BL_CHUNK_SIZE];
                        uint16_t read_len = sizeof(read_buf);
                        dfu_response_t read_status = bl_dfu_process_cmd(DFU_CMD_READ,
                            payload, payload_len, read_buf, &read_len);
                        if (read_status == DFU_RSP_OK) {
                            bl_spi_store_read_data(read_buf, read_len);
                        }
                        bl_spi_send_response(read_status, NULL, 0);
                    }
                    return;
                case SPI_CMD_DFU_READ_DATA:
                    /* READ_DATA returns data stored by previous READ_REQ */
                    if (bl_spi_has_read_data()) {
                        uint8_t read_buf[BL_CHUNK_SIZE];
                        uint16_t read_len = bl_spi_get_read_data(read_buf, sizeof(read_buf));
                        bl_spi_send_response(DFU_RSP_OK, read_buf, read_len);
                    } else {
                        bl_spi_send_response(DFU_RSP_NOT_READY, NULL, 0);
                    }
                    return;
                case SPI_CMD_DFU_VERIFY:    dfu_cmd = DFU_CMD_VERIFY; break;
                case SPI_CMD_DFU_RESET:     dfu_cmd = DFU_CMD_RESET; break;
                case SPI_CMD_DFU_JUMP:      dfu_cmd = DFU_CMD_JUMP; break;
                case SPI_CMD_DFU_GET_STATUS:dfu_cmd = DFU_CMD_GET_STATUS; break;
                case SPI_CMD_DFU_SET_HEADER:dfu_cmd = DFU_CMD_SET_HEADER; break;
                case SPI_CMD_DFU_ABORT:     dfu_cmd = DFU_CMD_ABORT; break;
                default: break;
            }
        }

        /* Process command */
        if (dfu_cmd != DFU_CMD_MAX) {
            /* Special handling for JUMP - must verify CRC first */
            if (dfu_cmd == DFU_CMD_JUMP) {
                if (!bl_dfu_can_jump()) {
                    bl_spi_send_response(DFU_RSP_NOT_READY, NULL, 0);
                    return;
                }
            }

            /* Set operation state before long operations */
            if (dfu_cmd == DFU_CMD_ERASE) {
                bl_spi_protocol_set_op_state(BL_DFU_STATE_ERASING);
            } else if (dfu_cmd == DFU_CMD_WRITE) {
                bl_spi_protocol_set_op_state(BL_DFU_STATE_WRITING);
            } else if (dfu_cmd == DFU_CMD_VERIFY) {
                bl_spi_protocol_set_op_state(BL_DFU_STATE_VERIFYING);
            }

            status = bl_dfu_process_cmd(dfu_cmd, payload, payload_len,
                                       response, &response_len);

            /* Update operation state after completion */
            if (status == DFU_RSP_OK) {
                if (dfu_cmd == DFU_CMD_ERASE || dfu_cmd == DFU_CMD_WRITE) {
                    bl_spi_protocol_set_op_state(BL_DFU_STATE_READY);
                } else if (dfu_cmd == DFU_CMD_VERIFY) {
                    bl_spi_protocol_set_op_state(BL_DFU_STATE_COMPLETE);
                }
            } else {
                bl_spi_protocol_set_op_state(BL_DFU_STATE_ERROR);
            }

            bl_spi_send_response(status, response, response_len);

            /* Handle special commands */
            if (status == DFU_RSP_OK) {
                if (dfu_cmd == DFU_CMD_JUMP) {
                    current_state = BL_STATE_JUMP_APP;
                } else if (dfu_cmd == DFU_CMD_RESET) {
                    bl_system_reset();
                } else if (dfu_cmd == DFU_CMD_VERIFY && bl_dfu_is_complete()) {
                    current_state = BL_STATE_VALIDATE;
                }
            }
        } else {
            bl_spi_send_response(DFU_RSP_INVALID_CMD, NULL, 0);
        }
    }
}

static void bl_state_validate(void)
{
    /* Final validation of written firmware */
    if (bl_check_app_valid()) {
        /* Send success response if still connected */
        if (detected_protocol == BL_PROTO_UART) {
            bl_uart_print("+DFU=OK,VALIDATED\r\n");
        }
        current_state = BL_STATE_JUMP_APP;
    } else {
        if (detected_protocol == BL_PROTO_UART) {
            bl_uart_print("+DFU=ERR,VALIDATION_FAILED\r\n");
        }
        current_state = BL_STATE_ERROR;
    }
}

static void bl_state_error(void)
{
    /* Stay in error state, wait for reset or new DFU attempt */
    /* Could blink LED or provide other indication */

    /* Re-enter DFU mode to allow retry */
    bl_dfu_reset();

    if (detected_protocol == BL_PROTO_SPI) {
        current_state = BL_STATE_DFU_SPI;
    } else {
        current_state = BL_STATE_DFU_UART;
    }
}

void bl_jump_to_app(void)
{
    const app_header_t* header = bl_get_app_header();
    uint32_t app_stack;
    uint32_t app_entry;

    /* Get stack pointer and entry point from vector table */
    if (header != NULL && bl_header_magic_valid(header)) {
        app_stack = *(__IO uint32_t*)(header->app_start_addr);
        app_entry = *(__IO uint32_t*)(header->app_start_addr + 4);
    } else {
        /* Fallback to default app address */
        app_stack = *(__IO uint32_t*)APP_FLASH_BASE;
        app_entry = *(__IO uint32_t*)(APP_FLASH_BASE + 4);
    }

    /* Validate stack pointer (must be in RAM range) */
    if (app_stack < 0x20000000UL || app_stack > 0x20010000UL) {
        return; /* Invalid stack pointer */
    }

    /* Validate entry point (must be in flash) */
    if (app_entry < APP_FLASH_BASE || app_entry > (APP_FLASH_BASE + APP_FLASH_SIZE)) {
        return; /* Invalid entry point */
    }

    /* Deinitialize peripherals */
    bl_hw_deinit();

    /* Disable all interrupts */
    __disable_irq();

    /* Disable SysTick */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /* Clear pending interrupts */
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    /* Relocate vector table to application */
    SCB->VTOR = header ? header->app_start_addr : APP_FLASH_BASE;

    /* Set stack pointer */
    __set_MSP(app_stack);

    /* Jump to application Reset_Handler */
    pFunction app_reset_handler = (pFunction)app_entry;
    app_reset_handler();

    /* Should never reach here */
    while (1);
}

bool bl_check_app_valid(void)
{
    const app_header_t* header = bl_get_app_header();

    /* Check header validity */
    if (!bl_validate_app_header(header)) {
        return false;
    }

    /* Check CRC */
    if (!bl_validate_app_crc(header)) {
        return false;
    }

    return true;
}

bool bl_validate_app_header(const app_header_t* header)
{
    if (header == NULL) {
        return false;
    }

    /* Check magic number */
    if (!bl_header_magic_valid(header)) {
        return false;
    }

    /* Check header version */
    if (!bl_header_version_valid(header)) {
        return false;
    }

    /* Check addresses */
    if (header->app_start_addr != APP_FLASH_BASE) {
        return false;
    }

    if (header->app_size == 0 || header->app_size > APP_MAX_SIZE) {
        return false;
    }

    /* Check bootloader compatibility */
    if (!bl_app_compatible(header)) {
        return false;
    }

    return true;
}

bool bl_validate_app_crc(const app_header_t* header)
{
    if (header == NULL) {
        return false;
    }

    /* Calculate CRC of application code */
    uint32_t calc_crc = bl_crc32_flash(header->app_start_addr, header->app_size);

    return (calc_crc == header->app_crc32);
}

void bl_system_reset(void)
{
    NVIC_SystemReset();
}

uint32_t bl_get_version(void)
{
    return BL_VERSION;
}

const char* bl_get_version_string(void)
{
    return BL_VERSION_STRING;
}

static void bl_hw_init(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /* Configure the main internal regulator output voltage */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Initialize HSI oscillator (16 MHz internal)
     * HSI is used for bootloader because:
     * - Always available (no external crystal dependency)
     * - Immediate startup (~2µs vs ms for HSE)
     * - Sufficient precision for UART 9600 baud (±1%)
     * - Maximum reliability for critical boot code */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        /* HSI failure is critical - should never happen */
        Error_Handler();
    }

    /* Configure system clock: SYSCLK = HSI = 16 MHz */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }

    /* Enable GPIO clocks */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

static void bl_hw_deinit(void)
{
    /* Deinitialize peripherals */
    bl_uart_deinit();
    bl_spi_deinit();
    bl_crc_deinit();

    /* Reset GPIO clocks */
    __HAL_RCC_GPIOA_CLK_DISABLE();
    __HAL_RCC_GPIOB_CLK_DISABLE();

    /* DeInit HAL */
    HAL_DeInit();
}

/* Main entry point for bootloader */
int main(void)
{
    bl_init();
    bl_run();

    /* Should never reach here */
    return 0;
}

/*******************************************************************************
 * HAL REQUIRED CALLBACKS AND STUBS
 ******************************************************************************/

/**
 * @brief Error handler for HAL
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Stay in error loop */
    }
}

/**
 * @brief Assert failed callback for HAL
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    /* In bootloader, just reset on assertion failure */
    NVIC_SystemReset();
}

/* Note: DMA functions are provided by stm32wlxx_hal_dma.c
 * SPI uses DMA for reliable slave operation - do not add stubs here */
