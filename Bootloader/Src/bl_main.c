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

/* Early DFU flag detection - set in main() before any initialization */
static volatile bool g_early_dfu_flag_detected = false;

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
void Error_Handler(void);

/* Forward declaration for early debug - exported for use by other modules */
void early_debug_print(const char *str);
static void early_debug_update_baudrate(void);

void bl_init(void)
{
    early_debug_print("[BL] bl_init: HAL_Init...\r\n");
    /* Initialize HAL */
    HAL_Init();

    early_debug_print("[BL] bl_init: bl_hw_init...\r\n");
    /* Configure system clock to 32 MHz PLL (same as application) */
    bl_hw_init();

    /* Update LPUART1 baud rate for new 32 MHz clock */
    early_debug_update_baudrate();

    early_debug_print("[BL] bl_init: clock now 32MHz PLL\r\n");
    early_debug_print("[BL] bl_init: bl_flash_init...\r\n");
    /* Initialize subsystems */
    bl_flash_init();
    early_debug_print("[BL] bl_init: bl_crc_init...\r\n");
    bl_crc_init();
    early_debug_print("[BL] bl_init: bl_dfu_init...\r\n");
    bl_dfu_init();

    early_debug_print("[BL] bl_init: done\r\n");
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
    /* Check for DFU request flag - use early detection result first (most reliable) */
    if (g_early_dfu_flag_detected || bl_flash_is_dfu_requested()) {
        /* Clear the flag (may have already been cleared in main) */
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
    static uint32_t last_print_tick = 0;

    early_debug_print("[BL] detect_protocol: Initializing UART...\r\n");
    /* Initialize both interfaces */
    bl_uart_init();
    early_debug_print("[BL] detect_protocol: Initializing SPI...\r\n");
    bl_spi_init();

    /* Start reception on both */
    early_debug_print("[BL] detect_protocol: Starting RX...\r\n");
    bl_uart_start_rx();
    bl_spi_start_rx();

    early_debug_print("[BL] detect_protocol: Waiting for activity...\r\n");
    last_print_tick = HAL_GetTick();

    /* Wait for activity on either interface */
    while ((HAL_GetTick() - start_tick) < BL_DETECTION_TIMEOUT_MS) {
        /* Print progress every second */
        if ((HAL_GetTick() - last_print_tick) >= 1000) {
            early_debug_print(".");
            last_print_tick = HAL_GetTick();
        }

        if (bl_uart_has_data()) {
            early_debug_print("\r\n[BL] UART activity detected!\r\n");
            detected_protocol = BL_PROTO_UART;
            bl_spi_stop_rx();
            bl_spi_deinit();
            current_state = BL_STATE_DFU_UART;
            return;
        }

        if (bl_spi_has_data()) {
            early_debug_print("\r\n[BL] SPI activity detected!\r\n");
            detected_protocol = BL_PROTO_SPI;
            bl_uart_stop_rx();
            /* NOTE: Do NOT call bl_uart_deinit() - it disables LPUART1 which
             * is used by early_debug_print() for debug output. Just stop RX. */
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
    static bool first_call = true;
    if (first_call) {
        first_call = false;
        early_debug_print("[BL] DFU_SPI state entered\r\n");
        /* CRITICAL: Do NOT flush RX here! The detect_protocol phase already
         * received the first command that triggered SPI detection.
         * Flushing would lose that command and cause communication failure.
         *
         * The first command that triggered detection is already in the RX buffer
         * and will be processed on the first call to bl_spi_process() below.
         */
    }

    /* Process SPI commands */
    if (bl_spi_process()) {
        uint8_t spi_cmd = bl_spi_get_command();
        /* Buffer must be BL_CHUNK_SIZE + 4 for WRITE_DATA (address prepended) */
        uint8_t payload[BL_CHUNK_SIZE + 4];
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
                case SPI_CMD_DFU_ENTER:
                    /* DFU_ENTER received while already in bootloader - just respond OK */
                    bl_spi_send_response(DFU_RSP_OK, NULL, 0);
                    return;
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

    /* Deinitialize peripherals - DISABLED for debug, causing crash */
    /* bl_hw_deinit(); */

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
    SCB->VTOR = (header && bl_header_magic_valid(header)) ? header->app_start_addr : APP_FLASH_BASE;

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
    if (bl_validate_app_header(header)) {
        /* Valid header - check CRC */
        if (bl_validate_app_crc(header)) {
            return true;
        }
    }

    /* Fallback: Check if there's valid code at APP_FLASH_BASE
     * This allows booting apps without a proper header (legacy mode)
     * Valid app has stack pointer in RAM range (0x20000000 - 0x20010000) */
    uint32_t app_stack = *(__IO uint32_t*)APP_FLASH_BASE;
    uint32_t app_entry = *(__IO uint32_t*)(APP_FLASH_BASE + 4);

    /* Stack pointer must be in RAM */
    if (app_stack < 0x20000000UL || app_stack > 0x20010000UL) {
        return false;
    }

    /* Entry point must be in app flash region */
    if (app_entry < APP_FLASH_BASE || app_entry > (APP_FLASH_BASE + APP_FLASH_SIZE)) {
        return false;
    }

    /* Looks like valid code - allow boot in legacy mode */
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

    /* Initialize HSI oscillator with PLL to match application clock (32 MHz)
     * This ensures SPI/DMA timing is identical to the working application.
     * PLL configuration: HSI 16MHz / 4 * 32 / 4 = 32 MHz */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
    RCC_OscInitStruct.PLL.PLLN = 32;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        /* Clock failure is critical */
        Error_Handler();
    }

    /* Configure system clock: SYSCLK = PLL = 32 MHz (same as application) */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
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

/* Simple early debug output via LPUART1 (same as app uses)
 * LPUART1 is on PA2 (TX) / PA3 (RX) with AF8
 * Switches to HSI 16MHz for predictable baud rate */
static void early_debug_init(void)
{
    /* When coming from app, the clock could be anything (MSI at various ranges,
     * HSI, PLL, etc.). To get reliable UART output, switch to HSI 16MHz first. */

    /* Enable HSI */
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));  /* Wait for HSI ready */

    /* Switch system clock to HSI */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (0x01UL << RCC_CFGR_SW_Pos);
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != (0x01UL << RCC_CFGR_SWS_Pos));

    /* Now clock is HSI 16MHz - enable clocks for GPIO and LPUART1 */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;  /* LPUART1 is on APB1ENR2 */
    __DSB();
    for (volatile int i = 0; i < 100; i++);  /* Small delay for clock stabilization */

    /* Reset LPUART1 first to clear any previous configuration */
    RCC->APB1RSTR2 |= RCC_APB1RSTR2_LPUART1RST;
    RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_LPUART1RST;

    /* PA2 = LPUART1_TX, AF8 (same as app uses) */
    GPIOA->MODER &= ~(3UL << (2 * 2));
    GPIOA->MODER |= (2UL << (2 * 2));  /* AF mode */
    GPIOA->AFR[0] &= ~(0xFUL << (2 * 4));
    GPIOA->AFR[0] |= (8UL << (2 * 4));  /* AF8 for LPUART1 */

    /* Configure LPUART1: 115200 baud @ 16MHz HSI
     * LPUART BRR formula: BRR = 256 * LPUART_CLK / BAUD
     * At 16MHz HSI: BRR = 256 * 16000000 / 115200 = 35556 (0x8AE4) */
    LPUART1->BRR = 35556;
    LPUART1->CR1 = USART_CR1_TE | USART_CR1_UE;

    /* Wait for USART to be ready */
    while (!(LPUART1->ISR & USART_ISR_TEACK));
}

/**
 * @brief Reconfigure LPUART1 clock source after bl_hw_init() switches to PLL
 *
 * The application uses HSI (16 MHz) as LPUART1 clock source, not PCLK.
 * This ensures the baud rate stays at 115200 regardless of SYSCLK.
 * We must do the same in bootloader to match the application behavior.
 */
static void early_debug_update_baudrate(void)
{
    /* Disable UART to modify clock source */
    LPUART1->CR1 &= ~USART_CR1_UE;

    /* Configure LPUART1 clock source to HSI (16 MHz) - same as application
     * RCC_CCIPR register: LPUART1SEL bits [11:10]
     * 00: PCLK, 01: SYSCLK, 10: HSI16, 11: LSE */
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_LPUART1SEL_Msk) |
                 (2UL << RCC_CCIPR_LPUART1SEL_Pos);  /* HSI16 */

    /* BRR stays at 35556 for 115200 baud @ 16MHz HSI */
    /* (already configured in early_debug_init) */

    /* Re-enable UART */
    LPUART1->CR1 |= USART_CR1_UE;

    /* Wait for USART to be ready */
    while (!(LPUART1->ISR & USART_ISR_TEACK));
}

void early_debug_print(const char *str)
{
    while (*str) {
        while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF));
        LPUART1->TDR = *str++;
    }
    while (!(LPUART1->ISR & USART_ISR_TC));
}

/* Helper to print a hex digit */
static void early_debug_hex_char(uint8_t nib)
{
    char c = (nib < 10) ? ('0' + nib) : ('A' + nib - 10);
    while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF));
    LPUART1->TDR = c;
}

/* Helper to print a 32-bit hex value */
static void early_debug_hex32(uint32_t val)
{
    for (int i = 7; i >= 0; i--) {
        early_debug_hex_char((val >> (i * 4)) & 0xF);
    }
}

/* Main entry point for bootloader */
int main(void)
{
    /* ABSOLUTE FIRST: Capture SRAM flag before ANYTHING else touches memory */
    volatile uint32_t early_sram_flag = *((volatile uint32_t *)0x2000FFF8);
    volatile uint32_t early_tamp_flag;

    /* Enable backup domain to read TAMP register */
    RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;
    PWR->CR1 |= PWR_CR1_DBP;
    for (volatile int i = 0; i < 100; i++);
    early_tamp_flag = *((volatile uint32_t *)0x4000B100);

    /* CRITICAL: Set vector table to bootloader's address BEFORE anything else.
     * When app jumps to bootloader, VTOR may still point to app's vector table.
     * If any interrupt fires before this, it'll use wrong handlers and crash. */
    SCB->VTOR = BL_FLASH_BASE;
    __DSB();
    __ISB();

    /* Now safe to enable interrupts - they'll use bootloader's vector table */
    __enable_irq();

    /* Initialize early debug output FIRST for diagnostic visibility */
    early_debug_init();
    early_debug_print("\r\n[BL] === Bootloader v1.0 started ===\r\n");

    /* Print the VERY EARLY captured flag values */
    early_debug_print("[BL] EARLY capture (before any init): SRAM=0x");
    early_debug_hex32(early_sram_flag);
    early_debug_print(" TAMP=0x");
    early_debug_hex32(early_tamp_flag);
    early_debug_print("\r\n");

    /* Print BSS boundaries to verify they don't touch 0x2000FFF8 */
    extern uint32_t _sbss, _ebss, _sdata, _edata;
    early_debug_print("[BL] BSS: 0x");
    early_debug_hex32((uint32_t)&_sbss);
    early_debug_print(" - 0x");
    early_debug_hex32((uint32_t)&_ebss);
    early_debug_print(" DATA: 0x");
    early_debug_hex32((uint32_t)&_sdata);
    early_debug_print(" - 0x");
    early_debug_hex32((uint32_t)&_edata);
    early_debug_print("\r\n");

    /* If early SRAM flag was valid, use it directly! */
    if (early_sram_flag == 0x4446554D || early_tamp_flag == 0x4446554D) {
        early_debug_print("[BL] *** DFU FLAG FOUND EARLY - FORCING DFU MODE ***\r\n");
        g_early_dfu_flag_detected = true;
        /* Clear the flags now */
        *((volatile uint32_t *)0x2000FFF8) = 0;
        *((volatile uint32_t *)0x4000B100) = 0;
        __DSB();
    }

    /* Visual indicator removed - PB5 is used for SPI1_MOSI */

    /* Check DFU flag before full init */
    early_debug_print("[BL] Checking DFU flags...\r\n");
    {
        /* Enable backup domain access */
        RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;
        PWR->CR1 |= PWR_CR1_DBP;
        for (volatile int i = 0; i < 1000; i++);

        uint32_t tamp_flag = *((volatile uint32_t *)0x4000B100);
        uint32_t sram_flag = *((volatile uint32_t *)0x2000FFF8);

        early_debug_print("[BL] TAMP=0x");
        early_debug_hex32(tamp_flag);
        early_debug_print(" SRAM=0x");
        early_debug_hex32(sram_flag);
        early_debug_print("\r\n");

        if (sram_flag == 0x4446554D || tamp_flag == 0x4446554D) {
            early_debug_print("[BL] DFU flag FOUND - entering DFU mode\r\n");
        } else {
            early_debug_print("[BL] No DFU flag - will check app validity\r\n");
        }
    }

    early_debug_print("[BL] Initializing HAL and peripherals...\r\n");
    bl_init();
    early_debug_print("[BL] Init complete, entering main loop\r\n");
    bl_run();

    /* Should never reach here */
    early_debug_print("[BL] ERROR: bl_run() returned!\r\n");
    while (1);
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
 * @brief SysTick interrupt handler - required by HAL for timeouts
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
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

/**
 * @brief HardFault handler - provides debug info before hanging
 * This overrides the weak Default_Handler from startup
 */
void HardFault_Handler(void)
{
    /* Try to output debug message if LPUART1 was initialized */
    const char msg[] = "\r\n!!! HARDFAULT !!!\r\n";
    for (int i = 0; msg[i]; i++) {
        /* Check if LPUART1 is enabled and try to send */
        if (LPUART1->CR1 & USART_CR1_UE) {
            while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF));
            LPUART1->TDR = msg[i];
        }
    }
    if (LPUART1->CR1 & USART_CR1_UE) {
        while (!(LPUART1->ISR & USART_ISR_TC));
    }

    /* Toggle GPIO to indicate fault visually */
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    GPIOB->MODER &= ~(3UL << (5 * 2));
    GPIOB->MODER |= (1UL << (5 * 2));
    while (1) {
        GPIOB->ODR ^= (1UL << 5);
        for (volatile int i = 0; i < 500000; i++);
    }
}

/* Note: DMA functions are provided by stm32wlxx_hal_dma.c
 * SPI uses DMA for reliable slave operation - do not add stubs here */
