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
#include <stdlib.h>  /* strtoul for UART hex parsing */

/* Current bootloader state */
static bl_state_t current_state = BL_STATE_INIT;
static bl_protocol_t detected_protocol = BL_PROTO_NONE;

/* Early DFU flag detection */
static volatile bool early_dfu_flag_detected = false;

/* Protocol forced by app (read from SRAM before it gets cleared) */
static volatile uint32_t early_proto_flag = DFU_PROTO_NONE;

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
void Error_Handler(void);

/* early_debug_print is declared in bl_main.h */
static void early_debug_update_baudrate(void);

/* Tick of the last VALID DFU command (any recognized AT+DFU / SPI DFU cmd).
 * Drives the BL_DFU_INACTIVITY_TIMEOUT_MS safety auto-exit in bl_run().
 * 0 = no DFU session started yet (lazily armed on the first DFU-state iter).
 * Declared before bl_run() so the safety check can reference it. */
static uint32_t bl_dfu_activity_tick = 0;

#ifdef BL_LED
/* ---- Optional bootloader activity LEDs (opt-in: Makefile BL_LED=1) --------
 * SMD_STDALONE RGB: PA1=RED, PB4=GREEN, PB5=BLUE, ACTIVE LOW (RESET = on).
 * These pins are also the SPI1 bus, but STANDALONE forces UART DFU and never
 * initialises SPI, so they stay free. Entirely compiled out by default, so
 * already-deployed bootloaders stay byte-identical. The R->G->B "rainbow scan"
 * is deliberately unlike the UW_DOPPLER app's LED use (blue/yellow/green) so
 * you can tell at a glance you are in the bootloader, not the app. */
static uint32_t bl_led_activity_tick = 0;   /* tick of last WRITE/ERASE */
static bool     bl_led_fault         = false;

static void bl_led_rgb(bool r, bool g, bool b)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, r ? GPIO_PIN_RESET : GPIO_PIN_SET); /* RED   */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, g ? GPIO_PIN_RESET : GPIO_PIN_SET); /* GREEN */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, b ? GPIO_PIN_RESET : GPIO_PIN_SET); /* BLUE  */
}

static void bl_led_init(void)
{
    GPIO_InitTypeDef io = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    io.Mode  = GPIO_MODE_OUTPUT_PP;
    io.Pull  = GPIO_NOPULL;
    io.Speed = GPIO_SPEED_FREQ_LOW;
    io.Pin = GPIO_PIN_1; HAL_GPIO_Init(GPIOA, &io);
    io.Pin = GPIO_PIN_4; HAL_GPIO_Init(GPIOB, &io);
    io.Pin = GPIO_PIN_5; HAL_GPIO_Init(GPIOB, &io);
    bl_led_rgb(false, false, false);
}

/* Non-blocking; called every bl_run() iteration. */
static void bl_led_service(void)
{
    /* Drive only in the UART-DFU states. Anything else (SPI, jump, app) ->
     * LEDs off, so the SPI bus and the running app are never disturbed.
     * STANDALONE only ever reaches the UART path. */
    if (current_state != BL_STATE_DFU_UART && current_state != BL_STATE_DFU_IDLE) {
        bl_led_rgb(false, false, false);
        return;
    }

    const uint32_t now = HAL_GetTick();

    if (bl_led_fault) {
        bl_led_rgb(true, false, false);                  /* ERROR: red solid    */
    } else if (bl_led_activity_tick != 0u && (now - bl_led_activity_tick) < 600u) {
        bl_led_rgb(false, (now % 160u) < 80u, false);    /* UPDATING: green fast */
    } else {
        const uint32_t phase = (now / 250u) % 3u;         /* ACTIVE: R->G->B scan */
        bl_led_rgb(phase == 0u, phase == 1u, phase == 2u);
    }
}
#endif /* BL_LED */

void bl_init(void)
{
    BL_DBG("[BL] HAL_Init...\r\n");
    HAL_Init();
    BL_DBG("[BL] HAL_Init done\r\n");

    BL_DBG("[BL] hw_init...\r\n");
    bl_hw_init();
    BL_DBG("[BL] hw_init done\r\n");

    early_debug_update_baudrate();
    BL_DBG("[BL] baudrate updated\r\n");

    bl_flash_init();
    bl_crc_init();
    bl_dfu_init();
#ifdef BL_LED
    bl_led_init();
#endif

    current_state = BL_STATE_INIT;
}

void bl_run(void)
{
    while (1) {
        /* Refresh IWDG every iteration.
         *
         * The Kineis application activates IWDG with a 16 s timeout
         * (Kineis/App/Managers/MGR_WDG/Src/mgr_wdg.c: prescaler /256,
         * reload=2000, LSI=32 kHz → 2000*256/32000 = 16 s). The STM32WL55
         * IWDG cannot be disabled by software once enabled — it survives
         * NVIC_SystemReset and keeps counting. AT+BOOT does a system reset
         * to land here, so the bootloader inherits an active 16 s watchdog
         * that nothing has been petting.
         *
         * Symptom without this refresh: UART DFU stalls deterministically
         * after ~16 s of activity (≈44 KB written at 115200 baud with
         * chunkSize=112). The host sends the next WRITE, the BL has just
         * reset, no reply, host times out at 10 s. From the operator's
         * perspective the flash "fails at 40 %".
         *
         * The main loop polls UART/SPI for a single command per iteration
         * and returns; the longest single call is ERASE (≈2.8 s for a full
         * 128-page app region erase), well under the 16 s budget. A single
         * refresh here is sufficient. Defense-in-depth refreshes inside
         * the page-erase / chunk-write loops in bl_dfu.c would also be
         * fine but are not required while page erase stays this short.
         *
         * Key value 0xAAAA per RM0461 §28.4.1 (IWDG_KR). */
        IWDG->KR = 0xAAAAU;

#ifdef BL_LED
        bl_led_service();
#endif

        /* Safety auto-exit from a stuck DFU session. If we have been sitting in
         * a DFU state with NO valid command for BL_DFU_INACTIVITY_TIMEOUT_MS and
         * a VALID application image is present, jump to it rather than wait
         * forever — recovers a wrong-baudrate / lost-host lockup without NRST
         * (esp. the dfu_explicitly_requested case, which never times out on its
         * own). A bricked unit (invalid app) is left in DFU so it stays
         * recoverable. The clock is lazily armed on the first DFU iteration and
         * refreshed by every valid command (bl_dfu_activity_tick), so an active
         * flash is never interrupted. */
        if (current_state == BL_STATE_DFU_UART ||
            current_state == BL_STATE_DFU_IDLE  ||
            current_state == BL_STATE_DFU_SPI) {
            if (bl_dfu_activity_tick == 0u) {
                bl_dfu_activity_tick = HAL_GetTick();
            } else if ((HAL_GetTick() - bl_dfu_activity_tick) >
                           BL_DFU_INACTIVITY_TIMEOUT_MS &&
                       bl_check_app_valid()) {
                BL_DBG("[BL] DFU inactivity timeout — valid app present, jumping\r\n");
                current_state = BL_STATE_JUMP_APP;
            }
        }

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

/* Track if DFU was explicitly requested (don't jump back to app after timeout) */
static bool dfu_explicitly_requested = false;

static void bl_state_init(void)
{
    if (early_dfu_flag_detected || bl_flash_is_dfu_requested()) {
        bl_flash_clear_dfu_request();
        dfu_explicitly_requested = true;  /* Remember DFU was requested */
        early_debug_print("[BL] DFU mode requested\r\n");
        current_state = BL_STATE_DETECT_PROTOCOL;
    } else {
        current_state = BL_STATE_CHECK_APP;
    }
}

static void bl_state_check_app(void)
{
    if (bl_check_app_valid()) {
        current_state = BL_STATE_JUMP_APP;
    } else {
        current_state = BL_STATE_DETECT_PROTOCOL;
    }
}

static void bl_state_detect_protocol(void)
{
    /* ── Fast path: protocol forced by app (no detection race) ── */
    if (early_proto_flag == DFU_PROTO_SPI) {
        early_debug_print("[BL] SPI forced — skipping detection\r\n");
        bl_spi_init();
        bl_spi_start_rx();
        detected_protocol = BL_PROTO_SPI;
        current_state = BL_STATE_DFU_SPI;
        return;
    }

    if (early_proto_flag == DFU_PROTO_UART) {
        early_debug_print("[BL] UART forced — skipping detection\r\n");
        bl_uart_init();
        bl_uart_start_rx();
        detected_protocol = BL_PROTO_UART;
        current_state = BL_STATE_DFU_UART;
        return;
    }

    /* ── Legacy path: auto-detect (race between UART and SPI) ── */
    uint32_t start_tick = HAL_GetTick();

    /* Initialize SPI first - gives SPI a head start for detection.
     * After direct jump from app (no hardware reset), the SPI master
     * may already be sending sync frames. Initialize SPI before UART
     * to catch these early frames. */
    bl_spi_init();
    bl_spi_start_rx();

    /* Initialize UART after SPI to give SPI priority */
    bl_uart_init();
    bl_uart_start_rx();

    /* Flush UART RX buffer after a short delay to discard noise bytes.
     * When PA3 is reconfigured from floating/analog to UART AF, the
     * transition can generate false start bits. Wait for any noise
     * to be received, then flush it. */
    HAL_Delay(5);
    bl_uart_flush_rx();

    /* Always use full timeout - DFU request can come from SPI or UART,
     * and the master may need time to re-sync after STM32 reset/jump */
    uint32_t timeout = BL_DETECTION_TIMEOUT_MS;

    BL_DBG("[BL] Detecting protocol (auto)...\r\n");

    while ((HAL_GetTick() - start_tick) < timeout) {
        /* Check SPI FIRST - SPI master may already be sending sync
         * frames from before bootloader started. UART is checked second
         * because it's more susceptible to noise-triggered false detection. */
        if (bl_spi_has_data()) {
            detected_protocol = BL_PROTO_SPI;
            bl_uart_stop_rx();
            BL_DBG("[BL] SPI detected\r\n");
            current_state = BL_STATE_DFU_SPI;
            return;
        }

        if (bl_uart_has_data()) {
            detected_protocol = BL_PROTO_UART;
            bl_spi_stop_rx();
            bl_spi_deinit();
            BL_DBG("[BL] UART detected\r\n");
            current_state = BL_STATE_DFU_UART;
            return;
        }
    }

    /* Timeout - check if we should stay in DFU mode */
    if (dfu_explicitly_requested) {
        /* DFU was explicitly requested but no data on either interface.
         * Default to UART for manual recovery. */
        detected_protocol = BL_PROTO_UART;
        bl_spi_stop_rx();
        bl_spi_deinit();
        BL_DBG("[BL] Timeout, defaulting to UART\r\n");
        current_state = BL_STATE_DFU_UART;
    } else if (bl_check_app_valid()) {
        bl_uart_stop_rx();
        bl_spi_stop_rx();
        bl_spi_deinit();
        current_state = BL_STATE_JUMP_APP;
    } else {
        detected_protocol = BL_PROTO_UART;
        bl_spi_stop_rx();
        bl_spi_deinit();
        current_state = BL_STATE_DFU_UART;
    }
}

static void bl_state_dfu_uart(void)
{
    /* Use static buffers to reduce stack usage (was causing stack overflow) */
    /* cmd_buffer sized in bl_config.h to hold a full WRITE line (largest chunk) */
    static char cmd_buffer[BL_CMD_BUFFER_SIZE];
    static uint8_t response[BL_TX_BUFFER_SIZE];
    static uint8_t payload[BL_CHUNK_SIZE];

    if (bl_uart_process()) {
        uint16_t cmd_len = bl_uart_get_command(cmd_buffer, sizeof(cmd_buffer));

        if (cmd_len > 0) {
            uint16_t response_len = sizeof(response);
            dfu_response_t status;

            dfu_cmd_t dfu_cmd = DFU_CMD_MAX;
            uint16_t payload_len = 0;

            if (strncmp(cmd_buffer, "AT+DFU=PING", 11) == 0) {
                dfu_cmd = DFU_CMD_PING;
            } else if (strncmp(cmd_buffer, "AT+DFU=INFO", 11) == 0) {
                dfu_cmd = DFU_CMD_GET_INFO;
            } else if (strncmp(cmd_buffer, "AT+DFU=ERASE", 12) == 0) {
                dfu_cmd = DFU_CMD_ERASE;
            } else if (strncmp(cmd_buffer, "AT+DFU=VERIFY", 13) == 0) {
                dfu_cmd = DFU_CMD_VERIFY;
                const char* crc_str = cmd_buffer + 14;
                if (*crc_str == ',') crc_str++;
                char* endptr;
                uint32_t crc = strtoul(crc_str, &endptr, 16);
                if (endptr == crc_str) {
                    bl_uart_send_response(DFU_RSP_CRC_ERROR, NULL, 0);
                    return;
                }
                memcpy(payload, &crc, 4);
                payload_len = 4;
            } else if (strncmp(cmd_buffer, "AT+DFU=WRITE", 12) == 0) {
                dfu_cmd = DFU_CMD_WRITE;
                const char* params = cmd_buffer + 13;
                if (*params == ',') params++;
                char* endptr;
                uint32_t addr = strtoul(params, &endptr, 16);
                if (endptr == params) {
                    bl_uart_send_response(DFU_RSP_ADDR_ERROR, NULL, 0);
                    return;
                }
                params = endptr;
                if (*params == ',') params++;

                memcpy(payload, &addr, 4);
                payload_len = 4;

                while (*params && payload_len < BL_CHUNK_SIZE) {
                    if (!((params[0] >= '0' && params[0] <= '9') ||
                          (params[0] >= 'A' && params[0] <= 'F') ||
                          (params[0] >= 'a' && params[0] <= 'f'))) {
                        break;
                    }
                    if (!params[1]) break;
                    char hex[3] = {params[0], params[1], 0};
                    payload[payload_len++] = (uint8_t)strtoul(hex, NULL, 16);
                    params += 2;
                }
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
                const char* params = cmd_buffer + 12;
                if (*params == ',') params++;
                char* endptr;
                uint32_t addr = strtoul(params, &endptr, 16);
                if (endptr == params) {
                    bl_uart_send_response(DFU_RSP_ADDR_ERROR, NULL, 0);
                    return;
                }
                params = endptr;
                if (*params == ',') params++;
                uint16_t len = (uint16_t)strtoul(params, &endptr, 10);
                if (endptr == params || len == 0) {
                    bl_uart_send_response(DFU_RSP_SIZE_ERROR, NULL, 0);
                    return;
                }
                memcpy(payload, &addr, 4);
                memcpy(payload + 4, &len, 2);
                payload_len = 6;
            }

            if (dfu_cmd != DFU_CMD_MAX) {
                /* Valid command → refresh the DFU inactivity safety timer so an
                 * active flash session is never auto-exited. */
                bl_dfu_activity_tick = HAL_GetTick();
                if (dfu_cmd == DFU_CMD_JUMP) {
                    if (!bl_dfu_can_jump()) {
                        bl_uart_send_response(DFU_RSP_NOT_READY, NULL, 0);
                        return;
                    }
                }

                status = bl_dfu_process_cmd(dfu_cmd, payload, payload_len,
                                           response, &response_len);
                bl_uart_send_response(status, response, response_len);
#ifdef BL_LED
                if (status == DFU_RSP_OK &&
                    (dfu_cmd == DFU_CMD_WRITE || dfu_cmd == DFU_CMD_ERASE)) {
                    bl_led_activity_tick = HAL_GetTick();
                    bl_led_fault = false;
                }
#endif

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
    /* Use static buffers to reduce stack usage (same approach as UART handler) */
    static uint8_t payload[BL_CHUNK_SIZE + 4];
    static uint8_t response[BL_TX_BUFFER_SIZE];

    if (bl_spi_process()) {
        uint8_t spi_cmd = bl_spi_get_command();
        uint16_t payload_len = bl_spi_get_payload(payload, sizeof(payload));

        uint16_t response_len = sizeof(response);
        dfu_response_t status;

        dfu_cmd_t dfu_cmd = DFU_CMD_MAX;

        if (spi_cmd >= SPI_CMD_DFU_BASE) {
            /* Valid DFU-range SPI command → refresh the inactivity safety timer
             * (covers ENTER/PING/WRITE/... incl. the early-return cases below). */
            bl_dfu_activity_tick = HAL_GetTick();
            switch (spi_cmd) {
                case SPI_CMD_DFU_PING:      dfu_cmd = DFU_CMD_PING; break;
                case SPI_CMD_DFU_GET_INFO:  dfu_cmd = DFU_CMD_GET_INFO; break;
                case SPI_CMD_DFU_ERASE:     dfu_cmd = DFU_CMD_ERASE; break;
                case SPI_CMD_DFU_WRITE_REQ:
                    bl_spi_send_response(DFU_RSP_OK, NULL, 0);
                    return;
                case SPI_CMD_DFU_WRITE_DATA:dfu_cmd = DFU_CMD_WRITE; break;
                case SPI_CMD_DFU_READ_REQ:
                    {
                        static uint8_t read_buf[BL_CHUNK_SIZE];
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
                    if (bl_spi_has_read_data()) {
                        static uint8_t read_buf[BL_CHUNK_SIZE];
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
                    bl_spi_send_response(DFU_RSP_OK, NULL, 0);
                    return;
                default: break;
            }
        }

        if (dfu_cmd != DFU_CMD_MAX) {
            if (dfu_cmd == DFU_CMD_JUMP) {
                if (!bl_dfu_can_jump()) {
                    bl_spi_send_response(DFU_RSP_NOT_READY, NULL, 0);
                    return;
                }
            }

            if (dfu_cmd == DFU_CMD_ERASE) {
                bl_spi_protocol_set_op_state(BL_DFU_STATE_ERASING);
            } else if (dfu_cmd == DFU_CMD_WRITE) {
                bl_spi_protocol_set_op_state(BL_DFU_STATE_WRITING);
            } else if (dfu_cmd == DFU_CMD_VERIFY) {
                bl_spi_protocol_set_op_state(BL_DFU_STATE_VERIFYING);
            }

            status = bl_dfu_process_cmd(dfu_cmd, payload, payload_len,
                                       response, &response_len);

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

            if (status == DFU_RSP_OK) {
                if (dfu_cmd == DFU_CMD_JUMP) {
                    bl_spi_wait_tx_done();
                    current_state = BL_STATE_JUMP_APP;
                } else if (dfu_cmd == DFU_CMD_RESET) {
                    bl_spi_wait_tx_done();
                    bl_system_reset();
                } else if (dfu_cmd == DFU_CMD_VERIFY && bl_dfu_is_complete()) {
                    bl_spi_wait_tx_done();
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
    BL_DBG("[BL] Validating app...\r\n");

    if (bl_check_app_valid()) {
        early_debug_print("[BL] App VALID, jumping!\r\n");
        if (detected_protocol == BL_PROTO_UART) {
            bl_uart_print("+DFU=OK,VALIDATED\r\n");
        }
        current_state = BL_STATE_JUMP_APP;
    } else {
        early_debug_print("[BL] App INVALID!\r\n");
        if (detected_protocol == BL_PROTO_UART) {
            bl_uart_print("+DFU=ERR,VALIDATION_FAILED\r\n");
        }
#ifdef BL_LED
        bl_led_fault = true;
#endif
        current_state = BL_STATE_ERROR;
    }
}

static void bl_state_error(void)
{
    bl_dfu_reset();
    bl_spi_protocol_set_op_state(BL_DFU_STATE_IDLE);

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

    if (header != NULL && bl_header_magic_valid(header)) {
        app_stack = *(__IO uint32_t*)(header->app_start_addr);
        app_entry = *(__IO uint32_t*)(header->app_start_addr + 4);
    } else {
        /* Fallback: ISR vector at APP_FLASH_BASE (0x08000000) */
        app_stack = *(__IO uint32_t*)APP_FLASH_BASE;
        app_entry = *(__IO uint32_t*)(APP_FLASH_BASE + 4);
    }

    /* Validate stack pointer (must be in RAM) */
    if (app_stack < SRAM_BASE_ADDR || app_stack > SRAM_END_ADDR) {
        return;
    }

    /* Validate entry point (must be in flash) */
    if (app_entry < APP_FLASH_BASE || app_entry > (APP_FLASH_BASE + APP_FLASH_SIZE)) {
        return;
    }

    early_debug_print("[BL] Jump to app\r\n");
#ifdef BL_LED
    bl_led_rgb(false, false, false);   /* LEDs off before handing over to the app */
#endif

    /* Reset SPI peripheral before jumping */
    SPI1->CR1 &= ~SPI_CR1_SPE;
    {
        volatile uint32_t tout = 10000;
        while ((SPI1->SR & SPI_SR_RXNE) && --tout) {
            (void)SPI1->DR;
        }
    }
    for (volatile int i = 0; i < 1000 && (SPI1->SR & SPI_SR_FTLVL); i++);

    __HAL_RCC_SPI1_FORCE_RESET();
    BL_SETTLE_DELAY(100);
    __HAL_RCC_SPI1_RELEASE_RESET();

    __HAL_RCC_DMA1_FORCE_RESET();
    BL_SETTLE_DELAY(100);
    __HAL_RCC_DMA1_RELEASE_RESET();

    __disable_irq();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    SCB->VTOR = (header && bl_header_magic_valid(header)) ? header->app_start_addr : APP_FLASH_BASE;

    __set_MSP(app_stack);

    pFunction app_reset_handler = (pFunction)app_entry;
    app_reset_handler();

    while (1);
}

bool bl_check_app_valid(void)
{
    const app_header_t* header = bl_get_app_header();

    if (bl_validate_app_header(header)) {
        if (bl_validate_app_crc(header)) {
            return true;
        }
    }

    /* Fallback: Check for valid code at APP_FLASH_BASE (ISR vector at 0x08000000) */
    uint32_t app_stack = *(__IO uint32_t*)APP_FLASH_BASE;
    uint32_t app_entry = *(__IO uint32_t*)(APP_FLASH_BASE + 4);

    if (app_stack < SRAM_BASE_ADDR || app_stack > SRAM_END_ADDR) {
        return false;
    }

    if (app_entry < APP_FLASH_BASE || app_entry > (APP_FLASH_BASE + APP_FLASH_SIZE)) {
        return false;
    }

    return true;
}

bool bl_validate_app_header(const app_header_t* header)
{
    if (header == NULL) {
        return false;
    }

    if (!bl_header_magic_valid(header)) {
        return false;
    }

    if (!bl_header_version_valid(header)) {
        return false;
    }

    if (header->app_start_addr != APP_FLASH_BASE) {
        return false;
    }

    if (header->app_size == 0 || header->app_size > APP_MAX_SIZE) {
        return false;
    }

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

    BL_DBG("[HW] voltage...\r\n");
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Configure PLL: HSI 16MHz / 4 * 32 / 4 = 32 MHz */
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
    BL_DBG("[HW] OscConfig...\r\n");
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        early_debug_print("[HW] OscConfig FAIL\r\n");
        Error_Handler();
    }
    BL_DBG("[HW] OscConfig OK\r\n");

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK |
                                  RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;
    BL_DBG("[HW] ClockConfig...\r\n");
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        early_debug_print("[HW] ClockConfig FAIL\r\n");
        Error_Handler();
    }
    BL_DBG("[HW] ClockConfig OK\r\n");

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}


/* Early debug output via LPUART1 */
/* BRR calculation for LPUART @ 16MHz (HSI):
 * BRR = 256 * clock / baudrate
 * 9600 baud:   BRR = 256 * 16000000 / 9600   = 426667
 * 115200 baud: BRR = 256 * 16000000 / 115200 = 35556
 */
#define BRR_9600_HSI16      426667UL
#define BRR_115200_HSI16    35556UL

/* Follow the resolved BL_UART_BAUDRATE (which may be overridden) so the boot
 * banner and the DFU UART always agree. Defaults are unchanged: PROTOCOL=UART
 * -> 9600 -> BRR_9600, PROTOCOL=SPI -> 115200 -> BRR_115200. */
#if BL_UART_BAUDRATE == 9600
#define EARLY_DEBUG_BRR     BRR_9600_HSI16
#else
#define EARLY_DEBUG_BRR     BRR_115200_HSI16
#endif

static void early_debug_init(void)
{
    volatile uint32_t timeout;

    /* Enable HSI */
    RCC->CR |= RCC_CR_HSION;
    timeout = 1000000;
    while (!(RCC->CR & RCC_CR_HSIRDY) && --timeout);
    if (!timeout) {
        return;
    }

    /* Switch to HSI */
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (0x01UL << RCC_CFGR_SW_Pos);
    timeout = 1000000;
    while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != (0x01UL << RCC_CFGR_SWS_Pos) && --timeout);
    if (!timeout) {
        return;
    }

    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
    __DSB();

    /* Wait for any pending TX to complete before resetting */
    if (LPUART1->CR1 & USART_CR1_UE) {
        timeout = 100000;
        while (!(LPUART1->ISR & USART_ISR_TC) && --timeout);
    }

    /* Disable UART before reset */
    LPUART1->CR1 = 0;

    /* Small delay for peripheral to settle */
    BL_SETTLE_DELAY(1000);

    /* Reset LPUART1 peripheral to clear all buffers */
    RCC->APB1RSTR2 |= RCC_APB1RSTR2_LPUART1RST;
    BL_SETTLE_DELAY(100);
    RCC->APB1RSTR2 &= ~RCC_APB1RSTR2_LPUART1RST;
    BL_SETTLE_DELAY(100);

    /* PA2 = LPUART1_TX, AF8 */
    GPIOA->MODER &= ~(3UL << (2 * 2));
    GPIOA->MODER |= (2UL << (2 * 2));
    GPIOA->AFR[0] &= ~(0xFUL << (2 * 4));
    GPIOA->AFR[0] |= (8UL << (2 * 4));

    /* CRITICAL: Set LPUART1 clock source to HSI16 (value 2) */
    /* This must be done BEFORE setting BRR, as BRR depends on clock frequency */
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_LPUART1SEL_Msk) |
                 (2UL << RCC_CCIPR_LPUART1SEL_Pos);

    /* Set baudrate based on protocol mode (BRR calc assumes HSI16) */
    LPUART1->BRR = EARLY_DEBUG_BRR;
    LPUART1->CR1 = USART_CR1_TE | USART_CR1_UE;

    timeout = 100000;
    while (!(LPUART1->ISR & USART_ISR_TEACK) && --timeout);
}

static void early_debug_update_baudrate(void)
{
    LPUART1->CR1 &= ~USART_CR1_UE;

    /* Use HSI16 as LPUART1 clock source */
    RCC->CCIPR = (RCC->CCIPR & ~RCC_CCIPR_LPUART1SEL_Msk) |
                 (2UL << RCC_CCIPR_LPUART1SEL_Pos);

    LPUART1->CR1 |= USART_CR1_UE;

    {
        volatile uint32_t tout = 100000;
        while (!(LPUART1->ISR & USART_ISR_TEACK) && --tout);
    }
}

void early_debug_print(const char *str)
{
    volatile uint32_t tout;
    while (*str) {
        tout = 100000;
        while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF) && --tout);
        if (!tout) return;
        LPUART1->TDR = *str++;
    }
    tout = 100000;
    while (!(LPUART1->ISR & USART_ISR_TC) && --tout);
}

/* Main entry point */
int main(void)
{
    /* Capture DFU flags early before any memory initialization */
    volatile uint32_t early_sram_flag = *((volatile uint32_t *)SRAM_DFU_FLAG_ADDR);
    volatile uint32_t early_proto_val = *((volatile uint32_t *)SRAM_DFU_PROTO_ADDR);
    volatile uint32_t early_tamp_flag;

    RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;
    PWR->CR1 |= PWR_CR1_DBP;
    BL_SETTLE_DELAY(100);
    early_tamp_flag = *((volatile uint32_t *)TAMP_BKP0R_ADDR);

    /* Set vector table before enabling interrupts */
    SCB->VTOR = BL_FLASH_BASE;
    __DSB();
    __ISB();

    __enable_irq();

    early_debug_init();
    early_debug_print("\r\n[BL] " BL_VERSION_STRING "\r\n");

    /* Check DFU flag */
    if (early_sram_flag == DFU_REQUEST_MAGIC || early_tamp_flag == DFU_REQUEST_MAGIC) {
        early_debug_print("[BL] DFU flag detected\r\n");
        early_dfu_flag_detected = true;

        /* Capture protocol selection before clearing */
        if (early_proto_val == DFU_PROTO_SPI) {
            early_proto_flag = DFU_PROTO_SPI;
            early_debug_print("[BL] Protocol: FORCED SPI\r\n");
        } else if (early_proto_val == DFU_PROTO_UART) {
            early_proto_flag = DFU_PROTO_UART;
            early_debug_print("[BL] Protocol: FORCED UART\r\n");
        } else {
            early_proto_flag = DFU_PROTO_NONE;
            early_debug_print("[BL] Protocol: AUTO-DETECT\r\n");
        }

        /* Clear all flags */
        *((volatile uint32_t *)SRAM_DFU_FLAG_ADDR) = 0;
        *((volatile uint32_t *)SRAM_DFU_PROTO_ADDR) = 0;
        *((volatile uint32_t *)TAMP_BKP0R_ADDR) = 0;
        __DSB();
    }

    BL_DBG("[BL] Starting init...\r\n");
    bl_init();
    BL_DBG("[BL] Init complete\r\n");
    bl_run();

    while (1);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    NVIC_SystemReset();
}

void HardFault_Handler(void)
{
    const char msg[] = "\r\n!!! HARDFAULT !!!\r\n";
    if (LPUART1->CR1 & USART_CR1_UE) {
        for (int i = 0; msg[i]; i++) {
            volatile uint32_t tout = 100000;
            while (!(LPUART1->ISR & USART_ISR_TXE_TXFNF) && --tout);
            if (!tout) break;
            LPUART1->TDR = msg[i];
        }
        volatile uint32_t tout = 100000;
        while (!(LPUART1->ISR & USART_ISR_TC) && --tout);
    }

    while (1);
}
