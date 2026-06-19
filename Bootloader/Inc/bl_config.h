/**
 * @file    bl_config.h
 * @brief   Bootloader configuration and memory layout definitions
 * @date    2025
 *
 * @defgroup BOOTLOADER Bootloader
 * @brief    DFU bootloader for STM32WL55 firmware updates
 *
 * The bootloader provides firmware update (DFU) capability via UART or SPI.
 * It is located at 0x08033000 (32 KB), after the main application (204 KB at 0x08000000).
 *
 * @defgroup BL_CONFIG Bootloader Configuration
 * @ingroup  BOOTLOADER
 * @brief    Memory layout, DFU commands, response codes, and state machine definitions
 * @{
 *
 * @page bootloader_page Bootloader Architecture
 *
 * @section bl_memory_layout Flash Memory Layout (256 KB)
 *
 * | Region        | Address Range             | Size   |
 * |---------------|---------------------------|--------|
 * | Application   | 0x08000000 - 0x08032FFF   | 204 KB |
 * | Bootloader    | 0x08033000 - 0x0803AFFF   | 32 KB  |
 * | NVM (Kineis)  | 0x0803B000 - 0x0803FFFF   | 20 KB  |
 *
 * @section bl_boot_flow Boot Flow
 *
 * @verbatim
 *   MCU Reset (0x08000000)
 *       |
 *       v
 *   Application starts
 *       |
 *       +-- Normal operation (no DFU flag)
 *       |
 *       +-- AT+BOOT received --> Set DFU flag in SRAM + TAMP register
 *                                  |
 *                                  v
 *                              NVIC_SystemReset()
 *                                  |
 *                                  v
 *                              Application restarts, detects DFU flag
 *                                  |
 *                                  v
 *                              Jump to bootloader (0x08033000)
 *                                  |
 *                                  v
 *                              Bootloader runs DFU mode
 *                                  |
 *                                  +-- UART mode (AT+DFU= commands, 9600 baud)
 *                                  |
 *                                  +-- SPI mode (A+ protocol, command IDs 0x30-0x3F)
 *                                  |
 *                                  v
 *                              Firmware written + CRC verified
 *                                  |
 *                                  v
 *                              JUMP command --> MCU Reset --> New app runs
 * @endverbatim
 *
 * @section bl_dfu_request DFU Request Mechanism
 *
 * The application requests DFU mode via:
 * - SRAM flag at 0x2000FFF8 (magic value 0x4446554D = "DFUM")
 * - TAMP backup register at 0x4000B100 (same magic, survives reset)
 *
 * @section bl_project_structure Bootloader Source Files
 *
 * | File                | Description                              |
 * |---------------------|------------------------------------------|
 * | bl_config.h         | Configuration, memory layout, commands   |
 * | bl_main.c/h         | Init, state machine, jump-to-app         |
 * | bl_dfu.c/h          | DFU command processing                   |
 * | bl_flash.c/h        | Flash erase/write/read/verify            |
 * | bl_uart.c/h         | UART DFU driver (AT+DFU= commands)       |
 * | bl_spi.c/h          | SPI slave DFU driver                     |
 * | bl_spi_protocol.c/h | A+ protocol framing for SPI              |
 * | bl_crc.c/h          | CRC-32/MPEG-2 calculation                |
 * | bl_app_header.c/h   | Application header validation            |
 */

#ifndef BL_CONFIG_H
#define BL_CONFIG_H

#include <stdint.h>

/*******************************************************************************
 * DEBUG CONTROL
 ******************************************************************************/
/** @brief Enable bootloader debug prints (controlled by Makefile DEBUG=1) */
#ifdef DEBUG
#define BL_DEBUG
#endif

/** @brief Command buffer size for UART DFU parsing.
 *  Must hold a full WRITE line for the largest chunk the bootloader accepts:
 *  "AT+DFU=WRITE," + 8 addr + "," + 2*BL_CHUNK_SIZE hex + "\r\n" = ~520 chars
 *  at BL_CHUNK_SIZE=248. Undersizing here silently truncates the line into a
 *  short WRITE (data gaps) -> keep >= that bound. */
#define BL_CMD_BUFFER_SIZE      600

/** @brief Peripheral settle delay (short busy-wait after reset, ~3us at 32MHz) */
#define BL_SETTLE_DELAY(n) do { for (volatile int _i = 0; _i < (n); _i++); } while(0)

/*******************************************************************************
 * BOOTLOADER VERSION
 ******************************************************************************/
#define BL_VERSION_MAJOR        1
#define BL_VERSION_MINOR        0
#define BL_VERSION_PATCH        0
#define BL_VERSION              ((BL_VERSION_MAJOR << 16) | (BL_VERSION_MINOR << 8) | BL_VERSION_PATCH)
#define BL_VERSION_STRING       "BL_V1.0.0"

/*******************************************************************************
 * FLASH MEMORY LAYOUT (256 KB Total)
 *
 * NEW LAYOUT: Bootloader AFTER application (Kineis library compatibility)
 *
 * 0x08000000 +------------------+
 *            | APPLICATION      | 204 KB (102 pages)
 * 0x08033000 +------------------+
 *            | BOOTLOADER       | 32 KB (16 pages)
 * 0x0803B000 +------------------+
 *            | FLASH_USER       | 20 KB (Kineis config) - PRESERVED
 * 0x08040000 +------------------+ End of Flash
 *
 * Boot process:
 * 1. MCU starts at 0x08000000 (application)
 * 2. Application checks DFU flag in RTC backup register
 * 3. If DFU requested, app jumps to bootloader at 0x08033000
 * 4. Bootloader handles DFU, then resets to run new app
 ******************************************************************************/

/* Application code region - starts at 0x08000000 for Kineis compatibility */
#define APP_FLASH_BASE          0x08000000UL
#define APP_FLASH_SIZE          0x33000UL       /* 204 KB */
#define APP_FLASH_END           (APP_FLASH_BASE + APP_FLASH_SIZE - 1)
#define APP_MAX_SIZE            APP_FLASH_SIZE

/* Bootloader region - AFTER application */
#define BL_FLASH_BASE           0x08033000UL
#define BL_FLASH_SIZE           0x8000UL        /* 32 KB */
#define BL_FLASH_END            (BL_FLASH_BASE + BL_FLASH_SIZE - 1)
#define BL_FLASH_PAGES          16              /* 32KB / 2KB per page */

/* Flash user data (Kineis config - shared with app) - PRESERVED AT ORIGINAL ADDRESS */
#define FLASH_USER_START        0x0803B000UL
#define FLASH_USER_SIZE_TOTAL   0x5000UL        /* 20 KB total */

/* Bootloader state storage - first 2KB of flash_user */
#define BL_STATE_FLASH_ADDR     0x0803B000UL
#define BL_STATE_FLASH_SIZE     0x800UL         /* 2 KB (1 page) */

/* ISR vector at 0x08000000, app header at 0x08000200 (after vector table) */
#define APP_HEADER_OFFSET       0x200UL                 /* Header offset from flash base */
#define APP_HEADER_ADDR         (APP_FLASH_BASE + APP_HEADER_OFFSET)  /* 0x08000200 */
#define APP_HEADER_SIZE         0x100UL                 /* 256 bytes header */

/* Flash parameters - use BL_ prefix to avoid conflict with HAL */
#define BL_FLASH_PAGE_SIZE      0x800UL         /* 2 KB per page */
#define BL_FLASH_TOTAL_SIZE     (256 * 1024UL)  /* 256 KB */

/*******************************************************************************
 * RAM AND DFU FLAG ADDRESSES (shared between bl_main.c and bl_flash.c)
 ******************************************************************************/
#define SRAM_BASE_ADDR          0x20000000UL
#define SRAM_END_ADDR           0x20010000UL    /* 64 KB SRAM */
#define SRAM_DFU_FLAG_ADDR      0x2000FFF8UL    /* DFU flag location in SRAM */
#define SRAM_DFU_PROTO_ADDR     0x2000FFFCUL    /* DFU protocol selection in SRAM */
#define TAMP_BKP0R_ADDR         0x4000B100UL    /* TAMP backup register 0 */
#define DFU_REQUEST_MAGIC       0x4446554DUL    /* "DFUM" - DFU Mode request */

/* Protocol selection values (written by app, read by bootloader) */
#define DFU_PROTO_NONE          0x00000000UL    /* Auto-detect (legacy race) */
#define DFU_PROTO_UART          0x55415254UL    /* "UART" - Force UART mode */
#define DFU_PROTO_SPI           0x53504921UL    /* "SPI!" - Force SPI mode */

/*******************************************************************************
 * APPLICATION HEADER MAGIC AND VERSION
 ******************************************************************************/
#define APP_HEADER_MAGIC        0x4B494E45UL    /* "KINE" in ASCII */
#define APP_HEADER_VERSION      0x0001

/*******************************************************************************
 * DFU PROTOCOL CONFIGURATION
 ******************************************************************************/

/* Protocol detection */
#define BL_DETECTION_TIMEOUT_MS     3000        /* 3 seconds */
#define BL_DEFAULT_PROTOCOL         BL_PROTO_UART

/* UART configuration - baudrate depends on protocol mode
 * UART mode: 9600 baud (for UART DFU communication)
 * SPI mode: 115200 baud (for debug output while SPI handles DFU)
 */
/* Baud can be overridden from the Makefile (-DBL_UART_BAUDRATE=...) to decouple
 * the UART DFU speed from the PROTOCOL flag — e.g. a UW_DOPPLER board whose app
 * console is 115200 needs the bootloader at 115200 even in UART-DFU mode. */
#ifndef BL_UART_BAUDRATE
#ifdef BL_PROTOCOL_UART
#define BL_UART_BAUDRATE            9600
#else
#define BL_UART_BAUDRATE            115200
#endif
#endif
#define BL_UART_WORDLENGTH          UART_WORDLENGTH_8B
#define BL_UART_STOPBITS            UART_STOPBITS_1
#define BL_UART_PARITY              UART_PARITY_NONE

/* Communication buffers - must be >= BL_SPI_TRANSACTION_SIZE (280) and, for
 * UART DFU, big enough to hold a full WRITE line plus the start of the next
 * one arriving while a flash write blocks the consumer (else the ring
 * overflows mid-transfer and the bootloader desyncs/freezes). */
#define BL_RX_BUFFER_SIZE           1024
#define BL_TX_BUFFER_SIZE           512
#define BL_CHUNK_SIZE               248         /* Data chunk size for writes (31×8=248 for 64-bit flash alignment) */

/* Timeouts */
#define BL_CMD_TIMEOUT_MS           5000        /* Command timeout */
#define BL_WRITE_TIMEOUT_MS         500         /* Flash write timeout */
#define BL_ERASE_TIMEOUT_MS         500         /* Flash erase timeout per page */

/* SPI transaction detection timeout (ms)
 * After receiving bytes, wait this long with no new bytes before
 * considering the transaction complete. With DMA callback fast path,
 * this timeout is only used as fallback. Match app's 3ms timeout. */
#define BL_SPI_RX_STABLE_TIMEOUT_MS 3

/* Retry counts */
#define BL_FLASH_WRITE_RETRIES      3
#define BL_COMM_RETRIES             3

/* SPI transaction size - MUST match Zephyr driver
 * Small commands use 64 bytes, but WRITE_DATA with 256-byte payload needs:
 * Header (4) + Payload (256) + CRC (1) = 261 bytes
 * Use 280 to match Zephyr's DFU_LARGE_TX_SIZE */
#define BL_SPI_TRANSACTION_SIZE     280

/* SPI idle pattern - sent on MISO when no response data
 * 0xAA (10101010) is easy to identify on scope and distinguishes from 0x00/0xFF */
#define BL_SPI_IDLE_PATTERN         0xAA

/* SPI busy pattern - sent while processing command */
#define BL_SPI_BUSY_PATTERN         0xBB

/*******************************************************************************
 * DFU COMMAND IDs (Unified for UART and SPI)
 ******************************************************************************/
/**
 * @brief DFU command identifiers
 *
 * These command IDs are shared between UART and SPI protocols.
 * In UART mode, they are sent as @c AT+DFU=<cmd_id>,<hex_data>.
 * In SPI mode, add @ref SPI_CMD_DFU_BASE (0x30) offset to get the SPI command byte.
 *
 * See @ref dfu_protocol_page for full protocol details.
 */
typedef enum {
    DFU_CMD_PING        = 0x01,     /**< Ping bootloader, returns version string */
    DFU_CMD_GET_INFO    = 0x02,     /**< Get bootloader info (version, app address, max size, page size) */
    DFU_CMD_ERASE       = 0x03,     /**< Erase application flash region, starts DFU session */
    DFU_CMD_WRITE       = 0x04,     /**< Write data chunk: [address(4B)][data(nB)] */
    DFU_CMD_READ        = 0x05,     /**< Read flash data: [address(4B)][length(2B)] */
    DFU_CMD_VERIFY      = 0x06,     /**< Verify CRC-32: [expected_crc(4B)] */
    DFU_CMD_RESET       = 0x07,     /**< System reset (MCU reboot) */
    DFU_CMD_JUMP        = 0x08,     /**< Jump to application (only if CRC verified) */
    DFU_CMD_GET_STATUS  = 0x09,     /**< Get DFU transfer status and progress */
    DFU_CMD_ABORT       = 0x0A,     /**< Abort current DFU session */
    DFU_CMD_SET_HEADER  = 0x0B,     /**< Write application header (256 bytes) */
    DFU_CMD_ENTER       = 0x0F,     /**< Enter DFU mode (sent from app before jump) */
    DFU_CMD_MAX
} dfu_cmd_t;

/* SPI-specific command IDs (offset for MGR_SPI_CMD compatibility) */
#define SPI_CMD_DFU_BASE        0x30
#define SPI_CMD_DFU_PING        (SPI_CMD_DFU_BASE + 0x00)   /* 0x30 */
#define SPI_CMD_DFU_GET_INFO    (SPI_CMD_DFU_BASE + 0x01)   /* 0x31 */
#define SPI_CMD_DFU_ERASE       (SPI_CMD_DFU_BASE + 0x02)   /* 0x32 */
#define SPI_CMD_DFU_WRITE_REQ   (SPI_CMD_DFU_BASE + 0x03)   /* 0x33 */
#define SPI_CMD_DFU_WRITE_DATA  (SPI_CMD_DFU_BASE + 0x04)   /* 0x34 */
#define SPI_CMD_DFU_READ_REQ    (SPI_CMD_DFU_BASE + 0x05)   /* 0x35 */
#define SPI_CMD_DFU_READ_DATA   (SPI_CMD_DFU_BASE + 0x06)   /* 0x36 */
#define SPI_CMD_DFU_VERIFY      (SPI_CMD_DFU_BASE + 0x07)   /* 0x37 */
#define SPI_CMD_DFU_RESET       (SPI_CMD_DFU_BASE + 0x08)   /* 0x38 */
#define SPI_CMD_DFU_JUMP        (SPI_CMD_DFU_BASE + 0x09)   /* 0x39 */
#define SPI_CMD_DFU_GET_STATUS  (SPI_CMD_DFU_BASE + 0x0A)   /* 0x3A */
#define SPI_CMD_DFU_ABORT       (SPI_CMD_DFU_BASE + 0x0B)   /* 0x3B */
#define SPI_CMD_DFU_SET_HEADER  (SPI_CMD_DFU_BASE + 0x0C)   /* 0x3C */
#define SPI_CMD_DFU_ENTER       (SPI_CMD_DFU_BASE + 0x0F)   /* 0x3F */

/*******************************************************************************
 * DFU RESPONSE CODES
 ******************************************************************************/
/**
 * @brief DFU response/error codes
 *
 * Returned by all DFU commands. In UART mode, sent as @c +DFU:<status>.
 * In SPI A+ protocol, sent in the STATUS field of the response frame.
 *
 * See @ref error_codes_page for descriptions of each error condition.
 */
typedef enum {
    DFU_RSP_OK              = 0x00, /**< Success */
    DFU_RSP_ERROR           = 0x01, /**< Generic error */
    DFU_RSP_CRC_ERROR       = 0x02, /**< CRC-32 mismatch during VERIFY */
    DFU_RSP_ADDR_ERROR      = 0x03, /**< Address out of range or not aligned */
    DFU_RSP_SIZE_ERROR      = 0x04, /**< Invalid data size */
    DFU_RSP_FLASH_ERROR     = 0x05, /**< Flash erase/write/read failure */
    DFU_RSP_BUSY            = 0x06, /**< Device busy, retry later */
    DFU_RSP_INVALID_CMD     = 0x07, /**< Unknown or unsupported command */
    DFU_RSP_TIMEOUT         = 0x08, /**< Operation timed out */
    DFU_RSP_NOT_READY       = 0x09, /**< Prerequisite not met (ERASE required first) */
    DFU_RSP_INVALID_HEADER  = 0x0A, /**< Application header magic invalid */
    DFU_RSP_VERIFY_ERROR    = 0x0B  /**< Flash write verification failed */
} dfu_response_t;

/*******************************************************************************
 * BOOTLOADER STATE MACHINE
 ******************************************************************************/
/**
 * @brief Bootloader state machine states
 *
 * The bootloader progresses through these states during initialization and DFU:
 * INIT -> CHECK_APP -> DETECT_PROTOCOL -> DFU_IDLE -> DFU_UART/DFU_SPI -> JUMP_APP
 */
typedef enum {
    BL_STATE_INIT = 0,          /**< Hardware initialization */
    BL_STATE_CHECK_APP,         /**< Checking application validity */
    BL_STATE_DETECT_PROTOCOL,   /**< Detecting UART or SPI protocol */
    BL_STATE_DFU_IDLE,          /**< DFU idle, waiting for commands */
    BL_STATE_DFU_UART,          /**< Processing UART DFU commands */
    BL_STATE_DFU_SPI,           /**< Processing SPI DFU commands */
    BL_STATE_ERASING,           /**< Erasing application flash */
    BL_STATE_RECEIVING,         /**< Receiving firmware data */
    BL_STATE_WRITING,           /**< Writing firmware to flash */
    BL_STATE_VALIDATE,          /**< Validating firmware CRC */
    BL_STATE_JUMP_APP,          /**< Jumping to application */
    BL_STATE_ERROR              /**< Error state */
} bl_state_t;

/*******************************************************************************
 * PROTOCOL TYPE
 ******************************************************************************/
/** @brief DFU communication protocol type */
typedef enum {
    BL_PROTO_NONE = 0,  /**< No protocol detected yet */
    BL_PROTO_UART,      /**< UART AT command protocol (9600 baud) */
    BL_PROTO_SPI        /**< SPI A+ protocol */
} bl_protocol_t;

/*******************************************************************************
 * BOOTLOADER FLAGS (stored in flash at BL_STATE_FLASH_ADDR)
 ******************************************************************************/
#define BL_FLAG_MAGIC           0x424C464CUL    /* "BLFL" */
#define BL_FLAG_DFU_REQUEST     0x00000001UL    /* App requested DFU mode */
#define BL_FLAG_APP_VALID       0x00000002UL    /* App CRC validated */
#define BL_FLAG_UPDATE_PENDING  0x00000004UL    /* Update in progress */

/** @brief Bootloader persistent state stored in flash at @ref BL_STATE_FLASH_ADDR */
typedef struct __attribute__((packed)) {
    uint32_t magic;             /**< Magic value (@ref BL_FLAG_MAGIC) */
    uint32_t flags;             /**< Status flags (BL_FLAG_xxx) */
    uint32_t last_error;        /**< Last DFU error code */
    uint32_t update_count;      /**< Number of successful firmware updates */
    uint32_t reserved[12];      /**< Reserved for future use */
} bl_state_flash_t;

/** @} */ /* end of BL_CONFIG group */

#endif /* BL_CONFIG_H */
