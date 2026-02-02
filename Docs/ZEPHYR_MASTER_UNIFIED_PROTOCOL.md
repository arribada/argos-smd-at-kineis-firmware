# Zephyr SPI Master - Unified Protocol Reference

This document provides a **unified** reference for implementing a Zephyr SPI master that communicates with both the **Bootloader DFU** and the **Application** firmware.

Both protocols now use **aligned status codes** for consistency.

---

## 1. Protocol Frame Format (A+ Protocol)

Both bootloader and application use the same A+ protocol framing:

```
Request (Master → Slave):
┌──────┬─────┬─────┬─────┬────────────┬───────┐
│ 0xAA │ SEQ │ CMD │ LEN │ DATA[0..n] │ CRC8  │
└──────┴─────┴─────┴─────┴────────────┴───────┘
   1B     1B    1B    1B     0-250B       1B

Response (Slave → Master):
┌──────┬─────┬────────┬─────┬────────────┬───────┐
│ 0x55 │ SEQ │ STATUS │ LEN │ DATA[0..n] │ CRC8  │
└──────┴─────┴────────┴─────┴────────────┴───────┘
   1B     1B     1B      1B     0-250B       1B
```

### CRC-8 CCITT Calculation

```c
uint8_t crc8_ccitt(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

---

## 2. Unified Protocol Status Codes

**These codes are now identical for Bootloader and Application:**

```c
typedef enum {
    /* Standard status codes (0x00-0x0F) */
    PROT_STATUS_OK              = 0x00,  /* Command successful */
    PROT_STATUS_ERROR           = 0x01,  /* Generic error */
    PROT_STATUS_CRC_ERROR       = 0x02,  /* CRC mismatch (data-level) */
    PROT_STATUS_ADDR_ERROR      = 0x03,  /* Invalid address */
    PROT_STATUS_SIZE_ERROR      = 0x04,  /* Invalid size/length */
    PROT_STATUS_FLASH_ERROR     = 0x05,  /* Flash operation failed */
    PROT_STATUS_BUSY            = 0x06,  /* Device busy, retry later */
    PROT_STATUS_INVALID_CMD     = 0x07,  /* Unknown command */
    PROT_STATUS_TIMEOUT         = 0x08,  /* Operation timeout */
    PROT_STATUS_NOT_READY       = 0x09,  /* Prerequisite not met */
    PROT_STATUS_INVALID_HEADER  = 0x0A,  /* Invalid header */
    PROT_STATUS_VERIFY_ERROR    = 0x0B,  /* Verification failed */

    /* Protocol-specific errors (0x10+) */
    PROT_STATUS_FRAME_CRC_ERROR = 0x10,  /* Frame CRC mismatch */
    PROT_STATUS_SEQ_ERROR       = 0x11,  /* Sequence number error */
    PROT_STATUS_FRAME_ERROR     = 0x12,  /* Malformed frame */
} ProtocolStatus;
```

### Status Code Usage Table

| Code | Name | When Returned |
|------|------|---------------|
| 0x00 | OK | Command successful |
| 0x01 | ERROR | Generic/unspecified error |
| 0x02 | CRC_ERROR | DFU data CRC mismatch |
| 0x03 | ADDR_ERROR | Invalid flash address |
| 0x04 | SIZE_ERROR | Invalid size, length, or parameters |
| 0x05 | FLASH_ERROR | Flash write/erase failed |
| 0x06 | BUSY | Device busy (retry later) |
| 0x07 | INVALID_CMD | Unknown command code |
| 0x08 | TIMEOUT | Operation timed out |
| 0x09 | NOT_READY | Prerequisite not met (e.g., no ERASE before WRITE) |
| 0x0A | INVALID_HEADER | Invalid application header |
| 0x0B | VERIFY_ERROR | CRC verification failed |
| 0x10 | FRAME_CRC_ERROR | A+ frame CRC mismatch (resend) |
| 0x11 | SEQ_ERROR | Sequence number mismatch |
| 0x12 | FRAME_ERROR | Malformed A+ frame |

---

## 3. Application MAC Status (Separate from Protocol Status)

The application has a **separate** MAC status for async operations (TX, RX, Satellite detection).
This is retrieved via command `CMD_MAC_STATUS (0x03)`:

```c
typedef enum {
    MAC_UNKNOWN         = 0x00,  /* Unknown status (initial) */
    MAC_OK              = 0x01,  /* Ready for commands */
    MAC_TX_DONE         = 0x02,  /* Transmission completed successfully */
    MAC_TX_SIZE_ERROR   = 0x03,  /* Invalid TX payload size */
    MAC_TXACK_DONE      = 0x04,  /* TX with ACK completed */
    MAC_TX_TIMEOUT      = 0x05,  /* Transmission timed out (no satellite) */
    MAC_TXACK_TIMEOUT   = 0x06,  /* ACK reception timed out */
    MAC_RX_ERROR        = 0x07,  /* Reception error */
    MAC_RX_TIMEOUT      = 0x08,  /* Reception timed out */
    MAC_ERROR           = 0x09,  /* General MAC error */
    /* Extended status codes */
    MAC_TX_IN_PROGRESS  = 0x0A,  /* TX operation queued/in progress */
    MAC_RX_RECEIVED     = 0x0B,  /* Downlink frame received */
    MAC_SAT_DETECTED    = 0x0C,  /* Satellite detected */
    MAC_SAT_LOST        = 0x0D,  /* Satellite lost */
    MAC_RF_ABORTED      = 0x0E,  /* RF operation aborted */
} MACStatus;
```

### MAC Status State Machine for TX

```
    CMD_WRITE_TX (0x16)
           │
           ▼
    ┌──────────────┐
    │TX_IN_PROGRESS│ (0x0A) - Poll MAC_STATUS
    │   (queued)   │
    └──────┬───────┘
           │ KNS_MAC events
           ▼
    ┌──────────────────────────────────┐
    │  TX_DONE (0x02)     = Success    │
    │  TXACK_DONE (0x04)  = Success    │
    │  TX_TIMEOUT (0x05)  = Failed     │
    │  TXACK_TIMEOUT(0x06)= Failed     │
    │  ERROR (0x09)       = Failed     │
    └──────────────────────────────────┘
```

**Important:** MAC status is **only** returned in the payload of `CMD_MAC_STATUS (0x03)`. It is **not** mixed with protocol status codes.

---

## 4. Command Ranges

| Range | Mode | Description |
|-------|------|-------------|
| 0x00 | Both | NOP (get previous response) |
| 0x01-0x2F | Application | Application commands |
| 0x30-0x3E | Bootloader | DFU commands |
| 0x3F | Both | Enter DFU mode |

---

## 5. Zephyr Master Implementation

### 5.1 SPI Configuration

```c
#define SPI_FREQUENCY   1000000  /* 1 MHz */
#define SPI_MODE        0        /* CPOL=0, CPHA=0 */

static const struct spi_config spi_cfg = {
    .frequency = SPI_FREQUENCY,
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
    .slave = 0,
    .cs = { .gpio = CS_GPIO_SPEC, .delay = 10 },
};
```

### 5.2 Unified Status Handler

```c
const char* protocol_status_to_string(uint8_t status)
{
    switch (status) {
        case 0x00: return "OK";
        case 0x01: return "ERROR";
        case 0x02: return "CRC_ERROR";
        case 0x03: return "ADDR_ERROR";
        case 0x04: return "SIZE_ERROR";
        case 0x05: return "FLASH_ERROR";
        case 0x06: return "BUSY";
        case 0x07: return "INVALID_CMD";
        case 0x08: return "TIMEOUT";
        case 0x09: return "NOT_READY";
        case 0x0A: return "INVALID_HEADER";
        case 0x0B: return "VERIFY_ERROR";
        case 0x10: return "FRAME_CRC_ERROR";
        case 0x11: return "SEQ_ERROR";
        case 0x12: return "FRAME_ERROR";
        default:   return "UNKNOWN";
    }
}

bool is_recoverable_error(uint8_t status)
{
    switch (status) {
        case 0x06:  /* BUSY - retry later */
        case 0x10:  /* FRAME_CRC_ERROR - resend */
            return true;
        default:
            return false;
    }
}
```

### 5.3 Transaction Model (Pipelined)

```
Transaction 1: Master TX [CMD_A]     → Master RX [idle 0xAA...]
               (slave processes CMD_A)

Transaction 2: Master TX [CMD_B/NOP] → Master RX [Response to CMD_A]
               (slave processes CMD_B)

Transaction 3: Master TX [CMD_C]     → Master RX [Response to CMD_B]
```

### 5.4 Generic Send/Receive Function

```c
static uint8_t sequence = 0;

int spi_send_command(uint8_t cmd, const uint8_t *data, uint8_t data_len,
                     uint8_t *resp_status, uint8_t *resp_data, uint8_t *resp_len,
                     uint32_t delay_ms)
{
    uint8_t tx_buf[260], rx_buf[260];
    int ret;

    /* Build A+ request frame */
    tx_buf[0] = 0xAA;              /* Magic */
    tx_buf[1] = sequence++;        /* Sequence */
    tx_buf[2] = cmd;               /* Command */
    tx_buf[3] = data_len;          /* Length */
    if (data_len > 0) {
        memcpy(&tx_buf[4], data, data_len);
    }
    tx_buf[4 + data_len] = crc8_ccitt(tx_buf, 4 + data_len);
    uint16_t tx_len = 5 + data_len;

    /* Transaction 1: Send command */
    struct spi_buf tx = { .buf = tx_buf, .len = tx_len };
    struct spi_buf rx = { .buf = rx_buf, .len = tx_len };
    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    if (ret < 0) return ret;

    /* Wait for processing */
    k_sleep(K_MSEC(delay_ms));

    /* Transaction 2: Get response with NOP */
    tx_buf[0] = 0xAA;
    tx_buf[1] = sequence++;
    tx_buf[2] = 0x00;  /* NOP */
    tx_buf[3] = 0x00;
    tx_buf[4] = crc8_ccitt(tx_buf, 4);
    tx.len = 64;
    rx.len = 64;

    ret = spi_transceive(spi_dev, &spi_cfg, &tx_set, &rx_set);
    if (ret < 0) return ret;

    /* Parse response */
    if (rx_buf[0] != 0x55) {
        return -EPROTO;
    }

    uint8_t status = rx_buf[2];
    uint8_t rlen = rx_buf[3];

    /* Verify CRC */
    uint8_t expected_crc = crc8_ccitt(rx_buf, 4 + rlen);
    if (rx_buf[4 + rlen] != expected_crc) {
        return -EILSEQ;
    }

    *resp_status = status;
    if (resp_len) *resp_len = rlen;
    if (resp_data && rlen > 0) {
        memcpy(resp_data, &rx_buf[4], rlen);
    }

    return (status == 0x00) ? 0 : -EIO;
}
```

---

## 6. Bootloader DFU Commands (0x30-0x3F)

| Code | Command | TX Payload | RX Payload | Delay |
|------|---------|------------|------------|-------|
| 0x30 | PING | 0 | ~10B (version) | 15ms |
| 0x31 | GET_INFO | 0 | 17B | 15ms |
| 0x32 | ERASE | 0 | 1B (status) | **3000ms** |
| 0x33 | WRITE_REQ | 6B (addr+len) | 1B | 15ms |
| 0x34 | WRITE_DATA | ≤256B | 1B | 20ms |
| 0x35 | READ_REQ | 6B (addr+len) | 1B | 15ms |
| 0x36 | READ_DATA | 0 | ≤256B | 15ms |
| 0x37 | VERIFY | 4B (CRC32) | 1B | 15ms |
| 0x38 | RESET | 0 | 1B | 100ms |
| 0x39 | JUMP | 0 | 1B | 100ms |
| 0x3A | GET_STATUS | 0 | 32B | 15ms |
| 0x3B | ABORT | 0 | 1B | 15ms |
| 0x3C | SET_HEADER | 256B | 1B | 20ms |
| 0x3F | DFU_ENTER | 0 | 1B | 100ms |

---

## 7. Application Commands (0x01-0x2F)

### General Commands

| Code | Command | TX | RX | Delay |
|------|---------|-----|-----|-------|
| 0x00 | NOP | 0 | Previous | 15ms |
| 0x02 | PING | 0 | 1B | 15ms |
| 0x03 | MAC_STATUS | 0 | 1B (MACStatus) | 15ms |
| 0x05 | READ_VERSION | 0 | 1B | 15ms |
| 0x06 | READ_FIRMWARE | 0 | 16B | 15ms |
| 0x07 | READ_ADDR | 0 | 4B | 15ms |
| 0x08 | READ_ID | 0 | 4B | 15ms |
| 0x09 | READ_SN | 0 | 17B | 15ms |

### TX Command (3-Phase)

```c
/* Phase 1: TX_REQ (0x14) */
spi_send_command(0x14, NULL, 0, &status, NULL, NULL, 15);

/* Phase 2: TX_SIZE (0x15) - size MSB first */
uint8_t size_data[2] = { (size >> 8) & 0xFF, size & 0xFF };
spi_send_command(0x15, size_data, 2, &status, NULL, NULL, 15);

/* Phase 3: TX_DATA (0x16) */
spi_send_command(0x16, payload, payload_len, &status, NULL, NULL, 100);

/* Poll MAC_STATUS until TX_DONE or error */
do {
    k_sleep(K_MSEC(200));
    spi_send_command(0x03, NULL, 0, &status, &mac_status, NULL, 15);
} while (mac_status == MAC_OK && timeout_not_reached);
```

---

## 8. DFU Sequence Example

```c
int perform_dfu(const uint8_t *firmware, size_t fw_size)
{
    uint8_t status, resp[64];
    uint8_t resp_len;

    /* 1. Enter DFU mode (from application) */
    spi_send_command(0x3F, NULL, 0, &status, NULL, NULL, 100);
    k_sleep(K_MSEC(500));  /* Wait for reset */

    /* 2. PING bootloader */
    spi_send_command(0x30, NULL, 0, &status, resp, &resp_len, 15);
    if (status != 0x00) return -1;

    /* 3. ERASE (CRITICAL: 3 second wait!) */
    spi_send_command(0x32, NULL, 0, &status, NULL, NULL, 3000);
    if (status != 0x00) return -2;

    /* 4. WRITE in chunks */
    uint32_t addr = 0x08008100;
    size_t offset = 0;
    while (offset < fw_size) {
        size_t chunk = MIN(256, fw_size - offset);

        /* WRITE_REQ */
        uint8_t req[6];
        memcpy(req, &addr, 4);
        uint16_t len16 = chunk;
        memcpy(req + 4, &len16, 2);
        spi_send_command(0x33, req, 6, &status, NULL, NULL, 15);

        /* WRITE_DATA */
        spi_send_command(0x34, &firmware[offset], chunk, &status, NULL, NULL, 20);
        if (status != 0x00) return -3;

        addr += chunk;
        offset += chunk;
    }

    /* 5. VERIFY */
    uint32_t crc = calculate_crc32(firmware, fw_size);
    spi_send_command(0x37, (uint8_t*)&crc, 4, &status, NULL, NULL, 15);
    if (status != 0x00) return -4;

    /* 6. JUMP to application */
    spi_send_command(0x39, NULL, 0, &status, NULL, NULL, 100);

    return 0;
}
```

---

## 9. Error Recovery

```c
int spi_send_with_retry(uint8_t cmd, const uint8_t *data, uint8_t len,
                         uint8_t *status, uint8_t *resp, uint8_t *resp_len,
                         uint32_t delay_ms, int max_retries)
{
    for (int i = 0; i < max_retries; i++) {
        int ret = spi_send_command(cmd, data, len, status, resp, resp_len, delay_ms);

        if (ret == 0 && *status == 0x00) {
            return 0;  /* Success */
        }

        /* Check for recoverable errors */
        if (*status == 0x10) {  /* FRAME_CRC_ERROR */
            LOG_WRN("Frame CRC error, retrying...");
            k_sleep(K_MSEC(50));
            continue;
        }

        if (*status == 0x06) {  /* BUSY */
            LOG_WRN("Device busy, waiting...");
            k_sleep(K_MSEC(100));
            continue;
        }

        /* Non-recoverable error */
        LOG_ERR("Error 0x%02X: %s", *status, protocol_status_to_string(*status));
        return -EIO;
    }

    return -ETIMEDOUT;
}
```

---

## 10. Quick Reference

### Protocol Status (Unified)
```
0x00 OK              0x06 BUSY           0x10 FRAME_CRC_ERROR
0x01 ERROR           0x07 INVALID_CMD    0x11 SEQ_ERROR
0x02 CRC_ERROR       0x08 TIMEOUT        0x12 FRAME_ERROR
0x03 ADDR_ERROR      0x09 NOT_READY
0x04 SIZE_ERROR      0x0A INVALID_HEADER
0x05 FLASH_ERROR     0x0B VERIFY_ERROR
```

### MAC Status (Application Only - via 0x03)
```
0x00 UNKNOWN         0x06 TXACK_TIMEOUT   0x0C SAT_DETECTED
0x01 OK              0x07 RX_ERROR        0x0D SAT_LOST
0x02 TX_DONE         0x08 RX_TIMEOUT      0x0E RF_ABORTED
0x03 TX_SIZE_ERROR   0x09 ERROR
0x04 TXACK_DONE      0x0A TX_IN_PROGRESS  ← Poll until done!
0x05 TX_TIMEOUT      0x0B RX_RECEIVED
```

### Timing Cheat Sheet
```
Standard commands:     15ms
WRITE_DATA:            20ms
TX_DATA:              100ms
ERASE:               3000ms (CRITICAL!)
RESET/JUMP:           100ms
MAC_STATUS polling:  100-500ms
```

