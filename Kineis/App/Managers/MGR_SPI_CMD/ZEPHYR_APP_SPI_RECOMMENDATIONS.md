# Recommendations for Zephyr SPI Application Master Implementation

This document provides guidelines for implementing the SPI master in Zephyr to communicate with the Kineis application firmware via SPI.

## 1. Protocol Overview

The application supports two protocols:
- **Legacy Protocol**: Direct command byte (backward compatible)
- **A+ Protocol**: Framed with magic byte, sequence, CRC-8 (recommended)

### A+ Protocol Frame Format

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

### CRC-8 Calculation

```c
uint8_t crc8_ccitt(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00;  /* Initial value */

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;  /* Polynomial 0x07 */
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
```

CRC is computed over: `MAGIC + SEQ + CMD/STATUS + LEN + DATA` (excludes CRC byte itself).

---

## 2. Command Reference

### 2.1 General Commands (0x01-0x0A)

| Code | Command | Payload TX | Payload RX | Description |
|------|---------|------------|------------|-------------|
| 0x00 | NOP | 0 | Previous | No operation (get previous response) |
| 0x01 | READ | 0 | 1B | Generic read |
| 0x02 | PING | 0 | 1B | Ping (returns 0x01) |
| 0x03 | MAC_STATUS | 0 | 1B | Read MAC layer status |
| 0x04 | SPI_STATUS | 0 | 1B | Read SPI state |
| 0x05 | READ_VERSION | 0 | 1B | Read SPI command version |
| 0x06 | READ_FIRMWARE | 0 | 16B | Read firmware version string |
| 0x07 | READ_ADDR | 0 | 4B | Read device address |
| 0x08 | READ_ID | 0 | 4B | Read device ID |
| 0x09 | READ_SN | 0 | 17B | Read serial number |
| 0x0A | READ_RCONF | 0 | 16B | Read radio configuration |

### 2.2 Configuration Write Commands (Multi-Phase)

| Phase 1 (REQ) | Phase 2 (DATA) | Payload | Description |
|---------------|----------------|---------|-------------|
| 0x0B WRITE_RCONF_REQ | 0x0C WRITE_RCONF | 16B | Write radio configuration |
| 0x0F WRITE_KMAC_REQ | 0x10 WRITE_KMAC | 1B | Write KMAC profile |
| 0x12 WRITE_LPM_REQ | 0x13 WRITE_LPM | 1B | Write low power mode |
| 0x20 WRITE_ID_REQ | 0x21 WRITE_ID | 4B | Write device ID |
| 0x22 WRITE_ADDR_REQ | 0x23 WRITE_ADDR | 4B | Write device address |
| 0x25 WRITE_SECKEY_REQ | 0x26 WRITE_SECKEY | 16B | Write secret key |
| 0x29 WRITE_TCXOWU_REQ | 0x2A WRITE_TCXOWU | 4B | Write TCXO warmup (ms) |

### 2.3 TX Command (3-Phase Protocol)

| Phase | Code | Command | Payload | Description |
|-------|------|---------|---------|-------------|
| 1 | 0x14 | WRITE_TX_REQ | 0 | Initiate TX request |
| 2 | 0x15 | WRITE_TX_SIZE | 2B (size) | Send payload size (uint16, MSB first) |
| 3 | 0x16 | WRITE_TX | N bytes | Send actual payload data |

### 2.4 Read Commands

| Code | Command | Payload RX | Description |
|------|---------|------------|-------------|
| 0x0E | READ_KMAC | 1B | Read KMAC profile |
| 0x11 | READ_LPM | 1B | Read low power mode |
| 0x24 | READ_SECKEY | 16B | Read secret key |
| 0x27 | READ_SPIMAC_STATE | 2B | Read SPI + MAC state |
| 0x28 | READ_TCXO_WU | 4B | Read TCXO warmup time |

### 2.5 Special Commands

| Code | Command | Description |
|------|---------|-------------|
| 0x3F | DFU_ENTER | Enter bootloader DFU mode (triggers reset) |

---

## 3. MAC Status Codes

The MAC_STATUS command (0x03) returns a single byte indicating the MAC layer state:

| Code | Status | Description |
|------|--------|-------------|
| 0x00 | MAC_UNKNOWN | Unknown status (initial) |
| 0x01 | MAC_OK | Ready for commands |
| 0x02 | MAC_TX_DONE | Transmission completed successfully |
| 0x03 | MAC_TX_SIZE_ERROR | Invalid TX payload size |
| 0x04 | MAC_TXACK_DONE | TX with ACK completed |
| 0x05 | MAC_TX_TIMEOUT | Transmission timed out (no satellite) |
| 0x06 | MAC_TXACK_TIMEOUT | ACK reception timed out |
| 0x07 | MAC_RX_ERROR | Reception error |
| 0x08 | MAC_RX_TIMEOUT | Reception timed out |
| 0x09 | MAC_ERROR | General MAC error |
| 0x0A | MAC_TX_IN_PROGRESS | **TX queued, poll until done!** |
| 0x0B | MAC_RX_RECEIVED | Downlink frame received |
| 0x0C | MAC_SAT_DETECTED | Satellite detected |
| 0x0D | MAC_SAT_LOST | Satellite lost |
| 0x0E | MAC_RF_ABORTED | RF operation aborted |

### TX Polling Flow
```
CMD_WRITE_TX (0x16) → MAC_TX_IN_PROGRESS (0x0A) → Poll CMD_MAC_STATUS (0x03)
                                                    │
                      ┌─────────────────────────────┤
                      ▼                             ▼
               MAC_TX_DONE (0x02)           MAC_TX_TIMEOUT (0x05)
               = Success!                   = Failed (no satellite)
```

---

## 4. Protocol Status Codes (A+ Protocol)

**Now aligned with bootloader for consistency:**

| Code | Status | Description |
|------|--------|-------------|
| 0x00 | OK | Success |
| 0x01 | ERROR | Generic error |
| 0x02 | CRC_ERROR | CRC mismatch (data-level) |
| 0x03 | ADDR_ERROR | Invalid address |
| 0x04 | SIZE_ERROR | Invalid size/length |
| 0x05 | FLASH_ERROR | Flash operation failed |
| 0x06 | BUSY | Device busy, retry later |
| 0x07 | INVALID_CMD | Unknown command |
| 0x08 | TIMEOUT | Operation timeout |
| 0x09 | NOT_READY | Prerequisite not met |
| 0x0A | INVALID_HEADER | Invalid header |
| 0x0B | VERIFY_ERROR | Verification failed |
| 0x10 | FRAME_CRC_ERROR | Frame CRC mismatch (resend) |
| 0x11 | SEQ_ERROR | Sequence number error |
| 0x12 | FRAME_ERROR | Malformed frame |

> **Note:** See `Docs/ZEPHYR_MASTER_UNIFIED_PROTOCOL.md` for complete unified reference.

---

## 5. Zephyr Implementation

### 5.1 SPI Configuration

```c
/* Recommended SPI settings */
#define APP_SPI_FREQUENCY   1000000  /* 1 MHz - safe for DMA slave */
#define APP_SPI_MODE        SPI_MODE_CPHA  /* Mode 0: CPOL=0, CPHA=0 */
#define APP_SPI_WORD_SIZE   8

static const struct spi_config spi_cfg = {
    .frequency = APP_SPI_FREQUENCY,
    .operation = SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |
                 SPI_WORD_SET(APP_SPI_WORD_SIZE),
    .slave = 0,
    .cs = {
        .gpio = APP_CS_GPIO_SPEC,
        .delay = 10,  /* 10us CS delay */
    },
};
```

### 5.2 Transaction Model (CRITICAL)

The application is a SPI slave using DMA. **Response to Transaction N is received during Transaction N+1.**

```
Transaction 1: Master sends CMD_PING
               Master receives: [idle pattern 0xAA...] (ignore)

Transaction 2: Master sends CMD_NOP (0x00) or next command
               Master receives: Response to PING

Transaction 3: Master sends next command
               Master receives: Response to NOP (or previous)
```

### 5.3 Basic Command Implementation

```c
/* Timing constants (milliseconds) */
#define APP_TIMING_DEFAULT_MS   15   /* Standard command delay */
#define APP_TIMING_TX_MS        100  /* TX operation initial delay */
#define APP_TIMING_DFU_MS       100  /* DFU entry delay */

/* Frame builder */
static uint16_t build_aplus_frame(uint8_t *buf, uint8_t seq, uint8_t cmd,
                                   const uint8_t *data, uint8_t data_len)
{
    uint16_t index = 0;

    buf[index++] = 0xAA;        /* Magic */
    buf[index++] = seq;         /* Sequence */
    buf[index++] = cmd;         /* Command */
    buf[index++] = data_len;    /* Length */

    if (data_len > 0 && data != NULL) {
        memcpy(&buf[index], data, data_len);
        index += data_len;
    }

    /* CRC over header + data */
    buf[index] = crc8_ccitt(buf, index);
    index++;

    return index;
}

int app_send_command(uint8_t cmd, const uint8_t *data, uint8_t len,
                     uint8_t *response, uint16_t *resp_len, uint32_t delay_ms)
{
    static uint8_t sequence = 0;
    uint8_t tx_buf[260];
    uint8_t rx_buf[260];

    /* Build A+ frame */
    uint16_t tx_len = build_aplus_frame(tx_buf, sequence++, cmd, data, len);

    /* Transaction 1: Send command */
    struct spi_buf tx = { .buf = tx_buf, .len = tx_len };
    struct spi_buf_set tx_bufs = { .buffers = &tx, .count = 1 };
    struct spi_buf rx = { .buf = rx_buf, .len = tx_len };
    struct spi_buf_set rx_bufs = { .buffers = &rx, .count = 1 };

    spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);

    /* Wait for slave to process */
    k_sleep(K_MSEC(delay_ms));

    /* Transaction 2: Get response with NOP */
    tx_len = build_aplus_frame(tx_buf, sequence++, 0x00, NULL, 0);  /* NOP */
    tx.len = 64;  /* Fixed size for response */
    rx.len = 64;
    memset(&tx_buf[tx_len], 0xAA, 64 - tx_len);  /* Pad with idle */

    spi_transceive(spi_dev, &spi_cfg, &tx_bufs, &rx_bufs);

    /* Parse response */
    return parse_aplus_response(rx_buf, response, resp_len);
}

static int parse_aplus_response(const uint8_t *rx_buf,
                                 uint8_t *response, uint16_t *resp_len)
{
    /* Check magic byte */
    if (rx_buf[0] != 0x55) {
        return -EPROTO;
    }

    uint8_t seq = rx_buf[1];
    uint8_t status = rx_buf[2];
    uint8_t data_len = rx_buf[3];

    /* Verify CRC */
    uint16_t crc_len = 4 + data_len;
    uint8_t expected_crc = crc8_ccitt(rx_buf, crc_len);
    if (rx_buf[crc_len] != expected_crc) {
        return -EILSEQ;  /* CRC error */
    }

    /* Copy data */
    if (data_len > 0 && response != NULL) {
        memcpy(response, &rx_buf[4], data_len);
    }
    if (resp_len != NULL) {
        *resp_len = data_len;
    }

    return (status == 0x00) ? 0 : -EIO;
}
```

---

## 6. TX Operation (3-Phase Protocol)

### 6.1 Complete TX Sequence

```c
/**
 * @brief Send data via Kineis satellite uplink
 * @param payload Data to transmit
 * @param payload_size Size in bytes (max 256)
 * @return 0 on success, negative on error
 */
int app_tx_data(const uint8_t *payload, uint16_t payload_size)
{
    int ret;
    uint8_t resp[8];
    uint16_t resp_len;

    /* Validate size */
    if (payload_size > 256 || payload_size == 0) {
        return -EINVAL;
    }

    /* Phase 1: TX_REQ (0x14) */
    ret = app_send_command(0x14, NULL, 0, resp, &resp_len, 15);
    if (ret != 0) {
        LOG_ERR("TX_REQ failed");
        return ret;
    }

    /* Phase 2: TX_SIZE (0x15) - size as 2 bytes, MSB first */
    uint8_t size_data[2] = {
        (payload_size >> 8) & 0xFF,  /* MSB */
        payload_size & 0xFF          /* LSB */
    };
    ret = app_send_command(0x15, size_data, 2, resp, &resp_len, 15);
    if (ret != 0) {
        LOG_ERR("TX_SIZE failed");
        return ret;
    }

    /* Phase 3: TX_DATA (0x16) - send actual payload */
    ret = app_send_command(0x16, payload, payload_size, resp, &resp_len, 100);
    if (ret != 0) {
        LOG_ERR("TX_DATA failed");
        return ret;
    }

    LOG_INF("TX queued successfully");
    return 0;
}
```

### 6.2 Polling TX Completion (CRITICAL)

After TX_DATA, the transmission is **asynchronous**. You MUST poll MAC_STATUS to know when it completes.

```c
/**
 * @brief Poll MAC status until TX completes or times out
 * @param timeout_ms Maximum time to wait
 * @return MAC status code, or negative error
 */
int app_wait_tx_complete(uint32_t timeout_ms)
{
    uint8_t resp[8];
    uint16_t resp_len;
    uint32_t start = k_uptime_get_32();

    while ((k_uptime_get_32() - start) < timeout_ms) {
        /* Poll MAC_STATUS (0x03) */
        int ret = app_send_command(0x03, NULL, 0, resp, &resp_len, 15);
        if (ret != 0) {
            k_sleep(K_MSEC(100));
            continue;
        }

        uint8_t mac_status = resp[0];

        switch (mac_status) {
            case 0x02:  /* MAC_TX_DONE */
                LOG_INF("TX completed successfully");
                return 0;

            case 0x04:  /* MAC_TXACK_DONE */
                LOG_INF("TX with ACK completed");
                return 0;

            case 0x05:  /* MAC_TX_TIMEOUT */
                LOG_WRN("TX timed out");
                return -ETIMEDOUT;

            case 0x06:  /* MAC_TXACK_TIMEOUT */
                LOG_WRN("TX ACK timed out");
                return -ETIMEDOUT;

            case 0x07:  /* MAC_RX_ERROR */
            case 0x09:  /* MAC_ERROR */
                LOG_ERR("TX error: 0x%02X", mac_status);
                return -EIO;

            case 0x01:  /* MAC_OK - still processing */
                k_sleep(K_MSEC(100));
                break;

            default:
                k_sleep(K_MSEC(100));
                break;
        }
    }

    LOG_ERR("TX polling timeout");
    return -ETIMEDOUT;
}
```

### 6.3 Complete TX with Status Polling

```c
/**
 * @brief Send data and wait for completion
 * @param payload Data to transmit
 * @param payload_size Size in bytes
 * @param timeout_ms Max time to wait for TX completion
 * @return 0 on success, negative on error
 */
int app_tx_data_blocking(const uint8_t *payload, uint16_t payload_size,
                          uint32_t timeout_ms)
{
    int ret;

    /* Send TX command sequence */
    ret = app_tx_data(payload, payload_size);
    if (ret != 0) {
        return ret;
    }

    /* Wait for completion */
    return app_wait_tx_complete(timeout_ms);
}
```

---

## 7. Configuration Commands (2-Phase Protocol)

### 7.1 Generic 2-Phase Write

```c
/**
 * @brief Write configuration using 2-phase protocol
 * @param req_cmd Request command code
 * @param data_cmd Data command code
 * @param data Configuration data
 * @param data_len Data length
 * @return 0 on success, negative on error
 */
int app_write_config(uint8_t req_cmd, uint8_t data_cmd,
                      const uint8_t *data, uint8_t data_len)
{
    int ret;
    uint8_t resp[8];
    uint16_t resp_len;

    /* Phase 1: Request */
    ret = app_send_command(req_cmd, NULL, 0, resp, &resp_len, 15);
    if (ret != 0) {
        LOG_ERR("Config REQ failed: 0x%02X", req_cmd);
        return ret;
    }

    /* Phase 2: Data */
    ret = app_send_command(data_cmd, data, data_len, resp, &resp_len, 15);
    if (ret != 0) {
        LOG_ERR("Config DATA failed: 0x%02X", data_cmd);
        return ret;
    }

    return 0;
}

/* Examples */
int app_write_device_id(uint32_t device_id)
{
    uint8_t data[4];
    memcpy(data, &device_id, 4);  /* Little-endian */
    return app_write_config(0x20, 0x21, data, 4);
}

int app_write_tcxo_warmup(uint32_t warmup_ms)
{
    if (warmup_ms > 30000) {
        return -EINVAL;
    }
    uint8_t data[4];
    memcpy(data, &warmup_ms, 4);  /* Little-endian */
    return app_write_config(0x29, 0x2A, data, 4);
}

int app_write_lpm(uint8_t lpm_mode)
{
    return app_write_config(0x12, 0x13, &lpm_mode, 1);
}
```

---

## 8. Complete Application Example

```c
/**
 * @brief Initialize and configure Kineis device
 */
int app_kineis_init(void)
{
    int ret;
    uint8_t resp[64];
    uint16_t resp_len;

    /* 1. PING - Verify device is responding */
    ret = app_send_command(0x02, NULL, 0, resp, &resp_len, 15);
    if (ret != 0 || resp[0] != 0x01) {
        LOG_ERR("PING failed");
        return -EIO;
    }
    LOG_INF("Device PING OK");

    /* 2. Read firmware version */
    ret = app_send_command(0x06, NULL, 0, resp, &resp_len, 15);
    if (ret == 0) {
        resp[resp_len] = '\0';
        LOG_INF("Firmware: %s", resp);
    }

    /* 3. Read device ID */
    ret = app_send_command(0x08, NULL, 0, resp, &resp_len, 15);
    if (ret == 0 && resp_len >= 4) {
        uint32_t dev_id;
        memcpy(&dev_id, resp, 4);
        LOG_INF("Device ID: %u", dev_id);
    }

    /* 4. Read device address */
    ret = app_send_command(0x07, NULL, 0, resp, &resp_len, 15);
    if (ret == 0 && resp_len >= 4) {
        LOG_INF("Address: %02X:%02X:%02X:%02X",
                resp[0], resp[1], resp[2], resp[3]);
    }

    /* 5. Check MAC status */
    ret = app_send_command(0x03, NULL, 0, resp, &resp_len, 15);
    if (ret == 0) {
        LOG_INF("MAC Status: 0x%02X", resp[0]);
    }

    return 0;
}

/**
 * @brief Send message via Kineis satellite
 */
int app_send_message(const char *message)
{
    int ret;
    uint16_t len = strlen(message);

    if (len > 256) {
        LOG_ERR("Message too long (%u > 256)", len);
        return -EINVAL;
    }

    LOG_INF("Sending %u bytes...", len);

    /* Send with 60 second timeout for TX completion */
    ret = app_tx_data_blocking((const uint8_t *)message, len, 60000);

    if (ret == 0) {
        LOG_INF("Message sent successfully!");
    } else {
        LOG_ERR("Message send failed: %d", ret);
    }

    return ret;
}

/**
 * @brief Enter DFU bootloader mode
 */
int app_enter_dfu(void)
{
    int ret;
    uint8_t resp[8];
    uint16_t resp_len;

    LOG_WRN("Entering DFU mode...");

    /* Send DFU_ENTER (0x3F) - device will reset */
    ret = app_send_command(0x3F, NULL, 0, resp, &resp_len, 100);

    /* Note: Device resets immediately, may not get response */
    LOG_INF("DFU command sent, device resetting...");

    /* Wait for device to reboot into bootloader */
    k_sleep(K_MSEC(500));

    return 0;
}
```

---

## 9. Timing Recommendations

### 9.1 Command Timing Table

| Operation | Min Delay After Command |
|-----------|-------------------------|
| PING, READ_* | 15ms |
| MAC_STATUS | 15ms |
| WRITE_*_REQ | 15ms |
| WRITE_* (data phase) | 15ms |
| TX_REQ, TX_SIZE | 15ms |
| TX_DATA | 100ms (queues RF TX) |
| DFU_ENTER | 100ms (triggers reset) |

### 9.2 TX Polling Intervals

| Operation | Recommended Interval |
|-----------|---------------------|
| MAC_STATUS polling during TX | 100-500ms |
| TX timeout (no satellite pass) | 60-120 seconds |
| Retry after error | 1000ms |

---

## 10. Error Handling and Retry

```c
#define APP_MAX_RETRIES 3

int app_send_with_retry(uint8_t cmd, const uint8_t *data, uint8_t len,
                         uint8_t *resp, uint16_t *resp_len, uint32_t delay_ms)
{
    for (int i = 0; i < APP_MAX_RETRIES; i++) {
        int ret = app_send_command(cmd, data, len, resp, resp_len, delay_ms);

        if (ret == 0) {
            return 0;  /* Success */
        }

        if (ret == -EILSEQ) {  /* CRC error */
            LOG_WRN("CRC error, retrying (%d/%d)", i + 1, APP_MAX_RETRIES);
            k_sleep(K_MSEC(50));
            continue;
        }

        /* Non-recoverable error */
        return ret;
    }

    return -ETIMEDOUT;
}
```

---

## 11. Low Power Mode Configuration

```c
/* Low Power Mode bitmap values */
#define LPM_NONE     0x00  /* No low power */
#define LPM_SLEEP    0x01  /* Sleep mode */
#define LPM_STOP     0x02  /* Stop mode */
#define LPM_STANDBY  0x04  /* Standby mode */
#define LPM_SHUTDOWN 0x08  /* Shutdown mode */

int app_set_low_power_mode(uint8_t mode)
{
    /* Validate mode */
    const uint8_t allowed = LPM_NONE | LPM_SLEEP | LPM_STOP |
                            LPM_STANDBY | LPM_SHUTDOWN;
    if ((mode & ~allowed) != 0) {
        return -EINVAL;
    }

    return app_write_config(0x12, 0x13, &mode, 1);
}
```

---

## 12. Summary Checklist

- [ ] Use A+ protocol with CRC-8 for reliability
- [ ] Implement pipelined transactions (response in next transaction)
- [ ] Send NOP (0x00) to retrieve response to previous command
- [ ] Use 3-phase protocol for TX: TX_REQ → TX_SIZE → TX_DATA
- [ ] Poll MAC_STATUS (0x03) for async TX completion
- [ ] Implement retry logic for CRC errors
- [ ] Handle MAC_STATUS codes appropriately
- [ ] Wait 15ms minimum between transactions
- [ ] Wait 100ms+ after TX_DATA before polling

---

## 13. Quick Reference Card

### Frame Format
```
Request:  [0xAA][SEQ][CMD][LEN][DATA...][CRC8]
Response: [0x55][SEQ][STATUS][LEN][DATA...][CRC8]
```

### Essential Commands
```
0x00 NOP          - Get previous response
0x02 PING         - Check device alive
0x03 MAC_STATUS   - Check TX status
0x14 TX_REQ       - Start TX sequence
0x15 TX_SIZE      - Send payload size
0x16 TX_DATA      - Send payload
0x3F DFU_ENTER    - Enter bootloader
```

### MAC Status Quick Ref
```
0x01 OK           - Ready
0x02 TX_DONE      - Success!
0x05 TX_TIMEOUT   - Failed
0x09 ERROR        - Failed
```

