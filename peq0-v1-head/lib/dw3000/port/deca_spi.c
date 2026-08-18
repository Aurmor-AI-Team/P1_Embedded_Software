/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   ESP-IDF SPI shim for DW3000 driver.
 */

#include "deca_spi.h"
#include "port.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "deca_spi";

extern spi_device_handle_t g_dw_spi;

#define SPI_BUF_LEN 256
static uint8_t s_tx_buf[SPI_BUF_LEN];
static uint8_t s_rx_buf[SPI_BUF_LEN];

int32_t writetospi(uint16_t headerLength, const uint8_t *headerBuffer,
                   uint16_t bodyLength,   const uint8_t *bodyBuffer)
{
    if (g_dw_spi == NULL) return -1;

    size_t total = (size_t)headerLength + (size_t)bodyLength;
    if (total > SPI_BUF_LEN) {
        ESP_LOGE(TAG, "writetospi too big: %u", (unsigned)total);
        return -1;
    }

    memcpy(s_tx_buf, headerBuffer, headerLength);
    if (bodyLength > 0) {
        memcpy(s_tx_buf + headerLength, bodyBuffer, bodyLength);
    }

    spi_transaction_t t = {
        .length    = total * 8,
        .tx_buffer = s_tx_buf,
        .rx_buffer = NULL,
    };
    return (spi_device_polling_transmit(g_dw_spi, &t) == ESP_OK) ? 0 : -1;
}

int32_t readfromspi(uint16_t headerLength, uint8_t *headerBuffer,
                    uint16_t readLength,   uint8_t *readBuffer)
{
    if (g_dw_spi == NULL) return -1;

    size_t total = (size_t)headerLength + (size_t)readLength;
    if (total > SPI_BUF_LEN) {
        ESP_LOGE(TAG, "readfromspi too big: %u", (unsigned)total);
        return -1;
    }

    memcpy(s_tx_buf, headerBuffer, headerLength);
    memset(s_tx_buf + headerLength, 0x00, readLength);

    spi_transaction_t t = {
        .length    = total * 8,
        .rxlength  = total * 8,
        .tx_buffer = s_tx_buf,
        .rx_buffer = s_rx_buf,
    };
    if (spi_device_polling_transmit(g_dw_spi, &t) != ESP_OK) return -1;

    memcpy(readBuffer, s_rx_buf + headerLength, readLength);

    /* Debug: log first few reads so we can confirm SPI is working. */
    static int dbg_count = 0;
    if (dbg_count < 3) {
        ESP_LOGI(TAG, "readfromspi hdrLen=%u rdLen=%u  hdr[0]=%02X  rx=%02X %02X %02X %02X",
                 headerLength, readLength, headerBuffer[0],
                 readBuffer[0],
                 readLength > 1 ? readBuffer[1] : 0,
                 readLength > 2 ? readBuffer[2] : 0,
                 readLength > 3 ? readBuffer[3] : 0);
        dbg_count++;
    }

    return 0;
}

int32_t writetospiwithcrc(uint16_t headerLength, const uint8_t *headerBuffer,
                          uint16_t bodyLength,   const uint8_t *bodyBuffer,
                          uint8_t crc8)
{
    (void)headerLength; (void)headerBuffer;
    (void)bodyLength;   (void)bodyBuffer;
    (void)crc8;
    return 0;
}