#include "dwm3000.h"
#include "port.h"
#include "deca_probe_interface.h"
#include "deca_device_api.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "dwm3000";

/* Provided by port.c */
extern spi_device_handle_t g_dw_spi;

/* The pin-config args to dwm3000_init are kept for backward compatibility
 * with main.c, but the actual pins live in port.h. We log a warning if
 * they don't match. */
static void check_pins(int mosi, int miso, int sclk, int cs, int rst)
{
    extern int DW3000_MOSI_PIN_CHK __attribute__((weak));
    if (mosi != DW3000_MOSI_PIN || miso != DW3000_MISO_PIN ||
        sclk != DW3000_SCLK_PIN || cs   != DW3000_CS_PIN   ||
        rst  != DW3000_RST_PIN) {
        ESP_LOGW(TAG, "pin mismatch with port.h — using port.h values");
    }
}

void dwm3000_hard_reset(void)
{
    reset_DWIC();
}

void dwm3000_reset_pin_only(int rst)
{
    (void)rst;
    reset_DWIC();
}

esp_err_t dwm3000_reconfigure_recover(const void *uwb_config,
                                      const void *tx_config,
                                      uint16_t tx_ant_dly, uint16_t rx_ant_dly)
{
    /* Put the transceiver into a known-idle state first. On this driver
     * dwt_forcetrxoff() only issues CMD_TXRXOFF if not already idle — it does
     * NOT reset the receiver, so the real work is the dwt_configure re-cal. */
    dwt_forcetrxoff();

    /* Force the next dwt_configure() to re-measure die temperature and
     * recalibrate PLL/RX for it. This is the fix for "works then degrades"
     * thermal drift: PGF/RX cal must re-run after ~20 °C of change. */
    dwt_setpllcaltemperature(TEMP_INIT);   /* -127 -> read on-chip sensor */

    if (dwt_configure((dwt_config_t *)uwb_config) != DWT_SUCCESS) {
        ESP_LOGW(TAG, "recover: dwt_configure (PGF/RX cal) failed");
        return ESP_FAIL;
    }

    /* Re-apply everything dwt_configure does not: TX RF, antenna delays,
     * LNA/PA. These are lost/irrelevant across a reconfigure but must be
     * restored for ranging. */
    dwt_configuretxrf((dwt_txconfig_t *)tx_config);
    dwt_settxantennadelay(tx_ant_dly);
    dwt_setrxantennadelay(rx_ant_dly);
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    ESP_LOGW(TAG, "recover: reconfigured + recalibrated (PLL cal T=%d C)",
             (int)dwt_getpllcaltemperature());
    return ESP_OK;
}

esp_err_t dwm3000_hard_recover(void)
{
    dwt_forcetrxoff();
    reset_DWIC();
    vTaskDelay(pdMS_TO_TICKS(50));

    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "hard_recover: dwt_probe failed");
        return ESP_FAIL;
    }
    int waited_ms = 0;
    while (!dwt_checkidlerc()) {
        vTaskDelay(pdMS_TO_TICKS(2));
        if ((waited_ms += 2) > 200) {
            ESP_LOGE(TAG, "hard_recover: IDLE_RC timeout");
            return ESP_ERR_TIMEOUT;
        }
    }
    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "hard_recover: dwt_initialise failed");
        return ESP_FAIL;
    }
    port_set_dw_ic_spi_fastrate();
    ESP_LOGW(TAG, "hard_recover: re-initialised (caller must reconfigure)");
    return ESP_OK;   /* caller must follow with dwm3000_reconfigure_recover() */
}

float dwm3000_read_temp_c(void)
{
    uint16_t raw = dwt_readtempvbat();          /* [15:8]=temp, [7:0]=vbat */
    return dwt_convertrawtemperature((uint8_t)(raw >> 8));
}


esp_err_t dwm3000_init(int mosi, int miso, int sclk, int cs, int rst)
{
    check_pins(mosi, miso, sclk, cs, rst);

    /* Idempotent setup */
    dwm3000_deinit();

    esp_err_t err = port_init_dw3000();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "port_init_dw3000 failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(200));  // generous boot delay
    gpio_set_pull_mode((gpio_num_t)DW3000_RST_PIN, GPIO_PULLUP_ONLY);  // belt + suspenders
    vTaskDelay(pdMS_TO_TICKS(10));

    uint32_t check_devid = 0;
    if (dwm3000_read_devid(&check_devid) == ESP_OK) {
        ESP_LOGI(TAG, "DEV_ID after extended wait: 0x%08lX", (unsigned long)check_devid);
    } else {
        ESP_LOGE(TAG, "Phase-1 DEV_ID read failed");
    }
    /* Hard reset before any driver activity. */
    reset_DWIC();
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Hand control to Qorvo's driver: probe → wait IDLE_RC → initialise. */
    if (dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "dwt_probe failed");
        return ESP_FAIL;
    }

    /* Wait for IDLE_RC. dwt_checkidlerc returns non-zero when ready. */
    int waited_ms = 0;
    while (!dwt_checkidlerc()) {
        vTaskDelay(pdMS_TO_TICKS(2));
        waited_ms += 2;
        if (waited_ms > 200) {
            ESP_LOGE(TAG, "IDLE_RC timeout");
            return ESP_ERR_TIMEOUT;
        }
    }
    ESP_LOGI(TAG, "IDLE_RC reached after %d ms", waited_ms);

    /* Driver init: reads OTP, kicks LDO/BIAS, applies XTAL_TRIM, etc. */
    if (dwt_initialise(DWT_DW_INIT) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "dwt_initialise failed");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "dwt_initialise OK");

    /* Driver's setfastrate is supposed to fire automatically but doesn't on
    * this version. Call it manually before any time-critical operation. */
    port_set_dw_ic_spi_fastrate();
    ESP_LOGI(TAG, "switched to fast SPI rate");
    
    return ESP_OK;
}

esp_err_t dwm3000_deinit(void)
{
    port_deinit_dw3000();
    return ESP_OK;
}

esp_err_t dwm3000_read_devid(uint32_t *out_devid)
{
    if (out_devid == NULL) return ESP_ERR_INVALID_ARG;
    if (g_dw_spi == NULL)  return ESP_ERR_INVALID_STATE;

    uint8_t tx[5] = {0}, rx[5] = {0};
    spi_transaction_t t = { .length = 5 * 8, .tx_buffer = tx, .rx_buffer = rx };
    esp_err_t err = spi_device_polling_transmit(g_dw_spi, &t);
    if (err != ESP_OK) return err;

    *out_devid = ((uint32_t)rx[1])
               | ((uint32_t)rx[2] << 8)
               | ((uint32_t)rx[3] << 16)
               | ((uint32_t)rx[4] << 24);
    return ESP_OK;
}

esp_err_t dwm3000_wait_ready(int timeout_ms)
{
    int elapsed = 0;
    uint32_t devid = 0;
    while (elapsed < timeout_ms) {
        if (dwm3000_read_devid(&devid) == ESP_OK &&
            (devid >> 16) == DW3000_DEV_ID_EXPECTED_HI) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
        elapsed += 5;
    }
    ESP_LOGE(TAG, "Not ready after %d ms (last DEV_ID=0x%08lX)",
             timeout_ms, (unsigned long)devid);
    return ESP_ERR_TIMEOUT;
}