/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   ESP-IDF port implementation for DW3000 driver.
 */

#include "port.h"
#include "deca_spi.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/event_groups.h"

static const char *TAG = "port";

static TaskHandle_t s_deca_task = NULL;

/* GPIO hard-ISR: no SPI, no logging, just kick the deca task. */
static void IRAM_ATTR dw_irq_isr(void *arg)
{
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(s_deca_task, &hpw);
    if (hpw) portYIELD_FROM_ISR();
}

/* Runs dwt_isr() in task context. Drains while the line is still high so a
 * second event that arrives mid-service isn't lost (DW3000 IRQ is level-ish:
 * it stays asserted until every pending SYS_STATUS source is cleared). */
static void deca_irq_task(void *arg)
{
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        do {
            process_deca_irq();                 /* -> dwt_isr() */
        } while (gpio_get_level((gpio_num_t)DW3000_IRQ_PIN));
    }
}

esp_err_t port_enable_dw3000_irq(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << DW3000_IRQ_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,   /* IRQ idles LOW */
        .intr_type    = GPIO_INTR_POSEDGE,
    };
    gpio_config(&io);

    if (s_deca_task == NULL) {
        /* Priority MUST exceed the ranging/uwb task (you have it at 5) so
         * dwt_isr runs and sets the event bits before the ranging task that
         * is waiting on them gets scheduled. */
        xTaskCreate(deca_irq_task, "deca_irq", 4096, NULL, 10, &s_deca_task);
    }
    static bool isr_svc = false;
    if (!isr_svc) { gpio_install_isr_service(0); isr_svc = true; }
    return gpio_isr_handler_add((gpio_num_t)DW3000_IRQ_PIN, dw_irq_isr, NULL);
}

/* ===== replace the existing stubs ===== */
void port_set_dwic_isr(port_dwic_isr_t isr) { s_port_dwic_isr = isr; }
void port_DisableEXT_IRQ(void) { gpio_intr_disable((gpio_num_t)DW3000_IRQ_PIN); }
void port_EnableEXT_IRQ(void)  { gpio_intr_enable((gpio_num_t)DW3000_IRQ_PIN); }
void process_deca_irq(void)    { if (s_port_dwic_isr) s_port_dwic_isr(); }

/* Single SPI device handle. We change clock rate by removing and re-adding
 * the device — this is the pattern Qorvo's reference Nordic port uses, and
 * it avoids the "two devices, same CS pin" trap that confuses ESP-IDF's
 * driver and causes CS to stop being asserted. */
spi_device_handle_t g_dw_spi = NULL;

static bool s_bus_initialized = false;
static bool s_current_rate_fast = false;
static port_dwic_isr_t s_port_dwic_isr = NULL;

static esp_err_t add_device_at_rate(int hz)
{
    spi_device_interface_config_t dev = {
        .clock_speed_hz = hz,
        .mode           = 0,
        .spics_io_num   = DW3000_CS_PIN,
        .queue_size     = 1,
    };
    return spi_bus_add_device(SPI2_HOST, &dev, &g_dw_spi);
}

esp_err_t port_init_dw3000(void)
{
    if (s_bus_initialized) return ESP_OK;

    /* Configure RST as input with pull-up by default; reset_DWIC drives
     * it low temporarily. The pull-up is internal, redundant with any
     * board-level pull-up but harmless. */
    gpio_config_t rst_cfg = {
        .pin_bit_mask = (1ULL << DW3000_RST_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_cfg);

    /* SPI bus init. */
    spi_bus_config_t bus = {
        .mosi_io_num   = DW3000_MOSI_PIN,
        .miso_io_num   = DW3000_MISO_PIN,
        .sclk_io_num   = DW3000_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 256,
    };
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Start at slow rate (driver will switch to fast after init). */
    err = add_device_at_rate(DW3000_SPI_FREQ_SLOW);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add_device(slow) failed: %s", esp_err_to_name(err));
        spi_bus_free(SPI2_HOST);
        return err;
    }
    s_current_rate_fast = false;
    s_bus_initialized = true;
    return ESP_OK;
}

void port_deinit_dw3000(void)
{
    if (g_dw_spi) {
        spi_bus_remove_device(g_dw_spi);
        g_dw_spi = NULL;
    }
    if (s_bus_initialized) {
        spi_bus_free(SPI2_HOST);
        s_bus_initialized = false;
    }
}

/* Called by the driver via the spi_fct struct. Switches between slow (2MHz,
 * used during init/OTP/PLL lock) and fast (16MHz, normal operation). We
 * remove the existing device and re-add at the new rate. */
void port_set_dw_ic_spi_slowrate(void)
{
    if (!s_bus_initialized) return;
    if (!s_current_rate_fast) return;   /* already slow */
    spi_bus_remove_device(g_dw_spi);
    g_dw_spi = NULL;
    add_device_at_rate(DW3000_SPI_FREQ_SLOW);
    s_current_rate_fast = false;
}

void port_set_dw_ic_spi_fastrate(void)
{
    ESP_LOGI("port", "setfastrate called");
    if (!s_bus_initialized) return;
    if (s_current_rate_fast) return;
    spi_bus_remove_device(g_dw_spi);
    g_dw_spi = NULL;
    add_device_at_rate(DW3000_SPI_FREQ_FAST);
    s_current_rate_fast = true;
}

void reset_DWIC(void)
{
    gpio_set_direction((gpio_num_t)DW3000_RST_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)DW3000_RST_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_direction((gpio_num_t)DW3000_RST_PIN, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void Sleep(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

/* Stubs — implemented properly in Phase 4 when IRQ-driven RX is added. */
void wakeup_device_with_io(void) { /* not in deepsleep in Phase 3 */ }
void port_set_dwic_isr(port_dwic_isr_t isr) { s_port_dwic_isr = isr; }
void port_DisableEXT_IRQ(void)        { }
void port_EnableEXT_IRQ(void)         { }
uint32_t port_GetEXT_IRQStatus(void)  { return 0; }
uint32_t port_CheckEXT_IRQ(void)      { return 0; }
void process_deca_irq(void) { if (s_port_dwic_isr) s_port_dwic_isr(); }