#include "lsm6dsv.h"
#include <string.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lsm6dsv80x_reg.h"   // ST's driver
#include <string.h>   // for memcpy in platform shim

static const char *TAG = "lsm6";

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev = NULL;
static uint8_t s_addr = 0;

static float s_accel_sens_g_per_lsb = 0.000061f;
static float s_gyro_sens_dps_per_lsb = 0.070f;
static float s_highg_sens_g_per_lsb = 0.0039f;  // ±80g default

// Platform shim: ST driver calls these to talk to the chip via I2C.
//
// 'handle' is whatever we put in dev_ctx.handle — we'll use the i2c device handle.
// Return 0 on success, non-zero on error (ST's convention, not ESP_OK).

static int32_t st_platform_write(void *handle, uint8_t reg,
                                 const uint8_t *bufp, uint16_t len)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)handle;

    // ST driver assumes register address is one byte, followed by data
    // We need a single contiguous buffer: [reg, data0, data1, ...]
    uint8_t tx_buf[len + 1];
    tx_buf[0] = reg;
    memcpy(&tx_buf[1], bufp, len);

    esp_err_t err = i2c_master_transmit(dev, tx_buf, len + 1, 100);
    return (err == ESP_OK) ? 0 : -1;
}

static int32_t st_platform_read(void *handle, uint8_t reg,
                                uint8_t *bufp, uint16_t len)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)handle;
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, bufp, len, 100);
    return (err == ESP_OK) ? 0 : -1;
}

static void st_platform_delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

// Driver context — initialized in lsm6_read_who_am_i as soon as we have
// a valid I2C device handle. Tracked separately so lsm6_read_sample can
// refuse to run if configure_default hasn't completed.
static stmdev_ctx_t s_st_ctx;
static bool s_st_ctx_ready = false;
static bool s_configured = false;

static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, 100);
}

static esp_err_t reg_read(uint8_t reg, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, buf, n, 100);
}

esp_err_t lsm6_init(int sda, int scl)
{
    lsm6_deinit();

    // Always force I2C mode first — the chip may be locked in I3C from a
    // previous firmware or power glitch. Power cycles are the only thing
    // that fully resets I3C state, so we have to bit-bang the exit sequence
    // every time we initialize.
    lsm6_force_i2c_mode(sda, scl);

    i2c_master_bus_config_t cfg = {
        .i2c_port = -1,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        s_bus = NULL;
    }
    return err;
}

esp_err_t lsm6_deinit(void)
{
    if (s_dev != NULL) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    if (s_bus != NULL) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
    s_addr = 0;
    s_st_ctx_ready = false;
    s_configured = false;
    return ESP_OK;
}

esp_err_t lsm6_scan(int *out_count)
{
    if (s_bus == NULL) return ESP_ERR_INVALID_STATE;

    int found = 0;
    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        if (i2c_master_probe(s_bus, addr, 50) == ESP_OK) {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
            found++;
        }
    }
    if (out_count) *out_count = found;
    return ESP_OK;
}

esp_err_t lsm6_read_who_am_i(uint8_t *out_addr, uint8_t *out_whoami)
{
    if (s_bus == NULL) return ESP_ERR_INVALID_STATE;

    // Drop any previous device handle so we can re-attach cleanly
    if (s_dev != NULL) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }

    const uint8_t addrs[] = {0x6A, 0x6B};
    const uint8_t reg = LSM6_REG_WHO_AM_I;

    for (int i = 0; i < 2; i++) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = 400000,
        };
        i2c_master_dev_handle_t dev;
        if (i2c_master_bus_add_device(s_bus, &dev_cfg, &dev) != ESP_OK) continue;

        uint8_t whoami = 0;
        esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, &whoami, 1, 100);

        if (err == ESP_OK) {
            s_dev = dev;
            s_addr = addrs[i];
            *out_addr = addrs[i];
            *out_whoami = whoami;

            // Wire up the ST driver context immediately — any code path
            // that has a working I2C device should be able to use it.
            s_st_ctx.write_reg = st_platform_write;
            s_st_ctx.read_reg  = st_platform_read;
            s_st_ctx.mdelay    = st_platform_delay;
            s_st_ctx.handle    = s_dev;
            s_st_ctx_ready = true;
            s_configured = false;  // configure_default must run before sampling

            return ESP_OK;
        } else {
            i2c_master_bus_rm_device(dev);
        }
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t lsm6_configure_default(void)
{
    if (s_dev == NULL || !s_st_ctx_ready) return ESP_ERR_INVALID_STATE;

    // Verify chip ID via ST driver (sanity check that shim works)
    uint8_t whoami;
    if (lsm6dsv80x_device_id_get(&s_st_ctx, &whoami) != 0) {
        ESP_LOGE(TAG, "ST driver: device_id_get failed");
        return ESP_FAIL;
    }
    if (whoami != LSM6DSV80X_ID) {
        ESP_LOGE(TAG, "Wrong device ID: 0x%02X (expected 0x%02X)",
                 whoami, LSM6DSV80X_ID);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "ST driver verified, WHO_AM_I=0x%02X", whoami);

    // Software power-on reset — single call, blocks internally
    if (lsm6dsv80x_sw_por(&s_st_ctx) != 0) {
        ESP_LOGE(TAG, "ST: sw_por failed");
        return ESP_FAIL;
    }
    vTaskDelay(pdMS_TO_TICKS(50));  // Settling time after reset
    ESP_LOGI(TAG, "ST: reset complete");

    // BDU + auto-increment (already standard practice)
    lsm6dsv80x_block_data_update_set(&s_st_ctx, PROPERTY_ENABLE);
    lsm6dsv80x_auto_increment_set(&s_st_ctx, PROPERTY_ENABLE);

    // ----- Low-g accelerometer: 960 Hz, ±16g, high-performance -----
    lsm6dsv80x_xl_data_rate_set(&s_st_ctx, LSM6DSV80X_ODR_AT_960Hz);
    lsm6dsv80x_xl_full_scale_set(&s_st_ctx, LSM6DSV80X_16g);
    lsm6dsv80x_xl_mode_set(&s_st_ctx, LSM6DSV80X_XL_HIGH_PERFORMANCE_MD);
    s_accel_sens_g_per_lsb = 0.000488f;  // ±16g: 0.488 mg/LSB
    ESP_LOGI(TAG, "ST: low-g accel 960Hz ±16g HP");

    // ----- High-g accelerometer: 960 Hz, ±80g -----
    lsm6dsv80x_hg_xl_data_rate_set(&s_st_ctx, LSM6DSV80X_HG_XL_ODR_AT_960Hz, 1);
    lsm6dsv80x_hg_xl_full_scale_set(&s_st_ctx, LSM6DSV80X_80g);
    s_highg_sens_g_per_lsb = 0.0039f;  // ±80g: 3.9 mg/LSB (verify against datasheet)
    ESP_LOGI(TAG, "ST: high-g accel 960Hz ±80g");

    // ----- Gyroscope: 960 Hz, ±4000 dps, high-performance -----
    lsm6dsv80x_gy_data_rate_set(&s_st_ctx, LSM6DSV80X_ODR_AT_960Hz);
    lsm6dsv80x_gy_full_scale_set(&s_st_ctx, LSM6DSV80X_4000dps);
    lsm6dsv80x_gy_mode_set(&s_st_ctx, LSM6DSV80X_GY_HIGH_PERFORMANCE_MD);
    s_gyro_sens_dps_per_lsb = 0.140f;  // ±4000 dps: 140 mdps/LSB
    ESP_LOGI(TAG, "ST: gyro 960Hz ±4000dps HP");

    vTaskDelay(pdMS_TO_TICKS(100));
    s_configured = true;
    return ESP_OK;
}

esp_err_t lsm6_read_sample(lsm6_sample_t *out)
{
    if (out == NULL) return ESP_ERR_INVALID_ARG;
    if (s_dev == NULL || !s_st_ctx_ready || !s_configured) {
        return ESP_ERR_INVALID_STATE;
    }

    int16_t raw_a[3], raw_h[3], raw_g[3];

    // Low-g accel
    if (lsm6dsv80x_acceleration_raw_get(&s_st_ctx, raw_a) != 0) return ESP_FAIL;
    out->ax_g = raw_a[0] * s_accel_sens_g_per_lsb;
    out->ay_g = raw_a[1] * s_accel_sens_g_per_lsb;
    out->az_g = raw_a[2] * s_accel_sens_g_per_lsb;

    // High-g accel
    if (lsm6dsv80x_hg_acceleration_raw_get(&s_st_ctx, raw_h) != 0) return ESP_FAIL;
    out->hx_g = raw_h[0] * s_highg_sens_g_per_lsb;
    out->hy_g = raw_h[1] * s_highg_sens_g_per_lsb;
    out->hz_g = raw_h[2] * s_highg_sens_g_per_lsb;

    // Gyro
    if (lsm6dsv80x_angular_rate_raw_get(&s_st_ctx, raw_g) != 0) return ESP_FAIL;
    out->gx_dps = raw_g[0] * s_gyro_sens_dps_per_lsb;
    out->gy_dps = raw_g[1] * s_gyro_sens_dps_per_lsb;
    out->gz_dps = raw_g[2] * s_gyro_sens_dps_per_lsb;

    int16_t temp_raw;
    if (lsm6dsv80x_temperature_raw_get(&s_st_ctx, &temp_raw) != 0) return ESP_FAIL;
    out->temp_c = temp_raw / 256.0f + 25.0f;

    return ESP_OK;
}

esp_err_t lsm6_software_reset(void) {
    // Try at both possible addresses — we don't know which yet
    const uint8_t addrs[] = {0x6A, 0x6B};
    if (s_bus == NULL) return ESP_ERR_INVALID_STATE;

    for (int i = 0; i < 2; i++) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = 100000,  // slower for reset reliability
        };
        i2c_master_dev_handle_t dev;
        if (i2c_master_bus_add_device(s_bus, &dev_cfg, &dev) != ESP_OK) continue;

        // Write SW_RESET bit to CTRL3
        uint8_t buf[2] = {LSM6_REG_CTRL3, 0x01};
        i2c_master_transmit(dev, buf, 2, 100);  // ignore errors
        i2c_master_bus_rm_device(dev);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
    return ESP_OK;
}

// Force the chip out of I3C mode by bit-banging an exit sequence.
// Must be called BEFORE any I2C bus init.
void lsm6_force_i2c_mode(int sda, int scl)
{
    // Configure SDA and SCL as plain GPIOs (open-drain output)
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << sda) | (1ULL << scl),
        .mode = GPIO_MODE_OUTPUT_OD,        // open-drain
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);

    // Idle: both high
    gpio_set_level((gpio_num_t)sda, 1);
    gpio_set_level((gpio_num_t)scl, 1);
    vTaskDelay(pdMS_TO_TICKS(2));

    // Pull SDA low while clocking SCL — this is the I3C exit pattern
    gpio_set_level((gpio_num_t)sda, 0);
    for (int i = 0; i < 16; i++) {
        gpio_set_level((gpio_num_t)scl, 0);
        esp_rom_delay_us(50);
        gpio_set_level((gpio_num_t)scl, 1);
        esp_rom_delay_us(50);
    }

    // Issue STOP: SDA goes high while SCL is high
    gpio_set_level((gpio_num_t)scl, 1);
    esp_rom_delay_us(10);
    gpio_set_level((gpio_num_t)sda, 1);
    esp_rom_delay_us(10);

    // Reset pins to inputs so the I2C peripheral can take over
    gpio_set_direction((gpio_num_t)sda, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)scl, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(2));
}