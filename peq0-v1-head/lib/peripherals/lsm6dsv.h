#pragma once
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Existing register defines stay the same...
#define LSM6_REG_FUNC_CFG_ACCESS  0x01
#define LSM6_REG_IF_CFG           0x03
#define LSM6_REG_WHO_AM_I         0x0F
#define LSM6_REG_CTRL1            0x10
#define LSM6_REG_CTRL2            0x11
#define LSM6_REG_CTRL3            0x12
#define LSM6_REG_CTRL6            0x15  // Gyro full scale
#define LSM6_REG_CTRL8            0x17  // Accel full scale (low-g)
#define LSM6_REG_OUT_TEMP_L       0x20
#define LSM6_REG_OUTX_L_G         0x22
#define LSM6_REG_OUTX_L_A         0x28
#define LSM6_REG_UI_OUTX_L_A_OIS_DUALC 0x34  // High-g accel output (UI channel)
// Note: high-g register address may differ — verify with datasheet section 9
// Common LSM6DSV80X high-g output register: 0x34-0x39 (low/high X, Y, Z)

typedef struct {
    float ax_g, ay_g, az_g;          // Low-g accelerometer (g)
    float hx_g, hy_g, hz_g;          // High-g accelerometer (g)
    float gx_dps, gy_dps, gz_dps;    // Gyroscope (degrees/sec)
    float temp_c;                    // Temperature (°C)
} lsm6_sample_t;

esp_err_t lsm6_init(int sda, int scl);
esp_err_t lsm6_deinit(void);
esp_err_t lsm6_scan(int *out_count);
esp_err_t lsm6_read_who_am_i(uint8_t *out_addr, uint8_t *out_whoami);
esp_err_t lsm6_software_reset(void);
esp_err_t lsm6_configure_default(void);
esp_err_t lsm6_read_sample(lsm6_sample_t *out);
void      lsm6_force_i2c_mode(int sda, int scl);

#ifdef __cplusplus
}
#endif