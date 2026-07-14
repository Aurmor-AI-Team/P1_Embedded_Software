#pragma once

#include "esp_err.h"
#include "lsm6dsv.h"   // lsm6_sample_t

#ifdef __cplusplus
extern "C" {
#endif

// Bring up the NimBLE stack, register the IMU GATT service, and start
// advertising as "XIAO-IMU". Call once from app_main after the IMU is up.
esp_err_t ble_imu_init(void);

// True once a client is connected AND has subscribed to notifications
// (wrote the CCCD). Cheap to poll.
bool ble_imu_ready(void);

// Push one IMU sample to the connected app as a notification. No-op if no
// client is subscribed. Safe to call from a normal FreeRTOS task (the IMU
// print task). Returns ESP_OK on success or if there's simply no subscriber.
esp_err_t ble_imu_notify(const lsm6_sample_t *s);

#ifdef __cplusplus
}
#endif
