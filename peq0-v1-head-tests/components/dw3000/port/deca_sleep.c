/*! ----------------------------------------------------------------------------
 * @file    deca_sleep.c
 * @brief   Sleep wrappers for DW3000 driver.
 */

#include "port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

/* Declared in deca_device_api.h, called by the driver. */
void deca_sleep(unsigned int time_ms)
{
    Sleep(time_ms);
}

void deca_usleep(unsigned long time_us)
{
    if (time_us < 1000) {
        esp_rom_delay_us((uint32_t)time_us);
    } else {
        vTaskDelay(pdMS_TO_TICKS(time_us / 1000));
    }
}