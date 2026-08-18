#include <unity.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lsm6dsv.h"

#define PIN_SDA 22
#define PIN_SCL 23

void setUp(void) {
    lsm6_deinit();
}

void tearDown(void) {
    lsm6_deinit();
}

void test_i2c_bus_initializes(void) {
    TEST_ASSERT_EQUAL(ESP_OK, lsm6_init(PIN_SDA, PIN_SCL));
}

void test_at_least_one_i2c_device_responds(void) {
    TEST_ASSERT_EQUAL(ESP_OK, lsm6_init(PIN_SDA, PIN_SCL));

    int count = 0;
    TEST_ASSERT_EQUAL(ESP_OK, lsm6_scan(&count));
    TEST_ASSERT_GREATER_THAN_MESSAGE(0, count,
        "No I2C devices found — check wiring and pull-ups");
}

void test_lsm6_responds_to_who_am_i(void) {
    TEST_ASSERT_EQUAL(ESP_OK, lsm6_init(PIN_SDA, PIN_SCL));

    uint8_t addr = 0, whoami = 0;
    TEST_ASSERT_EQUAL(ESP_OK, lsm6_read_who_am_i(&addr, &whoami));
    TEST_ASSERT_NOT_EQUAL(0x00, whoami);
    TEST_ASSERT_NOT_EQUAL(0xFF, whoami);
    printf("LSM6 WHO_AM_I = 0x%02X at address 0x%02X\n", whoami, addr);
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Force the chip into I2C mode in case a previous firmware put it in I3C
    lsm6_force_i2c_mode(PIN_SDA, PIN_SCL);

    // Then continue with normal init/reset
    lsm6_init(PIN_SDA, PIN_SCL);
    lsm6_software_reset();
    lsm6_deinit();

    UNITY_BEGIN();
    RUN_TEST(test_i2c_bus_initializes);
    RUN_TEST(test_at_least_one_i2c_device_responds);
    RUN_TEST(test_lsm6_responds_to_who_am_i);
    UNITY_END();
}