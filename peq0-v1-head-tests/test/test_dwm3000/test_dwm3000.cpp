#include <unity.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "dwm3000.h"

#define PIN_MOSI 18
#define PIN_MISO 20
#define PIN_SCLK 19
#define PIN_CS   16
#define PIN_RST  17

void setUp(void) {
    // Defensive — make sure no previous test left state behind
    dwm3000_deinit();
}

void tearDown(void) {
    dwm3000_deinit();
}

void test_dwm3000_init_succeeds(void) {
    TEST_ASSERT_EQUAL(ESP_OK,
        dwm3000_init(PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS, PIN_RST));
}

void test_dwm3000_devid_is_readable(void) {
    TEST_ASSERT_EQUAL(ESP_OK,
        dwm3000_init(PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS, PIN_RST));

    uint32_t devid = 0;
    TEST_ASSERT_EQUAL(ESP_OK, dwm3000_read_devid(&devid));
    TEST_ASSERT_NOT_EQUAL(0x00000000, devid);
    TEST_ASSERT_NOT_EQUAL(0xFFFFFFFF, devid);
}

void test_dwm3000_devid_matches_qorvo_signature(void) {
    TEST_ASSERT_EQUAL(ESP_OK,
        dwm3000_init(PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS, PIN_RST));

    uint32_t devid = 0;
    TEST_ASSERT_EQUAL(ESP_OK, dwm3000_read_devid(&devid));
    TEST_ASSERT_EQUAL_HEX16(DW3000_DEV_ID_EXPECTED_HI, (devid >> 16) & 0xFFFF);
}

void test_dwm3000_ready_within_timeout(void) {
    TEST_ASSERT_EQUAL(ESP_OK,
        dwm3000_init(PIN_MOSI, PIN_MISO, PIN_SCLK, PIN_CS, PIN_RST));
    TEST_ASSERT_EQUAL(ESP_OK, dwm3000_wait_ready(200));
}

extern "C" void app_main(void) {
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Hard-reset the DWM3000 at boot to clear any state from previous firmware
    gpio_set_direction((gpio_num_t)PIN_RST, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_direction((gpio_num_t)PIN_RST, GPIO_MODE_INPUT);
    vTaskDelay(pdMS_TO_TICKS(100));

    UNITY_BEGIN();
    RUN_TEST(test_dwm3000_init_succeeds);
    RUN_TEST(test_dwm3000_devid_is_readable);
    RUN_TEST(test_dwm3000_devid_matches_qorvo_signature);
    RUN_TEST(test_dwm3000_ready_within_timeout);
    UNITY_END();
}