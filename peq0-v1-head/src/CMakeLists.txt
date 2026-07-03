# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

file(GLOB app_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
 
idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "."
    REQUIRES
        peripherals          # <-- our component (ble_provision.h, wifi_udp_tx.h, lsm6dsv.h)
        esp_timer              # main.cpp uses esp_timer_* directly
        nvs_flash              # main.cpp calls nvs_flash_init()
)
