# ---------------------------------------------------------------------------
# components/peripherals/CMakeLists.txt
# Registers everything in this folder (ble_provision, wifi_udp_tx, lsm6dsv, ...)
# as the ESP-IDF component "peripherals" (component name = folder name).
#
# REQUIRES vs PRIV_REQUIRES:
#   REQUIRES      = components whose headers appear in THIS component's PUBLIC
#                   headers (so consumers like src/ get them transitively).
#   PRIV_REQUIRES = components used only inside the .cpp implementations.
# The public headers here expose only esp_err_t + (for lsm6dsv) the I2C driver,
# so "driver" is public and the radios are private.
# ---------------------------------------------------------------------------
file(GLOB peripherals_srcs
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

idf_component_register(
    SRCS ${peripherals_srcs}
    INCLUDE_DIRS "."
    REQUIRES
        driver                 # I2C, if lsm6dsv.h exposes i2c types
    PRIV_REQUIRES
        bt                     # NimBLE host + controller
        esp_wifi               # Wi-Fi station
        esp_event              # default event loop
        esp_netif              # netif / DHCP
        lwip                   # BSD UDP sockets
        nvs_flash              # persisted provisioning
        esp_timer              # timestamps in the packets
        
)
