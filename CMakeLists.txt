cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in the Pico SDK (set PICO_SDK_PATH, or pass -DPICO_SDK_FETCH_FROM_GIT=ON)
include(pico_sdk_import.cmake)

project(lego_dbw C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(lego_dbw
    src/lego_dbw_system.c
)

# btstack_config.h lives in config/
target_include_directories(lego_dbw PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/config
)

target_compile_options(lego_dbw PRIVATE -Wall)

# BLE only: no WiFi / lwIP stack
target_compile_definitions(lego_dbw PRIVATE CYW43_LWIP=0)

target_link_libraries(lego_dbw
    pico_stdlib
    hardware_gpio
    hardware_timer
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_poll
)

pico_enable_stdio_usb(lego_dbw 1)
pico_enable_stdio_uart(lego_dbw 0)

pico_add_extra_outputs(lego_dbw)
