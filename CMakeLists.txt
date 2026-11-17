cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in the Pico SDK (set PICO_SDK_PATH, or pass -DPICO_SDK_FETCH_FROM_GIT=ON)
include(pico_sdk_import.cmake)

project(lego_dbw C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# Feature stage, see src/config.h and docs/test-plan.md:  cmake -S . -B build -DDBW_STAGE=3
set(DBW_STAGE 1 CACHE STRING "Feature stage 0..7")
message(STATUS "LEGO DBW feature stage: ${DBW_STAGE}")

add_executable(lego_dbw
    src/lego_dbw_system.c
    src/lwp3.c
    src/steering.c
    src/drive.c
    src/supervisor.c
    src/mavlink_lite.c
    src/telemetry.c
    src/pwm_capture.c
)

pico_generate_pio_header(lego_dbw ${CMAKE_CURRENT_LIST_DIR}/src/pwm_capture.pio)

# btstack_config.h lives in config/
target_include_directories(lego_dbw PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/config
    ${CMAKE_CURRENT_LIST_DIR}/src
)

target_compile_options(lego_dbw PRIVATE -Wall)

# BLE only: no WiFi / lwIP stack
target_compile_definitions(lego_dbw PRIVATE
    CYW43_LWIP=0
    DBW_STAGE=${DBW_STAGE}
)

target_link_libraries(lego_dbw
    pico_stdlib
    hardware_gpio
    hardware_timer
    hardware_watchdog
    hardware_sync
    hardware_pio
    hardware_clocks
    hardware_uart
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_poll
)

pico_enable_stdio_usb(lego_dbw 1)
pico_enable_stdio_uart(lego_dbw 0)

pico_add_extra_outputs(lego_dbw)
