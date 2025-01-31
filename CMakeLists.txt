cmake_minimum_required(VERSION 3.12)

set(PICO_TOOLCHAIN_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi/")

set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk/")
# Name project
SET(ProjectName air-s1)

# Import those libraries
include(pico_sdk_import.cmake)

# Define project
project(${ProjectName} CXX C ASM)

# needed to allow reset using openocd
add_compile_definitions(PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/nanopb/extra)

add_subdirectory(lib/pico_neopixels)

add_subdirectory(lib/nanopb)

add_subdirectory(lib/S8_UART)

add_subdirectory(lib/embedded-i2c-sen5x)

add_subdirectory(src)