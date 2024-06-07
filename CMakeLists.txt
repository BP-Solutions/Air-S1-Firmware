cmake_minimum_required(VERSION 3.12)

set(PICO_TOOLCHAIN_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/gcc-arm-none-eabi/")

set(PICO_SDK_PATH "C:/Program Files/Raspberry Pi/Pico SDK v1.5.1/pico-sdk/")
# Name project
SET(ProjectName air-s1)

# set board
set(PICO_BOARD pico_w)


# Set any variables required for importing libraries
SET(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)

# Import those libraries
include(pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Define project
project(${ProjectName} CXX C ASM)

# needed to allow reset using openocd
add_compile_definitions(PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=1)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/nanopb/extra)

add_subdirectory(lib/pico_neopixels)

add_subdirectory(lib/nanopb)

# Add subdirectories
add_subdirectory(src)
