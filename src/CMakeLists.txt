# Add the executable
add_executable(${ProjectName}
        main.cpp
)

# Include directories
target_include_directories(${ProjectName} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib
)

# Link libraries
target_link_libraries(${ProjectName}
        pico_stdlib
        hardware_gpio
        hardware_spi
        hardware_timer
        pico_lwip_mqtt
        pico_lwip_mbedtls
        pico_mbedtls
        pico_cyw43_arch_lwip_threadsafe_background
#        pico_cyw43_arch_lwip_sys_freertos
        FreeRTOS-Kernel-Heap4
        pico_neopixel
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 1)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${ProjectName})
