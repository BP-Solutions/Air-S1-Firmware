# Add the executable
add_executable(${ProjectName}
        main.cpp
)

# Include directories
target_include_directories(${ProjectName} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../lib
)

find_package(Nanopb REQUIRED)

# Generate C++ code for the proto files
nanopb_generate_cpp(
        TARGET proto
        protobufs/SBCDeviceTelemetry.proto
        protobufs/RPDeviceReading.proto
)

# Link libraries
target_link_libraries(${ProjectName}
        pico_stdlib
        hardware_gpio
        hardware_spi
        hardware_timer
        pico_neopixel
        proto
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(${ProjectName} 0)
pico_enable_stdio_uart(${ProjectName} 1)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(${ProjectName})
