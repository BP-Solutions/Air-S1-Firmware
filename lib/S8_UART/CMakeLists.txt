# Define library
add_library(S8UART
        src/s8_uart.cpp
        src/s8_uart.h
        src/modbus_crc.cpp
        src/modbus_crc.h
        src/utils.cpp
        src/utils.h
)

target_link_libraries(S8UART pico_stdlib)

target_include_directories(S8UART INTERFACE ${CMAKE_CURRENT_LIST_DIR}/src)

# Include directories
target_include_directories(S8UART PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})