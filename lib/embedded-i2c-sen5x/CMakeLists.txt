add_library(SEN54
        src/sen5x_i2c.c
        src/sen5x_i2c.h
        src/sensirion_common.c
        src/sensirion_common.h
        src/sensirion_config.h
        src/sensirion_i2c.c
        src/sensirion_i2c.c
        src/sensirion_i2c_hal.c
        src/sensirion_i2c_hal.c
)

target_link_libraries(SEN54 pico_stdlib hardware_i2c)

target_include_directories(SEN54 INTERFACE ${CMAKE_CURRENT_LIST_DIR}/src)

target_include_directories(SEN54 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})