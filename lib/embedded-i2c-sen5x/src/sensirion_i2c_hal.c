#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "sensirion_common.h"
#include "sensirion_config.h"
#include "sensirion_i2c_hal.h"

#define I2C_PORT i2c1
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7

void sensirion_i2c_hal_init(void) {
    // Initialize I2C with 100kHz frequency
    i2c_init(I2C_PORT, 100 * 1000);

    // Set up the I2C pins
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
}

void sensirion_i2c_hal_free(void) {
    // Not required for the Pico SDK
}

int8_t sensirion_i2c_hal_read(uint8_t address, uint8_t* data, uint16_t count) {
    int result = i2c_read_blocking(I2C_PORT, address, data, count, false);
    return result == PICO_ERROR_GENERIC ? -1 : 0;
}

int8_t sensirion_i2c_hal_write(uint8_t address, const uint8_t* data, uint16_t count) {
    int result = i2c_write_blocking(I2C_PORT, address, data, count, false);
    return result == PICO_ERROR_GENERIC ? -1 : 0;
}

void sensirion_i2c_hal_sleep_usec(uint32_t useconds) {
    sleep_us(useconds);
}
