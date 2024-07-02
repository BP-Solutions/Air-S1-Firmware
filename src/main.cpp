#include <cstring>
#include <cmath>
#include <cstdio>
#include <pb_encode.h>
#include <pb_decode.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/structs/rosc.h"
#include "hardware/gpio.h"

#include "Adafruit_NeoPixel.hpp"

#include "RPDeviceReading.pb.h"
#include "SBCDeviceTelemetry.pb.h"

#include "s8_uart.h"

#include "sen5x_i2c.h"
#include "sensirion_i2c_hal.h"

using namespace std;

//control vars
int ledStatus = 0;
bool takeMeasure = false;

Adafruit_NeoPixel pixels(1, 12, NEO_GRB + NEO_KHZ800);

#define SBC_UART_ID uart1
#define SBC_UART_TX_PIN 4
#define SBC_UART_RX_PIN 5

#define S8_UART_ID uart0
#define S8_TX_PIN 0
#define S8_RX_PIN 1
S8_UART *sensor_S8;
S8_sensor sensor;

int16_t error = 0;
struct SensorValues {
    int mass_concentration_pm1p0;
    int mass_concentration_pm2p5;
    int mass_concentration_pm4p0;
    int mass_concentration_pm10p0;
    int ambient_humidity;
    int ambient_temperature;
    int voc_index;
};

void ledShowStatus(int receivedStatus) {
    absolute_time_t startTime = get_absolute_time();

    int g = 0;
    int r = 0;
    int b = 0;

    int s = 0; //speed of fade
    int n = 0; //number of flashes per cycle
    bool on = true;

    if (receivedStatus == 1) { //orange ish color, sensor error
        r = 255;
        g = 136;
        b = 0;

        s = 3;
        n = 2;
        on = false;
    } else if (receivedStatus == 2) { //wifi or connection error
        r = 255;
        g = 0;
        b = 0;

        s = 3;
        n = 2;
        on = false;
    } else if (receivedStatus == 3) { //booting up i think
        r = 0;
        g = 255;
        b = 0;

        s = 3;
        n = 2;
        on = false;
    } else if (receivedStatus == 4) { //all good, taking measurements
        r = 255;
        g = 0;
        b = 255;

        s = 3;
        n = 2;
        on = false;
    } else if (receivedStatus == 5) { //all good, taking measurements
        r = 0;
        g = 0;
        b = 255;

        s = 10;
        n = 1;
        on = true;
    } else { //didn't set value yet, just be white
        r = 255;
        g = 255;
        b = 255;

        s = 3;
        n = 2;
        on = false;
    }

    pixels.setPixelColor(0, pixels.Color(g, r, b));

    for (int t = n; t > 0; t--) {
        for (int i = 150; i <= 250; i++) {
            int value = pow(2, i / 32.0);
            pixels.setBrightness(value);
            pixels.show();
            sleep_ms(s);
        }

        for (int i = 250; i >= 150; i--) {
            int value = pow(2, i / 32.0);
            pixels.setBrightness(value);
            pixels.show();
            sleep_ms(s);
        }
    }

    if (!on) {
        pixels.clear();
        pixels.show();
    }

    absolute_time_t endTime = get_absolute_time();
    int64_t duration = absolute_time_diff_us(startTime, endTime) / 1000;

    sleep_ms(4000 - duration); //superloop iteration duration controller: loop time is 4S, logic calls cannot exceed 4Hz
}

int readS8() {
    sensor.co2 = sensor_S8->get_co2();
    return sensor.co2;
}

SensorValues readSEN54() {
    uint16_t mass_concentration_pm1p0;
    uint16_t mass_concentration_pm2p5;
    uint16_t mass_concentration_pm4p0;
    uint16_t mass_concentration_pm10p0;
    int16_t ambient_humidity;
    int16_t ambient_temperature;
    int16_t voc_index;
    int16_t nox_index;

    int error = sen5x_read_measured_values(&mass_concentration_pm1p0, &mass_concentration_pm2p5,
                                           &mass_concentration_pm4p0, &mass_concentration_pm10p0, &ambient_humidity,
                                           &ambient_temperature, &voc_index, &nox_index);

    SensorValues values{};

    //todo run fixed division here for ppm values, or just leave as long float
    if (!error) {
        values.mass_concentration_pm1p0 = mass_concentration_pm1p0;
        values.mass_concentration_pm2p5 = mass_concentration_pm2p5;
        values.mass_concentration_pm4p0 = mass_concentration_pm4p0;
        values.mass_concentration_pm10p0 = mass_concentration_pm10p0;
        values.ambient_humidity = (ambient_humidity == 0x7fff) ? -1 : ambient_humidity / 100.0f;
        values.ambient_temperature = (ambient_temperature == 0x7fff) ? -1 : ambient_temperature / 200.0f;
        values.voc_index = static_cast<int>(voc_index / 10.0f);
    }

    return values;
}

void pollSensors() {
    uint8_t buffer[128];
    size_t message_length;
    bool status;

    RPDeviceReading message = RPDeviceReading_init_default;
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    SensorValues data = readSEN54();
    message.co2 = readS8();

    message.pm1p0 = data.mass_concentration_pm1p0;
    message.pm2p5 = data.mass_concentration_pm2p5;
    message.pm4p0 = data.mass_concentration_pm4p0;
    message.pm10p0 = data.mass_concentration_pm10p0;
    message.temperature = data.ambient_temperature;
    message.humidity = data.ambient_humidity;
    message.voc = data.voc_index;

    status = pb_encode(&stream, RPDeviceReading_fields, &message);
    message_length = stream.bytes_written;

    if (!status) {
        printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
    } else {
        uart_puts(SBC_UART_ID, "M:");
        char uart_string[5];
        for (size_t i = 0; i < message_length; ++i) {
            snprintf(uart_string, sizeof(uart_string), "%02x", buffer[i]);
            uart_puts(SBC_UART_ID, uart_string);
        }
        uart_puts(SBC_UART_ID, "\r\n");
    }
}

void waitForSBC() {
    char input_hex[256];
    size_t input_pos = 0;

    while (true) {
        if (uart_is_readable(SBC_UART_ID)) {
            int c = uart_getc(SBC_UART_ID);
            if (c != PICO_ERROR_TIMEOUT) {
                if (c == '\n' || c == '\r') {
                    if (input_pos > 0) {
                        input_hex[input_pos] = '\0';
                        size_t input_length = input_pos;

                        if (strncmp(input_hex, "R:", 2) != 0) {
                            input_pos = 0;
                            continue;
                        }

                        uint8_t buffer[128];
                        size_t buffer_length = (input_length - 2) / 2;

                        for (size_t i = 0; i < buffer_length; ++i) {
                            sscanf(input_hex + 2 * i + 2, "%2hhx", &buffer[i]);
                        }

                        SBCDeviceTelemetry message = SBCDeviceTelemetry_init_default;
                        pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_length);
                        bool status = pb_decode(&stream, SBCDeviceTelemetry_fields, &message);

                        if (status) {
                            ledStatus = message.statCode;
                            takeMeasure = message.sampleSensors;
                            return;
                        }
                        input_pos = 0;
                    }
                } else if (input_pos < sizeof(input_hex) - 1) {
                    input_hex[input_pos++] = c;
                }
            }
        } else {
            sleep_ms(10);
        }
    }
}

void setup() {
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();

    uart_init(SBC_UART_ID, 115200);
    gpio_set_function(SBC_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(SBC_UART_RX_PIN, GPIO_FUNC_UART);

    sensor_S8 = new S8_UART(S8_UART_ID, S8_TX_PIN, S8_RX_PIN);

    sensirion_i2c_hal_init();
    error = sen5x_device_reset();
    if (error) {
        printf("Error executing sen5x_device_reset(): %i\n", error);
    }
    float temp_offset = 0.0f;
    int16_t default_slope = 0;
    uint16_t default_time_constant = 0;
    error = sen5x_set_temperature_offset_parameters((int16_t) (200 * temp_offset), default_slope,
                                                    default_time_constant);
    if (error) {
        printf("Error executing sen5x_set_temperature_offset_parameters(): %i\n", error);
    }
    error = sen5x_start_measurement();
    if (error) {
        printf("Error executing sen5x_start_measurement(): %i\n", error);
    }
}

int main() {
    stdio_init_all();
    sleep_ms(100);
    setup();
    sleep_ms(5000); // let sensors come online

    while (true) {
        waitForSBC();

        if (takeMeasure) {
            pollSensors();
        }

        ledShowStatus(ledStatus);
    }
    return 0;
}