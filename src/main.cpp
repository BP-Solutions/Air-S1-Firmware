#include "hardware/structs/rosc.h"

#include <cmath>
#include <cstdio>

#include "pico/stdlib.h"

#include "Adafruit_NeoPixel.hpp"

#include <pb_encode.h>
#include <pb_decode.h>

#include "RPDeviceReading.pb.h"
#include "SBCDeviceTelemetry.pb.h"

using namespace std;


#define PIN 27
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int ledstatus = 0;

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

#include "s8_uart.h"

/* BEGIN CONFIGURATION */
#define DEBUG_BAUDRATE 115200
#define S8_RX_PIN 1         // Rx pin which the S8 Tx pin is attached to
#define S8_TX_PIN 0         // Tx pin which the S8 Rx pin is attached to
#define UART_ID uart0       // UART port
/* END CONFIGURATION */

S8_UART *sensor_S8;
S8_sensor sensor;


void testSensor() {
    // Initialize UART for S8 sensor


    sleep_ms(100);
    // Check if S8 is available
    sensor_S8->get_firmware_version(sensor.firm_version);
    int len = strlen(sensor.firm_version);
    if (len == 0) {
        printf("SenseAir S8 CO2 sensor not found!\n");
        while (1) {
            sleep_ms(1);
        };
    }

    // Show S8 sensor info
    printf(">>> SenseAir S8 NDIR CO2 sensor <<<\n");

    printf("Firmware version: %s\n", sensor.firm_version);

    sensor.sensor_type_id = sensor_S8->get_sensor_type_ID();
    printf("Sensor type: 0x%08X\n", sensor.sensor_type_id);

    sensor.sensor_id = sensor_S8->get_sensor_ID();
    printf("Sensor ID: 0x%08X\n", sensor.sensor_id);

    sensor.map_version = sensor_S8->get_memory_map_version();
    printf("Memory map version: %d\n", sensor.map_version);

    sensor.abc_period = sensor_S8->get_ABC_period();

    if (sensor.abc_period > 0) {
        printf("ABC (automatic background calibration) period: %d hours\n", sensor.abc_period);
    } else {
        printf("ABC (automatic calibration) is disabled\n");
    }

}


void ledStatus(int receivedStatus) {
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

    sleep_ms(4000 - duration);
}

int readCO2(){
    sensor.co2 = sensor_S8->get_co2();
    return sensor.co2;
}

void pollSensors() {
    uint8_t buffer[128];
    size_t message_length;
    bool status;

    RPDeviceReading message = RPDeviceReading_init_default;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    message.co2 = readCO2();
    message.ppm = 20;
    message.temp = 72;
    message.humid = 50;

    status = pb_encode(&stream, RPDeviceReading_fields, &message);
    message_length = stream.bytes_written;

    if (!status) {
        printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
    } else {
        printf("M:");
        for (size_t i = 0; i < message_length; ++i) {
            printf("%02x", buffer[i]);
        }
        printf("\n");
    }
}

bool waitForSBC() {
    char input_hex[256];
    size_t input_pos = 0;

    while (true) {
        int c = getchar_timeout_us(0);
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

                    if (!status) {
                        printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
                    } else {
                        ledstatus = message.statCode;
                        return message.sampleSensors;
                    }
                    input_pos = 0;
                }
            } else if (input_pos < sizeof(input_hex) - 1) {
                input_hex[input_pos++] = c;
            }
        }
    }
}

void setup() {
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();

    sensor_S8 = new S8_UART(UART_ID, S8_TX_PIN, S8_RX_PIN);
}

int main() {
    stdio_init_all();
    sleep_ms(100);
    setup();
    sleep_ms(5000);

    while (true) {

        bool takeMeasure = waitForSBC();

        if (takeMeasure) {
            pollSensors();
        }

        ledStatus(ledstatus);
    }
    return 0;
}