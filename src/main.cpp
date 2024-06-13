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

void pollSensors() {
    uint8_t buffer[128];
    size_t message_length;
    bool status;

    RPDeviceReading message = RPDeviceReading_init_default;

    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    message.co2 = 315;
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
}

int main() {
    stdio_init_all();
    sleep_ms(100);
    setup();
    sleep_ms(100);

    while (true) {
        bool takeMeasure = waitForSBC();

        if(takeMeasure){
            pollSensors();
        }

        ledStatus(ledstatus);
    }
    return 0;
}