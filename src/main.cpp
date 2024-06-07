#include "hardware/structs/rosc.h"

#include <iostream>
#include <sstream>
#include <cmath>
#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "Adafruit_NeoPixel.hpp"
#include "queue.h"


#include <pb_encode.h>
#include <pb_decode.h>
#include "simple.pb.h"


using namespace std;

#define PIN 27
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


char ssid[] = "birdpump-cisco-iot";
char pass[] = "Wato.pato554!";


QueueHandle_t xStatusQueue;


void testPB() {
    uint8_t buffer[128];
    size_t message_length;
    bool status;

    /* Encode our message */
    {
        /* Allocate space on the stack to store the message data.
         *
         * Nanopb generates simple struct definitions for all the messages.
         * - check out the contents of simple.pb.h!
         * It is a good idea to always initialize your structures
         * so that you do not have garbage data from RAM in there.
         */
        SimpleMessage message = SimpleMessage_init_zero;

        /* Create a stream that will write to our buffer. */
        pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

        /* Fill in the lucky number */
        message.lucky_number = 13;
        message.value = 21;

        /* Now we are ready to encode the message! */
        status = pb_encode(&stream, SimpleMessage_fields, &message);
        message_length = stream.bytes_written;

        /* Then just check for any errors.. */
        if (!status)
        {
            printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        }

        // Print the encoded buffer as a hex string
        printf("Encoded message (hex): ");
        for (size_t i = 0; i < message_length; ++i) {
            printf("%02x", buffer[i]);
        }
        printf("\n");
    }

    /* Now we could transmit the message over network, store it in a file or
     * wrap it to a pigeon's leg.
     */

    /* But because we are lazy, we will just decode it immediately. */

    {
        /* Allocate space for the decoded message. */
        SimpleMessage message = SimpleMessage_init_zero;

        /* Create a stream that reads from the buffer. */
        pb_istream_t stream = pb_istream_from_buffer(buffer, message_length);

        /* Now we are ready to decode the message. */
        status = pb_decode(&stream, SimpleMessage_fields, &message);

        /* Check for errors... */
        if (!status)
        {
            printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
        }

        /* Print the data contained in the message. */
        printf("number was %d!\n", message.lucky_number);
    }
}

void changeColor(int tests){
    if (xQueueSend(xStatusQueue, &tests, portMAX_DELAY) != pdPASS) {
        printf("Failed to send to queue\n");
    }
}

void serial_task(void *pvParameters) {
    char input_hex[256];
    size_t input_pos = 0;

    int colors = 2;

    while (1) {


        // Check if there is input available
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            if (c == '\n' || c == '\r') {
                if (input_pos > 0) {
                    input_hex[input_pos] = '\0';
                    size_t input_length = input_pos;
                    uint8_t buffer[128];
                    size_t buffer_length = input_length / 2;

                    for (size_t i = 0; i < buffer_length; ++i) {
                        sscanf(input_hex + 2 * i, "%2hhx", &buffer[i]);
                    }

                    SimpleMessage message = SimpleMessage_init_zero;
                    pb_istream_t stream = pb_istream_from_buffer(buffer, buffer_length);
                    bool status = pb_decode(&stream, SimpleMessage_fields, &message);

                    if (!status) {
                        printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
                    } else {
                        printf("Decoded message: lucky_number = %d, value = %d\n", message.lucky_number, message.value);
                    }

                    if(message.value == 21){
                        if (colors == 2) {
                            colors = 3;
                        } else {
                            colors = 2;
                        }
                        changeColor(colors);
                        printf("changed led color");
                    }
                    input_pos = 0; // Reset input buffer
                }
            } else if (input_pos < sizeof(input_hex) - 1) {
                input_hex[input_pos++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Short delay to yield to other tasks
    }
}


void led_task(void *pvParameters) {
    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(100);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        vTaskDelay(100);
    }
}


void neopixel_task(void *pvParameters) {
    int receivedStatus;

    int g = 0;
    int r = 0;
    int b = 0;

    int s = 0; //speed of fade
    int n = 0; //number of flashes per cycle
    bool on = true;

    while (1) {
        xQueueReceive(xStatusQueue, &receivedStatus, 0);

        printf("led thing is called with status %d \n", receivedStatus);

        if (receivedStatus == 1) {
            r = 255;
            g = 136;
            b = 0;

            s = 2;
            n = 2;
            on = false;
        } else if (receivedStatus == 2) {
            r = 255;
            g = 0;
            b = 0;

            s = 3;
            n = 2;
            on = false;
        } else if (receivedStatus == 3) {
            r = 0;
            g = 255;
            b = 0;

            s = 4;
            n = 2;
            on = false;
        } else if (receivedStatus == 4) {
            r = 0;
            g = 0;
            b = 255;

            s = 25;
            n = 1;
            on = true;
        }else{
            r = 255;
            g = 255;
            b = 255;

            s = 2;
            n = 2;
            on = false;
        }

        pixels.setPixelColor(0, pixels.Color(g, r, b));

        for (int t = n; t > 0; t--) {
            for (int i = 150; i <= 250; i++) {
                int value = pow(2, i / 32.0);
                pixels.setBrightness(value);
                pixels.show();
                vTaskDelay(pdMS_TO_TICKS(s));
            }

            for (int i = 250; i >= 150; i--) {
                int value = pow(2, i / 32.0);
                pixels.setBrightness(value);
                pixels.show();
                vTaskDelay(pdMS_TO_TICKS(s));
            }
        }

        if (!on) {
            pixels.clear();
            pixels.show();
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void vSetStatusTask(void *pvParameters) {
    int status = 4;

    while (1) {
        xQueueSend(xStatusQueue, &status, portMAX_DELAY);

        if (status == 4) {
            status = 3;
        } else {
            status = 4;
        }

        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}


void setup() {
    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();

    if (cyw43_arch_init_with_country(CYW43_COUNTRY_USA)) {
        printf("failed to initialise\n");
        return;
    }
    printf("initialised\n");

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(ssid, pass, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
        printf("failed to connect\n");
        return;
    }
    printf("connected\n");

    xStatusQueue = xQueueCreate(10, sizeof(int));
}


int main() {
    stdio_init_all();

    setup();

    testPB();


    xTaskCreate(led_task, "led_task", 256, NULL, 1, NULL);
//    xTaskCreate(vSetStatusTask, "vSetStatusTask", 256, NULL, 2, NULL);
    xTaskCreate(neopixel_task, "neopixel_task", 512, NULL, 3, NULL);
    xTaskCreate(serial_task, "serial_task", 512, NULL, 4, NULL);

    vTaskStartScheduler();
}