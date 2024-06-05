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

#define PIN 27
#define NUMPIXELS 1

using namespace std;

char ssid[] = "birdpump-cisco-iot";
char pass[] = "Wato.pato554!";

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

QueueHandle_t xStatusQueue;


void led_task(void *pvParameters) {
    while (true) {
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        vTaskDelay(100);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        printf("on\n"); // Replacing cout with printf
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

            s = 2;
            n = 2;
            on = false;
        } else if (receivedStatus == 3) {
            r = 0;
            g = 255;
            b = 0;

            s = 3;
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

    xTaskCreate(led_task, "led_task", 256, NULL, 1, NULL);

    xTaskCreate(vSetStatusTask, "vSetStatusTask", 256, NULL, 2, NULL);

    xTaskCreate(neopixel_task, "neopixel_task", 256, NULL, 2, NULL);


    vTaskStartScheduler();
}