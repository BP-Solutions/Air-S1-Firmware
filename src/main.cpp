#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "FreeRTOS.h"
#include "task.h"

#include <iostream>
#include <sstream>
#include <cstring>


#include "hardware/structs/rosc.h"

#include <iostream>
#include <sstream>
#include <cmath>
#include <cstdio>

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lwip/pbuf.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"

#include "lwip/altcp_tcp.h"
#include "lwip/altcp_tls.h"
#include "lwip/apps/mqtt.h"

#include "lwip/apps/mqtt_priv.h"

#include "Adafruit_NeoPixel.hpp"
#include "queue.h"

#define MQTT_SERVER_IP "10.0.0.101"
#define MQTT_SERVER_PORT 1883
#define WIFI_SSID "birdpump-cisco-iot"
#define WIFI_PASSWORD "Wato.pato554!"


#define PIN 27
#define NUMPIXELS 1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


int numberTest = 0;

typedef struct {
    ip_addr_t remote_addr;
    mqtt_client_t *mqtt_client;
} MQTT_CLIENT_T;

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status) {
    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT connected.\n");
    } else {
        printf("MQTT connection failed: %d\n", status);
    }
}

static void mqtt_pub_request_cb(void *arg, err_t err) {
    if (err == ERR_OK) {
        printf("Publish successful.\n");
    } else {
        printf("Publish failed: %d\n", err);
    }
}

static err_t mqtt_connect(MQTT_CLIENT_T *state) {
    struct mqtt_connect_client_info_t ci;
    memset(&ci, 0, sizeof(ci));
    ci.client_id = "PicoW-1";

    return mqtt_client_connect(state->mqtt_client, &state->remote_addr, MQTT_SERVER_PORT, mqtt_connection_cb, state,
                               &ci);
}

static void mqtt_publish_data(MQTT_CLIENT_T *state) {
    std::stringstream ss;
    ss << "{\"message\":\"hello from picow\", \"number\":" << numberTest << "}";
    std::string message = ss.str();

    printf("Publishing message: %s\n", message.c_str());
    err_t err = mqtt_publish(state->mqtt_client, "pico_w/test", message.c_str(), message.length(), 0, 0,
                             mqtt_pub_request_cb, state);
    printf("Publish status: %d\n", err);

    numberTest++;
}

static void mqtt_task(void *pvParameters) {
    MQTT_CLIENT_T *state = (MQTT_CLIENT_T *) pvParameters;

    while (1) {
        if (mqtt_client_is_connected(state->mqtt_client)) {
            mqtt_publish_data(state);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Run every 10 seconds
    }
}


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
    int receivedStatus = 3;

    int g = 0;
    int r = 0;
    int b = 0;

    int s = 0; //speed of fade
    int n = 0; //number of flashes per cycle
    bool on = true;

    while (1) {


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
        } else {
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


void wifi_init() {
    stdio_init_all();

    pixels.begin();
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show();


    if (cyw43_arch_init()) {
        printf("Failed to initialize.\n");
        return;
    }
    cyw43_arch_enable_sta_mode();

    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect to WiFi.\n");
        return;
    }
    printf("Connected to WiFi.\n");

}

void mqtt_client_setup(MQTT_CLIENT_T *state) {
    state->mqtt_client = mqtt_client_new();
    if (state->mqtt_client == NULL) {
        printf("Failed to create MQTT client.\n");
        return;
    }

    ip4addr_aton(MQTT_SERVER_IP, &state->remote_addr);

    if (mqtt_connect(state) == ERR_OK) {
        int timeout = 100;
        while (!mqtt_client_is_connected(state->mqtt_client) && timeout > 0) {
            cyw43_arch_poll();
            sleep_ms(100);
            timeout--;
        }

        if (!mqtt_client_is_connected(state->mqtt_client)) {
            printf("MQTT connection timeout\n");
        } else {

            printf("MQTT connected successfully\n");
            xTaskCreate(mqtt_task, "MQTT Task", 1024, state, 3, NULL);
        }
    } else {
        printf("Failed to connect to MQTT server.\n");
    }
}

int main() {


    wifi_init();


    MQTT_CLIENT_T *state = (MQTT_CLIENT_T *) calloc(1, sizeof(MQTT_CLIENT_T));
    if (state == NULL) {
        printf("Failed to allocate memory for MQTT state.\n");
        return 1;
    }

    mqtt_client_setup(state);
    xTaskCreate(neopixel_task, "neopixel_task", 256, NULL, 2, NULL);

    xTaskCreate(led_task, "led_task", 256, NULL, 1, NULL);

    printf("Starting tasks...\n");

    vTaskStartScheduler();
}
