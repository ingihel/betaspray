#include "camera.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "fatfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include "route.h"
#include "server.h"
#include "servo.h"
#include "wifi.h"
#include <stdio.h>

#define LED_PIN 38
#define CAM_UART_NUM UART_NUM_0
#define CAM_UART_BAUD 115200

void app_main(void) {
    ESP_LOGI("MAIN", "Starting app_main");
    esp_err_t ret = nvs_flash_init();
    ESP_LOGI("MAIN", "nvs_flash_init() done");
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI("MAIN", "Erasing NVS flash");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI("MAIN", "Creating default event loop");
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_LOGI("MAIN", "Event loop created");

    // Bump UART0 baud rate up to 921600 for camera frame streaming (TEMPORARY, but likely can stay)
    // All ESP log output also comes through here - keep log level at ERROR s.t. binary frame data
    // isn't drowned out by text
    ESP_LOGI("MAIN", "Configuring UART");
    uart_config_t uart_config = {
        .baud_rate = CAM_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(CAM_UART_NUM, &uart_config);
    uart_driver_install(CAM_UART_NUM, 4096, 0, 0, NULL, 0);
    ESP_LOGI("MAIN", "UART configured");

    // LED strip
    // NOTE: This should be removed for non-devkit configurations
    ESP_LOGI("MAIN", "Initializing LED strip");
    led_strip_handle_t led_strip;
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = 1,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB,
        .led_model = LED_MODEL_WS2812,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    ESP_LOGI("MAIN", "LED strip created");
    led_strip_clear(led_strip);
    ESP_LOGI("MAIN", "LED strip cleared");

    // Initialize FatFS on internal SPI flash
    ESP_LOGI("MAIN", "Initializing FatFS");
    ESP_ERROR_CHECK(fatfs_init(true));
    ESP_LOGI("MAIN", "FatFS initialized");

    // Initialize AP and HTTP server component
    ESP_LOGI("MAIN", "Initializing WiFi (SoftAP)");
    wifi_init_softap();
    ESP_LOGI("MAIN", "WiFi (SoftAP) initialized");

    ESP_LOGI("MAIN", "Starting HTTP server");
    server_start();
    ESP_LOGI("MAIN", "HTTP server started");

    // Servo init — GPIO1 (servo0), GPIO2 (servo1).  GPIO5-11 conflict with camera;
    // do not increase NUM_SERVOS beyond 2 without first reassigning those pins.
    // SG90 power note: use a dedicated supply, NOT the ESP32 3.3 V pin (40 mA max).
    ESP_LOGI("MAIN", "Initializing servos");
    servo_init();
    ESP_LOGI("MAIN", "Servos initialized");

    // Route management system
    ESP_LOGI("MAIN", "Initializing route system");
    route_init();
    ESP_LOGI("MAIN", "Route system initialized");

    // TMP: perform servo tests
    servo_testbench_x(0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    servo_testbench_y(1);

    // Set log level to ERROR now that initialization is complete
    // Frame streaming uses UART0, so INFO logs would corrupt binary data.
    esp_log_level_set("*", ESP_LOG_ERROR);

    // But keep route, fatfs, and server logs at INFO level for debugging
    esp_log_level_set("route", ESP_LOG_INFO);
    esp_log_level_set("fatfs", ESP_LOG_INFO);
    esp_log_level_set("server", ESP_LOG_INFO);
    esp_log_level_set("servo", ESP_LOG_INFO);

    uint8_t colors[][3] = {
        {20, 0, 0},  // Red
        {0, 20, 0},  // Green
        {0, 0, 20},  // Blue
        {20, 20, 0}, // Yellow
        {0, 20, 20}, // Cyan
        {20, 0, 20}, // Magenta
    };
    int num_colors = sizeof(colors) / sizeof(colors[0]);
    int current_color = 0;

    // Take a picture every 10s
    // As is, the LED cycles every 500 ms so we count ticks.
    const int ticks_per_photo = 10000 / 500; // 20 ticks
    int ticks = 0;

    while (1) {
        led_strip_set_pixel(led_strip, 0, colors[current_color][0], colors[current_color][1],
                            colors[current_color][2]);
        led_strip_refresh(led_strip);
        current_color = (current_color + 1) % num_colors;

        if (++ticks >= ticks_per_photo) {
            ticks = 0;
            // Virtual slower event loop
            // Remove?
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
