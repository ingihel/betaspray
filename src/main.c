#include "camera.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
// #include "fatfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include "server.h"
#include "servo.h"
#include "wifi.h"
#include <stdio.h>

#define LED_PIN 38
#define CAM_UART_NUM UART_NUM_0
#define CAM_UART_BAUD 115200

// Frame framing: START(4) + LENGTH(4, LE uint32) + JPEG DATA + END(4)
static const uint8_t FRAME_START[] = {0xAA, 0xBB, 0xCC, 0xDD};
static const uint8_t FRAME_END[] = {0xDD, 0xCC, 0xBB, 0xAA};

static void uart_send_frame(const uint8_t *data, size_t len) {
    uint32_t length = (uint32_t)len;
    uart_write_bytes(CAM_UART_NUM, FRAME_START, sizeof(FRAME_START));
    uart_write_bytes(CAM_UART_NUM, (const char *)&length, sizeof(length));
    uart_write_bytes(CAM_UART_NUM, (const char *)data, len);
    uart_write_bytes(CAM_UART_NUM, FRAME_END, sizeof(FRAME_END));
}

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

    // Initialize AP and HTTP server component
    ESP_LOGI("MAIN", "Initializing WiFi");
    wifi_init_softap();
    ESP_LOGI("MAIN", "WiFi initialized");

    ESP_LOGI("MAIN", "Starting HTTP server");
    server_start();
    ESP_LOGI("MAIN", "HTTP server started");

    // CAMERA: testing configuration
    // using QVGA (320x240) JPEG keeps frames small (~5-20 KB)
    // This allows streaming at 921600 baud with a comfortable margin
    ESP_LOGI("MAIN", "Initializing camera");
    ESP_ERROR_CHECK(camera_init());
    ESP_LOGI("MAIN", "Camera initialized");

    ESP_LOGI("MAIN", "Setting camera resolution");
    camera_set_resolution(FRAMESIZE_QVGA);
    ESP_LOGI("MAIN", "Setting camera format");
    camera_set_format(CAMERA_FORMAT_JPEG);
    ESP_LOGI("MAIN", "Camera setup complete");

    // Set log level to ERROR now that initialization is complete
    // Frame streaming uses UART0, so INFO logs would corrupt binary data.
    esp_log_level_set("*", ESP_LOG_ERROR);

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
            camera_fb_t *fb = camera_capture_frame();
            if (fb) {
                uart_send_frame(fb->buf, fb->len);
                camera_return_frame(fb);
            }
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}
