#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "nvs_flash.h"
#include "server.h"
#include "servo.h"
#include "wifi.h"
#include <stdio.h>

// The standard built-in RGB LED pin for ESP32-S3-DevKitC-1
#define LED_PIN 38

void app_main(void) {
  // Required before any WiFi or network init
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  printf("Starting RGB LED Color Cycle...\n");

  // 1. Configure the LED strip
  led_strip_handle_t led_strip;

  // Set up the GPIO and the type of LED
  led_strip_config_t strip_config = {
      .strip_gpio_num = LED_PIN,
      .max_leds = 1, // We only have 1 built-in LED
      .led_pixel_format = LED_PIXEL_FORMAT_GRB,
      .led_model = LED_MODEL_WS2812,
  };

  // Set up the hardware peripheral (RMT) to generate the signal
  led_strip_rmt_config_t rmt_config = {
      .resolution_hz = 10 * 1000 * 1000, // 10MHz resolution
  };

  // Initialize the LED strip
  led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);

  // Clear the LED (turns off that blinding white light!)
  led_strip_clear(led_strip);

  // 2. Define a list of colors (Red, Green, Blue)
  // Values range from 0 to 255. Kept at 20 so it isn't blinding.
  uint8_t colors[][3] = {
      {20, 0, 0},  // Red
      {0, 20, 0},  // Green
      {0, 0, 20},  // Blue
      {20, 20, 0}, // Yellow
      {0, 20, 20}, // Cyan
      {20, 0, 20}  // Magenta
  };

  int num_colors = sizeof(colors) / sizeof(colors[0]);
  int current_color = 0;

  wifi_init_softap();
  server_start();

  servo_init();

  // 3. Infinite loop to cycle colors
  while (1) {
    // Set the color of pixel 0 (our only LED)
    led_strip_set_pixel(led_strip, 0, colors[current_color][0],
                        colors[current_color][1], colors[current_color][2]);

    // Push the data to the LED to apply the color
    led_strip_refresh(led_strip);

    // Move to the next color in the array
    current_color++;
    if (current_color >= num_colors) {
      current_color = 0; // Wrap back to the start
    }
    servo_drive(0, 30);

    // Wait 500 milliseconds
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}
