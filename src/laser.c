#include "laser.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "laser";
static bool s_on = false;

esp_err_t laser_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LASER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }
    gpio_set_level(LASER_PIN, 0);
    s_on = false;
    ESP_LOGI(TAG, "Laser init on GPIO %d (off)", LASER_PIN);
    return ESP_OK;
}

void laser_set(bool on) {
    gpio_set_level(LASER_PIN, on ? 1 : 0);
    s_on = on;
    ESP_LOGI(TAG, "Laser %s", on ? "ON" : "OFF");
}

void laser_toggle(void) {
    laser_set(!s_on);
}

bool laser_is_on(void) {
    return s_on;
}
