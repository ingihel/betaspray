#include "servo.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "servo";

// Must match NUM_SERVOS — index maps channel to GPIO
static const int servo_pins[8] = {
    SERVO_PIN_0, SERVO_PIN_1, SERVO_PIN_2, SERVO_PIN_3,
    SERVO_PIN_4, SERVO_PIN_5, SERVO_PIN_6, SERVO_PIN_7,
};

// SG90: 50 Hz, 14-bit resolution
// 1 ms pulse (0°)  = 819  counts
// 2 ms pulse (180°)= 1638 counts
#define LEDC_FREQ_HZ     50
#define LEDC_RESOLUTION  LEDC_TIMER_14_BIT
#define DUTY_MIN         819
#define DUTY_MAX         1638

static uint32_t angle_to_duty(int angle)
{
    return DUTY_MIN + ((uint32_t)angle * (DUTY_MAX - DUTY_MIN)) / 180;
}

void servo_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < NUM_SERVOS; i++) {
        ledc_channel_config_t ch = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)i,
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = servo_pins[i],
            .duty       = 0,
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
        ESP_LOGI(TAG, "Servo %d init on GPIO %d", i, servo_pins[i]);
    }

    ESP_LOGI(TAG, "%d servo(s) ready at %d Hz", NUM_SERVOS, LEDC_FREQ_HZ);
}

void servo_drive(int id, int angle)
{
    if (id < 0 || id >= NUM_SERVOS) {
        ESP_LOGE(TAG, "Servo id %d out of range (0-%d)", id, NUM_SERVOS - 1);
        return;
    }
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;

    uint32_t duty = angle_to_duty(angle);
    ESP_LOGI(TAG, "Servo %d -> %d deg (duty %u) for %d ms",
             id, angle, (unsigned)duty, SERVO_DURATION_MS);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)id, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)id);

    vTaskDelay(SERVO_DURATION_MS / portTICK_PERIOD_MS);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)id, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)id);
    ESP_LOGI(TAG, "Servo %d released", id);
}
