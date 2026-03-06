#include "servo.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "servo";

// Tracks the last commanded angle per servo so servo_drive can step from
// the current position rather than jumping.  -1 = position unknown (first move).
static int servo_current_angle[8];

// Must match NUM_SERVOS - index maps channel to GPIO
static const int servo_pins[8] = {
    SERVO_PIN_0, SERVO_PIN_1, SERVO_PIN_2, SERVO_PIN_3,
    SERVO_PIN_4, SERVO_PIN_5, SERVO_PIN_6, SERVO_PIN_7,
};

// SG90: 50 Hz, 14-bit resolution (16384 ticks per 20 ms period)
// Full travel requires the extended pulse range, not just 1-2 ms:
//   0.5 ms (0°)   = 16384 * 0.5 / 20  =  410 counts
//   2.4 ms (180°) = 16384 * 2.4 / 20  = 1966 counts
// At 3.3 V the ends may have reduced torque; back off DUTY_MIN/MAX slightly
// if the servo buzzes or binds at the extremes.
#define LEDC_FREQ_HZ 50
#define LEDC_RESOLUTION LEDC_TIMER_14_BIT
#define DUTY_MIN 410
#define DUTY_MAX 1966

// One PWM period in ms — used as the minimum fade floor and duty-restore delay.
// Derived from LEDC_FREQ_HZ so it stays correct if the frequency is changed.
#define SERVO_PERIOD_MS (1000 / LEDC_FREQ_HZ)

static uint32_t angle_to_duty(int angle) {
    return DUTY_MIN + ((uint32_t)angle * (DUTY_MAX - DUTY_MIN)) / 180;
}

void servo_init(void) {
    for (int i = 0; i < 8; i++)
        servo_current_angle[i] = -1;

    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    for (int i = 0; i < NUM_SERVOS; i++) {
        ledc_channel_config_t ch = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = servo_pins[i],
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch));
        ESP_LOGI(TAG, "Servo %d init on GPIO %d", i, servo_pins[i]);
    }

    // Required for ledc_set_fade_with_time().  ESP_ERR_INVALID_STATE means the
    // ISR is already installed (e.g. servo_init called twice) — that is fine.
    esp_err_t fade_err = ledc_fade_func_install(0);
    if (fade_err != ESP_OK && fade_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(fade_err);
    }

    ESP_LOGI(TAG, "%d servo(s) ready at %d Hz", NUM_SERVOS, LEDC_FREQ_HZ);
}

void servo_drive(int id, int angle) {
    if (id < 0 || id >= NUM_SERVOS) {
        ESP_LOGE(TAG, "Servo id %d out of range (0-%d)", id, NUM_SERVOS - 1);
        return;
    }
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    ledc_channel_t ch = (ledc_channel_t)id;
    int from = servo_current_angle[id];

    if (from < 0 || from == angle) {
        // Unknown position or no movement: set duty directly.
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, angle_to_duty(angle));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    } else {
        // Proportional fade time so shorter moves are faster.
        int fade_ms = abs(angle - from) * SERVO_FADE_TIME_MS / 180;
        if (fade_ms < SERVO_PERIOD_MS)
            fade_ms = SERVO_PERIOD_MS; // floor at one PWM period

        ESP_LOGI(TAG, "Servo %d: %d -> %d deg (fade %d ms)", id, from, angle, fade_ms);

        // After the previous release the LEDC duty is 0.  Restore it to the
        // tracked position so the hardware fade starts from the correct level.
        // Wait one full PWM period (25 ms > 20 ms at 50 Hz) so ledc_update_duty
        // is applied before ledc_set_fade_with_time reads the duty register.
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, angle_to_duty(from));
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
        vTaskDelay(pdMS_TO_TICKS(SERVO_PERIOD_MS + 5)); // +5 ms margin for tick quantisation

        // Hardware-driven duty ramp — no FreeRTOS timing involvement.
        ledc_set_fade_with_time(LEDC_LOW_SPEED_MODE, ch, angle_to_duty(angle), fade_ms);
        ledc_fade_start(LEDC_LOW_SPEED_MODE, ch, LEDC_FADE_WAIT_DONE);
    }

    servo_current_angle[id] = angle;
    vTaskDelay(pdMS_TO_TICKS(SERVO_DURATION_MS));

    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    ESP_LOGI(TAG, "Servo %d released at %d deg", id, angle);
}

void servo_testbench_x(int id) {
    ESP_LOGI(TAG, "=== Servo %d testbench X start ===", id);

    // Phase 1: 15-degree increments across full range.
    ESP_LOGI(TAG, "Phase 1: 0 -> 180 in 15-degree steps");
    for (int a = 0; a <= 180; a += 15)
        servo_drive(id, a);

    servo_drive(id, 0); // return to start before phase 2

    // Phase 2: coarse jumps — 30, 45, 90, 180.
    ESP_LOGI(TAG, "Phase 2: 30 -> 45 -> 90 -> 180");
    int seq[] = {30, 45, 90, 180};
    for (int i = 0; i < (int)(sizeof(seq) / sizeof(seq[0])); i++)
        servo_drive(id, seq[i]);

    ESP_LOGI(TAG, "=== Servo %d testbench X complete ===", id);
}

void servo_testbench_y(int id) {
    ESP_LOGI(TAG, "=== Servo %d testbench Y start ===", id);

    // Phase 1: 15-degree increments across the active range (90-180°).
    // Angles are offset +90 since this servo's flat position is at 90°.
    ESP_LOGI(TAG, "Phase 1: 90 -> 180 in 15-degree steps");
    for (int a = 0; a <= 90; a += 15)
        servo_drive(id, a + 90);

    servo_drive(id, 90); // return to flat (90°) before phase 2

    // Phase 2: coarse jumps — 120, 135, 180 (30/45/90 + 90 offset).
    ESP_LOGI(TAG, "Phase 2: 120 -> 135 -> 180");
    int seq[] = {120, 135, 180};
    for (int i = 0; i < (int)(sizeof(seq) / sizeof(seq[0])); i++)
        servo_drive(id, seq[i]);

    ESP_LOGI(TAG, "=== Servo %d testbench Y complete ===", id);
}
