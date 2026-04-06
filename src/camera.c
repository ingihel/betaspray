#include "camera.h"
#include "conf.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "camera";

static camera_config_t s_cfg = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000, // Specify 24MHz fall back clock (OV5640 uses internal 24 MHz, seems
                              // good enough for now and probably safest)
    // LEDC_TIMER_1 / LEDC_CHANNEL_7 - avoids servo's LEDC_TIMER_0 / CH0-7.
    // Not actually used when CAM_PIN_XCLK == -1 but this should be safe
    .ledc_timer = LEDC_TIMER_1,
    .ledc_channel = LEDC_CHANNEL_7,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,               // 0-63; lower = better quality / larger file
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM, // DRAM insufficient after WiFi — must use PSRAM
    .grab_mode = CAMERA_GRAB_LATEST,
};

static bool s_initialized = false;

void camera_reset(void) {
    if (CAM_PIN_RESET == -1) {
        ESP_LOGI(TAG, "Reset pin not connected (NC), skipping reset");
        return;
    }

#if CAM_PIN_RESET == -1
#else
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CAM_PIN_RESET),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
#endif

    ESP_LOGI(TAG, "Resetting OV5640 via GPIO%d", CAM_PIN_RESET);
    gpio_set_level(CAM_PIN_RESET, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(CAM_PIN_RESET, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Reset complete");
}

esp_err_t camera_init(void) {
    if (s_initialized) {
        ESP_LOGW(TAG, "camera: oops, already initialized!");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Initializing OV5640 - DVP parallel, DMA via LCD_CAM");
    ESP_LOGI(TAG, "  D0-D7 : GPIO%d-%d/%d/%d/%d/%d/%d/%d", CAM_PIN_D0, CAM_PIN_D1, CAM_PIN_D2,
             CAM_PIN_D3, CAM_PIN_D4, CAM_PIN_D5, CAM_PIN_D6, CAM_PIN_D7);
    ESP_LOGI(TAG, "  VSYNC/HREF/PCLK : GPIO%d/%d/%d", CAM_PIN_VSYNC, CAM_PIN_HREF, CAM_PIN_PCLK);
    ESP_LOGI(TAG, "  SIOD/SIOC : GPIO%d/%d  PWDN/RST : GPIO%d/%d", CAM_PIN_SIOD, CAM_PIN_SIOC,
             CAM_PIN_PWDN, CAM_PIN_RESET);

    esp_err_t err = esp_camera_init(&s_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_camera_init failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;

    // Discard first few frames — OV5640 outputs garbage until the sensor stabilizes.
    for (int i = 0; i < 3; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "Camera ready - framesize=%d format=%d fb_count=%d loc=%s", s_cfg.frame_size,
             s_cfg.pixel_format, s_cfg.fb_count,
             s_cfg.fb_location == CAMERA_FB_IN_PSRAM ? "PSRAM" : "DRAM");
    ESP_LOGI(TAG, "Calibration %dx%d  fx=%.2f fy=%.2f  cx=%.2f cy=%.2f", CALIB_WIDTH, CALIB_HEIGHT,
             CALIB_FX, CALIB_FY, CALIB_CX, CALIB_CY);
    ESP_LOGI(TAG, "  dist k1=%.4f k2=%.4f p1=%.4f p2=%.4f k3=%.4f", CALIB_K1, CALIB_K2, CALIB_P1,
             CALIB_P2, CALIB_K3);
    return ESP_OK;
}

camera_fb_t *camera_capture_frame(void) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Frame capture failed");
        return NULL;
    }
    ESP_LOGI(TAG, "got frame: %u bytes  %ux%u  fmt=%d", (unsigned)fb->len, fb->width, fb->height,
             fb->format);
    return fb;
}

void camera_release_frame(camera_fb_t *fb) {
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

// TODO: add s_cfg->set_fb_location

void camera_set_resolution(framesize_t size) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "set_resolution: sensor not available");
        return;
    }
    s->set_framesize(s, size);
    s_cfg.frame_size = size;
    ESP_LOGI(TAG, "Resolution -> %d", size);
}

void camera_set_format(pixformat_t fmt) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "set_format: sensor not available");
        return;
    }
    s->set_pixformat(s, fmt);
    s_cfg.pixel_format = fmt;
    ESP_LOGI(TAG, "Format -> %d", fmt);
}

void camera_deinit(void) {
    if (!s_initialized) {
        ESP_LOGW(TAG, "Camera not initialized");
        return;
    }
    esp_camera_deinit();
    s_initialized = false;
    ESP_LOGI(TAG, "Camera deinitialized");
}

bool camera_is_initialized(void) { return s_initialized; }
