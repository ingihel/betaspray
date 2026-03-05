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
    .fb_count = 1,                    // single buffer to save memory
    .fb_location = CAMERA_FB_IN_DRAM, // Use internal DRAM (PSRAM alloc failing)
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

static bool s_initialized = false;

// Internal BSS capture buffer - BSS is zero-initialised, hence no heap allocation needed
static uint8_t s_bss_buf[CAMERA_BSS_BUF_SIZE];

void camera_reset(void) {
    if (CAM_PIN_RESET == -1) {
        ESP_LOGI(TAG, "Reset pin not connected (NC), skipping reset");
        return;
    }

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CAM_PIN_RESET),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    ESP_LOGI(TAG, "Resetting OV5640 via GPIO%d", CAM_PIN_RESET);
    gpio_set_level(CAM_PIN_RESET, 0);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    gpio_set_level(CAM_PIN_RESET, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Reset complete");
}

esp_err_t camera_init(void) {
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
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
    ESP_LOGI(TAG, "Camera ready - framesize=%d format=%d fb_count=%d loc=%s", s_cfg.frame_size,
             s_cfg.pixel_format, s_cfg.fb_count,
             s_cfg.fb_location == CAMERA_FB_IN_PSRAM ? "PSRAM" : "DRAM");
    ESP_LOGI(TAG, "Calibration %dx%d  fx=%.2f fy=%.2f  cx=%.2f cy=%.2f",
             CALIB_WIDTH, CALIB_HEIGHT, CALIB_FX, CALIB_FY, CALIB_CX, CALIB_CY);
    ESP_LOGI(TAG, "  dist k1=%.4f k2=%.4f p1=%.4f p2=%.4f k3=%.4f",
             CALIB_K1, CALIB_K2, CALIB_P1, CALIB_P2, CALIB_K3);
    return ESP_OK;
}

esp_err_t camera_config(framesize_t res, pixformat_t fmt, int sccb_freq_hz) {
    // sccb_freq_hz is accepted but not applied - camera_config_t does not
    // expose this field. The component defaults to 100 kHz internally.
    if (sccb_freq_hz != CAM_SCCB_FREQ_HZ) {
        ESP_LOGW(TAG,
                 "sccb_freq_hz=%d requested but not applied "
                 "(not supported by camera_config_t)",
                 sccb_freq_hz);
    }

    if (s_initialized) {
        esp_camera_deinit();
        s_initialized = false;
    }

    s_cfg.frame_size = res;
    s_cfg.pixel_format = fmt;

    ESP_LOGI(TAG, "Reconfiguring - res=%d fmt=%d sccb=%d Hz (internal clk)", res, fmt,
             sccb_freq_hz);
    return camera_init();
}

camera_fb_t *camera_capture_frame(void) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Frame capture failed");
        return NULL;
    }
    ESP_LOGI(TAG, "Frame: %u bytes  %ux%u  fmt=%d", (unsigned)fb->len, fb->width, fb->height,
             fb->format);
    return fb;
}

void camera_return_frame(camera_fb_t *fb) {
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

esp_err_t camera_click_pic(camera_mem_t dest, uint8_t **out_buf, size_t *out_len) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "click_pic: frame capture failed");
        return ESP_FAIL;
    }

    esp_err_t err = ESP_OK;

    switch (dest) {
    /* Support possible destinations for DVP framebuffer */
    // NOTE: this is a good idea to sanity check.
    case CAMERA_MEM_BSS:
        if (fb->len > CAMERA_BSS_BUF_SIZE) {
            ESP_LOGE(TAG, "click_pic: frame %u B > BSS buf %u B", (unsigned)fb->len,
                     (unsigned)CAMERA_BSS_BUF_SIZE);
            err = ESP_ERR_NO_MEM;
            break;
        }
        memcpy(s_bss_buf, fb->buf, fb->len);
        *out_buf = s_bss_buf;
        *out_len = fb->len;
        ESP_LOGI(TAG, "click_pic: %u B -> BSS", (unsigned)fb->len);
        break;

    case CAMERA_MEM_DATA_RAM:
        *out_buf = malloc(fb->len);
        if (!*out_buf) {
            ESP_LOGE(TAG, "click_pic: malloc(%u) failed (DATA_RAM)", (unsigned)fb->len);
            err = ESP_ERR_NO_MEM;
            break;
        }
        memcpy(*out_buf, fb->buf, fb->len);
        *out_len = fb->len;
        ESP_LOGI(TAG, "click_pic: %u B -> DATA_RAM @ %p", (unsigned)fb->len, *out_buf);
        break;

    case CAMERA_MEM_PSRAM:
        *out_buf = heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM);
        if (!*out_buf) {
            ESP_LOGE(TAG, "click_pic: heap_caps_malloc(%u) failed (PSRAM)", (unsigned)fb->len);
            err = ESP_ERR_NO_MEM;
            break;
        }
        memcpy(*out_buf, fb->buf, fb->len);
        *out_len = fb->len;
        ESP_LOGI(TAG, "click_pic: %u B -> PSRAM @ %p", (unsigned)fb->len, *out_buf);
        break;

    case CAMERA_MEM_STACK:
        if (!*out_buf) {
            ESP_LOGE(TAG, "click_pic: STACK dest requires *out_buf != NULL");
            err = ESP_ERR_INVALID_ARG;
            break;
        }
        memcpy(*out_buf, fb->buf, fb->len);
        *out_len = fb->len;
        ESP_LOGI(TAG, "click_pic: %u B -> STACK @ %p", (unsigned)fb->len, *out_buf);
        break;
    }

    esp_camera_fb_return(fb);
    return err;
}

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
