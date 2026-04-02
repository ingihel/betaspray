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
#if CAMERA_USE_PSRAM
    // PCB: N16R8 — 8 MB PSRAM available, full UXGA single-shot capture
    .frame_size  = FRAMESIZE_UXGA,        // 1600x1200
    .jpeg_quality = 10,                   // higher quality; PSRAM has headroom
    .fb_count    = 2,                     // double-buffer for grab latency
    .fb_location = CAMERA_FB_IN_PSRAM,
#else
    // Dev board: N8 — DRAM only.
    // OV5640 hardware JPEG encoder compresses large frames before DMA transfer,
    // so the fb only holds the compressed JPEG bitstream (~30-80 KB for SVGA),
    // not a raw bitmap. SVGA at quality 12 fits comfortably in DRAM.
    //
    // NOTE: Tiled high-res capture via /capture/tile is NOT supported.
    // In all subsampled modes the OV5640 reads the full 2592x1944 sensor array
    // and subsamples it — you cannot select a sub-region over DVP.
    // Changing only the crop window registers (0x3800-0x3807) without also
    // updating HTS/VTS/DVPHO/DVPVO/ISP-scale stalls the sensor (no VSYNC).
    // High-res on the dev board is achieved here via JPEG compression instead.
    .frame_size  = FRAMESIZE_SVGA,        // 800x600
    .jpeg_quality = 12,                   // 0-63; lower = better quality / larger file
    .fb_count    = 1,                     // single buffer to conserve DRAM
    .fb_location = CAMERA_FB_IN_DRAM,
#endif
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
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

esp_err_t camera_set_window(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "set_window: sensor unavailable");
        return ESP_ERR_INVALID_STATE;
    }

    // Clamp to active array bounds
    if ((uint32_t)x + w > OV5640_ACTIVE_W)
        w = OV5640_ACTIVE_W - x;
    if ((uint32_t)y + h > OV5640_ACTIVE_H)
        h = OV5640_ACTIVE_H - y;

    uint16_t xs = x + OV5640_ARRAY_X_OFFSET;
    uint16_t ys = y + OV5640_ARRAY_Y_OFFSET;
    uint16_t xe = xs + w - 1;
    uint16_t ye = ys + h - 1;

    // Sensor crop window — OV5640 timing registers (§12.2 of datasheet)
    s->set_reg(s, 0x3800, 0x0F, (xs >> 8) & 0x0F); // TIMING_HS[11:8]
    s->set_reg(s, 0x3801, 0xFF, xs & 0xFF);          // TIMING_HS[7:0]
    s->set_reg(s, 0x3802, 0x07, (ys >> 8) & 0x07);  // TIMING_VS[10:8]
    s->set_reg(s, 0x3803, 0xFF, ys & 0xFF);          // TIMING_VS[7:0]
    s->set_reg(s, 0x3804, 0x0F, (xe >> 8) & 0x0F);  // TIMING_HW[11:8] (H end)
    s->set_reg(s, 0x3805, 0xFF, xe & 0xFF);          // TIMING_HW[7:0]
    s->set_reg(s, 0x3806, 0x07, (ye >> 8) & 0x07);  // TIMING_VH[10:8] (V end)
    s->set_reg(s, 0x3807, 0xFF, ye & 0xFF);          // TIMING_VH[7:0]

    // DVP output size is intentionally NOT changed here — the DMA buffer is
    // allocated at esp_camera_init() time for the configured framesize (QVGA
    // on DRAM builds) and cannot be resized at runtime. The ISP scales the
    // crop window to the fixed output size (~1% scale for 324→320 / 243→240).

    // Subsampling registers (0x3814, 0x3815) and binning bits (0x3820, 0x3821)
    // are intentionally NOT changed. Altering them at runtime changes the DVP
    // pixel-clock cadence; the LCD_CAM DMA was configured at esp_camera_init()
    // for a specific clock count per line and loses sync on the next frame.
    // The ISP scales whatever crop window we set here down to the fixed
    // DVPHO x DVPVO output size, so the DMA never sees a timing change.
    //
    // NOTE: AE/AWB will re-converge for each tile window. For consistent
    // exposure lock AE before the first tile and unlock after the last.

    ESP_LOGI(TAG, "set_window (%u,%u)+%ux%u → sensor regs xs=%u xe=%u ys=%u ye=%u",
             x, y, w, h, xs, xe, ys, ye);
    return ESP_OK;
}

void camera_reset_window(void) {
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        ESP_LOGE(TAG, "reset_window: sensor unavailable");
        return;
    }
    // Re-apply all timing registers for the configured framesize, then wait for
    // the sensor pipeline to stabilize before any subsequent set_window call.
    // Without this delay the next set_window writes timing regs while the
    // sensor is still transitioning, leaving it in an inconsistent state that
    // stalls DVP sync and causes esp_camera_fb_get() to time out.
    s->set_framesize(s, s_cfg.frame_size);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "reset_window → framesize %d", s_cfg.frame_size);
}
