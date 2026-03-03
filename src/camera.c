#include "camera.h"
#include "esp_log.h"

static const char *TAG = "camera";

esp_err_t camera_init(void) {
  camera_config_t config = {
      .pin_pwdn     = CAM_PIN_PWDN,
      .pin_reset    = CAM_PIN_RESET,
      .pin_xclk     = CAM_PIN_XCLK,
      .pin_sccb_sda = CAM_PIN_SIOD,
      .pin_sccb_scl = CAM_PIN_SIOC,
      .pin_d7       = CAM_PIN_D7,
      .pin_d6       = CAM_PIN_D6,
      .pin_d5       = CAM_PIN_D5,
      .pin_d4       = CAM_PIN_D4,
      .pin_d3       = CAM_PIN_D3,
      .pin_d2       = CAM_PIN_D2,
      .pin_d1       = CAM_PIN_D1,
      .pin_d0       = CAM_PIN_D0,
      .pin_vsync    = CAM_PIN_VSYNC,
      .pin_href     = CAM_PIN_HREF,
      .pin_pclk     = CAM_PIN_PCLK,

      .xclk_freq_hz = CAM_XCLK_FREQ_HZ,  // 0 = skip LEDC clock generation
      .ledc_timer   = LEDC_TIMER_0,       // unused when xclk_freq_hz = 0
      .ledc_channel = LEDC_CHANNEL_0,     // unused when xclk_freq_hz = 0

      .pixel_format = PIXFORMAT_JPEG,
      .frame_size   = FRAMESIZE_VGA,   // 640x480 — good default
      .jpeg_quality = CAM_JPEG_QUALITY,
      .fb_count     = CAM_FB_COUNT,
      .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
  };

  esp_err_t ret = esp_camera_init(&config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed: 0x%x", ret);
    return ret;
  }

  // Confirm we found an OV5640
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    ESP_LOGE(TAG, "Failed to get sensor handle");
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "Camera init OK — sensor PID: 0x%04x", sensor->id.PID);

  return ESP_OK;
}

esp_err_t camera_deinit(void) {
  esp_err_t ret = esp_camera_deinit();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Camera deinit failed: 0x%x", ret);
  }
  return ret;
}

camera_fb_t *camera_capture_frame(void) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == NULL) {
    ESP_LOGE(TAG, "Frame capture failed");
    return NULL;
  }
  ESP_LOGD(TAG, "Captured frame: %zu bytes (%dx%d)", fb->len, fb->width, fb->height);
  return fb;
}

void camera_return_frame(camera_fb_t *fb) {
  if (fb != NULL) {
    esp_camera_fb_return(fb);
  }
}

esp_err_t camera_set_resolution(framesize_t size) {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  int ret = sensor->set_framesize(sensor, size);
  return (ret == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t camera_set_pixel_format(pixformat_t format) {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  int ret = sensor->set_pixformat(sensor, format);
  return (ret == 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t camera_set_format(camera_format_t format) {
  pixformat_t pf;
  const char *name;

  switch (format) {
    case CAMERA_FORMAT_JPEG:
      pf   = PIXFORMAT_JPEG;
      name = "JPEG";
      break;
    case CAMERA_FORMAT_RAW:
      pf   = PIXFORMAT_RGB565;
      name = "RAW (RGB565)";
      break;
    default:
      ESP_LOGE(TAG, "Unknown camera format %d", format);
      return ESP_ERR_INVALID_ARG;
  }

  esp_err_t ret = camera_set_pixel_format(pf);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Format set to %s", name);
  }
  return ret;
}

sensor_t *camera_get_sensor(void) {
  return esp_camera_sensor_get();
}
