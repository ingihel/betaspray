#pragma once

#include "esp_camera.h"
#include "esp_err.h"

// --- Pin Configuration ---
// Wired per schematic. OV5640 breakout labels data pins D2–D9
// (D2 = LSB, D9 = MSB). RT and XC are NC — the Adafruit board
// has its own 24 MHz oscillator so XCLK from the host is not needed.
#define CAM_PIN_PWDN    15   // PD
#define CAM_PIN_RESET   -1   // RT → NC
#define CAM_PIN_XCLK    -1   // XC → NC (onboard oscillator)
#define CAM_PIN_SIOD    4   // SDA
#define CAM_PIN_SIOC    5   // SCL
#define CAM_PIN_D7      16   // D9 (MSB)
#define CAM_PIN_D6      17   // D8
#define CAM_PIN_D5      18   // D7
#define CAM_PIN_D4      12   // D6
#define CAM_PIN_D3      10   // D5
#define CAM_PIN_D2       8   // D4
#define CAM_PIN_D1       9   // D3
#define CAM_PIN_D0      11   // D2 (LSB)
#define CAM_PIN_VSYNC    6   // VS
#define CAM_PIN_HREF     7   // HS
#define CAM_PIN_PCLK    13   // PC

// --- Default Camera Settings ---
// XCLK is 0 — host does not generate the clock (onboard oscillator used).
#define CAM_XCLK_FREQ_HZ  0           // Onboard 24 MHz oscillator
#define CAM_JPEG_QUALITY  12        // 0–63, lower = better quality
#define CAM_FB_COUNT      2         // Frame buffers (2 for double-buffering)

// Initialize the OV5640 camera. Call once at startup after NVS init.
esp_err_t camera_init(void);

// Deinitialize the camera and free resources.
esp_err_t camera_deinit(void);

// Capture a single frame. Returns a pointer to the frame buffer,
// or NULL on failure. You MUST call camera_return_frame() when done.
camera_fb_t *camera_capture_frame(void);

// Return a frame buffer obtained from camera_capture_frame().
void camera_return_frame(camera_fb_t *fb);

// Change the capture resolution at runtime.
// See esp_camera.h for FRAMESIZE_* constants (e.g. FRAMESIZE_VGA).
esp_err_t camera_set_resolution(framesize_t size);

// Change the pixel format at runtime.
// Common values: PIXFORMAT_JPEG, PIXFORMAT_RGB565, PIXFORMAT_YUV422
esp_err_t camera_set_pixel_format(pixformat_t format);

// Simple format selector — use this instead of camera_set_pixel_format()
// when you just want to switch between compressed and uncompressed output.
typedef enum {
    CAMERA_FORMAT_JPEG,   // Compressed JPEG — smaller, good for streaming/storage
    CAMERA_FORMAT_RAW,    // Uncompressed RGB565 — larger, good for image processing
} camera_format_t;

// Switch between JPEG and RAW (RGB565) output.
esp_err_t camera_set_format(camera_format_t format);

// Get a handle to the sensor so you can tweak brightness, contrast, etc.
// Returns NULL if the camera is not initialized.
sensor_t *camera_get_sensor(void);
