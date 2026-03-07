#pragma once

#include "esp_camera.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Adafruit's OV5640 breakout board labels the bus as D2-D9
// (D2 = LSB, D9 = MSB). RT and XC are noConn right as-is: the Adafruit board
// has its own 24 MHz oscillator so xtal clock not necessary

#define CAM_PIN_PWDN 15  // PD
#define CAM_PIN_RESET -1 // RT is noConn
#define CAM_PIN_XCLK -1  // XC is noConn
#define CAM_PIN_SIOD 4   // SDA
#define CAM_PIN_SIOC 5   // SCL
#define CAM_PIN_D7 16    // D9 (MSB)
#define CAM_PIN_D6 17    // D8
#define CAM_PIN_D5 18    // D7
#define CAM_PIN_D4 12    // D6
#define CAM_PIN_D3 10    // D5
#define CAM_PIN_D2 8     // D4
#define CAM_PIN_D1 9     // D3
#define CAM_PIN_D0 11    // D2 (LSB)
#define CAM_PIN_VSYNC 6  // VS
#define CAM_PIN_HREF 7   // HS
#define CAM_PIN_PCLK 13  // PC

// The esp32-camera component should ignore xclk_freq_hz when XCLK=-1

// SCCB (I2C) clock: accepted by camera_config() but cannot be applied at
// runtime - camera_config_t does not expose sccb_freq_hz. The component
// defaults to 100 kHz internally.
#define CAM_SCCB_FREQ_HZ 100000 // 100 kHz - informational only

#define CAMERA_FORMAT_JPEG PIXFORMAT_JPEG
#define CAMERA_FORMAT_RGB565 PIXFORMAT_RGB565
#define CAMERA_FORMAT_GRAYSCALE PIXFORMAT_GRAYSCALE

typedef enum {
    CAMERA_MEM_BSS,      // Static buffer in .bss - no alloc, fixed 40 KB cap
    CAMERA_MEM_DATA_RAM, // heap malloc from internal DRAM - caller must free()
    CAMERA_MEM_PSRAM,    // heap malloc from PSRAM via MALLOC_CAP_SPIRAM - caller
                         // must free()
    CAMERA_MEM_STACK,    // Caller pre-allocates; *out_buf must point to buffer on
                         // entry
} camera_mem_t;

// Size of the internal BSS capture buffer. Must be >= largest expected frame.
// QVGA JPEG worst case ~40 KB; increase if using higher resolutions.
#define CAMERA_BSS_BUF_SIZE (40 * 1024)

// Hardware-reset the OV5640 by toggling RESET low for 10 ms.
// Call before camera_init() if the sensor is in an unknown state.
void camera_reset(void);

// Initialize the camera with default config:
//   QVGA (320x240), JPEG, 2 frame buffers in PSRAM, DMA via LCD_CAM.
// DMA is enabled automatically by the esp32-camera component.
esp_err_t camera_init(void);

// Reconfigure resolution, pixel format, and SCCB clock, then re-init.
// sccb_freq_hz is accepted but cannot be applied at runtime - camera_config_t
// does not expose this field; the component defaults to 100 kHz internally.
// XCLK is not a parameter: the OV5640 uses its internal 24 MHz oscillator.
esp_err_t camera_config(framesize_t res, pixformat_t fmt, int sccb_freq_hz);

// Capture a frame into the esp32-camera DMA-managed buffer.
// Must be released with camera_return_frame() - do not hold long.
camera_fb_t *camera_capture_frame(void);
void camera_return_frame(camera_fb_t *fb);

// Capture a frame and copy it into the specified memory region.
//
//   CAMERA_MEM_BSS      *out_buf is set to the internal static buffer.
//                        Not thread-safe; overwritten on next call.
//   CAMERA_MEM_DATA_RAM *out_buf is set to a heap_malloc'd block. Caller frees.
//   CAMERA_MEM_PSRAM    *out_buf is set to a PSRAM-malloc'd block. Caller
//   frees. CAMERA_MEM_STACK    *out_buf must already point to a buffer of
//   sufficient
//                        size. The frame is copied in; nothing is allocated.
//
// *out_len receives the number of bytes written.
esp_err_t camera_click_pic(camera_mem_t dest, uint8_t **out_buf, size_t *out_len);

// Change resolution or format live via OV5640 sensor register writes.
// No re-init required - takes effect on the next captured frame.
void camera_set_resolution(framesize_t size);
void camera_set_format(pixformat_t fmt);

// Deinitialize the camera and release resources.
void camera_deinit(void);

// Check if camera is currently initialized.
bool camera_is_initialized(void);
