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

// META(Ingi): do we actually anticipate needing this? What is the business logic
// to initiate this?
//
// Hardware-reset the OV5640 by toggling RESET low for 10 ms.
// Call before camera_init() if the sensor is in an unknown state.
void camera_reset(void);

// Initialize the camera with default config:
//   QVGA (320x240), JPEG, DMA via LCD_CAM.
//
// NOTE: frame buffer location is determined by CAMERA_FB_IN_PSRAM macro
//
// NOTE: the esp32-camera library handles DMA object registry.
esp_err_t camera_init(void);

// Capture a frame into the esp32-camera DMA-managed buffer.
// Must be released with camera_release_frame(), quickly is better.
camera_fb_t *camera_capture_frame(void);

// Release esp32-camera DMA buffer.
// MAKE SURE THIS IS BEING CALLED.
void camera_release_frame(camera_fb_t *fb);

// Change resolution of the OV5640 sensor.
// This can be updated live, without re-initialization: the next
// frame shall have the desired settings.
void camera_set_resolution(framesize_t size);

// Change image format of the OV5640 sensor.
// This can be updated live, without re-initialization: the next
// frame shall have the desired settings.
void camera_set_format(pixformat_t fmt);

// Deinitialize the camera and release resources.
void camera_deinit(void);

// Check if camera is currently initialized.
bool camera_is_initialized(void);
