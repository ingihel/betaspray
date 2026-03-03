#pragma once

#include "esp_camera.h"
#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

// ── DVP GPIO pin assignments ───────────────────────────────────────────────
// Verify against KiCad schematic before flashing.
// GPIO39–41 are JTAG pins repurposed as GPIO; hardware JTAG unavailable.
// GPIO4–11 reserved for servos. GPIO43/44 reserved for UART0.

#define CAM_PIN_PWDN    39  // Power down — active high (JTAG MTCK)
#define CAM_PIN_RESET   40  // Reset — active low      (JTAG MTDO)
#define CAM_PIN_XCLK    -1  // External clock — OV5640 uses internal 24 MHz osc
#define CAM_PIN_SDA      1  // SCCB / I²C data
#define CAM_PIN_SCL     41  // SCCB / I²C clock        (JTAG MTDI)

// DVP parallel data bus (D0 = LSB)
#define CAM_PIN_D0      12
#define CAM_PIN_D1      13
#define CAM_PIN_D2      14
#define CAM_PIN_D3      15
#define CAM_PIN_D4      16
#define CAM_PIN_D5      17
#define CAM_PIN_D6      18
#define CAM_PIN_D7      21

// Sync signals
#define CAM_PIN_VSYNC   47
#define CAM_PIN_HREF    48
#define CAM_PIN_PCLK     2

// ── Clock defaults ─────────────────────────────────────────────────────────
// XCLK pin is -1: the OV5640 runs off its internal 24 MHz oscillator.
// The esp32-camera component ignores xclk_freq_hz when the pin is -1.
//
// SCCB (I²C) clock: accepted by camera_config() but cannot be applied at
// runtime — camera_config_t does not expose sccb_freq_hz. The component
// defaults to 100 kHz internally.
#define CAM_SCCB_FREQ_HZ   100000     // 100 kHz — informational only

// ── Format aliases (thin wrappers over esp32-camera pixformat_t) ───────────
#define CAMERA_FORMAT_JPEG       PIXFORMAT_JPEG
#define CAMERA_FORMAT_RGB565     PIXFORMAT_RGB565
#define CAMERA_FORMAT_GRAYSCALE  PIXFORMAT_GRAYSCALE

// ── Memory destinations for camera_click_pic() ────────────────────────────
typedef enum {
    CAMERA_MEM_BSS,       // Static buffer in .bss — no alloc, fixed 40 KB cap
    CAMERA_MEM_DATA_RAM,  // heap malloc from internal DRAM — caller must free()
    CAMERA_MEM_PSRAM,     // heap malloc from PSRAM via MALLOC_CAP_SPIRAM — caller must free()
    CAMERA_MEM_STACK,     // Caller pre-allocates; *out_buf must point to buffer on entry
} camera_mem_t;

// Size of the internal BSS capture buffer. Must be >= largest expected frame.
// QVGA JPEG worst case ~40 KB; increase if using higher resolutions.
#define CAMERA_BSS_BUF_SIZE (40 * 1024)

// ── API ────────────────────────────────────────────────────────────────────

// Hardware-reset the OV5640 by toggling RESET low for 10 ms.
// Call before camera_init() if the sensor is in an unknown state.
void camera_reset(void);

// Initialize the camera with default config:
//   QVGA (320x240), JPEG, 2 frame buffers in PSRAM, DMA via LCD_CAM.
// DMA is enabled automatically by the esp32-camera component.
esp_err_t camera_init(void);

// Reconfigure resolution, pixel format, and SCCB clock, then re-init.
// sccb_freq_hz is accepted but cannot be applied at runtime — camera_config_t
// does not expose this field; the component defaults to 100 kHz internally.
// XCLK is not a parameter: the OV5640 uses its internal 24 MHz oscillator.
esp_err_t camera_config(framesize_t res, pixformat_t fmt, int sccb_freq_hz);

// Capture a frame into the esp32-camera DMA-managed buffer.
// Must be released with camera_return_frame() — do not hold long.
camera_fb_t *camera_capture_frame(void);
void         camera_return_frame(camera_fb_t *fb);

// Capture a frame and copy it into the specified memory region.
//
//   CAMERA_MEM_BSS      *out_buf is set to the internal static buffer.
//                        Not thread-safe; overwritten on next call.
//   CAMERA_MEM_DATA_RAM *out_buf is set to a heap_malloc'd block. Caller frees.
//   CAMERA_MEM_PSRAM    *out_buf is set to a PSRAM-malloc'd block. Caller frees.
//   CAMERA_MEM_STACK    *out_buf must already point to a buffer of sufficient
//                        size. The frame is copied in; nothing is allocated.
//
// *out_len receives the number of bytes written.
esp_err_t camera_click_pic(camera_mem_t dest,
                           uint8_t **out_buf, size_t *out_len);

// Change resolution or format live via OV5640 sensor register writes.
// No re-init required — takes effect on the next captured frame.
void camera_set_resolution(framesize_t size);
void camera_set_format(pixformat_t fmt);
