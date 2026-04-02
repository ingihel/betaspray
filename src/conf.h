#pragma once

// TODO(Max): use calibration figures in the loop

// ---------------------------------------------------------------------------
// Hardware target selection
// ---------------------------------------------------------------------------
// Set to 1 when using the soldered PCB (ESP32-S3-WROOM-1-N16R8, 8 MB PSRAM).
// Set to 0 for the dev board (ESP32-S3-WROOM-1-N8, DRAM only).
//
// PSRAM=1: camera inits at UXGA (1600x1200), full frame captured in one shot.
// PSRAM=0: camera inits at QVGA (320x240) in DRAM; use /capture/tile for
//          high-res coverage via OV5640 sensor-window tiling.
// ---------------------------------------------------------------------------
#define CAMERA_USE_PSRAM 0

// Tiled capture grid (CAMERA_USE_PSRAM == 0 only).
//
// Each tile is one QVGA (CALIB_WIDTH x CALIB_HEIGHT) frame captured from a
// different crop window on the OV5640 array, stitched on the host.
// Stitched image size: (CALIB_WIDTH * TILE_COLS) x (CALIB_HEIGHT * TILE_ROWS)
//
// MAX TILE COUNT is constrained by the sensor's 2x subsampling mode (QVGA):
//   register 0x3814/0x3815 = 0x31 → every-other-pixel readout (2x H and V).
//   The ISP requires at least CALIB_WIDTH * 2 = 640 input pixels per tile row
//   to down-scale to the 320-pixel DVP output. Narrower crops produce an
//   upscale the ISP cannot perform → sensor stalls, no frames generated.
//   Max cols = OV5640_ACTIVE_W / (CALIB_WIDTH * 2) = 2592 / 640 = 4 (648 px/tile)
//   Max rows = OV5640_ACTIVE_H / (CALIB_HEIGHT * 2) = 1944 / 480 = 4 (486 px/tile)
//   Stitched output at 4x4: 1280x960.
#define CAMERA_TILE_COLS 4
#define CAMERA_TILE_ROWS 4

// OV5640 active pixel array dimensions (per datasheet §2.1)
#define OV5640_ACTIVE_W 2592
#define OV5640_ACTIVE_H 1944

// Hardware offset of the pixel array origin in register space.
// If corner tiles show black borders, increment these by 8–16 until they clear.
#define OV5640_ARRAY_X_OFFSET 0
#define OV5640_ARRAY_Y_OFFSET 0

/* Camera intrinsic calibration — OV5640, 320x240
 * Calibrated with calibrate_camera.py, RMS reprojection error: 0.7909 px */

#define CALIB_WIDTH 320
#define CALIB_HEIGHT 240
#define CALIB_FX 143.755937f
#define CALIB_FY 141.176552f
#define CALIB_CX 156.018487f
#define CALIB_CY 122.142135f
#define CALIB_K1 -0.237602f
#define CALIB_K2 0.238395f
#define CALIB_P1 0.007217f
#define CALIB_P2 0.001089f
#define CALIB_K3 -0.113216f

/* Scaling K for a different resolution:
 *   scale    = new_width / CALIB_WIDTH
 *   fx_new   = CALIB_FX * scale
 *   fy_new   = CALIB_FY * scale
 *   cx_new   = CALIB_CX * scale
 *   cy_new   = CALIB_CY * scale
 *   dist coefficients (K1–K3, P1–P2) stay the same */
