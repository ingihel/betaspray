#pragma once

#include "esp_err.h"
#include <stdbool.h>

#define ROUTE_MAX_HOLDS 64
#define ROUTE_FILE_PATH "/fatfs/route%d.xy"
#define GIMBAL_POLY_FILE "/fatfs/gimcal.bin"

typedef enum {
    ROUTE_MODE_SEQUENTIAL = 0, // all gimbals advance together each step (default)
    ROUTE_MODE_LEAPFROG,       // gimbals advance one at a time in rotation
} route_play_mode_t;

typedef struct {
    float x;
    float y;
} route_hold_t;

// Camera and distance parameters for geometric pixel-to-angle conversion
typedef struct {
    // Camera optics (degrees)
    float hfov_deg;                // horizontal field of view
    float vfov_deg;                // vertical field of view

    // Camera resolution (pixels)
    int image_width;
    int image_height;

    // Distance to wall (meters)
    float distance_m;
} route_transform_t;

// File management
// Note: route_create_from_json writes directly without pre-buffering all holds
esp_err_t route_create(int n, const route_hold_t *holds, int num_holds);
esp_err_t route_create_streaming(int n); // For streaming writes (advanced)
esp_err_t route_delete(int n);
esp_err_t route_load(int n);

// Playback control (called from HTTP handlers)
void route_play(void);
void route_pause(void);
void route_next(void);
void route_restart(void);
void route_stop(void);  // pause + restart atomically

// Set timed auto-advance interval in ms. 0 = manual (wait for route_next).
void route_set_timed_interval(int ms);

// Set playback mode and number of active gimbals for leapfrog.
// leap_num is ignored in SEQUENTIAL mode.
// leap_num must be 2 or 4 for LEAPFROG mode (capped to NUM_SERVOS/2).
void route_set_mode(route_play_mode_t mode, int leap_num);

// Transform config
void route_set_transform(const route_transform_t *t);
void route_set_distance(float distance_m);

// Per-gimbal pointing correction applied on top of pixel→angle conversion.
// dx/dy are signed degree offsets clamped so final angle stays in [0,180].
void route_set_gimbal_offset(int gimbal, int dx, int dy);

// Set the physical XYZ position of a gimbal relative to the camera (meters).
// +X = right, +Y = behind camera, +Z = up.
void route_set_gimbal_position(int gimbal, float x, float y, float z);

// Per-gimbal bilinear polynomial correction (8 floats per gimbal):
//   coeffs[0..3] = a0,a1,a2,a3 for X:  corrected_x = raw_x + a0 + a1*rx + a2*ry + a3*rx*ry
//   coeffs[4..7] = b0,b1,b2,b3 for Y:  corrected_y = raw_y + b0 + b1*rx + b2*ry + b3*rx*ry
void route_set_gimbal_poly(int gimbal, const float coeffs[8]);
void route_get_gimbal_poly(int gimbal, float coeffs[8]);
esp_err_t route_save_gimbal_poly(void);
esp_err_t route_load_gimbal_poly(void);

// Compute raw (uncorrected) servo angles for a pixel + gimbal.
// Used by calibration so computed angles match route playback exactly.
void route_compute_angles(float px, float py, int gimbal, int *out_x, int *out_y);

// Init — call once from app_main; starts playback FreeRTOS task
void route_init(void);
