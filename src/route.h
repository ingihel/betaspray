#pragma once

#include "esp_err.h"
#include <stdbool.h>

#define ROUTE_MAX_HOLDS 64
#define ROUTE_FILE_PATH "/fatfs/route%d.xy"

typedef struct {
    float x;
    float y;
} route_hold_t;

// Configurable pixel-to-angle linear transform per axis
typedef struct {
    float x_scale; // angle = x * x_scale + x_offset
    float x_offset;
    float y_scale;
    float y_offset;
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

// Transform config
void route_set_transform(const route_transform_t *t);

// Init — call once from app_main; starts playback FreeRTOS task
void route_init(void);
