#include "route.h"
#include "fatfs.h"
#include "servo.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "route";

#define ROUTE_AUTO_ADVANCE 1
#define ROUTE_DWELL_MS 3000

static route_hold_t s_holds[ROUTE_MAX_HOLDS];
static int s_num_holds = 0;
static volatile int s_current_hold = 0;
static volatile bool s_playing = false;
static SemaphoreHandle_t s_next_sem = NULL;
static SemaphoreHandle_t s_state_mutex = NULL;

static route_transform_t s_transform = {
    .hfov_deg = 120.0f,
    .vfov_deg = 60.0f,
    .image_width = 320,
    .image_height = 240,
    .distance_m = 3.0f,
};

static int pixel_to_servo_x(float px) {
    // Center coordinates: image center = 0°, edges = ±FOV/2
    float center_x = s_transform.image_width / 2.0f;
    float angle_deg = (px - center_x) * (s_transform.hfov_deg / s_transform.image_width);

    // Offset to servo range (0-180°, center at 90°)
    int servo_angle = (int)(angle_deg + 90.0f);
    return servo_angle < 0 ? 0 : servo_angle > 180 ? 180 : servo_angle;
}

static int pixel_to_servo_y(float py) {
    // Y axis: py=0 (top) maps to angle=FOV, py=240 (bottom) maps to angle=0
    float angle_deg = (s_transform.image_height - py) * (s_transform.vfov_deg / s_transform.image_height);

    // Servo range: 90° at bottom, 90+FOV at top
    int servo_angle = (int)(90.0f + angle_deg);
    return servo_angle < 0 ? 0 : servo_angle > 180 ? 180 : servo_angle;
}

esp_err_t route_create(int n, const route_hold_t *holds, int num_holds) {
    if (num_holds <= 0 || num_holds > ROUTE_MAX_HOLDS) {
        ESP_LOGE(TAG, "Invalid num_holds: %d", num_holds);
        return ESP_ERR_INVALID_ARG;
    }

    char path[64];
    snprintf(path, sizeof(path), ROUTE_FILE_PATH, n);

    ESP_LOGI(TAG, "[ROUTE_CREATE] Creating route %d with %d holds at '%s'", n, num_holds, path);

    // Binary format: [uint32 hold_count][float32 x0][float32 y0][float32 x1][float32 y1]...
    // Max size: 4 + (64 * 8) = 516 bytes (safe for stack)

    uint8_t buf[4 + ROUTE_MAX_HOLDS * 8];
    uint32_t *hold_count = (uint32_t *)buf;
    *hold_count = (uint32_t)num_holds;

    float *coords = (float *)(buf + 4);
    for (int i = 0; i < num_holds; i++) {
        coords[i * 2] = holds[i].x;
        coords[i * 2 + 1] = holds[i].y;
        ESP_LOGI(TAG, "[ROUTE_CREATE]   Hold %d: pixel(%.1f, %.1f)", i, holds[i].x, holds[i].y);
    }

    int total_size = 4 + (num_holds * 8);

    // Create and write file
    esp_err_t err = fatfs_create(path);
    if (err != ESP_OK) {
        return err;
    }

    err = fatfs_write(path, buf, 0, total_size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "[ROUTE_CREATE] Failed to write route file");
        return err;
    }

    ESP_LOGI(TAG, "[ROUTE_CREATE] SUCCESS: Route %d created with %d holds (%d bytes)", n, num_holds,
             total_size);
    return ESP_OK;
}

esp_err_t route_delete(int n) {
    char path[64];
    snprintf(path, sizeof(path), ROUTE_FILE_PATH, n);
    ESP_LOGI(TAG, "[ROUTE_DELETE] Deleting route %d from '%s'", n, path);
    esp_err_t err = fatfs_delete(path);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "[ROUTE_DELETE] SUCCESS: Route %d deleted", n);
    } else {
        ESP_LOGE(TAG, "[ROUTE_DELETE] Failed to delete route %d", n);
    }
    return err;
}

esp_err_t route_load(int n) {
    char path[64];
    snprintf(path, sizeof(path), ROUTE_FILE_PATH, n);

    ESP_LOGI(TAG, "[ROUTE_LOAD] Loading route %d from '%s'", n, path);

    // Read binary header (4 bytes: hold count)
    uint32_t hold_count = 0;
    size_t bytes_read = 0;
    esp_err_t err = fatfs_read(path, &hold_count, 0, sizeof(uint32_t), &bytes_read);
    if (err != ESP_OK || bytes_read != sizeof(uint32_t)) {
        ESP_LOGE(TAG, "[ROUTE_LOAD] Failed to read route header from '%s'", path);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_LOGI(TAG, "[ROUTE_LOAD] Header read: %u holds", (unsigned)hold_count);

    if (hold_count == 0 || hold_count > ROUTE_MAX_HOLDS) {
        ESP_LOGE(TAG, "[ROUTE_LOAD] Invalid hold count: %u (max: %d)", (unsigned)hold_count,
                 ROUTE_MAX_HOLDS);
        return ESP_FAIL;
    }

    // Read coordinates (hold_count * 8 bytes)
    uint8_t coords_buf[ROUTE_MAX_HOLDS * 8];
    int coords_size = hold_count * 8;
    err = fatfs_read(path, coords_buf, 4, coords_size, &bytes_read);
    if (err != ESP_OK || bytes_read != (size_t)coords_size) {
        ESP_LOGE(TAG, "[ROUTE_LOAD] Failed to read route data: expected %d bytes, got %u",
                 coords_size, (unsigned)bytes_read);
        return ESP_ERR_NOT_FOUND;
    }

    // Parse binary data
    float *coords = (float *)coords_buf;
    for (int i = 0; i < (int)hold_count; i++) {
        s_holds[i].x = coords[i * 2];
        s_holds[i].y = coords[i * 2 + 1];
        ESP_LOGI(TAG, "[ROUTE_LOAD]   Hold %d: pixel(%.1f, %.1f)", i, s_holds[i].x, s_holds[i].y);
    }

    s_num_holds = hold_count;
    s_current_hold = 0;
    ESP_LOGI(TAG, "[ROUTE_LOAD] SUCCESS: Loaded route %d with %u holds (%d bytes total)", n,
             (unsigned)hold_count, 4 + coords_size);
    return ESP_OK;
}

void route_play(void) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_playing = true;
    int current = s_current_hold;
    int total = s_num_holds;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "[PLAYBACK] PLAY: starting from hold %d/%d", current, total);

    // Log all holds in the route for trajectory reference
    ESP_LOGI(TAG, "[ROUTE] Total holds in route: %d", total);
    for (int i = 0; i < total && i < 10; i++) {
        ESP_LOGI(TAG, "[ROUTE] Hold %d: pixel(%.1f,%.1f)", i, s_holds[i].x, s_holds[i].y);
    }
    if (total > 10) {
        ESP_LOGI(TAG, "[ROUTE] ... and %d more holds", total - 10);
    }
}

void route_pause(void) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_playing = false;
    int current = s_current_hold;
    int total = s_num_holds;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "[PLAYBACK] PAUSE: halted at hold %d/%d", current, total);
}

void route_next(void) {
    xSemaphoreGive(s_next_sem);
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    int current = s_current_hold;
    int total = s_num_holds;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "[PLAYBACK] NEXT: advancing from hold %d/%d", current, total);
}

void route_restart(void) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_current_hold = 0;
    int total = s_num_holds;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "[PLAYBACK] RESTART: reset to hold 0/%d", total);
}

void route_set_transform(const route_transform_t *t) {
    if (t) {
        s_transform = *t;
        ESP_LOGI(TAG, "Transform set: camera=%.0f°H×%.0f°V %d×%d px, distance=%.2f m",
                 t->hfov_deg, t->vfov_deg, t->image_width, t->image_height, t->distance_m);
    }
}

void route_set_distance(float distance_m) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_transform.distance_m = distance_m;
    ESP_LOGI(TAG, "Distance set: %.2f m", distance_m);
    xSemaphoreGive(s_state_mutex);
}

static void route_playback_task(void *arg) {
    ESP_LOGI(TAG, "Playback task started");

    while (1) {
        // Check if playing
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        bool playing = s_playing;
        int current_hold = s_current_hold;
        int num_holds = s_num_holds;
        xSemaphoreGive(s_state_mutex);

        if (!playing) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        // Drive servos for current hold(s)
        int num_gimbals = NUM_SERVOS / 2;
        for (int g = 0; g < num_gimbals; g++) {
            int hold_idx = current_hold + g;
            if (hold_idx >= num_holds) {
                break;
            }

            float px = s_holds[hold_idx].x;
            float py = s_holds[hold_idx].y;
            int angle_x = pixel_to_servo_x(px);
            int angle_y = pixel_to_servo_y(py);

            ESP_LOGI(TAG, "[PLAYBACK] *** EXECUTING HOLD %d/%d ***", hold_idx, num_holds);
            ESP_LOGI(TAG, "[TRAJECTORY] Hold %d: pixel(%.1f,%.1f) -> servo_X=%d° servo_Y=%d°",
                     hold_idx, px, py, angle_x, angle_y);

            servo_drive(g * 2, angle_x);
            servo_drive(g * 2 + 1, angle_y);
        }

        // Wait for next signal
#if ROUTE_AUTO_ADVANCE
        TickType_t timeout = pdMS_TO_TICKS(ROUTE_DWELL_MS);
        ESP_LOGI(TAG, "[PLAYBACK] Waiting %d ms for servo settle before next hold...",
                 ROUTE_DWELL_MS);
#else
        TickType_t timeout = portMAX_DELAY;
        ESP_LOGI(TAG, "[PLAYBACK] Waiting for manual next command...");
#endif

        xSemaphoreTake(s_next_sem, timeout);

        // Advance hold index
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        int old_hold = s_current_hold;
        s_current_hold += num_gimbals;
        if (s_current_hold >= s_num_holds) {
            ESP_LOGI(TAG, "[PLAYBACK] Reached end of route (hold %d). Looping back to hold 0.",
                     old_hold);
            s_current_hold = 0;
        } else {
            ESP_LOGI(TAG, "[PLAYBACK] Advancing: hold %d -> hold %d", old_hold, s_current_hold);
        }
        xSemaphoreGive(s_state_mutex);
    }
}

void route_init(void) {
    s_next_sem = xSemaphoreCreateBinary();
    s_state_mutex = xSemaphoreCreateMutex();

    if (!s_next_sem || !s_state_mutex) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return;
    }

    xTaskCreate(route_playback_task, "route_play", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Route system initialized");
}
