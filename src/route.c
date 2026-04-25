#include "route.h"
#include "conf.h"
#include "fatfs.h"
#include "servo.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "route";

#define ROUTE_DEFAULT_DWELL_MS 3000

static route_hold_t s_holds[ROUTE_MAX_HOLDS];
static int s_num_holds = 0;
static volatile int s_current_hold = 0;
static volatile bool s_playing = false;
static SemaphoreHandle_t s_next_sem = NULL;
static SemaphoreHandle_t s_state_mutex = NULL;

static volatile route_play_mode_t s_mode = ROUTE_MODE_SEQUENTIAL;
static volatile int s_leap_num = NUM_SERVOS / 2;              // active gimbals in leapfrog
static volatile int s_leap_next = 0;                          // which gimbal advances next
static volatile int s_leap_hold[NUM_SERVOS / 2] = {0,1,2,3}; // current hold index per gimbal
static volatile int s_timed_interval_ms = 0;     // 0 = manual (wait for next), >0 = auto-advance
static int s_gimbal_x_angle[NUM_SERVOS / 2];    // last driven X angle per gimbal, for collision checks
static int s_gimbal_offset_x[NUM_SERVOS / 2];  // legacy — mapped into poly[g][0]/[4]
static int s_gimbal_offset_y[NUM_SERVOS / 2];

// Per-gimbal bilinear polynomial correction coefficients.
// Layout: [a0, a1, a2, a3, b0, b1, b2, b3]
//   corrected_x = raw_x + a0 + a1*raw_x + a2*raw_y + a3*raw_x*raw_y
//   corrected_y = raw_y + b0 + b1*raw_x + b2*raw_y + b3*raw_x*raw_y
static float s_gimbal_poly[NUM_SERVOS / 2][8] = {{0}};

// Physical position of each gimbal relative to camera (meters).
// +X = right, +Y = behind camera (depth), +Z = up.
static float s_gimbal_pos[NUM_SERVOS / 2][3] = {
    { GIMBAL0_X, GIMBAL0_Y, GIMBAL0_Z },  // Gimbal 0 — Front-Right
    { GIMBAL1_X, GIMBAL1_Y, GIMBAL1_Z },  // Gimbal 1 — Front-Left
    { GIMBAL2_X, GIMBAL2_Y, GIMBAL2_Z },  // Gimbal 2 — Back-Right
    { GIMBAL3_X, GIMBAL3_Y, GIMBAL3_Z },  // Gimbal 3 — Back-Left
};

static route_transform_t s_transform = {
    .hfov_deg = 120.0f,
    .vfov_deg = 60.0f,
    .image_width = 320,
    .image_height = 240,
    .distance_m = 3.0f,
};

// Convert a pixel coordinate to servo angles for a specific gimbal.
//
// 1. Project pixel onto the wall plane using camera intrinsics + wall distance.
// 2. Compute the vector from the gimbal's physical position to that wall point.
// 3. Convert the vector to pan (X servo) and tilt (Y servo) angles.
static void pixel_to_servo_angles(float px, float py, int gimbal,
                                   int *out_x, int *out_y) {
    float dist = s_transform.distance_m;

    // Scale intrinsics if the current image resolution differs from calibration.
    float scale_x = (float)s_transform.image_width  / (float)CALIB_WIDTH;
    float scale_y = (float)s_transform.image_height / (float)CALIB_HEIGHT;
    float fx = CALIB_FX * scale_x;
    float fy = CALIB_FY * scale_y;
    float cx = CALIB_CX * scale_x;
    float cy = CALIB_CY * scale_y;

    // Step 1: pixel -> 3D point on the wall (camera frame).
    // Camera looks along +Y (forward toward wall). +X = right, +Z = up.
    float wall_x = (px - cx) / fx * dist;
    float wall_z = (cy - py) / fy * dist;   // image Y-down -> world Z-up
    float wall_y = dist;

    // Step 2: vector from gimbal to wall point.
    float gx = s_gimbal_pos[gimbal][0];
    float gy = s_gimbal_pos[gimbal][1];
    float gz = s_gimbal_pos[gimbal][2];

    float dx = wall_x - gx;
    float dy = wall_y + gy;   // gimbal is *behind* camera, so effective forward distance increases
    float dz = wall_z - gz;

    // Step 3: convert to servo angles.
    // Pan: atan2(dx, dy).  0 = straight ahead, positive = right.
    // Servo X: 90° = straight ahead. Camera-left pixel -> servo > 90.
    float pan_rad  = atan2f(dx, dy);
    int servo_x = (int)(90.0f - pan_rad * (180.0f / (float)M_PI));

    // Tilt: atan2(dz, dy).  0 = horizontal, positive = up.
    // Servo Y: 90° = horizontal/flat, increasing = tilting up.
    float tilt_rad = atan2f(dz, dy);
    int servo_y = (int)(90.0f + tilt_rad * (180.0f / (float)M_PI));

    *out_x = servo_x < 0 ? 0 : servo_x > 180 ? 180 : servo_x;
    *out_y = servo_y < 0 ? 0 : servo_y > 180 ? 180 : servo_y;
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
    ESP_LOGI(TAG, "[PLAYBACK] Zeroing all servos before route start");
    // Zero gimbal 1 first to avoid clashing with gimbal 0 during retract.
    servo_drive(2, 0); servo_drive(3, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    servo_drive(0, 0); servo_drive(1, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    servo_drive(4, 0); servo_drive(5, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    servo_drive(6, 0); servo_drive(7, 0);
    for (int i = 0; i < NUM_SERVOS / 2; i++) s_gimbal_x_angle[i] = 0;

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

void route_set_gimbal_position(int gimbal, float x, float y, float z) {
    if (gimbal < 0 || gimbal >= NUM_SERVOS / 2) {
        ESP_LOGW(TAG, "route_set_gimbal_position: invalid gimbal %d", gimbal);
        return;
    }
    s_gimbal_pos[gimbal][0] = x;
    s_gimbal_pos[gimbal][1] = y;
    s_gimbal_pos[gimbal][2] = z;
    ESP_LOGI(TAG, "Gimbal %d position: (%.4f, %.4f, %.4f) m", gimbal, x, y, z);
}

// Apply bilinear polynomial correction to raw servo angles.
static void apply_poly_correction(int gimbal, int raw_x, int raw_y,
                                   int *out_x, int *out_y) {
    float *c = s_gimbal_poly[gimbal];
    float rx = (float)raw_x;
    float ry = (float)raw_y;
    float cx = rx + (c[0] + c[1] * rx + c[2] * ry + c[3] * rx * ry);
    float cy = ry + (c[4] + c[5] * rx + c[6] * ry + c[7] * rx * ry);
    int ix = (int)(cx + 0.5f);
    int iy = (int)(cy + 0.5f);
    *out_x = ix < 0 ? 0 : ix > 180 ? 180 : ix;
    *out_y = iy < 0 ? 0 : iy > 180 ? 180 : iy;
}

void route_set_gimbal_poly(int gimbal, const float coeffs[8]) {
    if (gimbal < 0 || gimbal >= NUM_SERVOS / 2) {
        ESP_LOGW(TAG, "route_set_gimbal_poly: invalid gimbal %d", gimbal);
        return;
    }
    memcpy(s_gimbal_poly[gimbal], coeffs, 8 * sizeof(float));
    s_gimbal_offset_x[gimbal] = (int)coeffs[0];
    s_gimbal_offset_y[gimbal] = (int)coeffs[4];
    ESP_LOGI(TAG, "Gimbal %d poly: a=[%.4f,%.4f,%.4f,%.4f] b=[%.4f,%.4f,%.4f,%.4f]",
             gimbal, coeffs[0], coeffs[1], coeffs[2], coeffs[3],
             coeffs[4], coeffs[5], coeffs[6], coeffs[7]);
}

void route_get_gimbal_poly(int gimbal, float coeffs[8]) {
    if (gimbal < 0 || gimbal >= NUM_SERVOS / 2) {
        memset(coeffs, 0, 8 * sizeof(float));
        return;
    }
    memcpy(coeffs, s_gimbal_poly[gimbal], 8 * sizeof(float));
}

esp_err_t route_save_gimbal_poly(void) {
    esp_err_t err = fatfs_create(GIMBAL_POLY_FILE);
    if (err != ESP_OK) return err;
    err = fatfs_write(GIMBAL_POLY_FILE, s_gimbal_poly, 0, sizeof(s_gimbal_poly));
    if (err == ESP_OK)
        ESP_LOGI(TAG, "Poly calibration saved to %s (%d bytes)", GIMBAL_POLY_FILE, (int)sizeof(s_gimbal_poly));
    else
        ESP_LOGE(TAG, "Failed to save poly calibration");
    return err;
}

esp_err_t route_load_gimbal_poly(void) {
    float buf[NUM_SERVOS / 2][8];
    size_t bytes_read = 0;
    esp_err_t err = fatfs_read(GIMBAL_POLY_FILE, buf, 0, sizeof(buf), &bytes_read);
    if (err != ESP_OK || bytes_read != sizeof(buf)) {
        ESP_LOGW(TAG, "No poly calibration found, using defaults");
        return ESP_ERR_NOT_FOUND;
    }
    for (int g = 0; g < NUM_SERVOS / 2; g++)
        route_set_gimbal_poly(g, buf[g]);
    ESP_LOGI(TAG, "Poly calibration loaded from %s", GIMBAL_POLY_FILE);
    return ESP_OK;
}

// Public: compute raw (uncorrected) servo angles — used by calibration endpoint.
void route_compute_angles(float px, float py, int gimbal, int *out_x, int *out_y) {
    pixel_to_servo_angles(px, py, gimbal, out_x, out_y);
}

void route_set_gimbal_offset(int gimbal, int dx, int dy) {
    if (gimbal < 0 || gimbal >= NUM_SERVOS / 2) {
        ESP_LOGW(TAG, "route_set_gimbal_offset: invalid gimbal %d", gimbal);
        return;
    }
    // Write into polynomial as constant-only correction (backward compat).
    memset(s_gimbal_poly[gimbal], 0, sizeof(s_gimbal_poly[gimbal]));
    s_gimbal_poly[gimbal][0] = (float)dx;
    s_gimbal_poly[gimbal][4] = (float)dy;
    s_gimbal_offset_x[gimbal] = dx;
    s_gimbal_offset_y[gimbal] = dy;
    ESP_LOGI(TAG, "Gimbal %d offset: X%+d° Y%+d° (poly reset to constant)", gimbal, dx, dy);
}

void route_set_mode(route_play_mode_t mode, int leap_num) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_mode = mode;
    int max_gimbals = NUM_SERVOS / 2;
    s_leap_num = (leap_num < 1) ? 2 : (leap_num > max_gimbals) ? max_gimbals : leap_num;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "Mode set: %s, leap_num=%d",
             mode == ROUTE_MODE_LEAPFROG ? "LEAPFROG" : "SEQUENTIAL", s_leap_num);
}

void route_stop(void) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_playing = false;
    s_current_hold = 0;
    int total = s_num_holds;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "[PLAYBACK] STOP: halted and reset to hold 0/%d", total);
}

void route_set_timed_interval(int ms) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_timed_interval_ms = ms;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "Timed interval set: %d ms (%s)", ms, ms > 0 ? "auto" : "manual");
}

// Drive one gimbal (X+Y servo pair) to a hold's pixel coordinates.
// 100 ms gap between X and Y lets the first servo's inrush settle before the
// second one starts, reducing peak current draw on a shared supply.
static void drive_gimbal(int g, int hold_idx, int total_holds, const route_hold_t *hold) {
    int raw_x, raw_y;
    pixel_to_servo_angles(hold->x, hold->y, g, &raw_x, &raw_y);
    int angle_x, angle_y;
    apply_poly_correction(g, raw_x, raw_y, &angle_x, &angle_y);
    ESP_LOGI(TAG, "[GIMBAL] Gimbal %d -> hold %d/%d  pixel(%.1f, %.1f)  servo X=%d° Y=%d°",
             g, hold_idx + 1, total_holds, hold->x, hold->y, angle_x, angle_y);
    ESP_LOGI(TAG, "[GIMBAL]   Servo %d (X) driving to %d°", g * 2, angle_x);
    servo_drive(g * 2, angle_x);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "[GIMBAL]   Servo %d (Y) driving to %d°", g * 2 + 1, angle_y);
    servo_drive(g * 2 + 1, angle_y);
    ESP_LOGI(TAG, "[GIMBAL]   Gimbal %d at hold %d/%d done", g, hold_idx + 1, total_holds);
}

static void route_playback_task(void *arg) {
    ESP_LOGI(TAG, "Playback task started");
    bool leapfrog_initialized = false;

    while (1) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        bool playing = s_playing;
        int current_hold = s_current_hold;
        int num_holds = s_num_holds;
        route_play_mode_t mode = s_mode;
        int leap_num = s_leap_num;
        xSemaphoreGive(s_state_mutex);

        if (!playing) {
            leapfrog_initialized = false;
            vTaskDelay(50 / portTICK_PERIOD_MS);
            continue;
        }

        if (mode == ROUTE_MODE_LEAPFROG) {
            // ---- LEAPFROG MODE ----
            // Init: send each gimbal to its starting hold (0..leap_num-1).
            if (!leapfrog_initialized) {
                ESP_LOGI(TAG, "[LEAPFROG] Init: %d gimbals, %d total holds", leap_num, num_holds);
                for (int g = 0; g < leap_num; g++) {
                    s_leap_hold[g] = g;
                    if (g < num_holds) {
                        drive_gimbal(g, g, num_holds, &s_holds[g]);
                        // Stagger gimbals: let the PSU capacitors recover before
                        // starting the next gimbal's inrush current.
                        if (g < leap_num - 1)
                            vTaskDelay(pdMS_TO_TICKS(300));
                    }
                }
                s_leap_next = 0;
                leapfrog_initialized = true;
                ESP_LOGI(TAG, "[LEAPFROG] Init complete. Gimbals 0-%d at holds 1-%d.",
                         leap_num - 1, leap_num);
            }

            {
                int interval = s_timed_interval_ms;
                if (interval > 0) {
                    ESP_LOGI(TAG, "[LEAPFROG] Waiting %d ms before advancing gimbal %d...",
                             interval, s_leap_next);
                } else {
                    ESP_LOGI(TAG, "[LEAPFROG] Waiting for manual /route/next (gimbal %d next)...",
                             s_leap_next);
                }
                TickType_t timeout = interval > 0 ? pdMS_TO_TICKS(interval) : portMAX_DELAY;
                BaseType_t triggered = xSemaphoreTake(s_next_sem, timeout);
                ESP_LOGI(TAG, "[LEAPFROG] Advance triggered (%s)",
                         triggered == pdTRUE ? "manual" : "timed");
            }

            // Advance the next gimbal in rotation by leap_num holds.
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            int g = s_leap_next;
            int next_hold = s_leap_hold[g] + leap_num;
            if (next_hold < s_num_holds) {
                ESP_LOGI(TAG, "[LEAPFROG] Gimbal %d: hold %d -> hold %d (of %d)",
                         g, s_leap_hold[g] + 1, next_hold + 1, s_num_holds);
                s_leap_hold[g] = next_hold;
                xSemaphoreGive(s_state_mutex);
                drive_gimbal(g, next_hold, num_holds, &s_holds[next_hold]);
            } else {
                ESP_LOGI(TAG, "[LEAPFROG] Gimbal %d: hold %d is last — route complete.",
                         g, s_leap_hold[g] + 1);
                s_playing = false;
                xSemaphoreGive(s_state_mutex);
            }

            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_leap_next = (s_leap_next + 1) % leap_num;
            ESP_LOGI(TAG, "[LEAPFROG] Next gimbal to advance: %d", s_leap_next);
            xSemaphoreGive(s_state_mutex);

        } else {
            // ---- SEQUENTIAL MODE (original behaviour) ----
            leapfrog_initialized = false;
            int num_gimbals = NUM_SERVOS / 2;
            int total_steps = (num_holds + num_gimbals - 1) / num_gimbals;
            int step_num = current_hold / num_gimbals + 1;
            ESP_LOGI(TAG, "[SEQ] Step %d/%d: positioning gimbals at holds %d-%d",
                     step_num, total_steps,
                     current_hold + 1,
                     current_hold + num_gimbals < num_holds
                         ? current_hold + num_gimbals : num_holds);
            for (int g = 0; g < num_gimbals; g++) {
                int hold_idx = current_hold + g;
                if (hold_idx >= num_holds) break;
                drive_gimbal(g, hold_idx, num_holds, &s_holds[hold_idx]);
                if (g < num_gimbals - 1 && (hold_idx + 1) < num_holds)
                    vTaskDelay(pdMS_TO_TICKS(300));
            }
            ESP_LOGI(TAG, "[SEQ] Step %d/%d complete.", step_num, total_steps);

            {
                int interval = s_timed_interval_ms;
                if (interval > 0) {
                    ESP_LOGI(TAG, "[SEQ] Waiting %d ms before step %d...",
                             interval, step_num + 1);
                } else {
                    ESP_LOGI(TAG, "[SEQ] Waiting for manual /route/next (step %d done)...",
                             step_num);
                }
                TickType_t timeout = interval > 0 ? pdMS_TO_TICKS(interval) : portMAX_DELAY;
                BaseType_t triggered = xSemaphoreTake(s_next_sem, timeout);
                ESP_LOGI(TAG, "[SEQ] Advance triggered (%s)",
                         triggered == pdTRUE ? "manual" : "timed");
            }

            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            int old_hold = s_current_hold;
            s_current_hold += num_gimbals;
            if (s_current_hold >= s_num_holds) {
                ESP_LOGI(TAG, "[SEQ] End of route after step %d. Looping to step 1.", step_num);
                s_current_hold = 0;
            } else {
                ESP_LOGI(TAG, "[SEQ] Advancing to step %d (hold %d -> hold %d)",
                         step_num + 1, old_hold + 1, s_current_hold + 1);
            }
            xSemaphoreGive(s_state_mutex);
        }
    }
}

void route_init(void) {
    s_next_sem = xSemaphoreCreateBinary();
    s_state_mutex = xSemaphoreCreateMutex();

    if (!s_next_sem || !s_state_mutex) {
        ESP_LOGE(TAG, "Failed to create semaphores");
        return;
    }

    route_load_gimbal_poly();

    xTaskCreate(route_playback_task, "route_play", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Route system initialized");
}
