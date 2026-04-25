#include "server.h"
#include "camera.h"
#include "fatfs.h"
#include "fsm.h"
#include "laser.h"
#include "route.h"
#include "servo.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <cJSON.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "server";

static uint8_t *s_frame_buf = NULL;
static size_t s_frame_len = 0;
static bool s_camera_enabled = false;
static bool s_use_psram = false;

/// --------- FSM GUARD ----------

#define FSM_GUARD(op) do { \
    if (fsm_check_op(op) != ESP_OK) { \
        httpd_resp_set_status(req, "409 Conflict"); \
        httpd_resp_sendstr(req, "Not allowed in current state"); \
        return ESP_OK; \
    } \
} while (0)

/// --------- HELPERS ----------

// Helper function to log an HTTP request in a formatted manner
static void log_request(const char *method, const char *uri, const char *details) {
#if SUPPRESS_HTTP_SERIAL_LOGGING == 1
    return;
#else
    if (details && strlen(details) > 0) {
        ESP_LOGI(TAG, "[HTTP] %s %s %s", method, uri, details);
    } else {
        ESP_LOGI(TAG, "[HTTP] %s %s", method, uri);
    }
#endif
}

// Helper function to log an HTTP response with status
static void log_response(const char *uri, const char *status, const char *details) {
#if SUPPRESS_HTTP_SERIAL_LOGGING == 1
    return;
#else
    if (details && strlen(details) > 0) {
        ESP_LOGI(TAG, "[HTTP] %s -> %s %s", uri, status, details);
    } else {
        ESP_LOGI(TAG, "[HTTP] %s -> %s", uri, status);
    }
#endif
}

/// --------- ROUTES ----------

// POST /capture - trigger camera capture and store frame
static esp_err_t capture_handler(httpd_req_t *req) {
    log_request("POST", "/capture", "");
    FSM_GUARD("capture");
    if (!s_camera_enabled) {
        httpd_resp_sendstr(req, "Camera disabled");
        return ESP_OK;
    }

    camera_fb_t *fb = camera_capture_frame();
    if (!fb) {
        ESP_LOGE(TAG, "Failed to capture frame");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Free previous frame if it exists
    if (s_frame_buf) {
        heap_caps_free(s_frame_buf);
    }

    s_frame_buf = s_use_psram
        ? heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM)
        : malloc(fb->len);
    if (!s_frame_buf) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer");
        camera_release_frame(fb);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    // Copy frame data to static buffer
    memcpy(s_frame_buf, fb->buf, fb->len);
    s_frame_len = fb->len;
    camera_release_frame(fb);

    ESP_LOGI(TAG, "POST /capture: stored %u bytes", s_frame_len);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// GET /get - return the stored frame as JPEG
static esp_err_t get_handler(httpd_req_t *req) {
    log_request("GET", "/get", "");
    FSM_GUARD("get");
    if (s_frame_len == 0) {
        httpd_resp_sendstr(req, "No frame captured");
        return ESP_OK;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

    esp_err_t ret = httpd_resp_send(req, (const char *)s_frame_buf, s_frame_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send frame");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "GET /get: sent %u bytes", s_frame_len);
    return ESP_OK;
}

// POST /configure - configure camera settings
static esp_err_t configure_handler(httpd_req_t *req) {
    log_request("POST", "/configure", "");
    FSM_GUARD("configure");
    int len = req->content_len;
    if (len <= 0 || len > 512) {
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }

    char *buf = (char *)malloc(len + 1);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (httpd_req_recv(req, buf, len) != len) {
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    // Handle enabled field
    cJSON *enabled = cJSON_GetObjectItem(json, "enabled");
    if (enabled && enabled->type == cJSON_True) {
        if (!s_camera_enabled) {
            if (camera_is_initialized()) {
                ESP_LOGW(TAG, "Camera already initialized");
            } else {
                ESP_ERROR_CHECK(camera_init());
            }
            s_camera_enabled = true;
            ESP_LOGI(TAG, "Camera enabled");
        }
    } else if (enabled && enabled->type == cJSON_False) {
        if (s_camera_enabled) {
            camera_deinit();
            s_camera_enabled = false;
            ESP_LOGI(TAG, "Camera disabled");
        }
    }

    // TODO: Handle resolution field (currently fixed to QVGA)
    cJSON *resolution = cJSON_GetObjectItem(json, "resolution");
    if (resolution && resolution->type == cJSON_String) {
        framesize_t size = FRAMESIZE_QVGA;
        if (strcmp(resolution->valuestring, "QVGA") == 0) {
            size = FRAMESIZE_QVGA;
        } else if (strcmp(resolution->valuestring, "VGA") == 0) {
            size = FRAMESIZE_VGA;
        } else if (strcmp(resolution->valuestring, "SVGA") == 0) {
            size = FRAMESIZE_SVGA;
        } else if (strcmp(resolution->valuestring, "5MP") == 0) {
            size = FRAMESIZE_5MP;
        }
        camera_set_resolution(size);
        ESP_LOGI(TAG, "Resolution set to %s", resolution->valuestring);
    }

    // Handle psram field
    cJSON *psram = cJSON_GetObjectItem(json, "psram");
    if (psram && cJSON_IsBool(psram)) {
        s_use_psram = cJSON_IsTrue(psram);
        ESP_LOGI(TAG, "Frame buffer allocation: %s", s_use_psram ? "PSRAM" : "DRAM");
    }

    // Handle format field
    cJSON *format = cJSON_GetObjectItem(json, "format");
    if (format && format->type == cJSON_String) {
        pixformat_t fmt = PIXFORMAT_JPEG;
        if (strcmp(format->valuestring, "JPEG") == 0) {
            fmt = PIXFORMAT_JPEG;
        } else if (strcmp(format->valuestring, "RGB565") == 0) {
            fmt = PIXFORMAT_RGB565;
        } else if (strcmp(format->valuestring, "GRAYSCALE") == 0) {
            fmt = PIXFORMAT_GRAYSCALE;
        }
        camera_set_format(fmt);
        ESP_LOGI(TAG, "Format set to %s", format->valuestring);
    }

    cJSON_Delete(json);
    free(buf);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /route/create - create a route from hold coordinates
static esp_err_t route_create_handler(httpd_req_t *req) {
    int len = req->content_len;
    log_request("POST", "/route/create", "");
    FSM_GUARD("route/create");
    if (len <= 0 || len > 2048) { // Reduced from 4096
        log_response("/route/create", "FAIL", "invalid content length");
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }

    // Use heap instead of stack for request buffer
    char *buf = (char *)malloc(len + 1);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (httpd_req_recv(req, buf, len) != len) {
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON *json = cJSON_Parse(buf);
    free(buf);

    if (!json) {
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    cJSON *route_num = cJSON_GetObjectItem(json, "route");
    cJSON *holds = cJSON_GetObjectItem(json, "holds");

    if (!route_num || !route_num->type || !holds || holds->type != cJSON_Array) {
        httpd_resp_sendstr(req, "Missing route number or holds array");
        cJSON_Delete(json);
        return ESP_OK;
    }

    int route_n = route_num->valueint;
    int num_holds = cJSON_GetArraySize(holds);
    if (num_holds <= 0 || num_holds > 256) {
        httpd_resp_sendstr(req, "Invalid number of holds");
        cJSON_Delete(json);
        return ESP_OK;
    }

    // Build hold array on heap instead of stack
    route_hold_t *hold_array = (route_hold_t *)malloc(num_holds * sizeof(route_hold_t));
    if (!hold_array) {
        cJSON_Delete(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cJSON *item = NULL;
    int idx = 0;
    cJSON_ArrayForEach(item, holds) {
        if (item->type == cJSON_Array && cJSON_GetArraySize(item) >= 2) {
            hold_array[idx].x = cJSON_GetArrayItem(item, 0)->valuedouble;
            hold_array[idx].y = cJSON_GetArrayItem(item, 1)->valuedouble;
            idx++;
        }
    }

    esp_err_t err = route_create(route_n, hold_array, idx);
    free(hold_array);
    cJSON_Delete(json);

    if (err != ESP_OK) {
        log_response("/route/create", "FAIL", "route_create error");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char detail[64];
    snprintf(detail, sizeof(detail), "route=%d holds=%d", route_n, idx);
    log_response("/route/create", "OK", detail);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /route/delete - delete a route file
static esp_err_t route_delete_handler(httpd_req_t *req) {
    log_request("POST", "/route/delete", "");
    int len = req->content_len;
    if (len <= 0 || len > 1024) {
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }

    char buf[len + 1];
    if (httpd_req_recv(req, buf, len) != len) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    cJSON *route_num = cJSON_GetObjectItem(json, "route");
    if (!route_num || !route_num->type) {
        httpd_resp_sendstr(req, "Missing route number");
        cJSON_Delete(json);
        return ESP_OK;
    }

    esp_err_t err = route_delete(route_num->valueint);
    cJSON_Delete(json);

    if (err != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /route/load - load a route from storage into RAM
static esp_err_t route_load_handler(httpd_req_t *req) {
    log_request("POST", "/route/load", "");
    int len = req->content_len;
    if (len <= 0 || len > 1024) {
        log_response("/route/load", "FAIL", "invalid content length");
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }

    char buf[len + 1];
    if (httpd_req_recv(req, buf, len) != len) {
        log_response("/route/load", "FAIL", "recv error");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        log_response("/route/load", "FAIL", "parse error");
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    cJSON *route_num = cJSON_GetObjectItem(json, "route");
    if (!route_num || !route_num->type) {
        log_response("/route/load", "FAIL", "missing route number");
        httpd_resp_sendstr(req, "Missing route number");
        cJSON_Delete(json);
        return ESP_OK;
    }

    int route_n = route_num->valueint;
    esp_err_t err = route_load(route_n);
    cJSON_Delete(json);

    if (err != ESP_OK) {
        log_response("/route/load", "FAIL", "route_load error");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char detail[32];
    snprintf(detail, sizeof(detail), "route=%d", route_n);
    log_response("/route/load", "OK", detail);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /route/play - load route and start playback
static esp_err_t route_play_handler(httpd_req_t *req) {
    log_request("POST", "/route/play", "");
    FSM_GUARD("route/play");
    int len = req->content_len;
    if (len <= 0 || len > 512) {
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }

    char *buf = heap_caps_malloc(len + 1, MALLOC_CAP_DEFAULT);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int recv_len = httpd_req_recv(req, buf, len);
    if (recv_len != len) {
        heap_caps_free(buf);
        httpd_resp_sendstr(req, "Failed to read body");
        return ESP_OK;
    }

    buf[len] = '\0';
    cJSON *json = cJSON_Parse(buf);
    heap_caps_free(buf);

    if (!json) {
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    cJSON *route_item = cJSON_GetObjectItem(json, "route");
    if (!route_item || !cJSON_IsNumber(route_item)) {
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "Missing or invalid 'route' field");
        return ESP_OK;
    }

    int route_num = route_item->valueint;

    // Optional: "mode" = "sequential" (default) or "leapfrog"
    // Optional: "gimbals" = active gimbal count for leapfrog (default NUM_SERVOS/2)
    // Optional: "interval_ms" = auto-advance interval in ms; 0 or absent = manual
    route_play_mode_t mode = ROUTE_MODE_SEQUENTIAL;
    int leap_num = NUM_SERVOS / 2;
    int interval_ms = 0;

    cJSON *mode_item = cJSON_GetObjectItem(json, "mode");
    if (mode_item && cJSON_IsString(mode_item)) {
        if (strcmp(mode_item->valuestring, "leapfrog") == 0) {
            mode = ROUTE_MODE_LEAPFROG;
            cJSON *gimbals_item = cJSON_GetObjectItem(json, "gimbals");
            if (gimbals_item && cJSON_IsNumber(gimbals_item)) {
                leap_num = gimbals_item->valueint;
            }
        }
    }

    cJSON *interval_item = cJSON_GetObjectItem(json, "interval_ms");
    if (interval_item && cJSON_IsNumber(interval_item)) {
        interval_ms = (int)interval_item->valuedouble;
        if (interval_ms < 0) interval_ms = 0;
    }

    // Optional: inline pixel-to-angle transform.  Must match the image dimensions
    // used when the route was created.  hfov_deg/vfov_deg default to camera FOV.
    cJSON *img_w = cJSON_GetObjectItem(json, "image_width");
    cJSON *img_h = cJSON_GetObjectItem(json, "image_height");
    if (img_w && cJSON_IsNumber(img_w) && img_h && cJSON_IsNumber(img_h)) {
        route_transform_t t = {
            .hfov_deg    = 120.0f,
            .vfov_deg    = 60.0f,
            .image_width  = (int)img_w->valuedouble,
            .image_height = (int)img_h->valuedouble,
            .distance_m  = 3.0f,
        };
        cJSON *hfov = cJSON_GetObjectItem(json, "hfov_deg");
        cJSON *vfov = cJSON_GetObjectItem(json, "vfov_deg");
        cJSON *dist = cJSON_GetObjectItem(json, "distance_m");
        if (hfov && cJSON_IsNumber(hfov)) t.hfov_deg = (float)hfov->valuedouble;
        if (vfov && cJSON_IsNumber(vfov)) t.vfov_deg = (float)vfov->valuedouble;
        if (dist && cJSON_IsNumber(dist)) t.distance_m = (float)dist->valuedouble;
        route_set_transform(&t);
    }

    route_set_mode(mode, leap_num);
    route_set_timed_interval(interval_ms);
    cJSON_Delete(json);

    // Load the route first
    esp_err_t err = route_load(route_num);
    if (err != ESP_OK) {
        char resp[128];
        snprintf(resp, sizeof(resp), "Failed to load route %d", route_num);
        httpd_resp_sendstr(req, resp);
        log_response("/route/play", "FAIL", "route load failed");
        return ESP_OK;
    }

    fsm_set_active_route(route_num);

    // Start playback
    route_play();
    char log_msg[128];
    snprintf(log_msg, sizeof(log_msg), "route=%d mode=%s gimbals=%d interval=%dms",
             route_num,
             mode == ROUTE_MODE_LEAPFROG ? "leapfrog" : "sequential",
             mode == ROUTE_MODE_LEAPFROG ? leap_num : NUM_SERVOS / 2,
             interval_ms);
    log_response("/route/play", "OK", log_msg);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// GET /route/pause - pause playback
static esp_err_t route_pause_handler(httpd_req_t *req) {
    log_request("GET", "/route/pause", "");
    FSM_GUARD("route/pause");
    route_pause();
    log_response("/route/pause", "OK", "");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// GET /route/next - advance to next hold
static esp_err_t route_next_handler(httpd_req_t *req) {
    log_request("GET", "/route/next", "");
    FSM_GUARD("route/next");
    route_next();
    log_response("/route/next", "OK", "");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /route/restart - restart route from beginning
static esp_err_t route_restart_handler(httpd_req_t *req) {
    log_request("POST", "/route/restart", "");
    FSM_GUARD("route/restart");
    route_restart();
    log_response("/route/restart", "OK", "");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /route/mapping - set XY to angular mapping (camera + distance parameters)
static esp_err_t route_mapping_handler(httpd_req_t *req) {
    log_request("POST", "/route/mapping", "");
    int len = req->content_len;
    if (len <= 0 || len > 512) {
        log_response("/route/mapping", "FAIL", "invalid content length");
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }

    uint8_t buf[512];
    int ret = httpd_req_recv(req, (char *)buf, len);
    if (ret <= 0) {
        log_response("/route/mapping", "FAIL", "recv error");
        httpd_resp_sendstr(req, "Request error");
        return ESP_FAIL;
    }

    buf[ret] = '\0';
    cJSON *json = cJSON_Parse((char *)buf);
    if (!json) {
        log_response("/route/mapping", "FAIL", "JSON parse error");
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    // Required fields: camera params and distance
    cJSON *hfov = cJSON_GetObjectItem(json, "hfov_deg");
    cJSON *vfov = cJSON_GetObjectItem(json, "vfov_deg");
    cJSON *width = cJSON_GetObjectItem(json, "image_width");
    cJSON *height = cJSON_GetObjectItem(json, "image_height");
    cJSON *distance = cJSON_GetObjectItem(json, "distance_m");

    if (!hfov || !vfov || !width || !height || !distance ||
        !cJSON_IsNumber(hfov) || !cJSON_IsNumber(vfov) ||
        !cJSON_IsNumber(width) || !cJSON_IsNumber(height) || !cJSON_IsNumber(distance)) {
        cJSON_Delete(json);
        log_response("/route/mapping", "FAIL", "missing required fields");
        httpd_resp_sendstr(req, "Required: hfov_deg, vfov_deg, image_width, image_height, distance_m");
        return ESP_OK;
    }

    float h_fov = (float)hfov->valuedouble;
    float v_fov = (float)vfov->valuedouble;
    int w = (int)width->valuedouble;
    int h = (int)height->valuedouble;
    float dist = (float)distance->valuedouble;

    if (h_fov <= 0.0f || v_fov <= 0.0f || w <= 0 || h <= 0 || dist <= 0.0f) {
        cJSON_Delete(json);
        log_response("/route/mapping", "FAIL", "all values must be positive");
        httpd_resp_sendstr(req, "All values must be positive");
        return ESP_OK;
    }

    // Set transform with camera params and distance
    route_transform_t t = {
        .hfov_deg = h_fov,
        .vfov_deg = v_fov,
        .image_width = w,
        .image_height = h,
        .distance_m = dist,
    };
    route_set_transform(&t);

    cJSON_Delete(json);

    char response[256];
    snprintf(response, sizeof(response),
             "{\"status\":\"ok\",\"hfov_deg\":%.1f,\"vfov_deg\":%.1f,\"image_width\":%d,\"image_height\":%d,\"distance_m\":%.2f}",
             h_fov, v_fov, w, h, dist);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, response);
    log_response("/route/mapping", "OK", "");
    return ESP_OK;
}
// GET /start - begin projecting the loaded route
static esp_err_t start_handler(httpd_req_t *req) {
    log_request("GET", "/start", "");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// GET /stop - stop projection
static esp_err_t stop_handler(httpd_req_t *req) {
    log_request("GET", "/stop", "");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /servo - command an arbitrary servo
static esp_err_t servo_handler(httpd_req_t *req) {
    log_request("POST", "/servo", "");
    FSM_GUARD("servo");
    int len = req->content_len;
    if (len <= 0 || len > 256) {
        log_response("/servo", "FAIL", "invalid content length");
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }

    char *buf = (char *)malloc(len + 1);
    if (!buf) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (httpd_req_recv(req, buf, len) != len) {
        free(buf);
        log_response("/servo", "FAIL", "recv error");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON *json = cJSON_Parse(buf);
    free(buf);

    if (!json) {
        log_response("/servo", "FAIL", "parse error");
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    cJSON *servo_id = cJSON_GetObjectItem(json, "servo");
    cJSON *angle = cJSON_GetObjectItem(json, "angle");

    if (!servo_id || servo_id->type != cJSON_Number || !angle || angle->type != cJSON_Number) {
        log_response("/servo", "FAIL", "missing servo or angle");
        httpd_resp_sendstr(req, "Missing servo or angle");
        cJSON_Delete(json);
        return ESP_OK;
    }

    int id = servo_id->valueint;
    int ang = angle->valueint;

    if (id < 0 || id >= NUM_SERVOS || ang < 0 || ang > 180) {
        char detail[64];
        snprintf(detail, sizeof(detail), "servo=%d invalid (0-%d) or angle=%d invalid (0-180)", id,
                 NUM_SERVOS - 1, ang);
        log_response("/servo", "FAIL", detail);
        httpd_resp_sendstr(req, "Invalid servo ID or angle");
        cJSON_Delete(json);
        return ESP_OK;
    }

    servo_drive(id, ang);
    cJSON_Delete(json);

    char detail[64];
    snprintf(detail, sizeof(detail), "servo=%d angle=%d", id, ang);
    log_response("/servo", "OK", detail);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /laser - turn laser on or off: {"on": true} or {"on": false}
static esp_err_t laser_handler(httpd_req_t *req) {
    log_request("POST", "/laser", "");
    FSM_GUARD("laser");
    int len = req->content_len;
    if (len <= 0 || len > 256) {
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }

    char buf[len + 1];
    if (httpd_req_recv(req, buf, len) != len) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    cJSON *on = cJSON_GetObjectItem(json, "on");
    if (on && cJSON_IsBool(on)) {
        laser_set(cJSON_IsTrue(on));
    } else {
        laser_toggle();
    }
    cJSON_Delete(json);

    log_response("/laser", "OK", laser_is_on() ? "on" : "off");
    httpd_resp_sendstr(req, laser_is_on() ? "ON" : "OFF");
    return ESP_OK;
}

// POST /test - echo back the request body prefixed with "ECHO: "
// GET /state - return FSM state as JSON
static esp_err_t state_get_handler(httpd_req_t *req) {
    log_request("GET", "/state", "");

    const char *mode_str = fsm_get_mode() == FSM_MODE_MANUAL ? "MANUAL" : "USER";
    const char *user_names[] = {"SCAN_WALL", "SET_ROUTE", "CLIMB_WALL"};
    const char *climb_names[] = {"IDLE", "RUNNING", "PAUSED"};
    const char *proceed_names[] = {"MANUAL", "TIMED", "AUTO"};

    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "mode", mode_str);

    if (fsm_get_mode() == FSM_MODE_USER) {
        cJSON_AddStringToObject(json, "user_state", user_names[fsm_get_user_state()]);
        if (fsm_get_user_state() == FSM_USER_CLIMB_WALL) {
            cJSON_AddStringToObject(json, "climb_state", climb_names[fsm_get_climb_state()]);
            cJSON_AddStringToObject(json, "proceed_mode", proceed_names[fsm_get_proceed_mode()]);
        }
    }
    cJSON_AddNumberToObject(json, "active_scan", fsm_get_active_scan());
    cJSON_AddNumberToObject(json, "active_route", fsm_get_active_route());

    char *str = cJSON_PrintUnformatted(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, str);
    free(str);
    cJSON_Delete(json);
    return ESP_OK;
}

// POST /state - transition FSM state
static esp_err_t state_set_handler(httpd_req_t *req) {
    log_request("POST", "/state", "");
    int len = req->content_len;
    if (len <= 0 || len > 512) {
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }

    char *buf = (char *)malloc(len + 1);
    if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }
    if (httpd_req_recv(req, buf, len) != len) {
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON *json = cJSON_Parse(buf);
    free(buf);
    if (!json) {
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    esp_err_t err = ESP_OK;

    // Handle mode change
    cJSON *mode = cJSON_GetObjectItem(json, "mode");
    if (mode && mode->type == cJSON_String) {
        if (strcmp(mode->valuestring, "MANUAL") == 0) {
            err = fsm_set_mode(FSM_MODE_MANUAL);
        } else if (strcmp(mode->valuestring, "USER") == 0) {
            err = fsm_set_mode(FSM_MODE_USER);
        }
        if (err != ESP_OK) goto fail;
    }

    // Handle user_state change
    cJSON *ustate = cJSON_GetObjectItem(json, "user_state");
    if (ustate && ustate->type == cJSON_String) {
        if (strcmp(ustate->valuestring, "SCAN_WALL") == 0) {
            err = fsm_set_user_state(FSM_USER_SCAN_WALL);
        } else if (strcmp(ustate->valuestring, "SET_ROUTE") == 0) {
            err = fsm_set_user_state(FSM_USER_SET_ROUTE);
        } else if (strcmp(ustate->valuestring, "CLIMB_WALL") == 0) {
            err = fsm_set_user_state(FSM_USER_CLIMB_WALL);
        }
        if (err != ESP_OK) goto fail;
    }

    // Handle proceed_mode
    cJSON *pmode = cJSON_GetObjectItem(json, "proceed_mode");
    if (pmode && pmode->type == cJSON_String) {
        fsm_proceed_mode_t pm = FSM_PROCEED_MANUAL;
        int interval = 3000;
        if (strcmp(pmode->valuestring, "TIMED") == 0) pm = FSM_PROCEED_TIMED;
        else if (strcmp(pmode->valuestring, "AUTO") == 0) pm = FSM_PROCEED_AUTO;

        cJSON *ival = cJSON_GetObjectItem(json, "interval_ms");
        if (ival && cJSON_IsNumber(ival)) interval = ival->valueint;

        err = fsm_set_proceed_mode(pm, interval);
        if (err != ESP_OK) goto fail;
    }

    // Handle climb_state change
    cJSON *cstate = cJSON_GetObjectItem(json, "climb_state");
    if (cstate && cstate->type == cJSON_String) {
        if (strcmp(cstate->valuestring, "RUNNING") == 0) {
            err = fsm_set_climb_state(FSM_CLIMB_RUNNING);
        } else if (strcmp(cstate->valuestring, "PAUSED") == 0) {
            err = fsm_set_climb_state(FSM_CLIMB_PAUSED);
        } else if (strcmp(cstate->valuestring, "IDLE") == 0) {
            err = fsm_set_climb_state(FSM_CLIMB_IDLE);
        }
        if (err != ESP_OK) goto fail;
    }

    cJSON_Delete(json);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;

fail:
    cJSON_Delete(json);
    httpd_resp_set_status(req, "409 Conflict");
    httpd_resp_sendstr(req, "Invalid state transition");
    return ESP_OK;
}

// POST /centroids/upload - store centroids to fatfs
static esp_err_t centroids_upload_handler(httpd_req_t *req) {
    log_request("POST", "/centroids/upload", "");
    FSM_GUARD("centroids/upload");

    int len = req->content_len;
    if (len <= 0 || len > 8192) {
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }

    char *buf = heap_caps_malloc(len + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) buf = (char *)malloc(len + 1);
    if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }

    if (httpd_req_recv(req, buf, len) != len) {
        free(buf);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON *json = cJSON_Parse(buf);
    free(buf);
    if (!json) {
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    cJSON *scan_id_j = cJSON_GetObjectItem(json, "scan_id");
    cJSON *centroids = cJSON_GetObjectItem(json, "centroids");
    if (!scan_id_j || !cJSON_IsNumber(scan_id_j) ||
        !centroids || centroids->type != cJSON_Array) {
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "Required: scan_id (int), centroids (array)");
        return ESP_OK;
    }

    int scan_id = scan_id_j->valueint;
    int count = cJSON_GetArraySize(centroids);
    if (count <= 0 || count > 1024) {
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "Invalid centroid count");
        return ESP_OK;
    }

    // Build binary: [uint32 count][float x, float y]...
    int data_size = 4 + count * 8;
    uint8_t *data = (uint8_t *)malloc(data_size);
    if (!data) {
        cJSON_Delete(json);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    *(uint32_t *)data = (uint32_t)count;
    float *coords = (float *)(data + 4);
    cJSON *item = NULL;
    int idx = 0;
    cJSON_ArrayForEach(item, centroids) {
        if (item->type == cJSON_Array && cJSON_GetArraySize(item) >= 2) {
            coords[idx * 2] = (float)cJSON_GetArrayItem(item, 0)->valuedouble;
            coords[idx * 2 + 1] = (float)cJSON_GetArrayItem(item, 1)->valuedouble;
            idx++;
        }
    }
    *(uint32_t *)data = (uint32_t)idx;
    int actual_size = 4 + idx * 8;

    char path[64];
    snprintf(path, sizeof(path), "/fatfs/scan%d.dat", scan_id);
    esp_err_t err = fatfs_create(path);
    if (err == ESP_OK) err = fatfs_write(path, data, 0, actual_size);
    free(data);
    cJSON_Delete(json);

    if (err != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    fsm_set_active_scan(scan_id);
    char detail[64];
    snprintf(detail, sizeof(detail), "scan_id=%d centroids=%d", scan_id, idx);
    log_response("/centroids/upload", "OK", detail);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// GET /centroids - read centroids from fatfs as JSON
static esp_err_t centroids_get_handler(httpd_req_t *req) {
    log_request("GET", "/centroids", "");
    FSM_GUARD("centroids/get");

    // Parse scan_id from query string
    char query[32] = {0};
    int scan_id = 0;
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[8];
        if (httpd_query_key_value(query, "scan_id", val, sizeof(val)) == ESP_OK) {
            scan_id = atoi(val);
        }
    }

    char path[64];
    snprintf(path, sizeof(path), "/fatfs/scan%d.dat", scan_id);

    // Read header
    uint32_t count = 0;
    size_t bytes_read = 0;
    esp_err_t err = fatfs_read(path, &count, 0, 4, &bytes_read);
    if (err != ESP_OK || bytes_read != 4 || count == 0 || count > 1024) {
        httpd_resp_sendstr(req, "Scan not found");
        return ESP_OK;
    }

    // Read coordinates
    int coords_size = count * 8;
    uint8_t *buf = (uint8_t *)malloc(coords_size);
    if (!buf) { httpd_resp_send_500(req); return ESP_FAIL; }

    err = fatfs_read(path, buf, 4, coords_size, &bytes_read);
    if (err != ESP_OK || bytes_read != (size_t)coords_size) {
        free(buf);
        httpd_resp_sendstr(req, "Read error");
        return ESP_OK;
    }

    // Build JSON response
    cJSON *json = cJSON_CreateObject();
    cJSON_AddNumberToObject(json, "scan_id", scan_id);
    cJSON_AddNumberToObject(json, "count", count);
    cJSON *arr = cJSON_AddArrayToObject(json, "centroids");

    float *coords = (float *)buf;
    for (int i = 0; i < (int)count; i++) {
        cJSON *pair = cJSON_CreateArray();
        cJSON_AddItemToArray(pair, cJSON_CreateNumber(coords[i * 2]));
        cJSON_AddItemToArray(pair, cJSON_CreateNumber(coords[i * 2 + 1]));
        cJSON_AddItemToArray(arr, pair);
    }
    free(buf);

    char *str = cJSON_PrintUnformatted(json);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, str);
    free(str);
    cJSON_Delete(json);
    return ESP_OK;
}

// POST /scan/save - save frame buffer to fatfs
static esp_err_t scan_save_handler(httpd_req_t *req) {
    log_request("POST", "/scan/save", "");
    FSM_GUARD("scan/save");

    int len = req->content_len;
    if (len <= 0 || len > 256) {
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }

    char buf[len + 1];
    if (httpd_req_recv(req, buf, len) != len) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON *json = cJSON_Parse(buf);
    if (!json) {
        httpd_resp_sendstr(req, "Invalid JSON");
        return ESP_OK;
    }

    cJSON *scan_id_j = cJSON_GetObjectItem(json, "scan_id");
    if (!scan_id_j || !cJSON_IsNumber(scan_id_j)) {
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "Required: scan_id");
        return ESP_OK;
    }
    int scan_id = scan_id_j->valueint;
    cJSON_Delete(json);

    if (s_frame_len == 0 || !s_frame_buf) {
        httpd_resp_sendstr(req, "No frame captured");
        return ESP_OK;
    }

    char path[64];
    snprintf(path, sizeof(path), "/fatfs/scan%d.jpg", scan_id);

    esp_err_t err = fatfs_create(path);
    if (err == ESP_OK) err = fatfs_write(path, s_frame_buf, 0, s_frame_len);

    if (err != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char detail[64];
    snprintf(detail, sizeof(detail), "scan_id=%d size=%u", scan_id, (unsigned)s_frame_len);
    log_response("/scan/save", "OK", detail);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /gimbal/zero — zeros all gimbals in collision-safe order (g1 before g0)
static esp_err_t gimbal_zero_handler(httpd_req_t *req) {
    log_request("POST", "/gimbal/zero", "");
    laser_set(false);
    servo_drive(0, 0); servo_drive(1, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    servo_drive(4, 0); servo_drive(5, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    servo_drive(2, 0); servo_drive(3, 0);
    log_response("/gimbal/zero", "OK", "");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /gimbal/offset — {"gimbal":0,"dx":5,"dy":-3}
static esp_err_t gimbal_offset_handler(httpd_req_t *req) {
    log_request("POST", "/gimbal/offset", "");
    int len = req->content_len;
    if (len <= 0 || len > 128) {
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }
    char buf[len + 1];
    if (httpd_req_recv(req, buf, len) != len) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';
    cJSON *json = cJSON_Parse(buf);
    if (!json) { httpd_resp_sendstr(req, "Invalid JSON"); return ESP_OK; }

    cJSON *g_j  = cJSON_GetObjectItem(json, "gimbal");
    cJSON *dx_j = cJSON_GetObjectItem(json, "dx");
    cJSON *dy_j = cJSON_GetObjectItem(json, "dy");
    if (!g_j || !cJSON_IsNumber(g_j)) {
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "Required: gimbal");
        return ESP_OK;
    }
    int g  = g_j->valueint;
    int dx = dx_j && cJSON_IsNumber(dx_j) ? dx_j->valueint : 0;
    int dy = dy_j && cJSON_IsNumber(dy_j) ? dy_j->valueint : 0;
    cJSON_Delete(json);

    route_set_gimbal_offset(g, dx, dy);
    char detail[64];
    snprintf(detail, sizeof(detail), "gimbal=%d dx=%d dy=%d", g, dx, dy);
    log_response("/gimbal/offset", "OK", detail);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /gimbal/poly — set polynomial correction coefficients
// {"gimbal":0,"coeffs":[a0,a1,a2,a3,b0,b1,b2,b3],"save":true}
// or {"all":[[8 floats],[8 floats],[8 floats],[8 floats]],"save":true}
static esp_err_t gimbal_poly_set_handler(httpd_req_t *req) {
    log_request("POST", "/gimbal/poly", "");
    int len = req->content_len;
    if (len <= 0 || len > 1024) {
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }
    char buf[len + 1];
    if (httpd_req_recv(req, buf, len) != len) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';
    cJSON *json = cJSON_Parse(buf);
    if (!json) { httpd_resp_sendstr(req, "Invalid JSON"); return ESP_OK; }

    bool do_save = false;
    cJSON *save_j = cJSON_GetObjectItem(json, "save");
    if (save_j && cJSON_IsTrue(save_j)) do_save = true;

    // Bulk mode: {"all": [[8],[8],[8],[8]]}
    cJSON *all_j = cJSON_GetObjectItem(json, "all");
    if (all_j && cJSON_IsArray(all_j)) {
        int n = cJSON_GetArraySize(all_j);
        for (int g = 0; g < n && g < NUM_SERVOS / 2; g++) {
            cJSON *arr = cJSON_GetArrayItem(all_j, g);
            if (!arr || cJSON_GetArraySize(arr) != 8) continue;
            float coeffs[8];
            for (int i = 0; i < 8; i++)
                coeffs[i] = (float)cJSON_GetArrayItem(arr, i)->valuedouble;
            route_set_gimbal_poly(g, coeffs);
        }
    } else {
        // Single gimbal: {"gimbal":0, "coeffs":[...]}
        cJSON *g_j = cJSON_GetObjectItem(json, "gimbal");
        cJSON *c_j = cJSON_GetObjectItem(json, "coeffs");
        if (!g_j || !c_j || cJSON_GetArraySize(c_j) != 8) {
            cJSON_Delete(json);
            httpd_resp_sendstr(req, "Required: gimbal + coeffs[8] or all[[8]x4]");
            return ESP_OK;
        }
        float coeffs[8];
        for (int i = 0; i < 8; i++)
            coeffs[i] = (float)cJSON_GetArrayItem(c_j, i)->valuedouble;
        route_set_gimbal_poly(g_j->valueint, coeffs);
    }

    cJSON_Delete(json);
    if (do_save) route_save_gimbal_poly();

    log_response("/gimbal/poly", "OK", "");
    httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
    return ESP_OK;
}

// GET /gimbal/poly — return all polynomial coefficients
static esp_err_t gimbal_poly_get_handler(httpd_req_t *req) {
    log_request("GET", "/gimbal/poly", "");
    int num_gimbals = NUM_SERVOS / 2;
    char resp[512];
    int pos = 0;
    pos += snprintf(resp + pos, sizeof(resp) - pos, "{\"gimbals\":[");
    for (int g = 0; g < num_gimbals; g++) {
        float c[8];
        route_get_gimbal_poly(g, c);
        if (g > 0) pos += snprintf(resp + pos, sizeof(resp) - pos, ",");
        pos += snprintf(resp + pos, sizeof(resp) - pos,
                        "[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f]",
                        c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]);
    }
    pos += snprintf(resp + pos, sizeof(resp) - pos, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// POST /gimbal/point — compute raw angles for a pixel + drive (no poly correction)
// {"gimbal":0, "px":100.0, "py":200.0, "image_width":2560, "image_height":1920}
static esp_err_t gimbal_point_handler(httpd_req_t *req) {
    log_request("POST", "/gimbal/point", "");
    int len = req->content_len;
    if (len <= 0 || len > 256) {
        httpd_resp_sendstr(req, "Invalid content length");
        return ESP_OK;
    }
    char buf[len + 1];
    if (httpd_req_recv(req, buf, len) != len) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    buf[len] = '\0';
    cJSON *json = cJSON_Parse(buf);
    if (!json) { httpd_resp_sendstr(req, "Invalid JSON"); return ESP_OK; }

    cJSON *g_j  = cJSON_GetObjectItem(json, "gimbal");
    cJSON *px_j = cJSON_GetObjectItem(json, "px");
    cJSON *py_j = cJSON_GetObjectItem(json, "py");
    cJSON *iw_j = cJSON_GetObjectItem(json, "image_width");
    cJSON *ih_j = cJSON_GetObjectItem(json, "image_height");

    if (!g_j || !px_j || !py_j) {
        cJSON_Delete(json);
        httpd_resp_sendstr(req, "Required: gimbal, px, py");
        return ESP_OK;
    }

    // Temporarily set transform image dimensions if provided.
    if (iw_j && ih_j && cJSON_IsNumber(iw_j) && cJSON_IsNumber(ih_j)) {
        route_transform_t t = {
            .hfov_deg = 120.0f, .vfov_deg = 60.0f,
            .image_width = (int)iw_j->valuedouble,
            .image_height = (int)ih_j->valuedouble,
            .distance_m = 3.0f,
        };
        cJSON *dist_j = cJSON_GetObjectItem(json, "distance_m");
        if (dist_j && cJSON_IsNumber(dist_j)) t.distance_m = (float)dist_j->valuedouble;
        route_set_transform(&t);
    }

    int g = g_j->valueint;
    float px = (float)px_j->valuedouble;
    float py = (float)py_j->valuedouble;
    cJSON_Delete(json);

    int angle_x, angle_y;
    route_compute_angles(px, py, g, &angle_x, &angle_y);

    // Drive servos without polynomial correction.
    servo_drive(g * 2, angle_x);
    vTaskDelay(pdMS_TO_TICKS(100));
    servo_drive(g * 2 + 1, angle_y);

    char resp[128];
    snprintf(resp, sizeof(resp), "{\"angle_x\":%d,\"angle_y\":%d}", angle_x, angle_y);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, resp);

    char detail[64];
    snprintf(detail, sizeof(detail), "g=%d px=%.0f py=%.0f -> X=%d Y=%d", g, px, py, angle_x, angle_y);
    log_response("/gimbal/point", "OK", detail);
    return ESP_OK;
}

// POST /test - echo back the request body prefixed with "ECHO: "
static esp_err_t test_handler(httpd_req_t *req) {
    log_request("POST", "/test", "");
    int len = req->content_len;
    char buf[len + 1];

    if (httpd_req_recv(req, buf, len) != len) {
        httpd_resp_send_500(req);
        return ESP_ERR_HTTPD_TASK;
    }
    buf[len] = '\0';

    char resp[len + 7]; // "ECHO: " + body + null
    snprintf(resp, sizeof(resp), "ECHO: %s", buf);
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static const httpd_uri_t uri_capture = {
    .uri = "/capture",
    .method = HTTP_POST,
    .handler = capture_handler,
};

static const httpd_uri_t uri_get = {
    .uri = "/get",
    .method = HTTP_GET,
    .handler = get_handler,
};

static const httpd_uri_t uri_configure = {
    .uri = "/configure",
    .method = HTTP_POST,
    .handler = configure_handler,
};

static const httpd_uri_t uri_servo = {
    .uri = "/servo",
    .method = HTTP_POST,
    .handler = servo_handler,
};

static const httpd_uri_t uri_test = {
    .uri = "/test",
    .method = HTTP_POST,
    .handler = test_handler,
};

static const httpd_uri_t uri_start = {
    .uri = "/start",
    .method = HTTP_GET,
    .handler = start_handler,
};

static const httpd_uri_t uri_stop = {
    .uri = "/stop",
    .method = HTTP_GET,
    .handler = stop_handler,
};

static const httpd_uri_t uri_route_create = {
    .uri = "/route/create",
    .method = HTTP_POST,
    .handler = route_create_handler,
};

static const httpd_uri_t uri_route_delete = {
    .uri = "/route/delete",
    .method = HTTP_POST,
    .handler = route_delete_handler,
};

static const httpd_uri_t uri_route_load = {
    .uri = "/route/load",
    .method = HTTP_POST,
    .handler = route_load_handler,
};

static const httpd_uri_t uri_route_play = {
    .uri = "/route/play",
    .method = HTTP_POST,
    .handler = route_play_handler,
};

static const httpd_uri_t uri_route_pause = {
    .uri = "/route/pause",
    .method = HTTP_GET,
    .handler = route_pause_handler,
};

static const httpd_uri_t uri_route_next = {
    .uri = "/route/next",
    .method = HTTP_GET,
    .handler = route_next_handler,
};

static const httpd_uri_t uri_route_restart = {
    .uri = "/route/restart",
    .method = HTTP_POST,
    .handler = route_restart_handler,
};

static const httpd_uri_t uri_route_mapping = {
    .uri = "/route/mapping",
    .method = HTTP_POST,
    .handler = route_mapping_handler,
};

static const httpd_uri_t uri_laser = {
    .uri = "/laser",
    .method = HTTP_POST,
    .handler = laser_handler,
};

static const httpd_uri_t uri_state_get = {
    .uri = "/state",
    .method = HTTP_GET,
    .handler = state_get_handler,
};

static const httpd_uri_t uri_state_set = {
    .uri = "/state",
    .method = HTTP_POST,
    .handler = state_set_handler,
};

static const httpd_uri_t uri_centroids_upload = {
    .uri = "/centroids/upload",
    .method = HTTP_POST,
    .handler = centroids_upload_handler,
};

static const httpd_uri_t uri_centroids_get = {
    .uri = "/centroids",
    .method = HTTP_GET,
    .handler = centroids_get_handler,
};

static const httpd_uri_t uri_scan_save = {
    .uri = "/scan/save",
    .method = HTTP_POST,
    .handler = scan_save_handler,
};

static const httpd_uri_t uri_gimbal_zero = {
    .uri = "/gimbal/zero",
    .method = HTTP_POST,
    .handler = gimbal_zero_handler,
};

static const httpd_uri_t uri_gimbal_offset = {
    .uri = "/gimbal/offset",
    .method = HTTP_POST,
    .handler = gimbal_offset_handler,
};

static const httpd_uri_t uri_gimbal_poly_set = {
    .uri = "/gimbal/poly",
    .method = HTTP_POST,
    .handler = gimbal_poly_set_handler,
};

static const httpd_uri_t uri_gimbal_poly_get = {
    .uri = "/gimbal/poly",
    .method = HTTP_GET,
    .handler = gimbal_poly_get_handler,
};

static const httpd_uri_t uri_gimbal_point = {
    .uri = "/gimbal/point",
    .method = HTTP_POST,
    .handler = gimbal_point_handler,
};

httpd_handle_t server_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 29;
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    httpd_register_uri_handler(server, &uri_capture);
    httpd_register_uri_handler(server, &uri_get);
    httpd_register_uri_handler(server, &uri_configure);
    httpd_register_uri_handler(server, &uri_servo);
    httpd_register_uri_handler(server, &uri_test);
    httpd_register_uri_handler(server, &uri_start);
    httpd_register_uri_handler(server, &uri_stop);
    httpd_register_uri_handler(server, &uri_route_create);
    httpd_register_uri_handler(server, &uri_route_delete);
    httpd_register_uri_handler(server, &uri_route_load);
    httpd_register_uri_handler(server, &uri_route_play);
    httpd_register_uri_handler(server, &uri_route_pause);
    httpd_register_uri_handler(server, &uri_route_next);
    httpd_register_uri_handler(server, &uri_route_restart);
    httpd_register_uri_handler(server, &uri_route_mapping);
    httpd_register_uri_handler(server, &uri_laser);
    httpd_register_uri_handler(server, &uri_state_get);
    httpd_register_uri_handler(server, &uri_state_set);
    httpd_register_uri_handler(server, &uri_centroids_upload);
    httpd_register_uri_handler(server, &uri_centroids_get);
    httpd_register_uri_handler(server, &uri_scan_save);
    httpd_register_uri_handler(server, &uri_gimbal_zero);
    httpd_register_uri_handler(server, &uri_gimbal_offset);
    httpd_register_uri_handler(server, &uri_gimbal_poly_set);
    httpd_register_uri_handler(server, &uri_gimbal_poly_get);
    httpd_register_uri_handler(server, &uri_gimbal_point);

    ESP_LOGI(TAG, "HTTP server started");
    return server;
}

void server_stop(httpd_handle_t server) {
    if (server) {
        httpd_stop(server);
    }
    // Static frame buffer doesn't need cleanup
    s_frame_len = 0;
}
