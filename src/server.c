#include "server.h"
#include "camera.h"
#include "route.h"
#include "servo.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <cJSON.h>
#include <stdio.h>

static const char *TAG = "server";

static uint8_t *s_frame_buf = NULL;
static size_t s_frame_len = 0;
static bool s_camera_enabled = false;
static bool s_use_psram = false;

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
    // cJSON *resolution = cJSON_GetObjectItem(json, "resolution");
    // if (resolution && resolution->type == cJSON_String) {
    //     framesize_t size = FRAMESIZE_QVGA;
    //     if (strcmp(resolution->valuestring, "QVGA") == 0) {
    //         size = FRAMESIZE_QVGA;
    //     } else if (strcmp(resolution->valuestring, "VGA") == 0) {
    //         size = FRAMESIZE_VGA;
    //     } else if (strcmp(resolution->valuestring, "SVGA") == 0) {
    //         size = FRAMESIZE_SVGA;
    //     }
    //     camera_set_resolution(size);
    //     ESP_LOGI(TAG, "Resolution set to %s", resolution->valuestring);
    // }

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
    int len = req->content_len;
    if (len <= 0 || len > 256) {
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
    // Optional: "gimbals" = active gimbal count for leapfrog (default 2, future: 4)
    route_play_mode_t mode = ROUTE_MODE_SEQUENTIAL;
    int leap_num = 2;
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
    route_set_mode(mode, leap_num);
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

    // Start playback
    route_play();
    char log_msg[128];
    snprintf(log_msg, sizeof(log_msg), "route=%d mode=%s gimbals=%d",
             route_num,
             mode == ROUTE_MODE_LEAPFROG ? "leapfrog" : "sequential",
             mode == ROUTE_MODE_LEAPFROG ? leap_num : NUM_SERVOS / 2);
    log_response("/route/play", "OK", log_msg);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// GET /route/pause - pause playback
static esp_err_t route_pause_handler(httpd_req_t *req) {
    log_request("GET", "/route/pause", "");
    route_pause();
    log_response("/route/pause", "OK", "");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// GET /route/next - advance to next hold
static esp_err_t route_next_handler(httpd_req_t *req) {
    log_request("GET", "/route/next", "");
    route_next();
    log_response("/route/next", "OK", "");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /route/restart - restart route from beginning
static esp_err_t route_restart_handler(httpd_req_t *req) {
    log_request("POST", "/route/restart", "");
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

httpd_handle_t server_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16; // 15 route+util handlers + 1 buffer
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
