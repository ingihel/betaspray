#include "fsm.h"
#include "route.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "fsm";

static fsm_mode_t s_mode = FSM_MODE_MANUAL;
static fsm_user_state_t s_user_state = FSM_USER_SCAN_WALL;
static fsm_climb_state_t s_climb_state = FSM_CLIMB_IDLE;
static fsm_proceed_mode_t s_proceed_mode = FSM_PROCEED_MANUAL;
static int s_timed_interval_ms = 3000;
static int s_active_scan = -1;
static int s_active_route = -1;
static SemaphoreHandle_t s_mutex = NULL;

void fsm_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "FSM initialized (MANUAL mode)");
}

// --- Getters ---

fsm_mode_t fsm_get_mode(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    fsm_mode_t m = s_mode;
    xSemaphoreGive(s_mutex);
    return m;
}

fsm_user_state_t fsm_get_user_state(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    fsm_user_state_t s = s_user_state;
    xSemaphoreGive(s_mutex);
    return s;
}

fsm_climb_state_t fsm_get_climb_state(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    fsm_climb_state_t s = s_climb_state;
    xSemaphoreGive(s_mutex);
    return s;
}

fsm_proceed_mode_t fsm_get_proceed_mode(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    fsm_proceed_mode_t m = s_proceed_mode;
    xSemaphoreGive(s_mutex);
    return m;
}

int fsm_get_active_scan(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int id = s_active_scan;
    xSemaphoreGive(s_mutex);
    return id;
}

int fsm_get_active_route(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int id = s_active_route;
    xSemaphoreGive(s_mutex);
    return id;
}

// --- Setters ---

void fsm_set_active_scan(int scan_id) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_active_scan = scan_id;
    xSemaphoreGive(s_mutex);
}

void fsm_set_active_route(int route_id) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_active_route = route_id;
    xSemaphoreGive(s_mutex);
}

esp_err_t fsm_set_mode(fsm_mode_t mode) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (mode == s_mode) {
        xSemaphoreGive(s_mutex);
        return ESP_OK;
    }

    // Leaving USER mode — stop any active playback
    if (s_mode == FSM_MODE_USER && s_climb_state == FSM_CLIMB_RUNNING) {
        route_pause();
        route_restart();
        s_climb_state = FSM_CLIMB_IDLE;
    }

    s_mode = mode;

    // Reset user sub-state when entering USER mode
    if (mode == FSM_MODE_USER) {
        s_user_state = FSM_USER_SCAN_WALL;
        s_climb_state = FSM_CLIMB_IDLE;
    }

    ESP_LOGI(TAG, "Mode -> %s", mode == FSM_MODE_MANUAL ? "MANUAL" : "USER");
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t fsm_set_user_state(fsm_user_state_t state) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (s_mode != FSM_MODE_USER) {
        ESP_LOGW(TAG, "Cannot set user_state in MANUAL mode");
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    // Validate transitions
    if (state == FSM_USER_SET_ROUTE && s_active_scan < 0) {
        ESP_LOGW(TAG, "Cannot enter SET_ROUTE without an active scan");
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    if (state == FSM_USER_CLIMB_WALL && s_active_route < 0) {
        ESP_LOGW(TAG, "Cannot enter CLIMB_WALL without an active route");
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    // Leaving CLIMB_WALL — stop playback
    if (s_user_state == FSM_USER_CLIMB_WALL && s_climb_state == FSM_CLIMB_RUNNING) {
        route_pause();
        route_restart();
    }

    s_user_state = state;
    s_climb_state = FSM_CLIMB_IDLE;

    const char *names[] = {"SCAN_WALL", "SET_ROUTE", "CLIMB_WALL"};
    ESP_LOGI(TAG, "User state -> %s", names[state]);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t fsm_set_climb_state(fsm_climb_state_t state) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    if (s_mode != FSM_MODE_USER || s_user_state != FSM_USER_CLIMB_WALL) {
        ESP_LOGW(TAG, "Cannot set climb_state outside CLIMB_WALL");
        xSemaphoreGive(s_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    s_climb_state = state;

    switch (state) {
    case FSM_CLIMB_RUNNING:
        // Set timed interval based on proceed mode
        if (s_proceed_mode == FSM_PROCEED_TIMED) {
            route_set_timed_interval(s_timed_interval_ms);
        } else {
            route_set_timed_interval(0);
        }
        route_play();
        ESP_LOGI(TAG, "Climb -> RUNNING");
        break;
    case FSM_CLIMB_PAUSED:
        route_pause();
        ESP_LOGI(TAG, "Climb -> PAUSED");
        break;
    case FSM_CLIMB_IDLE:
        route_stop();
        ESP_LOGI(TAG, "Climb -> IDLE");
        break;
    }

    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

esp_err_t fsm_set_proceed_mode(fsm_proceed_mode_t mode, int interval_ms) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_proceed_mode = mode;
    if (mode == FSM_PROCEED_TIMED && interval_ms > 0) {
        s_timed_interval_ms = interval_ms;
    }
    const char *names[] = {"MANUAL", "TIMED", "AUTO"};
    ESP_LOGI(TAG, "Proceed mode -> %s (interval=%d ms)", names[mode], s_timed_interval_ms);
    xSemaphoreGive(s_mutex);
    return ESP_OK;
}

// --- Operation guard ---

esp_err_t fsm_check_op(const char *op) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    esp_err_t ret = ESP_OK;

    if (s_mode == FSM_MODE_MANUAL) {
        // MANUAL mode: everything except user-workflow operations allowed
        if (strcmp(op, "centroids/upload") == 0 ||
            strcmp(op, "centroids/get") == 0 ||
            strcmp(op, "scan/save") == 0) {
            ret = ESP_ERR_INVALID_STATE;
        }
        goto done;
    }

    // USER mode — check sub-state
    if (s_user_state == FSM_USER_SCAN_WALL) {
        if (strcmp(op, "capture") == 0 || strcmp(op, "get") == 0 ||
            strcmp(op, "configure") == 0 ||
            strcmp(op, "centroids/upload") == 0 || strcmp(op, "centroids/get") == 0 ||
            strcmp(op, "scan/save") == 0 || strcmp(op, "laser") == 0) {
            goto done;
        }
        ret = ESP_ERR_INVALID_STATE;
    } else if (s_user_state == FSM_USER_SET_ROUTE) {
        if (strcmp(op, "route/create") == 0 || strcmp(op, "route/delete") == 0 ||
            strcmp(op, "centroids/get") == 0) {
            goto done;
        }
        ret = ESP_ERR_INVALID_STATE;
    } else if (s_user_state == FSM_USER_CLIMB_WALL) {
        if (strcmp(op, "route/play") == 0 || strcmp(op, "route/pause") == 0 ||
            strcmp(op, "route/next") == 0 || strcmp(op, "route/restart") == 0 ||
            strcmp(op, "laser") == 0) {
            goto done;
        }
        // AUTO mode allows capture/get/configure
        if (s_proceed_mode == FSM_PROCEED_AUTO) {
            if (strcmp(op, "capture") == 0 || strcmp(op, "get") == 0 ||
                strcmp(op, "configure") == 0) {
                goto done;
            }
        }
        ret = ESP_ERR_INVALID_STATE;
    }

done:
    xSemaphoreGive(s_mutex);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Operation '%s' rejected in current state", op);
    }
    return ret;
}
