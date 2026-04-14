#pragma once

#include "esp_err.h"
#include <stdbool.h>

typedef enum {
    FSM_MODE_MANUAL = 0,
    FSM_MODE_USER,
} fsm_mode_t;

typedef enum {
    FSM_USER_SCAN_WALL = 0,
    FSM_USER_SET_ROUTE,
    FSM_USER_CLIMB_WALL,
} fsm_user_state_t;

typedef enum {
    FSM_CLIMB_IDLE = 0,
    FSM_CLIMB_RUNNING,
    FSM_CLIMB_PAUSED,
} fsm_climb_state_t;

typedef enum {
    FSM_PROCEED_MANUAL = 0,
    FSM_PROCEED_TIMED,
    FSM_PROCEED_AUTO,
} fsm_proceed_mode_t;

// Initialize FSM — call once from app_main after fatfs_init
void fsm_init(void);

// Getters
fsm_mode_t fsm_get_mode(void);
fsm_user_state_t fsm_get_user_state(void);
fsm_climb_state_t fsm_get_climb_state(void);
fsm_proceed_mode_t fsm_get_proceed_mode(void);
int fsm_get_active_scan(void);
int fsm_get_active_route(void);

// State transitions — return ESP_OK or ESP_ERR_INVALID_STATE
esp_err_t fsm_set_mode(fsm_mode_t mode);
esp_err_t fsm_set_user_state(fsm_user_state_t state);
esp_err_t fsm_set_climb_state(fsm_climb_state_t state);
esp_err_t fsm_set_proceed_mode(fsm_proceed_mode_t mode, int interval_ms);

// Set active scan/route IDs
void fsm_set_active_scan(int scan_id);
void fsm_set_active_route(int route_id);

// Guard: check if an operation is allowed in the current state
esp_err_t fsm_check_op(const char *operation);
