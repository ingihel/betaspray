#pragma once

#include "esp_err.h"
#include <stdbool.h>

#define LASER_PIN 17

// Initialize the laser GPIO as output (off by default).
esp_err_t laser_init(void);

// Turn the laser on or off.
void laser_set(bool on);

// Toggle the laser state.
void laser_toggle(void);

// Return current laser state.
bool laser_is_on(void);
