#pragma once

#include "esp_err.h"

/// Run memory smoke tests at startup.
/// Logs results via ESP_LOGI/W/E. Returns ESP_OK if all tests pass,
/// ESP_FAIL if any critical test fails.
esp_err_t smoketest_run(void);
