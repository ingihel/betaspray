#pragma once

#include "esp_http_server.h"

#define SUPPRESS_HTTP_SERIAL_LOGGING 0

// Create the HTTP server instance, and register all relevant handlers
// After genesis, the server may dynamically allocate a single frame-buffer.
// soft todo: ensure that the configuration here is fully appropriate.
httpd_handle_t server_start(void);

// Stop the HTTP server instance if it seems alive, and clean up the frame
// buffer if one is in use.
void server_stop(httpd_handle_t server);
