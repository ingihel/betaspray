#pragma once

#include "esp_http_server.h"

httpd_handle_t server_start(void);
void server_stop(httpd_handle_t server);
