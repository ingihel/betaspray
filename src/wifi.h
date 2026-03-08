#pragma once

#include "esp_wifi.h"
#include "esp_netif.h"

#define BETASPRAY_WIFI_SSID "BetaSpray"
#define BETASPRAY_WIFI_PASS "betaspray123"
#define BETASPRAY_MAX_STA_CONN 4

// Initialize SoftAP mode, using the settings from wifi.h
void wifi_init_softap(void);
