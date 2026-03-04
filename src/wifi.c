#include "wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include <stdio.h>

static const char *TAG = "wifi";

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id,
                               void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "Station %02x:%02x:%02x:%02x:%02x:%02x joined, AID=%d", event->mac[0],
                 event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5],
                 event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "Station %02x:%02x:%02x:%02x:%02x:%02x left, AID=%d", event->mac[0],
                 event->mac[1], event->mac[2], event->mac[3], event->mac[4], event->mac[5],
                 event->aid);
    }
}

void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));

    wifi_config_t wifi_config = {
        .ap =
            {
                .ssid = BETASPRAY_WIFI_SSID,
                .ssid_len = sizeof(BETASPRAY_WIFI_SSID) - 1,
                .channel = 1,
                .password = BETASPRAY_WIFI_PASS,
                .max_connection = BETASPRAY_MAX_STA_CONN,
                .authmode = WIFI_AUTH_WPA2_PSK,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    esp_netif_ip_info_t ip_info;
    ESP_ERROR_CHECK(esp_netif_get_ip_info(ap_netif, &ip_info));
    ESP_LOGI(TAG, "SoftAP started - SSID: %s  IP: %d.%d.%d.%d", BETASPRAY_WIFI_SSID,
             IP2STR(&ip_info.ip));
}
