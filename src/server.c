#include "server.h"
#include "esp_log.h"

static const char *TAG = "server";

// POST /route - receive route hold data from the client
static esp_err_t route_handler(httpd_req_t *req) {
    int len = req->content_len;
    char buf[len];

    if (httpd_req_recv(req, buf, len) != len) {
        ESP_LOGE(TAG, "Failed to read request body");
        httpd_resp_send_500(req);
        return ESP_ERR_HTTPD_TASK;
    }

    ESP_LOGI(TAG, "POST /route (%d bytes): %.*s", len, len, buf);
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// GET /start - begin projecting the loaded route
static esp_err_t start_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "GET /start");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// GET /stop - stop projection
static esp_err_t stop_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "GET /stop");
    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

// POST /test - echo back the request body prefixed with "ECHO: "
static esp_err_t test_handler(httpd_req_t *req) {
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

static const httpd_uri_t uri_test = {
    .uri = "/test",
    .method = HTTP_POST,
    .handler = test_handler,
};

static const httpd_uri_t uri_route = {
    .uri = "/route",
    .method = HTTP_POST,
    .handler = route_handler,
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

httpd_handle_t server_start(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }

    httpd_register_uri_handler(server, &uri_test);
    httpd_register_uri_handler(server, &uri_route);
    httpd_register_uri_handler(server, &uri_start);
    httpd_register_uri_handler(server, &uri_stop);

    ESP_LOGI(TAG, "HTTP server started");
    return server;
}

void server_stop(httpd_handle_t server) {
    if (server) {
        httpd_stop(server);
    }
}
