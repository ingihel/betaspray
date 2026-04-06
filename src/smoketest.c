#include "smoketest.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "smoketest";

// Expected PSRAM size: 8 MB (WROOM-1-N16R8).
// We allow a 512 KB margin for allocator metadata and system reservations.
#define PSRAM_EXPECTED_BYTES   (8 * 1024 * 1024)
#define PSRAM_MIN_BYTES        (PSRAM_EXPECTED_BYTES - 512 * 1024)

#define PSRAM_TEST_PATTERN 0xA5

// Minimum acceptable free internal SRAM after init (WiFi + FreeRTOS stacks need ~80 KB).
#define INTERNAL_MIN_FREE_BYTES (80 * 1024)

static esp_err_t test_psram_size(void) {
    size_t total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t free  = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);

    ESP_LOGI(TAG, "PSRAM total=%u KB  free=%u KB  (expected >=%u KB total)",
             (unsigned)(total / 1024), (unsigned)(free / 1024),
             (unsigned)(PSRAM_MIN_BYTES / 1024));

    if (total < PSRAM_MIN_BYTES) {
        ESP_LOGE(TAG, "PSRAM FAIL: total %u KB < expected %u KB",
                 (unsigned)(total / 1024), (unsigned)(PSRAM_MIN_BYTES / 1024));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "PSRAM size OK");
    return ESP_OK;
}

static esp_err_t test_psram_readwrite(void) {
    size_t alloc_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM)/2;
    uint8_t *buf = heap_caps_malloc(alloc_size, MALLOC_CAP_SPIRAM);
    if (!buf) {
        ESP_LOGE(TAG, "PSRAM FAIL: could not allocate %u KB", (unsigned)(alloc_size / 1024));
        return ESP_FAIL;
    }

    // Write pattern
    memset(buf, PSRAM_TEST_PATTERN, alloc_size);

    // Verify pattern in 4 KB chunks to keep watchdog happy
    esp_err_t result = ESP_OK;
    const size_t chunk = 4096;
    for (size_t offset = 0; offset < alloc_size; offset += chunk) {
        size_t n = (offset + chunk <= alloc_size) ? chunk : (alloc_size - offset);
        for (size_t i = 0; i < n; i++) {
            if (buf[offset + i] != PSRAM_TEST_PATTERN) {
                ESP_LOGE(TAG, "PSRAM FAIL: data mismatch at offset %u (got 0x%02x, want 0x%02x)",
                         (unsigned)(offset + i), buf[offset + i], PSRAM_TEST_PATTERN);
                goto done;
            }
        }
    }

    ESP_LOGI(TAG, "PSRAM read/write OK (%u KB verified)", (unsigned)(alloc_size / 1024));

done:
    heap_caps_free(buf);
    return result;
}

static esp_err_t test_internal_sram(void) {
    size_t total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    size_t free  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    ESP_LOGI(TAG, "Internal SRAM total=%u KB  free=%u KB  (min free required=%u KB)",
             (unsigned)(total / 1024), (unsigned)(free / 1024),
             (unsigned)(INTERNAL_MIN_FREE_BYTES / 1024));

    if (free < INTERNAL_MIN_FREE_BYTES) {
        ESP_LOGE(TAG, "Internal SRAM FAIL: only %u KB free, need %u KB",
                 (unsigned)(free / 1024), (unsigned)(INTERNAL_MIN_FREE_BYTES / 1024));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Internal SRAM free OK");
    return ESP_OK;
}

esp_err_t smoketest_run(void) {
    ESP_LOGI(TAG, "=== Memory smoke tests begin ===");

    esp_err_t overall = ESP_OK;

    if (test_psram_size()      != ESP_OK) overall = ESP_FAIL;
    if (test_psram_readwrite() != ESP_OK) overall = ESP_FAIL;
    if (test_internal_sram()   != ESP_OK) overall = ESP_FAIL;

    if (overall == ESP_OK) {
        ESP_LOGI(TAG, "=== All smoke tests PASSED ===");
    } else {
        ESP_LOGE(TAG, "=== One or more smoke tests FAILED ===");
    }

    return overall;
}
