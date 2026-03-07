#include "fatfs.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "wear_levelling.h"
#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

static const char *TAG = "fatfs";

static wl_handle_t s_wl = WL_INVALID_HANDLE;

static esp_err_t do_mount(bool format_if_empty) {
    esp_vfs_fat_mount_config_t cfg = {
        .max_files = FATFS_MAX_FILES,
        .format_if_mount_failed = format_if_empty,
        .allocation_unit_size = 4096,
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(FATFS_BASE_PATH, FATFS_PARTITION, &cfg, &s_wl);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mount failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t fatfs_init(bool format_if_empty) {
    if (s_wl != WL_INVALID_HANDLE) {
        ESP_LOGW(TAG, "Already mounted");
        return ESP_OK;
    }
    esp_err_t err = do_mount(format_if_empty);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Mounted '%s' at %s", FATFS_PARTITION, FATFS_BASE_PATH);
    }
    return err;
}

esp_err_t fatfs_reset(void) {
    if (s_wl == WL_INVALID_HANDLE) {
        esp_err_t err = do_mount(true);
        if (err != ESP_OK)
            return err;
    }
    esp_err_t err = esp_vfs_fat_spiflash_format_rw_wl(FATFS_BASE_PATH, FATFS_PARTITION);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Format failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "Partition '%s' formatted", FATFS_PARTITION);
    return ESP_OK;
}

esp_err_t fatfs_list(const char *dir) {
    ESP_LOGI(TAG, "[FILE_LIST] Listing directory '%s'", dir);
    DIR *dp = opendir(dir);
    if (!dp) {
        ESP_LOGE(TAG, "[FILE_LIST] ERROR: Cannot open '%s': errno %d", dir, errno);
        return ESP_FAIL;
    }
    struct dirent *entry;
    int count = 0;
    while ((entry = readdir(dp)) != NULL) {
        ESP_LOGI(TAG, "[FILE_LIST]   - %s", entry->d_name);
        count++;
    }
    if (count == 0) {
        ESP_LOGI(TAG, "[FILE_LIST]   (directory is empty)");
    } else {
        ESP_LOGI(TAG, "[FILE_LIST] Total: %d file(s)", count);
    }
    closedir(dp);
    return ESP_OK;
}

esp_err_t fatfs_read(const char *path, void *buf, size_t offset, size_t size, size_t *bytes_read) {
    ESP_LOGI(TAG, "[FILE_READ] Opening '%s' for reading", path);
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "[FILE_READ] ERROR: Cannot open '%s' for read: errno %d", path, errno);
        return ESP_ERR_NOT_FOUND;
    }
    if (fseek(f, (long)offset, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "[FILE_READ] ERROR: fseek failed on '%s' at offset %u", path, (unsigned)offset);
        fclose(f);
        return ESP_FAIL;
    }
    *bytes_read = fread(buf, 1, size, f);
    fclose(f);
    ESP_LOGI(TAG, "[FILE_READ] SUCCESS: Read %u/%u bytes from '%s' (offset:%u)",
             (unsigned)*bytes_read, (unsigned)size, path, (unsigned)offset);
    return ESP_OK;
}

esp_err_t fatfs_write(const char *path, const void *buf, size_t offset, size_t size) {
    ESP_LOGI(TAG, "[FILE_WRITE] Opening '%s' for writing (offset:%u, size:%u)", path, (unsigned)offset, (unsigned)size);
    FILE *f = fopen(path, "r+b");
    if (!f) {
        ESP_LOGE(TAG, "[FILE_WRITE] ERROR: Cannot open '%s' for write (call fatfs_create first): errno %d", path,
                 errno);
        return ESP_ERR_NOT_FOUND;
    }
    if (fseek(f, (long)offset, SEEK_SET) != 0) {
        ESP_LOGE(TAG, "[FILE_WRITE] ERROR: fseek failed on '%s' at offset %u", path, (unsigned)offset);
        fclose(f);
        return ESP_FAIL;
    }
    size_t written = fwrite(buf, 1, size, f);
    fclose(f);
    if (written != size) {
        ESP_LOGE(TAG, "[FILE_WRITE] ERROR: Short write on '%s': %u/%u bytes", path, (unsigned)written, (unsigned)size);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "[FILE_WRITE] SUCCESS: Wrote %u bytes to '%s' (offset:%u)", (unsigned)written, path, (unsigned)offset);
    return ESP_OK;
}

esp_err_t fatfs_create(const char *path) {
    ESP_LOGI(TAG, "[FILE_CREATE] Creating file '%s'", path);
    FILE *f = fopen(path, "wb");
    if (!f) {
        ESP_LOGE(TAG, "[FILE_CREATE] ERROR: Cannot create '%s': errno %d", path, errno);
        return ESP_FAIL;
    }
    fclose(f);
    ESP_LOGI(TAG, "[FILE_CREATE] SUCCESS: Created '%s'", path);
    return ESP_OK;
}

esp_err_t fatfs_delete(const char *path) {
    ESP_LOGI(TAG, "[FILE_DELETE] Deleting file '%s'", path);
    if (unlink(path) != 0) {
        ESP_LOGE(TAG, "[FILE_DELETE] ERROR: Cannot delete '%s': errno %d", path, errno);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "[FILE_DELETE] SUCCESS: Deleted '%s'", path);
    return ESP_OK;
}
