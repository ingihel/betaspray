#pragma once

#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

// VFS mount point and partition label (must match partitions.csv)
#define FATFS_BASE_PATH "/fatfs"
#define FATFS_PARTITION "storage"
#define FATFS_MAX_FILES 5

// Mount the FAT filesystem on the internal SPI flash.
// Pass format_if_empty=true to auto-format on first boot.
esp_err_t fatfs_init(bool format_if_empty);

// Unmount, format, and remount - erases all stored files.
esp_err_t fatfs_reset(void);

// Log all files in dir (e.g. FATFS_BASE_PATH).
esp_err_t fatfs_list(const char *dir);

// Read size bytes starting at offset from path into buf.
// Sets *bytes_read to the number of bytes actually read.
esp_err_t fatfs_read(const char *path, void *buf, size_t offset, size_t size, size_t *bytes_read);

// Write size bytes from buf to path at offset.
// File must already exist - call fatfs_create() first if needed.
// If offset > file size, the gap content is undefined.
esp_err_t fatfs_write(const char *path, const void *buf, size_t offset, size_t size);

// Create an empty file at path. Truncates if it already exists.
esp_err_t fatfs_create(const char *path);

// Delete the file at path.
esp_err_t fatfs_delete(const char *path);
