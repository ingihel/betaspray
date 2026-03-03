# FatFS API

FAT filesystem backed by the ESP32-S3's internal SPI flash, using ESP-IDF wear
levelling. Mounted at `/fatfs` from the `storage` partition.

---

## Partition layout (`partitions.csv`)

| Partition | Type | Offset | Size |
|-----------|------|--------|------|
| nvs | data/nvs | 0x9000 | 24 KB |
| phy_init | data/phy | 0xf000 | 4 KB |
| factory | app | 0x10000 | 1 MB |
| storage | data/fat | 0x110000 | 960 KB |

Total flash used: 2 MB. The `storage` partition provides ~952 KB of usable
space after wear-levelling overhead.

---

## Configuration macros (`fatfs.h`)

| Macro | Value | Description |
|-------|-------|-------------|
| `FATFS_BASE_PATH` | `"/fatfs"` | VFS mount point |
| `FATFS_PARTITION` | `"storage"` | Partition label in `partitions.csv` |
| `FATFS_MAX_FILES` | `5` | Max simultaneously open file handles |

---

## API

### `fatfs_init(bool format_if_empty)`

Mount the `storage` partition. Call once at startup before any other fatfs
calls.

Pass `true` to auto-format on the first boot (when no filesystem is found).
Pass `false` to fail instead — useful if you want to detect a missing/corrupt
filesystem explicitly.

```c
ESP_ERROR_CHECK(fatfs_init(true));
```

---

### `fatfs_reset()`

Format the partition and remount. **Deletes all stored files.**

```c
ESP_ERROR_CHECK(fatfs_reset());
```

---

### `fatfs_list(const char *dir)`

Log all files in a directory to the ESP log.

```c
fatfs_list(FATFS_BASE_PATH);
// I (123) fatfs: Files in /fatfs:
// I (123) fatfs:   route.bin
// I (123) fatfs:   config.txt
```

---

### `fatfs_create(const char *path)`

Create a new empty file. Truncates the file if it already exists.
Must be called before `fatfs_write()` on a new path.

```c
fatfs_create(FATFS_BASE_PATH "/route.bin");
```

---

### `fatfs_delete(const char *path)`

Delete a file.

```c
fatfs_delete(FATFS_BASE_PATH "/route.bin");
```

---

### `fatfs_read(const char *path, void *buf, size_t offset, size_t size, size_t *bytes_read)`

Read `size` bytes from `path` starting at `offset` into `buf`. Sets
`*bytes_read` to the number of bytes actually read (may be less than `size`
near end of file).

| Parameter | Description |
|-----------|-------------|
| `path` | Full path including mount point |
| `buf` | Destination buffer |
| `offset` | Byte offset into the file |
| `size` | Max bytes to read |
| `*bytes_read` | Actual bytes read (out) |

```c
uint8_t buf[64];
size_t n;
fatfs_read(FATFS_BASE_PATH "/route.bin", buf, 0, sizeof(buf), &n);   // first 64 bytes
fatfs_read(FATFS_BASE_PATH "/route.bin", buf, 128, sizeof(buf), &n); // bytes 128–191
```

---

### `fatfs_write(const char *path, const void *buf, size_t offset, size_t size)`

Write `size` bytes from `buf` into `path` at `offset`. The file must already
exist — call `fatfs_create()` first. If `offset` is beyond the current end of
file, the gap content is undefined.

| Parameter | Description |
|-----------|-------------|
| `path` | Full path including mount point |
| `buf` | Source buffer |
| `offset` | Byte offset into the file |
| `size` | Bytes to write |

```c
uint8_t data[256] = { ... };
fatfs_write(FATFS_BASE_PATH "/route.bin", data, 0, 128);          // write first chunk
fatfs_write(FATFS_BASE_PATH "/route.bin", data + 128, 128, 128);  // write second chunk
```

---

## Full example

```c
#include "fatfs.h"

// Startup
fatfs_init(true);

// Write a file in two chunks
const char *path = FATFS_BASE_PATH "/route.bin";
uint8_t chunk_a[128] = { /* hold data part 1 */ };
uint8_t chunk_b[128] = { /* hold data part 2 */ };

fatfs_create(path);
fatfs_write(path, chunk_a, 0,   sizeof(chunk_a));
fatfs_write(path, chunk_b, 128, sizeof(chunk_b));

// Read it back
uint8_t readbuf[256];
size_t n;
fatfs_read(path, readbuf, 0, sizeof(readbuf), &n);

// Inspect and clean up
fatfs_list(FATFS_BASE_PATH);
fatfs_delete(path);
```
