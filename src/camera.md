# Camera API

OV5640 DVP camera driver wrapping the `espressif/esp32-camera` component.
DMA is handled automatically by the ESP32-S3's `LCD_CAM` peripheral — no
manual DMA configuration is required.

---

## GPIO pin assignments (`camera.h`)

| Signal | GPIO | Notes |
|--------|------|-------|
| PWDN | 39 | Active high. JTAG MTCK repurposed. |
| RESET | 40 | Active low. JTAG MTDO repurposed. |
| XCLK | — | Not connected. OV5640 uses internal 24 MHz oscillator. |
| SDA | 1 | SCCB / I²C data |
| SCL | 41 | SCCB / I²C clock. JTAG MTDI repurposed. |
| D0–D7 | 12–18, 21 | DVP parallel data bus |
| VSYNC | 47 | Vertical sync |
| HREF | 48 | Horizontal reference |
| PCLK | 2 | Pixel clock |

> **Note:** GPIO39–41 are JTAG pins. Repurposing them for the camera means
> hardware JTAG debugging is unavailable. Verify all assignments match the
> KiCad schematic before flashing.

---

## DMA recommendation

The `esp32-camera` component uses the ESP32-S3's `LCD_CAM` peripheral with
**GDMA (Generic DMA)** to transfer pixel data from the DVP bus into frame
buffers without CPU involvement. This is enabled automatically on init.

To maximise DMA benefit, keep `fb_location = CAMERA_FB_IN_PSRAM` (the
default). This lets DMA write frames directly into the 8 MB PSRAM, keeping
the 512 KB internal SRAM free for the stack, WiFi buffers, and other tasks.
Avoid `CAMERA_FB_IN_DRAM` unless PSRAM is unavailable.

---

## LEDC conflict avoidance

The servo driver uses `LEDC_TIMER_0` and channels `0–NUM_SERVOS-1`. The
camera driver is configured to use `LEDC_TIMER_1` / `LEDC_CHANNEL_7`. Since
`CAM_PIN_XCLK = -1` (internal oscillator), LEDC is never actually started by
the camera — but the non-conflicting assignment is kept as a safeguard.

---

## API

### `camera_reset()`

Hardware-resets the OV5640 by pulling RESET low for 10 ms then releasing.
Call before `camera_init()` if the sensor is in an unknown state.

```c
camera_reset();
```

---

### `camera_init()`

Initialise the camera with defaults: QVGA (320×240) JPEG, 2 frame buffers in
PSRAM, DMA enabled via LCD_CAM.

```c
ESP_ERROR_CHECK(camera_init());
```

---

### `camera_config(res, fmt, sccb_freq_hz)`

Reconfigure resolution, pixel format, and SCCB clock. Performs a full
de-init + re-init cycle.

| Parameter | Type | Description |
|-----------|------|-------------|
| `res` | `framesize_t` | e.g. `FRAMESIZE_QVGA`, `FRAMESIZE_VGA` |
| `fmt` | `pixformat_t` | e.g. `CAMERA_FORMAT_JPEG`, `CAMERA_FORMAT_RGB565` |
| `sccb_freq_hz` | `int` | Accepted but not applied — `camera_config_t` does not expose this field. The component defaults to 100 kHz internally. |

XCLK is not a parameter. The OV5640 uses its internal 24 MHz oscillator
(`CAM_PIN_XCLK = -1`); the ESP32-S3 does not generate a clock signal.

```c
camera_config(FRAMESIZE_VGA, CAMERA_FORMAT_JPEG, 100000);
```

---

### `camera_set_resolution(size)` / `camera_set_format(fmt)`

Change resolution or pixel format live via OV5640 sensor register writes.
No re-init needed — takes effect on the next captured frame.

```c
camera_set_resolution(FRAMESIZE_QVGA);
camera_set_format(CAMERA_FORMAT_JPEG);
```

---

### `camera_capture_frame()` / `camera_return_frame(fb)`

Capture a frame into the esp32-camera DMA-managed double buffer. **Must** be
returned with `camera_return_frame()` promptly — holding the buffer blocks
DMA from reusing it.

```c
camera_fb_t *fb = camera_capture_frame();
if (fb) {
    // use fb->buf, fb->len, fb->width, fb->height
    camera_return_frame(fb);
}
```

---

### `camera_click_pic(dest, &out_buf, &out_len)`

Capture a frame and copy it into a specific memory region. Useful when you
need to hold the image data independently of the DMA buffer.

| `dest` | Behaviour |
|--------|-----------|
| `CAMERA_MEM_BSS` | Copies into a 40 KB static `.bss` buffer. No allocation. Not thread-safe — overwritten on next call. |
| `CAMERA_MEM_DATA_RAM` | `malloc()` from internal DRAM. **Caller must `free()`**. |
| `CAMERA_MEM_PSRAM` | `heap_caps_malloc()` from PSRAM. **Caller must `free()`**. |
| `CAMERA_MEM_STACK` | `*out_buf` must already point to a caller-allocated buffer of sufficient size. Nothing is allocated. |

```c
// BSS — no alloc, just a pointer into the static buffer
uint8_t *buf = NULL;
size_t   len = 0;
camera_click_pic(CAMERA_MEM_BSS, &buf, &len);

// PSRAM — heap allocated, caller frees
camera_click_pic(CAMERA_MEM_PSRAM, &buf, &len);
// ... use buf ...
free(buf);

// STACK — caller owns the buffer
uint8_t stack_buf[40 * 1024];
uint8_t *ptr = stack_buf;
camera_click_pic(CAMERA_MEM_STACK, &ptr, &len);
```

`CAMERA_BSS_BUF_SIZE` (default 40 KB) must be >= largest expected frame.
Increase it in `camera.h` if using resolutions above QVGA.

---

## Pixel formats

| Alias | Underlying | Notes |
|-------|------------|-------|
| `CAMERA_FORMAT_JPEG` | `PIXFORMAT_JPEG` | Smallest frames, lossy. Quality 0–63 (lower = better). |
| `CAMERA_FORMAT_RGB565` | `PIXFORMAT_RGB565` | Raw, 2 bytes/pixel. Large frames. |
| `CAMERA_FORMAT_GRAYSCALE` | `PIXFORMAT_GRAYSCALE` | 1 byte/pixel. Good for CV pipelines. |

## Common resolutions (`framesize_t`)

| Constant | Size |
|----------|------|
| `FRAMESIZE_QQVGA` | 160×120 |
| `FRAMESIZE_QVGA` | 320×240 |
| `FRAMESIZE_VGA` | 640×480 |
| `FRAMESIZE_SVGA` | 800×600 |
| `FRAMESIZE_UXGA` | 1600×1200 |
