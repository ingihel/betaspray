# Camera API

OV5640 DVP camera driver wrapping the `espressif/esp32-camera` component.
DMA is handled automatically by the ESP32-S3's `LCD_CAM` peripheral - no
manual DMA configuration is required.

---

## GPIO pin assignments (`camera.h`)

| Signal | GPIO | Notes |
|--------|------|-------|
| PWDN | 39 | Active high. JTAG MTCK is repurposed. |
| RESET | 40 | Active low. JTAG MTDO is repurposed. |
| XCLK | - | Not connected. OV5640 uses internal 24 MHz oscillator. |
| SDA | 1 | SCCB / I2C data |
| SCL | 41 | SCCB / I2C clock. JTAG MTDI repurposed. |
| D0-D7* | 12-18, 21 | DVP parallel data bus |
| VSYNC | 47 | Vertical sync |
| HREF | 48 | Horizontal reference |
| PCLK | 2 | Pixel clock |

> **Note:** D0-D7 are labeled as D2-D9 on the Adafruit breakout board

> **Note:** GPIO39-41 are JTAG pins. Repurposing them for the camera means
> hardware JTAG debugging is unavailable with this configuration.

##### These values need to change whenever ESP32 pin assignments change.

---

## DMA recommendation

The `esp32-camera` component uses the ESP32-S3's `LCD_CAM` peripheral with
**GDMA (Generic DMA)** to transfer pixel data from the DVP bus into frame
buffers without CPU involvement. This is enabled automatically on init.

To maximise DMA benefit, we would prefer `fb_location = CAMERA_FB_IN_PSRAM` (the
default). This lets DMA write frames directly into the 8 MB PSRAM, keeping
the 512 KB internal SRAM free for the stack, WiFi buffers, and other tasks.
Avoid `CAMERA_FB_IN_DRAM` unless PSRAM is unavailable.

Note: this is unavoidable with our current setup, since this devkit SKU does not have
PSRAM. D:

---

## LEDC conflict avoidance

The servo driver uses `LEDC_TIMER_0` and channels `0-NUM_SERVOS-1`. The
camera driver is configured to use `LEDC_TIMER_1` / `LEDC_CHANNEL_7`. Since
`CAM_PIN_XCLK = -1` (internal oscillator), LEDC is never actually started by
the camera - but the non-conflicting assignment is kept as a safeguard.

---

## API

### `camera_reset()`

Hardware-resets the OV5640 by pulling RESET low for 10 ms then releasing.
Call before `camera_init()` if the sensor is in an unknown state.

It's not clear that we need this, but why not.

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

### `camera_set_resolution(size)` / `camera_set_format(fmt)`

Change resolution or pixel format live via OV5640 sensor register writes.
No re-init needed - takes effect on the next captured frame.

```c
camera_set_resolution(FRAMESIZE_QVGA);
camera_set_format(CAMERA_FORMAT_JPEG);
```

---

### `camera_capture_frame()` / `camera_release_frame(fb)`

Capture a frame into the esp32-camera DMA-managed double buffer. **Must** be
returned with `camera_release_frame()` promptly - holding the buffer blocks
DMA from reusing it.

```c
camera_fb_t *fb = camera_capture_frame();
if (fb) {
    // use fb->buf, fb->len, fb->width, fb->height
    camera_release_frame(fb);
}
```
## Common resolutions (`framesize_t`)

| Constant | Size |
|----------|------|
| `FRAMESIZE_QQVGA` | 160×120 |
| `FRAMESIZE_QVGA` | 320×240 |
| `FRAMESIZE_VGA` | 640×480 |
| `FRAMESIZE_SVGA` | 800×600 |
| `FRAMESIZE_UXGA` | 1600×1200 |
