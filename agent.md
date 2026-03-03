# BetaSpray — Software Development Approach

## Stack

| Layer | Technology |
|-------|-----------|
| Firmware | ESP-IDF 5.x via PlatformIO (`framework = espidf`) |
| Target | ESP32-S3-DevKitC-1 (WROOM-1-N16R8, 16 MB flash, 8 MB PSRAM) |
| Build system | PlatformIO + CMake (`src/CMakeLists.txt` glob-compiles `src/*.c`) |
| External components | `espressif/esp32-camera`, `espressif/led_strip` (managed components) |

---

## Module structure

Each subsystem is a self-contained `.c` / `.h` pair. All modules live flat
under `src/`. Each has a companion `.md` documenting its public API.

| Module | Files | Responsibility |
|--------|-------|---------------|
| Wi-Fi | `wifi.c/h` | Soft AP (SSID `BetaSpray`, WPA2). Exposes `wifi_init_softap()`. |
| HTTP server | `server.c/h` | `esp_http_server` wrapper. Routes: `POST /route`, `GET /start`, `GET /stop`, `POST /test`. |
| Servo | `servo.c/h` | LEDC PWM, 50 Hz, 14-bit. Up to 8 SG90s on GPIO4–11. One servo at a time for `SERVO_DURATION_MS`. |
| Camera | `camera.c/h` | OV5640 DVP via `esp32-camera`. DMA via LCD_CAM → PSRAM. Capture to BSS / DRAM / PSRAM / stack. |
| FatFS | `fatfs.c/h` | FAT on internal SPI flash via wear-levelling. Chunked read/write with offset+size. |
| Main | `main.c` | Ties everything together. NVS + event loop init, UART frame streaming, LED colour cycle. |

---

## Startup sequence (`app_main`)

```
nvs_flash_init()
esp_event_loop_create_default()
    │
    ├── UART0 config (115200 baud, frame streaming)
    ├── LED strip init (GPIO38, WS2812)
    ├── wifi_init_softap()
    ├── server_start()
    └── camera_init()
            │
            └── main loop: capture → uart_send_frame → LED cycle
```

---

## GPIO allocation

| GPIO | Assignment |
|------|-----------|
| 1 | Camera SDA (SCCB) |
| 2 | Camera PCLK |
| 4–11 | Servo PWM (LEDC CH0–7) |
| 12–18, 21 | Camera D0–D7 |
| 38 | RGB LED (WS2812) |
| 39 | Camera PWDN |
| 40 | Camera RESET |
| 41 | Camera SCL (SCCB) |
| 43, 44 | UART0 TX/RX (serial + frame stream) |
| 47, 48 | Camera VSYNC, HREF |

Avoided: strapping (0, 3, 45, 46), PSRAM SPI (35–37), USB (19, 20), JTAG
(39–41 repurposed for camera; 42 spare).

---

## Flash layout (`partitions.csv`)

```
nvs       0x9000    24 KB
phy_init  0xf000     4 KB
factory   0x10000    1 MB   ← app binary
storage   0x110000 960 KB   ← FAT filesystem
```

Total: 2 MB. `board_build.flash_size = 2MB` in `platformio.ini`.

---

## LEDC usage

| Timer | Channel(s) | User |
|-------|-----------|------|
| `LEDC_TIMER_0` | 0 – `NUM_SERVOS-1` | Servo PWM |
| `LEDC_TIMER_1` | 7 | Camera XCLK (reserved; unused — internal osc) |

---

## DMA

The `esp32-camera` component uses the ESP32-S3 `LCD_CAM` peripheral with
GDMA. Frame buffers are allocated in PSRAM (`CAMERA_FB_IN_PSRAM`) so DMA
writes bypass internal SRAM entirely, leaving it free for the stack, WiFi
buffers, and FreeRTOS.

---

## Build & flash

```sh
pio run                    # build
pio run -t upload          # flash
pio device monitor         # serial monitor at 115200
```

---

## Development conventions

- **One module per subsystem.** No cross-includes between modules except
  through `main.c`.
- **`ESP_ERROR_CHECK` at call sites.** Modules return `esp_err_t`; callers
  decide whether to abort.
- **Logging via `ESP_LOGI/W/E`** with a per-file `TAG`. Log level set to
  `ESP_LOG_ERROR` globally in `main.c` during camera streaming to avoid
  binary frame data being drowned by text.
- **No dynamic reconfiguration at runtime** except through the HTTP server
  endpoints.
- **Pin constraints documented in each header.** Every `*.h` lists why
  specific GPIOs were chosen and what to avoid.

---

## Per-module docs

- [`src/server.md`](src/server.md) — HTTP API endpoints
- [`src/servo.md`](src/servo.md) — servo PWM API and pin table
- [`src/camera.md`](src/camera.md) — camera init, config, capture, DMA notes
- [`src/fatfs.md`](src/fatfs.md) — filesystem API and partition layout
