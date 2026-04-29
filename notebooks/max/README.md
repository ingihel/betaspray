# Max's Lab Notebook

---

### Feb 24, 2026

**PCB — Servo control schematic**

Started laying out the servo control subsystem in KiCad. Added servo control schematic and created a dedicated `servo_subsystem` sheet, separating it from the main schematic for clarity. Also added custom connector symbols to `betaspray_Connector.kicad_sym`.

Commits: `353742f` (add servo control), `2e1eb66` (add servo_subsystem)

---

### Feb 26, 2026

**PCB — Added tracks**

Added routing tracks to the PCB layout.

Commit: `061b5df`

---

### Mar 2, 2026

**PCB — Schematic revisions and bulk capacitors for servos**

Updated power and servo subsystem schematics. Added bulk decoupling capacitors on the servo supply rails. Worked on full PCB layout revision (Ver0).

Commits: `89470c7` (added bulk CAP servos), `fc46deb` (Ver0, pls')

---

### Mar 3, 2026

**Firmware — FatFS integration**

Merged in FatFS support: added `fatfs.c/h` and `fatfs.md`, updated `platformio.ini`, and wired `fatfs_init()` into `main.c`. This enables wear-levelled FAT on internal flash mounted at `/fatfs`, used for route file storage.

Commit: `f3b38aa` (cooking with propane or methane Idk)

---

### Mar 4, 2026

**Camera — Lens distortion documentation**

Added `src/Camera_Distortion.md` documenting the OV5640 lens distortion model, the calibration approach, and how the distortion coefficients are used. Kevin helped with this.

Commit: `76ab29b` (Added lens distortion Guide (ty Kevin))

---

### Mar 5, 2026

**Camera — Calibration tooling and conf.h intrinsics**

Added `calibrate_camera.py` (checkerboard-based intrinsic calibration using OpenCV), partition table (`partitions.csv`), and camera intrinsic constants in `conf.h`. Noted that PSRAM is needed for fast/reliable calibration — the dev board DRAM is too limited for reliable multi-frame capture.

Also expanded `Camera_Distortion.md` with calibration procedure details.

Commit: `89bbbd2`

---

### Mar 5, 2026

**Firmware — GPIO fix for servo 0, servo pin reassignment**

Discovered that the originally assigned GPIO for servo 0 conflicted with camera pins. Moved `SERVO_PIN_0` to GPIO1 and `SERVO_PIN_1` to GPIO2, which are safe (no camera or strapping pin conflicts). Updated `agent.md` with the corrected pin map.

Commit: `133a38b`

---

### Mar 5, 2026

**Firmware — Servo driver fully working on pins 1 and 2**

Got servo test functions working on GPIO1 (X-axis) and GPIO2 (Y-axis). Added hardware fade + hold logic to `servo.c`, updated `servo.h` with the new API, and updated `servo.md`. Also wired servo testbench calls into `main.c` startup sequence.

PCB Gerbers generated and 2-layer board submitted for fabrication.

Commits: `bfce7ad` (Servo test functions working for pins 1,2), `e904c0f` (2 layer PCB submitted)

---

### Mar 7, 2026 *(team context — Ingi's notebook)*

**CV — Difference of Gaussian blob detector ready**

Ingi completed a DoG-based blob detector (`differenceofgaussian.c`) for climbing hold detection. Runs on host (gcc, libjpeg), designed to be portable to ESP32-S3 later. Algorithm: multi-octave Gaussian pyramid, DoG layers, extremum detection, NMS. Ingi also extended `client.py` to support an image viewer and sending detected blob coordinates directly as a route file to the ESP32.

Noted that the dev board (no PSRAM) can't run this on-device yet — needs PSRAM headroom for the DoG scale buffers at 320×240. The PCB (N16R8) will unblock on-device CV.

---

### Mar 5 – Mar 31, 2026

*No notebook entries or commits from Max in this period. Team was likely integrating servo + camera subsystems and preparing for design document deadlines. See `git log` for other contributors' activity.*

---

### Apr 1, 2026

**Camera — Tiled high-res capture for low-memory dev board**

The N8 dev board has no PSRAM, so a full UXGA (1600×1200) frame can't be captured in one shot. Implemented a tiling approach: the OV5640 sensor crop window is set via timing registers (0x3800–0x3807) to capture a QVGA-sized tile at 1:1 native pixels from a different region of the full 2592×1944 active array. The ISP scales the crop to the fixed QVGA DMA buffer, keeping DMA timing unchanged.

New `/capture/tile` endpoint takes `{"col": N, "row": N}` and returns the tile JPEG with `X-Tile-Grid` and `X-Tile-Pos` response headers. Default grid is 8×8, producing a stitched 2560×1920 image.

Added `camera_set_window()` and `camera_reset_window()` to `camera.c/h`. Added `CAMERA_USE_PSRAM`, `CAMERA_TILE_COLS/ROWS`, and OV5640 array constants to `conf.h`. Updated `client.py` with `capture_tile()` / `capture_tiled()` methods and a `capture-tiled` CLI subcommand (uses Pillow for stitching).

Key constraint: AE/AWB re-converges per tile. For consistent exposure, lock AE before the first tile. Subsampling and binning registers are intentionally not touched at runtime to avoid losing DMA sync.

Commit: `2a6c89f`

---

### Apr 14, 2026

**Camera — JPEG quality floor established**

Through on-device testing, determined that the minimum viable JPEG quality setting on the OV5640 is **3** (on the firmware's 0–63 scale, where 0 = best quality, 63 = most compressed). Values below 3 produce artifacts severe enough to break the hold detection pipeline. Camera is running at full 5MP (2592×1944) for maximum detection accuracy.

At 5MP a decoded RGB888 frame is ~15 MB, which exceeds the 8 MB PSRAM ceiling — on-device image processing is not feasible at this resolution. All CV work runs host-side.

**CV — Ported ImageJ hold detector to Python/OpenCV in `client.py`**

Ported `Betaspray-hold-extractor.ijm` to a native Python/OpenCV pipeline (`detect_holds()` in `client.py`). Pipeline mirrors the IJM: median blur → Gaussian background subtraction (approximates rolling ball r=100) → CLAHE → second median + despeckle → HSV threshold (V ≤ 183) → fill holes → morphological open → connected components (area ≥ 150 px). Outputs a list of hold centroids and an annotated image.

Added `select_holds_interactive()`: matplotlib window showing detected holds overlaid on the captured image. Clicking toggles selection (red → green); closing the window confirms and optionally sends the hold list to the ESP32 via `POST /route/create`.

Added `detect` command to the `client.py` interactive shell.

**App — Wired CV pipeline into `app.py` web UI**

Modified `app.py` to replace the DoG binary subprocess in `/detect` with a direct call to `detect_holds()` imported from `client.py`. "New Route (detect holds)" in the web UI now runs the full OpenCV pipeline server-side. Mapping panel kept in codebase but hidden from the UI.

All work on branch `codev2`.

Commits: `ac1fb4f` (pushed image quality jpeg=3, added hold recognition pipeline), `404f85c` (flask app now?), `32907e7` (app.py modified to run hold detection)

---

### Apr 19, 2026

**App — Upload image feature for offline hold detection**

Added an "Upload Image" button to the `app.py` web UI so wall images can be loaded without a live camera. A hidden `<input type="file">` accepts any JPEG/PNG; on selection the file is loaded into the canvas as `wallImage` via `URL.createObjectURL`. The existing "New Route (detect holds)" button then runs detection on the uploaded image using the same `canvas.toBlob` → `/detect` pipeline as live captures.

**CV — Fixed hold detection pipeline in `client.py`**

Two bugs in `detect_holds()` were causing only 1 centroid to be returned (always near the image center):

1. **Background subtraction algebra error** — the first implementation simplified algebraically to `GaussianBlur(working)` rather than true background removal. Fixed to correctly mirror ImageJ "light background" rolling-ball: `inv = 255 - working; result_inv = clip(inv - GaussianBlur(inv), 0, 255); working = 255 - result_inv`.

2. **Missing watershed step** — the port of `Betaspray-hold-extractor.ijm` omitted `run("Watershed")` (IJM line 67). Added OpenCV distance-transform watershed after the morphological open: local maxima of the distance transform seed markers, then `cv2.watershed()` splits touching blobs.

After both fixes, detection on `test.jpg` (2560×1709) returns 200+ hold candidates spread across the full image, matching the IJM reference output.

---

### Apr 20, 2026

**Firmware — Leapfrog route playback mode**

Redesigned route playback from a simple sequential model (all gimbals move together, hold by hold) to a leapfrog model where each gimbal advances independently through the hold list. This keeps N gimbals always pointing at N adjacent holds simultaneously, so the wall is never dark while servos are moving.

Key implementation decisions:
- `drive_gimbal(g, hold_idx, ...)` helper encapsulates pixel→angle conversion, poly correction, and both servo drives for one gimbal-to-hold assignment
- `s_leap_hold[NUM_SERVOS/2]` tracks each gimbal's current hold index independently; `s_leap_next` round-robins which gimbal advances on each `/route/next` trigger
- At route start, gimbals are staggered with a 300 ms delay between each initialization to prevent PSU voltage droop from simultaneous inrush current
- Both manual (wait for `/route/next`) and timed (interval_ms from the play request) advance modes supported

Servo direction and axis corrections also fixed this session — X/Y mappings in `route.c` were inverted for some gimbals.

Commit: `e394266` (3 gimballs moving leapfrog type shit), `614a61a` (servo directions correct)

---

### Apr 23, 2026

**Firmware/App — Gimbal calibration system and safe zero ordering**

Added a full per-gimbal pointing calibration workflow:

*Firmware (`server.c`, `route.c`):*
- `POST /gimbal/zero` zeros all gimbals in a mechanically safe sequence to prevent physical clashes on return. Order: gimbal 0 first, then gimbal 2 (back-left before back-right), then gimbal 1. This mirrors the inward-before-outward pattern on the front row.
- `POST /gimbal/offset` stores a per-gimbal signed degree correction `(dx, dy)` and applies it during playback

*App (`app.py`):*
- New gimbal calibration panel in the web UI: "Enter Cal Mode" zeros gimbals, then clicking anywhere on the canvas drives the selected gimbal to that pixel position
- Arrow nudge buttons (1°/2°/5° steps) let you fine-tune until the laser hits the intended spot; "Save Offset" persists the correction to `gimbal_offsets.json`
- `/gimcal/point` endpoint proxies to ESP `POST /gimbal/point` so the same geometric formula used at playback is used during calibration
- Rate limiter bypassed during calibration (`/gimcal/drive` uses direct `requests.post`)

Commit: `f9d279a` (added gimbal calibration and zero ordering)

---

### Apr 25, 2026

**Firmware — Geometric pixel→angle projection formula**

Replaced the old linear pixel→angle approximation with a proper geometric projection that accounts for camera position relative to each gimbal:

Old formula (linear, wrong at wide FOV):
```
angle_x = (center_x - px) * (HFOV / width) + 90
```

New formula (geometric, matches atan2 projection at any FOV):
1. Scale OV5640 intrinsics from calibration resolution (320×240) to current image resolution
2. Project pixel onto wall plane at distance `dist`: `wall_x = (px - cx)/fx * dist`, `wall_z = (cy - py)/fy * dist`
3. Compute vector from gimbal's physical position to the wall point
4. `servo_x = 90 - degrees(atan2(dx, dy))` for pan, `servo_y = 90 + degrees(atan2(dz, dy))` for tilt

Gimbal physical positions added to `conf.h` in a right-hand coordinate frame (+X right, +Y toward wall, +Z up). Initial estimates: front row 3.5 cm behind camera, back row 7 cm behind/above. Columns ±1.75 cm from center.

Commit: `03a6e8a` (Some servo to camera offset calibration — Prakhar)

---

**Hardware — Measured and corrected gimbal physical positions**

The initial conf.h estimates were off. After measuring the assembled enclosure:
- Front gimbals: 6 cm behind camera (Y), 3.5 cm above (Z) — Y was nearly double the estimate
- Back gimbals: 13 cm behind (Y), 13 cm above (Z) — both roughly double

Updated `conf.h` accordingly. These values feed directly into the atan2 angle computation so accuracy scales with how precisely they're measured.

Also this session: gimbal 0 (front-right, old servos 0/1) suffered a mechanical failure and was taken out of service. `NUM_SERVOS` reduced from 8 to 6; `servo_pins[]` remapped to skip old pins 0/1 and start at SERVO_PIN_2. Leapfrog updated to 3 gimbals.

Discovered that old servos 1 and 3 (Y-axis of the two front gimbals) are mounted 45° offset from the rear Y-axis servos — they sit mechanically flat at 45° PWM rather than 90°. Added a `servo_offset[]` table in `servo.c` and a `physical_angle()` helper so all callers can treat 90° as flat regardless of individual mounting. The offset is applied just before duty cycle conversion, keeping commanded-angle space consistent throughout the codebase.

Commit: `418faee` (changed offsets), `cf32e63` (new calibratio setup)

---

**Firmware/App — Bilinear polynomial correction system**

A constant per-gimbal offset (dx, dy) can only correct a uniform pointing bias. It cannot handle errors that vary across the frame — parallax, residual lens distortion, or axis coupling (e.g. tilting the Y servo slightly deflecting the X axis). Added a bilinear polynomial correction layer on top of the geometric baseline.

*Model (8 coefficients per gimbal):*
```
corrected_x = raw_x + (a0 + a1·raw_x + a2·raw_y + a3·raw_x·raw_y)
corrected_y = raw_y + (b0 + b1·raw_x + b2·raw_y + b3·raw_x·raw_y)
```
- `a0/b0`: constant bias (subsumes the old per-gimbal offset)
- `a1/b1`: scale error along each axis
- `a2/b2`: cross-axis bleed (Y movement affects X and vice versa)
- `a3/b3`: position-dependent coupling

*Firmware (`route.c`, `server.c`):*
- `apply_poly_correction()` runs at playback for every hold
- Coefficients persist to FatFS (`/fatfs/gimbal_poly.bin`) and reload on boot
- `POST /gimbal/poly` sets coefficients; `GET /gimbal/poly` reads them back
- `POST /gimbal/point` drives gimbal to a pixel using raw geometric angles only (no correction), returns computed angles — used during calibration data collection

*Host (`app.py`):*
- `_pixel_to_servo_angles_geometric()` mirrors the firmware formula exactly (using same conf.h constants and gimbal positions) so angle computation is consistent on both sides
- `/gimcal/poly/collect`: accepts `{target_px, target_py, actual_px, actual_py, img_w, img_h}`, computes angles for both pixels, stores the angular deviation as a calibration point
- `/gimcal/poly/fit`: least-squares fit using design matrix `[1, rx, ry, rx·ry]`; residual is `target_angle - actual_angle`; sends result to ESP

New poly calibration UX: enter Poly Cal mode → 9-point crosshair grid overlaid on canvas → click crosshair to drive gimbal + auto-capture fresh image → click where laser dot actually appears → point recorded. Fit once ≥4 points collected per gimbal.

**Laser control** integrated with route playback: laser turns off during gimbal zero sequence; turns on only after the first set of holds is aimed at during leapfrog initialization.

Commit: `aadfa3e` (polyfit laser — Prakhar), `cf32e63` (new calibratio setup)

---

### Apr 28, 2026

**Firmware — Camera frame staleness fix**

With `fb_count=1` and `CAMERA_GRAB_WHEN_EMPTY` mode, the OV5640 DMA continuously fills the single frame buffer. When `POST /capture` arrived and called `esp_camera_fb_get()`, it returned whatever frame was already queued — captured *before* the HTTP request, meaning before any recent servo movement or laser state change. Getting a fresh image required sending two consecutive capture+get pairs.

Root cause: `esp_camera_fb_get()` returns immediately if a completed frame is already in the buffer. There is no way to tell the DMA "start a new frame from now."

Fix in `camera_capture_frame()`: discard the queued stale frame first, then block on the next `get` which waits for a genuinely new frame from the sensor:

```c
camera_fb_t *stale = esp_camera_fb_get();
if (stale) esp_camera_fb_return(stale);
camera_fb_t *fb = esp_camera_fb_get();  // blocks until post-request frame is ready
```

One single `POST /capture` + `GET /get` now reliably returns a current frame. The 400 ms sleep that was compensating for this in the poly-cal capture flow was also removed.

Commit: `8d18d36` (solve frame update issue)
