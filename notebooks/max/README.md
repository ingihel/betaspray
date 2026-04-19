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
