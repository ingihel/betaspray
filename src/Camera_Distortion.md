# OV5640 Lens Distortion — Research Notes

## What Distortion to Expect

The OV5640 sensor itself introduces **no distortion** — distortion is entirely a property of the
**lens module** attached to it. The OV5640 is commonly sold in modules with cheap wide-angle
plastic lenses, which typically exhibit:

- **Radial distortion** (usually barrel) — the dominant effect. Straight lines bow outward from center.
- **Tangential distortion** — minor, from lens/sensor non-parallelism during assembly.

The standard model (Brown-Conrady) parameterizes this as:

```
Distortion coefficients: [k1, k2, p1, p2, k3]
  k1, k2, k3 — radial terms
  p1, p2     — tangential terms

Corrected coordinates:
  x' = x(1 + k1*r² + k2*r⁴ + k3*r⁶) + 2p1*xy + p2(r² + 2x²)
  y' = y(1 + k1*r² + k2*r⁴ + k3*r⁶) + p1(r² + 2y²) + 2p2*xy
```

---

## How to Determine If Distortion Is Significant

### Qualitative (quick check)

Point the camera at a flat grid or doorframe and capture a frame. If straight lines near the image
edges curve noticeably, correction is needed. For a climbing wall application, hold edges appearing
curved near frame borders would directly affect projection accuracy.

### Quantitative (proper calibration)

Use OpenCV's `calibrateCamera()` with a checkerboard pattern (Zhang method). Collect 10-20 frames
from different angles/positions via the UART stream.

```python
import cv2, numpy as np, glob

# Capture 10-20 frames of a checkerboard (e.g., 9x6 inner corners)
# from different angles/positions via the HTTP or UART stream

objp = np.zeros((6*9, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

obj_points, img_points = [], []

for img_path in glob.glob('calibration_frames/*.jpg'):
    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)
    if ret:
        obj_points.append(objp)
        img_points.append(corners)

ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points, img_points, gray.shape[::-1], None, None
)
# K    = intrinsic matrix (fx, fy, cx, cy)
# dist = [k1, k2, p1, p2, k3]
print("Distortion coefficients:", dist)
```

A `k1` magnitude > 0.05 generally warrants correction for precision applications.

---

## Is the Matrix Constant or Variable?

**For a fixed-focus OV5640 module: the coefficients are constant** — they are a physical property
of the optics and do not change between power cycles.

However, there are OV5640-specific factors that affect this:

| Factor                                | Effect on Distortion                                                                                                                              |
| ------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| Resolution change (e.g., QVGA vs 5MP) | Coefficients scale proportionally — same optical distortion, different pixel coordinates. Must rescale K matrix; dist coefficients stay the same. |
| OV5640 internal ISP cropping          | Changes effective field of view, altering cx/cy and focal length, but **not** k1/k2/k3                                                            |
| Temperature                           | Negligible for plastic lenses in typical indoor range                                                                                             |
| Variable-focus / motorized lenses     | Coefficients change with focus distance — not typical for OV5640 modules                                                                          |
| Manufacturing variance between units  | Different physical units **will** have different coefficients — per-unit calibration needed for multi-device builds                               |

**Conclusion:** Calibrate once per physical unit, store the 9 numbers (4 intrinsic + 5 distortion),
and apply at runtime. No recalibration on power cycle is needed.

---

## Practical Calibration Workflow for BetaSpray

1. **Collect frames**: Capture 15-20 JPEG frames from the UART stream pointing at a printed
   checkerboard at various angles/distances.
2. **Run calibration**: Obtain `K` (intrinsic matrix) and `dist` (distortion coefficients).
3. **Store coefficients**: Hardcode in `conf.h`, or flash via a `/calibrate` endpoint to FatFS
   once storage is re-enabled.
4. **Apply correction**: Either on the ESP32 (expensive at QVGA) or on the receiving host before
   further image processing.

---

## Projection Pipeline Consideration

For a projection system, lens undistortion is only step 1. The full pipeline to map camera
observations to projector output is:

```
Wall → Camera (lens distortion) → Undistort → Homography → Projector pixel
```

A **projector-camera homography** is needed to account for perspective geometry between the
camera's viewpoint and the projector's throw angle onto the wall. This is a separate calibration
step from lens distortion correction.

============================================================
RMS reprojection error: 0.7909 px (< 1.0 is good)
Calibration resolution: 320x240

Intrinsic matrix K:
fx=143.7559 fy=141.1766
cx=156.0185 cy=122.1421

Distortion coefficients [k1,k2,p1,p2,k3]:
[-0.23760211 0.23839472 0.0072167 0.00108888 -0.11321597]
============================================================

/_ --- paste into conf.h --- _/
#define CALIB_WIDTH 320
#define CALIB_HEIGHT 240
#define CALIB_FX 143.755937f
#define CALIB_FY 141.176552f
#define CALIB_CX 156.018487f
#define CALIB_CY 122.142135f
#define CALIB_K1 -0.237602f
#define CALIB_K2 0.238395f
#define CALIB_P1 0.007217f
#define CALIB_P2 0.001089f
#define CALIB_K3 -0.113216f
/\* ---- scaling K for a different resolution ----

- scale = new_width / CALIB_WIDTH
- fx_new = CALIB_FX _ scale fy_new = CALIB_FY _ scale
- cx_new = CALIB_CX _ scale cy_new = CALIB_CY _ scale
- dist coefficients stay the same \*/

---

## On-Device Undistortion

Since the client is unknown, undistortion must happen on the ESP32 before sending each frame.
The camera should be configured to output a raw format (`PIXFORMAT_GRAYSCALE` or `PIXFORMAT_RGB565`)
to avoid a JPEG decode step. The pipeline then becomes:

```
Raw frame → remap → frame2jpg() → send
```

`frame2jpg()` is already available in the `esp32-camera` component.

At 1 fps the remap can be computed on-the-fly per frame (no table needed) — 76,800 pixels at
240 MHz is well under 100 ms.

---

## Remap Table Options (PSRAM Required)

When PSRAM with sufficient space is available, a precomputed remap table eliminates per-frame
math. The table is computed once at startup from the `conf.h` constants using the Brown-Conrady
inverse formula, allocated in PSRAM via `heap_caps_malloc(size, MALLOC_CAP_SPIRAM)`.

| Option | Table Size | Per-frame Cost | Quality |
|---|---|---|---|
| Nearest-neighbour LUT | ~150 KB — one `uint16` index per output pixel | Pure lookup, no math | Low — aliasing visible |
| Fixed-point int16 maps | ~307 KB — two `int16` maps (x, y) at sub-pixel precision | Integer bilinear interp | Good |
| Float32 maps | ~614 KB — two `float32` maps (x, y) | Float bilinear interp | Best |
| Packed bilinear LUT | ~1.2 MB — precompute 4 neighbour indices + weights per pixel | Pure table lookup, no per-frame math | Best, fastest per-frame |

**Recommendation:** Fixed-point int16 maps are the sweet spot — ~307 KB, integer math only,
and at QVGA the precision loss vs float32 is imperceptible.

The packed bilinear LUT is optimal if minimising per-frame CPU matters more than PSRAM usage.
