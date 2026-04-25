#!/usr/bin/env python3
"""
BetaSpray Web Application
Flask server that serves the frontend UI and proxies requests to the ESP32-S3.
Optionally runs hold detection locally via the DoG pipeline.

Usage:
    python app.py                          # default: ESP at 192.168.4.1, host detection
    python app.py --esp 192.168.4.1        # specify ESP address
    python app.py --detect esp             # use on-device detection instead
    python app.py --port 8080              # serve on a different port
"""

import argparse
import collections
import io
import json
import struct
import subprocess
import sys
import tempfile
import threading
import time
import traceback
from pathlib import Path
from typing import List, Tuple

import requests
from flask import Flask, Response, jsonify, request, send_file

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    from client import detect_holds as _cv_detect_holds
    HAS_CV_PIPELINE = True
except ImportError:
    HAS_CV_PIPELINE = False

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
DEFAULT_ESP_HOST = "192.168.4.1"
DEFAULT_ESP_PORT = 80
DEFAULT_APP_PORT = 5000

app = Flask(__name__)

# ---------------------------------------------------------------------------
# Rate limiter — 1 request per second to ESP
# ---------------------------------------------------------------------------
_last_esp_request_time = 0.0
RATE_LIMIT_INTERVAL = 1.0  # seconds

def rate_limit():
    """Return True if the request is allowed, False if it should be dropped."""
    global _last_esp_request_time
    now = time.time()
    if now - _last_esp_request_time < RATE_LIMIT_INTERVAL:
        return False
    _last_esp_request_time = now
    return True

# Set at startup from CLI args
ESP_BASE = ""
DETECT_MODE = "host"  # "host" or "esp"
DOG_BINARY = "./differenceofgaussian"

CALIB_FILE = Path(__file__).parent / "calibration.json"

# ---------------------------------------------------------------------------
# Calibration — store K matrix + distortion, apply undistort before CV
# ---------------------------------------------------------------------------
_calib_K = None       # 3x3 camera matrix (numpy)
_calib_dist = None    # distortion coefficients (numpy)
_calib_res = None     # (width, height) at which calibration was done


def _load_calibration():
    """Load calibration from disk if it exists."""
    global _calib_K, _calib_dist, _calib_res
    if not CALIB_FILE.exists():
        return False
    try:
        data = json.loads(CALIB_FILE.read_text())
        _calib_K = np.array(data["K"])
        _calib_dist = np.array(data["dist"])
        _calib_res = tuple(data["resolution"])
        return True
    except Exception as e:
        print(f"Warning: failed to load {CALIB_FILE}: {e}")
        return False


def _save_calibration():
    """Persist current calibration to disk."""
    if _calib_K is None:
        return
    data = {
        "K": _calib_K.tolist(),
        "dist": _calib_dist.tolist(),
        "resolution": list(_calib_res),
    }
    CALIB_FILE.write_text(json.dumps(data, indent=2))


def undistort_image(img):
    """Apply lens undistortion if calibration is loaded. Returns corrected image."""
    if _calib_K is None or _calib_dist is None:
        return img
    h, w = img.shape[:2]
    K = _calib_K.copy()
    if _calib_res and (w != _calib_res[0] or h != _calib_res[1]):
        sx = w / _calib_res[0]
        sy = h / _calib_res[1]
        K[0, 0] *= sx
        K[1, 1] *= sy
        K[0, 2] *= sx
        K[1, 2] *= sy
    return cv2.undistort(img, K, _calib_dist)


# Server-side log ring buffer (last 200 entries)
_server_logs = collections.deque(maxlen=200)
_log_counter = 0

def slog(msg: str, level: str = "info"):
    """Append a timestamped message to the server log buffer."""
    global _log_counter
    _log_counter += 1
    entry = {
        "id": _log_counter,
        "ts": time.strftime("%H:%M:%S"),
        "level": level,
        "msg": msg,
    }
    _server_logs.append(entry)
    # Also print to terminal
    tag = {"info": "INFO", "err": "ERROR", "warn": "WARN"}.get(level, level.upper())
    print(f"[{entry['ts']}] [{tag}] {msg}", flush=True)

# ---------------------------------------------------------------------------
# ESP proxy helpers
# ---------------------------------------------------------------------------

def esp_url(path: str) -> str:
    return f"{ESP_BASE}{path}"

def proxy_get(path: str, throttle: bool = True) -> Response:
    """Proxy a GET request to the ESP and return the response."""
    if throttle and not rate_limit():
        slog(f"-- GET {path} dropped (rate limited)", "warn")
        return jsonify({"error": "rate limited"}), 429
    slog(f"-> GET {path}")
    try:
        r = requests.get(esp_url(path), timeout=10)
        ct = r.headers.get('content-type', 'text/plain')
        body_preview = r.text[:120] if 'text' in ct else f"[{len(r.content)} bytes]"
        slog(f"<- GET {path} {r.status_code} {body_preview}")
        excluded = {'content-encoding', 'transfer-encoding', 'connection'}
        headers = {k: v for k, v in r.headers.items() if k.lower() not in excluded}
        return Response(r.content, status=r.status_code, headers=headers,
                        content_type=ct)
    except requests.RequestException as e:
        slog(f"<- GET {path} FAILED: {e}", "err")
        return jsonify({"error": str(e)}), 502

def proxy_post(path: str, **kwargs) -> Response:
    """Proxy a POST request to the ESP and return the response."""
    if not rate_limit():
        slog(f"-- POST {path} dropped (rate limited)", "warn")
        return jsonify({"error": "rate limited"}), 429
    slog(f"-> POST {path}")
    try:
        r = requests.post(esp_url(path), timeout=10, **kwargs)
        ct = r.headers.get('content-type', 'text/plain')
        body_preview = r.text[:120] if 'text' in ct else f"[{len(r.content)} bytes]"
        slog(f"<- POST {path} {r.status_code} {body_preview}")
        excluded = {'content-encoding', 'transfer-encoding', 'connection'}
        headers = {k: v for k, v in r.headers.items() if k.lower() not in excluded}
        return Response(r.content, status=r.status_code, headers=headers,
                        content_type=ct)
    except requests.RequestException as e:
        slog(f"<- POST {path} FAILED: {e}", "err")
        return jsonify({"error": str(e)}), 502

# ---------------------------------------------------------------------------
# ESP proxy routes — all under /esp/
# ---------------------------------------------------------------------------

@app.route('/esp/capture', methods=['POST'])
def esp_capture():
    return proxy_post('/capture')

@app.route('/esp/get', methods=['GET'])
def esp_get_frame():
    return proxy_get('/get')

@app.route('/esp/configure', methods=['POST'])
def esp_configure():
    return proxy_post('/configure', json=request.get_json(force=True))

@app.route('/esp/servo', methods=['POST'])
def esp_servo():
    return proxy_post('/servo', json=request.get_json(force=True))

@app.route('/esp/route/create', methods=['POST'])
def esp_route_create():
    return proxy_post('/route/create', json=request.get_json(force=True))

@app.route('/esp/route/delete', methods=['POST'])
def esp_route_delete():
    return proxy_post('/route/delete', json=request.get_json(force=True))

@app.route('/esp/route/load', methods=['POST'])
def esp_route_load():
    return proxy_post('/route/load', json=request.get_json(force=True))

@app.route('/esp/route/play', methods=['POST'])
def esp_route_play():
    return proxy_post('/route/play', json=request.get_json(force=True))

@app.route('/esp/route/pause', methods=['GET'])
def esp_route_pause():
    return proxy_get('/route/pause')

@app.route('/esp/route/next', methods=['GET'])
def esp_route_next():
    return proxy_get('/route/next')

@app.route('/esp/route/restart', methods=['POST'])
def esp_route_restart():
    return proxy_post('/route/restart')

@app.route('/esp/route/mapping', methods=['POST'])
def esp_route_mapping():
    return proxy_post('/route/mapping', json=request.get_json(force=True))

@app.route('/esp/start', methods=['GET'])
def esp_start():
    return proxy_get('/start')

@app.route('/esp/stop', methods=['GET'])
def esp_stop():
    return proxy_get('/stop')

@app.route('/esp/test', methods=['POST'])
def esp_test():
    return proxy_post('/test', data=request.get_data())

@app.route('/esp/state', methods=['GET'])
def esp_get_state():
    return proxy_get('/state', throttle=False)

@app.route('/esp/state', methods=['POST'])
def esp_set_state():
    return proxy_post('/state', json=request.get_json(force=True))

@app.route('/esp/laser', methods=['POST'])
def esp_laser():
    return proxy_post('/laser', json=request.get_json(force=True))

@app.route('/esp/centroids/upload', methods=['POST'])
def esp_centroids_upload():
    return proxy_post('/centroids/upload', json=request.get_json(force=True))

@app.route('/esp/centroids', methods=['GET'])
def esp_centroids_get():
    scan_id = request.args.get('scan_id', '0')
    return proxy_get(f'/centroids?scan_id={scan_id}')

@app.route('/esp/scan/save', methods=['POST'])
def esp_scan_save():
    return proxy_post('/scan/save', json=request.get_json(force=True))

@app.route('/esp/gimbal/zero', methods=['POST'])
def esp_gimbal_zero():
    return proxy_post('/gimbal/zero')

@app.route('/esp/gimbal/offset', methods=['POST'])
def esp_gimbal_offset():
    return proxy_post('/gimbal/offset', json=request.get_json(force=True))

@app.route('/esp/gimbal/poly', methods=['POST'])
def esp_gimbal_poly_set():
    return proxy_post('/gimbal/poly', json=request.get_json(force=True))

@app.route('/esp/gimbal/poly', methods=['GET'])
def esp_gimbal_poly_get():
    return proxy_get('/gimbal/poly', throttle=False)

@app.route('/esp/gimbal/point', methods=['POST'])
def esp_gimbal_point():
    return proxy_post('/gimbal/point', json=request.get_json(force=True))

# ---------------------------------------------------------------------------
# Calibration endpoints
# ---------------------------------------------------------------------------

@app.route('/calibrate/set', methods=['POST'])
def calibrate_set():
    """Set calibration parameters manually.
    JSON: {fx, fy, cx, cy, width, height, k1?, k2?, p1?, p2?, k3?}
    """
    global _calib_K, _calib_dist, _calib_res
    if not HAS_CV2:
        return jsonify({"error": "OpenCV not available"}), 500

    data = request.get_json(force=True)
    required = ["fx", "fy", "cx", "cy", "width", "height"]
    missing = [k for k in required if k not in data]
    if missing:
        return jsonify({"error": f"Missing: {missing}"}), 400

    _calib_K = np.array([
        [data["fx"], 0, data["cx"]],
        [0, data["fy"], data["cy"]],
        [0, 0, 1],
    ], dtype=np.float64)
    _calib_dist = np.array([
        data.get("k1", 0), data.get("k2", 0),
        data.get("p1", 0), data.get("p2", 0),
        data.get("k3", 0),
    ], dtype=np.float64)
    _calib_res = (int(data["width"]), int(data["height"]))

    _save_calibration()
    slog(f"Calibration set: fx={data['fx']}, fy={data['fy']}, {_calib_res[0]}x{_calib_res[1]}")
    return jsonify({"status": "ok"})


@app.route('/calibrate/get', methods=['GET'])
def calibrate_get():
    """Return current calibration values."""
    if _calib_K is None:
        return jsonify({"calibrated": False})
    return jsonify({
        "calibrated": True,
        "fx": round(_calib_K[0, 0], 4),
        "fy": round(_calib_K[1, 1], 4),
        "cx": round(_calib_K[0, 2], 4),
        "cy": round(_calib_K[1, 2], 4),
        "k1": round(float(_calib_dist[0]), 6),
        "k2": round(float(_calib_dist[1]), 6),
        "p1": round(float(_calib_dist[2]), 6),
        "p2": round(float(_calib_dist[3]), 6),
        "k3": round(float(_calib_dist[4]), 6),
        "width": _calib_res[0],
        "height": _calib_res[1],
    })


@app.route('/calibrate/clear', methods=['POST'])
def calibrate_clear():
    """Clear stored calibration."""
    global _calib_K, _calib_dist, _calib_res
    _calib_K = None
    _calib_dist = None
    _calib_res = None
    if CALIB_FILE.exists():
        CALIB_FILE.unlink()
    slog("Calibration cleared")
    return jsonify({"status": "cleared"})


# ---------------------------------------------------------------------------
# Gimbal pointing calibration
# ---------------------------------------------------------------------------

IMAGE_WIDTH  = 2560
IMAGE_HEIGHT = 1920
HFOV_DEG     = 120.0
GIMCAL_FILE  = Path(__file__).parent / "gimbal_offsets.json"

# In-memory offsets: list of {"dx": int, "dy": int} per gimbal
_gimbal_offsets: List[dict] = [{"dx": 0, "dy": 0} for _ in range(4)]


def _load_gimbal_offsets():
    if not GIMCAL_FILE.exists():
        return
    try:
        data = json.loads(GIMCAL_FILE.read_text())
        for i, entry in enumerate(data):
            if i < len(_gimbal_offsets):
                _gimbal_offsets[i] = entry
        slog(f"Gimbal offsets loaded from {GIMCAL_FILE.name}")
    except Exception as e:
        slog(f"Warning: failed to load gimbal offsets: {e}", "warn")


def _save_gimbal_offsets():
    GIMCAL_FILE.write_text(json.dumps(_gimbal_offsets, indent=2))


def _pixel_to_servo_x(px: float, image_width: int = IMAGE_WIDTH, hfov_deg: float = HFOV_DEG) -> int:
    """Must match pixel_to_servo_x() in route.c exactly."""
    center_x = image_width / 2.0
    angle_deg = (center_x - px) * (hfov_deg / image_width)
    return max(0, min(180, round(angle_deg + 90.0)))


def _pixel_to_servo_y(py: float, image_height: int = IMAGE_HEIGHT) -> int:
    """Must match pixel_to_servo_y() in route.c exactly."""
    angle_deg = (image_height - py) * (30.0 / image_height)
    return max(0, min(180, round(90.0 + angle_deg)))


@app.route('/gimcal/start', methods=['POST'])
def gimcal_start():
    """Zero all gimbals in safe order (g1 before g0) to begin calibration."""
    try:
        r = requests.post(f"{ESP_BASE}/gimbal/zero", timeout=15)
        slog("[GIMCAL] Gimbals zeroed for calibration")
        return jsonify({"status": "ok"})
    except requests.RequestException as e:
        slog(f"[GIMCAL] Zero failed: {e}", "err")
        return jsonify({"error": str(e)}), 502


@app.route('/gimcal/point', methods=['POST'])
def gimcal_point():
    """Drive a gimbal to raw (uncorrected) servo angles computed from a pixel.
    Proxies to ESP /gimbal/point so the geometric computation matches route playback.
    JSON: {gimbal, x, y, image_width?, image_height?}
    Returns: {angle_x, angle_y}
    """
    data = request.get_json(force=True)
    body = {
        "gimbal": int(data.get("gimbal", 0)),
        "px": float(data["x"]),
        "py": float(data["y"]),
    }
    if "image_width" in data:
        body["image_width"] = int(data["image_width"])
    if "image_height" in data:
        body["image_height"] = int(data["image_height"])
    try:
        r = requests.post(f"{ESP_BASE}/gimbal/point", json=body, timeout=15)
        result = r.json()
        slog(f"[GIMCAL] Gimbal {body['gimbal']} -> pixel({body['px']:.0f},{body['py']:.0f}) "
             f"X={result.get('angle_x')}° Y={result.get('angle_y')}°")
        return jsonify(result)
    except requests.RequestException as e:
        slog(f"[GIMCAL] Point drive failed: {e}", "err")
        return jsonify({"error": str(e)}), 502


@app.route('/gimcal/offset', methods=['POST'])
def gimcal_offset():
    """Store and apply per-gimbal angle offsets.
    JSON: {gimbal, dx, dy}   — dx/dy are signed integer degrees.
    """
    data = request.get_json(force=True)
    g  = int(data.get("gimbal", 0))
    dx = int(data.get("dx", 0))
    dy = int(data.get("dy", 0))
    if g < 0 or g >= len(_gimbal_offsets):
        return jsonify({"error": "invalid gimbal"}), 400

    _gimbal_offsets[g] = {"dx": dx, "dy": dy}
    _save_gimbal_offsets()

    try:
        requests.post(f"{ESP_BASE}/gimbal/offset", json={"gimbal": g, "dx": dx, "dy": dy}, timeout=10)
        slog(f"[GIMCAL] Offset saved: gimbal {g} dx={dx:+d}° dy={dy:+d}°")
    except requests.RequestException as e:
        slog(f"[GIMCAL] Offset send to ESP failed: {e}", "warn")

    return jsonify({"status": "ok", "gimbal": g, "dx": dx, "dy": dy})


@app.route('/gimcal/drive', methods=['POST'])
def gimcal_drive():
    """Drive both servos of one gimbal directly, bypassing the rate limiter.
    JSON: {gimbal, angle_x, angle_y}
    """
    data = request.get_json(force=True)
    g   = int(data['gimbal'])
    ax  = max(0, min(180, int(data['angle_x'])))
    ay  = max(0, min(180, int(data['angle_y'])))
    try:
        requests.post(f"{ESP_BASE}/servo", json={"servo": g * 2,     "angle": ax}, timeout=10)
        time.sleep(0.15)
        requests.post(f"{ESP_BASE}/servo", json={"servo": g * 2 + 1, "angle": ay}, timeout=10)
        return jsonify({"ok": True, "angle_x": ax, "angle_y": ay})
    except requests.RequestException as e:
        slog(f"[GIMCAL] Drive failed: {e}", "err")
        return jsonify({"error": str(e)}), 502


@app.route('/gimcal/offsets', methods=['GET'])
def gimcal_offsets_get():
    return jsonify(_gimbal_offsets)


# ---------------------------------------------------------------------------
# Polynomial calibration — collect points, fit bilinear model, send to ESP
# ---------------------------------------------------------------------------

# In-memory calibration points per gimbal (cleared each session).
# Each entry: {"computed_x": float, "computed_y": float, "actual_x": float, "actual_y": float}
_polycal_points: List[List[dict]] = [[] for _ in range(4)]


@app.route('/gimcal/poly/collect', methods=['POST'])
def gimcal_poly_collect():
    """Record a calibration data point.
    JSON: {gimbal, computed_x, computed_y, actual_x, actual_y}
    """
    data = request.get_json(force=True)
    g = int(data["gimbal"])
    if g < 0 or g >= 4:
        return jsonify({"error": "invalid gimbal"}), 400
    point = {
        "computed_x": float(data["computed_x"]),
        "computed_y": float(data["computed_y"]),
        "actual_x": float(data["actual_x"]),
        "actual_y": float(data["actual_y"]),
    }
    _polycal_points[g].append(point)
    slog(f"[POLYCAL] G{g} point #{len(_polycal_points[g])}: "
         f"computed=({point['computed_x']},{point['computed_y']}) "
         f"actual=({point['actual_x']},{point['actual_y']})")
    return jsonify({"status": "ok", "gimbal": g, "count": len(_polycal_points[g])})


@app.route('/gimcal/poly/fit', methods=['POST'])
def gimcal_poly_fit():
    """Fit bilinear polynomial from collected points and send to ESP.
    JSON: {} or {gimbal: N} to fit just one.
    Requires numpy and at least 4 points per gimbal.
    """
    if not HAS_CV2:
        return jsonify({"error": "numpy not available"}), 500

    data = request.get_json(force=True) if request.content_length else {}
    target_gimbal = data.get("gimbal")  # None = fit all

    results = {}
    all_coeffs = []

    for g in range(4):
        if target_gimbal is not None and g != int(target_gimbal):
            # Keep existing coefficients for non-targeted gimbals.
            all_coeffs.append(None)
            continue

        pts = _polycal_points[g]
        if len(pts) < 4:
            results[f"g{g}"] = {"error": f"need >= 4 points, have {len(pts)}"}
            all_coeffs.append(None)
            continue

        # Build design matrix: [1, cx, cy, cx*cy]
        cx = np.array([p["computed_x"] for p in pts])
        cy = np.array([p["computed_y"] for p in pts])
        ax = np.array([p["actual_x"] for p in pts])
        ay = np.array([p["actual_y"] for p in pts])

        A = np.column_stack([np.ones_like(cx), cx, cy, cx * cy])
        err_x = ax - cx  # residual to fit
        err_y = ay - cy

        # Least-squares fit
        coeff_x, res_x, _, _ = np.linalg.lstsq(A, err_x, rcond=None)
        coeff_y, res_y, _, _ = np.linalg.lstsq(A, err_y, rcond=None)

        coeffs = list(coeff_x) + list(coeff_y)  # [a0,a1,a2,a3,b0,b1,b2,b3]
        all_coeffs.append([float(c) for c in coeffs])

        # Compute RMS residual for reporting
        pred_x = cx + A @ coeff_x
        pred_y = cy + A @ coeff_y
        rms_x = float(np.sqrt(np.mean((ax - pred_x) ** 2)))
        rms_y = float(np.sqrt(np.mean((ay - pred_y) ** 2)))

        results[f"g{g}"] = {
            "coeffs": [round(float(c), 6) for c in coeffs],
            "rms_x": round(rms_x, 3),
            "rms_y": round(rms_y, 3),
            "points": len(pts),
        }
        slog(f"[POLYCAL] G{g} fit: rms_x={rms_x:.3f}° rms_y={rms_y:.3f}° ({len(pts)} pts)")

    # Send fitted coefficients to ESP.
    # For gimbals we didn't fit, fetch current coefficients from ESP.
    try:
        esp_resp = requests.get(f"{ESP_BASE}/gimbal/poly", timeout=10)
        current = esp_resp.json().get("gimbals", [[0]*8]*4)
    except Exception:
        current = [[0]*8]*4

    bulk = []
    for g in range(4):
        if all_coeffs[g] is not None:
            bulk.append(all_coeffs[g])
        else:
            bulk.append(current[g] if g < len(current) else [0]*8)

    try:
        requests.post(f"{ESP_BASE}/gimbal/poly",
                      json={"all": bulk, "save": True}, timeout=10)
        slog("[POLYCAL] Coefficients sent to ESP and saved")
    except requests.RequestException as e:
        slog(f"[POLYCAL] Failed to send coefficients to ESP: {e}", "err")
        return jsonify({"error": str(e), "results": results}), 502

    return jsonify({"status": "ok", "results": results})


@app.route('/gimcal/poly/points', methods=['GET'])
def gimcal_poly_points():
    """Return collected calibration points."""
    return jsonify({f"g{g}": pts for g, pts in enumerate(_polycal_points)})


@app.route('/gimcal/poly/clear', methods=['POST'])
def gimcal_poly_clear():
    """Clear collected calibration points.
    JSON: {} for all, or {gimbal: N} for one.
    """
    data = request.get_json(force=True) if request.content_length else {}
    g = data.get("gimbal")
    if g is not None:
        g = int(g)
        if 0 <= g < 4:
            _polycal_points[g] = []
            slog(f"[POLYCAL] Cleared points for gimbal {g}")
    else:
        for i in range(4):
            _polycal_points[i] = []
        slog("[POLYCAL] Cleared all calibration points")
    return jsonify({"status": "ok"})


@app.route('/gimcal/poly/coeffs', methods=['GET'])
def gimcal_poly_coeffs():
    """Proxy to ESP GET /gimbal/poly — return current polynomial coefficients."""
    try:
        r = requests.get(f"{ESP_BASE}/gimbal/poly", timeout=10)
        return jsonify(r.json())
    except requests.RequestException as e:
        return jsonify({"error": str(e)}), 502


# ---------------------------------------------------------------------------
# AUTO mode — frame-diff based climber detection + hold advance
# ---------------------------------------------------------------------------

_auto_lock = threading.Lock()
_auto_thread = None
_auto_status = {"running": False}


def _capture_frame_raw():
    """Capture + download a JPEG from ESP, return as BGR numpy array or None."""
    try:
        requests.post(f"{ESP_BASE}/capture", timeout=10)
        time.sleep(0.2)
        resp = requests.get(f"{ESP_BASE}/get", timeout=10)
        if resp.status_code != 200 or resp.headers.get("content-type") != "image/jpeg":
            return None
        arr = np.frombuffer(resp.content, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None and _calib_K is not None:
            img = undistort_image(img)
        return img
    except Exception as e:
        slog(f"[AUTO] Frame capture failed: {e}", "err")
        return None


def _auto_advance_loop(holds, interval_ms, radius_px, threshold_pct):
    """
    Background loop:
      1. Capture baseline frame (no climber)
      2. Every interval_ms: capture frame, diff vs baseline near current hold
      3. If enough pixels changed near hold → send /route/next, advance index
      4. Stop when all holds visited or state changes
    """
    global _auto_status

    current_idx = 0
    num_holds = len(holds)

    with _auto_lock:
        _auto_status = {
            "running": True,
            "current_hold": current_idx,
            "total_holds": num_holds,
            "msg": "Capturing baseline...",
        }

    slog(f"[AUTO] Starting: {num_holds} holds, interval={interval_ms}ms, radius={radius_px}px, threshold={threshold_pct}%")

    # Capture baseline
    baseline = _capture_frame_raw()
    if baseline is None:
        with _auto_lock:
            _auto_status = {"running": False, "msg": "Failed to capture baseline"}
        slog("[AUTO] Failed to capture baseline frame", "err")
        return

    baseline_gray = cv2.cvtColor(baseline, cv2.COLOR_BGR2GRAY)
    baseline_gray = cv2.GaussianBlur(baseline_gray, (21, 21), 0)
    h, w = baseline_gray.shape

    with _auto_lock:
        _auto_status["msg"] = f"Running — hold {current_idx + 1}/{num_holds}"

    slog(f"[AUTO] Baseline captured ({w}x{h}), monitoring hold 0")

    interval_s = interval_ms / 1000.0

    while current_idx < num_holds:
        # Check if we should stop
        with _auto_lock:
            if not _auto_status.get("running"):
                break

        time.sleep(interval_s)

        # Check ESP state is still RUNNING
        try:
            state = requests.get(f"{ESP_BASE}/state", timeout=5).json()
            if state.get("climb_state") != "RUNNING":
                slog("[AUTO] Climb no longer RUNNING, stopping")
                break
        except Exception:
            pass

        # Capture current frame
        frame = _capture_frame_raw()
        if frame is None:
            continue

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_gray = cv2.GaussianBlur(frame_gray, (21, 21), 0)

        # Compute diff
        diff = cv2.absdiff(baseline_gray, frame_gray)
        _, thresh = cv2.threshold(diff, 30, 255, cv2.THRESH_BINARY)

        # Check region around current hold
        hx, hy = int(holds[current_idx][0]), int(holds[current_idx][1])
        # Clamp ROI to image bounds
        x1 = max(0, hx - radius_px)
        y1 = max(0, hy - radius_px)
        x2 = min(w, hx + radius_px)
        y2 = min(h, hy + radius_px)

        roi = thresh[y1:y2, x1:x2]
        roi_area = roi.shape[0] * roi.shape[1]
        if roi_area == 0:
            continue

        changed_pct = (cv2.countNonZero(roi) / roi_area) * 100

        if changed_pct >= threshold_pct:
            slog(f"[AUTO] Hold {current_idx}: {changed_pct:.1f}% change detected (>={threshold_pct}%), advancing")
            try:
                requests.get(f"{ESP_BASE}/route/next", timeout=5)
            except Exception as e:
                slog(f"[AUTO] Failed to send /route/next: {e}", "err")

            current_idx += 1
            with _auto_lock:
                _auto_status["current_hold"] = current_idx
                _auto_status["msg"] = f"Running — hold {current_idx + 1}/{num_holds}" if current_idx < num_holds else "Complete"

    with _auto_lock:
        _auto_status["running"] = False
        if current_idx >= num_holds:
            _auto_status["msg"] = f"Complete — all {num_holds} holds reached"
        slog(f"[AUTO] Stopped at hold {current_idx}/{num_holds}")


@app.route('/auto/start', methods=['POST'])
def auto_start():
    """Start AUTO advance mode.
    JSON: {holds: [[x,y],...], interval_ms?: 500, radius_px?: 40, threshold_pct?: 15}
    Or: {scan_id: N} to fetch holds from ESP centroids.
    """
    global _auto_thread
    if not HAS_CV2:
        return jsonify({"error": "OpenCV not available"}), 500

    with _auto_lock:
        if _auto_status.get("running"):
            return jsonify({"error": "Already running"}), 409

    data = request.get_json(force=True) if request.content_length else {}
    interval_ms = data.get("interval_ms", 500)
    radius_px = data.get("radius_px", 40)
    threshold_pct = data.get("threshold_pct", 15)

    # Get holds
    holds = data.get("holds")
    if not holds and "scan_id" in data:
        try:
            resp = requests.get(f"{ESP_BASE}/centroids?scan_id={data['scan_id']}", timeout=10)
            if resp.ok:
                holds = resp.json().get("centroids", [])
        except Exception as e:
            return jsonify({"error": f"Failed to fetch centroids: {e}"}), 502

    if not holds or len(holds) < 1:
        return jsonify({"error": "No holds provided"}), 400

    _auto_thread = threading.Thread(
        target=_auto_advance_loop,
        args=(holds, interval_ms, radius_px, threshold_pct),
        daemon=True,
    )
    _auto_thread.start()
    return jsonify({"status": "started", "holds": len(holds)})


@app.route('/auto/stop', methods=['POST'])
def auto_stop():
    """Stop AUTO advance mode."""
    with _auto_lock:
        _auto_status["running"] = False
    slog("[AUTO] Stop requested")
    return jsonify({"status": "stopping"})


@app.route('/auto/status', methods=['GET'])
def auto_status():
    """Get AUTO mode status."""
    with _auto_lock:
        return jsonify(dict(_auto_status))


# ---------------------------------------------------------------------------
# Hold detection
# ---------------------------------------------------------------------------

@app.route('/detect', methods=['POST'])
def detect_holds():
    """
    Run hold detection on a JPEG image.

    If detect mode is 'esp', proxies to the ESP's /detect endpoint.
    If detect mode is 'host', runs the local DoG binary on the image.

    Accepts: multipart form with 'image' file, or raw JPEG body.
    Returns: {"centroids": [[x, y], ...]}
    """
    if DETECT_MODE == 'esp':
        slog("Detection mode: ESP (proxying to /detect)")
        return proxy_post('/detect')

    if not HAS_CV_PIPELINE:
        slog("CV pipeline unavailable — pip install opencv-python numpy", "err")
        return jsonify({"error": "CV pipeline not available (pip install opencv-python numpy)"}), 500

    # Get JPEG bytes
    if 'image' in request.files:
        jpeg_bytes = request.files['image'].read()
    else:
        jpeg_bytes = request.get_data()

    if not jpeg_bytes:
        slog("No image provided for detection", "err")
        return jsonify({"error": "No image provided"}), 400

    slog(f"Running hold detection on {len(jpeg_bytes)} byte image")

    # Undistort if calibration is available
    if HAS_CV2 and _calib_K is not None:
        arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            img = undistort_image(img)
            _, jpeg_bytes = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 95])
            jpeg_bytes = jpeg_bytes.tobytes()
            slog("Applied lens undistortion before detection")

    with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as f:
        f.write(jpeg_bytes)
        tmp_path = f.name

    try:
        _, holds = _cv_detect_holds(tmp_path)
        centroids = [[h['centroid'][0], h['centroid'][1]] for h in holds]
        slog(f"Detection complete: {len(centroids)} holds found")
        return jsonify({"centroids": centroids})
    except Exception as e:
        slog(f"Detection exception: {traceback.format_exc()}", "err")
        return jsonify({"error": str(e)}), 500
    finally:
        Path(tmp_path).unlink(missing_ok=True)

# ---------------------------------------------------------------------------
# Video recording — capture frames from ESP at ~2 fps and write to .avi
# ---------------------------------------------------------------------------

_rec_lock = threading.Lock()
_rec_thread = None
_rec_status = {"recording": False, "frames": 0, "file": ""}


def _record_loop(output_path: str, interval: float):
    global _rec_status
    if not HAS_CV2:
        with _rec_lock:
            _rec_status = {"recording": False, "frames": 0, "file": "", "error": "OpenCV not available"}
        return

    writer = None
    frame_count = 0

    try:
        while True:
            with _rec_lock:
                if not _rec_status["recording"]:
                    break

            try:
                requests.post(f"{ESP_BASE}/capture", timeout=10)
                time.sleep(0.2)
                resp = requests.get(f"{ESP_BASE}/get", timeout=10)
                if resp.status_code != 200:
                    continue
                arr = np.frombuffer(resp.content, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is None:
                    continue
            except Exception as e:
                slog(f"[REC] Frame grab failed: {e}", "warn")
                continue

            if writer is None:
                h, w = img.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                fps = 1.0 / interval
                writer = cv2.VideoWriter(output_path, fourcc, fps, (w, h))
                slog(f"[REC] Recording to {output_path} ({w}x{h} @ {fps:.1f} fps)")

            writer.write(img)
            frame_count += 1
            with _rec_lock:
                _rec_status["frames"] = frame_count

            time.sleep(max(0, interval - 0.2))

    finally:
        if writer:
            writer.release()
        with _rec_lock:
            _rec_status["recording"] = False
            _rec_status["frames"] = frame_count
        slog(f"[REC] Stopped — {frame_count} frames saved to {output_path}")


@app.route('/record/start', methods=['POST'])
def record_start():
    global _rec_thread
    if not HAS_CV2:
        return jsonify({"error": "OpenCV not available"}), 500

    with _rec_lock:
        if _rec_status["recording"]:
            return jsonify({"error": "Already recording"}), 409

    data = request.get_json(force=True) if request.content_length else {}
    interval = float(data.get("interval", 0.5))
    filename = data.get("filename", f"recording_{int(time.time())}.avi")
    output_path = str(Path(__file__).parent / filename)

    with _rec_lock:
        _rec_status.update({"recording": True, "frames": 0, "file": filename})

    _rec_thread = threading.Thread(target=_record_loop, args=(output_path, interval), daemon=True)
    _rec_thread.start()
    return jsonify({"status": "recording", "file": filename})


@app.route('/record/stop', methods=['POST'])
def record_stop():
    with _rec_lock:
        _rec_status["recording"] = False
    slog("[REC] Stop requested")
    return jsonify({"status": "stopping", "frames": _rec_status["frames"], "file": _rec_status["file"]})


@app.route('/record/status', methods=['GET'])
def record_status():
    with _rec_lock:
        return jsonify(dict(_rec_status))


# ---------------------------------------------------------------------------
# Server log endpoint
# ---------------------------------------------------------------------------

@app.route('/logs', methods=['GET'])
def get_logs():
    """Return server logs newer than the given ?since= id."""
    since = int(request.args.get('since', 0))
    entries = [e for e in _server_logs if e['id'] > since]
    return jsonify(entries)

# ---------------------------------------------------------------------------
# Frontend
# ---------------------------------------------------------------------------

@app.route('/')
def index():
    return FRONTEND_HTML

# ---------------------------------------------------------------------------
# Frontend HTML (inline)
# ---------------------------------------------------------------------------

FRONTEND_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>BetaSpray</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; background: #1a1a2e; color: #e0e0e0; }

  header { background: #16213e; padding: 12px 24px; display: flex; align-items: center; justify-content: space-between; border-bottom: 2px solid #0f3460; }
  header h1 { font-size: 1.4em; color: #e94560; }
  header .status { font-size: 0.85em; color: #888; }

  .container { display: grid; grid-template-columns: 1fr 300px; gap: 16px; padding: 16px; max-width: 1200px; margin: 0 auto; }

  .panel { background: #16213e; border-radius: 8px; padding: 16px; border: 1px solid #0f3460; }
  .panel h2 { font-size: 1em; color: #e94560; margin-bottom: 12px; border-bottom: 1px solid #0f3460; padding-bottom: 6px; }

  button { background: #0f3460; color: #e0e0e0; border: 1px solid #e94560; border-radius: 4px; padding: 6px 14px; cursor: pointer; font-size: 0.85em; transition: background 0.15s; }
  button:hover { background: #e94560; color: #fff; }
  button:disabled { opacity: 0.4; cursor: not-allowed; }
  button.active { background: #e94560; color: #fff; }
  button.danger { border-color: #ff6b6b; }
  button.danger:hover { background: #ff6b6b; }

  .btn-row { display: flex; gap: 8px; flex-wrap: wrap; margin-bottom: 10px; }

  .canvas-wrap { position: relative; background: #0a0a1a; border-radius: 6px; overflow: hidden; min-height: 240px; display: flex; align-items: center; justify-content: center; }
  .canvas-wrap canvas { display: block; max-width: 100%; cursor: crosshair; }
  .canvas-wrap .placeholder { color: #555; font-size: 0.9em; }

  .mode-banner { background: #e94560; color: #fff; text-align: center; padding: 6px; font-size: 0.85em; font-weight: bold; border-radius: 4px; margin-bottom: 10px; }

  .control-group { margin-bottom: 14px; }
  .control-group label { display: block; font-size: 0.8em; color: #aaa; margin-bottom: 4px; }
  .control-group input, .control-group select { width: 100%; padding: 5px 8px; background: #0a0a1a; border: 1px solid #0f3460; color: #e0e0e0; border-radius: 4px; font-size: 0.85em; }

  .hold-list { max-height: 200px; overflow-y: auto; font-size: 0.8em; }
  .hold-list .hold-item { display: flex; justify-content: space-between; padding: 3px 6px; border-bottom: 1px solid #0f3460; }
  .hold-list .hold-item:hover { background: #0f3460; }
  .hold-item .remove-hold { color: #ff6b6b; cursor: pointer; border: none; background: none; font-size: 0.85em; }

  #log { background: #0a0a1a; border: 1px solid #0f3460; border-radius: 4px; padding: 8px; font-family: monospace; font-size: 0.75em; height: 220px; overflow-y: auto; color: #8f8; white-space: pre-wrap; word-break: break-all; }
  #log .err { color: #ff6b6b; }
  #log .warn { color: #ffb347; }
  #log .info { color: #88f; }
  #log .srv { color: #aaa; }
  #log .srv.err { color: #ff6b6b; }
  #log .srv.warn { color: #ffb347; }

  .servo-row { display: flex; align-items: center; gap: 8px; margin-bottom: 6px; font-size: 0.85em; }
  .servo-row label { width: 70px; }
  .servo-row input[type="range"] { flex: 1; }
  .servo-row span { width: 36px; text-align: right; }

  .route-num-row { display: flex; gap: 8px; align-items: center; margin-bottom: 8px; }
  .route-num-row input { width: 60px; }

  @media (max-width: 800px) { .container { grid-template-columns: 1fr; } }
</style>
</head>
<body>

<header>
  <h1>BetaSpray</h1>
  <span class="status">Proxied via Flask</span>
</header>

<div class="container">
  <div>
    <div id="modeBanner" class="mode-banner" style="display:none;"></div>

    <div class="canvas-wrap" id="canvasWrap">
      <span class="placeholder" id="canvasPlaceholder">No image captured</span>
      <canvas id="wallCanvas" style="display:none;"></canvas>
    </div>

    <div class="panel" style="margin-top:12px;">
      <h2>Camera</h2>
      <div class="btn-row">
        <button id="btnEnableCam">Enable Camera</button>
        <button id="btnCapture" disabled>Capture</button>
        <button id="btnFetchImage" disabled>Fetch Image</button>
        <button id="btnLiveView" disabled>Live View</button>
        <button id="btnUploadImage">Upload Image</button>
        <button id="btnRecordStart">Record</button>
        <button id="btnRecordStop" disabled>Stop Record</button>
        <input type="file" id="fileInput" accept="image/*" style="display:none;">
      </div>
      <div id="recStatus" style="font-size:0.8em;color:#aaa;display:none;"></div>

      <h2 style="margin-top:12px;">Route Creation</h2>
      <div class="btn-row">
        <button id="btnNewRoute">New Route (detect holds)</button>
        <button id="btnManualRoute">New Route (manual clicks)</button>
        <button id="btnFinishRoute" disabled>Save Route</button>
        <button id="btnCancelRoute" class="danger" disabled>Cancel</button>
      </div>
      <div class="hold-list" id="holdList"></div>

      <h2 style="margin-top:12px;">Playback</h2>
      <div class="btn-row">
        <button id="btnPlay">Play</button>
        <button id="btnPause">Pause</button>
        <button id="btnNext">Next</button>
        <button id="btnRestart">Restart</button>
        <button id="btnStop" class="danger">Stop</button>
      </div>

      <h2 style="margin-top:12px;">Auto Advance</h2>
      <div style="font-size:0.85em; margin-bottom:6px;">
        <span id="autoStatus">Idle</span>
      </div>
      <div style="display:grid; grid-template-columns:1fr 1fr; gap:4px;">
        <div class="control-group"><label>Interval (ms)</label><input type="number" id="inputAutoInterval" value="500" min="100" step="100" /></div>
        <div class="control-group"><label>Radius (px)</label><input type="number" id="inputAutoRadius" value="40" min="10" /></div>
        <div class="control-group"><label>Threshold (%)</label><input type="number" id="inputAutoThreshold" value="15" min="1" max="100" /></div>
        <div class="control-group"><label>Scan ID</label><input type="number" id="inputAutoScanId" value="0" min="0" /></div>
      </div>
      <div class="btn-row" style="margin-top:6px;">
        <button id="btnAutoStart">Start Auto</button>
        <button id="btnAutoStop" class="danger">Stop Auto</button>
      </div>
    </div>

    <div class="panel" style="margin-top:12px;">
      <h2>Console</h2>
      <div id="log"></div>
    </div>
  </div>

  <div>
    <div class="panel">
      <h2>FSM State</h2>
      <div style="font-size:0.85em; margin-bottom:8px;">
        <div><strong>Mode:</strong> <span id="stateMode">--</span></div>
        <div><strong>User:</strong> <span id="stateUser">--</span></div>
        <div><strong>Climb:</strong> <span id="stateClimb">--</span></div>
        <div><strong>Proceed:</strong> <span id="stateProceed">--</span></div>
        <div><strong>Scan:</strong> <span id="stateScan">--</span> &nbsp; <strong>Route:</strong> <span id="stateRoute">--</span></div>
      </div>
      <div class="control-group">
        <label>Mode</label>
        <select id="selectMode"><option value="">--</option><option>MANUAL</option><option>USER</option></select>
      </div>
      <div class="control-group">
        <label>User State</label>
        <select id="selectUserState"><option value="">--</option><option>SCAN_WALL</option><option>SET_ROUTE</option><option>CLIMB_WALL</option></select>
      </div>
      <div class="control-group">
        <label>Climb State</label>
        <select id="selectClimbState"><option value="">--</option><option>IDLE</option><option>RUNNING</option><option>PAUSED</option></select>
      </div>
      <div class="control-group">
        <label>Proceed Mode</label>
        <select id="selectProceedMode"><option value="">--</option><option>MANUAL</option><option>TIMED</option><option>AUTO</option></select>
      </div>
      <div class="control-group">
        <label>Interval (ms)</label>
        <input type="number" id="inputInterval" value="3000" min="100" step="100" />
      </div>
      <div class="btn-row">
        <button id="btnSetState">Set State</button>
      </div>
    </div>

    <div class="panel" style="margin-top:12px;">
      <h2>Laser</h2>
      <div class="btn-row">
        <button id="btnLaserOn">ON</button>
        <button id="btnLaserOff">OFF</button>
        <button id="btnLaserToggle">Toggle</button>
      </div>
    </div>

    <div class="panel" style="margin-top:12px;">
      <h2>Route</h2>
      <div class="route-num-row">
        <label style="font-size:0.85em;">Route #</label>
        <input type="number" id="inputRouteNum" value="0" min="0" max="99" />
      </div>
      <div class="btn-row">
        <button id="btnLoadRoute">Load</button>
        <button id="btnDeleteRoute" class="danger">Delete</button>
      </div>
      <div class="control-group" style="margin-top:8px;">
        <label>Play Mode</label>
        <select id="selectPlayMode">
          <option value="sequential">Sequential (all gimbals per step)</option>
          <option value="leapfrog" selected>Leapfrog (one gimbal advances at a time)</option>
        </select>
      </div>
      <div class="control-group">
        <label>Auto-advance interval (ms, 0 = manual Next)</label>
        <input type="number" id="inputPlayInterval" value="3000" min="0" step="500" />
      </div>
      <div class="control-group">
        <label>Distance to wall (m)</label>
        <input type="number" id="inputDistance" value="3.0" min="0.1" step="0.1" />
      </div>
    </div>

    <div class="panel" style="margin-top:12px;display:none;">
      <h2>Mapping</h2>
      <div class="control-group">
        <label>H-FOV (deg)</label>
        <input type="number" id="inputHfov" value="120" step="0.1" />
      </div>
      <div class="control-group">
        <label>V-FOV (deg)</label>
        <input type="number" id="inputVfov" value="60" step="0.1" />
      </div>
      <div class="control-group">
        <label>Image Width (px)</label>
        <input type="number" id="inputWidth" value="" />
      </div>
      <div class="control-group">
        <label>Image Height (px)</label>
        <input type="number" id="inputHeight" value="1920" />
      </div>
      <div class="control-group">
        <label>Distance (m)</label>
        <input type="number" id="inputDist" value="3.0" step="0.1" />
      </div>
      <button id="btnSetMapping">Set Mapping</button>
    </div>

    <div class="panel" style="margin-top:12px;">
      <h2>Servo Control</h2>
      <div id="servoControls"></div>
    </div>

    <div class="panel" style="margin-top:12px;">
      <h2>Calibration</h2>
      <div style="font-size:0.85em; margin-bottom:8px;">
        <strong>Active:</strong> <span id="calibActive">--</span>
      </div>
      <div style="display:grid; grid-template-columns:1fr 1fr; gap:4px;">
        <div class="control-group"><label>fx</label><input type="number" id="inputCalibFx" step="0.01" value="143.76" /></div>
        <div class="control-group"><label>fy</label><input type="number" id="inputCalibFy" step="0.01" value="141.18" /></div>
        <div class="control-group"><label>cx</label><input type="number" id="inputCalibCx" step="0.01" value="156.02" /></div>
        <div class="control-group"><label>cy</label><input type="number" id="inputCalibCy" step="0.01" value="122.14" /></div>
        <div class="control-group"><label>k1</label><input type="number" id="inputCalibK1" step="0.0001" value="-0.2381" /></div>
        <div class="control-group"><label>k2</label><input type="number" id="inputCalibK2" step="0.0001" value="0.2381" /></div>
        <div class="control-group"><label>p1</label><input type="number" id="inputCalibP1" step="0.0001" value="0.0073" /></div>
        <div class="control-group"><label>p2</label><input type="number" id="inputCalibP2" step="0.0001" value="0.0013" /></div>
        <div class="control-group"><label>k3</label><input type="number" id="inputCalibK3" step="0.0001" value="-0.1131" /></div>
        <div class="control-group"><label>Width</label><input type="number" id="inputCalibW" value="2560" /></div>
        <div class="control-group"><label>Height</label><input type="number" id="inputCalibH" value="1920" /></div>
      </div>
      <div class="btn-row" style="margin-top:6px;">
        <button id="btnCalibSet">Apply</button>
        <button id="btnCalibClear" class="danger">Clear</button>
      </div>
    </div>

    <div class="panel" style="margin-top:12px;">
      <h2>Gimbal Calibration</h2>
      <div style="font-size:0.8em;color:#aaa;margin-bottom:8px;">
        Zero gimbals, click a point on the image to drive the selected gimbal there,
        then fine-tune offsets until laser/nozzle hits the correct spot.
        <strong>Click near image edges for lowest error.</strong>
      </div>
      <div class="btn-row">
        <button id="btnGimCalEnter">Enter Cal Mode</button>
        <button id="btnGimCalExit" class="danger" disabled>Exit Cal Mode</button>
      </div>
      <div class="control-group">
        <label>Gimbal to calibrate</label>
        <select id="selectGimbal"><option value="0">Gimbal 0</option><option value="1">Gimbal 1</option><option value="2">Gimbal 2</option><option value="3">Gimbal 3</option></select>
      </div>
      <div id="gimCalBaseAngles" style="font-size:0.8em;margin-bottom:8px;display:none;">
        Base: X=<span id="gimCalBaseX">--</span>°  Y=<span id="gimCalBaseY">--</span>°
        &nbsp;|&nbsp; Current: X=<span id="gimCalCurX">--</span>°  Y=<span id="gimCalCurY">--</span>°
      </div>
      <div class="control-group">
        <label>Step size (°)</label>
        <select id="gimCalStep" style="width:80px;">
          <option value="1" selected>1°</option>
          <option value="2">2°</option>
          <option value="5">5°</option>
        </select>
      </div>
      <div style="margin-bottom:8px;">
        <div style="font-size:0.8em;color:#aaa;margin-bottom:4px;">X axis &nbsp; offset: <span id="gimCalDxVal">0</span>°</div>
        <div class="btn-row">
          <button id="btnGimCalXLeft">◄ Left</button>
          <button id="btnGimCalXRight">Right ►</button>
        </div>
      </div>
      <div style="margin-bottom:10px;">
        <div style="font-size:0.8em;color:#aaa;margin-bottom:4px;">Y axis &nbsp; offset: <span id="gimCalDyVal">0</span>°</div>
        <div class="btn-row">
          <button id="btnGimCalYUp">▲ Up</button>
          <button id="btnGimCalYDown">▼ Down</button>
        </div>
      </div>
      <div class="btn-row">
        <button id="btnGimCalSave">Save Offset</button>
        <button id="btnGimCalClearAll" class="danger">Clear All Offsets</button>
      </div>
      <div id="gimCalOffsetDisplay" style="font-size:0.78em;color:#aaa;margin-top:6px;"></div>

      <h2 style="margin-top:12px;">Polynomial Calibration</h2>
      <div style="font-size:0.8em;color:#aaa;margin-bottom:6px;">
        Points: <span id="polyCalPoints">G0:0 G1:0 G2:0 G3:0</span>
      </div>
      <div class="btn-row">
        <button id="btnPolyCollect">Record Point</button>
        <button id="btnPolyFit">Fit &amp; Apply</button>
        <button id="btnPolyClear" class="danger">Clear Points</button>
      </div>
      <div id="polyCalCoeffs" style="font-size:0.72em;color:#888;margin-top:6px;white-space:pre-wrap;max-height:80px;overflow-y:auto;"></div>
    </div>
  </div>
</div>

<script>
const NUM_SERVOS = 8;
const CENTROID_RADIUS = 8;
const SELECTED_COLOR = '#e94560';
const UNSELECTED_COLOR = 'rgba(0, 200, 255, 0.7)';
const ROUTE_LINE_COLOR = 'rgba(233, 69, 96, 0.5)';

// All requests go to the Flask server (same origin) which proxies to ESP
const API = '';  // same origin

let wallImage = null;
let wallImageOriginalHeight = null;  // full sensor height before bottom-crop
const CROP_KEEP = 2 / 3;            // keep the top 2/3 of every loaded image
let allCentroids = [];
let selectedHolds = [];
let routeMode = null;
let cameraEnabled = false;

// Crop the bottom (1 - CROP_KEEP) fraction of an Image object.
// Sets wallImageOriginalHeight and returns a Promise<Image> of the cropped result.
function cropImage(img) {
  const origW = img.naturalWidth;
  const origH = img.naturalHeight;
  const keepH = Math.round(origH * CROP_KEEP);
  wallImageOriginalHeight = origH;
  const tmp = document.createElement('canvas');
  tmp.width = origW; tmp.height = keepH;
  tmp.getContext('2d').drawImage(img, 0, 0);
  return new Promise(resolve => {
    const out = new Image();
    out.onload = () => resolve(out);
    out.src = tmp.toDataURL('image/jpeg', 0.92);
  });
}

const canvas = document.getElementById('wallCanvas');
const ctx = canvas.getContext('2d');

function log(msg, cls = '') {
  const el = document.getElementById('log');
  const line = document.createElement('div');
  if (cls) line.className = cls;
  line.textContent = '[' + new Date().toLocaleTimeString() + '] ' + msg;
  el.appendChild(line);
  el.scrollTop = el.scrollHeight;
}

async function post(path, body = null) {
  try {
    const opts = { method: 'POST' };
    if (body !== null) {
      opts.headers = { 'Content-Type': 'application/json' };
      opts.body = JSON.stringify(body);
    }
    const res = await fetch(API + path, opts);
    const text = await res.text();
    log('POST ' + path + ' -> ' + res.status + ' ' + text);
    return { ok: res.ok, status: res.status, text };
  } catch (e) {
    log('POST ' + path + ' FAILED: ' + e.message, 'err');
    return { ok: false, status: 0, text: e.message };
  }
}

async function get(path) {
  try {
    const res = await fetch(API + path);
    log('GET ' + path + ' -> ' + res.status);
    return { ok: res.ok, status: res.status, response: res };
  } catch (e) {
    log('GET ' + path + ' FAILED: ' + e.message, 'err');
    return { ok: false, status: 0 };
  }
}

// -- Canvas drawing --

function drawScene() {
  if (!wallImage) return;
  canvas.width = wallImage.naturalWidth;
  canvas.height = wallImage.naturalHeight;
  canvas.style.display = 'block';
  document.getElementById('canvasPlaceholder').style.display = 'none';

  ctx.drawImage(wallImage, 0, 0);

  // Unselected centroids (detect mode only)
  if (routeMode === 'detect') {
    for (const c of allCentroids) {
      if (selectedHolds.find(h => h.x === c.x && h.y === c.y)) continue;
      ctx.beginPath();
      ctx.arc(c.x, c.y, CENTROID_RADIUS, 0, Math.PI * 2);
      ctx.fillStyle = UNSELECTED_COLOR;
      ctx.fill();
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 1;
      ctx.stroke();
    }
  }

  // Route line
  if (selectedHolds.length > 1) {
    ctx.beginPath();
    ctx.moveTo(selectedHolds[0].x, selectedHolds[0].y);
    for (let i = 1; i < selectedHolds.length; i++) {
      ctx.lineTo(selectedHolds[i].x, selectedHolds[i].y);
    }
    ctx.strokeStyle = ROUTE_LINE_COLOR;
    ctx.lineWidth = 3;
    ctx.stroke();
  }

  // Selected holds
  for (let i = 0; i < selectedHolds.length; i++) {
    const h = selectedHolds[i];
    ctx.beginPath();
    ctx.arc(h.x, h.y, CENTROID_RADIUS, 0, Math.PI * 2);
    ctx.fillStyle = SELECTED_COLOR;
    ctx.fill();
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 10px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(String(i + 1), h.x, h.y);
  }
}

function updateHoldList() {
  const el = document.getElementById('holdList');
  el.innerHTML = '';
  selectedHolds.forEach((h, i) => {
    const div = document.createElement('div');
    div.className = 'hold-item';
    div.innerHTML = '<span>#' + (i+1) + ': (' + Math.round(h.x) + ', ' + Math.round(h.y) + ')</span>' +
      '<button class="remove-hold" data-idx="' + i + '">&times;</button>';
    el.appendChild(div);
  });
  el.querySelectorAll('.remove-hold').forEach(btn => {
    btn.addEventListener('click', () => {
      selectedHolds.splice(parseInt(btn.dataset.idx), 1);
      updateHoldList();
      drawScene();
    });
  });
}

// -- Canvas clicks --

canvas.addEventListener('click', (e) => {
  if (gimCalMode) return;
  if (!routeMode) return;
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const cx = (e.clientX - rect.left) * scaleX;
  const cy = (e.clientY - rect.top) * scaleY;

  if (routeMode === 'detect') {
    let nearest = null, nearestDist = Infinity;
    for (const c of allCentroids) {
      const d = Math.hypot(c.x - cx, c.y - cy);
      if (d < nearestDist) { nearest = c; nearestDist = d; }
    }
    if (nearest && nearestDist < CENTROID_RADIUS * 3) {
      const idx = selectedHolds.findIndex(h => h.x === nearest.x && h.y === nearest.y);
      if (idx >= 0) selectedHolds.splice(idx, 1);
      else selectedHolds.push({ x: nearest.x, y: nearest.y });
    }
  } else if (routeMode === 'manual') {
    selectedHolds.push({ x: Math.round(cx), y: Math.round(cy) });
  }

  updateHoldList();
  drawScene();
});

// -- Camera --

document.getElementById('btnEnableCam').addEventListener('click', async () => {
  if (!cameraEnabled) {
    await post('/esp/configure', { enabled: true, format: 'JPEG' });
    cameraEnabled = true;
    document.getElementById('btnEnableCam').textContent = 'Disable Camera';
    document.getElementById('btnEnableCam').classList.add('active');
    document.getElementById('btnCapture').disabled = false;
    document.getElementById('btnFetchImage').disabled = false;
    document.getElementById('btnLiveView').disabled = false;
  } else {
    await post('/esp/configure', { enabled: false });
    cameraEnabled = false;
    document.getElementById('btnEnableCam').textContent = 'Enable Camera';
    document.getElementById('btnEnableCam').classList.remove('active');
    document.getElementById('btnCapture').disabled = true;
    document.getElementById('btnFetchImage').disabled = true;
    document.getElementById('btnLiveView').disabled = true;
    if (liveViewActive) { liveViewActive = false; clearInterval(liveViewTimer); document.getElementById('btnLiveView').textContent = 'Live View'; document.getElementById('btnLiveView').classList.remove('active'); }
  }
});

document.getElementById('btnCapture').addEventListener('click', () => post('/esp/capture'));

document.getElementById('btnUploadImage').addEventListener('click', () => {
  document.getElementById('fileInput').click();
});
document.getElementById('fileInput').addEventListener('change', (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const url = URL.createObjectURL(file);
  const img = new Image();
  img.onload = async () => { wallImage = await cropImage(img); drawScene(); log('Uploaded: ' + file.name + ' (' + img.naturalWidth + 'x' + img.naturalHeight + ', cropped to ' + wallImage.naturalHeight + 'px tall)', 'info'); };
  img.src = url;
  e.target.value = '';
});

document.getElementById('btnFetchImage').addEventListener('click', async () => {
  try {
    const res = await fetch(API + '/esp/get');
    if (!res.ok) { log('GET /esp/get failed: ' + res.status, 'err'); return; }
    const blob = await res.blob();
    const url = URL.createObjectURL(blob);
    const img = new Image();
    img.onload = async () => { wallImage = await cropImage(img); drawScene(); log('Image loaded: ' + img.naturalWidth + 'x' + img.naturalHeight + ', cropped to ' + wallImage.naturalHeight + 'px tall', 'info'); };
    img.src = url;
  } catch (e) { log('Fetch image failed: ' + e.message, 'err'); }
});

// -- Route creation --

function enterRouteMode(mode) {
  routeMode = mode;
  selectedHolds = [];
  allCentroids = [];
  updateHoldList();
  document.getElementById('btnFinishRoute').disabled = false;
  document.getElementById('btnCancelRoute').disabled = false;
  document.getElementById('btnNewRoute').disabled = true;
  document.getElementById('btnManualRoute').disabled = true;
  const banner = document.getElementById('modeBanner');
  banner.textContent = mode === 'detect'
    ? 'ROUTE CREATION — Click detected holds to add to route'
    : 'ROUTE CREATION — Click anywhere on image to place holds';
  banner.style.display = 'block';
}

function exitRouteMode() {
  routeMode = null;
  document.getElementById('modeBanner').style.display = 'none';
  document.getElementById('btnFinishRoute').disabled = true;
  document.getElementById('btnCancelRoute').disabled = true;
  document.getElementById('btnNewRoute').disabled = false;
  document.getElementById('btnManualRoute').disabled = false;
  drawScene();
}

document.getElementById('btnNewRoute').addEventListener('click', async () => {
  if (!wallImage) { log('Capture an image first', 'err'); return; }
  enterRouteMode('detect');
  log('Running hold detection...', 'info');

  try {
    const blob = await new Promise((resolve, reject) => {
      canvas.toBlob(b => b ? resolve(b) : reject(new Error('Canvas export failed')), 'image/jpeg', 0.95);
    });

    const formData = new FormData();
    formData.append('image', blob, 'capture.jpg');

    const detectRes = await fetch(API + '/detect', { method: 'POST', body: formData });
    if (!detectRes.ok) throw new Error('Detection returned ' + detectRes.status);
    const data = await detectRes.json();
    const centroids = data.centroids || data.holds || [];

    allCentroids = centroids.map(c => Array.isArray(c) ? { x: c[0], y: c[1] } : c);
    log('Detected ' + allCentroids.length + ' holds', 'info');
    drawScene();
  } catch (e) {
    log('Hold detection failed: ' + e.message, 'err');
    exitRouteMode();
  }
});

document.getElementById('btnManualRoute').addEventListener('click', () => {
  if (!wallImage) { log('Capture an image first', 'err'); return; }
  enterRouteMode('manual');
});

document.getElementById('btnFinishRoute').addEventListener('click', async () => {
  if (selectedHolds.length === 0) { log('No holds selected', 'err'); return; }
  const routeNum = parseInt(document.getElementById('inputRouteNum').value);
  const holds = selectedHolds.map(h => [h.x, h.y]);
  await post('/esp/route/create', { route: routeNum, holds });
  log('Route ' + routeNum + ' saved with ' + holds.length + ' holds', 'info');
  exitRouteMode();
});

document.getElementById('btnCancelRoute').addEventListener('click', () => { exitRouteMode(); log('Route creation cancelled'); });

// -- Route management --

document.getElementById('btnLoadRoute').addEventListener('click', () => {
  post('/esp/route/load', { route: parseInt(document.getElementById('inputRouteNum').value) });
});
document.getElementById('btnDeleteRoute').addEventListener('click', () => {
  post('/esp/route/delete', { route: parseInt(document.getElementById('inputRouteNum').value) });
});

// -- Playback --

document.getElementById('btnPlay').addEventListener('click', () => {
  const mode = document.getElementById('selectPlayMode').value;
  const interval = parseInt(document.getElementById('inputPlayInterval').value) || 0;
  const body = {
    route: parseInt(document.getElementById('inputRouteNum').value),
    mode,
    interval_ms: interval,
  };
  if (mode === 'leapfrog') body.gimbals = NUM_SERVOS / 2;
  // Always send image dimensions so the ESP32 maps pixel coordinates correctly.
  // Falls back to the default 320x240 if no image is loaded.
  body.image_width  = wallImage ? wallImage.naturalWidth           : 2560;
  body.image_height = wallImageOriginalHeight ?? (wallImage ? wallImage.naturalHeight : 1920);
  body.hfov_deg = parseFloat(document.getElementById('inputHfov').value) || 120;
  body.vfov_deg = parseFloat(document.getElementById('inputVfov').value) || 60;
  body.distance_m = parseFloat(document.getElementById('inputDistance').value) || 3.0;
  post('/esp/route/play', body);
});
document.getElementById('btnPause').addEventListener('click', () => get('/esp/route/pause'));
document.getElementById('btnNext').addEventListener('click', () => get('/esp/route/next'));
document.getElementById('btnRestart').addEventListener('click', () => post('/esp/route/restart'));
document.getElementById('btnStop').addEventListener('click', () => get('/esp/stop'));

// -- Mapping --

document.getElementById('btnSetMapping').addEventListener('click', () => {
  post('/esp/route/mapping', {
    hfov_deg: parseFloat(document.getElementById('inputHfov').value),
    vfov_deg: parseFloat(document.getElementById('inputVfov').value),
    image_width: parseInt(document.getElementById('inputWidth').value),
    image_height: parseInt(document.getElementById('inputHeight').value),
    distance_m: parseFloat(document.getElementById('inputDist').value),
  });
});

// -- Servos --

(function buildServoControls() {
  const container = document.getElementById('servoControls');
  for (let i = 0; i < NUM_SERVOS; i++) {
    const row = document.createElement('div');
    row.className = 'servo-row';
    row.innerHTML = '<label>Servo ' + i + '</label>' +
      '<input type="range" min="0" max="180" value="90" data-servo="' + i + '" />' +
      '<span>90</span>';
    container.appendChild(row);
    const slider = row.querySelector('input[type="range"]');
    const display = row.querySelector('span');
    let debounce = null;
    slider.addEventListener('input', () => {
      display.textContent = slider.value;
      clearTimeout(debounce);
      debounce = setTimeout(() => post('/esp/servo', { servo: i, angle: parseInt(slider.value) }), 150);
    });
  }
})();

// -- Server log polling --
let lastLogId = 0;
async function pollServerLogs() {
  try {
    const res = await fetch(API + '/logs?since=' + lastLogId);
    if (!res.ok) return;
    const entries = await res.json();
    const el = document.getElementById('log');
    for (const e of entries) {
      const line = document.createElement('div');
      line.className = 'srv' + (e.level === 'err' ? ' err' : e.level === 'warn' ? ' warn' : '');
      line.textContent = '[' + e.ts + '] [SERVER] ' + e.msg;
      el.appendChild(line);
      lastLogId = e.id;
    }
    if (entries.length > 0) el.scrollTop = el.scrollHeight;
  } catch (e) { /* ignore poll failures */ }
}
setInterval(pollServerLogs, 1000);

// -- FSM state polling --
async function pollState() {
  try {
    const res = await fetch(API + '/esp/state');
    if (!res.ok) return;
    const s = await res.json();
    document.getElementById('stateMode').textContent = s.mode || '--';
    document.getElementById('stateUser').textContent = s.user_state || '--';
    document.getElementById('stateClimb').textContent = s.climb_state || '--';
    document.getElementById('stateProceed').textContent = s.proceed_mode || '--';
    document.getElementById('stateScan').textContent = s.active_scan ?? '--';
    document.getElementById('stateRoute').textContent = s.active_route ?? '--';
  } catch (e) { /* ignore */ }
}
setInterval(pollState, 2000);
pollState();

// -- FSM state set --
document.getElementById('btnSetState').addEventListener('click', async () => {
  const body = {};
  const mode = document.getElementById('selectMode').value;
  if (mode) body.mode = mode;
  const us = document.getElementById('selectUserState').value;
  if (us) body.user_state = us;
  const cs = document.getElementById('selectClimbState').value;
  if (cs) body.climb_state = cs;
  const pm = document.getElementById('selectProceedMode').value;
  if (pm) {
    body.proceed_mode = pm;
    body.interval_ms = parseInt(document.getElementById('inputInterval').value) || 3000;
  }
  await post('/esp/state', body);
  pollState();
});

// -- Laser --
document.getElementById('btnLaserOn').addEventListener('click', () => post('/esp/laser', { on: true }));
document.getElementById('btnLaserOff').addEventListener('click', () => post('/esp/laser', { on: false }));
document.getElementById('btnLaserToggle').addEventListener('click', () => post('/esp/laser', {}));

// -- Live view --
let liveViewActive = false;
let liveViewTimer = null;
document.getElementById('btnLiveView').addEventListener('click', () => {
  liveViewActive = !liveViewActive;
  const btn = document.getElementById('btnLiveView');
  if (liveViewActive) {
    btn.textContent = 'Stop Live';
    btn.classList.add('active');
    liveViewTimer = setInterval(async () => {
      try {
        await fetch(API + '/esp/capture', { method: 'POST' });
        await new Promise(r => setTimeout(r, 200));
        const res = await fetch(API + '/esp/get');
        if (!res.ok) return;
        const blob = await res.blob();
        const url = URL.createObjectURL(blob);
        const img = new Image();
        img.onload = async () => { wallImage = await cropImage(img); drawScene(); };
        img.src = url;
      } catch (e) { /* ignore */ }
    }, 1500);
  } else {
    btn.textContent = 'Live View';
    btn.classList.remove('active');
    clearInterval(liveViewTimer);
  }
});

// -- Recording --
let recPollTimer = null;

document.getElementById('btnRecordStart').addEventListener('click', async () => {
  const res = await post('/record/start', { interval: 0.5 });
  if (res.ok) {
    document.getElementById('btnRecordStart').disabled = true;
    document.getElementById('btnRecordStop').disabled = false;
    document.getElementById('recStatus').style.display = 'block';
    document.getElementById('recStatus').textContent = 'Recording...';
    recPollTimer = setInterval(async () => {
      try {
        const r = await fetch(API + '/record/status');
        if (!r.ok) return;
        const s = await r.json();
        document.getElementById('recStatus').textContent = s.recording
          ? `Recording: ${s.frames} frames — ${s.file}`
          : `Stopped: ${s.frames} frames — ${s.file}`;
        if (!s.recording) {
          clearInterval(recPollTimer);
          document.getElementById('btnRecordStart').disabled = false;
          document.getElementById('btnRecordStop').disabled = true;
        }
      } catch (e) { /* ignore */ }
    }, 1000);
  }
});

document.getElementById('btnRecordStop').addEventListener('click', async () => {
  await post('/record/stop');
  document.getElementById('btnRecordStart').disabled = false;
  document.getElementById('btnRecordStop').disabled = true;
  if (recPollTimer) { clearInterval(recPollTimer); recPollTimer = null; }
});

// -- Calibration --
async function loadCalibValues() {
  try {
    const res = await fetch(API + '/calibrate/get');
    if (!res.ok) return;
    const s = await res.json();
    if (s.calibrated) {
      document.getElementById('calibActive').textContent = 'Yes (' + s.width + 'x' + s.height + ')';
      document.getElementById('inputCalibFx').value = s.fx;
      document.getElementById('inputCalibFy').value = s.fy;
      document.getElementById('inputCalibCx').value = s.cx;
      document.getElementById('inputCalibCy').value = s.cy;
      document.getElementById('inputCalibK1').value = s.k1;
      document.getElementById('inputCalibK2').value = s.k2;
      document.getElementById('inputCalibP1').value = s.p1;
      document.getElementById('inputCalibP2').value = s.p2;
      document.getElementById('inputCalibK3').value = s.k3;
      document.getElementById('inputCalibW').value = s.width;
      document.getElementById('inputCalibH').value = s.height;
    } else {
      document.getElementById('calibActive').textContent = 'No';
    }
  } catch (e) { /* ignore */ }
}

document.getElementById('btnCalibSet').addEventListener('click', async () => {
  const body = {
    fx: parseFloat(document.getElementById('inputCalibFx').value),
    fy: parseFloat(document.getElementById('inputCalibFy').value),
    cx: parseFloat(document.getElementById('inputCalibCx').value),
    cy: parseFloat(document.getElementById('inputCalibCy').value),
    k1: parseFloat(document.getElementById('inputCalibK1').value),
    k2: parseFloat(document.getElementById('inputCalibK2').value),
    p1: parseFloat(document.getElementById('inputCalibP1').value),
    p2: parseFloat(document.getElementById('inputCalibP2').value),
    k3: parseFloat(document.getElementById('inputCalibK3').value),
    width: parseInt(document.getElementById('inputCalibW').value),
    height: parseInt(document.getElementById('inputCalibH').value),
  };
  const res = await post('/calibrate/set', body);
  if (res.ok) {
    document.getElementById('calibActive').textContent = 'Yes (' + body.width + 'x' + body.height + ')';
    log('Calibration applied: fx=' + body.fx + ' fy=' + body.fy, 'info');
  }
});

document.getElementById('btnCalibClear').addEventListener('click', async () => {
  await post('/calibrate/clear');
  document.getElementById('calibActive').textContent = 'No';
  log('Calibration cleared');
});

loadCalibValues();

// -- Auto Advance --
let autoPollTimer = null;

async function pollAutoStatus() {
  try {
    const res = await fetch(API + '/auto/status');
    if (!res.ok) return;
    const s = await res.json();
    const el = document.getElementById('autoStatus');
    if (s.running) {
      el.textContent = s.msg || ('Hold ' + (s.current_hold + 1) + '/' + s.total_holds);
      el.style.color = '#8f8';
    } else {
      el.textContent = s.msg || 'Idle';
      el.style.color = '#e0e0e0';
      if (autoPollTimer) { clearInterval(autoPollTimer); autoPollTimer = null; }
    }
  } catch (e) { /* ignore */ }
}

document.getElementById('btnAutoStart').addEventListener('click', async () => {
  const body = {
    scan_id: parseInt(document.getElementById('inputAutoScanId').value) || 0,
    interval_ms: parseInt(document.getElementById('inputAutoInterval').value) || 500,
    radius_px: parseInt(document.getElementById('inputAutoRadius').value) || 40,
    threshold_pct: parseInt(document.getElementById('inputAutoThreshold').value) || 15,
  };
  const res = await post('/auto/start', body);
  if (res.ok) {
    log('Auto advance started', 'info');
    autoPollTimer = setInterval(pollAutoStatus, 1000);
  }
});

document.getElementById('btnAutoStop').addEventListener('click', async () => {
  await post('/auto/stop');
  log('Auto advance stopped');
  pollAutoStatus();
});

log('BetaSpray frontend loaded. Enable camera to begin.', 'info');

// ---------------------------------------------------------------------------
// Gimbal Calibration
// ---------------------------------------------------------------------------
let gimCalMode = false;
let gimCalBaseX = null, gimCalBaseY = null;
let gimCalDx = 0, gimCalDy = 0;

async function loadGimCalOffsets() {
  try {
    const res = await fetch(API + '/gimcal/offsets');
    if (!res.ok) return;
    const offsets = await res.json();
    let s = '';
    offsets.forEach((o, i) => { s += `G${i}: dx=${o.dx>=0?'+':''}${o.dx}° dy=${o.dy>=0?'+':''}${o.dy}°  `; });
    document.getElementById('gimCalOffsetDisplay').textContent = s.trim();
  } catch (e) { /* ignore */ }
}

function gimCalUpdateDisplay() {
  document.getElementById('gimCalDxVal').textContent = (gimCalDx >= 0 ? '+' : '') + gimCalDx;
  document.getElementById('gimCalDyVal').textContent = (gimCalDy >= 0 ? '+' : '') + gimCalDy;
  if (gimCalBaseX !== null) {
    document.getElementById('gimCalCurX').textContent = Math.max(0, Math.min(180, gimCalBaseX + gimCalDx));
    document.getElementById('gimCalCurY').textContent = Math.max(0, Math.min(180, gimCalBaseY + gimCalDy));
  }
}

async function gimCalDrive() {
  if (!gimCalMode || gimCalBaseX === null) return;
  const g  = parseInt(document.getElementById('selectGimbal').value);
  const ax = Math.max(0, Math.min(180, gimCalBaseX + gimCalDx));
  const ay = Math.max(0, Math.min(180, gimCalBaseY + gimCalDy));
  // Use /gimcal/drive which bypasses rate limiting and drives both servos atomically
  await fetch(API + '/gimcal/drive', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ gimbal: g, angle_x: ax, angle_y: ay }),
  });
  gimCalUpdateDisplay();
}

document.getElementById('btnGimCalEnter').addEventListener('click', async () => {
  log('[GIMCAL] Zeroing gimbals...', 'info');
  await post('/gimcal/start');
  gimCalMode = true;
  gimCalBaseX = null; gimCalBaseY = null;
  gimCalDx = 0; gimCalDy = 0;
  gimCalUpdateDisplay();
  document.getElementById('gimCalBaseAngles').style.display = 'none';
  document.getElementById('btnGimCalEnter').disabled = true;
  document.getElementById('btnGimCalExit').disabled = false;
  document.getElementById('modeBanner').textContent = 'GIMBAL CALIBRATION — Click near image edges to drive selected gimbal';
  document.getElementById('modeBanner').style.display = 'block';
  await loadGimCalOffsets();
  log('[GIMCAL] Ready — click image to point gimbal', 'info');
});

document.getElementById('btnGimCalExit').addEventListener('click', () => {
  gimCalMode = false;
  document.getElementById('btnGimCalEnter').disabled = false;
  document.getElementById('btnGimCalExit').disabled = true;
  document.getElementById('modeBanner').style.display = 'none';
  document.getElementById('gimCalBaseAngles').style.display = 'none';
  log('[GIMCAL] Exited calibration mode');
});

document.getElementById('btnGimCalXLeft').addEventListener('click', () => {
  gimCalDx -= parseInt(document.getElementById('gimCalStep').value);
  gimCalDrive();
});
document.getElementById('btnGimCalXRight').addEventListener('click', () => {
  gimCalDx += parseInt(document.getElementById('gimCalStep').value);
  gimCalDrive();
});
document.getElementById('btnGimCalYUp').addEventListener('click', () => {
  gimCalDy += parseInt(document.getElementById('gimCalStep').value);
  gimCalDrive();
});
document.getElementById('btnGimCalYDown').addEventListener('click', () => {
  gimCalDy -= parseInt(document.getElementById('gimCalStep').value);
  gimCalDrive();
});

document.getElementById('btnGimCalSave').addEventListener('click', async () => {
  const g = parseInt(document.getElementById('selectGimbal').value);
  const res = await post('/gimcal/offset', { gimbal: g, dx: gimCalDx, dy: gimCalDy });
  if (res.ok) {
    log(`[GIMCAL] Offset saved: gimbal ${g} dx=${gimCalDx>=0?'+':''}${gimCalDx}° dy=${gimCalDy>=0?'+':''}${gimCalDy}°`, 'info');
    await loadGimCalOffsets();
  }
});

document.getElementById('btnGimCalClearAll').addEventListener('click', async () => {
  for (let g = 0; g < 4; g++) await post('/gimcal/offset', { gimbal: g, dx: 0, dy: 0 });
  gimCalDx = 0; gimCalDy = 0;
  gimCalUpdateDisplay();
  log('[GIMCAL] All offsets cleared', 'info');
  await loadGimCalOffsets();
});

// Intercept canvas clicks in calibration mode
canvas.addEventListener('click', async (e) => {
  if (!gimCalMode) return;
  if (!wallImage) { log('[GIMCAL] Load an image first', 'err'); return; }
  const rect = canvas.getBoundingClientRect();
  const px = (e.clientX - rect.left) * (canvas.width / rect.width);
  const py = (e.clientY - rect.top)  * (canvas.height / rect.height);
  const g  = parseInt(document.getElementById('selectGimbal').value);
  const res = await fetch(API + '/gimcal/point', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ gimbal: g, x: Math.round(px), y: Math.round(py),
                           image_width: canvas.width, image_height: wallImageOriginalHeight ?? canvas.height }),
  });
  if (!res.ok) { log('[GIMCAL] Point drive failed', 'err'); return; }
  const data = await res.json();
  gimCalBaseX = data.angle_x;
  gimCalBaseY = data.angle_y;
  gimCalDx = 0; gimCalDy = 0;
  gimCalUpdateDisplay();
  document.getElementById('gimCalBaseX').textContent = data.angle_x;
  document.getElementById('gimCalBaseY').textContent = data.angle_y;
  document.getElementById('gimCalCurX').textContent  = data.angle_x;
  document.getElementById('gimCalCurY').textContent  = data.angle_y;
  document.getElementById('gimCalBaseAngles').style.display = 'block';
  log(`[GIMCAL] G${g} -> pixel(${Math.round(px)},${Math.round(py)}) X=${data.angle_x}° Y=${data.angle_y}°`, 'info');
}, true);

loadGimCalOffsets();

// ---------------------------------------------------------------------------
// Polynomial Calibration
// ---------------------------------------------------------------------------
let polyCalCounts = [0, 0, 0, 0];

async function polyCalUpdatePoints() {
  try {
    const res = await fetch(API + '/gimcal/poly/points');
    if (!res.ok) return;
    const data = await res.json();
    let s = '';
    for (let g = 0; g < 4; g++) {
      const pts = data['g' + g] || [];
      polyCalCounts[g] = pts.length;
      s += `G${g}:${pts.length} `;
    }
    document.getElementById('polyCalPoints').textContent = s.trim();
  } catch (e) { /* ignore */ }
}

async function polyCalUpdateCoeffs() {
  try {
    const res = await fetch(API + '/gimcal/poly/coeffs');
    if (!res.ok) return;
    const data = await res.json();
    const gimbals = data.gimbals || [];
    let s = '';
    gimbals.forEach((c, g) => {
      const allZero = c.every(v => Math.abs(v) < 1e-9);
      if (!allZero) {
        s += `G${g} X: ${c.slice(0,4).map(v=>v.toFixed(4)).join(', ')}\n`;
        s += `G${g} Y: ${c.slice(4,8).map(v=>v.toFixed(4)).join(', ')}\n`;
      }
    });
    document.getElementById('polyCalCoeffs').textContent = s || '(no polynomial set)';
  } catch (e) { /* ignore */ }
}

document.getElementById('btnPolyCollect').addEventListener('click', async () => {
  if (!gimCalMode || gimCalBaseX === null) {
    log('[POLYCAL] Enter cal mode and click a target point first', 'err');
    return;
  }
  const g = parseInt(document.getElementById('selectGimbal').value);
  const body = {
    gimbal: g,
    computed_x: gimCalBaseX,
    computed_y: gimCalBaseY,
    actual_x: Math.max(0, Math.min(180, gimCalBaseX + gimCalDx)),
    actual_y: Math.max(0, Math.min(180, gimCalBaseY + gimCalDy)),
  };
  const res = await post('/gimcal/poly/collect', body);
  if (res.ok) {
    log(`[POLYCAL] G${g} point recorded: computed(${body.computed_x},${body.computed_y}) actual(${body.actual_x},${body.actual_y})`, 'info');
    polyCalUpdatePoints();
  }
});

document.getElementById('btnPolyFit').addEventListener('click', async () => {
  log('[POLYCAL] Fitting polynomial...', 'info');
  const res = await post('/gimcal/poly/fit', {});
  if (res.ok) {
    try {
      const data = JSON.parse(res.text);
      for (const [key, val] of Object.entries(data.results || {})) {
        if (val.rms_x !== undefined) {
          log(`[POLYCAL] ${key}: rms_x=${val.rms_x}° rms_y=${val.rms_y}° (${val.points} pts)`, 'info');
        } else if (val.error) {
          log(`[POLYCAL] ${key}: ${val.error}`, 'warn');
        }
      }
    } catch (e) { /* ok */ }
    polyCalUpdateCoeffs();
  }
});

document.getElementById('btnPolyClear').addEventListener('click', async () => {
  await post('/gimcal/poly/clear', {});
  log('[POLYCAL] Calibration points cleared');
  polyCalUpdatePoints();
});

polyCalUpdatePoints();
polyCalUpdateCoeffs();
</script>
</body>
</html>
"""

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global ESP_BASE, DETECT_MODE, DOG_BINARY

    parser = argparse.ArgumentParser(description="BetaSpray Web Application")
    parser.add_argument("--esp", default=DEFAULT_ESP_HOST,
                        help=f"ESP32 IP address (default: {DEFAULT_ESP_HOST})")
    parser.add_argument("--esp-port", type=int, default=DEFAULT_ESP_PORT,
                        help=f"ESP32 HTTP port (default: {DEFAULT_ESP_PORT})")
    parser.add_argument("--port", type=int, default=DEFAULT_APP_PORT,
                        help=f"Flask server port (default: {DEFAULT_APP_PORT})")
    parser.add_argument("--detect", choices=["host", "esp"], default="host",
                        help="Hold detection mode: 'host' runs CV pipeline locally, 'esp' proxies to device (default: host)")

    args = parser.parse_args()
    ESP_BASE = f"http://{args.esp}:{args.esp_port}"
    DETECT_MODE = args.detect

    # Load saved calibration
    if HAS_CV2 and _load_calibration():
        print(f"  Calibration: loaded ({_calib_res[0]}x{_calib_res[1]}, fx={_calib_K[0,0]:.1f})")
    else:
        print(f"  Calibration: none (set from UI or place {CALIB_FILE.name})")

    _load_gimbal_offsets()
    print(f"  Gimbal offsets: {_gimbal_offsets}")

    print(f"BetaSpray Web App")
    print(f"  ESP32:     {ESP_BASE}")
    print(f"  Detection: {DETECT_MODE}" + (" (OpenCV pipeline)" if DETECT_MODE == "host" else ""))
    print(f"  Frontend:  http://localhost:{args.port}")
    print()

    import logging

    class QuietLogFilter(logging.Filter):
        def filter(self, record):
            return '/logs?since=' not in record.getMessage()

    logging.getLogger('werkzeug').addFilter(QuietLogFilter())

    app.run(host="0.0.0.0", port=args.port, debug=False)

if __name__ == "__main__":
    main()
