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
import sys
import tempfile
import time
import traceback
from pathlib import Path
from typing import List, Tuple

import requests
from flask import Flask, Response, jsonify, request, send_file

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
      </div>

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
        <input type="number" id="inputHeight" value="240" />
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
  </div>
</div>

<script>
const NUM_SERVOS = 2;
const CENTROID_RADIUS = 8;
const SELECTED_COLOR = '#e94560';
const UNSELECTED_COLOR = 'rgba(0, 200, 255, 0.7)';
const ROUTE_LINE_COLOR = 'rgba(233, 69, 96, 0.5)';

// All requests go to the Flask server (same origin) which proxies to ESP
const API = '';  // same origin

let wallImage = null;
let allCentroids = [];
let selectedHolds = [];
let routeMode = null;
let cameraEnabled = false;

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

document.getElementById('btnFetchImage').addEventListener('click', async () => {
  try {
    const res = await fetch(API + '/esp/get');
    if (!res.ok) { log('GET /esp/get failed: ' + res.status, 'err'); return; }
    const blob = await res.blob();
    const url = URL.createObjectURL(blob);
    const img = new Image();
    img.onload = () => { wallImage = img; drawScene(); log('Image loaded: ' + img.naturalWidth + 'x' + img.naturalHeight, 'info'); };
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
    // Fetch the current JPEG from ESP via proxy, then send to /detect
    const imgRes = await fetch(API + '/esp/get');
    if (!imgRes.ok) throw new Error('Failed to fetch image');
    const blob = await imgRes.blob();

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
  post('/esp/route/play', { route: parseInt(document.getElementById('inputRouteNum').value) });
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
        img.onload = () => { wallImage = img; drawScene(); };
        img.src = url;
      } catch (e) { /* ignore */ }
    }, 1500);
  } else {
    btn.textContent = 'Live View';
    btn.classList.remove('active');
    clearInterval(liveViewTimer);
  }
});

log('BetaSpray frontend loaded. Enable camera to begin.', 'info');
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
