#!/usr/bin/env python3
"""
BetaSpray HTTP Client
Interfaces with the ESP32-S3 over HTTP for camera, route, and servo control.

Modified by ingi on March 7 to add "interactive/terminal session" mode
"""

import requests
import json
import argparse
import sys
import struct
import shlex
import subprocess
import math
import threading
import time
from typing import List, Tuple, Optional
from pathlib import Path

try:
    import readline
except ImportError:
    pass

try:
    import cv2
    import numpy as np
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

try:
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 80

class BetaSprayClient:
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT):
        self.base_url = f"http://{host}:{port}"
        self.session = requests.Session()
        self.session.timeout = 10
        self.pending_holds: List[Tuple[float, float]] = []  # holds from last detect run

    def _post(self, endpoint: str, json_data: Optional[dict] = None, data: Optional[bytes] = None):
        """Send POST request."""
        url = f"{self.base_url}{endpoint}"
        try:
            if json_data:
                resp = self.session.post(url, json=json_data)
            else:
                resp = self.session.post(url, data=data)
            resp.raise_for_status()
            return resp
        except requests.RequestException as e:
            print(f"Error: {e}", file=sys.stderr)
            raise

    def _get(self, endpoint: str):
        """Send GET request."""
        url = f"{self.base_url}{endpoint}"
        try:
            resp = self.session.get(url)
            resp.raise_for_status()
            return resp
        except requests.RequestException as e:
            print(f"Error: {e}", file=sys.stderr)
            raise

    # Camera endpoints
    def capture(self):
        """Trigger camera capture and store frame."""
        resp = self._post("/capture")
        print(f"Capture: {resp.text}")

    def get_frame(self, output_file: str = "capture.jpg"):
        """Retrieve and save captured frame as JPEG."""
        resp = self._get("/get")
        if resp.headers.get("content-type") == "image/jpeg":
            with open(output_file, "wb") as f:
                f.write(resp.content)
            print(f"Frame saved to {output_file}")
        else:
            print(f"Response: {resp.text}")

    def configure_camera(self, enabled: Optional[bool] = None, resolution: Optional[str] = None,
                         format: Optional[str] = None, psram: Optional[bool] = None):
        """Configure camera settings."""
        config = {}
        if enabled is not None:
            config["enabled"] = enabled
        if resolution:
            config["resolution"] = resolution
        if format:
            config["format"] = format
        if psram is not None:
            config["psram"] = psram

        resp = self._post("/configure", json_data=config)
        print(f"Configure: {resp.text}")

    # Route endpoints
    def create_route(self, route_num: int, holds: List[Tuple[float, float]]):
        """Create a route with list of (x, y) holds."""
        data = {
            "route": route_num,
            "holds": [[x, y] for x, y in holds]
        }
        resp = self._post("/route/create", json_data=data)
        print(f"Create route {route_num}: {resp.text}")

    def create_route_binary(self, route_num: int, holds: List[Tuple[float, float]]):
        """Create a route using binary format (direct file write for testing)."""
        buf = struct.pack("<I", len(holds))  # hold count as little-endian uint32
        for x, y in holds:
            buf += struct.pack("<ff", x, y)  # x, y as little-endian float32

        data = {
            "route": route_num,
            "holds": [[x, y] for x, y in holds]
        }
        resp = self._post("/route/create", json_data=data)
        print(f"Create route {route_num} (binary): {resp.text}")

    def delete_route(self, route_num: int):
        """Delete a route file."""
        data = {"route": route_num}
        resp = self._post("/route/delete", json_data=data)
        print(f"Delete route {route_num}: {resp.text}")

    def load_route(self, route_num: int):
        """Load a route from storage into RAM."""
        data = {"route": route_num}
        resp = self._post("/route/load", json_data=data)
        print(f"Load route {route_num}: {resp.text}")

    def play_route(self, route_num: int):
        """Load route and start playback."""
        data = {"route": route_num}
        resp = self._post("/route/play", json_data=data)
        print(f"Play route {route_num}: {resp.text}")

    def pause_route(self):
        """Pause route playback."""
        resp = self._get("/route/pause")
        print(f"Pause: {resp.text}")

    def command_servo(self, servo_id: int, angle: int):
        """Command an arbitrary servo to move to specified angle."""
        data = {"servo": servo_id, "angle": angle}
        resp = self._post("/servo", json_data=data)
        print(f"Servo {servo_id} -> {angle}°: {resp.text}")

    def next_hold(self):
        """Advance to next hold in route."""
        resp = self._get("/route/next")
        print(f"Next: {resp.text}")

    def restart_route(self):
        """Restart route from beginning."""
        resp = self._post("/route/restart")
        print(f"Restart: {resp.text}")

    def set_mapping(self, hfov_deg: float, vfov_deg: float, image_width: int, image_height: int,
                    distance_m: float):
        """Set XY to angular mapping from camera FOV and distance."""
        data = {
            "hfov_deg": hfov_deg,
            "vfov_deg": vfov_deg,
            "image_width": image_width,
            "image_height": image_height,
            "distance_m": distance_m,
        }
        resp = self._post("/route/mapping", json_data=data)
        print(f"Mapping set: {resp.text}")

    # Laser endpoints
    def laser(self, on: Optional[bool] = None):
        """Control laser. If on is None, toggles."""
        data = {"on": on} if on is not None else {}
        resp = self._post("/laser", json_data=data)
        print(f"Laser: {resp.text}")

    # FSM endpoints
    def get_state(self) -> dict:
        """Get current FSM state."""
        resp = self._get("/state")
        data = resp.json()
        return data

    def set_state(self, **kwargs):
        """Set FSM state. kwargs: mode, user_state, climb_state, proceed_mode, interval_ms."""
        resp = self._post("/state", json_data=kwargs)
        if resp.status_code == 409:
            print(f"State transition rejected: {resp.text}", file=sys.stderr)
        else:
            print(f"State: {resp.text}")

    def upload_centroids(self, scan_id: int, centroids: List[Tuple[float, float]]):
        """Upload centroids to ESP fatfs."""
        data = {
            "scan_id": scan_id,
            "centroids": [[x, y] for x, y in centroids],
        }
        resp = self._post("/centroids/upload", json_data=data)
        print(f"Upload centroids (scan {scan_id}, {len(centroids)} points): {resp.text}")

    def get_centroids(self, scan_id: int = 0) -> Optional[List[Tuple[float, float]]]:
        """Fetch centroids from ESP fatfs."""
        resp = self._get(f"/centroids?scan_id={scan_id}")
        data = resp.json()
        centroids = [(c[0], c[1]) for c in data.get("centroids", [])]
        print(f"Got {len(centroids)} centroids from scan {scan_id}")
        return centroids

    def save_scan(self, scan_id: int):
        """Save current frame buffer to fatfs as scanN.jpg."""
        resp = self._post("/scan/save", json_data={"scan_id": scan_id})
        print(f"Save scan {scan_id}: {resp.text}")

    # Utility endpoints
    def test(self, message: str = "hello"):
        """Echo test."""
        resp = self._post("/test", data=message.encode())
        print(f"Test: {resp.text}")

def detect_holds(image_path: str, min_area: int = 150) -> Tuple["np.ndarray", List[dict]]:
    """
    Port of Betaspray-hold-extractor.ijm using OpenCV.

    Pipeline (mirrors the IJM):
      median blur (r=5)  →  background subtraction (Gaussian approx of rolling ball r=100)
      →  CLAHE (blocksize=300)  →  median blur + despeckle
      →  HSV threshold (V ≤ 183, all H/S)
      →  fill holes  →  morphological open
      →  connected components (area ≥ min_area)

    Returns (annotated_bgr_image, holds) where each hold dict has:
      'id', 'centroid' (cx, cy), 'area', 'bbox' (x, y, w, h)
    """
    if not HAS_CV2:
        raise ImportError("pip install opencv-python numpy")

    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Could not load image: {image_path}")

    original = img.copy()
    h, w = img.shape[:2]
    print(f"  Image: {w}×{h}  ({w * h / 1e6:.1f} MP)")

    # 1. Median filter — IJM radius=5, kernel must be odd → ksize=11
    print("  Median filter...")
    working = cv2.medianBlur(img, 11)

    # 2. Subtract background — IJM: rolling ball r=100, "light" background.
    #    "light" mode = invert → subtract background → invert back:
    #      result_inv = clip(inv - Gaussian(inv), 0, 255)
    #      result     = 255 - result_inv
    #    Uniform wood background: inv≈bg_inv → result_inv=0 → result=255 (V>183, excluded).
    #    Dark hold surrounded by background: inv>>bg_inv → large result_inv → result<183 (selected).
    print("  Background subtraction (light)...")
    inv = 255.0 - working.astype(np.float32)
    bg_inv = cv2.GaussianBlur(inv, (0, 0), 50)
    result_inv = np.clip(inv - bg_inv, 0.0, 255.0)
    working = (255.0 - result_inv).astype(np.uint8)

    # 3. CLAHE — IJM blocksize=300; map to OpenCV tileGridSize
    print("  CLAHE...")
    tile_cols = max(1, round(w / 300))
    tile_rows = max(1, round(h / 300))
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(tile_cols, tile_rows))
    lab = cv2.cvtColor(working, cv2.COLOR_BGR2LAB)
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    working = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # 4. Second median + despeckle (3×3 median)
    print("  Median + despeckle...")
    working = cv2.medianBlur(working, 11)
    working = cv2.medianBlur(working, 3)

    # 5. HSV threshold: H 0–255 (all), S 0–255 (all), V 0–183.
    #    ImageJ HSB all channels 0–255; OpenCV H is 0–179, S/V are 0–255.
    print("  HSV threshold (V ≤ 183)...")
    hsv = cv2.cvtColor(working, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 0), (179, 255, 183))

    # 6. Fill holes — flood fill from border, OR back into mask
    print("  Fill holes...")
    flooded = mask.copy()
    flood_fill_mask = np.zeros((h + 2, w + 2), np.uint8)
    cv2.floodFill(flooded, flood_fill_mask, (0, 0), 255)
    mask = cv2.bitwise_or(mask, cv2.bitwise_not(flooded))

    # 7. Morphological open — removes thin noise bridges, mirrors IJM "Open"
    print("  Morphological open...")
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 8. Watershed — IJM line 67: run("Watershed").  Separates touching blobs by
    #    flooding from the local maxima of the distance transform.
    print("  Watershed...")
    dist = cv2.distanceTransform(mask, cv2.DIST_L2, cv2.DIST_MASK_PRECISE)
    # Regional maxima of the distance transform = object centres (UEPs in ImageJ).
    k_ws = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    peaks = np.uint8((dist >= cv2.dilate(dist, k_ws) - 0.01) & (dist >= 1.0)) * 255
    _, markers = cv2.connectedComponents(peaks)
    markers = markers + 1          # 1 = background, 2..N+1 = hold seeds
    markers[mask == 0] = 1         # sure background
    dist_inv = np.uint8(255 - cv2.normalize(dist, None, 0, 255, cv2.NORM_MINMAX))
    cv2.watershed(cv2.cvtColor(dist_inv, cv2.COLOR_GRAY2BGR), markers)
    mask[markers == -1] = 0        # watershed boundaries become background

    # 9. Connected components — mirrors IJM "Analyze Particles" size=150–Infinity
    print("  Connected components...")
    num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)

    holds = []
    for i in range(1, num_labels):  # label 0 is background
        area = int(stats[i, cv2.CC_STAT_AREA])
        if area >= min_area:
            holds.append({
                'id':       len(holds),
                'centroid': (float(centroids[i][0]), float(centroids[i][1])),
                'area':     area,
                'bbox':     (int(stats[i, cv2.CC_STAT_LEFT]),  int(stats[i, cv2.CC_STAT_TOP]),
                             int(stats[i, cv2.CC_STAT_WIDTH]), int(stats[i, cv2.CC_STAT_HEIGHT])),
            })

    print(f"  Detected {len(holds)} holds")

    # Annotate original with bounding boxes, centroids, and IDs
    annotated = original.copy()
    thickness  = max(1, round(w / 1296))
    marker_r   = max(5, round(w / 260))
    font_scale = max(0.5, w / 2592.0)

    for hold in holds:
        cx, cy = int(hold['centroid'][0]), int(hold['centroid'][1])
        x, y, bw, bh = hold['bbox']
        cv2.rectangle(annotated, (x, y), (x + bw, y + bh), (0, 0, 255), thickness)
        cv2.circle(annotated, (cx, cy), marker_r, (0, 0, 255), -1)
        cv2.putText(annotated, str(hold['id']), (cx + marker_r + 2, cy - marker_r),
                    cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 255), thickness)

    return annotated, holds


def select_holds_interactive(image_path: str, holds: List[dict]) -> List[Tuple[float, float]]:
    """
    Show the image with all detected holds overlaid. Click a hold to select it
    (turns green); click again to deselect (back to red). Close the window to
    confirm. Returns centroids in the order they were clicked.
    """
    if not HAS_MATPLOTLIB:
        raise ImportError("pip install matplotlib")
    if not HAS_CV2:
        raise ImportError("pip install opencv-python")

    img_rgb = cv2.cvtColor(cv2.imread(image_path), cv2.COLOR_BGR2RGB)
    h, w = img_rgb.shape[:2]

    marker_r  = max(15, round(w / 130))
    font_size = max(7,  round(w / 370))

    selected_ids: List[int] = []  # ordered

    fig, ax = plt.subplots(figsize=(14, 9))
    ax.imshow(img_rgb)
    ax.set_title(
        f"{len(holds)} holds detected  —  click to select/deselect, close window to confirm",
        fontsize=11,
    )
    ax.axis("off")

    circles: dict = {}
    texts:   dict = {}
    for hold in holds:
        cx, cy = hold['centroid']
        hid = hold['id']
        c = plt.Circle((cx, cy), radius=marker_r, color="red", fill=False, linewidth=2)
        ax.add_patch(c)
        t = ax.text(cx + marker_r + 3, cy, str(hid), color="red",
                    fontsize=font_size, fontweight="bold", va="center")
        circles[hid] = c
        texts[hid]   = t

    def on_click(event):
        if event.inaxes is not ax or event.button != 1:
            return
        mx, my = event.xdata, event.ydata
        if mx is None or my is None:
            return
        # Nearest hold within 3× marker radius
        best, best_dist = None, float("inf")
        for hold in holds:
            cx, cy = hold['centroid']
            d = ((cx - mx) ** 2 + (cy - my) ** 2) ** 0.5
            if d < best_dist:
                best_dist = d
                best = hold['id']
        if best is None or best_dist > marker_r * 3:
            return
        if best in selected_ids:
            selected_ids.remove(best)
            circles[best].set_edgecolor("red")
            texts[best].set_color("red")
        else:
            selected_ids.append(best)
            circles[best].set_edgecolor("lime")
            texts[best].set_color("lime")
        # Update title with running count
        ax.set_title(
            f"{len(holds)} holds detected  —  {len(selected_ids)} selected  —  "
            "click to select/deselect, close window to confirm",
            fontsize=11,
        )
        fig.canvas.draw_idle()

    fig.canvas.mpl_connect("button_press_event", on_click)
    plt.tight_layout()
    plt.show()

    selected = [(holds[i]["centroid"][0], holds[i]["centroid"][1]) for i in selected_ids]
    print(f"Selected {len(selected)} holds (IDs: {selected_ids})")
    return selected


def compute_scales_from_fov(hfov: float, vfov: float, width: int, height: int,
                            distance_m: float, ref_distance_m: float) -> Tuple[float, float, float, float]:
    """Compute servo scales from camera FOV parameters.

    Args:
        hfov: Horizontal field of view (degrees)
        vfov: Vertical field of view (degrees)
        width: Image width (pixels)
        height: Image height (pixels)
        distance_m: Current distance to wall (meters)
        ref_distance_m: Reference/calibration distance (meters)

    Returns:
        (x_scale, x_offset, y_scale, y_offset)
    """
    deg_to_rad = math.pi / 180

    # Real-world dimensions at current distance
    wall_width = 2 * distance_m * math.tan(hfov / 2 * deg_to_rad)
    wall_height = 2 * distance_m * math.tan(vfov / 2 * deg_to_rad)

    # Basic pixel-to-angle mapping (full image → full servo range)
    x_scale = 180.0 / width
    y_scale = 90.0 / height

    # Apply distance correction
    distance_scale = ref_distance_m / distance_m
    x_scale *= distance_scale
    y_scale *= distance_scale

    x_offset = 0.0
    y_offset = 90.0

    return x_scale, x_offset, y_scale, y_offset

def parse_holds(holds_str: str) -> List[Tuple[float, float]]:
    """Parse holds from format: '160,120;80,60;240,180' or JSON array."""
    if holds_str.startswith("["):
        data = json.loads(holds_str)
        return [(h[0], h[1]) for h in data]
    else:
        holds = []
        for pair in holds_str.split(";"):
            x, y = map(float, pair.split(","))
            holds.append((x, y))
        return holds

def load_route_binary(route_num: int, filepath: str = None) -> List[Tuple[float, float]]:
    """Load route from binary file (.bin) and return holds list."""
    if not filepath:
        filepath = f"route{route_num}.bin"

    holds = []
    try:
        with open(filepath, "rb") as f:
            count_bytes = f.read(4)
            if len(count_bytes) < 4:
                print(f"Error: Invalid file format", file=sys.stderr)
                return []

            hold_count = struct.unpack("<I", count_bytes)[0]

            for _ in range(hold_count):
                coord_bytes = f.read(8)
                if len(coord_bytes) < 8:
                    print(f"Error: Incomplete hold data", file=sys.stderr)
                    break
                x, y = struct.unpack("<ff", coord_bytes)
                holds.append((x, y))

        print(f"Loaded {len(holds)} holds from {filepath}")
    except Exception as e:
        print(f"Error loading {filepath}: {e}", file=sys.stderr)
    return holds

def save_route_binary(route_num: int, holds: List[Tuple[float, float]], filepath: str = None) -> None:
    """Save route to binary file (.bin) format."""
    if not filepath:
        filepath = f"route{route_num}.bin"

    try:
        with open(filepath, "wb") as f:
            f.write(struct.pack("<I", len(holds)))
            for x, y in holds:
                f.write(struct.pack("<ff", x, y))

        print(f"Saved {len(holds)} holds to {filepath}")
    except Exception as e:
        print(f"Error saving {filepath}: {e}", file=sys.stderr)

def build_interactive_parser():
    """Builds the argparse parser for interactive shell commands."""
    parser = argparse.ArgumentParser(prog="", description="BetaSpray Commands")
    subparsers = parser.add_subparsers(dest="command", help="Command to execute")

    # Camera commands
    camera_parser = subparsers.add_parser("capture", help="Capture frame from camera")
    camera_parser.add_argument("--get", action="store_true", help="Also retrieve and save frame")
    camera_parser.add_argument("--output", default="capture.jpg", help="Output filename for frame")

    get_parser = subparsers.add_parser("get", help="Retrieve last captured frame")
    get_parser.add_argument("--output", "-o", default="capture.jpg", help="Output filename")

    config_parser = subparsers.add_parser("config", help="Configure camera")
    config_parser.add_argument("--enable", action="store_true", help="Enable camera")
    config_parser.add_argument("--disable", action="store_true", help="Disable camera")
    config_parser.add_argument("--res", choices=["QVGA", "VGA", "SVGA", "5MP"], help="Resolution")
    config_parser.add_argument("--fmt", choices=["JPEG", "RGB565", "GRAYSCALE"], help="Format")
    config_parser.add_argument("--psram", action="store_true", default=None, help="Use PSRAM for frame buffer")
    config_parser.add_argument("--no-psram", action="store_true", default=None, help="Use DRAM for frame buffer")

    # Route commands
    route_create_parser = subparsers.add_parser("create-route", help="Create a route")
    route_create_parser.add_argument("route_num", type=int, help="Route number")
    route_create_parser.add_argument("holds", help="Holds: '160,120;80,60' or JSON array")

    route_save_parser = subparsers.add_parser("save-route", help="Save route to binary file")
    route_save_parser.add_argument("route_num", type=int, help="Route number")
    route_save_parser.add_argument("holds", help="Holds: '160,120;80,60' or JSON array")
    route_save_parser.add_argument("--output", "-o", help="Output filename (default: routeN.bin)")

    route_load_file_parser = subparsers.add_parser("load-route-file", help="Load route from binary file")
    route_load_file_parser.add_argument("route_num", type=int, help="Route number")
    route_load_file_parser.add_argument("--input", "-i", help="Input filename (default: routeN.bin)")

    route_delete_parser = subparsers.add_parser("delete-route", help="Delete a route")
    route_delete_parser.add_argument("route_num", type=int, help="Route number")

    route_load_parser = subparsers.add_parser("load-route", help="Load a route")
    route_load_parser.add_argument("route_num", type=int, help="Route number")

    play_parser = subparsers.add_parser("play", help="Load and start route playback")
    play_parser.add_argument("route_num", type=int, help="Route number to play")
    subparsers.add_parser("pause", help="Pause route playback")
    subparsers.add_parser("next", help="Advance to next hold")
    subparsers.add_parser("restart", help="Restart route")

    mapping_parser = subparsers.add_parser("mapping", help="Set XY to angular mapping from camera FOV")
    mapping_parser.add_argument("distance_m", type=float, help="Distance to wall (meters)")
    mapping_parser.add_argument("--hfov", type=float, default=120.0, help="Horizontal FOV (degrees, default: 120)")
    mapping_parser.add_argument("--vfov", type=float, default=60.0, help="Vertical FOV (degrees, default: 60)")
    mapping_parser.add_argument("--width", type=int, default=320, help="Image width (pixels, default: 320)")
    mapping_parser.add_argument("--height", type=int, default=240, help="Image height (pixels, default: 240)")

    # Servo command
    servo_parser = subparsers.add_parser("servo", help="Command a servo")
    servo_parser.add_argument("servo_id", type=int, help="Servo ID (0-N)")
    servo_parser.add_argument("angle", type=int, help="Target angle (0-180)")

    # Laser
    laser_parser = subparsers.add_parser("laser", help="Control laser")
    laser_parser.add_argument("--on", action="store_true", default=None, help="Turn laser on")
    laser_parser.add_argument("--off", action="store_true", default=None, help="Turn laser off")

    # FSM commands
    mode_parser = subparsers.add_parser("mode", help="Set FSM mode (manual/user)")
    mode_parser.add_argument("mode", choices=["manual", "user"], help="Mode to set")

    subparsers.add_parser("status", help="Show current FSM state")

    scan_parser = subparsers.add_parser("scan", help="Full scan workflow: capture, download, run CV, upload centroids, save scan")
    scan_parser.add_argument("--scan-id", type=int, default=0, help="Scan ID (default: 0)")
    scan_parser.add_argument("--output", "-o", default="capture.jpg", help="Local image filename")

    set_route_parser = subparsers.add_parser("set-route", help="Create route from stored centroids")
    set_route_parser.add_argument("route_num", type=int, help="Route number")
    set_route_parser.add_argument("hold_indices", help="Comma-separated centroid indices, e.g. '0,1,5,10'")
    set_route_parser.add_argument("--scan-id", type=int, default=0, help="Scan ID to fetch centroids from")

    climb_parser = subparsers.add_parser("climb", help="Enter CLIMB_WALL state and load route")
    climb_parser.add_argument("route_num", type=int, help="Route number to climb")
    climb_parser.add_argument("--mode", dest="proceed_mode", choices=["manual", "timed", "auto"], default="manual", help="Proceed mode")
    climb_parser.add_argument("--interval", type=int, default=3000, help="Timed interval in ms (default: 3000)")

    subparsers.add_parser("start", help="Start climbing (RUNNING)")
    subparsers.add_parser("stop", help="Stop climbing (IDLE + reset)")

    # Utility
    test_parser = subparsers.add_parser("test", help="Echo test")
    test_parser.add_argument("message", nargs="?", default="hello", help="Message to echo")

    detect_parser = subparsers.add_parser("detect", help="Detect holds in image, select interactively, optionally send as route")
    detect_parser.add_argument("--input", "-i", default="capture.jpg",
                               help="Input JPEG (default: capture.jpg)")
    detect_parser.add_argument("--min-area", type=int, default=150,
                               help="Minimum blob area in pixels (default: 150)")
    detect_parser.add_argument("--save-annotated", metavar="PATH",
                               help="Save annotated detection image to PATH")

    process_local_parser = subparsers.add_parser("process_local", help="Run local differenceofgaussian on capture.jpg and display blobs")

    help_parser = subparsers.add_parser("help", help="Show help message")

    return parser

def execute_command(client: BetaSprayClient, args: argparse.Namespace, parser: argparse.ArgumentParser):
    """Executes the mapped command. Exceptions are caught to prevent crashing the interactive loop."""
    try:
        if args.command == "help":
            parser.print_help()

        elif args.command == "capture":
            client.capture()
            if args.get:
                client.get_frame(args.output)

        elif args.command == "get":
            client.get_frame(args.output)

        elif args.command == "config":
            enabled = True if args.enable else (False if args.disable else None)
            psram = True if args.psram else (False if args.no_psram else None)
            client.configure_camera(enabled=enabled, resolution=args.res, format=args.fmt, psram=psram)

        elif args.command == "create-route":
            holds = parse_holds(args.holds)
            client.create_route(args.route_num, holds)

        elif args.command == "delete-route":
            client.delete_route(args.route_num)

        elif args.command == "load-route":
            client.load_route(args.route_num)

        elif args.command == "play":
            client.play_route(args.route_num)

        elif args.command == "pause":
            client.pause_route()

        elif args.command == "next":
            client.next_hold()

        elif args.command == "restart":
            client.restart_route()

        elif args.command == "mapping":
            print(f"\nSetting mapping:")
            print(f"  Camera: {args.hfov}° H × {args.vfov}° V, {args.width}×{args.height} px")
            print(f"  Distance: {args.distance_m} m\n")

            # Ask for confirmation
            confirm = input("Send this mapping to device? (y/n): ").strip().lower()
            if confirm == 'y':
                client.set_mapping(
                    hfov_deg=args.hfov,
                    vfov_deg=args.vfov,
                    image_width=args.width,
                    image_height=args.height,
                    distance_m=args.distance_m
                )
            else:
                print("Cancelled.")

        elif args.command == "save-route":
            holds = parse_holds(args.holds)
            output = args.output if hasattr(args, 'output') and args.output else f"route{args.route_num}.bin"
            save_route_binary(args.route_num, holds, output)

        elif args.command == "load-route-file":
            input_file = args.input if hasattr(args, 'input') and args.input else f"route{args.route_num}.bin"
            load_route_binary(args.route_num, input_file)

        elif args.command == "servo":
            client.command_servo(args.servo_id, args.angle)

        elif args.command == "laser":
            on = True if args.on else (False if args.off else None)
            client.laser(on)

        elif args.command == "test":
            client.test(args.message)
        elif args.command == "mode":
            client.set_state(mode=args.mode.upper())

        elif args.command == "status":
            state = client.get_state()
            mode = state.get("mode", "?")
            line = f"Mode: {mode}"
            if mode == "USER":
                us = state.get("user_state", "?")
                line += f"  |  User state: {us}"
                if us == "CLIMB_WALL":
                    line += f"  |  Climb: {state.get('climb_state', '?')}"
                    line += f"  |  Proceed: {state.get('proceed_mode', '?')}"
            line += f"  |  Scan: {state.get('active_scan', -1)}  Route: {state.get('active_route', -1)}"
            print(line)

        elif args.command == "scan":
            sid = args.scan_id
            out = args.output
            print(f"--- Scan workflow (scan_id={sid}) ---")
            # 1. Capture
            client.capture()
            # 2. Download
            client.get_frame(out)
            # 3. Run local CV
            centroids = []
            try:
                result = subprocess.run(
                    ["./differenceofgaussian", out],
                    capture_output=True, text=True, check=True,
                )
                # Parse centroids from stdout: expect lines like "x,y"
                for line in result.stdout.strip().splitlines():
                    parts = line.strip().split(",")
                    if len(parts) >= 2:
                        try:
                            centroids.append((float(parts[0]), float(parts[1])))
                        except ValueError:
                            pass
                print(f"CV found {len(centroids)} centroids")
            except (subprocess.CalledProcessError, FileNotFoundError) as e:
                print(f"CV failed ({e}), enter centroids manually or re-run", file=sys.stderr)
                return
            # 4. Upload centroids
            if centroids:
                client.upload_centroids(sid, centroids)
            # 5. Save scan image
            client.save_scan(sid)
            print(f"--- Scan complete (scan_id={sid}, {len(centroids)} centroids) ---")

        elif args.command == "set-route":
            # Fetch centroids from ESP
            centroids = client.get_centroids(args.scan_id)
            if not centroids:
                print("No centroids available", file=sys.stderr)
                return
            # Parse indices
            indices = [int(i) for i in args.hold_indices.split(",")]
            selected = []
            for i in indices:
                if 0 <= i < len(centroids):
                    selected.append(centroids[i])
                else:
                    print(f"Warning: index {i} out of range (0-{len(centroids)-1})", file=sys.stderr)
            if not selected:
                print("No valid holds selected", file=sys.stderr)
                return
            client.create_route(args.route_num, selected)
            print(f"Route {args.route_num} created with {len(selected)} holds")

        elif args.command == "climb":
            # Transition: USER -> CLIMB_WALL, set proceed mode, load route
            client.set_state(user_state="CLIMB_WALL")
            client.set_state(
                proceed_mode=args.proceed_mode.upper(),
                interval_ms=args.interval,
            )
            client.load_route(args.route_num)
            print(f"Ready to climb route {args.route_num} ({args.proceed_mode} mode)")
            print("Use 'start' to begin, 'next' to advance, 'stop' to halt")

        elif args.command == "start":
            client.set_state(climb_state="RUNNING")

        elif args.command == "stop":
            client.set_state(climb_state="IDLE")
        elif args.command == "detect":
            if not HAS_CV2 or not HAS_MATPLOTLIB:
                print("Error: pip install opencv-python numpy matplotlib", file=sys.stderr)
                return
            input_file = args.input
            if not Path(input_file).exists():
                print(f"Error: '{input_file}' not found. Run 'capture --get' first.", file=sys.stderr)
                return
            print(f"Running hold detection on {input_file}...")
            annotated, holds = detect_holds(input_file, min_area=args.min_area)
            if args.save_annotated:
                cv2.imwrite(args.save_annotated, annotated)
                print(f"Annotated image saved to {args.save_annotated}")
            if not holds:
                print("No holds detected. Try lowering --min-area or check image quality.")
                return
            selected = select_holds_interactive(input_file, holds)
            if not selected:
                print("No holds selected.")
                return
            client.pending_holds = selected
            print(f"\n{len(selected)} hold(s) stored in session.")
            route_q = input("Send to device as route? Enter route number (or Enter to skip): ").strip()
            if route_q.isdigit():
                client.create_route(int(route_q), selected)

        elif args.command == "process_local":
            try:
                # Run the differenceofgaussian program
                result = subprocess.run(
                    ["./differenceofgaussian", "capture.jpg"],
                    capture_output=True,
                    text=True,
                    check=True
                )
                print(result.stdout)
                subprocess.run(["feh", "-d", "output_blobs.ppm"])

            except subprocess.CalledProcessError as e:
                print(f"Error executing differenceofgaussian: {e}\n{e.stderr}", file=sys.stderr)
            except FileNotFoundError as e:
                print(f"Executable not found: {e}. Make sure ./differenceofgaussian and feh are installed/compiled.", file=sys.stderr)
    except Exception as e:
        print(f"Command Error: {e}", file=sys.stderr)

def get_prompt(client: BetaSprayClient) -> str:
    """Build state-aware prompt string."""
    try:
        state = client.get_state()
        mode = state.get("mode", "?")
        if mode == "USER":
            us = state.get("user_state", "?")
            if us == "CLIMB_WALL":
                cs = state.get("climb_state", "?")
                return f"betaspray[USER/{us}:{cs}]> "
            return f"betaspray[USER/{us}]> "
        return f"betaspray[{mode}]> "
    except Exception:
        return "betaspray> "


class AutoModeThread:
    """Background thread for AUTO proceed mode: captures frames, runs CV, sends /route/next."""

    def __init__(self, client: BetaSprayClient, interval: float = 0.2):
        self.client = client
        self.interval = interval
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print("AUTO mode thread started")

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        print("AUTO mode thread stopped")

    def _run(self):
        while not self._stop.is_set():
            try:
                # Check if still in RUNNING state
                state = self.client.get_state()
                if state.get("climb_state") != "RUNNING":
                    print("\n[AUTO] Climb no longer RUNNING, stopping auto thread")
                    break

                # Capture + download
                self.client.session.post(f"{self.client.base_url}/capture", timeout=5)
                resp = self.client.session.get(f"{self.client.base_url}/get", timeout=5)
                if resp.headers.get("content-type") == "image/jpeg":
                    with open("auto_frame.jpg", "wb") as f:
                        f.write(resp.content)

                    # Run CV
                    result = subprocess.run(
                        ["./differenceofgaussian", "auto_frame.jpg"],
                        capture_output=True, text=True, timeout=5,
                    )
                    # If CV detects climber near hold, advance
                    # For now: always advance (placeholder for real CV logic)
                    if result.returncode == 0:
                        self.client.session.get(f"{self.client.base_url}/route/next", timeout=5)

            except Exception as e:
                print(f"\n[AUTO] Error: {e}", file=sys.stderr)

            self._stop.wait(self.interval)


def main():
    base_parser = argparse.ArgumentParser(description="BetaSpray HTTP Client Initialization")
    base_parser.add_argument("--host", default=DEFAULT_HOST, help=f"Device IP (default: {DEFAULT_HOST})")
    base_parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Device port (default: {DEFAULT_PORT})")

    base_args, remaining_args = base_parser.parse_known_args()

    client = BetaSprayClient(base_args.host, base_args.port)
    cmd_parser = build_interactive_parser()
    auto_thread = AutoModeThread(client)

    # Infer that CLI intent can exist too...
    # if an argument was provided directly via command line, just do that then exit
    if remaining_args:
        try:
            args = cmd_parser.parse_args(remaining_args)
            if args.command:
                execute_command(client, args, cmd_parser)
                return
        except SystemExit:
            return

    print(f"Connected to BetaSpray device at http://{base_args.host}:{base_args.port}")
    print("Type 'help' to see available commands. Press Ctrl-D or type 'exit' to quit.\n")

    while True:
        try:
            prompt = get_prompt(client)
            line = input(prompt).strip()
            if not line:
                continue

            if line.lower() in ["exit", "quit"]:
                auto_thread.stop()
                print("Exiting...")
                break

            # recreate shell-style argument fmt
            args_list = shlex.split(line)
            args = cmd_parser.parse_args(args_list)

            if not args.command:
                cmd_parser.print_help()
                continue

            execute_command(client, args, cmd_parser)

            # Auto-start/stop AUTO thread based on state transitions
            if args.command == "start":
                try:
                    state = client.get_state()
                    if state.get("proceed_mode") == "AUTO" and state.get("climb_state") == "RUNNING":
                        auto_thread.start()
                except Exception:
                    pass
            elif args.command in ("stop", "pause", "mode"):
                auto_thread.stop()

        except KeyboardInterrupt:
            print("\n(Cancelled) Press Ctrl-D or type 'exit' to quit.")
            continue
        except EOFError: # catch user EOF (C-d)
            auto_thread.stop()
            print("\nExiting...")
            break
        except ValueError as e: # shlex
            print(f"input parsing error: {e}")
        except SystemExit:
            pass

if __name__ == "__main__":
    main()
