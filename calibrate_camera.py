#!/usr/bin/env python3
"""
Camera calibration for BetaSpray ESP32 over UART.

Triggered mode (default): sends 0xCA trigger byte over UART, ESP32 responds
with one JPEG frame. Requires main.c to be running in calibration mode
(see uart_calib_task).

Passive mode (--passive): reads from the existing auto-streaming loop.
Use this if you haven't added the calibration task to main.c yet.

Offline mode (--from-dir): run calibration on already-captured frames,
skipping UART entirely.

Frame format (same as capture_frames.py):
    START(4) + LENGTH(4, LE uint32) + JPEG DATA + END(4)
    START: 0xAA 0xBB 0xCC 0xDD
    END:   0xDD 0xCC 0xBB 0xAA

Calibration trigger byte: 0xCA

Usage:
    python calibrate_camera.py -p /dev/ttyUSB0 -n 20
    python calibrate_camera.py -p /dev/ttyUSB0 --passive -n 20
    python calibrate_camera.py --from-dir calibration_frames/
"""

# NOTE(Ingi) for (MAX): please make the default mode to be from images, not UART
# We should be using HTTP for image transfers now, and not use the UART workaround unless need is dire.

import sys
import os
import struct
import time
import glob
import argparse
import numpy as np
from pathlib import Path

try:
    import cv2
except ImportError:
    print("ERROR: OpenCV not found. Install with: pip install opencv-python")
    sys.exit(1)

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not found. Install with: pip install pyserial")
    sys.exit(1)

# ── UART framing ────────────────────────────────────────────────────────────
FRAME_START    = b'\xaa\xbb\xcc\xdd'
FRAME_END      = b'\xdd\xcc\xbb\xaa'
CALIB_TRIGGER  = b'\xca'

# ── Checkerboard config ──────────────────────────────────────────────────────
# Inner corners (cols-1, rows-1) of your printed checkerboard.
# Measured from actual captured frames: 9×7 square board → 8×6 inner corners.
CHESSBOARD_SIZE = (8, 6)

# ── Thresholds ────────────────────────────────────────────────────────────────
MIN_FRAMES = 10   # minimum accepted frames before calibration is offered
GOOD_FRAMES = 15  # recommended minimum for a reliable result


def list_ports():
    for p in serial.tools.list_ports.comports():
        print(f"  {p.device} - {p.description}")


def open_serial(port, baudrate):
    try:
        return serial.Serial(port, baudrate, timeout=5)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        print("Available ports:")
        list_ports()
        sys.exit(1)


def recv_frame(ser, overall_timeout=15):
    """Block until a complete framed JPEG is received. Returns bytes or None."""
    ser.timeout = 2  # short per-read timeout so we can check the deadline
    deadline = time.monotonic() + overall_timeout
    while time.monotonic() < deadline:
        byte = ser.read(1)
        if not byte:
            continue  # per-read timeout; keep trying until overall deadline
        if byte != FRAME_START[0:1]:
            continue
        rest = ser.read(3)
        if byte + rest != FRAME_START:
            continue
        length_bytes = ser.read(4)
        if len(length_bytes) < 4:
            continue
        length = struct.unpack('<I', length_bytes)[0]
        if length == 0 or length > 200 * 1024:
            continue
        jpeg = ser.read(length)
        if len(jpeg) < length:
            continue
        end = ser.read(4)
        if end != FRAME_END:
            continue
        return jpeg
    return None  # overall timeout


def trigger_and_recv(ser):
    """Send calibration trigger byte and wait for one frame."""
    ser.reset_input_buffer()
    ser.write(CALIB_TRIGGER)
    ser.flush()
    return recv_frame(ser)


def decode_jpeg(data):
    """Decode JPEG bytes to a BGR numpy array. Returns None on failure."""
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return img


def find_corners(img, board_size):
    """
    Detect checkerboard corners in img.
    Returns (corners, gray) if found, (None, gray) otherwise.

    Detection order (each attempt also tried on a CLAHE-enhanced copy):
      1. findChessboardCornersSB (more robust, OpenCV 4.x+)
      2. findChessboardCorners with adaptive thresh + normalize + filter quads
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(4, 4))
    gray_clahe = clahe.apply(gray)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    classic_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
                     cv2.CALIB_CB_NORMALIZE_IMAGE |
                     cv2.CALIB_CB_FILTER_QUADS)

    for g in (gray, gray_clahe):
        # Attempt A: newer, more robust detector
        if hasattr(cv2, 'findChessboardCornersSB'):
            found, corners = cv2.findChessboardCornersSB(g, board_size)
            if found:
                return corners, g

        # Attempt B: classic detector
        found, corners = cv2.findChessboardCorners(g, board_size, classic_flags)
        if found:
            corners = cv2.cornerSubPix(g, corners, (11, 11), (-1, -1), criteria)
            return corners, g

    return None, gray


def run_calibration(obj_points, img_points, img_shape):
    """Run cv2.calibrateCamera and return (K, dist, rms)."""
    print("\nRunning calibration...")
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_shape[::-1], None, None
    )
    return K, dist.flatten(), rms


def print_results(K, dist, rms, calib_res):
    w, h = calib_res
    print(f"\n{'='*60}")
    print(f"  RMS reprojection error: {rms:.4f} px  (< 1.0 is good)")
    print(f"  Calibration resolution: {w}x{h}")
    print(f"\n  Intrinsic matrix K:")
    print(f"    fx={K[0,0]:.4f}  fy={K[1,1]:.4f}")
    print(f"    cx={K[0,2]:.4f}  cy={K[1,2]:.4f}")
    print(f"\n  Distortion coefficients [k1,k2,p1,p2,k3]:")
    print(f"    {dist}")
    print(f"{'='*60}\n")

    # C header snippet
    print("/* --- paste into conf.h --- */")
    print(f"#define CALIB_WIDTH   {w}")
    print(f"#define CALIB_HEIGHT  {h}")
    print(f"#define CALIB_FX      {K[0,0]:.6f}f")
    print(f"#define CALIB_FY      {K[1,1]:.6f}f")
    print(f"#define CALIB_CX      {K[0,2]:.6f}f")
    print(f"#define CALIB_CY      {K[1,2]:.6f}f")
    print(f"#define CALIB_K1      {dist[0]:.6f}f")
    print(f"#define CALIB_K2      {dist[1]:.6f}f")
    print(f"#define CALIB_P1      {dist[2]:.6f}f")
    print(f"#define CALIB_P2      {dist[3]:.6f}f")
    print(f"#define CALIB_K3      {dist[4]:.6f}f")
    print("/* ---- scaling K for a different resolution ----")
    print(" * scale = new_width / CALIB_WIDTH")
    print(" * fx_new = CALIB_FX * scale   fy_new = CALIB_FY * scale")
    print(" * cx_new = CALIB_CX * scale   cy_new = CALIB_CY * scale")
    print(" * dist coefficients stay the same */")


def calibrate_from_images(img_paths, board_size):
    """Load images from disk and run calibration."""
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)

    obj_points, img_points = [], []
    img_shape = None
    accepted = 0

    for path in sorted(img_paths):
        img = cv2.imread(path)
        if img is None:
            print(f"  SKIP (unreadable): {path}")
            continue
        corners, gray = find_corners(img, board_size)
        if corners is not None:
            obj_points.append(objp)
            img_points.append(corners)
            img_shape = gray.shape
            accepted += 1
            print(f"  OK  [{accepted:3d}] {os.path.basename(path)}")
        else:
            print(f"  --  (no board)  {os.path.basename(path)}")

    if accepted < MIN_FRAMES:
        print(f"\nERROR: only {accepted} usable frames (need {MIN_FRAMES}). Collect more.")
        sys.exit(1)

    return obj_points, img_points, img_shape, accepted


def interactive_capture(ser, triggered, board_size, output_dir, target_frames):
    """
    Interactively capture calibration frames from UART.
    triggered=True: send 0xCA and wait for one frame per keypress.
    triggered=False (passive): wait for next auto-streamed frame per keypress.
    """
    Path(output_dir).mkdir(exist_ok=True)

    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)

    obj_points, img_points = [], []
    img_shape = None
    frame_num = 0
    calib_res = None

    mode_str = "triggered (0xCA)" if triggered else "passive (auto-stream)"
    print(f"\nCapture mode : {mode_str}")
    print(f"Board size   : {board_size[0]}x{board_size[1]} inner corners")
    print(f"Target frames: {target_frames}")
    print(f"Output dir   : {output_dir}/")
    print()
    print("Hold a checkerboard in view, then press ENTER to capture.")
    print("Press 'q' + ENTER to stop and run calibration early.")
    print()

    while len(obj_points) < target_frames:
        prompt = f"[{len(obj_points)}/{target_frames} accepted] Press ENTER to capture (q=quit): "
        try:
            key = input(prompt).strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if key == 'q':
            break

        print("  Capturing...", end=' ', flush=True)
        if triggered:
            jpeg = trigger_and_recv(ser)
        else:
            jpeg = recv_frame(ser)

        if jpeg is None:
            print("TIMEOUT or framing error, try again.")
            continue

        img = decode_jpeg(jpeg)
        if img is None:
            print("JPEG decode failed, try again.")
            continue

        if calib_res is None:
            calib_res = (img.shape[1], img.shape[0])  # (width, height)

        corners, gray = find_corners(img, board_size)
        frame_num += 1
        filename = f"{output_dir}/calib_{frame_num:04d}.jpg"

        if corners is not None:
            obj_points.append(objp)
            img_points.append(corners)
            img_shape = gray.shape

            # Draw corners and save annotated copy
            annotated = img.copy()
            cv2.drawChessboardCorners(annotated, board_size, corners, True)
            cv2.imwrite(filename, annotated)
            print(f"OK  -> {filename}  (accepted {len(obj_points)})")
        else:
            cv2.imwrite(filename, img)
            print(f"NO BOARD DETECTED -> {filename}  (discarded)")

    print(f"\nCapture done: {len(obj_points)} accepted frames.")

    if len(obj_points) < MIN_FRAMES:
        print(f"ERROR: need at least {MIN_FRAMES} frames, got {len(obj_points)}. Aborting.")
        sys.exit(1)

    if len(obj_points) < GOOD_FRAMES:
        print(f"WARNING: fewer than {GOOD_FRAMES} frames — result may be noisy.")

    return obj_points, img_points, img_shape, calib_res


def main():
    parser = argparse.ArgumentParser(
        description='Calibrate OV5640 camera on BetaSpray ESP32',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('-p', '--port', default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                        help='Baudrate (default: 115200)')
    parser.add_argument('-n', '--frames', type=int, default=GOOD_FRAMES,
                        help=f'Target accepted frames (default: {GOOD_FRAMES})')
    parser.add_argument('-o', '--output', default='calibration_frames',
                        help='Directory to save captured frames (default: calibration_frames)')
    parser.add_argument('--passive', action='store_true',
                        help='Read from auto-stream instead of sending trigger byte')
    parser.add_argument('--from-dir', metavar='DIR',
                        help='Skip UART, calibrate from JPEG images in DIR')
    parser.add_argument('--board', default='8x6',
                        help='Checkerboard inner corners WxH (default: 8x6)')

    args = parser.parse_args()

    # Parse board size
    try:
        bw, bh = [int(x) for x in args.board.lower().split('x')]
        board_size = (bw, bh)
    except ValueError:
        print(f"ERROR: invalid --board '{args.board}', expected format like 9x6")
        sys.exit(1)

    # ── Offline mode ─────────────────────────────────────────────────────────
    if args.from_dir:
        paths = glob.glob(os.path.join(args.from_dir, '*.jpg')) + \
                glob.glob(os.path.join(args.from_dir, '*.jpeg'))
        if not paths:
            print(f"ERROR: no JPEG files found in '{args.from_dir}'")
            sys.exit(1)
        print(f"Loading {len(paths)} images from '{args.from_dir}'...")
        obj_points, img_points, img_shape, accepted = calibrate_from_images(paths, board_size)
        # calib_res comes from img_shape (last accepted frame) — consistent with calibrateCamera input
        calib_res = (img_shape[1], img_shape[0])  # (width, height)
        K, dist, rms = run_calibration(obj_points, img_points, img_shape)
        print_results(K, dist, rms, calib_res)
        return

    # ── UART mode ─────────────────────────────────────────────────────────────
    if not args.passive:
        print(f"Triggered mode: will send 0xCA to {args.port} to request each frame.")
        print("Make sure main.c is running uart_calib_task (not the normal streaming loop).")
    else:
        print(f"Passive mode: reading auto-streamed frames from {args.port}.")

    ser = open_serial(args.port, args.baudrate)
    print(f"Opened {args.port} at {args.baudrate} baud.")

    try:
        obj_points, img_points, img_shape, calib_res = interactive_capture(
            ser,
            triggered=not args.passive,
            board_size=board_size,
            output_dir=args.output,
            target_frames=args.frames
        )
    finally:
        ser.close()

    K, dist, rms = run_calibration(obj_points, img_points, img_shape)
    print_results(K, dist, rms, calib_res)


if __name__ == '__main__':
    main()
