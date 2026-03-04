#!/usr/bin/env python3
"""
Capture JPEG frames from BetaSpray ESP32 over UART.

Frame format: START(4) + LENGTH(4, LE) + JPEG DATA + END(4)
  START: 0xAA, 0xBB, 0xCC, 0xDD
  LENGTH: uint32_t little-endian frame size
  JPEG DATA: raw JPEG bytes
  END: 0xDD, 0xCC, 0xBB, 0xAA
"""

import serial
import serial.tools.list_ports
import struct
import sys
import os
from pathlib import Path
from datetime import datetime

FRAME_START = b'\xaa\xbb\xcc\xdd'
FRAME_END = b'\xdd\xcc\xbb\xaa'

def capture_frames(port='/dev/ttyUSB0', baudrate=115200, output_dir='frames', max_frames=None):
    """
    Capture frames from ESP32 over UART.

    Args:
        port: Serial port (e.g., /dev/ttyUSB0, COM3)
        baudrate: UART baudrate (115200)
        output_dir: Directory to save frames
        max_frames: Maximum frames to capture (None = infinite)
    """
    # Create output directory
    Path(output_dir).mkdir(exist_ok=True)

    print(f"Opening {port} at {baudrate} baud...")
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        print("Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device} - {p.description}")
        return

    frame_num = 0
    bytes_synced = 0

    print(f"Capturing frames to '{output_dir}/'... (Press Ctrl+C to stop)")
    print()

    try:
        while True:
            # Wait for START marker
            byte = ser.read(1)
            if not byte:
                continue

            # Look for START marker
            if byte == FRAME_START[0:1]:
                rest = ser.read(3)
                if byte + rest == FRAME_START:
                    # Read length
                    length_bytes = ser.read(4)
                    if len(length_bytes) < 4:
                        print("ERROR: Could not read frame length")
                        continue

                    try:
                        length = struct.unpack('<I', length_bytes)[0]
                    except struct.error:
                        print("ERROR: Invalid frame length")
                        continue

                    # Sanity check frame size (QVGA JPEG ~5-40 KB)
                    if length > 100 * 1024:
                        print(f"WARNING: Frame size {length} bytes seems large, skipping")
                        continue
                    if length == 0:
                        print("WARNING: Frame size 0, skipping")
                        continue

                    # Read JPEG data
                    jpeg_data = ser.read(length)
                    if len(jpeg_data) < length:
                        print(f"ERROR: Expected {length} bytes, got {len(jpeg_data)}")
                        continue

                    # Read END marker
                    end = ser.read(4)
                    if end != FRAME_END:
                        print(f"WARNING: Invalid END marker: {end.hex()}")
                        continue

                    # Save frame
                    frame_num += 1
                    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                    filename = f"{output_dir}/frame_{frame_num:04d}_{timestamp}.jpg"

                    with open(filename, 'wb') as f:
                        f.write(jpeg_data)

                    print(f"✓ Frame {frame_num}: {len(jpeg_data):6d} bytes -> {filename}")

                    if max_frames and frame_num >= max_frames:
                        print(f"\nCaptured {frame_num} frames. Exiting.")
                        break

    except KeyboardInterrupt:
        print(f"\n\nCaptured {frame_num} frames. Exiting.")
    finally:
        ser.close()

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Capture JPEG frames from BetaSpray ESP32 over UART')
    parser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200, help='Baudrate (default: 115200)')
    parser.add_argument('-o', '--output', default='frames', help='Output directory (default: frames)')
    parser.add_argument('-n', '--max-frames', type=int, default=None, help='Max frames to capture (default: infinite)')

    args = parser.parse_args()

    capture_frames(port=args.port, baudrate=args.baudrate, output_dir=args.output, max_frames=args.max_frames)
