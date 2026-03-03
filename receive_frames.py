"""
BetaSpray camera UART receiver.

Listens on the serial port for framed JPEG data and saves each frame
to the frames/ directory.

Usage:
    pip install pyserial
    python receive_frames.py          # auto-detects port
    python receive_frames.py COM3     # or specify port explicitly
    python receive_frames.py /dev/ttyUSB0

Frame format (from firmware):
    [0xAA 0xBB 0xCC 0xDD]  4-byte start marker
    [uint32 LE]             JPEG byte count
    [... JPEG data ...]
    [0xDD 0xCC 0xBB 0xAA]  4-byte end marker
"""

import os
import struct
import sys
import serial
import serial.tools.list_ports

BAUD        = 921600
FRAME_START = bytes([0xAA, 0xBB, 0xCC, 0xDD])
FRAME_END   = bytes([0xDD, 0xCC, 0xBB, 0xAA])
MAX_FRAME   = 500_000  # sanity cap (bytes)


def pick_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        sys.exit(1)
    if len(ports) == 1:
        return ports[0].device
    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} — {p.description}")
    idx = int(input("Select port number: "))
    return ports[idx].device


def find_marker(ser, marker):
    """Read byte-by-byte until marker is found in the stream."""
    buf = bytearray()
    while True:
        b = ser.read(1)
        if not b:
            return False  # timeout
        buf.append(b[0])
        if len(buf) >= len(marker) and bytes(buf[-len(marker):]) == marker:
            return True


def read_frame(ser):
    if not find_marker(ser, FRAME_START):
        return None

    raw_len = ser.read(4)
    if len(raw_len) < 4:
        return None
    length = struct.unpack("<I", raw_len)[0]

    if length == 0 or length > MAX_FRAME:
        print(f"  [!] Bad frame length {length}, skipping")
        return None

    data = ser.read(length)
    if len(data) < length:
        return None

    end = ser.read(4)
    if end != FRAME_END:
        print(f"  [!] Bad end marker, skipping frame")
        return None

    return bytes(data)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else pick_port()
    os.makedirs("frames", exist_ok=True)

    print(f"Opening {port} at {BAUD} baud ...")
    with serial.Serial(port, BAUD, timeout=5) as ser:
        print("Waiting for frames  (Ctrl-C to stop)\n")
        frame_num = 0
        while True:
            frame = read_frame(ser)
            if frame:
                path = f"frames/frame_{frame_num:04d}.jpg"
                with open(path, "wb") as f:
                    f.write(frame)
                print(f"  frame {frame_num:04d}  {len(frame):>6} bytes  -> {path}")
                frame_num += 1


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nDone.")
