#!/usr/bin/env python3
"""
BetaSpray HTTP Client
Interfaces with the ESP32-S3 over HTTP for camera, route, and servo control.
"""

import requests
import json
import argparse
import sys
import struct
from typing import List, Tuple, Optional
from pathlib import Path

DEFAULT_HOST = "192.168.4.1"
DEFAULT_PORT = 80


class BetaSprayClient:
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT):
        self.base_url = f"http://{host}:{port}"
        self.session = requests.Session()
        self.session.timeout = 10

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
                        format: Optional[str] = None):
        """Configure camera settings."""
        config = {}
        if enabled is not None:
            config["enabled"] = enabled
        if resolution:
            config["resolution"] = resolution
        if format:
            config["format"] = format

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
        # Binary format: [uint32: hold_count][float32: x0][float32: y0]...
        buf = struct.pack("<I", len(holds))  # hold count as little-endian uint32
        for x, y in holds:
            buf += struct.pack("<ff", x, y)  # x, y as little-endian float32

        # Still send as JSON to HTTP endpoint, which converts to binary
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

    # Utility endpoints
    def test(self, message: str = "hello"):
        """Echo test."""
        resp = self._post("/test", data=message.encode())
        print(f"Test: {resp.text}")


def parse_holds(holds_str: str) -> List[Tuple[float, float]]:
    """Parse holds from format: '160,120;80,60;240,180' or JSON array."""
    if holds_str.startswith("["):
        # JSON format
        data = json.loads(holds_str)
        return [(h[0], h[1]) for h in data]
    else:
        # Semicolon-separated format
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
    with open(filepath, "rb") as f:
        # Read hold count
        count_bytes = f.read(4)
        if len(count_bytes) < 4:
            print(f"Error: Invalid file format", file=sys.stderr)
            return []

        hold_count = struct.unpack("<I", count_bytes)[0]

        # Read holds
        for _ in range(hold_count):
            coord_bytes = f.read(8)
            if len(coord_bytes) < 8:
                print(f"Error: Incomplete hold data", file=sys.stderr)
                break
            x, y = struct.unpack("<ff", coord_bytes)
            holds.append((x, y))

    print(f"Loaded {len(holds)} holds from {filepath}")
    return holds


def save_route_binary(route_num: int, holds: List[Tuple[float, float]],
                     filepath: str = None) -> None:
    """Save route to binary file (.bin) format."""
    if not filepath:
        filepath = f"route{route_num}.bin"

    with open(filepath, "wb") as f:
        # Write hold count
        f.write(struct.pack("<I", len(holds)))
        # Write holds
        for x, y in holds:
            f.write(struct.pack("<ff", x, y))

    print(f"Saved {len(holds)} holds to {filepath}")


def main():
    parser = argparse.ArgumentParser(description="BetaSpray HTTP Client")
    parser.add_argument("--host", default=DEFAULT_HOST, help=f"Device IP (default: {DEFAULT_HOST})")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Device port (default: {DEFAULT_PORT})")

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
    config_parser.add_argument("--res", choices=["QVGA", "VGA", "SVGA"], help="Resolution")
    config_parser.add_argument("--fmt", choices=["JPEG", "RGB565", "GRAYSCALE"], help="Format")

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

    # Servo command
    servo_parser = subparsers.add_parser("servo", help="Command a servo")
    servo_parser.add_argument("servo_id", type=int, help="Servo ID (0-N)")
    servo_parser.add_argument("angle", type=int, help="Target angle (0-180)")

    # Utility
    test_parser = subparsers.add_parser("test", help="Echo test")
    test_parser.add_argument("message", nargs="?", default="hello", help="Message to echo")

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        return

    client = BetaSprayClient(args.host, args.port)

    try:
        if args.command == "capture":
            client.capture()
            if args.get:
                client.get_frame(args.output)

        elif args.command == "get":
            client.get_frame(args.output)

        elif args.command == "config":
            enabled = True if args.enable else (False if args.disable else None)
            client.configure_camera(enabled=enabled, resolution=args.res, format=args.fmt)

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

        elif args.command == "save-route":
            holds = parse_holds(args.holds)
            output = args.output if hasattr(args, 'output') and args.output else f"route{args.route_num}.bin"
            save_route_binary(args.route_num, holds, output)

        elif args.command == "load-route-file":
            input_file = args.input if hasattr(args, 'input') and args.input else f"route{args.route_num}.bin"
            load_route_binary(args.route_num, input_file)

        elif args.command == "servo":
            client.command_servo(args.servo_id, args.angle)

        elif args.command == "test":
            client.test(args.message)

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
