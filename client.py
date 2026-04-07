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
from typing import List, Tuple, Optional
from pathlib import Path

try:
    import readline
except ImportError:
    pass

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

    # Utility endpoints
    def test(self, message: str = "hello"):
        """Echo test."""
        resp = self._post("/test", data=message.encode())
        print(f"Test: {resp.text}")


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
    config_parser.add_argument("--res", choices=["QVGA", "VGA", "SVGA"], help="Resolution")
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

    # Utility
    test_parser = subparsers.add_parser("test", help="Echo test")
    test_parser.add_argument("message", nargs="?", default="hello", help="Message to echo")

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

        elif args.command == "test":
            client.test(args.message)
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


def main():
    base_parser = argparse.ArgumentParser(description="BetaSpray HTTP Client Initialization")
    base_parser.add_argument("--host", default=DEFAULT_HOST, help=f"Device IP (default: {DEFAULT_HOST})")
    base_parser.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"Device port (default: {DEFAULT_PORT})")
    
    base_args, remaining_args = base_parser.parse_known_args()

    client = BetaSprayClient(base_args.host, base_args.port)
    cmd_parser = build_interactive_parser()

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
            line = input("betaspray> ").strip()
            if not line:
                continue
            
            if line.lower() in ["exit", "quit"]:
                print("Exiting...")
                break
 
            # recreate shell-style argument fmt
            args_list = shlex.split(line)
            args = cmd_parser.parse_args(args_list)

            if not args.command:
                cmd_parser.print_help()
                continue
            
            execute_command(client, args, cmd_parser)

        except KeyboardInterrupt:
            print("\n(Cancelled) Press Ctrl-D or type 'exit' to quit.")
            continue
        except EOFError: # catch user EOF (C-d)
            print("\nExiting...")
            break
        except ValueError as e: # shlex
            print(f"input parsing error: {e}")
        except SystemExit:
            pass

if __name__ == "__main__":
    main()