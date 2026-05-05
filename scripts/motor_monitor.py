#!/usr/bin/env python3
"""
motor_monitor.py — Serial → UDP bridge for NUControl live telemetry.

Reads JSON lines from the Teensy's data port (SerialUSB1) and forwards them
to PlotJuggler via UDP. Launches PlotJuggler automatically if available.

Install plotjuggler here: https://appimage.github.io/PlotJuggler/

Usage:
    python3 scripts/motor_monitor.py /dev/ttyACM1
    python3 scripts/motor_monitor.py /dev/ttyACM1 --udp-port 9870
    python3 scripts/motor_monitor.py /dev/ttyACM1 --no-plotjuggler

PlotJuggler first-time setup:
    1. Streaming → UDP Server → port 9870 → Start
    2. Drag signal names from the left panel onto plot panes
    3. File → Save Layout → scripts/plotjuggler_layout.xml
    Subsequent runs load this layout automatically.

Dependencies:
    pip install pyserial
"""

import argparse
import socket
import subprocess
import sys
import time
from pathlib import Path
from typing import Optional

try:
    import serial
    import serial.serialutil
except ImportError:
    sys.exit("pyserial not found.  Install it with:  pip install pyserial")


SCRIPT_DIR = Path(__file__).parent
DEFAULT_LAYOUT = SCRIPT_DIR / "plotjuggler_layout.xml"
RECONNECT_DELAY_S = 2.0
STATS_INTERVAL = 1000  # print throughput every N packets


def launch_plotjuggler(layout_path: Path) -> Optional[subprocess.Popen]:
    args = ["--buffer_size", "60"]
    cmd = ["plotjuggler", *args]

    if layout_path.exists():
        cmd += ["--layout", str(layout_path)]
        print(f"Loading PlotJuggler layout: {layout_path}")
    else:
        print(f"No layout file found at {layout_path}.")
        print("After configuring your plots, save it:")
        print("  File → Save Layout → scripts/plotjuggler_layout.xml")

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
        )
        print(f"PlotJuggler launched (PID {proc.pid})")
        return proc
    except FileNotFoundError:
        print("Warning: 'plotjuggler' not found in PATH — skipping auto-launch.")
        print("  Install:  sudo snap install plotjuggler")
        print("  Or pass:  --no-plotjuggler")
        return None


def bridge(port: str, udp_port: int) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = ("127.0.0.1", udp_port)
    packets = 0

    while True:
        print(f"Connecting to {port} ...")
        try:
            with serial.Serial(port, timeout=1.0) as ser:
                print(f"Connected. Streaming to UDP {udp_port}.  Ctrl+C to stop.\n")
                while True:
                    line = ser.readline()
                    if not line:
                        continue
                    sock.sendto(line, target)
                    packets += 1
                    if packets % STATS_INTERVAL == 0:
                        print(f"  {packets} packets forwarded")

        except serial.serialutil.SerialException as exc:
            print(f"Serial error: {exc}")
            print(f"Retrying in {RECONNECT_DELAY_S}s ...  (Ctrl+C to quit)\n")
            time.sleep(RECONNECT_DELAY_S)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "port",
        help="Serial port for SerialUSB1, e.g. /dev/ttyACM1",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=9870,
        metavar="PORT",
        help="UDP port PlotJuggler is listening on (default: 9870)",
    )
    parser.add_argument(
        "--layout",
        type=Path,
        default=DEFAULT_LAYOUT,
        metavar="FILE",
        help=f"PlotJuggler layout file (default: {DEFAULT_LAYOUT})",
    )
    parser.add_argument(
        "--no-plotjuggler",
        action="store_true",
        help="Skip launching PlotJuggler (run bridge only)",
    )
    args = parser.parse_args()

    pj_proc: Optional[subprocess.Popen] = None
    if not args.no_plotjuggler:
        pj_proc = launch_plotjuggler(args.layout)

    try:
        bridge(args.port, args.udp_port)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if pj_proc is not None and pj_proc.poll() is None:
            pj_proc.terminate()
            print(f"PlotJuggler (PID {pj_proc.pid}) terminated.")


if __name__ == "__main__":
    main()
