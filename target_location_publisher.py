#!/usr/bin/env python3
"""
Publishes a landing target location to a shared file that the C++ RTH
state machine reads via GetTargetLocation().

File format (one line):
    lat_deg,lon_deg,unix_timestamp

Python writes to a .tmp file then atomically renames it, so C++ never
reads a half-written file.

Usage:
    python3 target_location_publisher.py              # random pick, 1 Hz
    python3 target_location_publisher.py --hz 5       # 5 Hz update
    python3 target_location_publisher.py --mode cycle # cycle through spots in order
"""

import time
import random
import os
import argparse
import signal
import sys

# ---------------------------------------------------------------------------
# Edit this list to add/remove candidate landing spots (decimal degrees).
# ---------------------------------------------------------------------------
LANDING_SPOTS = [
    (12.994319, 101.443273),   # spot A
    (12.995100, 101.444500),   # spot B
    (12.993800, 101.442100),   # spot C
]

TARGET_FILE = "/tmp/rth_target.txt"


def publish(lat: float, lon: float) -> None:
    """Write lat/lon atomically so C++ never sees a partial file."""
    tmp = TARGET_FILE + ".tmp"
    with open(tmp, "w") as f:
        f.write(f"{lat:.8f},{lon:.8f},{time.time():.3f}\n")
    os.replace(tmp, TARGET_FILE)


def main() -> None:
    parser = argparse.ArgumentParser(description="RTH target location publisher")
    parser.add_argument(
        "--hz", type=float, default=1.0,
        help="Publish frequency in Hz (default: 1.0)"
    )
    parser.add_argument(
        "--mode", choices=["random", "cycle"], default="random",
        help="random: pick a random spot each tick | cycle: go through spots in order"
    )
    args = parser.parse_args()

    if not LANDING_SPOTS:
        print("ERROR: LANDING_SPOTS is empty.", file=sys.stderr)
        sys.exit(1)

    period = 1.0 / args.hz

    # Clean up the file on exit so C++ sees stale data immediately.
    def on_exit(sig, frame):
        print("\nShutting down. Removing target file.")
        try:
            os.remove(TARGET_FILE)
        except FileNotFoundError:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT,  on_exit)
    signal.signal(signal.SIGTERM, on_exit)

    print(f"Publishing target at {args.hz} Hz (mode={args.mode}) → {TARGET_FILE}")
    for i, (lat, lon) in enumerate(LANDING_SPOTS):
        print(f"  spot {i}: ({lat:.6f}, {lon:.6f})")
    print()

    index = 0
    while True:
        if args.mode == "random":
            lat, lon = random.choice(LANDING_SPOTS)
        else:
            lat, lon = LANDING_SPOTS[index % len(LANDING_SPOTS)]
            index += 1

        publish(lat, lon)
        print(f"[{time.strftime('%H:%M:%S')}] target → ({lat:.6f}, {lon:.6f})")
        time.sleep(period)


if __name__ == "__main__":
    main()
