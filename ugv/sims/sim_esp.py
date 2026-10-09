#!/usr/bin/env python3
"""
Simulated ESP32: writes sim_esp.dat (space-padded JSON) at specified update rate.

Modes:
  python sim_esp.py                 # Circle, r=2 m, 0.5 m/s
  python sim_esp.py --straight      # Straight line at 0.5 m/s
  python sim_esp.py --radius 0      # Stationary
"""

import argparse
import json
import math
import mmap
import os
import random
import time

DEFAULT_FILE = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "sim_esp.dat")
)
MMAP_SIZE = 512
G = 9.80665


def publish(path, payload):
    data = json.dumps(payload, separators=(",", ":")).ljust(MMAP_SIZE).encode("utf-8")
    if len(data) > MMAP_SIZE:
        raise ValueError("Payload exceeds MMAP_SIZE")

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    
    # Create or resize file to match required size
    if not os.path.exists(path) or os.path.getsize(path) != MMAP_SIZE:
        with open(path, "wb") as f:
            f.write(b" " * MMAP_SIZE)

    # Use direct file binary write and flush to guarantee disk/memory visibility
    with open(path, "r+b") as f:
        f.seek(0)
        f.write(data)
        f.flush()
        os.fsync(f.fileno())


def main():
    ap = argparse.ArgumentParser(description="Spoof an ESP32 (encoders + IMU)")
    ap.add_argument("--file", default=DEFAULT_FILE, help="Output path (default ../esp/sim_esp.dat)")
    ap.add_argument("--hz", type=float, default=4.0, help="Update rate in Hz (default 4.0)")
    ap.add_argument("--radius", type=float, default=2.0, help="Circle radius in m (default 2.0; 0 = stationary unless --straight)")
    ap.add_argument("--speed", type=float, default=0.5, help="Forward speed in m/s (default 0.5)")
    ap.add_argument("--straight", action="store_true", help="Drive in a straight line instead of a circle")
    ap.add_argument("--wheel-base", type=float, default=0.30, help="Distance between wheels in meters")
    ap.add_argument("--wheel-diameter", type=float, default=0.15, help="Wheel diameter in meters")
    ap.add_argument("--ticks-per-rev", type=int, default=1000, help="Encoder ticks per wheel revolution")
    ap.add_argument("--left-ticks", type=int, default=0, help="Starting left tick count")
    ap.add_argument("--right-ticks", type=int, default=0, help="Starting right tick count")
    args = ap.parse_args()

    dt = 1.0 / args.hz
    seq = 0

    if args.straight:
        moving = True
        v = args.speed
        omega = 0.0
        centripetal = 0.0
    elif args.radius > 0 and args.speed != 0:
        moving = True
        v = args.speed
        omega = v / args.radius  # rad/s, CCW positive
        centripetal = v * omega  # m/s^2, body y-axis
    else:
        moving = False
        v = 0.0
        omega = 0.0
        centripetal = 0.0

    # Differential drive wheel speeds
    v_left = v - omega * args.wheel_base / 2.0
    v_right = v + omega * args.wheel_base / 2.0
    
    ticks_per_m = args.ticks_per_rev / (math.pi * args.wheel_diameter)
    left_tps = v_left * ticks_per_m
    right_tps = v_right * ticks_per_m

    left_f = float(args.left_ticks)
    right_f = float(args.right_ticks)
    last = time.monotonic()
    start_time = time.time()

    print(f"Simulating ESP32 -> {args.file} @ {args.hz} Hz")
    if args.straight:
        print(f"Mode: Straight line @ {v} m/s")
    elif moving:
        print(f"Mode: Circle r={args.radius} m @ {v} m/s (omega={omega:.3f} rad/s)")
    else:
        print("Mode: Stationary")

    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            now = time.monotonic()
            step = now - last
            last = now

            left_f += left_tps * step
            right_f += right_tps * step
            seq += 1

            # Heading calculation
            if args.straight or not moving:
                yaw = 0.0
            else:
                yaw = (omega * (time.time() - start_time)) + (math.pi / 2.0)

            quat = [round(math.cos(yaw / 2.0), 5), 0.0, 0.0, round(math.sin(yaw / 2.0), 5)]

            payload = {
                "seq": seq,
                "ts": time.time(),
                "enc": {
                    "left_ticks": int(round(left_f)),
                    "right_ticks": int(round(right_f)),
                    "left_vel": round(left_tps, 2),
                    "right_vel": round(right_tps, 2),
                },
                "imu": {
                    "gyro": [
                        round(random.gauss(0, 0.002), 5),
                        round(random.gauss(0, 0.002), 5),
                        round(omega + random.gauss(0, 0.003), 5),
                    ],
                    "accel": [
                        round(random.gauss(0, 0.02), 4),
                        round(centripetal + random.gauss(0, 0.02), 4),
                        round(G + random.gauss(0, 0.03), 4),
                    ],
                    "quat": quat,
                },
            }

            publish(args.file, payload)
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nSimulator stopped.")


if __name__ == "__main__":
    main()