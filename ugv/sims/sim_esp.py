#!/usr/bin/env python3
"""
Simulated ESP32: writes sim_esp.dat (space-padded JSON, same style as rover_pos.dat)
at 4 Hz. By default the robot drives counter-clockwise around a 2 m radius circle at
0.5 m/s: encoder counts climb, the IMU shows the turn rate and centripetal accel.

Layout: ugv/sims/sim_esp.py  ->  ugv/esp/sim_esp.dat

  python sim_esp.py                       # circle, r=2 m, 0.5 m/s
  python sim_esp.py --radius 0            # stationary
  python sim_esp.py --radius 3 --speed 1  # bigger/faster circle
  python sim_esp.py --wheel-base 0.4 --wheel-diameter 0.2 --ticks-per-rev 1440

Fields
  seq        increments every update (reader can detect fresh data / dropouts)
  ts         unix time of the update
  enc        cumulative signed tick counts per wheel (NOT deltas) + wheel velocity (ticks/s)
  imu.gyro   rad/s (x, y, z)
  imu.accel  m/s^2 (x, y, z); z ~ 9.81 when level, y = centripetal accel when circling
  imu.quat   orientation (w, x, y, z), yaw only

Run alongside sim_rover.py: both derive the circle angle from wall-clock time with the
same formula, so GNSS position and ESP heading/encoders stay consistent.
Wheel base / diameter / ticks-per-rev are PLACEHOLDERS: set them to your robot's values.
"""

import argparse
import json
import math
import mmap
import os
import random
import time

DEFAULT_FILE = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "esp", "sim_esp.dat")
)
MMAP_SIZE = 512
G = 9.80665


def publish(path, payload):
    data = json.dumps(payload, separators=(",", ":")).ljust(MMAP_SIZE).encode("utf-8")
    if len(data) > MMAP_SIZE:
        raise ValueError("payload exceeds MMAP_SIZE")

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    if not os.path.exists(path) or os.path.getsize(path) < MMAP_SIZE:
        with open(path, "wb") as f:
            f.write(b" " * MMAP_SIZE)

    with open(path, "r+b") as f:
        mm = mmap.mmap(f.fileno(), MMAP_SIZE)
        mm.seek(0)
        mm.write(data)
        mm.close()


def main():
    ap = argparse.ArgumentParser(description="Spoof an ESP32 (encoders + IMU), driving a circle")
    ap.add_argument("--file", default=DEFAULT_FILE, help="output path (default ../esp/sim_esp.dat)")
    ap.add_argument("--hz", type=float, default=4.0, help="update rate (default 4)")
    ap.add_argument("--radius", type=float, default=2.0, help="circle radius in m (default 2; 0 = stationary)")
    ap.add_argument("--speed", type=float, default=0.5, help="forward speed in m/s (default 0.5)")
    ap.add_argument("--wheel-base", type=float, default=0.30, help="distance between wheels, m (default 0.30)")
    ap.add_argument("--wheel-diameter", type=float, default=0.15, help="wheel diameter, m (default 0.15)")
    ap.add_argument("--ticks-per-rev", type=int, default=1000, help="encoder ticks per wheel rev (default 1000)")
    ap.add_argument("--left-ticks", type=int, default=0, help="starting left tick count")
    ap.add_argument("--right-ticks", type=int, default=0, help="starting right tick count")
    args = ap.parse_args()

    dt = 1.0 / args.hz
    seq = 0
    moving = args.radius > 0 and args.speed != 0

    v = args.speed if moving else 0.0
    omega = (v / args.radius) if moving else 0.0            # rad/s, CCW positive
    # differential drive: CCW turn -> right wheel faster, left wheel slower
    v_left = v - omega * args.wheel_base / 2
    v_right = v + omega * args.wheel_base / 2
    ticks_per_m = args.ticks_per_rev / (math.pi * args.wheel_diameter)
    left_tps = v_left * ticks_per_m                          # ticks/s
    right_tps = v_right * ticks_per_m
    centripetal = v * omega                                  # m/s^2, toward circle centre (body +y for CCW)

    left_f = float(args.left_ticks)
    right_f = float(args.right_ticks)
    last = time.monotonic()

    print(f"Simulating ESP32 -> {args.file} @ {args.hz} Hz")
    if moving:
        print(f"Circle r={args.radius} m, v={v} m/s, omega={omega:.3f} rad/s (period {2 * math.pi / omega:.1f}s)")
        print(f"Wheel speeds: L={left_tps:.0f} R={right_tps:.0f} ticks/s "
              f"(base={args.wheel_base} m, dia={args.wheel_diameter} m, {args.ticks_per_rev} tpr)")
    else:
        print("Stationary")
    print("Ctrl+C to stop.\n")

    try:
        while True:
            now = time.monotonic()
            step = now - last
            last = now
            left_f += left_tps * step
            right_f += right_tps * step
            seq += 1

            # heading: same wall-clock angle formula as sim_rover.py (+90 deg = direction of travel)
            yaw = ((omega * time.time()) + math.pi / 2) if moving else 0.0
            quat = [round(math.cos(yaw / 2), 5), 0.0, 0.0, round(math.sin(yaw / 2), 5)]

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
                    "gyro": [round(random.gauss(0, 0.002), 5),
                             round(random.gauss(0, 0.002), 5),
                             round(omega + random.gauss(0, 0.003), 5)],
                    "accel": [round(random.gauss(0, 0.02), 4),
                              round(centripetal + random.gauss(0, 0.02), 4),
                              round(G + random.gauss(0, 0.03), 4)],
                    "quat": quat,
                },
            }
            publish(args.file, payload)
            print(f"[ESP] seq={seq:5d} L={payload['enc']['left_ticks']} R={payload['enc']['right_ticks']} "
                  f"gyro_z={payload['imu']['gyro'][2]:+.3f} accel_y={payload['imu']['accel'][1]:+.3f}")
            time.sleep(dt)

    except KeyboardInterrupt:
        print("\nSimulator stopped.")


if __name__ == "__main__":
    main()
