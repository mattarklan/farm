#!/usr/bin/env python3
"""
Combined robot simulator: ONE process simulates the robot and writes BOTH shared files
from the same state, so GNSS (rover_pos.dat) and ESP32 data (sim_esp.dat) can never
disagree or drift out of sync. It reads velocity commands from cmd_vel.dat.

Layout: ugv/sims/sim_robot.py
  writes  ugv/rtk/rover_pos.dat   (GNSS fix / lat / lon / sats / hdop / corr_age)
  writes  ugv/esp/sim_esp.dat     (encoder ticks, wheel vel, IMU gyro/accel/quat)
  reads   ugv/esp/cmd_vel.dat     {"linear": m/s, "angular": rad/s, "ts": unix time}
  sends   UDP pose to the lidar sim (only with --lidar-pose-port; --cmap gives the map size)

The visualiser normally launches this for you. Standalone:

  python sim_robot.py                        # sits still until a cmd_vel.dat arrives
  python sim_robot.py --circle 2             # drives a 2 m radius circle by itself
  python sim_robot.py --fix "Fixed RTK"      # skip the fix warm-up sequence

cmd_vel.dat has a watchdog: if its "ts" is older than 0.5 s the robot is told to stop
(like a real motor controller). Anything can write it: the visualiser's arrow keys, or
your own program (+x forward, +angular = counter-clockwise).

Frame: local metres, +x east, +y north, yaw 0 = facing east, counter-clockwise positive.
Do NOT run this together with sim_rover.py / sim_esp.py (they'd fight over the same files).
"""

import argparse
import json
import math
import mmap
import os
import random
import socket
import time

HERE = os.path.dirname(os.path.abspath(__file__))
RTK_FILE = os.path.normpath(os.path.join(HERE, "..", "rtk", "rover_pos.dat"))
ESP_FILE = os.path.normpath(os.path.join(HERE, "..", "sims", "sim_esp.dat"))
CMD_FILE = os.path.normpath(os.path.join(HERE, "..", "sims", "cmd_vel.dat"))
RTK_SIZE = 256
ESP_SIZE = 512
M_PER_DEG = 111320.0
G = 9.80665
CMD_TIMEOUT = 0.5   # s

# name, seconds in stage (0 = stay forever), sats, hdop, corr_age, position noise (m)
STAGES = [
    ("No Fix",    4,  (0, 4),   (99.0, 99.0), (0.0, 0.0), 0.0),
    ("3D Fix",    6,  (7, 11),  (1.2, 2.0),   (0.0, 0.0), 2.0),
    ("DGNSS",     6,  (10, 14), (0.8, 1.3),   (1.0, 3.0), 0.5),
    ("Float RTK", 10, (14, 20), (0.5, 0.9),   (0.5, 2.0), 0.1),
    ("Fixed RTK", 0,  (18, 26), (0.4, 0.7),   (0.3, 1.5), 0.01),
]
STAGE_NAMES = [s[0] for s in STAGES]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def write_dat(path, size, payload):
    data = json.dumps(payload).ljust(size).encode("utf-8")
    if len(data) > size:
        raise ValueError(f"payload too large for {size}-byte file")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    if not os.path.exists(path) or os.path.getsize(path) < size:
        with open(path, "wb") as f:
            f.write(b" " * size)
    with open(path, "r+b") as f:
        mm = mmap.mmap(f.fileno(), size)
        mm.seek(0)
        mm.write(data)
        mm.close()


def read_cmd(path):
    """Return (linear, angular) from cmd_vel.dat; (0, 0) if missing, unreadable or stale."""
    try:
        with open(path, "rb") as f:
            raw = f.read()
        d = json.loads(raw.decode("utf-8", "ignore").replace("\x00", "").strip())
        if time.time() - float(d.get("ts", 0)) > CMD_TIMEOUT:
            return 0.0, 0.0
        return float(d.get("linear", 0.0)), float(d.get("angular", 0.0))
    except (OSError, ValueError, TypeError):
        return 0.0, 0.0


def main():
    ap = argparse.ArgumentParser(description="Combined GNSS + ESP32 robot simulator")
    ap.add_argument("--lat", type=float, default=-31.9505, help="latitude of the map centre / local origin")
    ap.add_argument("--lon", type=float, default=115.8605, help="longitude of the map centre / local origin")
    ap.add_argument("--start-x", type=float, default=0.0, help="start east offset from origin, m")
    ap.add_argument("--start-y", type=float, default=0.0, help="start north offset from origin, m")
    ap.add_argument("--start-yaw", type=float, default=0.0, help="start heading, deg (0 = east, CCW positive)")
    ap.add_argument("--circle", type=float, default=0.0, help="autopilot: drive a circle of this radius (m); 0 = off")
    ap.add_argument("--speed", type=float, default=0.5, help="circle autopilot speed, m/s")
    ap.add_argument("--lin-accel", type=float, default=1.0, help="max linear accel, m/s^2")
    ap.add_argument("--ang-accel", type=float, default=3.0, help="max angular accel, rad/s^2")
    ap.add_argument("--esp-hz", type=float, default=1.0, help="ESP32 (encoders + IMU) update rate, default 1")
    ap.add_argument("--gnss-hz", type=float, default=1.0, help="GNSS update rate, default 1")
    ap.add_argument("--fix", choices=STAGE_NAMES, help="lock to one fix state (default: realistic warm-up)")
    ap.add_argument("--wheel-base", type=float, default=0.30)
    ap.add_argument("--wheel-diameter", type=float, default=0.15)
    ap.add_argument("--ticks-per-rev", type=int, default=1000)
    ap.add_argument("--rtk-file", default=RTK_FILE)
    ap.add_argument("--esp-file", default=ESP_FILE)
    ap.add_argument("--cmd-file", default=CMD_FILE)
    ap.add_argument("--lidar-pose-port", type=int, default=0,
                    help="UDP port of the lidar sim to stream the TRUE robot pose to (0 = off)")
    ap.add_argument("--lidar-pose-hz", type=float, default=20.0, help="pose stream rate (default 20)")
    ap.add_argument("--cmap", help="costmap PNG (only used for its size, to place the pose in the lidar's map frame)")
    ap.add_argument("--res", type=float, default=0.05, help="metres per costmap pixel (default 0.05)")
    args = ap.parse_args()

    tpm = args.ticks_per_rev / (math.pi * args.wheel_diameter)   # ticks per metre of wheel travel
    x, y = args.start_x, args.start_y
    yaw = math.radians(args.start_yaw)
    v = w = lin_acc = 0.0
    left_f = right_f = 0.0
    seq = 0
    cos_lat0 = math.cos(math.radians(args.lat))

    # optional: stream the TRUE pose to the lidar simulator. Its world frame is millimetres,
    # origin at the top-left of the map image, +x right, +y DOWN, heading clockwise-positive.
    pose_sock = None
    map_w_mm = map_h_mm = 0.0
    next_pose = time.monotonic()
    if args.lidar_pose_port:
        try:
            from PIL import Image
            with Image.open(args.cmap) as im:
                mw, mh = im.size
            map_w_mm, map_h_mm = mw * args.res * 1000.0, mh * args.res * 1000.0
            pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"[SIM] streaming pose to lidar sim on UDP {args.lidar_pose_port} "
                  f"(map {mw}x{mh}px)", flush=True)
        except Exception as e:
            print(f"[SIM] lidar pose output disabled: {e}", flush=True)

    stage_idx = STAGE_NAMES.index(args.fix) if args.fix else 0
    last_stage = None
    t_prev = time.monotonic()
    stage_start = t_prev
    next_g = next_e = t_prev
    next_status = t_prev + 5
    cv = cw = 0.0

    print(f"[SIM] origin {args.lat:.7f}, {args.lon:.7f}  start=({x:.2f}, {y:.2f}) m yaw={args.start_yaw:.0f} deg", flush=True)
    print(f"[SIM] writing {args.rtk_file}", flush=True)
    print(f"[SIM] writing {args.esp_file}", flush=True)
    if args.circle > 0:
        print(f"[SIM] circle autopilot r={args.circle} m v={args.speed} m/s", flush=True)
    else:
        print(f"[SIM] reading commands from {args.cmd_file}", flush=True)

    try:
        while True:
            now = time.monotonic()
            dt = min(now - t_prev, 0.1)
            t_prev = now

            # ---- fix state machine ----
            if not args.fix:
                dur = STAGES[stage_idx][1]
                if dur and now - stage_start >= dur:
                    stage_idx += 1
                    stage_start = now
            if stage_idx != last_stage:
                last_stage = stage_idx
                print(f"[SIM] *** FIX: {STAGES[stage_idx][0]} ***", flush=True)
            name, _, sat_r, hdop_r, age_r, noise_m = STAGES[stage_idx]

            # ---- physics ----
            if dt > 0:
                if args.circle > 0:
                    cv, cw = args.speed, args.speed / args.circle
                else:
                    cv, cw = read_cmd(args.cmd_file)
                lim_v, lim_w = args.lin_accel * dt, args.ang_accel * dt
                dv = max(-lim_v, min(lim_v, cv - v))
                dw = max(-lim_w, min(lim_w, cw - w))
                v += dv
                w += dw
                lin_acc = dv / dt
                mid = yaw + w * dt / 2
                x += v * math.cos(mid) * dt
                y += v * math.sin(mid) * dt
                yaw = wrap(yaw + w * dt)
                left_f += (v - w * args.wheel_base / 2) * tpm * dt
                right_f += (v + w * args.wheel_base / 2) * tpm * dt

            # ---- publish GNSS ----
            if now >= next_g:
                next_g = max(next_g + 1.0 / args.gnss_hz, now)
                if name == "No Fix":
                    write_dat(args.rtk_file, RTK_SIZE, {
                        "lat": 0.0, "lon": 0.0, "fix": name, "sats": random.randint(*sat_r),
                        "hdop": 99.0, "corr_age": 0.0, "ts": time.time()})
                else:
                    east = x + random.gauss(0, noise_m)
                    north = y + random.gauss(0, noise_m)
                    write_dat(args.rtk_file, RTK_SIZE, {
                        "lat": round(args.lat + north / M_PER_DEG, 9),
                        "lon": round(args.lon + east / (M_PER_DEG * cos_lat0), 9),
                        "fix": name, "sats": random.randint(*sat_r),
                        "hdop": round(random.uniform(*hdop_r), 2),
                        "corr_age": round(random.uniform(*age_r), 1),
                        "ts": time.time()})

            # ---- publish ESP32 ----
            if now >= next_e:
                next_e = max(next_e + 1.0 / args.esp_hz, now)
                seq += 1
                write_dat(args.esp_file, ESP_SIZE, {
                    "seq": seq,
                    "ts": time.time(),
                    "enc": {
                        "left_ticks": int(round(left_f)),
                        "right_ticks": int(round(right_f)),
                        "left_vel": round((v - w * args.wheel_base / 2) * tpm, 2),
                        "right_vel": round((v + w * args.wheel_base / 2) * tpm, 2),
                    },
                    "imu": {
                        "gyro": [round(random.gauss(0, 0.002), 5),
                                 round(random.gauss(0, 0.002), 5),
                                 round(w + random.gauss(0, 0.003), 5)],
                        "accel": [round(lin_acc + random.gauss(0, 0.03), 4),
                                  round(v * w + random.gauss(0, 0.02), 4),
                                  round(G + random.gauss(0, 0.03), 4)],
                        "quat": [round(math.cos(yaw / 2), 5), 0.0, 0.0, round(math.sin(yaw / 2), 5)],
                    },
                })

            # ---- stream true pose to the lidar sim ----
            if pose_sock is not None and now >= next_pose:
                next_pose = max(next_pose + 1.0 / args.lidar_pose_hz, now)
                try:
                    pose_sock.sendto(json.dumps({
                        "x": map_w_mm / 2 + x * 1000.0,      # east of map centre -> +x
                        "y": map_h_mm / 2 - y * 1000.0,      # north of map centre -> -y (image rows go down)
                        "heading": -math.degrees(yaw),       # CCW-from-east -> clockwise in image coords
                    }).encode("utf-8"), ("127.0.0.1", args.lidar_pose_port))
                except OSError:
                    pass

            if now >= next_status:
                next_status = now + 5
                print(f"[SIM] {name:<9} pos=({x:+.2f}, {y:+.2f}) yaw={math.degrees(yaw):+.0f} deg "
                      f"v={v:+.2f} w={w:+.2f} cmd=({cv:+.2f}, {cw:+.2f})", flush=True)

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[SIM] stopped.", flush=True)


if __name__ == "__main__":
    main()
