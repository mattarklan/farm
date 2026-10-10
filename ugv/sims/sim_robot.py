#!/usr/bin/env python3
"""
Simulated robot. Stands in for the REAL rover + ESP32 so the visualiser (and your own program) can't
tell the difference. Everything goes over the UDP bus defined in common/protocol.py:

  listens   cmd_vel   (port 5006)   velocity commands (stops if none for 0.5 s, like a real motor controller)
  publishes esp       (port 5006)   cumulative encoder ticks + wheel speed + IMU (gyro, accel, quaternion)
  publishes rtk       (port 5007)   lat / lon / fix / sats / hdop / corr_age, with a realistic fix warm-up
  sends     true pose to sim_lidar.py (plain UDP, 127.0.0.1:5000), so the simulated lidar knows where to
            ray-cast from. Nothing else ever sees that.

The robot obeys the same speed and acceleration limits as everything else (common/config.py). With
--cmap it also knows the hidden WORLD, and stops instead of driving through walls.

  python sim_robot.py --cmap ../cmap.png                 # sits still until a cmd_vel arrives
  python sim_robot.py --cmap ../cmap.png --circle 2      # drives a 2 m radius circle by itself
  python sim_robot.py --fix "Fixed RTK"                  # skip the fix warm-up sequence
  python sim_robot.py --esp-hz 5 --gnss-hz 5             # faster telemetry (default 1 Hz each)

Normally started for you by run_sim.py.   Layout: ugv/sims/sim_robot.py
"""

import argparse
import json
import math
import os
import random
import socket
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from common import config as cfg                       # noqa: E402
from common.protocol import Publisher, Subscriber, ProtocolError, selftest   # noqa: E402

M_PER_DEG = 111320.0
G = 9.80665

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


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _load_gray(path, invert=False):
    """Greyscale world image. Transparent pixels count as FREE (white, or black with --invert)."""
    from PIL import Image
    im = Image.open(path)
    if im.mode in ("RGBA", "LA", "PA") or (im.mode == "P" and "transparency" in im.info):
        im = im.convert("RGBA")
        bg = Image.new("RGBA", im.size, (0, 0, 0, 255) if invert else (255, 255, 255, 255))
        bg.alpha_composite(im)
        im = bg
    return im.convert("L")


class World:
    """The hidden world (cmap.png). Used only for collisions and for the lidar's ground-truth pose."""

    def __init__(self, path, res, invert):
        import numpy as np
        gray = np.asarray(_load_gray(path, invert), dtype=np.uint8)
        self.wall = (gray > 127) if invert else (gray < 128)
        self.h, self.w = self.wall.shape
        self.res = res
        self.left, self.top = -self.w * res / 2, self.h * res / 2

    def is_wall(self, x, y):
        col = math.floor((x - self.left) / self.res)
        row = math.floor((self.top - y) / self.res)
        return 0 <= row < self.h and 0 <= col < self.w and bool(self.wall[row, col])

    def hits(self, x, y, yaw, length, width):
        c, s = math.cos(yaw), math.sin(yaw)
        hl, hw = length / 2, width / 2
        for px, py in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw), (hl, 0), (-hl, 0), (0, hw), (0, -hw)):
            if self.is_wall(x + px * c - py * s, y + px * s + py * c):
                return True
        return False


def main():
    ap = argparse.ArgumentParser(description="Simulated rover + ESP32 on the UDP bus")
    ap.add_argument("--lat", type=float, default=cfg.ORIGIN_LAT, help="latitude of the map centre (default: common/config.py)")
    ap.add_argument("--lon", type=float, default=cfg.ORIGIN_LON, help="longitude of the map centre")
    ap.add_argument("--start-x", type=float, default=0.0, help="start east of the map centre, m")
    ap.add_argument("--start-y", type=float, default=0.0, help="start north of the map centre, m")
    ap.add_argument("--start-yaw", type=float, default=0.0, help="start heading, deg (0 = east, CCW positive)")
    ap.add_argument("--circle", type=float, default=0.0, help="autopilot: drive a circle of this radius (m), ignoring cmd_vel")
    ap.add_argument("--speed", type=float, default=0.5, help="circle autopilot speed, m/s")
    ap.add_argument("--max-speed", type=float, default=cfg.MAX_SPEED)
    ap.add_argument("--max-turn", type=float, default=cfg.MAX_TURN)
    ap.add_argument("--lin-accel", type=float, default=cfg.LIN_ACCEL)
    ap.add_argument("--ang-accel", type=float, default=cfg.ANG_ACCEL)
    ap.add_argument("--esp-hz", type=float, default=1.0, help="encoder/IMU message rate (default 1)")
    ap.add_argument("--gnss-hz", type=float, default=1.0, help="rtk message rate (default 1)")
    ap.add_argument("--fix", choices=STAGE_NAMES, help="lock to one fix state (default: realistic warm-up)")
    ap.add_argument("--wheel-base", type=float, default=cfg.WHEEL_BASE)
    ap.add_argument("--wheel-diameter", type=float, default=cfg.WHEEL_DIAMETER)
    ap.add_argument("--ticks-per-rev", type=int, default=cfg.TICKS_PER_REV)
    ap.add_argument("--robot-length", type=float, default=cfg.ROBOT_LENGTH)
    ap.add_argument("--robot-width", type=float, default=cfg.ROBOT_WIDTH)
    ap.add_argument("--cmap", help="the hidden world image: collisions + lidar ground truth (black = wall)")
    ap.add_argument("--res", type=float, default=cfg.MAP_RES, help="metres per world pixel")
    ap.add_argument("--invert", action="store_true", help="world image is black = free, white = wall")
    ap.add_argument("--lidar-pose-port", type=int, default=5000, help="UDP port of sim_lidar's ground-truth pose input (0 = off)")
    ap.add_argument("--lidar-pose-hz", type=float, default=20.0)
    ap.add_argument("--quiet", action="store_true", help="no periodic status line")
    args = ap.parse_args()

    world = None
    if args.cmap:
        try:
            world = World(args.cmap, args.res, args.invert)
            import numpy as np
            frac = float(world.wall.mean())
            near = world.is_wall(args.start_x, args.start_y)
            r = int(1.0 / args.res)
            r0, c0 = int((world.top - args.start_y) / args.res), int((args.start_x - world.left) / args.res)
            around = world.wall[max(0, r0 - r):r0 + r, max(0, c0 - r):c0 + r]
            if near or frac > 0.5 or (around.size and around.mean() > 0.5):
                print(f"[SIM] *** WARNING: {frac * 100:.0f}% of the world image counts as WALL and the start position is "
                      f"{'INSIDE a wall' if near else 'in a mostly-wall area'}. If your image is black = free / white = wall, "
                      f"add --invert (to run_sim.py or sim_robot.py AND sim_lidar.py). Otherwise the lidar sees walls "
                      f"0.2 m away in every direction.", flush=True)
            print(f"[SIM] world {args.cmap}: {world.w}x{world.h}px = {world.w * args.res:.1f} x {world.h * args.res:.1f} m "
                  f"(collisions on)", flush=True)
        except Exception as e:
            print(f"[SIM] could not load world {args.cmap}: {e} - collisions and lidar pose are off", flush=True)
    else:
        print("[SIM] no --cmap given: no collisions, and no ground-truth pose for the lidar sim", flush=True)

    pub, sub = Publisher(), Subscriber(["cmd_vel"])
    pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if (world and args.lidar_pose_port) else None

    tpm = args.ticks_per_rev / (math.pi * args.wheel_diameter)        # ticks per metre of wheel travel
    cos_lat0 = math.cos(math.radians(args.lat))
    x, y, yaw = args.start_x, args.start_y, math.radians(args.start_yaw)
    v = w = lin_acc = 0.0
    left_f = right_f = 0.0
    seq = 0
    cmd_v = cmd_w = 0.0
    last_cmd = -1e9                                                   # monotonic arrival time of the last cmd_vel
    stage_idx = STAGE_NAMES.index(args.fix) if args.fix else 0
    last_stage = None
    t_prev = time.monotonic()
    stage_start = next_g = next_e = next_pose = t_prev
    next_status = t_prev + 5
    last_collision = 0.0

    print(f"[SIM] origin {args.lat:.7f}, {args.lon:.7f}   start ({x:.2f}, {y:.2f}) m, yaw {args.start_yaw:.0f} deg", flush=True)
    print(f"[SIM] limits {args.max_speed} m/s, {args.max_turn} rad/s, accel {args.lin_accel} m/s^2, {args.ang_accel} rad/s^2", flush=True)
    print(f"[SIM] publishing esp @ {args.esp_hz} Hz (port 5006), rtk @ {args.gnss_hz} Hz (port 5007)", flush=True)
    print("[SIM] circle autopilot r=%.1f m" % args.circle if args.circle > 0 else "[SIM] listening for cmd_vel on port 5006", flush=True)

    try:
        while True:
            now = time.monotonic()
            dt = min(now - t_prev, 0.1)
            t_prev = now

            for m in sub.poll():
                cmd_v, cmd_w, last_cmd = m["linear"], m["angular"], now

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
                    tv, tw = args.speed, args.speed / args.circle
                elif now - last_cmd > cfg.CMD_TIMEOUT:
                    tv = tw = 0.0                                     # watchdog
                else:
                    tv, tw = cmd_v, cmd_w
                tv, tw = clamp(tv, -args.max_speed, args.max_speed), clamp(tw, -args.max_turn, args.max_turn)
                dv = clamp(tv - v, -args.lin_accel * dt, args.lin_accel * dt)
                dw = clamp(tw - w, -args.ang_accel * dt, args.ang_accel * dt)
                nv, nw = v + dv, w + dw
                mid = yaw + nw * dt / 2
                nx, ny, nyaw = x + nv * math.cos(mid) * dt, y + nv * math.sin(mid) * dt, wrap(yaw + nw * dt)
                if world and (nv or nw) and world.hits(nx, ny, nyaw, args.robot_length, args.robot_width):
                    nv = nw = 0.0                                    # blocked by a wall: stop dead
                    nx, ny, nyaw = x, y, yaw
                    if now - last_collision > 2.0:
                        last_collision = now
                        print(f"[SIM] collision at ({x:+.2f}, {y:+.2f}) - robot stopped", flush=True)
                lin_acc = (nv - v) / dt
                v, w, x, y, yaw = nv, nw, nx, ny, nyaw
                left_f += (v - w * args.wheel_base / 2) * tpm * dt
                right_f += (v + w * args.wheel_base / 2) * tpm * dt

            # ---- publish rtk ----
            if now >= next_g:
                next_g = max(next_g + 1.0 / args.gnss_hz, now)
                try:
                    if name == "No Fix":
                        pub.send("rtk", lat=0.0, lon=0.0, fix=name, sats=random.randint(*sat_r), hdop=99.0, corr_age=0.0)
                    else:
                        east, north = x + random.gauss(0, noise_m), y + random.gauss(0, noise_m)
                        pub.send("rtk", lat=args.lat + north / M_PER_DEG, lon=args.lon + east / (M_PER_DEG * cos_lat0),
                                 fix=name, sats=random.randint(*sat_r), hdop=round(random.uniform(*hdop_r), 2),
                                 corr_age=round(random.uniform(*age_r), 1))
                except ProtocolError as e:
                    print(f"[SIM] rtk message rejected: {e}", flush=True)

            # ---- publish esp ----
            if now >= next_e:
                next_e = max(next_e + 1.0 / args.esp_hz, now)
                seq += 1
                try:
                    pub.send("esp", seq=seq,
                             enc={"left_ticks": int(round(left_f)), "right_ticks": int(round(right_f)),
                                  "left_vel": (v - w * args.wheel_base / 2) * tpm,
                                  "right_vel": (v + w * args.wheel_base / 2) * tpm},
                             imu={"gyro": [random.gauss(0, 0.002), random.gauss(0, 0.002), w + random.gauss(0, 0.003)],
                                  "accel": [lin_acc + random.gauss(0, 0.03), v * w + random.gauss(0, 0.02), G + random.gauss(0, 0.03)],
                                  "quat": [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]})
                except ProtocolError as e:
                    print(f"[SIM] esp message rejected: {e}", flush=True)

            # ---- ground-truth pose for the lidar simulator (image frame: mm from top-left, +y down, heading clockwise) ----
            if pose_sock is not None and now >= next_pose:
                next_pose = max(next_pose + 1.0 / args.lidar_pose_hz, now)
                try:
                    pose_sock.sendto(json.dumps({
                        "x": world.w * world.res * 500.0 + x * 1000.0,
                        "y": world.h * world.res * 500.0 - y * 1000.0,
                        "heading": -math.degrees(yaw),
                        "invert": bool(args.invert), "mm_per_pixel": args.res * 1000.0}).encode("utf-8"), ("127.0.0.1", args.lidar_pose_port))
                except OSError:
                    pass

            if not args.quiet and now >= next_status:
                next_status = now + 5
                print(f"[SIM] {name:<9} pos=({x:+.2f}, {y:+.2f}) yaw={math.degrees(yaw):+4.0f} deg  v={v:+.2f} w={w:+.2f}  "
                      f"cmd=({tv:+.2f}, {tw:+.2f})", flush=True)

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\n[SIM] stopped.", flush=True)


if __name__ == "__main__":
    main()
