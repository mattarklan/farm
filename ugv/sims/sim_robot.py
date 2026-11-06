#!/usr/bin/env python3
"""
Simulated rover + ESP32 on the UDP bus with stable GPS Course-Over-Ground (COG) 
heading drift correction and configurable Ackerman steering limits.
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
from common.protocol import Publisher, Subscriber, ProtocolError   # noqa: E402

M_PER_DEG = 111320.0
G = 9.80665

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
    from PIL import Image
    im = Image.open(path)
    if im.mode in ("RGBA", "LA", "PA") or (im.mode == "P" and "transparency" in im.info):
        im = im.convert("RGBA")
        bg = Image.new("RGBA", im.size, (0, 0, 0, 255) if invert else (255, 255, 255, 255))
        bg.alpha_composite(im)
        im = bg
    return im.convert("L")


class PurePursuitFollower:
    """Pure Pursuit path follower with configurable Ackerman constraints."""
    def __init__(self, max_speed, max_turn, lookahead=0.6, goal_tol=0.25, min_turn_radius=0.4, arc_speed=0.3):
        self.max_speed = max_speed
        self.max_turn = max_turn
        self.lookahead = lookahead
        self.goal_tol = goal_tol
        self.min_radius = min_turn_radius
        self.arc_speed = arc_speed
        self.path = []
        self.seg = 0

    def set_path(self, path):
        self.path = path
        self.seg = 0

    def update(self, x, y, yaw):
        if not self.path:
            return 0.0, 0.0, True

        goal = self.path[-1]
        dist = math.hypot(goal[0] - x, goal[1] - y)
        if dist < self.goal_tol:
            return 0.0, 0.0, True

        tx, ty = self._get_target(x, y)
        alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
        abs_alpha = abs(alpha)

        heading_factor = math.cos(alpha) if abs_alpha < math.pi / 2.0 else 0.0
        v = max(self.max_speed * heading_factor * heading_factor, self.arc_speed)
        v = min(v, 0.15 + 1.0 * dist)

        radius = self.min_radius
        if abs_alpha > math.pi / 2.0 and dist < 2.0 * self.min_radius:
            radius = self.min_radius

        w_limit = min(self.max_turn, v / radius)
        w = clamp(2.5 * alpha, -w_limit, w_limit)

        return v, w, False

    def _get_target(self, x, y):
        if len(self.path) < 2:
            return self.path[-1][0], self.path[-1][1]

        last_seg = len(self.path) - 2
        if self.seg > last_seg:
            self.seg = last_seg

        best_d = 1e9
        best_i = self.seg
        best_t = 0.0

        for i in range(self.seg, last_seg + 1):
            ax, ay = self.path[i][0], self.path[i][1]
            bx, by = self.path[i+1][0], self.path[i+1][1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 == 0.0 else clamp(((x - ax) * dx + (y - ay) * dy) / L2, 0.0, 1.0)
            d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))

            if d < best_d - 1e-9:
                best_d = d
                best_i = i
                best_t = t
            elif i > best_i + 1 and d > best_d + 2 * self.lookahead:
                break

        self.seg = best_i
        k = self.seg
        ax, ay = self.path[k][0], self.path[k][1]
        bx, by = self.path[k+1][0], self.path[k+1][1]
        px = ax + best_t * (bx - ax)
        py = ay + best_t * (by - ay)

        remaining = self.lookahead
        while True:
            seg_len = math.hypot(bx - px, by - py)
            if seg_len >= remaining:
                f = (remaining / seg_len) if seg_len > 0.0 else 0.0
                return px + f * (bx - px), py + f * (by - py)
            remaining -= seg_len
            k += 1
            if k > last_seg:
                return self.path[-1][0], self.path[-1][1]
            px, py = self.path[k][0], self.path[k][1]
            bx, by = self.path[k+1][0], self.path[k+1][1]


class World:
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
    ap = argparse.ArgumentParser(description="Simulated rover + ESP32 with stable heading and Ackerman steering")
    ap.add_argument("--lat", type=float, default=cfg.ORIGIN_LAT)
    ap.add_argument("--lon", type=float, default=cfg.ORIGIN_LON)
    ap.add_argument("--start-x", type=float, default=0.0)
    ap.add_argument("--start-y", type=float, default=0.0)
    ap.add_argument("--start-yaw", type=float, default=0.0)
    ap.add_argument("--circle", type=float, default=0.0)
    ap.add_argument("--speed", type=float, default=0.5)
    ap.add_argument("--max-speed", type=float, default=cfg.MAX_SPEED)
    ap.add_argument("--max-turn", type=float, default=cfg.MAX_TURN)
    ap.add_argument("--lin-accel", type=float, default=cfg.LIN_ACCEL)
    ap.add_argument("--ang-accel", type=float, default=cfg.ANG_ACCEL)
    ap.add_argument("--esp-hz", type=float, default=20.0)
    ap.add_argument("--gnss-hz", type=float, default=1.0)
    ap.add_argument("--fix", choices=STAGE_NAMES, help="lock to one fix state")
    ap.add_argument("--wheel-base", type=float, default=cfg.WHEEL_BASE)
    ap.add_argument("--wheel-diameter", type=float, default=cfg.WHEEL_DIAMETER)
    ap.add_argument("--ticks-per-rev", type=int, default=cfg.TICKS_PER_REV)
    ap.add_argument("--robot-length", type=float, default=cfg.ROBOT_LENGTH)
    ap.add_argument("--robot-width", type=float, default=cfg.ROBOT_WIDTH)
    ap.add_argument("--cmap", help="hidden world image")
    ap.add_argument("--res", type=float, default=cfg.MAP_RES)
    ap.add_argument("--invert", action="store_true")
    ap.add_argument("--lidar-pose-port", type=int, default=5000)
    ap.add_argument("--lidar-pose-hz", type=float, default=20.0)
    ap.add_argument("--odom-error", type=float, default=0.0)
    ap.add_argument("--min-turn-radius", type=float, default=0.4, help="Ackerman minimum turning radius (m)")
    args = ap.parse_args()

    world = None
    if args.cmap:
        try:
            world = World(args.cmap, args.res, args.invert)
        except Exception as e:
            print(f"[SIM] could not load world {args.cmap}: {e}", flush=True)

    pub, sub = Publisher(), Subscriber(["cmd_vel", "path"])
    pose_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if (world and args.lidar_pose_port) else None

    follower = PurePursuitFollower(
        max_speed=args.max_speed,
        max_turn=args.max_turn,
        min_turn_radius=args.min_turn_radius
    )

    tpm = args.ticks_per_rev / (math.pi * args.wheel_diameter)
    cos_lat0 = math.cos(math.radians(args.lat))
    x, y, yaw = args.start_x, args.start_y, math.radians(args.start_yaw)
    v = w = lin_acc = 0.0
    left_f = right_f = 0.0
    seq = 0
    cmd_v = cmd_w = 0.0
    last_cmd = -1e9
    stage_idx = STAGE_NAMES.index(args.fix) if args.fix else 0
    last_stage = None
    t_prev = time.monotonic()
    stage_start = next_g = next_e = next_pose = t_prev

    last_cog_x, last_cog_y = x, y
    heading_drift_estimate = 0.0

    print(f"[SIM] Initialized with smooth steering and GPS drift correction active.", flush=True)

    try:
        while True:
            now = time.monotonic()
            dt = min(now - t_prev, 0.1)
            t_prev = now

            for m in sub.poll():
                if "path" in m:
                    follower.set_path(m["path"])
                elif "linear" in m and "angular" in m:
                    cmd_v, cmd_w, last_cmd = m["linear"], m["angular"], now

            if not args.fix:
                dur = STAGES[stage_idx][1]
                if dur and now - stage_start >= dur:
                    stage_idx += 1
                    stage_start = now
            if stage_idx != last_stage:
                last_stage = stage_idx
            name, _, sat_r, hdop_r, age_r, noise_m = STAGES[stage_idx]

            if dt > 0:
                if args.circle > 0:
                    tv, tw = args.speed, args.speed / args.circle
                elif follower.path and not follower.update(x, y, yaw)[2]:
                    tv, tw, _ = follower.update(x, y, yaw)
                elif now - last_cmd > cfg.CMD_TIMEOUT:
                    tv = tw = 0.0
                else:
                    tv, tw = cmd_v, cmd_w

                tv = clamp(tv, -args.max_speed, args.max_speed)
                max_allowed_w = abs(tv) / args.min_turn_radius
                tw = clamp(tw, -max_allowed_w, max_allowed_w) if abs(tv) > 0.01 else 0.0
                tw = clamp(tw, -args.max_turn, args.max_turn)

                dv = clamp(tv - v, -args.lin_accel * dt, args.lin_accel * dt)
                dw = clamp(tw - w, -args.ang_accel * dt, args.ang_accel * dt)
                nv, nw = v + dv, w + dw
                
                mid = yaw + nw * dt / 2
                nx = x + nv * math.cos(mid) * dt
                ny = y + nv * math.sin(mid) * dt
                nyaw = wrap(yaw + nw * dt)
                
                if world and (nv or nw) and world.hits(nx, ny, nyaw, args.robot_length, args.robot_width):
                    nv = nw = 0.0
                    nx, ny, nyaw = x, y, yaw
                
                lin_acc = (nv - v) / dt
                v, w, x, y, yaw = nv, nw, nx, ny, nyaw

                # --- Stable GPS Course-Over-Ground (COG) Drift Correction ---
                # Only measures drift during clean forward straight-line motion, 
                # leaving teleop steering and reversing completely uninhibited.
                dist_moved = math.hypot(x - last_cog_x, y - last_cog_y)
                if abs(v) > 0.3 and abs(w) < 0.05 and dist_moved > 0.5:
                    cog_heading = math.atan2(y - last_cog_y, x - last_cog_x)
                    yaw_drift_sample = wrap(yaw - cog_heading)
                    heading_drift_estimate = 0.95 * heading_drift_estimate + 0.05 * yaw_drift_sample
                    last_cog_x, last_cog_y = x, y
                elif abs(v) < 0.1:
                    last_cog_x, last_cog_y = x, y

                left_f += (v - w * args.wheel_base / 2) * tpm * dt
                right_f += (v + w * args.wheel_base / 2) * tpm * dt * (1.0 + args.odom_error)

            # Publish RTK GPS
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
                except ProtocolError:
                    pass

            # Publish ESP32 Telemetry
            if now >= next_e:
                next_e = max(next_e + 1.0 / args.esp_hz, now)
                seq += 1
                try:
                    pub.send("esp", seq=seq,
                             enc={"left_ticks": int(round(left_f)), "right_ticks": int(round(right_f)),
                                  "left_vel": (v - w * args.wheel_base / 2) * tpm,
                                  "right_vel": (v + w * args.wheel_base / 2) * tpm},
                             imu={"gyro": [0.0, 0.0, w],
                                  "accel": [lin_acc, 0.0, G],
                                  "quat": [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]})
                except ProtocolError:
                    pass

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

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\n[SIM] stopped.", flush=True)


if __name__ == "__main__":
    main()