#!/usr/bin/env python3
"""
Robot visualiser + keyboard driver. Shows the robot on a costmap, live.

Two robots are drawn:
  GOAL   (solid blue)   a smooth, high-frame-rate robot that goes exactly where you drive it.
                        It integrates your commanded velocity (arrow keys) with the same
                        acceleration limits as the simulator, so it is the teleop goal pose.
  GHOST  (transparent)  where the TELEMETRY says the robot actually is (GNSS when it has a
                        fix, otherwise wheel odometry). It is smoothed + extrapolated between
                        updates so it glides, and it chases the goal robot. The gap between
                        them is the tracking error / latency.

By default it LAUNCHES the simulator (ugv/sims/sim_robot.py). That one process writes both
the GNSS file and the ESP32 file from a single robot state, so they stay in sync.

Reads (same files your program reads):
  ugv/rtk/rover_pos.dat   GNSS position + fix state
  ugv/esp/sim_esp.dat     encoders + IMU
Writes (arrow keys):
  ugv/esp/cmd_vel.dat     {"linear": m/s, "angular": rad/s, "ts": unix time}

Layout: ugv/viz/visualiser.py

  pip install matplotlib numpy pillow
  python visualiser.py                       # launch sim + drive with arrows
  python visualiser.py --circle 2            # sim drives a 2 m circle by itself
  python visualiser.py --fix "Fixed RTK"     # skip the fix warm-up
  python visualiser.py --no-sim              # don't launch a sim (real hardware / sim already running)
  python visualiser.py --no-drive            # arrow keys do nothing (goal follows cmd_vel.dat instead)
  python visualiser.py --cmap ../maps/cmap.png --invert

Keys (click the window first so it has focus):
  up / down      forward / reverse        left / right   turn left / right (CCW / CW)
  space          stop                     r  snap goal robot to the telemetry ghost
  f              follow robot / whole map c  clear trails      o  re-anchor odometry to GNSS     q  quit

Map placement: the CENTRE of cmap.png is the sim's origin (--ref-lat/--ref-lon, default
-31.9505, 115.8605). The robot starts there unless you use --start-x / --start-y (metres
east / north of the centre) and --start-yaw (degrees, 0 = east, CCW positive).
With --no-sim and no --ref-lat/--ref-lon, the centre is anchored to the first GNSS fix.
"""

import argparse
import json
import math
import mmap
import os
import subprocess
import sys
import time
from collections import deque

try:
    import numpy as np
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Polygon
    from PIL import Image
except ImportError as e:
    sys.exit(f"missing dependency ({e}). Run: pip install matplotlib numpy pillow")

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROVER = os.path.normpath(os.path.join(HERE,  "rtk", "rover_pos.dat"))
DEFAULT_ESP = os.path.normpath(os.path.join(HERE,  "esp", "sim_esp.dat"))
DEFAULT_CMD = os.path.normpath(os.path.join(HERE,  "esp", "cmd_vel.dat"))
DEFAULT_SIM = os.path.normpath(os.path.join(HERE,  "sims", "sim_robot.py"))
CMAP_SEARCH = [
    os.path.join(HERE, "cmap.png"),
    os.path.normpath(os.path.join(HERE, "..", "cmap.png")),
    os.path.join(os.getcwd(), "cmap.png"),
]
DEFAULT_LAT, DEFAULT_LON = -31.9505, 115.8605

M_PER_DEG = 111320.0   # same constant the sim uses
GNSS_STALE_S = 3.0
ESP_STALE_S = 1.5
CMD_TIMEOUT_S = 0.5
CMD_SIZE = 128
ARROWS = ("up", "down", "left", "right")

FIX_COLORS = {
    "Fixed RTK": "#2ecc71",
    "Float RTK": "#f1c40f",
    "DGNSS": "#3498db",
    "3D Fix": "#e67e22",
    "No Fix": "#e74c3c",
}
WHITE, GREY, RED, GREEN, HDR = "#e8e8e8", "#8a8a8a", "#ff5555", "#2ecc71", "#58a6ff"
GOAL_COLOR, GHOST_COLOR = "#4aa3ff", "#ffb347"


def read_dat(path):
    """Read a space-padded JSON shared file. Returns dict or None (missing / mid-write)."""
    try:
        with open(path, "rb") as f:
            raw = f.read()
        txt = raw.decode("utf-8", "ignore").replace("\x00", "").strip()
        return json.loads(txt) if txt else None
    except (OSError, ValueError):
        return None


def write_cmd(path, lin, ang):
    data = json.dumps({"linear": round(lin, 3), "angular": round(ang, 3), "ts": time.time()}).ljust(CMD_SIZE)
    data = data.encode("utf-8")
    try:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        if not os.path.exists(path) or os.path.getsize(path) < CMD_SIZE:
            with open(path, "wb") as f:
                f.write(b" " * CMD_SIZE)
        with open(path, "r+b") as f:
            mm = mmap.mmap(f.fileno(), CMD_SIZE)
            mm.seek(0)
            mm.write(data)
            mm.close()
    except OSError as e:
        print(f"[CMD ERROR] could not write {path}: {e}")


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def launch_sim(args):
    """Start sim_robot.py as a child process. Returns the Popen or None."""
    if not os.path.exists(args.sim_script):
        print(f"[SIM] {args.sim_script} not found - not launching a simulator (use --sim-script).")
        return None
    for p in (args.rover_file, args.esp_file, args.cmd_file):   # drop stale data from earlier runs
        try:
            os.remove(p)
        except OSError:
            pass
    cmd = [
        sys.executable, args.sim_script,
        "--lat", str(args.ref_lat), "--lon", str(args.ref_lon),
        "--start-x", str(args.start_x), "--start-y", str(args.start_y), "--start-yaw", str(args.start_yaw),
        "--circle", str(args.circle), "--speed", str(args.circle_speed),
        "--lin-accel", str(args.lin_accel), "--ang-accel", str(args.ang_accel),
        "--wheel-base", str(args.wheel_base), "--wheel-diameter", str(args.wheel_diameter),
        "--ticks-per-rev", str(args.ticks_per_rev),
        "--rtk-file", args.rover_file, "--esp-file", args.esp_file, "--cmd-file", args.cmd_file,
    ]
    if args.fix:
        cmd += ["--fix", args.fix]
    print("[SIM] launching:", " ".join(cmd))
    return subprocess.Popen(cmd)


def stop_sim(proc):
    if proc is not None and proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()


class Visualiser:
    def __init__(self, args, sim_proc):
        self.a = args
        self.sim = sim_proc
        self.ticks_per_m = args.ticks_per_rev / (math.pi * args.wheel_diameter)
        self.ref = (args.ref_lat, args.ref_lon) if args.ref_lat is not None and args.ref_lon is not None else None
        # where odometry starts until GNSS gives us a position to anchor to
        self.odom_origin = [args.start_x, args.start_y] if sim_proc is not None else [0.0, 0.0]

        # costmap: cost 0 (free) .. 255 (lethal)
        self.cost = None
        path = args.cmap or next((p for p in CMAP_SEARCH if os.path.exists(p)), None)
        if path and os.path.exists(path):
            gray = np.asarray(Image.open(path).convert("L"), dtype=np.uint8)
            self.cost = gray.copy() if args.invert else (255 - gray)
            h, w = self.cost.shape
            self.left, self.top = -w * args.res / 2, h * args.res / 2
            self.extent = (self.left, -self.left, -self.top, self.top)
            print(f"Costmap: {path}  {w}x{h}px = {w * args.res:.1f} x {h * args.res:.1f} m")
        else:
            print("Costmap: cmap.png not found (searched next to this script, ../, and cwd) - showing blank grid. Use --cmap.")
            self.extent = (-10, 10, -10, 10)

        # telemetry state
        self.gpos = None
        self.gfix = "No Fix"
        self.last_rts = None
        self.odom = None
        self.odom_anchored = False
        self.prev = None
        self.last_seq = None
        self.o_yaw = math.radians(args.start_yaw) if sim_proc is not None else 0.0
        self.have_yaw = False
        self.g_t = self.o_t = self.e_t = 0.0      # monotonic time of last GNSS / odom / ESP update
        self.meas_v = self.meas_w = 0.0           # measured speed (m/s) and yaw rate (rad/s)
        self.g_trail = deque(maxlen=args.trail)
        self.o_trail = deque(maxlen=args.trail)
        self.follow = True

        # goal robot (smooth, from commands) and telemetry ghost
        self.goal = [args.start_x, args.start_y, math.radians(args.start_yaw)] if sim_proc is not None else None
        self.gv = self.gw = 0.0
        self.goal_trail = deque(maxlen=args.trail)
        self.ghost = None
        self.t_last = time.monotonic()
        self._panel_t = 0.0

        # keyboard control
        self.keys = set()
        self.released = {}
        self.cmd = (0.0, 0.0)
        self.cmd_active = False

        self._build_figure()

    # ---------- figure ----------
    def _footprint(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        hl, hw = self.a.robot_length / 2, self.a.robot_width / 2
        pts = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
        return [(x + px * c - py * s, y + px * s + py * c) for px, py in pts]

    def _heading_pts(self, x, y, yaw):
        reach = self.a.robot_length / 2 + 0.15
        return [x, x + reach * math.cos(yaw)], [y, y + reach * math.sin(yaw)]

    def _build_figure(self):
        plt.style.use("dark_background")
        # free up matplotlib's default shortcuts (arrows, f, c, o, ...) - keep only quit
        for k in list(plt.rcParams.keys()):
            if k.startswith("keymap.") and k != "keymap.quit":
                plt.rcParams[k] = []
        self.fig = plt.figure(figsize=(13, 8))
        try:
            self.fig.canvas.manager.set_window_title("UGV visualiser")
        except Exception:
            pass
        self.ax = self.fig.add_axes([0.06, 0.07, 0.62, 0.88])
        ax = self.ax
        if self.cost is not None:
            ax.imshow(255 - self.cost, extent=self.extent, origin="upper", cmap="gray",
                      vmin=0, vmax=255, interpolation="nearest", zorder=0)
            l, r, b, t = self.extent
            ax.plot([l, r, r, l, l], [b, b, t, t, b], color="#555", lw=1, zorder=1)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.25, zorder=1)
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_title("Goal (solid) vs telemetry ghost (transparent)   |   white = free, black = obstacle", fontsize=10)

        self.goal_line, = ax.plot([], [], ":", color="#ffffff", lw=1.2, alpha=0.7, label="Goal path", zorder=3)
        self.g_line, = ax.plot([], [], "-", color="#2ecc71", lw=1.3, alpha=0.7, label="GNSS trail", zorder=3)
        self.o_line, = ax.plot([], [], "--", color="#e056fd", lw=1.3, alpha=0.8, label="Odometry trail", zorder=3)
        self.g_dot, = ax.plot([], [], "o", ms=6, mec="w", mew=0.8, zorder=5, label="Raw GNSS")
        self.o_dot, = ax.plot([], [], "D", ms=5, color="#e056fd", mec="w", mew=0.8, zorder=5, label="Raw odometry")

        self.err_line, = ax.plot([], [], "-", color="#ffe066", lw=1.2, alpha=0.9, zorder=5)
        self.ghost_body = Polygon(np.zeros((4, 2)), closed=True, fc=GHOST_COLOR, ec=GHOST_COLOR, alpha=0.35,
                                  lw=1.5, ls="--", zorder=6, label="Telemetry ghost")
        self.ghost_body.set_visible(False)
        ax.add_patch(self.ghost_body)
        self.ghost_head, = ax.plot([], [], "-", color=GHOST_COLOR, lw=2, alpha=0.8, zorder=7)

        self.goal_body = Polygon(np.zeros((4, 2)), closed=True, fc=GOAL_COLOR, ec="w", alpha=0.85,
                                 lw=1.5, zorder=8, label="Goal (smooth, teleop)")
        self.goal_body.set_visible(False)
        ax.add_patch(self.goal_body)
        self.goal_head, = ax.plot([], [], "-", color="w", lw=2.5, zorder=9)

        ax.legend(loc="upper right", fontsize=8, framealpha=0.5)
        ax.set_xlim(-6, 6)
        ax.set_ylim(-6, 6)

        panel = self.fig.add_axes([0.70, 0.05, 0.29, 0.90])
        panel.axis("off")
        self.rows = [panel.text(0.0, 1 - i * 0.0245, "", family="monospace", fontsize=9, va="top",
                                transform=panel.transAxes, color=WHITE) for i in range(40)]
        self.fig.canvas.mpl_connect("key_press_event", self._on_press)
        self.fig.canvas.mpl_connect("key_release_event", self._on_release)

        # command publisher runs on its own 20 Hz timer, independent of redraw rate
        self.timer = self.fig.canvas.new_timer(interval=50)
        self.timer.add_callback(self._control_tick)
        self.timer.start()

    # ---------- keyboard ----------
    def _on_press(self, ev):
        k = ev.key
        if k in ARROWS:
            self.keys.add(k)
            self.released.pop(k, None)
        elif k == " ":
            self.keys.clear()
            self.released.clear()
        elif k == "c":
            self.g_trail.clear()
            self.o_trail.clear()
            self.goal_trail.clear()
        elif k == "f":
            self.follow = not self.follow
        elif k == "o":
            if self.gpos is not None and self.odom is not None:
                self.odom = list(self.gpos)
                self.o_trail.clear()
        elif k == "r":
            if self.ghost is not None:
                self.goal = list(self.ghost)
                self.gv = self.gw = 0.0
                self.goal_trail.clear()

    def _on_release(self, ev):
        if ev.key in ARROWS:
            # deferred: some platforms send release+press pairs while a key auto-repeats
            self.released[ev.key] = time.monotonic()

    def _control_tick(self):
        if not self.a.drive:
            return
        now = time.monotonic()
        for k, t in list(self.released.items()):
            if now - t > 0.06:
                self.keys.discard(k)
                del self.released[k]
        lin = (("up" in self.keys) - ("down" in self.keys)) * self.a.max_speed
        ang = (("left" in self.keys) - ("right" in self.keys)) * self.a.max_turn
        active = bool(lin or ang)
        if active or self.cmd_active:       # keep sending while driving, plus one final zero
            write_cmd(self.a.cmd_file, lin, ang)
        self.cmd_active = active
        self.cmd = (lin, ang)

    def current_cmd(self):
        """Velocity command the goal robot should follow: (linear m/s, angular rad/s)."""
        if self.sim is not None and self.a.circle > 0:
            return self.a.circle_speed, self.a.circle_speed / self.a.circle
        if self.a.drive:
            return self.cmd
        d = read_dat(self.a.cmd_file)          # external program is driving: follow its cmd_vel.dat
        if d and time.time() - float(d.get("ts", 0)) <= CMD_TIMEOUT_S:
            return float(d.get("linear", 0.0)), float(d.get("angular", 0.0))
        return 0.0, 0.0

    # ---------- data ----------
    def to_local(self, lat, lon):
        lat0, lon0 = self.ref
        return ((lon - lon0) * M_PER_DEG * math.cos(math.radians(lat0)), (lat - lat0) * M_PER_DEG)

    def process_rover(self, rv):
        fix = rv.get("fix", "No Fix")
        lat, lon = rv.get("lat", 0.0), rv.get("lon", 0.0)
        self.gfix = fix
        if fix == "No Fix" or (lat == 0 and lon == 0):
            self.gpos = None
            return
        if self.ref is None:
            self.ref = (lat, lon)
            print(f"Map centre anchored at first fix: {lat:.7f}, {lon:.7f}")
        self.gpos = self.to_local(lat, lon)
        ts = rv.get("ts")
        if ts != self.last_rts:
            self.last_rts = ts
            self.g_t = time.monotonic()
            self.g_trail.append(self.gpos)

    def process_esp(self, es):
        seq = es.get("seq")
        if seq is None or seq == self.last_seq:
            return
        if self.last_seq is not None and seq < self.last_seq:   # sender restarted
            self.prev = None
        self.last_seq = seq
        enc = es.get("enc", {})
        L, R = enc.get("left_ticks"), enc.get("right_ticks")
        if L is None or R is None:
            return
        imu = es.get("imu", {})
        q = imu.get("quat")
        yaw = yaw_from_quat(q) if q and len(q) == 4 else None
        if yaw is not None:
            self.have_yaw = True
        self.e_t = time.monotonic()
        self.meas_v = ((enc.get("left_vel", 0.0) + enc.get("right_vel", 0.0)) / 2) / self.ticks_per_m
        gyro = imu.get("gyro")
        self.meas_w = gyro[2] if gyro and len(gyro) == 3 else 0.0

        if self.prev is None:
            self.prev = (L, R, yaw)
            if self.odom is None:
                self.odom = list(self.odom_origin)
            if yaw is not None:
                self.o_yaw = yaw
            self.o_t = self.e_t
            return

        pl, pr, pyaw = self.prev
        dl, dr = (L - pl) / self.ticks_per_m, (R - pr) / self.ticks_per_m
        d = (dl + dr) / 2
        if yaw is not None:
            base = pyaw if pyaw is not None else yaw
            mid = base + wrap(yaw - base) / 2
            self.o_yaw = yaw
        else:
            dth = (dr - dl) / self.a.wheel_base
            mid = self.o_yaw + dth / 2
            self.o_yaw += dth
        self.odom[0] += d * math.cos(mid)
        self.odom[1] += d * math.sin(mid)
        self.prev = (L, R, yaw)
        self.o_t = self.e_t
        self.o_trail.append(tuple(self.odom))

    def cost_at(self, x, y):
        if self.cost is None:
            return None
        col = int((x - self.left) / self.a.res)
        row = int((self.top - y) / self.a.res)
        h, w = self.cost.shape
        if 0 <= row < h and 0 <= col < w:
            return float(self.cost[row, col]) / 255 * 100
        return -1.0   # off map

    # ---------- per-frame models ----------
    def telemetry_target(self, now_m, g_fresh):
        """Best telemetry pose (x, y, yaw), extrapolated forward from the last sample using the
        measured speed and yaw rate so it moves continuously between 4-5 Hz updates."""
        if g_fresh and self.gpos is not None:
            bx, by = self.gpos
            t0 = self.g_t
        elif self.odom is not None:
            bx, by = self.odom
            t0 = self.o_t
        else:
            return None
        ey = clamp(now_m - self.e_t, 0.0, 0.5) if self.e_t else 0.0
        yaw = self.o_yaw + self.meas_w * ey
        ex = clamp(now_m - t0, 0.0, 0.5)
        return bx + self.meas_v * ex * math.cos(yaw), by + self.meas_v * ex * math.sin(yaw), yaw

    def step_models(self, dt, now_m, g_fresh):
        # ghost: low-pass toward the telemetry target
        tgt = self.telemetry_target(now_m, g_fresh)
        if tgt is not None:
            if self.ghost is None:
                self.ghost = list(tgt)
            else:
                k = 1.0 - math.exp(-dt / max(self.a.ghost_tau, 1e-3))
                self.ghost[0] += (tgt[0] - self.ghost[0]) * k
                self.ghost[1] += (tgt[1] - self.ghost[1]) * k
                self.ghost[2] = wrap(self.ghost[2] + wrap(tgt[2] - self.ghost[2]) * k)
        # goal: first appearance snaps to the ghost when there is no known start pose
        if self.goal is None and self.ghost is not None:
            self.goal = list(self.ghost)
        if self.goal is None:
            return
        cv, cw = self.current_cmd()
        lv, lw = self.a.lin_accel * dt, self.a.ang_accel * dt
        self.gv += clamp(cv - self.gv, -lv, lv)
        self.gw += clamp(cw - self.gw, -lw, lw)
        mid = self.goal[2] + self.gw * dt / 2
        self.goal[0] += self.gv * math.cos(mid) * dt
        self.goal[1] += self.gv * math.sin(mid) * dt
        self.goal[2] = wrap(self.goal[2] + self.gw * dt)
        if not self.goal_trail or math.hypot(self.goal[0] - self.goal_trail[-1][0],
                                             self.goal[1] - self.goal_trail[-1][1]) > 0.02:
            self.goal_trail.append((self.goal[0], self.goal[1]))

    # ---------- frame ----------
    def update(self, _frame):
        now = time.time()
        now_m = time.monotonic()
        dt = clamp(now_m - self.t_last, 0.0, 0.1)
        self.t_last = now_m

        rv = read_dat(self.a.rover_file)
        es = read_dat(self.a.esp_file)
        if rv:
            self.process_rover(rv)
        if es:
            self.process_esp(es)

        # anchor odometry to GNSS the first time both exist
        if self.odom is not None and not self.odom_anchored and self.gpos is not None:
            self.odom = list(self.gpos)
            self.o_trail.clear()
            self.o_t = now_m
            self.odom_anchored = True

        g_age = (now - rv["ts"]) if rv and "ts" in rv else None
        e_age = (now - es["ts"]) if es and "ts" in es else None
        g_fresh = self.gpos is not None and g_age is not None and g_age <= GNSS_STALE_S

        self.step_models(dt, now_m, g_fresh)

        # trails + raw markers
        for line, tr in ((self.g_line, self.g_trail), (self.o_line, self.o_trail), (self.goal_line, self.goal_trail)):
            if tr:
                xs, ys = zip(*tr)
                line.set_data(xs, ys)
            else:
                line.set_data([], [])
        if self.gpos is not None:
            self.g_dot.set_data([self.gpos[0]], [self.gpos[1]])
            self.g_dot.set_color(FIX_COLORS.get(self.gfix, "#95a5a6"))
        else:
            self.g_dot.set_data([], [])
        if self.odom is not None:
            self.o_dot.set_data([self.odom[0]], [self.odom[1]])
        else:
            self.o_dot.set_data([], [])

        # ghost
        if self.ghost is not None:
            gx, gy, gyaw = self.ghost
            self.ghost_body.set_xy(self._footprint(gx, gy, gyaw))
            self.ghost_body.set_visible(True)
            hx, hy = self._heading_pts(gx, gy, gyaw)
            self.ghost_head.set_data(hx, hy)
        else:
            self.ghost_body.set_visible(False)
            self.ghost_head.set_data([], [])

        # goal
        if self.goal is not None:
            x, y, yaw = self.goal
            self.goal_body.set_xy(self._footprint(x, y, yaw))
            self.goal_body.set_visible(True)
            hx, hy = self._heading_pts(x, y, yaw)
            self.goal_head.set_data(hx, hy)
            if self.ghost is not None:
                self.err_line.set_data([x, self.ghost[0]], [y, self.ghost[1]])
            else:
                self.err_line.set_data([], [])
        else:
            self.goal_body.set_visible(False)
            self.goal_head.set_data([], [])
            self.err_line.set_data([], [])

        # camera follows the smooth goal robot (falls back to the ghost)
        focus = self.goal if self.goal is not None else self.ghost
        if self.follow and focus is not None:
            half = self.a.view / 2
            self.ax.set_xlim(focus[0] - half, focus[0] + half)
            self.ax.set_ylim(focus[1] - half, focus[1] + half)
        elif not self.follow:
            l, r, b, t = self.extent
            m = 0.5
            self.ax.set_xlim(l - m, r + m)
            self.ax.set_ylim(b - m, t + m)

        if now_m - self._panel_t >= 0.1:        # text panel at 10 Hz is plenty
            self._panel_t = now_m
            self._draw_panel(rv, es, g_age, e_age)
        return []

    @staticmethod
    def _cost_txt(c):
        if c is None:
            return "n/a", GREY
        if c < 0:
            return "OFF MAP", RED
        return (f"{c:.0f}%" + (" OCCUPIED" if c > 50 else "")), (RED if c > 50 else WHITE)

    def _draw_panel(self, rv, es, g_age, e_age):
        rows = [("GNSS  (rover_pos.dat)", HDR)]
        if not rv:
            rows.append(("  NO DATA", RED))
        else:
            stale = g_age is not None and g_age > GNSS_STALE_S
            rows += [
                (f"  fix    {self.gfix}", FIX_COLORS.get(self.gfix, WHITE)),
                (f"  sats   {rv.get('sats', '?')}    hdop {rv.get('hdop', '?')}", WHITE),
                (f"  corr   {rv.get('corr_age', '?')} s", WHITE),
                (f"  lat    {rv.get('lat', 0):.7f}", WHITE),
                (f"  lon    {rv.get('lon', 0):.7f}", WHITE),
                (f"  age    {g_age:.1f} s{'  STALE' if stale else ''}" if g_age is not None else "  age    ?",
                 RED if stale else GREY),
            ]

        rows += [("", WHITE), ("POSE  (m from map centre)", HDR)]
        if self.goal is not None:
            rows.append((f"  GOAL   x={self.goal[0]:+8.2f}  y={self.goal[1]:+8.2f}", GOAL_COLOR))
        else:
            rows.append(("  GOAL   waiting for telemetry", GREY))
        if self.ghost is not None:
            rows.append((f"  GHOST  x={self.ghost[0]:+8.2f}  y={self.ghost[1]:+8.2f}", GHOST_COLOR))
            if self.goal is not None:
                err = math.hypot(self.goal[0] - self.ghost[0], self.goal[1] - self.ghost[1])
                rows.append((f"  track  {err:.2f} m behind goal" + ("  [r] resync" if err > 1.0 else ""),
                             RED if err > 1.0 else WHITE))
        else:
            rows.append(("  GHOST  no telemetry", GREY))
        if self.gpos is not None:
            rows.append((f"  GNSS   x={self.gpos[0]:+8.2f}  y={self.gpos[1]:+8.2f}", WHITE))
        else:
            rows.append(("  GNSS   no position", GREY))
        if self.odom is not None:
            rows.append((f"  ODOM   x={self.odom[0]:+8.2f}  y={self.odom[1]:+8.2f}", WHITE))
            if self.gpos is not None:
                drift = math.hypot(self.odom[0] - self.gpos[0], self.odom[1] - self.gpos[1])
                rows.append((f"  drift  {drift:.2f} m (odom vs GNSS)", WHITE))
        else:
            rows.append(("  ODOM   no data", GREY))
        gy = math.degrees(self.goal[2]) if self.goal is not None else float("nan")
        rows.append((f"  yaw    goal {gy:+6.1f}  tel {math.degrees(self.o_yaw):+6.1f}"
                     + ("" if self.have_yaw else " (enc)"), WHITE))
        if self.ghost is not None:
            txt, col = self._cost_txt(self.cost_at(self.ghost[0], self.ghost[1]))
            rows.append((f"  cost   robot {txt}", col))
        if self.goal is not None:
            txt, col = self._cost_txt(self.cost_at(self.goal[0], self.goal[1]))
            rows.append((f"  cost   goal  {txt}", col))

        rows += [("", WHITE), ("ESP32  (sim_esp.dat)", HDR)]
        if not es:
            rows.append(("  NO DATA", RED))
        else:
            enc, imu = es.get("enc", {}), es.get("imu", {})
            gyro, acc = imu.get("gyro", [0, 0, 0]), imu.get("accel", [0, 0, 0])
            stale = e_age is not None and e_age > ESP_STALE_S
            rows += [
                (f"  seq    {es.get('seq', '?')}", WHITE),
                (f"  L ticks {enc.get('left_ticks', '?')}", WHITE),
                (f"  R ticks {enc.get('right_ticks', '?')}", WHITE),
                (f"  speed  {self.meas_v:+.2f} m/s", WHITE),
                (f"  gyro z {gyro[2]:+.3f} rad/s", WHITE),
                (f"  accel  {acc[0]:+.2f} {acc[1]:+.2f} {acc[2]:+.2f}", WHITE),
                (f"  age    {e_age:.1f} s{'  STALE' if stale else ''}" if e_age is not None else "  age    ?",
                 RED if stale else GREY),
            ]

        rows += [("", WHITE), ("COMMAND", HDR)]
        if self.a.circle > 0 and self.sim is not None:
            src = f"sim autopilot (circle r={self.a.circle} m)"
        elif self.a.drive:
            src = "arrow keys"
        else:
            src = "external cmd_vel.dat (--no-drive)"
        cv, cw = self.current_cmd()
        rows.append((f"  src    {src}", WHITE))
        rows.append((f"  cmd    v={cv:+.2f} m/s  w={cw:+.2f} rad/s", GREEN if (cv or cw) else WHITE))
        rows.append((f"  goal   v={self.gv:+.2f} m/s  w={self.gw:+.2f} rad/s", GOAL_COLOR))
        if self.sim is None:
            sim_txt, sim_col = "not launched", GREY
        elif self.sim.poll() is None:
            sim_txt, sim_col = f"running (pid {self.sim.pid})", WHITE
        else:
            sim_txt, sim_col = f"EXITED (code {self.sim.returncode})", RED
        rows.append((f"  sim    {sim_txt}", sim_col))

        rows += [("", WHITE),
                 ("arrows drive   space stop", GREY),
                 ("[r] snap goal to ghost  [f] follow/full", GREY),
                 ("[c] clear trails  [o] re-anchor odom", GREY)]
        for t, (txt, col) in zip(self.rows, rows + [("", WHITE)] * len(self.rows)):
            t.set_text(txt)
            t.set_color(col)


def main():
    ap = argparse.ArgumentParser(description="Live robot + costmap visualiser with keyboard driving")
    ap.add_argument("--rover-file", default=DEFAULT_ROVER, help="GNSS shared file (default ../rtk/rover_pos.dat)")
    ap.add_argument("--esp-file", default=DEFAULT_ESP, help="ESP32 shared file (default ../esp/sim_esp.dat)")
    ap.add_argument("--cmd-file", default=DEFAULT_CMD, help="velocity command file (default ../esp/cmd_vel.dat)")
    ap.add_argument("--sim-script", default=DEFAULT_SIM, help="simulator to launch (default ../sims/sim_robot.py)")
    ap.add_argument("--no-sim", action="store_true", help="don't launch the simulator")
    ap.add_argument("--no-drive", action="store_true", help="arrow keys don't write cmd_vel.dat")
    ap.add_argument("--circle", type=float, default=0.0, help="sim autopilot: circle of this radius (m)")
    ap.add_argument("--circle-speed", type=float, default=0.5, help="autopilot speed, m/s")
    ap.add_argument("--max-speed", type=float, default=1.0, help="arrow-key forward speed, m/s (default 1.0)")
    ap.add_argument("--max-turn", type=float, default=1.2, help="arrow-key turn rate, rad/s (default 1.2)")
    ap.add_argument("--lin-accel", type=float, default=1.0, help="goal robot + sim max linear accel, m/s^2")
    ap.add_argument("--ang-accel", type=float, default=3.0, help="goal robot + sim max angular accel, rad/s^2")
    ap.add_argument("--ghost-tau", type=float, default=0.15,
                    help="ghost smoothing time constant, s (lower = snappier, higher = smoother; default 0.15)")
    ap.add_argument("--start-x", type=float, default=0.0, help="sim start, m east of map centre")
    ap.add_argument("--start-y", type=float, default=0.0, help="sim start, m north of map centre")
    ap.add_argument("--start-yaw", type=float, default=0.0, help="sim start heading, deg (0 = east, CCW +)")
    ap.add_argument("--fix", choices=["No Fix", "3D Fix", "DGNSS", "Float RTK", "Fixed RTK"],
                    help="sim: lock to one fix state (default: realistic warm-up)")
    ap.add_argument("--cmap", help="costmap PNG (default: cmap.png next to this script, ../, or cwd)")
    ap.add_argument("--res", type=float, default=0.05, help="metres per costmap pixel (default 0.05 = 50 mm)")
    ap.add_argument("--invert", action="store_true", help="costmap is black = free, white = obstacle")
    ap.add_argument("--ref-lat", type=float, help="latitude at the map centre")
    ap.add_argument("--ref-lon", type=float, help="longitude at the map centre")
    ap.add_argument("--view", type=float, default=12.0, help="follow-view window size in m (default 12)")
    ap.add_argument("--trail", type=int, default=2000, help="max trail points (default 2000)")
    ap.add_argument("--fps", type=float, default=30.0, help="redraw rate (default 30; lower it if the window lags)")
    ap.add_argument("--robot-length", type=float, default=0.45, help="footprint length, m")
    ap.add_argument("--robot-width", type=float, default=0.35, help="footprint width, m")
    # must match the robot (the sim is launched with these too)
    ap.add_argument("--wheel-base", type=float, default=0.30)
    ap.add_argument("--wheel-diameter", type=float, default=0.15)
    ap.add_argument("--ticks-per-rev", type=int, default=1000)
    args = ap.parse_args()
    args.drive = not args.no_drive

    sim = None
    if not args.no_sim:
        # the sim's origin IS the map centre, so the robot starts where you expect on the costmap
        if args.ref_lat is None or args.ref_lon is None:
            args.ref_lat, args.ref_lon = DEFAULT_LAT, DEFAULT_LON
        sim = launch_sim(args)

    print(f"GNSS file: {args.rover_file}")
    print(f"ESP file : {args.esp_file}")
    print(f"CMD file : {args.cmd_file}")
    try:
        viz = Visualiser(args, sim)
        anim = FuncAnimation(viz.fig, viz.update, interval=int(1000 / args.fps), cache_frame_data=False)
        plt.show()
        del anim
    finally:
        stop_sim(sim)


if __name__ == "__main__":
    main()