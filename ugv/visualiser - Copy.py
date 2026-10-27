#!/usr/bin/env python3
"""
Robot visualiser, teleop and click-to-drive using Pygame. A pure client of the UDP bus (common/protocol.py):
it knows nothing about whether the robot is simulated or real. Start run_sim.py (simulation) OR your
real rover / ESP32 / lidar publishers, then start this.

  listens   rtk (5007)   position + fix          esp (5006)   encoders + IMU
            lidar (5005) scans                   cmd_vel (5006)  what is being commanded
  publishes cmd_vel      (arrow keys, or the navigator after you click the map)

What you see
  MAP     starts all unknown (grey). It is BUILT from the lidar scans, placed at the robot's estimated
          pose: white = free, black = wall. The robot only knows what its lidar has seen.
  GOAL    (solid blue) a smooth robot that follows the commands at the same speed limits as the real
          robot (common/config.py). It never waits for telemetry: it is where the robot SHOULD be.
  GHOST   (dashed orange) where the GNSS/IMU telemetry says the robot actually is (arrives ~1 Hz, glides).
  SCAN    red dots: the latest lidar scan, drawn around the robot.

Driving
  arrow keys     teleop (up/down = forward/back, left/right = turn)
  left click     on the map: plan a path to that point on the learned map and drive it (replans as the map
                 fills in; unknown space counts as free, so it explores towards the goal)
  right click    or  x / esc : stop and cancel.   Any arrow key also takes over from the navigator.
  space          PLAN A ROUTE. The map lights up FUCHSIA wherever the robot's centre can go without coming
                 within --inflate of a wall (seen-free space only). Left-click inside the fuchsia to queue
                 numbered waypoints, right-click undoes the last one, space again sends the whole route.
  [  and  ]      change the keep-out distance by one planning cell (0.1 m). The fuchsia updates live and the
                 new value is sent with every goal, so you can watch a narrow gap open up before you drive.
  Needs an RTK fix of at least --min-fix (default Float RTK) for mapping and click-to-drive.

Other keys   f follow robot / whole map     c clear trails     o re-anchor odometry     g snap goal to telemetry
             r reset the learned map        s save it (learned_map.png; black wall, white free, grey unknown)

  pip install pygame numpy pillow
  python visualiser.py                       # simulated or real, whatever is on the bus
  python visualiser.py --cmap sims/cmap.png --show-world     # also overlay the HIDDEN world (red) to check the map
  python visualiser.py --builtin-nav         # drive with the built-in planner
  python visualiser.py --no-drive            # view only: no keys, no clicks, never publishes

Layout: ugv/visualiser.py
"""

import argparse
import math
import os
import sys
import time
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

try:
    import numpy as np
    import pygame
    from PIL import Image
except ImportError as e:
    sys.exit(f"missing dependency ({e}). Run: pip install pygame numpy pillow")

from common import config as cfg                               # noqa: E402
from common.protocol import Publisher, Subscriber, FIX_RANK    # noqa: E402
from common.estimator import PoseEstimator                        # noqa: E402
from common.mapping import OccupancyGrid, FREE_T                  # noqa: E402
from common.localiser import Localiser                            # noqa: E402
from common.navigator import Navigator                            # noqa: E402
from common.planner import load_planner                           # noqa: E402

CMAP_SEARCH = [os.path.join(HERE, "cmap.png"),
               os.path.join(os.getcwd(), "cmap.png"),
               os.path.join(HERE, "sims", "cmap.png"),
               os.path.join(os.getcwd(), "sims", "cmap.png")]
M_PER_DEG = 111320.0
LIDAR_STALE_S = 1.0
ARROWS = (pygame.K_UP, pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT)

# Colors (RGB)
BLACK = (17, 17, 17)
WHITE = (232, 232, 232)
GREY = (138, 138, 138)
DARK_GREY = (40, 40, 40)
RED = (255, 85, 85)
GREEN = (46, 204, 113)
AMBER = (241, 196, 15)
BLUE = (74, 163, 255)
CYAN = (0, 229, 255)
MAGENTA = (224, 86, 253)
FUCHSIA = (255, 0, 204)
GHOST_RGB = (255, 178, 71)
PANEL_BG = (24, 24, 28)

FIX_COLORS = {
    "Fixed RTK": (46, 204, 113),
    "Float RTK": (241, 196, 15),
    "DGNSS": (52, 152, 219),
    "3D Fix": (230, 126, 34),
    "No Fix": (231, 76, 60),
    "Unknown": (149, 165, 166)
}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def find_cmap(args):
    if args.cmap:
        return args.cmap if os.path.exists(args.cmap) else None
    return next((p for p in CMAP_SEARCH if os.path.exists(p)), None)


def drivable_area(grid, inflate_m, plan_res=0.10):
    walls = grid.blocked()
    free = grid.grid < FREE_T
    keep = math.floor(inflate_m / plan_res + 0.5) * plan_res if plan_res > 0 else inflate_m
    reach = keep / grid.res
    r = int(math.floor(reach + 1e-9))
    if r <= 0 or not walls.any():
        return free
    h, w = walls.shape
    pad = np.pad(walls, r)
    near = np.zeros_like(walls)
    lim = reach * reach + 1e-9
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= lim:
                near |= pad[r + dy:r + dy + h, r + dx:r + dx + w]
    return free & ~near


class PygameVisualiser:
    def __init__(self, args):
        self.a = args
        self.ticks_per_m = args.ticks_per_rev / (math.pi * args.wheel_diameter)
        self.ref = None if args.anchor_first_fix else (args.ref_lat, args.ref_lon)
        self.min_rank = FIX_RANK[args.min_fix]
        self.gnss_stale = max(3.0, 2.5 / args.gnss_hz)
        self.esp_stale = max(2.5, 2.5 / args.esp_hz)

        # Bus communication
        self.sub = Subscriber(["rtk", "esp", "lidar", "cmd_vel", "path", "goal"])
        self.pub = Publisher()

        # Map setup
        self.world = None
        path = args.cmap_path
        if path:
            im = Image.open(path)
            if im.mode in ("RGBA", "LA", "PA") or (im.mode == "P" and "transparency" in im.info):
                im = im.convert("RGBA")
                bg = Image.new("RGBA", im.size, (0, 0, 0, 255) if args.invert else (255, 255, 255, 255))
                bg.alpha_composite(im)
                im = bg
            gray = np.asarray(im.convert("L"), dtype=np.uint8)
            self.world = gray.copy() if args.invert else (255 - gray)
            mh, mw = self.world.shape
            wm, hm = mw * args.res, mh * args.res
            print(f"World image {os.path.abspath(path)}: {mw}x{mh}px = {wm:.1f} x {hm:.1f} m")
        else:
            wm = hm = args.map_size
            print(f"No cmap.png found: learned map covers {wm:.0f} x {hm:.0f} m")
        
        self.grid = OccupancyGrid(wm, hm, args.res)
        self.extent = self.grid.extent

        # Navigation
        self.est = PoseEstimator()
        if args.planner:
            load_planner(args.planner)
        self.nav = Navigator(self.grid, args.planner, args.max_speed, args.max_turn, plan_res=args.plan_res,
                             inflate_m=args.inflate, replan_s=args.replan, lookahead=args.lookahead,
                             goal_tol=args.goal_tol, stop_dist=args.stop_dist)
        self.nav_goal_xy = None
        self.builtin = args.builtin_nav
        self.ext_path = None
        self.last_pose_pub = 0.0
        self.msg, self.msg_until = "", 0.0

        # Route planning & state
        self.inflate = clamp(args.inflate, 0.0, 1.5)
        self.plan_mode = False
        self.queue = []
        self.route_shown = []
        self.route_done = 0
        self.route_todo = []
        self.chain_after = 0.0
        self.drive_key, self.drive_mask = None, None

        self.rv = self.es = None
        self.gpos, self.g_ts, self.gfix = None, None, "No Fix"
        self.odom, self.odom_anchored, self.prev, self.last_seq = None, False, None, None
        self.o_yaw, self.yaw_ts, self.have_yaw, self.e_ts = 0.0, None, False, None
        self.v_meas = self.w_meas = 0.0
        self.g_trail = deque(maxlen=args.trail)
        self.o_trail = deque(maxlen=args.trail)
        self.follow = True
        self.tel = None
        self.ghost = None
        self.placed = False
        self.yaw_off = 0.0
        self.pend = [0.0, 0.0]
        self.anchored = False
        self.gps_err = None
        self.loc = Localiser(self.grid)

        self.lidar_ang = self.lidar_dist = self.lidar_ts = None
        self.lidar_seq = self.map_seq = 0
        self.lidar_times = deque(maxlen=200)
        self.front_min = None
        self.lidar_near = 0.0

        self.goal = None
        self.disp = None
        self.goal_trail = deque(maxlen=args.trail)
        self.cmd_seen = (0.0, 0.0, -1e9)
        self.t_last = None

        self.keys = set()
        self.released = {}
        self.cmd, self.cmd_active = (0.0, 0.0), False

        # Pygame setup (Windowed and Resizable by default)
        pygame.init()
        pygame.font.init()
        self.width, self.height = 1200, 750
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        pygame.display.set_caption("UGV Visualiser (Pygame)")
        self.clock = pygame.time.Clock()

        # Layout dimensions
        self.map_panel_width = 800
        self.side_panel_width = self.width - self.map_panel_width

        # Caching for high performance & CPU throttling
        self.cached_grid_surf = None
        self.cached_grid_scans = -1
        self.cached_view = -1
        self.cached_drivable_surf = None
        self._last_map_redraw = 0.0

        # Fonts
        try:
            self.font_sm = pygame.font.SysFont("Consolas", 13)
            self.font_md = pygame.font.SysFont("Consolas", 15)
            self.font_lg = pygame.font.SysFont("Consolas", 18, bold=True)
        except Exception:
            self.font_sm = pygame.font.Font(None, 18)
            self.font_md = pygame.font.Font(None, 20)
            self.font_lg = pygame.font.Font(None, 24)

        self.running = True

    def say(self, text, secs=4.0):
        self.msg, self.msg_until = text, time.monotonic() + secs

    def run(self):
        last_control_tick = time.monotonic()
        while self.running:
            now_mono = time.monotonic()
            dt = 1.0 / self.a.fps if self.t_last is None else min(now_mono - self.t_last, 0.5)
            self.t_last = now_mono

            self._handle_events()

            # Control tick at 20 Hz
            if now_mono - last_control_tick >= 0.05:
                last_control_tick = now_mono
                self._control_tick()

            self.tick(dt)
            self.render()
            self.clock.tick(self.a.fps)

        self.nav.close()
        if self.cmd_active and self.a.drive:
            try:
                self.pub.send("cmd_vel", linear=0.0, angular=0.0)
            except Exception:
                pass
        pygame.quit()

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.VIDEORESIZE:
                self.width, self.height = event.w, event.h
                self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
                self.map_panel_width = min(self.width - 400, int(self.width * 0.7))
                self.side_panel_width = self.width - self.map_panel_width
                self.cached_grid_surf = None
            elif event.type == pygame.KEYDOWN:
                k = event.key
                if k in ARROWS:
                    self.keys.add(k)
                    self.released.pop(k, None)
                elif k == pygame.K_SPACE:
                    self.toggle_plan()
                elif k in (pygame.K_x, pygame.K_ESCAPE):
                    self.keys.clear()
                    self.released.clear()
                    self.plan_mode, self.queue = False, []
                    self.cancel_nav()
                    self.say("stopped: navigation and route cancelled")
                elif k in (pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET):
                    self.change_inflate(-1 if k == pygame.K_LEFTBRACKET else 1)
                elif k == pygame.K_c:
                    self.g_trail.clear()
                    self.o_trail.clear()
                    self.goal_trail.clear()
                elif k == pygame.K_f:
                    self.follow = not self.follow
                elif k == pygame.K_o:
                    if self.gpos is not None and self.odom is not None:
                        self.odom = list(self.gpos)
                        self.o_trail.clear()
                elif k == pygame.K_g:
                    self.snap_goal()
                elif k == pygame.K_r:
                    self.grid.reset()
                    self.cached_grid_surf = None
                    self.say("map reset")
                elif k == pygame.K_s:
                    try:
                        self.grid.save_png(self.a.save_map)
                        self.say(f"saved {os.path.abspath(self.a.save_map)}", 6)
                        print(f"[MAP] saved {os.path.abspath(self.a.save_map)}")
                    except OSError as e:
                        self.say(f"could not save map: {e}", 6)
            elif event.type == pygame.KEYUP:
                k = event.key
                if k in ARROWS:
                    self.released[k] = time.monotonic()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                if mx < self.map_panel_width:
                    wx, wy = self.screen_to_world(mx, my)
                    if self.plan_mode:
                        if event.button == 1:
                            self.add_waypoint(wx, wy)
                        elif event.button == 3:
                            self.undo_waypoint()
                    else:
                        if event.button == 3:
                            self.cancel_nav()
                            self.say("navigation cancelled")
                        elif event.button == 1:
                            self.start_nav(wx, wy)

    def screen_to_world(self, sx, sy):
        target = (self.disp[0], self.disp[1]) if self.disp is not None else (self.tel if self.tel else (0.0, 0.0))
        cx, cy = target if self.follow else (0.0, 0.0)
        ppm = self.map_panel_width / self.a.view
        wx = cx + (sx - self.map_panel_width / 2) / ppm
        wy = cy + (self.map_panel_height / 2 - sy) / ppm
        return wx, wy

    def world_to_screen(self, wx, wy):
        target = (self.disp[0], self.disp[1]) if self.disp is not None else (self.tel if self.tel else (0.0, 0.0))
        cx, cy = target if self.follow else (0.0, 0.0)
        ppm = self.map_panel_width / self.a.view
        sx = self.map_panel_width / 2 + (wx - cx) * ppm
        sy = (self.map_panel_height / 2) - (wy - cy) * ppm
        return int(sx), int(sy)

    @property
    def map_panel_height(self):
        return self.height

    def toggle_plan(self):
        if not self.a.drive:
            self.say("view-only mode (--no-drive): not driving")
            return
        if not self.plan_mode:
            self.plan_mode, self.queue = True, []
            self.cached_drivable_surf = None
            self.say("PLAN: click inside fuchsia to queue waypoints | right-click = undo | space = go | x = abort", 10)
            return
        self.plan_mode = False
        queued, self.queue = list(self.queue), []
        if queued:
            self.send_route(queued)
        else:
            self.say("plan mode off (nothing was queued)")

    def add_waypoint(self, x, y):
        mask = self.drivable()
        row, col = self.grid.to_cell(x, y)
        if not self.grid.inside(row, col) or not mask[row, col]:
            self.say(f"can't go there: not free or within {self.inflate:.2f}m of wall", 4)
            return
        self.queue.append((float(x), float(y)))
        self.say(f"waypoint {len(self.queue)} queued ({x:+.1f}, {y:+.1f}) | space = go")

    def undo_waypoint(self):
        if self.queue:
            self.queue.pop()
            self.say(f"removed last waypoint; {len(self.queue)} queued")
        else:
            self.say("nothing queued")

    def change_inflate(self, direction):
        step = self.a.plan_res if self.a.plan_res > 0 else 0.05
        self.inflate = clamp(round(self.inflate + direction * step, 3), 0.0, 1.5)
        self.drive_key = None
        self.cached_drivable_surf = None
        note = ""
        if self.plan_mode and self.queue:
            mask = self.drivable()
            keep = []
            for (qx, qy) in self.queue:
                r, c = self.grid.to_cell(qx, qy)
                if self.grid.inside(r, c) and mask[r, c]:
                    keep.append((qx, qy))
            dropped, self.queue = len(self.queue) - len(keep), keep
            if dropped:
                note += f" ({dropped} dropped)"
        self.say(f"keep-out distance {self.inflate:.2f} m{note}", 6)

    def drivable(self):
        key = (self.grid.scans, round(self.inflate, 3), self.a.plan_res)
        if key != self.drive_key:
            self.drive_key = key
            self.drive_mask = drivable_area(self.grid, self.inflate, self.a.plan_res)
            self.cached_drivable_surf = None
        return self.drive_mask

    def send_route(self, pts):
        self.keys.clear()
        try:
            self.pub.send("goal", action="route", route=[[x, y] for x, y in pts], inflate=float(self.inflate))
        except Exception as e:
            self.say(f"could not send route: {e}", 6)
            return
        self.nav_goal_xy = pts[-1]
        self.route_shown, self.route_done = list(pts), 0
        n = len(pts)
        if not self.builtin:
            self.ext_path = None
            self.say(f"route of {n} waypoints sent on bus (port 5008)", 6)
            return
        pose, why = self.nav_pose()
        if pose is None:
            self.say(f"can't navigate yet: {why}", 6)
            return
        self.route_todo = list(pts[1:])
        self.nav.set_goal(pts[0][0], pts[0][1], pose)
        self.chain_after = time.monotonic() + 1.5
        self.say(f"driving route of {n} waypoints")

    def cancel_nav(self, publish=True):
        self.nav.cancel()
        self.nav_goal_xy = None
        self.ext_path = None
        self.route_shown, self.route_todo, self.route_done = [], [], 0
        if publish and self.a.drive:
            try:
                self.pub.send("goal", action="cancel")
            except Exception:
                pass

    def start_nav(self, x, y):
        if not self.a.drive:
            self.say("view-only mode (--no-drive)")
            return
        self.keys.clear()
        try:
            self.pub.send("goal", action="go", x=x, y=y, inflate=float(self.inflate))
        except Exception as e:
            self.say(f"could not publish goal: {e}", 6)
        self.nav_goal_xy = (x, y)
        if not self.builtin:
            self.ext_path = None
            self.say(f"goal ({x:+.1f}, {y:+.1f}) sent on bus", 6)
            return
        pose, why = self.nav_pose()
        if pose is None:
            self.say(f"can't navigate yet: {why}", 6)
            return
        self.nav.set_goal(x, y, pose)
        self.say(f"driving to ({x:+.1f}, {y:+.1f})")

    def nav_pose(self):
        now = time.time()
        if not self.placed or not self.est.has_fix():
            return None, f"waiting for {self.a.min_fix} fix"
        age = self.est.fix_age(now)
        if age is None or age > self.esp_stale:
            return None, "odometry data stale"
        return self.est.pose_at(now), ""

    def _control_tick(self):
        if not self.a.drive:
            return
        now = time.monotonic()
        for k, t in list(self.released.items()):
            if now - t > 0.06:
                self.keys.discard(k)
                del self.released[k]
        lin = ((pygame.K_UP in self.keys) - (pygame.K_DOWN in self.keys)) * self.a.max_speed
        ang = ((pygame.K_LEFT in self.keys) - (pygame.K_RIGHT in self.keys)) * self.a.max_turn
        cmd = None
        if lin or ang:
            if self.nav.active or self.nav_goal_xy:
                self.cancel_nav()
                self.say("teleop took over")
            cmd = (lin, ang)
        elif self.nav.active:
            pose, why = self.nav_pose()
            if pose is None:
                cmd = (0.0, 0.0)
                self.nav.reason = why
            else:
                cmd = self.nav.update(pose, self.front_min)
        
        if now - self.last_pose_pub >= 0.1:
            self.last_pose_pub = now
            pose, _why = self.nav_pose()
            if pose is not None:
                try:
                    self.pub.send("pose", x=pose[0], y=pose[1], yaw=pose[2], fix=self.gfix)
                except Exception:
                    pass

        if cmd is not None:
            self.pub.send("cmd_vel", linear=cmd[0], angular=cmd[1])
            self.cmd, self.cmd_active = cmd, True
        elif self.cmd_active:
            self.pub.send("cmd_vel", linear=0.0, angular=0.0)
            self.cmd, self.cmd_active = (0.0, 0.0), False

    def goal_cmd(self):
        lin, ang, t = self.cmd_seen
        if time.monotonic() - t > cfg.CMD_TIMEOUT:
            return 0.0, 0.0
        return clamp(lin, -self.a.max_speed, self.a.max_speed), clamp(ang, -self.a.max_turn, self.a.max_turn)

    def step_goal(self, dt, now):
        g = self.goal
        if g is None or dt <= 0:
            return
        cv, cw = self.goal_cmd()
        if (cv > 0 and self.front_min is not None and self.lidar_ts is not None and time.time() - self.lidar_ts < 0.5
                and self.front_min < self.a.robot_length / 2 + 0.08):
            cv = 0.0
        g[3] += clamp(cv - g[3], -self.a.lin_accel * dt, self.a.lin_accel * dt)
        g[4] += clamp(cw - g[4], -self.a.ang_accel * dt, self.a.ang_accel * dt)
        mid = g[2] + g[4] * dt / 2
        g[0] += g[3] * math.cos(mid) * dt
        g[1] += g[3] * math.sin(mid) * dt
        g[2] = wrap(g[2] + g[4] * dt)
        self.est.push_model(now, g[0], g[1], g[2])
        est = self.est.pose_at(now) or (g[0], g[1], g[2])
        d = self.disp
        if d is None:
            d = self.disp = list(est)
        else:
            k = 1 - math.exp(-dt / 0.3)
            d[0] += (est[0] - d[0]) * k
            d[1] += (est[1] - d[1]) * k
            d[2] = wrap(d[2] + wrap(est[2] - d[2]) * k)
        if not self.goal_trail or math.hypot(d[0] - self.goal_trail[-1][0], d[1] - self.goal_trail[-1][1]) > 0.02:
            self.goal_trail.append((d[0], d[1]))

    def snap_goal(self):
        if not self.est.has_fix():
            return
        t_pos, x, y, t_yaw, yaw = self.est.fix
        self.goal = [x, y, yaw, 0.0, 0.0]
        self.disp = [x, y, yaw]
        self.est.reset_model()
        self.est.push_model(time.time(), x, y, yaw)
        self.goal_trail.clear()

    def footprint_corners(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        hl, hw = self.a.robot_length / 2, self.a.robot_width / 2
        local_pts = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
        poly = []
        for px, py in local_pts:
            wx = x + px * c - py * s
            wy = y + px * s + py * c
            poly.append(self.world_to_screen(wx, wy))
        return poly

    def to_local(self, lat, lon):
        lat0, lon0 = self.ref
        return (lon - lon0) * M_PER_DEG * math.cos(math.radians(lat0)), (lat - lat0) * M_PER_DEG

    def _shift_fix(self, dx, dy, dyaw):
        t_pos, x, y, t_yaw, yaw = self.est.fix
        self.est.set_fix(t_pos, x + dx, y + dy, t_yaw, wrap(yaw + dyaw))
        self.yaw_off += dyaw

    def _gps_fuse(self, rank):
        p = self.est.pose_at(self.g_ts)
        if p is None:
            return
        ex, ey = self.gpos[0] - p[0], self.gpos[1] - p[1]
        self.gps_err = math.hypot(ex, ey)
        if self.a.no_gps_fusion or rank < 3:
            return
        if rank >= 4 and not self.anchored:
            self.anchored, gain = True, 1.0
        elif rank >= 4:
            if self.gps_err > 2.0:
                return
            gain = 0.15
        else:
            gain = 0.03
        self.pend[0] += gain * ex
        self.pend[1] += gain * ey

    def on_rtk(self, m):
        self.rv = m
        self.gfix = m["fix"]
        if m["fix"] == "No Fix" or (m["lat"] == 0 and m["lon"] == 0):
            self.gpos = None
            return
        if self.ref is None:
            self.ref = (m["lat"], m["lon"])
        self.gpos, self.g_ts = self.to_local(m["lat"], m["lon"]), m["ts"]
        self.g_trail.append(self.gpos)
        rank = FIX_RANK.get(m["fix"], 0)
        if not self.placed:
            if rank >= self.min_rank and self.yaw_ts is not None:
                self.placed = True
                self.yaw_off = 0.0
                self.est.set_fix(self.g_ts, self.gpos[0], self.gpos[1], self.yaw_ts, self.o_yaw)
                self.say(f"robot placed at {m['fix']} position", 6)
        elif self.est.has_fix():
            self._gps_fuse(rank)

    def on_esp(self, m):
        self.es = m
        seq = m["seq"]
        if seq == self.last_seq:
            return
        if self.last_seq is not None and seq < self.last_seq:
            self.prev = None
        self.last_seq = seq
        enc, imu = m["enc"], m["imu"]
        L, R = enc["left_ticks"], enc["right_ticks"]
        self.e_ts = m["ts"]
        self.v_meas = ((enc["left_vel"] + enc["right_vel"]) / 2) / self.ticks_per_m
        self.w_meas = imu["gyro"][2]
        q = imu.get("quat")
        yaw = yaw_from_quat(q) if q else None
        if yaw is not None:
            self.have_yaw = True
        if self.prev is None:
            self.prev = (L, R, yaw)
            if self.odom is None:
                self.odom = [0.0, 0.0]
            if yaw is not None:
                self.o_yaw = yaw
        else:
            pl, pr, pyaw = self.prev
            dl, dr = (L - pl) / self.ticks_per_m, (R - pr) / self.ticks_per_m
            d = (dl + dr) / 2
            if yaw is not None:
                self.o_yaw = yaw
            else:
                dth = (dr - dl) / self.a.wheel_base
                self.o_yaw += dth
            self.odom[0] += d * math.cos(self.o_yaw)
            self.odom[1] += d * math.sin(self.o_yaw)
            if self.placed and self.est.has_fix():
                _, fx, fy, _, _ = self.est.fix
                hm = self.o_yaw + self.yaw_off
                self.est.set_fix(m["ts"], fx + d * math.cos(hm), fy + d * math.sin(hm), m["ts"], wrap(self.o_yaw + self.yaw_off))
            self.prev = (L, R, yaw)
            self.o_trail.append(tuple(self.odom))
        self.yaw_ts = m["ts"]

    def on_lidar(self, m):
        arr = np.asarray(m["points"], dtype=float).reshape(-1, 2)
        self.lidar_ang = (arr[:, 0] + self.a.lidar_yaw_offset) % 360.0
        self.lidar_dist, self.lidar_ts = arr[:, 1], m["ts"]
        self.lidar_seq += 1
        self.lidar_times.append(time.monotonic())
        front = (self.lidar_ang <= 35.0) | (self.lidar_ang >= 325.0)
        self.front_min = float(self.lidar_dist[front].min()) / 1000.0 if front.any() else None
        self.lidar_near = float((self.lidar_dist < 300.0).mean()) if len(self.lidar_dist) else 0.0

    def on_cmd(self, m):
        self.cmd_seen = (m["linear"], m["angular"], time.monotonic())

    def tick(self, dt):
        now = time.time()
        for m in self.sub.poll():
            t = m["type"]
            if t == "rtk":
                self.on_rtk(m)
            elif t == "esp":
                self.on_esp(m)
            elif t == "lidar":
                self.on_lidar(m)
            elif t == "cmd_vel":
                self.on_cmd(m)
            elif t == "path":
                self.ext_path = [tuple(p) for p in m["points"]] or None
            elif t == "goal" and not self.builtin:
                if m["action"] == "go":
                    self.nav_goal_xy = (m["x"], m["y"])
                elif m["action"] == "route":
                    self.nav_goal_xy = tuple(m["route"][-1])
                else:
                    self.nav_goal_xy, self.ext_path = None, None
                    self.route_shown, self.route_done = [], 0

        if self.odom is not None and not self.odom_anchored and self.gpos is not None:
            self.odom, self.odom_anchored = list(self.gpos), True
            self.o_trail.clear()

        g_age = (now - self.rv["ts"]) if self.rv else None
        e_age = (now - self.es["ts"]) if self.es else None
        g_fresh = self.gpos is not None and g_age is not None and g_age <= self.gnss_stale
        fx = self.est.fix if (self.placed and self.est.has_fix()) else None
        self.tel = (fx[1], fx[2]) if fx else (self.gpos if g_fresh else (tuple(self.odom) if self.odom is not None else None))

        if self.pend[0] or self.pend[1]:
            k = 1 - math.exp(-dt / 1.0)
            sx, sy = self.pend[0] * k, self.pend[1] * k
            self.pend[0] -= sx
            self.pend[1] -= sy
            if fx:
                self._shift_fix(sx, sy, 0.0)
            if abs(self.pend[0]) + abs(self.pend[1]) < 1e-4:
                self.pend = [0.0, 0.0]

        if self.goal is None and fx:
            self.snap_goal()
        self.step_goal(dt, now)

        if fx:
            if self.ghost is None or self.a.ghost_smooth <= 0:
                self.ghost = [fx[1], fx[2], fx[4]]
            else:
                k = 1 - math.exp(-dt / self.a.ghost_smooth)
                gh = self.ghost
                gh[0] += (fx[1] - gh[0]) * k
                gh[1] += (fx[2] - gh[1]) * k
                gh[2] = wrap(gh[2] + wrap(fx[4] - gh[2]) * k)

        self._route_progress()
        self._maybe_map(now)

    def _route_progress(self):
        if self.route_shown and self.ghost is not None:
            while self.route_shown and math.hypot(self.route_shown[0][0] - self.ghost[0],
                                                  self.route_shown[0][1] - self.ghost[1]) < 0.4:
                self.route_shown.pop(0)
                self.route_done += 1
        if (self.builtin and self.route_todo and not self.nav.active and self.nav.state == "arrived"
                and time.monotonic() > self.chain_after):
            pose, _why = self.nav_pose()
            if pose is not None:
                nx, ny = self.route_todo.pop(0)
                self.nav.set_goal(nx, ny, pose)
                self.chain_after = time.monotonic() + 1.5

    def _maybe_map(self, now):
        if self.lidar_dist is None or self.lidar_seq == self.map_seq:
            return
        self.map_seq = self.lidar_seq
        if now - self.lidar_ts > LIDAR_STALE_S or self.lidar_near > 0.6:
            return
        pose, _why = self.nav_pose()
        if pose is None:
            return
        pose = self.est.pose_at(self.lidar_ts)
        if not self.a.no_localise:
            self.loc.refresh()
            r = self.loc.match(pose, self.lidar_ang, self.lidar_dist)
            if r and (r[0] or r[1] or r[2]):
                self._shift_fix(0.3 * r[0], 0.3 * r[1], 0.3 * r[2])
                pose = self.est.pose_at(self.lidar_ts)
        self.grid.integrate_scan(pose, self.lidar_ang, self.lidar_dist, free_range=self.a.free_range)
        
        # Throttle map cache invalidation to at most once per second to save CPU
        mono = time.monotonic()
        if mono - self._last_map_redraw > 1.0:
            self._last_map_redraw = mono
            self.cached_grid_surf = None

    def render(self):
        self.screen.fill(BLACK)

        # 1. Render Map Surface
        map_surface = self._create_map_surface()
        self.screen.blit(map_surface, (0, 0))

        # 2. Render Side Panel
        panel_rect = pygame.Rect(self.map_panel_width, 0, self.side_panel_width, self.height)
        pygame.draw.rect(self.screen, PANEL_BG, panel_rect)
        pygame.draw.line(self.screen, DARK_GREY, (self.map_panel_width, 0), (self.map_panel_width, self.height), 2)
        self._render_side_panel()

        pygame.display.flip()

    def _create_map_surface(self):
        surf = pygame.Surface((self.map_panel_width, self.map_panel_height))
        surf.fill(DARK_GREY)

        ppm = self.map_panel_width / self.a.view
        l, r, b, t = self.extent
        world_w = r - l
        world_h = t - b
        scaled_w = int(world_w * ppm)
        scaled_h = int(world_h * ppm)

        # Use cached grid image to prevent high CPU / fan usage
        grid_img = self.grid.image()
        if grid_img is not None:
            if self.grid.scans != self.cached_grid_scans or abs(self.a.view - self.cached_view) > 1e-3 or self.cached_grid_surf is None:
                self.cached_grid_scans = self.grid.scans
                self.cached_view = self.a.view
                gh, gw = grid_img.shape
                rgb_arr = np.repeat((grid_img * 255).astype(np.uint8)[..., None], 3, axis=-1)
                grid_surf = pygame.image.frombuffer(rgb_arr.tobytes(), (gw, gh), "RGB")
                self.cached_grid_surf = pygame.transform.scale(grid_surf, (scaled_w, scaled_h))
                self.cached_drivable_surf = None

            top_left_wx, top_left_wy = l, t
            tl_sx, tl_sy = self.world_to_screen(top_left_wx, top_left_wy)
            surf.blit(self.cached_grid_surf, (tl_sx, tl_sy))

        # Drivable overlay in plan mode (cached for performance)
        if self.plan_mode:
            mask = self.drivable()
            if mask is not None:
                if self.cached_drivable_surf is None:
                    mh, mw = mask.shape
                    rgba = np.zeros((mh, mw, 4), dtype=np.uint8)
                    rgba[mask] = [255, 0, 204, 120]
                    drivable_raw = pygame.image.frombuffer(rgba.tobytes(), (mw, mh), "RGBA")
                    self.cached_drivable_surf = pygame.transform.scale(drivable_raw, (scaled_w, scaled_h))
                
                top_left_wx, top_left_wy = l, t
                tl_sx, tl_sy = self.world_to_screen(top_left_wx, top_left_wy)
                surf.blit(self.cached_drivable_surf, (tl_sx, tl_sy))

        # Draw trails (optimized using standard fast lines and downsampling)
        if len(self.g_trail) > 1:
            pts = [self.world_to_screen(p[0], p[1]) for p in list(self.g_trail)[::2]]
            if len(pts) > 1:
                pygame.draw.lines(surf, GREEN, False, pts, 1)
        if len(self.o_trail) > 1:
            pts = [self.world_to_screen(p[0], p[1]) for p in list(self.o_trail)[::2]]
            if len(pts) > 1:
                pygame.draw.lines(surf, MAGENTA, False, pts, 1)
        if len(self.goal_trail) > 1:
            pts = [self.world_to_screen(p[0], p[1]) for p in list(self.goal_trail)[::2]]
            if len(pts) > 1:
                pygame.draw.lines(surf, BLUE, False, pts, 1)

        # Planned path
        shown = self.nav.path if self.builtin else self.ext_path
        if shown and len(shown) > 1:
            pts = [self.world_to_screen(p[0], p[1]) for p in shown]
            pygame.draw.lines(surf, CYAN, False, pts, 2)

        # Nav goal marker
        if self.nav_goal_xy and (self.nav.active or not self.builtin):
            gx, gy = self.world_to_screen(*self.nav_goal_xy)
            pygame.draw.circle(surf, CYAN, (gx, gy), 8)
            pygame.draw.circle(surf, WHITE, (gx, gy), 8, 1)

        # Lidar scan points (optimized with downsampling and fast rects)
        if self.lidar_dist is not None and time.time() - self.lidar_ts <= LIDAR_STALE_S and self.est.has_fix():
            x, y, yaw = self.est.pose_at(self.lidar_ts)
            bearing = yaw - np.radians(self.lidar_ang)
            d = self.lidar_dist / 1000.0
            scan_xs = x + d * np.cos(bearing)
            scan_ys = y + d * np.sin(bearing)
            for sx_w, sy_w in zip(scan_xs[::2], scan_ys[::2]):
                sx, sy = self.world_to_screen(sx_w, sy_w)
                if 0 <= sx < self.map_panel_width and 0 <= sy < self.height:
                    pygame.draw.rect(surf, RED, (sx, sy, 2, 2))

        # Ghost robot polygon
        if self.ghost is not None:
            poly = self.footprint_corners(self.ghost[0], self.ghost[1], self.ghost[2])
            pygame.draw.polygon(surf, GHOST_RGB, poly)
            pygame.draw.polygon(surf, WHITE, poly, 2)

        # Goal robot polygon
        if self.disp is not None:
            poly = self.footprint_corners(self.disp[0], self.disp[1], self.disp[2])
            pygame.draw.polygon(surf, (74, 163, 255), poly)
            pygame.draw.polygon(surf, WHITE, poly, 2)

        # Queue waypoints / route
        pts = self.queue if self.plan_mode else self.route_shown
        first_num = 1 if self.plan_mode else self.route_done + 1
        if pts:
            screen_pts = [self.world_to_screen(p[0], p[1]) for p in pts]
            if len(screen_pts) > 1:
                pygame.draw.lines(surf, FUCHSIA, False, screen_pts, 2)
            for i, (sx, sy) in enumerate(screen_pts):
                pygame.draw.circle(surf, FUCHSIA, (sx, sy), 8)
                pygame.draw.circle(surf, WHITE, (sx, sy), 8, 1)
                lbl = self.font_sm.render(str(first_num + i), True, WHITE)
                surf.blit(lbl, (sx - 4, sy - 6))

        # Overlay message & instructions
        txt = self.msg if time.monotonic() < self.msg_until else ""
        status = self._nav_status() if (self.nav.active or self.nav.state in ("arrived",) or not self.builtin) else ""
        if self.plan_mode:
            status = f"PLAN MODE: {len(self.queue)} queued. Click to add, Space to go."
        
        display_txt = txt if txt else (status if status else "Click map to drive | Space: plan route | Arrows: teleop")
        msg_surf = self.font_md.render(display_txt, True, CYAN)
        surf.blit(msg_surf, (15, self.height - 35))

        return surf

    def _nav_status(self):
        if self.builtin:
            return self.nav.status()
        if self.nav_goal_xy is None:
            return "external planner: click map to send goal"
        n = len(self.ext_path) if self.ext_path else 0
        return f"goal ({self.nav_goal_xy[0]:+.1f}, {self.nav_goal_xy[1]:+.1f}) -> path {n} pts"

    def _render_side_panel(self):
        rv, es = self.rv, self.es
        g_age = (time.time() - rv["ts"]) if rv else None
        e_age = (time.time() - es["ts"]) if es else None

        rows = []
        rows.append(("GNSS  (rtk, port 5007)", CYAN, self.font_lg))
        if not rv:
            rows.append(("  NO DATA", RED, self.font_md))
        else:
            stale = g_age is not None and g_age > self.gnss_stale
            fix_col = FIX_COLORS.get(self.gfix, WHITE)
            rows.append((f"  fix    {self.gfix}", fix_col, self.font_md))
            rows.append((f"  sats   {rv['sats']}    hdop {rv['hdop']}", WHITE, self.font_md))
            rows.append((f"  lat    {rv['lat']:.7f}", WHITE, self.font_md))
            rows.append((f"  lon    {rv['lon']:.7f}", WHITE, self.font_md))
            rows.append((f"  age    {g_age:.1f} s{' STALE' if stale else ''}", RED if stale else GREY, self.font_md))

        rows.append(("", WHITE, self.font_sm))
        rows.append(("POSE  (m from map centre)", CYAN, self.font_lg))
        fx = self.est.fix if (self.placed and self.est.has_fix()) else None
        if fx:
            rows.append((f"  ROBOT x={fx[1]:+6.2f} y={fx[2]:+6.2f}", WHITE, self.font_md))
        else:
            rows.append((f"  Waiting for {self.a.min_fix}", AMBER, self.font_md))
        if self.gpos is not None:
            rows.append((f"  GNSS  x={self.gpos[0]:+6.2f} y={self.gpos[1]:+6.2f}", WHITE, self.font_md))
        if self.odom is not None:
            rows.append((f"  ODOM  x={self.odom[0]:+6.2f} y={self.odom[1]:+6.2f}", GREY, self.font_md))
        rows.append((f"  yaw   {math.degrees(fx[4]) if fx else math.degrees(self.o_yaw):+6.1f} deg", WHITE, self.font_md))

        rows.append(("", WHITE, self.font_sm))
        rows.append(("ESP32  (esp, port 5006)", CYAN, self.font_lg))
        if not es:
            rows.append(("  NO DATA", RED, self.font_md))
        else:
            stale = e_age is not None and e_age > self.esp_stale
            rows.append((f"  seq    {es['seq']}", WHITE, self.font_md))
            rows.append((f"  speed  {self.v_meas:+.2f} m/s", WHITE, self.font_md))
            rows.append((f"  gyro z {es['imu']['gyro'][2]:+.3f}", WHITE, self.font_md))
            rows.append((f"  age    {e_age:.1f} s{' STALE' if stale else ''}", RED if stale else GREY, self.font_md))

        rows.append(("", WHITE, self.font_sm))
        rows.append(("LIDAR  (port 5005)", CYAN, self.font_lg))
        if self.lidar_ts is None:
            rows.append(("  waiting for scans...", AMBER, self.font_md))
        else:
            age = time.time() - self.lidar_ts
            rows.append((f"  points {len(self.lidar_dist)}", WHITE, self.font_md))
            rows.append((f"  front  {'-' if self.front_min is None else f'{self.front_min:.2f}m'}", WHITE, self.font_md))

        known, walls = self.grid.stats()
        rows.append(("", WHITE, self.font_sm))
        rows.append(("MAP & NAVIGATION", CYAN, self.font_lg))
        rows.append((f"  known  {known:.1f}%  walls {walls}", WHITE, self.font_md))
        rows.append((f"  keep-out {self.inflate:.2f}m ([ ])", WHITE, self.font_md))
        rows.append((f"  {self._nav_status()[:38]}", WHITE, self.font_sm))

        rows.append(("", WHITE, self.font_sm))
        rows.append(("SHORTCUTS", CYAN, self.font_lg))
        rows.append(("  arrows : teleop", GREY, self.font_sm))
        rows.append(("  space  : route plan", GREY, self.font_sm))
        rows.append(("  c / f / o / g / r / s : actions", GREY, self.font_sm))

        x_offset = self.map_panel_width + 15
        y_offset = 20
        for text, color, font in rows:
            if text == "":
                y_offset += 10
                continue
            rendered = font.render(text, True, color)
            self.screen.blit(rendered, (x_offset, y_offset))
            y_offset += font.get_height() + 4


def main():
    ap = argparse.ArgumentParser(description="Live robot visualiser + teleop + click-to-drive (Pygame)")
    ap.add_argument("--no-drive", action="store_true", help="view only: arrow keys and clicks do nothing, never publishes")
    ap.add_argument("--cmap", help="hidden world image")
    ap.add_argument("--invert", action="store_true", help="world image is black=free, white=wall")
    ap.add_argument("--show-world", action="store_true", help="overlay hidden world in translucent red")
    ap.add_argument("--map-size", type=float, default=40.0, help="learned map width/height in m")
    ap.add_argument("--res", type=float, default=cfg.MAP_RES)
    ap.add_argument("--ref-lat", type=float, default=cfg.ORIGIN_LAT)
    ap.add_argument("--ref-lon", type=float, default=cfg.ORIGIN_LON)
    ap.add_argument("--anchor-first-fix", action="store_true")
    ap.add_argument("--min-fix", default="Float RTK", choices=["3D Fix", "DGNSS", "Float RTK", "Fixed RTK"])
    ap.add_argument("--free-range", type=float, default=6.0)
    ap.add_argument("--no-localise", action="store_true")
    ap.add_argument("--no-gps-fusion", action="store_true")
    ap.add_argument("--builtin-nav", action="store_true")
    ap.add_argument("--planner", help="path planner as module:function")
    ap.add_argument("--plan-res", type=float, default=0.10)
    ap.add_argument("--inflate", type=float, default=0.40)
    ap.add_argument("--replan", type=float, default=1.0)
    ap.add_argument("--lookahead", type=float, default=0.6)
    ap.add_argument("--goal-tol", type=float, default=0.25)
    ap.add_argument("--stop-dist", type=float, default=0.45)
    ap.add_argument("--max-speed", type=float, default=cfg.MAX_SPEED)
    ap.add_argument("--max-turn", type=float, default=cfg.MAX_TURN)
    ap.add_argument("--lin-accel", type=float, default=cfg.LIN_ACCEL)
    ap.add_argument("--ang-accel", type=float, default=cfg.ANG_ACCEL)
    ap.add_argument("--gnss-hz", type=float, default=1.0)
    ap.add_argument("--esp-hz", type=float, default=1.0)
    ap.add_argument("--ghost-smooth", type=float, default=0.3)
    ap.add_argument("--lidar-yaw-offset", type=float, default=0.0)
    ap.add_argument("--save-map", default="learned_map.png")
    ap.add_argument("--view", type=float, default=12.0)
    ap.add_argument("--trail", type=int, default=2000)
    ap.add_argument("--fps", type=float, default=15.0, help="animation rate (default 15 for low CPU usage)")
    ap.add_argument("--robot-length", type=float, default=cfg.ROBOT_LENGTH)
    ap.add_argument("--robot-width", type=float, default=cfg.ROBOT_WIDTH)
    ap.add_argument("--wheel-base", type=float, default=cfg.WHEEL_BASE)
    ap.add_argument("--wheel-diameter", type=float, default=cfg.WHEEL_DIAMETER)
    ap.add_argument("--ticks-per-rev", type=int, default=cfg.TICKS_PER_REV)
    args = ap.parse_args()
    args.drive = not args.no_drive
    args.cmap_path = find_cmap(args)

    viz = PygameVisualiser(args)
    viz.run()


if __name__ == "__main__":
    main()