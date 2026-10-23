#!/usr/bin/env python3
"""
Robot visualiser, teleop and click-to-drive. A pure client of the UDP bus (common/protocol.py): it
knows nothing about whether the robot is simulated or real. Start run_sim.py (simulation) OR your
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

Your own Theta*:   python visualiser.py --planner my_theta:plan      (see common/planner.py for the signature)

Other keys   f follow robot / whole map     c clear trails     o re-anchor odometry     g snap goal to telemetry
             r reset the learned map        s save it (learned_map.png; black wall, white free, grey unknown)
Mouse zoom/pan toolbar still works; click-to-drive is ignored while a toolbar tool is active.

  pip install matplotlib numpy pillow
  python visualiser.py                       # simulated or real, whatever is on the bus
  python visualiser.py --cmap cmap.png --show-world     # also overlay the HIDDEN world (red) to check the map
  python visualiser.py --builtin-nav         # drive with the built-in planner (default: clicks are only PUBLISHED as goals on port 5008)
  python visualiser.py --no-drive            # view only: no keys, no clicks, never publishes

Map placement: the CENTRE of the map is --ref-lat/--ref-lon (default: ORIGIN in common/config.py).
With --anchor-first-fix it is the first GNSS fix instead. Map size = the cmap.png size if one is
found (next to this script, in ugv/, or the current folder), otherwise --map-size (default 40 m).

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
    import matplotlib
    try:
        import tkinter                                   # the telemetry panel is a separate Tk window
        matplotlib.use("TkAgg")
        HAVE_TK = True
    except Exception:
        HAVE_TK = False
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from PIL import Image
except ImportError as e:
    sys.exit(f"missing dependency ({e}). Run: pip install matplotlib numpy pillow")

from common import config as cfg                               # noqa: E402
from common.protocol import Publisher, Subscriber, FIX_RANK    # noqa: E402
from common.estimator import PoseEstimator                        # noqa: E402
from common.mapping import OccupancyGrid, FREE_T                  # noqa: E402
from common.localiser import Localiser                            # noqa: E402
from common.navigator import Navigator                            # noqa: E402
from common.planner import load_planner                           # noqa: E402

CMAP_SEARCH = [os.path.join(HERE, "cmap.png"),
               os.path.join(os.getcwd(), "cmap.png")]
M_PER_DEG = 111320.0
LIDAR_STALE_S = 1.0
ARROWS = ("up", "down", "left", "right")
MIN_FULL_S, MAX_FULL_S = 0.4, 1.0      # full (map/panel) redraws: at most this often, at least this often

FIX_COLORS = {"Fixed RTK": "#2ecc71", "Float RTK": "#f1c40f", "DGNSS": "#3498db", "3D Fix": "#e67e22",
              "No Fix": "#e74c3c", "Unknown": "#95a5a6"}
WHITE, GREY, RED, GREEN, AMBER, HDR = "#e8e8e8", "#8a8a8a", "#ff5555", "#2ecc71", "#f1c40f", "#58a6ff"
GOAL_FC, GHOST_RGB = (0.29, 0.64, 1.0, 0.85), (1.0, 0.70, 0.28)
FUCHSIA, FUCHSIA_RGBA = "#ff00cc", (1.0, 0.0, 0.8, 0.62)     # the "where can I go" overlay and queued waypoints


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
    """Where the robot's CENTRE can go: cells the lidar has seen to be free that are farther than the keep-out
    distance from every wall. This is the rule the rover's planner uses. Its planning cells (plan_res) make the
    keep-out distance a whole number of cells, so the distance is rounded to that cell size here too; that is why
    a gap has to be a little wider than 2 x keep-out before the planner will use it.
    Returns a boolean array shaped like grid.grid."""
    walls = grid.blocked()
    free = grid.grid < FREE_T
    keep = math.floor(inflate_m / plan_res + 0.5) * plan_res if plan_res > 0 else inflate_m   # round half UP, like the rover's std::round
    reach = keep / grid.res                                  # keep-out distance in map cells
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


class SidePanel:
    """Telemetry text in its own scrollable window (so redrawing it never costs the map any time)."""

    KEYS = {"Up": "up", "Down": "down", "Left": "left", "Right": "right", "space": " ", "x": "x",
            "Escape": "escape", "bracketleft": "[", "bracketright": "]"}

    def __init__(self, root, on_press, on_release):
        import tkinter as tk
        import tkinter.font as tkfont
        self.tk = tk
        self.top = tk.Toplevel(root)
        self.top.title("UGV telemetry")
        sw = self.top.winfo_screenwidth()
        self.top.geometry(f"430x560+{max(0, sw - 450)}+40")      # right-hand edge of the screen, so it does not cover the map
        self.top.configure(bg="#111")
        bar = tk.Scrollbar(self.top)
        bar.pack(side="right", fill="y")
        fam = "Consolas" if "Consolas" in tkfont.families() else "Courier"
        self.text = tk.Text(self.top, bg="#111", fg=WHITE, font=(fam, 10), width=48, height=30, wrap="none",
                            borderwidth=0, highlightthickness=0, cursor="arrow", yscrollcommand=bar.set,
                            state="disabled", spacing1=1)
        self.text.pack(side="left", fill="both", expand=True)
        bar.config(command=self.text.yview)
        self._tags = set()
        self._last = None

        def fwd(handler):
            def f(ev):
                k = self.KEYS.get(ev.keysym)
                if k:
                    handler(type("Ev", (), {"key": k})())
                    return "break"
            return f
        for w in (self.top, self.text):                     # the arrow keys still drive the robot from this window
            w.bind("<KeyPress>", fwd(on_press))
            w.bind("<KeyRelease>", fwd(on_release))
        self.text.bind("<Button-1>", lambda e: self.text.focus_set())

    def set_rows(self, rows):
        if rows == self._last:
            return
        self._last = list(rows)
        t = self.text
        top_frac = t.yview()[0]
        t.configure(state="normal")
        t.delete("1.0", "end")
        for txt, col in rows:
            if col not in self._tags:
                t.tag_configure(col, foreground=col)
                self._tags.add(col)
            t.insert("end", txt + "\n", col)
        t.configure(state="disabled")
        t.yview_moveto(top_frac)                            # keep the scroll position you chose


class Visualiser:
    def __init__(self, args):
        self.a = args
        self.ticks_per_m = args.ticks_per_rev / (math.pi * args.wheel_diameter)
        self.ref = None if args.anchor_first_fix else (args.ref_lat, args.ref_lon)
        self.min_rank = FIX_RANK[args.min_fix]
        self.gnss_stale = max(3.0, 2.5 / args.gnss_hz)
        self.esp_stale = max(2.5, 2.5 / args.esp_hz)

        # the bus
        self.sub = Subscriber(["rtk", "esp", "lidar", "cmd_vel", "path", "goal"])
        self.pub = Publisher()

        # the learned map. cmap.png (if present) is only used for its SIZE and the optional --show-world overlay
        self.world = None
        path = args.cmap_path
        if path:
            im = Image.open(path)
            if im.mode in ("RGBA", "LA", "PA") or (im.mode == "P" and "transparency" in im.info):   # transparent = free
                im = im.convert("RGBA")
                bg = Image.new("RGBA", im.size, (0, 0, 0, 255) if args.invert else (255, 255, 255, 255))
                bg.alpha_composite(im)
                im = bg
            gray = np.asarray(im.convert("L"), dtype=np.uint8)
            self.world = gray.copy() if args.invert else (255 - gray)       # 255 = wall
            mh, mw = self.world.shape
            wm, hm = mw * args.res, mh * args.res
            print(f"World image {os.path.abspath(path)}: {mw}x{mh}px = {wm:.1f} x {hm:.1f} m (hidden from the robot; sets the map size)")
        else:
            wm = hm = args.map_size
            print(f"No cmap.png found: learned map covers {wm:.0f} x {hm:.0f} m (use --map-size to change)")
        self.grid = OccupancyGrid(wm, hm, args.res)
        self.extent = self.grid.extent

        # navigation
        self.est = PoseEstimator()
        if args.planner:
            load_planner(args.planner)                                  # fail now, not in the planner process, if it can't load
        self.nav = Navigator(self.grid, args.planner, args.max_speed, args.max_turn, plan_res=args.plan_res,
                             inflate_m=args.inflate, replan_s=args.replan, lookahead=args.lookahead,
                             goal_tol=args.goal_tol, stop_dist=args.stop_dist)
        self.nav_goal_xy = None
        self.builtin = args.builtin_nav   # False: clicks are PUBLISHED as goals for an external planner
        self.ext_path = None
        self.last_pose_pub = 0.0
        self.last_panel = 0.0
        self.msg, self.msg_until = "", 0.0

        # route planning (space bar)
        self.inflate = clamp(args.inflate, 0.0, 1.5)   # keep-out distance (m): drawn in fuchsia, sent with every goal
        self.plan_mode = False                         # space pressed: map shows where the robot can go, clicks queue waypoints
        self.queue = []                                # waypoints queued in plan mode [(x, y), ...]
        self.route_shown = []                          # waypoints sent to the rover and not reached yet
        self.route_done = 0                            # how many of that route's waypoints are behind us (for the numbering)
        self.route_todo = []                           # built-in planner only: legs still to drive after the current one
        self.chain_after = 0.0                         # built-in planner only: do not start the next leg before this time
        self.drive_key, self.drive_mask, self.drive_drawn = None, None, None
        self._q_sig, self.q_texts = None, []

        # telemetry state
        self.rv = self.es = None
        self.gpos, self.g_ts, self.gfix = None, None, "No Fix"
        self.last_rts = None
        self.odom, self.odom_anchored, self.prev, self.last_seq = None, False, None, None
        self.o_yaw, self.yaw_ts, self.have_yaw, self.e_ts = 0.0, None, False, None
        self.v_meas = self.w_meas = 0.0
        self.g_trail, self.o_trail = deque(maxlen=args.trail), deque(maxlen=args.trail)
        self.follow = True
        self.tel = None                  # (x, y) best raw telemetry position (GNSS, else odometry)
        self.ghost = None                # [x, y, yaw] the robot as odometry + lidar say it is (what the sim/real robot does)
        # localisation: the robot's pose = wheel odometry + IMU yaw, corrected by lidar and (when good) GNSS.
        self.placed = False              # robot placed on the map (first GNSS fix of the --min-fix quality)
        self.yaw_off = 0.0               # correction added to the IMU yaw (from lidar)
        self.pend = [0.0, 0.0]           # GNSS correction still to be applied (slid in smoothly)
        self.anchored = False            # robot has been pulled onto the first Fixed RTK position
        self.gps_err = None              # distance GNSS is from the robot (m)
        self.loc = Localiser(self.grid)

        # lidar
        self.lidar_ang = self.lidar_dist = self.lidar_ts = None
        self.lidar_seq = self.map_seq = 0
        self.lidar_times = deque(maxlen=200)
        self.front_min = None
        self.lidar_near = 0.0            # fraction of the last scan closer than 0.3 m

        # goal robot [x, y, yaw, v, w] and the commands it follows
        self.goal = None
        self.disp = None                 # what is drawn: best estimate of the real pose NOW, low-passed so it glides
        self.goal_trail = deque(maxlen=args.trail)
        self.synced = False              # goal re-snapped to telemetry once the fix is good
        self.cmd_seen = (0.0, 0.0, -1e9)  # (linear, angular, arrival monotonic) of the latest cmd_vel on the bus
        self.t_last = None

        # redraw bookkeeping
        self.bg = None
        self.last_full = -1e9
        self.need_full = True

        # keyboard
        self.keys, self.released = set(), {}
        self.cmd, self.cmd_active = (0.0, 0.0), False

        self._build_figure()

    # ================================================================= figure
    def _build_figure(self):
        plt.style.use("dark_background")
        for k in list(plt.rcParams.keys()):                 # free up matplotlib's default key shortcuts
            if k.startswith("keymap.") and k != "keymap.quit":
                plt.rcParams[k] = []
        self.fig = plt.figure(figsize=(13, 9))
        try:
            self.fig.canvas.manager.set_window_title("UGV visualiser")
        except Exception:
            pass
        self.tkp = None
        if HAVE_TK:
            try:
                self.tkp = SidePanel(self.fig.canvas.manager.window, self._on_press, self._on_release)
            except Exception as e:
                print(f"(telemetry window unavailable: {e}; using the in-figure panel)")
                self.tkp = None
        self.ax = self.fig.add_axes([0.06, 0.07, 0.62, 0.88] if self.tkp is None else [0.055, 0.07, 0.93, 0.88])
        ax = self.ax
        self.map_im = ax.imshow(self.grid.image(), extent=self.extent, origin="upper", cmap="gray",
                                vmin=0, vmax=1, interpolation="nearest", zorder=0)
        self.drive_im = ax.imshow(np.zeros(self.grid.grid.shape + (4,), np.float32), extent=self.extent, origin="upper",
                                  interpolation="nearest", zorder=1)          # fuchsia: where the robot can go (space bar)
        self.drive_im.set_visible(False)
        l, r, b, t = self.extent
        ax.plot([l, r, r, l, l], [b, b, t, t, b], color="#555", lw=1, zorder=1)
        if self.a.show_world and self.world is not None:     # ground truth, for checking the learned map
            rgba = np.zeros(self.world.shape + (4,), np.float32)
            rgba[..., 0] = 1.0
            rgba[..., 3] = np.where(self.world > 127, 0.35, 0.0)
            ax.imshow(rgba, extent=self.extent, origin="upper", interpolation="nearest", zorder=1)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.25, zorder=1)
        ax.set_xlabel("East (m)")
        ax.set_ylabel("North (m)")
        ax.set_title("Learned map (grey = unknown, white = free, black = wall)  -  click to drive", fontsize=9)

        # slow layer: redrawn when new telemetry arrives (~1 Hz)
        self.g_line, = ax.plot([], [], "-", color="#2ecc71", lw=1.5, alpha=0.8, label="GNSS trail", zorder=3)
        self.o_line, = ax.plot([], [], "--", color="#e056fd", lw=1.5, alpha=0.9, label="Odometry trail", zorder=3)
        self.g_dot, = ax.plot([], [], "o", ms=7, mec="w", mew=1, zorder=6, label="GNSS position")
        self.o_dot, = ax.plot([], [], "D", ms=5, color="#e056fd", mec="w", mew=1, zorder=6, label="Odometry position")

        # fast layer (animated=True: blitted every frame)
        self.goal_line, = ax.plot([], [], "-", color="#4aa3ff", lw=1.0, alpha=0.7, label="Goal trail", zorder=2, animated=True)
        self.path_line, = ax.plot([], [], "-", color="#00e5ff", lw=2.0, alpha=0.9, label="Planned path", zorder=3, animated=True)
        self.goal_mark, = ax.plot([], [], "*", ms=16, color="#00e5ff", mec="w", mew=1, zorder=7, label="Nav goal", animated=True)
        self.lidar_sc = ax.scatter([], [], s=5, c="#ff4d4d", linewidths=0, zorder=4, label="LiDAR scan", animated=True)
        self.ghost_body = Polygon(np.zeros((4, 2)), closed=True, fc=GHOST_RGB + (0.30,), ec=GHOST_RGB + (0.95,),
                                  ls="--", lw=1.8, zorder=4, label="Robot (odometry + lidar)", animated=True)
        self.ghost_body.set_visible(False)
        ax.add_patch(self.ghost_body)
        self.ghost_head, = ax.plot([], [], "--", color=GHOST_RGB, lw=2, zorder=4, animated=True)
        self.goal_body = Polygon(np.zeros((4, 2)), closed=True, fc=GOAL_FC, ec="w", lw=1.5, zorder=5,
                                 label="Goal pose (smooth)", animated=True)
        self.goal_body.set_visible(False)
        ax.add_patch(self.goal_body)
        self.goal_head, = ax.plot([], [], "-", color="w", lw=2.5, zorder=5, animated=True)
        self.nav_text = ax.text(0.01, 0.012, "", transform=ax.transAxes, fontsize=9, color="#00e5ff", va="bottom",
                                family="monospace", bbox=dict(fc="black", ec="none", alpha=0.6, pad=3),
                                zorder=9, animated=True)
        self.q_line, = ax.plot([], [], "-", color=FUCHSIA, lw=1.6, alpha=0.9, zorder=7, animated=True)
        self.q_dots, = ax.plot([], [], "o", ms=13, mfc=FUCHSIA, mec="w", mew=1.2, zorder=8, animated=True)
        self.slow_artists = [self.map_im, self.drive_im, self.g_line, self.o_line, self.g_dot, self.o_dot]
        for a in self.slow_artists:                          # drawn at ~1 Hz into a cached layer, not on every frame
            a.set_animated(True)
        self.fast_artists = (self.goal_line, self.path_line, self.lidar_sc, self.ghost_body, self.ghost_head,
                             self.goal_body, self.goal_head, self.goal_mark, self.q_line, self.q_dots, self.nav_text)

        # colour key (a real matplotlib legend is slow to draw and clipped at this size)
        for i, (txt, col) in enumerate((("blue = predicted pose (smooth)", "#4aa3ff"), ("dashed = robot (odometry + lidar)", "#ffb347"),
                                        ("green = GNSS trail", "#2ecc71"), ("magenta = odometry", "#e056fd"),
                                        ("cyan = planned path", "#00e5ff"), ("red dots = lidar", "#ff4d4d"),
                                        ("fuchsia = drivable (press space)", FUCHSIA))):
            ax.text(0.99, 0.985 - i * 0.03, txt, transform=ax.transAxes, ha="right", va="top", fontsize=7, color=col,
                    bbox=dict(fc="black", ec="none", alpha=0.5, pad=1))
        ax.set_xlim(-6, 6)
        ax.set_ylim(-6, 6)

        # fallback only (no Tk): panel inside the figure, one text object per colour
        self.panel_txt = {}
        self.panel_ax = None
        if self.tkp is None:
            self.panel_ax = self.fig.add_axes([0.70, 0.04, 0.29, 0.92])
            self.panel_ax.axis("off")
        c = self.fig.canvas
        c.mpl_connect("key_press_event", self._on_press)
        c.mpl_connect("key_release_event", self._on_release)
        c.mpl_connect("button_press_event", self._on_click)
        c.mpl_connect("draw_event", self._on_draw)
        c.mpl_connect("close_event", self._on_close)

        self.ctl_timer = c.new_timer(interval=50)           # command publisher: 20 Hz
        self.ctl_timer.add_callback(self._control_tick)
        self.ctl_timer.start()
        self.frame_timer = c.new_timer(interval=max(5, int(1000 / self.a.fps)))
        self.frame_timer.add_callback(self.tick)
        self.frame_timer.start()

    def _on_close(self, _ev):
        self.nav.close()
        if self.cmd_active and self.a.drive:                 # leave the robot stopped
            try:
                self.pub.send("cmd_vel", linear=0.0, angular=0.0)
            except Exception:
                pass

    # ================================================================= blitting
    def _on_draw(self, _ev):
        # layers: static (axes, ticks, legend)  ->  slow (map, trails, panel; ~1 Hz)  ->  fast (robots, scan; every frame)
        cv = self.fig.canvas
        self.bg_static = cv.copy_from_bbox(self.fig.bbox)
        self._draw_slow()
        self.bg = cv.copy_from_bbox(self.fig.bbox)
        self._draw_fast()

    def _draw_slow(self):
        for a in self.slow_artists:
            self.ax.draw_artist(a)
        for t in self.panel_txt.values():
            self.panel_ax.draw_artist(t)

    def _refresh_slow(self):
        cv = self.fig.canvas
        cv.restore_region(self.bg_static)
        self._draw_slow()
        self.bg = cv.copy_from_bbox(self.fig.bbox)
        self._blit()

    def _draw_fast(self):
        for a in self.fast_artists:
            self.ax.draw_artist(a)
        for t in self.q_texts:
            self.ax.draw_artist(t)

    def _blit(self):
        cv = self.fig.canvas
        cv.restore_region(self.bg)
        self._draw_fast()
        cv.blit(self.fig.bbox)

    # ================================================================= input
    def say(self, text, secs=4.0):
        self.msg, self.msg_until = text, time.monotonic() + secs

    def _on_press(self, ev):
        k = ev.key
        if k in ARROWS:
            self.keys.add(k)
            self.released.pop(k, None)
        elif k == " ":
            self.toggle_plan()
        elif k in ("x", "escape"):
            self.keys.clear()
            self.released.clear()
            self.plan_mode, self.queue = False, []
            self.need_full = True
            self.cancel_nav()
            self.say("stopped: navigation and route cancelled")
        elif k in ("[", "]"):
            self.change_inflate(-1 if k == "[" else 1)
        elif k == "c":
            self.g_trail.clear()
            self.o_trail.clear()
            self.goal_trail.clear()
            self.need_full = True
        elif k == "f":
            self.follow = not self.follow
            self._lim = None
            self.need_full = True
        elif k == "o":
            if self.gpos is not None and self.odom is not None:
                self.odom = list(self.gpos)
                self.o_trail.clear()
                self.need_full = True
        elif k == "g":
            self.snap_goal()
        elif k == "r":
            self.grid.reset()
            self.need_full = True
            self.say("map reset")
        elif k == "s":
            try:
                self.grid.save_png(self.a.save_map)
                self.say(f"saved {os.path.abspath(self.a.save_map)}", 6)
                print(f"[MAP] saved {os.path.abspath(self.a.save_map)}")
            except OSError as e:
                self.say(f"could not save map: {e}", 6)

    def _on_release(self, ev):
        if ev.key in ARROWS:
            self.released[ev.key] = time.monotonic()     # deferred: key auto-repeat sends release+press pairs

    def _on_click(self, ev):
        if ev.inaxes is not self.ax or ev.xdata is None:
            return
        tb = getattr(self.fig.canvas, "toolbar", None)
        if tb is not None and str(getattr(tb, "mode", "")):   # zoom / pan tool active: not a drive click
            return
        if self.plan_mode:                                    # planning a route: clicks queue waypoints
            if ev.button == 1:
                self.add_waypoint(ev.xdata, ev.ydata)
            elif ev.button == 3:
                self.undo_waypoint()
            return
        if ev.button == 3:
            self.cancel_nav()
            self.say("navigation cancelled")
        elif ev.button == 1:
            self.start_nav(ev.xdata, ev.ydata)

    def start_nav(self, x, y):
        if not self.a.drive:
            self.say("view-only mode (--no-drive): not driving")
            return
        self.keys.clear()
        try:
            self.pub.send("goal", action="go", x=x, y=y, inflate=float(self.inflate))   # always tell the bus; an external planner picks it up
        except Exception as e:
            self.say(f"could not publish the goal: {e}", 6)
        self.nav_goal_xy = (x, y)
        if not self.builtin:
            self.ext_path = None
            self.say(f"goal ({x:+.1f}, {y:+.1f}) sent on the bus (port 5008) for your planner")
            return
        pose, why = self.nav_pose()
        if pose is None:
            self.say(f"can't navigate yet: {why}", 6)
            return
        self.nav.set_goal(x, y, pose)
        self.say(f"driving to ({x:+.1f}, {y:+.1f})")

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

    # ================================================================= route planning (space bar)
    def drivable(self):
        """Boolean map of where the robot's centre can go, cached until the map or the keep-out distance changes."""
        key = (self.grid.scans, round(self.inflate, 3), self.a.plan_res)
        if key != self.drive_key:
            self.drive_key = key
            self.drive_mask = drivable_area(self.grid, self.inflate, self.a.plan_res)
        return self.drive_mask

    def toggle_plan(self):
        if not self.a.drive:
            self.say("view-only mode (--no-drive): not driving")
            return
        if not self.plan_mode:
            self.plan_mode, self.queue = True, []
            self.need_full = True
            self.say("PLAN: click inside the fuchsia to queue waypoints  |  right-click = undo  |  space = go  |  x = abort", 10)
            return
        self.plan_mode = False
        self.need_full = True
        queued, self.queue = list(self.queue), []
        if queued:
            self.send_route(queued)
        else:
            self.say("plan mode off (nothing was queued)")

    def add_waypoint(self, x, y):
        mask = self.drivable()
        row, col = self.grid.to_cell(x, y)
        if not self.grid.inside(row, col) or not mask[row, col]:
            self.say(f"can't go there: not seen as free, or within {self.inflate:.2f} m of a wall", 4)
            return
        self.queue.append((float(x), float(y)))
        self.say(f"waypoint {len(self.queue)} queued ({x:+.1f}, {y:+.1f})  |  space = go")

    def undo_waypoint(self):
        if self.queue:
            self.queue.pop()
            self.say(f"removed the last waypoint; {len(self.queue)} queued")
        else:
            self.say("nothing queued  |  space leaves plan mode, x stops the robot")

    def change_inflate(self, direction):
        step = self.a.plan_res if self.a.plan_res > 0 else 0.05
        self.inflate = clamp(round(self.inflate + direction * step, 3), 0.0, 1.5)
        self.drive_key = None
        self.need_full = True
        note = "  (the built-in planner keeps the --inflate it started with)" if self.builtin else ""
        if self.plan_mode and self.queue:                         # waypoints that are no longer reachable are dropped
            mask = self.drivable()
            keep = []
            for (qx, qy) in self.queue:
                r, c = self.grid.to_cell(qx, qy)
                if self.grid.inside(r, c) and mask[r, c]:
                    keep.append((qx, qy))
            dropped, self.queue = len(self.queue) - len(keep), keep
            if dropped:
                note += f"  ({dropped} waypoint{'s' if dropped != 1 else ''} dropped: no longer reachable)"
        self.say(f"keep-out distance {self.inflate:.2f} m, sent with your next goal{note}", 6)

    def send_route(self, pts):
        self.keys.clear()
        try:
            self.pub.send("goal", action="route", route=[[x, y] for x, y in pts], inflate=float(self.inflate))
        except Exception as e:
            self.say(f"could not send the route: {e}", 6)
            return
        self.nav_goal_xy = pts[-1]
        self.route_shown, self.route_done = list(pts), 0
        n = len(pts)
        if not self.builtin:
            self.ext_path = None
            self.say(f"route of {n} waypoint{'s' if n != 1 else ''} sent on the bus (port 5008) for your planner", 6)
            return
        pose, why = self.nav_pose()
        if pose is None:
            self.say(f"can't navigate yet: {why}", 6)
            return
        self.route_todo = list(pts[1:])
        self.nav.set_goal(pts[0][0], pts[0][1], pose)
        self.chain_after = time.monotonic() + 1.5
        self.say(f"driving a route of {n} waypoint{'s' if n != 1 else ''}")

    def _route_progress(self):
        """Tick waypoints off as the robot reaches them; for the built-in planner also start the next leg."""
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

    def nav_pose(self):
        """Pose to navigate/map from, or (None, why)."""
        now = time.time()
        if not self.placed or not self.est.has_fix():
            return None, f"waiting for a {self.a.min_fix} fix to place the robot (have {self.gfix})"
        age = self.est.fix_age(now)
        if age is None or age > self.esp_stale:
            return None, "odometry data is stale"
        return self.est.pose_at(now), ""

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
        t = time.monotonic()
        if t - self.last_pose_pub >= 0.1:                    # tell an external planner where the robot is (10 Hz)
            self.last_pose_pub = t
            pose, _why = self.nav_pose()
            if pose is not None:
                try:
                    self.pub.send("pose", x=pose[0], y=pose[1], yaw=pose[2], fix=self.gfix)
                except Exception:
                    pass
        if cmd is not None:
            self.pub.send("cmd_vel", linear=cmd[0], angular=cmd[1])
            self.cmd, self.cmd_active = cmd, True
        elif self.cmd_active:                               # just stopped: one final zero
            self.pub.send("cmd_vel", linear=0.0, angular=0.0)
            self.cmd, self.cmd_active = (0.0, 0.0), False

    # ================================================================= goal robot
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
            cv = 0.0                         # a wall right in front: the real robot can't drive on, so the model must not either
        g[3] += clamp(cv - g[3], -self.a.lin_accel * dt, self.a.lin_accel * dt)
        g[4] += clamp(cw - g[4], -self.a.ang_accel * dt, self.a.ang_accel * dt)
        mid = g[2] + g[4] * dt / 2
        g[0] += g[3] * math.cos(mid) * dt
        g[1] += g[3] * math.sin(mid) * dt
        g[2] = wrap(g[2] + g[4] * dt)
        self.est.push_model(now, g[0], g[1], g[2])
        # drawn pose = telemetry + the model's motion since the sample (so it can't wander off), glided at 30 fps
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
        """Put the goal robot exactly where the telemetry says the robot is."""
        if not self.est.has_fix():
            return
        t_pos, x, y, t_yaw, yaw = self.est.fix
        self.goal = [x, y, yaw, 0.0, 0.0]
        self.disp = [x, y, yaw]
        self.est.reset_model()
        self.est.push_model(time.time(), x, y, yaw)
        self.goal_trail.clear()

    def footprint(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        hl, hw = self.a.robot_length / 2, self.a.robot_width / 2
        poly = [(x + px * c - py * s, y + px * s + py * c) for px, py in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]
        return poly, ([x, x + (hl + 0.15) * c], [y, y + (hl + 0.15) * s])

    # ================================================================= bus messages
    def to_local(self, lat, lon):
        lat0, lon0 = self.ref
        return (lon - lon0) * M_PER_DEG * math.cos(math.radians(lat0)), (lat - lat0) * M_PER_DEG

    def _shift_fix(self, dx, dy, dyaw):
        t_pos, x, y, t_yaw, yaw = self.est.fix
        self.est.set_fix(t_pos, x + dx, y + dy, t_yaw, wrap(yaw + dyaw))
        self.yaw_off += dyaw

    def _gps_fuse(self, rank):
        """GNSS never moves the robot directly (it jitters). Good fixes pull it gently; the first Fixed RTK
        fix pulls it all the way onto the GNSS position, smoothly."""
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
            if self.gps_err > 2.0:                                   # a Fixed RTK jump this big is more likely a glitch
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
            print(f"Map centre anchored at first fix: {m['lat']:.7f}, {m['lon']:.7f}")
        self.gpos, self.g_ts = self.to_local(m["lat"], m["lon"]), m["ts"]
        self.g_trail.append(self.gpos)
        rank = FIX_RANK.get(m["fix"], 0)
        if not self.placed:
            if rank >= self.min_rank and self.yaw_ts is not None:     # put the robot where GNSS says, once, then odometry takes over
                self.placed = True
                self.yaw_off = 0.0
                self.est.set_fix(self.g_ts, self.gpos[0], self.gpos[1], self.yaw_ts, self.o_yaw)
                self.say(f"robot placed at the {m['fix']} position; odometry + lidar take over", 6)
        elif self.est.has_fix():
            self._gps_fuse(rank)

    def on_esp(self, m):
        self.es = m
        seq = m["seq"]
        if seq == self.last_seq:
            return
        if self.last_seq is not None and seq < self.last_seq:     # sender restarted
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
                base = pyaw if pyaw is not None else yaw
                mid = base + wrap(yaw - base) / 2
                self.o_yaw = yaw
            else:
                dth = (dr - dl) / self.a.wheel_base
                mid = self.o_yaw + dth / 2
                self.o_yaw += dth
            self.odom[0] += d * math.cos(mid)
            self.odom[1] += d * math.sin(mid)
            if self.placed and self.est.has_fix():                     # the robot moves by what the wheels/IMU measured
                _, fx, fy, _, _ = self.est.fix
                hm = mid + self.yaw_off
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

    # ================================================================= frame
    def tick(self):
        t_mono = time.monotonic()
        dt = 1.0 / self.a.fps if self.t_last is None else min(t_mono - self.t_last, 0.5)
        self.t_last = t_mono
        now = time.time()

        new = False
        for m in self.sub.poll():
            t = m["type"]
            if t == "rtk":
                self.on_rtk(m)
                new = True
            elif t == "esp":
                self.on_esp(m)
                new = True
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

        # GNSS corrections slide in over about a second instead of jumping
        if self.pend[0] or self.pend[1]:
            k = 1 - math.exp(-dt / 1.0)
            sx, sy = self.pend[0] * k, self.pend[1] * k
            self.pend[0] -= sx
            self.pend[1] -= sy
            if fx:
                self._shift_fix(sx, sy, 0.0)
            if abs(self.pend[0]) + abs(self.pend[1]) < 1e-4:
                self.pend = [0.0, 0.0]

        # the goal (smooth) robot appears when the robot is placed
        if self.goal is None and fx:
            self.snap_goal()
        self.step_goal(dt, now)

        if fx:                                                      # ghost = the robot per odometry + lidar (does not bounce with GNSS)
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
        self._set_fast_artists(now)

        if self.tkp is not None and t_mono - self.last_panel >= 0.25:       # separate window: costs the map nothing
            self.last_panel = t_mono
            self._draw_panel(g_age, e_age)

        since = t_mono - self.last_full
        if self.bg is None or self.need_full or since >= self.a.slow_refresh:
            force = self.bg is None or self.need_full
            self.last_full, self.need_full = t_mono, False
            moved = self._update_slow(g_age, e_age)
            if force or moved:
                self.fig.canvas.draw()
            else:
                self._refresh_slow()
        else:
            self._blit()

    def _nav_status(self):
        if self.builtin:
            return self.nav.status()
        if self.nav_goal_xy is None:
            return "external planner: click the map to send a goal"
        n = len(self.ext_path) if self.ext_path else 0
        return (f"goal ({self.nav_goal_xy[0]:+.1f}, {self.nav_goal_xy[1]:+.1f}) -> planner, "
                + (f"path {n} pts" if n else "no path received yet"))

    def _maybe_map(self, now):
        """Fold the newest lidar scan into the map, at the pose the robot had when the scan was taken."""
        if self.lidar_dist is None or self.lidar_seq == self.map_seq:
            return
        self.map_seq = self.lidar_seq
        if now - self.lidar_ts > LIDAR_STALE_S:
            return
        if self.lidar_near > 0.6:                                   # walls all round at 0.2 m = a bad scan / bad world image
            return
        pose, _why = self.nav_pose()                                # also enforces fix quality + fresh GNSS
        if pose is None:
            return
        pose = self.est.pose_at(self.lidar_ts)
        if not self.a.no_localise:                                  # line the scan up with the map first, then add it to the map
            self.loc.refresh()
            r = self.loc.match(pose, self.lidar_ang, self.lidar_dist)
            if r and (r[0] or r[1] or r[2]):
                self._shift_fix(0.3 * r[0], 0.3 * r[1], 0.3 * r[2])
                pose = self.est.pose_at(self.lidar_ts)
        self.grid.integrate_scan(pose, self.lidar_ang, self.lidar_dist, free_range=self.a.free_range)

    def _set_fast_artists(self, now):
        for line, tr in ((self.goal_line, self.goal_trail),):
            if tr:
                xs, ys = zip(*tr)
                line.set_data(xs, ys)
            else:
                line.set_data([], [])
        shown = self.nav.path if self.builtin else self.ext_path
        if shown:
            xs, ys = zip(*shown)
            self.path_line.set_data(xs, ys)
        else:
            self.path_line.set_data([], [])
        if self.nav_goal_xy and (self.nav.active or not self.builtin):
            self.goal_mark.set_data([self.nav_goal_xy[0]], [self.nav_goal_xy[1]])
        else:
            self.goal_mark.set_data([], [])

        if self.ghost is not None and self.tel is not None:
            poly, head = self.footprint(*self.ghost)
            self.ghost_body.set_xy(poly)
            self.ghost_body.set_visible(True)
            self.ghost_head.set_data(*head)
        else:
            self.ghost_body.set_visible(False)
            self.ghost_head.set_data([], [])
        if self.disp is not None:
            poly, head = self.footprint(*self.disp)
            self.goal_body.set_xy(poly)
            self.goal_body.set_visible(True)
            self.goal_head.set_data(*head)
        else:
            self.goal_body.set_visible(False)
            self.goal_head.set_data([], [])

        pts = np.empty((0, 2))
        if self.lidar_dist is not None and now - self.lidar_ts <= LIDAR_STALE_S and self.est.has_fix():
            x, y, yaw = self.est.pose_at(self.lidar_ts)
            bearing = yaw - np.radians(self.lidar_ang)
            d = self.lidar_dist / 1000.0
            pts = np.column_stack((x + d * np.cos(bearing), y + d * np.sin(bearing)))
        self.lidar_sc.set_offsets(pts)

        text = self.msg if time.monotonic() < self.msg_until else ""
        status = self._nav_status() if (self.nav.active or self.nav.state in ("arrived",) or not self.builtin) else ""
        if self.plan_mode:                                           # two lines, so a message never runs off the edge
            n = len(self.queue)
            status = (f"PLAN: {n} queued, keep-out {self.inflate:.2f} m  |  click = add   right-click = undo   "
                      "space = go   x = abort")
            self.nav_text.set_text("\n".join(s for s in (text, status) if s))
        else:
            self.nav_text.set_text("   ".join(s for s in (status, text) if s)
                                   or "click the map to drive there  |  space: plan a route  |  arrows: teleop")
        self._set_queue_artists()

    def _set_queue_artists(self):
        """Numbered fuchsia waypoints: the ones being queued in plan mode, else the ones the rover has still to reach."""
        if self.plan_mode:
            pts, first = self.queue, 1
        else:
            pts, first = self.route_shown, self.route_done + 1
        sig = (tuple(pts), first)
        if sig != self._q_sig:                                       # labels are rebuilt only when the list changes
            self._q_sig = sig
            for t in self.q_texts:
                try:
                    t.remove()
                except Exception:
                    pass
            self.q_texts = [self.ax.text(x, y, str(first + i), color="white", fontsize=8, fontweight="bold",
                                         ha="center", va="center", zorder=9, animated=True)
                            for i, (x, y) in enumerate(pts)]
        if pts:
            xs, ys = [p[0] for p in pts], [p[1] for p in pts]
            if self.disp is not None:                                # the line starts at the robot
                self.q_line.set_data([self.disp[0]] + xs, [self.disp[1]] + ys)
            else:
                self.q_line.set_data(xs, ys)
            self.q_dots.set_data(xs, ys)
        else:
            self.q_line.set_data([], [])
            self.q_dots.set_data([], [])

    def _update_slow(self, g_age, e_age):
        self.map_im.set_data(self.grid.image())
        if self.plan_mode:
            mask = self.drivable()
            if self.drive_drawn != self.drive_key:
                rgba = np.zeros(mask.shape + (4,), np.float32)
                rgba[mask] = FUCHSIA_RGBA
                self.drive_im.set_data(rgba)
                self.drive_drawn = self.drive_key
            self.drive_im.set_visible(True)
        else:
            self.drive_im.set_visible(False)
        for line, tr in ((self.g_line, self.g_trail), (self.o_line, self.o_trail)):
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
        moved = False
        if self.follow:
            target = (self.disp[0], self.disp[1]) if self.disp is not None else self.tel
            if target is not None:
                # re-centre only when the robot leaves the middle of the view: re-drawing the axes is the slow part
                half = self.a.view / 2
                (x0, x1), (y0, y1) = self.ax.get_xlim(), self.ax.get_ylim()
                cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
                if (abs(x1 - x0 - 2 * half) > 1e-6 or abs(target[0] - cx) > 0.3 * half
                        or abs(target[1] - cy) > 0.3 * half):
                    self.ax.set_xlim(target[0] - half, target[0] + half)
                    self.ax.set_ylim(target[1] - half, target[1] + half)
                    moved = True
        else:
            l, r, b, t = self.extent
            lim = (l - 0.5, r + 0.5, b - 0.5, t + 0.5)
            if getattr(self, "_lim", None) != lim:
                self._lim = lim
                self.ax.set_xlim(lim[0], lim[1])
                self.ax.set_ylim(lim[2], lim[3])
                moved = True
        if self.tkp is None:
            self._draw_panel(g_age, e_age)
        return moved

    def _draw_panel(self, g_age, e_age):
        rows = self._panel_rows(g_age, e_age)
        if self.tkp is not None:
            self.tkp.set_rows(rows)
            return
        self._panel_in_figure(rows)

    def _panel_rows(self, g_age, e_age):
        rv, es = self.rv, self.es
        rows = [("GNSS  (rtk, port 5007)", HDR)]
        if not rv:
            rows.append(("  NO DATA", RED))
        else:
            stale = g_age is not None and g_age > self.gnss_stale
            rows += [(f"  fix    {self.gfix}", FIX_COLORS.get(self.gfix, WHITE)),
                     (f"  sats   {rv['sats']}    hdop {rv['hdop']}", WHITE),
                     (f"  corr   {rv['corr_age']} s", WHITE),
                     (f"  lat    {rv['lat']:.7f}", WHITE),
                     (f"  lon    {rv['lon']:.7f}", WHITE),
                     (f"  age    {g_age:.1f} s{'  STALE' if stale else ''}", RED if stale else GREY)]

        rows += [("", WHITE), ("POSE  (m from map centre)", HDR)]
        fx = self.est.fix if (self.placed and self.est.has_fix()) else None
        if fx:
            rows.append((f"  ROBOT  x={fx[1]:+8.2f}  y={fx[2]:+8.2f}   (odom + lidar)", WHITE))
        else:
            rows.append((f"  ROBOT  not placed yet: waiting for {self.a.min_fix}", AMBER))
        rows.append((f"  GNSS   x={self.gpos[0]:+8.2f}  y={self.gpos[1]:+8.2f}", WHITE) if self.gpos is not None
                    else ("  GNSS   no position", GREY))
        if self.gps_err is not None and fx:
            col = GREEN if self.gps_err < 0.3 else (AMBER if self.gps_err < 1.5 else RED)
            rows.append((f"  GNSS is {self.gps_err:.2f} m from the robot" + ("" if self.anchored else "  (not anchored)"), col))
        if self.odom is not None:
            rows.append((f"  ODOM   x={self.odom[0]:+8.2f}  y={self.odom[1]:+8.2f}   (wheels only)", GREY))
        else:
            rows.append(("  ODOM   no data", GREY))
        rows.append((f"  yaw    {math.degrees(fx[4]) if fx else math.degrees(self.o_yaw):+7.1f} deg"
                     + ("" if self.have_yaw else "  (from enc)")
                     + (f"  (lidar fix {math.degrees(self.yaw_off):+.1f})" if abs(self.yaw_off) > 1e-3 else ""), WHITE))
        if not self.a.no_localise:
            rows.append((f"  lidar  {self.loc.status}", GREY if "poor" in self.loc.status or "waiting" in self.loc.status else GREEN))
        if self.tel is not None:
            st = self.grid.state_at(*self.tel)
            rows.append((f"  map    {st}", RED if st in ("OCCUPIED", "off map") else WHITE))

        rows += [("", WHITE), ("ESP32  (esp, port 5006)", HDR)]
        if not es:
            rows.append(("  NO DATA", RED))
        else:
            enc, imu = es["enc"], es["imu"]
            stale = e_age is not None and e_age > self.esp_stale
            rows += [(f"  seq    {es['seq']}", WHITE),
                     (f"  L ticks {enc['left_ticks']}", WHITE),
                     (f"  R ticks {enc['right_ticks']}", WHITE),
                     (f"  speed  {self.v_meas:+.2f} m/s", WHITE),
                     (f"  gyro z {imu['gyro'][2]:+.3f} rad/s", WHITE),
                     (f"  accel  {imu['accel'][0]:+.2f} {imu['accel'][1]:+.2f} {imu['accel'][2]:+.2f}", WHITE),
                     (f"  age    {e_age:.1f} s{'  STALE' if stale else ''}", RED if stale else GREY)]

        rows += [("", WHITE), ("SPEED", HDR)]
        if self.goal is not None:
            rows.append((f"  predicted {self.goal[3]:+.2f}   measured {self.v_meas:+.2f} m/s", WHITE))
        else:
            rows.append(("  waiting for the robot to be placed", GREY))

        rows += [("", WHITE), ("LIDAR  (port 5005)", HDR)]
        if self.lidar_ts is None:
            rows.append(("  waiting for scans...", AMBER))
        else:
            age = time.time() - self.lidar_ts
            mono = time.monotonic()
            rate = sum(1 for t in self.lidar_times if mono - t <= 2.0) / 2.0
            rows.append((f"  points {len(self.lidar_dist)}   rate {rate:.1f} Hz", WHITE))
            rows.append((f"  front  {'-' if self.front_min is None else f'{self.front_min:.2f} m'}"
                         f"   age {age:.1f} s{'  STALE' if age > LIDAR_STALE_S else ''}", RED if age > LIDAR_STALE_S else WHITE))
            if self.lidar_near > 0.6:
                rows.append((f"  BAD SCAN: {self.lidar_near * 100:.0f}% of points < 0.3 m,", RED))
                rows.append(("  not mapped. Sim: wrong --invert?", RED))

        known, walls = self.grid.stats()
        pose, why = self.nav_pose()
        rows += [("", WHITE), ("MAP  (built from lidar)", HDR),
                 (f"  known  {known:.1f} %   walls {walls} cells", WHITE),
                 (f"  scans  {self.grid.scans}", WHITE),
                 ("  mapping ON" if pose is not None else f"  mapping OFF: {why}", WHITE if pose is not None else AMBER)]

        rows += [("", WHITE), ("NAVIGATION", HDR), (f"  {self._nav_status()[:44]}", WHITE)]
        rows.append((f"  keep-out {self.inflate:.2f} m   ( [ and ] change it )", WHITE))
        if self.plan_mode:
            rows.append((f"  PLAN MODE: {len(self.queue)} queued, space = go", FUCHSIA))
        elif self.route_shown:
            rows.append((f"  route: {len(self.route_shown)} waypoint(s) still to reach", FUCHSIA))
        if self.builtin and (self.nav.active or self.nav.state in ("arrived", "no path")):
            rows.append((f"  plan   {self.nav.plan_ms:.0f} ms, {len(self.nav.path) if self.nav.path else 0} waypoints", WHITE))

        rows += [("", WHITE), ("COMMAND  (cmd_vel on the bus)", HDR)]
        lin, ang, t = self.cmd_seen
        live = time.monotonic() - t <= cfg.CMD_TIMEOUT
        rows.append((f"  v={lin:+.2f} m/s  w={ang:+.2f} rad/s" + ("" if live else "  (idle)"), GREEN if live and (lin or ang) else WHITE))
        rows.append((f"  limits {self.a.max_speed:g} m/s, {self.a.max_turn:g} rad/s", GREY))
        rows += [("", WHITE),
                 ("arrows drive   x / esc / right-click = stop", GREY),
                 ("space = plan a route (fuchsia = reachable)", GREY),
                 ("[ ] = keep-out distance", GREY),
                 ("[g] snap goal  [f] follow/full  [c] clear trails", GREY),
                 ("[o] odom  [r] reset map  [s] save map", GREY)]
        if self.sub.bad:
            rows.append((f"  {self.sub.bad} bad bus messages dropped: {self.sub.last_error[:30]}", AMBER))
        return rows

    def _panel_in_figure(self, rows):
        for col in {c for _, c in rows} | set(self.panel_txt):
            if col not in self.panel_txt:
                self.panel_txt[col] = self.panel_ax.text(0.0, 1.0, "", family="monospace", fontsize=8, va="top",
                                                         linespacing=1.55, transform=self.panel_ax.transAxes, color=col,
                                                         animated=True)
            self.panel_txt[col].set_text("\n".join(txt if c == col else "" for txt, c in rows))


def main():
    ap = argparse.ArgumentParser(description="Live robot visualiser + teleop + click-to-drive (UDP bus client)")
    ap.add_argument("--no-drive", action="store_true", help="view only: arrow keys and clicks do nothing, never publishes")
    ap.add_argument("--cmap", help="hidden world image: only used for the map SIZE and --show-world (default: cmap.png near the script / ugv/ / cwd)")
    ap.add_argument("--invert", action="store_true", help="the world image is black = free, white = wall")
    ap.add_argument("--show-world", action="store_true", help="overlay the hidden world (cmap.png walls) in translucent red")
    ap.add_argument("--map-size", type=float, default=40.0, help="learned map width/height in m when there is no cmap.png (default 40)")
    ap.add_argument("--res", type=float, default=cfg.MAP_RES, help="metres per map cell (default from common/config.py)")
    ap.add_argument("--ref-lat", type=float, default=cfg.ORIGIN_LAT, help="latitude of the map centre (default: common/config.py)")
    ap.add_argument("--ref-lon", type=float, default=cfg.ORIGIN_LON, help="longitude of the map centre")
    ap.add_argument("--anchor-first-fix", action="store_true", help="put the map centre at the first GNSS fix instead")
    ap.add_argument("--min-fix", default="Float RTK", choices=["3D Fix", "DGNSS", "Float RTK", "Fixed RTK"],
                    help="worst fix quality allowed for mapping and navigation (default Float RTK)")
    ap.add_argument("--free-range", type=float, default=6.0,
                    help="metres of free space to mark along lidar beams that return nothing (open ground). 0 = off (default 6)")
    ap.add_argument("--no-localise", action="store_true", help="don't correct the robot pose with the lidar")
    ap.add_argument("--no-gps-fusion", action="store_true", help="don't let GNSS correct the robot after it is placed (testing)")
    ap.add_argument("--builtin-nav", action="store_true",
                    help="drive with the built-in planner in this program (testing). Default: only PUBLISH the clicked goal "
                         "(port 5008) and let your own planner publish cmd_vel and a path")
    ap.add_argument("--planner", help="your path planner as module:function (default: built-in Theta*; see common/planner.py)")
    ap.add_argument("--plan-res", type=float, default=0.10, help="planning grid cell size, m (default 0.10)")
    ap.add_argument("--inflate", type=float, default=0.40,
                    help="keep the robot's centre this far from walls, m (default 0.40). Shown in fuchsia on space, sent with "
                         "every goal; [ and ] change it while running")
    ap.add_argument("--replan", type=float, default=1.0, help="replan every this many seconds while driving (default 1)")
    ap.add_argument("--lookahead", type=float, default=0.6, help="path-following lookahead, m")
    ap.add_argument("--goal-tol", type=float, default=0.25, help="arrived when this close, m")
    ap.add_argument("--stop-dist", type=float, default=0.45, help="stop if the lidar sees something this close ahead, m")
    ap.add_argument("--max-speed", type=float, default=cfg.MAX_SPEED, help="teleop AND navigation speed, m/s (default common/config.py)")
    ap.add_argument("--max-turn", type=float, default=cfg.MAX_TURN, help="teleop AND navigation turn rate, rad/s")
    ap.add_argument("--lin-accel", type=float, default=cfg.LIN_ACCEL, help="goal-robot acceleration limit (match the robot)")
    ap.add_argument("--ang-accel", type=float, default=cfg.ANG_ACCEL)
    ap.add_argument("--gnss-hz", type=float, default=1.0, help="expected rtk rate (only sets the 'stale' threshold)")
    ap.add_argument("--esp-hz", type=float, default=1.0, help="expected esp rate (only sets the 'stale' threshold)")
    ap.add_argument("--ghost-smooth", type=float, default=0.3, help="seconds the ghost glides to each sample; 0 = jump")
    ap.add_argument("--lidar-yaw-offset", type=float, default=0.0, help="degrees between the lidar's 0 and the robot's front")
    ap.add_argument("--save-map", default="learned_map.png", help="file the [s] key saves the learned map to")
    ap.add_argument("--view", type=float, default=12.0, help="follow-view window size in m (default 12)")
    ap.add_argument("--trail", type=int, default=2000, help="max trail points")
    ap.add_argument("--slow-refresh", type=float, default=MAX_FULL_S,
                    help="seconds between redraws of the map and trails (default 1; try 5 if the window stutters). "
                         "Robots, lidar scan and path still move at --fps; the telemetry window updates 4x a second regardless")
    ap.add_argument("--fps", type=float, default=30.0, help="goal/ghost/scan animation rate")
    ap.add_argument("--robot-length", type=float, default=cfg.ROBOT_LENGTH)
    ap.add_argument("--robot-width", type=float, default=cfg.ROBOT_WIDTH)
    ap.add_argument("--wheel-base", type=float, default=cfg.WHEEL_BASE)
    ap.add_argument("--wheel-diameter", type=float, default=cfg.WHEEL_DIAMETER)
    ap.add_argument("--ticks-per-rev", type=int, default=cfg.TICKS_PER_REV)
    args = ap.parse_args()
    args.drive = not args.no_drive
    args.cmap_path = find_cmap(args)

    viz = Visualiser(args)
    plt.show()
    del viz


if __name__ == "__main__":
    main()
