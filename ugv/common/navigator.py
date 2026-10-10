"""
Navigator: "drive to that point". Plans a path on the learned map, follows it, and replans as the
map fills in (unknown space counts as free, so the robot explores towards the goal and routes around
walls it discovers).

Planning runs in a separate PROCESS (not a thread): a busy Python thread would starve the GUI of the
interpreter lock and freeze the window while it searches. update() is cheap: call it ~20 Hz
with the robot's pose; it returns the (speed, turn rate) to command, or None when not navigating.

Layout: ugv/common/navigator.py
"""

import multiprocessing as mp
import threading
import time

from .follower import PurePursuit
from .planner import load_planner, plan_world


def _plan_process(conn, planner_spec):
    """Runs in the child process: receive a map snapshot + start/goal, send back the path."""
    try:
        planner = load_planner(planner_spec)
        err = ""
    except Exception as e:                                   # a bad --planner must not kill the app
        planner, err = None, f"could not load planner {planner_spec!r}: {e}"
    while True:
        try:
            req = conn.recv()
        except (EOFError, OSError):
            return
        if req is None:
            return
        token, snap, left, top, res, start, goal, plan_res, inflate_m = req
        t0 = time.perf_counter()
        if planner is None:
            path, reason = None, err
        else:
            try:
                path, reason = plan_world(snap, left, top, res, start, goal, plan_res, inflate_m, planner)
            except Exception as e:
                path, reason = None, f"planner error: {e}"
        conn.send((token, path, reason, (time.perf_counter() - t0) * 1000))


class Navigator:
    def __init__(self, grid, planner_spec, max_speed, max_turn, plan_res=0.10, inflate_m=0.40,
                 replan_s=1.0, lookahead=0.6, goal_tol=0.25, stop_dist=0.45):
        self.grid, self.planner_spec = grid, planner_spec
        self._proc = self._conn = None
        self.plan_res, self.inflate_m, self.replan_s, self.stop_dist = plan_res, inflate_m, replan_s, stop_dist
        self.follower = PurePursuit(max_speed, max_turn, lookahead, goal_tol)
        self.goal = None
        self.path = None                  # latest path, list of (x, y)
        self.state = "idle"
        self.reason = ""
        self.plan_ms = 0.0
        self._lock = threading.Lock()
        self._evt = threading.Event()
        self._req = None                  # (token, logodds snapshot, start_xy, goal_xy)
        self._res = None                  # (token, path, reason, ms)
        self._token = 0
        self._busy = False
        self._last_req = 0.0
        self._stop = False
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    # ---- control ----
    @property
    def active(self):
        return self.goal is not None

    def set_goal(self, x, y, pose):
        self.goal = (x, y)
        self.path = None
        self.state, self.reason = "planning", ""
        self._token += 1
        self._request(pose)

    def cancel(self):
        self.goal = None
        self.path = None
        self._token += 1
        self.state, self.reason = "idle", ""

    def close(self):
        self._stop = True
        self._evt.set()
        try:
            if self._conn is not None:
                self._conn.send(None)
        except Exception:
            pass
        if self._proc is not None and self._proc.is_alive():
            self._proc.join(0.5)
            if self._proc.is_alive():
                self._proc.terminate()

    def _start_proc(self):
        ctx = mp.get_context("spawn")                         # same behaviour on Windows and Linux
        parent, child = ctx.Pipe()
        self._proc = ctx.Process(target=_plan_process, args=(child, self.planner_spec), daemon=True)
        self._proc.start()
        self._conn = parent

    # ---- planning thread ----
    def _request(self, pose):
        self._last_req = time.monotonic()
        with self._lock:
            self._req = (self._token, self.grid.grid.copy(), (pose[0], pose[1]), self.goal)
        self._evt.set()

    def _worker(self):
        try:
            self._start_proc()                                    # warm up now so the first click is not delayed
        except Exception:
            pass
        while not self._stop:
            self._evt.wait(0.2)
            self._evt.clear()
            with self._lock:
                req, self._req = self._req, None
            if req is None:
                continue
            token, snap, start, goal = req
            self._busy = True
            try:
                if self._proc is None or not self._proc.is_alive():
                    self._start_proc()
                self._conn.send((token, snap, self.grid.left, self.grid.top, self.grid.res, start, goal,
                                 self.plan_res, self.inflate_m))
                while not self._conn.poll(0.2):
                    if self._stop or not self._proc.is_alive():
                        raise RuntimeError("planner process died")
                tok, path, reason, ms = self._conn.recv()
            except Exception as e:
                tok, path, reason, ms = token, None, f"planner process error: {e}", 0.0
                self._proc = None
            with self._lock:
                self._res = (tok, path, reason, ms)
            self._busy = False

    # ---- called ~20 Hz ----
    def update(self, pose, front_min):
        """pose = (x, y, yaw). front_min = nearest lidar return in front (m), or None.
        Returns (v, w) to command, or None when not navigating."""
        if self.goal is None:
            return None
        with self._lock:
            res, self._res = self._res, None
        if res is not None and res[0] == self._token:
            _, path, reason, self.plan_ms = res
            if path:
                self.path = path
                self.follower.set_path(path)
                self.state, self.reason = "driving", reason
            else:
                self.path = None
                self.state, self.reason = "no path", reason
        wait = self.replan_s if self.state == "driving" else max(self.replan_s, 3.0)   # don't hammer when there is no route
        if (self.state in ("driving", "no path") and not self._busy
                and time.monotonic() - self._last_req > wait):
            self._request(pose)
        if self.path is None:
            return 0.0, 0.0                               # waiting for a plan, or none exists
        v, w, done = self.follower.update(*pose)
        if done:
            self.goal, self.path = None, None
            self.state, self.reason = "arrived", ""
            return 0.0, 0.0
        if front_min is not None and front_min < self.stop_dist and v > 0:
            v = 0.0                                       # something close ahead: stop and let the replan route round
            self.reason = f"obstacle {front_min:.2f} m ahead"
        elif self.reason.startswith("obstacle"):
            self.reason = ""
        return v, w

    def status(self):
        if self.state == "idle":
            return "idle (click the map to drive somewhere)"
        s = self.state + (f": {self.reason}" if self.reason else "")
        if self.goal is not None:
            s += f"  goal ({self.goal[0]:+.1f}, {self.goal[1]:+.1f})"
        return s
