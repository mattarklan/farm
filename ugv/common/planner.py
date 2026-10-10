"""
Path planning on the learned map.

THE HOOK FOR YOUR OWN THETA*
----------------------------
A planner is any function with this signature:

    def plan(blocked, start, goal):
        '''blocked: numpy bool array (rows x cols), True = obstacle (already inflated by the
                    robot's radius, so the robot can be treated as a single point).
                    Row 0 is the NORTH edge, rows increase southwards, columns increase eastwards.
           start, goal: (row, col) cells, both guaranteed to be inside the array and not blocked.
           returns: a list of (row, col) cells from start to goal (any-angle waypoints are fine;
                    Theta* output goes straight in), or None if there is no path.'''

To use yours:   python visualiser.py --planner my_theta:plan      (module:function, importable
from the folder you run in, or from ugv/). The built-in theta_star() below is a plain Theta* that
does the same job, so everything works until yours is ready.

Unknown space is treated as FREE (optimistic): the robot heads for the goal through unexplored
areas, discovers walls with its lidar, and the visualiser replans as the map fills in.

Layout: ugv/common/planner.py
"""

import heapq
import importlib
import math
import time

import numpy as np

from .mapping import OCC_T


def inflate(blocked, r):
    """Grow obstacles by r cells (disk)."""
    if r <= 0:
        return blocked.copy()
    H, W = blocked.shape
    out = blocked.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if (dy or dx) and dy * dy + dx * dx <= r * r:
                out[max(0, dy):H + min(0, dy), max(0, dx):W + min(0, dx)] |= \
                    blocked[max(0, -dy):H + min(0, -dy), max(0, -dx):W + min(0, -dx)]
    return out


def line_of_sight(blocked, a, b):
    (r0, c0), (r1, c1) = a, b
    n = int(max(abs(r1 - r0), abs(c1 - c0)) * 2) + 1
    if n <= 1:
        return not (blocked[r0, c0] or blocked[r1, c1])
    t = np.linspace(0.0, 1.0, n)
    rr = np.rint(r0 + (r1 - r0) * t).astype(np.intp)
    cc = np.rint(c0 + (c1 - c0) * t).astype(np.intp)
    return not blocked[rr, cc].any()


_deadline = [None]          # perf_counter() time after which theta_star gives up (set by plan_world)


class SearchTooLong(Exception):
    pass


def theta_star(blocked, start, goal):
    """Basic Theta* (A* with line-of-sight parent shortcuts -> any-angle paths) on an 8-connected grid."""
    H, W = blocked.shape
    start, goal = tuple(start), tuple(goal)
    if blocked[start] or blocked[goal]:
        return None
    if start == goal:
        return [start]
    bl = blocked.tolist()
    inf = float("inf")
    g, parent = {start: 0.0}, {start: start}
    closed = set()

    def h(n):
        return math.hypot(n[0] - goal[0], n[1] - goal[1])

    heap, tick = [(h(start), 0, start)], 0
    while heap:
        if tick % 256 == 255 and _deadline[0] is not None and time.perf_counter() > _deadline[0]:
            raise SearchTooLong()
        _, _, s = heapq.heappop(heap)
        if s in closed:
            continue
        if s == goal:
            path = [s]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1]
        closed.add(s)
        r, c = s
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if not (dr or dc):
                    continue
                nr, nc = r + dr, c + dc
                if not (0 <= nr < H and 0 <= nc < W) or bl[nr][nc] or (nr, nc) in closed:
                    continue
                if dr and dc and (bl[r + dr][c] or bl[r][c + dc]):      # no cutting corners
                    continue
                n, p = (nr, nc), parent[s]
                if p != s and line_of_sight(blocked, p, n):
                    cand, par = g[p] + math.hypot(p[0] - nr, p[1] - nc), p
                else:
                    cand, par = g[s] + math.hypot(dr, dc), s
                if cand < g.get(n, inf):
                    g[n], parent[n] = cand, par
                    tick += 1
                    heapq.heappush(heap, (cand + h(n), tick, n))
    return None


def load_planner(spec):
    """'module:function' -> callable. None / '' gives the built-in theta_star."""
    if not spec:
        return theta_star
    mod, _, fn = spec.partition(":")
    return getattr(importlib.import_module(mod), fn or "plan")


def plan_world(logodds, left, top, res, start_xy, goal_xy, plan_res, inflate_m, planner, budget_s=3.0):
    """Try the full safety margin first; if that finds nothing, squeeze closer to walls (half, then 0.15 m)
    so a robot parked near a wall, or a narrow gap, still gets a path. Returns (path|None, reason)."""
    reason = ""
    _deadline[0] = time.perf_counter() + budget_s            # one time budget shared by all attempts
    try:
        for k, m in enumerate((inflate_m, inflate_m / 2, 0.15)):
            if k and m >= inflate_m:
                continue
            path, reason = _plan_once(logodds, left, top, res, start_xy, goal_xy, plan_res, m, planner)
            if path:
                return path, ("" if k == 0 else f"tight (margin {m:.2f} m)")
            if "outside" in reason or "blocked" in reason:
                break
    except SearchTooLong:
        return None, f"gave up after {budget_s:.0f} s (map too big/complex for the built-in planner)"
    finally:
        _deadline[0] = None
    return None, reason


def _plan_once(logodds, left, top, res, start_xy, goal_xy, plan_res, inflate_m, planner):
    """Plan on a snapshot of the map. Pure function (safe to run in a thread).
    Returns (path as a list of (x, y) metres, "") or (None, reason)."""
    f = max(1, int(round(plan_res / res)))
    pr = res * f
    H, W = logodds.shape
    Hc, Wc = H // f, W // f
    b = (logodds[:Hc * f, :Wc * f] > OCC_T).reshape(Hc, f, Wc, f).any(axis=(1, 3))
    ri = int(math.ceil(inflate_m / pr))
    b = inflate(b, ri)

    def cell(xy):
        return math.floor((top - xy[1]) / pr), math.floor((xy[0] - left) / pr)

    def centre(rc):
        return left + (rc[1] + 0.5) * pr, top - (rc[0] + 0.5) * pr

    s, gl = cell(start_xy), cell(goal_xy)
    if not (0 <= s[0] < Hc and 0 <= s[1] < Wc):
        return None, "robot is outside the map"
    if not (0 <= gl[0] < Hc and 0 <= gl[1] < Wc):
        return None, "goal is outside the map"
    # the robot may be standing close to a wall: free a small disk around it so it can drive away
    r0, c0 = s
    for dy in range(-ri, ri + 1):
        for dx in range(-ri, ri + 1):
            if dy * dy + dx * dx <= ri * ri and 0 <= r0 + dy < Hc and 0 <= c0 + dx < Wc:
                b[r0 + dy, c0 + dx] = False
    if b[gl]:                                           # goal inside an inflated wall: nearest free cell
        win = int(round(1.5 / pr))
        ra, rb, ca, cb = max(0, gl[0] - win), min(Hc, gl[0] + win + 1), max(0, gl[1] - win), min(Wc, gl[1] + win + 1)
        rr, cc = np.nonzero(~b[ra:rb, ca:cb])
        if rr.size == 0:
            return None, "goal is blocked"
        k = int(np.argmin((rr + ra - gl[0]) ** 2 + (cc + ca - gl[1]) ** 2))
        gl = (int(rr[k] + ra), int(cc[k] + ca))
    cells = planner(b, s, gl)
    if not cells:
        return None, "no route: the known walls block every way to the goal"
    path = [centre(rc) for rc in cells]
    path[0] = tuple(start_xy)
    return path, ""
