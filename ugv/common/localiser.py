"""
Lidar localisation: nudge the robot's pose so that what the lidar sees lines up with the map built so far.

Odometry drifts (wheel slip, wheel-size error, gyro bias). Every scan we try a small grid of
"what if the robot were a bit to the left / forward / turned" and score each by how many of the scan's
end points land on (or next to) walls already in the map. The best-scoring offset, if it is clearly
better than "no change" and the fit is good, becomes a small correction.

It needs walls in the map to match against, so it stays quiet until some have been seen, and it never
makes a big jump: the search is limited to a few centimetres / degrees per scan.

Layout: ugv/common/localiser.py
"""

import math
import time

import numpy as np

from .mapping import OCC_T


class Localiser:
    def __init__(self, grid, search_xy=0.20, step_xy=0.05, search_yaw_deg=6.0, step_yaw_deg=2.0,
                 max_points=150, min_walls=150, min_score=0.40, min_gain=0.02):
        self.grid = grid
        self.dxy = np.arange(-search_xy, search_xy + 1e-9, step_xy)
        self.dyaw = np.radians(np.arange(-search_yaw_deg, search_yaw_deg + 1e-9, step_yaw_deg))
        self.max_points, self.min_walls, self.min_score, self.min_gain = max_points, min_walls, min_score, min_gain
        self.field = None
        self.field_t = -1e9
        self.score = 0.0          # fit of the last scan (0..1)
        self.last = (0.0, 0.0, 0.0)
        self.status = "waiting for walls in the map"

    # ---- the "how close to a wall" field ----
    def refresh(self, now=None, every=0.5):
        now = time.monotonic() if now is None else now
        if now - self.field_t < every:
            return
        self.field_t = now
        occ = self.grid.grid > OCC_T
        if int(occ.sum()) < self.min_walls:
            self.field = None
            return
        f = occ.astype(np.float32)
        out = f.copy()
        cur = f
        for weight in (0.7, 0.4, 0.2):                     # 1, 2, 3 cells away from a wall
            d = cur.copy()
            d[1:, :] = np.maximum(d[1:, :], cur[:-1, :])
            d[:-1, :] = np.maximum(d[:-1, :], cur[1:, :])
            d[:, 1:] = np.maximum(d[:, 1:], cur[:, :-1])
            d[:, :-1] = np.maximum(d[:, :-1], cur[:, 1:])
            cur = d
            out = np.maximum(out, cur * weight)
        self.field = out

    def _score(self, xs, ys):
        g = self.grid
        rows = np.floor((g.top - ys) / g.res).astype(np.intp)
        cols = np.floor((xs - g.left) / g.res).astype(np.intp)
        ok = (rows >= 0) & (rows < g.h) & (cols >= 0) & (cols < g.w)
        vals = np.zeros(rows.shape, np.float32)
        vals[ok] = self.field[rows[ok], cols[ok]]
        return vals.mean(axis=-1)

    def match(self, pose, angles_deg, dists_mm, yaw_offset_deg=0.0):
        """Returns (dx, dy, dyaw) to ADD to `pose` (map frame), or None when there is nothing trustworthy to say."""
        if self.field is None:
            self.status = "waiting for walls in the map"
            return None
        d = np.asarray(dists_mm, float) / 1000.0
        a = np.asarray(angles_deg, float) + yaw_offset_deg
        keep = (d > 0.30) & (d < 8.0)
        d, a = d[keep], a[keep]
        if d.size < 30:
            self.status = "too few lidar points"
            return None
        if d.size > self.max_points:
            idx = np.linspace(0, d.size - 1, self.max_points).astype(int)
            d, a = d[idx], a[idx]
        b = -np.radians(a)                                          # clockwise -> counter-clockwise
        lx, ly = d * np.cos(b), d * np.sin(b)                        # points in the robot frame
        px, py, yaw = pose
        # every candidate pose = pose + (dx, dy, dyaw); evaluate all of them at once
        DY, DX, DT = np.meshgrid(self.dxy, self.dxy, self.dyaw, indexing="ij")
        DX, DY, DT = DX.ravel()[:, None], DY.ravel()[:, None], DT.ravel()[:, None]
        c, s = np.cos(yaw + DT), np.sin(yaw + DT)
        xs = px + DX + c * lx[None, :] - s * ly[None, :]
        ys = py + DY + s * lx[None, :] + c * ly[None, :]
        score = self._score(xs, ys)
        # prefer small corrections when scores are close (stops it sliding along a plain wall)
        penalty = 0.15 * (np.hypot(DX, DY).ravel() + 0.5 * np.abs(DT).ravel())
        total = score - penalty
        k = int(np.argmax(total))
        zero = int(np.argmin(np.abs(DX.ravel()) + np.abs(DY.ravel()) + np.abs(DT.ravel())))
        self.score = float(score[k])
        if score[k] < self.min_score:
            self.status = f"poor match ({score[k]:.2f})"
            return None
        if total[k] - total[zero] < self.min_gain:
            self.status = f"locked (fit {score[k]:.2f})"
            self.last = (0.0, 0.0, 0.0)
            return 0.0, 0.0, 0.0
        self.last = (float(DX.ravel()[k]), float(DY.ravel()[k]), float(DT.ravel()[k]))
        self.status = f"correcting {math.hypot(self.last[0], self.last[1]) * 100:.0f} cm, {math.degrees(self.last[2]):.1f} deg (fit {score[k]:.2f})"
        return self.last
