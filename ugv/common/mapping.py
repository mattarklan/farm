"""
Occupancy grid built from lidar scans. The robot starts knowing nothing (every cell unknown) and
learns the world only from what its lidar sees.

Each scan is placed at the robot's pose when the scan was taken. Every ray is marked FREE up to a
cell short of its hit, and the hit cell is marked OCCUPIED. Cells hold a log-odds value, so a few
noisy returns don't flip a cell and repeated scans make it more certain.

Frame: x east / y north in metres, centred on the map origin. Row 0 of the array is the NORTH edge
(like an image), so row = (top - y) / res and col = (x - left) / res.

Layout: ugv/common/mapping.py
"""

import math

import numpy as np

L_OCC, L_FREE, L_MIN, L_MAX = 0.85, -0.4, -4.0, 4.0   # log-odds change per scan, and clamp
OCC_T, FREE_T = 0.4, -0.3                              # > OCC_T is a wall, < FREE_T is free, else unknown
UNKNOWN_GREY = 0.42                                    # shade used for unknown cells when drawing


class OccupancyGrid:
    def __init__(self, width_m, height_m, res):
        self.res = float(res)
        self.w = int(round(width_m / res))
        self.h = int(round(height_m / res))
        self.left = -self.w * self.res / 2
        self.top = self.h * self.res / 2
        self.extent = (self.left, -self.left, -self.top, self.top)    # left, right, bottom, top (metres)
        self.grid = np.zeros((self.h, self.w), np.float32)
        self.scans = 0

    # ---- coordinates ----
    def to_cell(self, x, y):
        return math.floor((self.top - y) / self.res), math.floor((x - self.left) / self.res)

    def to_xy(self, row, col):
        return self.left + (col + 0.5) * self.res, self.top - (row + 0.5) * self.res

    def inside(self, row, col):
        return 0 <= row < self.h and 0 <= col < self.w

    # ---- building the map ----
    def reset(self):
        self.grid[:] = 0.0
        self.scans = 0

    def _bump(self, x, y, delta, skip=None):
        col = np.floor((x - self.left) / self.res).astype(np.intp)
        row = np.floor((self.top - y) / self.res).astype(np.intp)
        ok = (row >= 0) & (row < self.h) & (col >= 0) & (col < self.w)
        row, col = row[ok], col[ok]
        if skip is not None and skip.size:                           # cells that must not be cleared by this scan
            keep = ~np.isin(row * self.w + col, skip)
            row, col = row[keep], col[keep]
        self.grid[row, col] = np.clip(self.grid[row, col] + delta, L_MIN, L_MAX)   # duplicates count once

    def _surface_cells(self, ang, d, hx, hy, join_deg=4.0, join_m=1.0):
        """Cells that lie ON a surface the lidar saw in this scan: the hit cells, and the straight line joining
        each pair of neighbouring hits that look like the same surface. On a wall seen at a steep angle the hits
        are far apart and the beams between them graze past wall cells that no beam ended in. Those cells are
        wall, not free space, so a scan must not clear them."""
        if d.size == 0:
            return np.empty(0, np.intp)
        o = np.argsort(ang)
        a, r, x, y = ang[o], d[o], hx[o], hy[o]
        na, nr, nx, ny = np.roll(a, -1), np.roll(r, -1), np.roll(x, -1), np.roll(y, -1)
        join = (((na - a) % 360.0) <= join_deg) & (np.hypot(nx - x, ny - y) <= join_m) & (np.abs(nr - r) <= join_m)
        if d.size < 2:
            join[:] = False
        x0, y0, x1, y1 = x[join], y[join], nx[join], ny[join]
        n = np.maximum(np.ceil(np.hypot(x1 - x0, y1 - y0) / (self.res * 0.5)).astype(int), 1) + 1
        seg = np.repeat(np.arange(x0.size), n)
        k = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)
        f = k / np.maximum(n[seg] - 1, 1)
        px = np.concatenate((x, x0[seg] + f * (x1[seg] - x0[seg])))
        py = np.concatenate((y, y0[seg] + f * (y1[seg] - y0[seg])))
        col = np.floor((px - self.left) / self.res).astype(np.intp)
        row = np.floor((self.top - py) / self.res).astype(np.intp)
        out = []
        for dr in (-1, 0, 1):                                        # one cell of margin either side of the surface
            for dc in (-1, 0, 1):
                rr, cc = row + dr, col + dc
                ok = (rr >= 0) & (rr < self.h) & (cc >= 0) & (cc < self.w)
                out.append(rr[ok] * self.w + cc[ok])
        return np.unique(np.concatenate(out))

    @staticmethod
    def _blur_kernel(sigma_cells):
        r = int(math.ceil(3 * sigma_cells))
        k = np.exp(-0.5 * (np.arange(-r, r + 1) / sigma_cells) ** 2)
        return k / k.max()

    def match_yaw(self, pose, angles_deg, dists_mm, yaw_offset_deg=0.0, max_deg=6.0, step_deg=0.5,
                  min_hits=25, min_occ=60, sigma_cells=0.7, max_range=9.0):
        """Heading correction by scan matching. The position (RTK) is trusted; only the heading is searched.
        Tries rotating the scan about the robot by -max_deg..+max_deg and keeps the rotation that puts the most
        lidar hits onto walls ALREADY in the map. Returns (dyaw_radians, fit) to ADD to the pose yaw, or None
        when there is nothing reliable to match against (map too empty, too few hits, no clear best answer).
        Call it BEFORE integrate_scan, so the scan is compared with what was learned earlier."""
        px, py, yaw = pose
        ang = (np.asarray(angles_deg, float) + yaw_offset_deg) % 360.0
        d = np.asarray(dists_mm, float) / 1000.0
        keep = (d > 0.3) & (d < max_range)
        ang, d = ang[keep], d[keep]
        if d.size < min_hits:
            return None
        # a window of the map around the robot, turned into a smooth "how close to a wall" field
        pad = int(math.ceil((max_range + 1.0) / self.res))
        r0, c0 = self.to_cell(px, py)
        ra, rb = max(r0 - pad, 0), min(r0 + pad + 1, self.h)
        ca, cb = max(c0 - pad, 0), min(c0 + pad + 1, self.w)
        if rb - ra < 3 or cb - ca < 3:
            return None
        occ = (self.grid[ra:rb, ca:cb] > OCC_T).astype(np.float32)
        if occ.sum() < min_occ:
            return None
        ker = self._blur_kernel(sigma_cells).astype(np.float32)
        field = np.apply_along_axis(lambda v: np.convolve(v, ker, mode="same"), 0, occ)
        field = np.apply_along_axis(lambda v: np.convolve(v, ker, mode="same"), 1, field)
        field = np.minimum(field, 1.0)
        cands = np.arange(-max_deg, max_deg + step_deg / 2, step_deg)
        scores = np.empty(cands.size)
        base = yaw - np.radians(ang)
        for i, dd in enumerate(cands):
            b = base + math.radians(dd)
            col = np.floor((px + d * np.cos(b) - self.left) / self.res).astype(np.intp) - ca
            row = np.floor((self.top - (py + d * np.sin(b))) / self.res).astype(np.intp) - ra
            ok = (row >= 0) & (row < rb - ra) & (col >= 0) & (col < cb - ca)
            scores[i] = field[row[ok], col[ok]].sum() / d.size
        scores -= 0.002 * np.abs(cands) / step_deg                   # when it's a tie, the smaller correction wins
        i = int(np.argmax(scores))
        zero = int(np.argmin(np.abs(cands)))
        if i == 0 or i == cands.size - 1:                            # best answer is at the edge of the search: not trustworthy
            return None
        if scores[i] < 0.35 or scores[i] - scores[zero] < 0.02:      # too few hits land on walls / no real improvement
            return None
        y0, y1, y2 = scores[i - 1], scores[i], scores[i + 1]
        denom = y0 - 2 * y1 + y2
        off = 0.0 if abs(denom) < 1e-9 else float(np.clip(0.5 * (y0 - y2) / denom, -1, 1))
        return math.radians(cands[i] + off * step_deg), float(scores[i])

    def integrate_scan(self, pose, angles_deg, dists_mm, yaw_offset_deg=0.0, free_range=6.0, gap_deg=3.0, protect_surfaces=True):
        """pose = (x, y, yaw) of the lidar when the scan was taken.
        angles are clockwise from the robot's front (like a real RPLidar), distances in mm.

        A lidar reports NOTHING for a beam that hits nothing (open ground, or a wall further than its range).
        That is information too: the space along that beam is empty. Gaps in the scan wider than `gap_deg`
        are therefore marked free out to `free_range` metres (0 turns this off). Narrow gaps are ignored,
        because a single missing beam is usually just a dropped reading, not open space."""
        px, py, yaw = pose
        ang = (np.asarray(angles_deg, float) + yaw_offset_deg) % 360.0
        d = np.asarray(dists_mm, float) / 1000.0
        keep = d > 0.05
        ang, d = ang[keep], d[keep]
        bearing = yaw - np.radians(ang)                              # clockwise -> counter-clockwise

        # beams that saw nothing: fill the wide gaps between the beams that did return
        gap_b = np.empty(0)
        if free_range > 0:
            if ang.size >= 2:
                srt = np.sort(ang)
                nxt = np.roll(srt, -1)
                width = (nxt - srt) % 360.0
                parts = []
                for lo, w in zip(srt[width > gap_deg], width[width > gap_deg]):
                    parts.append(lo + np.arange(1.0, w - 0.5, 1.0))   # one beam per degree inside the gap
                if parts:
                    gap_b = yaw - np.radians(np.concatenate(parts))
            elif ang.size == 0:                                       # nothing at all returned: a completely open field
                gap_b = yaw - np.radians(np.arange(0.0, 360.0, 1.0))
        if d.size == 0 and gap_b.size == 0:
            return

        all_b = np.concatenate((bearing, gap_b))
        all_d = np.concatenate((d, np.full(gap_b.size, free_range + self.res)))
        c, s = np.cos(all_b), np.sin(all_b)
        step = self.res * 0.5
        t = np.arange(int(all_d.max() / step) + 1) * step
        bi, ti = np.nonzero(t[None, :] < (all_d[:, None] - self.res))   # free space: up to a cell short of the hit
        surface = self._surface_cells(ang, d, px + d * np.cos(bearing), py + d * np.sin(bearing)) if protect_surfaces else None
        self._bump(px + t[ti] * c[bi], py + t[ti] * s[bi], L_FREE, skip=surface)
        if d.size:
            self._bump(px + d * np.cos(bearing), py + d * np.sin(bearing), L_OCC)   # the hit itself (after free, so it wins)
        self.scans += 1

    # ---- reading the map ----
    def blocked(self):
        return self.grid > OCC_T

    def state_at(self, x, y):
        row, col = self.to_cell(x, y)
        if not self.inside(row, col):
            return "off map"
        v = self.grid[row, col]
        return "OCCUPIED" if v > OCC_T else ("free" if v < FREE_T else "unknown")

    def image(self):
        """Float image for drawing: 1.0 free, 0.0 wall, grey unknown."""
        img = np.full(self.grid.shape, UNKNOWN_GREY, np.float32)
        img[self.grid < FREE_T] = 1.0
        img[self.grid > OCC_T] = 0.0
        return img

    def stats(self):
        known = np.count_nonzero((self.grid > OCC_T) | (self.grid < FREE_T)) / self.grid.size * 100.0
        return known, int(np.count_nonzero(self.grid > OCC_T))

    def save_png(self, path):
        """Black = wall, white = free, grey (205) = unknown. Same orientation as cmap.png."""
        from PIL import Image
        img = np.full(self.grid.shape, 205, np.uint8)
        img[self.grid < FREE_T] = 255
        img[self.grid > OCC_T] = 0
        Image.fromarray(img).save(path)
