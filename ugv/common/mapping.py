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

    def _bump(self, x, y, delta):
        col = np.floor((x - self.left) / self.res).astype(np.intp)
        row = np.floor((self.top - y) / self.res).astype(np.intp)
        ok = (row >= 0) & (row < self.h) & (col >= 0) & (col < self.w)
        row, col = row[ok], col[ok]
        self.grid[row, col] = np.clip(self.grid[row, col] + delta, L_MIN, L_MAX)   # duplicates count once

    def integrate_scan(self, pose, angles_deg, dists_mm, yaw_offset_deg=0.0, free_range=6.0, gap_deg=3.0):
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
        self._bump(px + t[ti] * c[bi], py + t[ti] * s[bi], L_FREE)
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
