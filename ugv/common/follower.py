"""
Path follower (pure pursuit): turns a list of waypoints into (speed, turn rate) commands.

Slows down for sharp turns and near the goal, rotates on the spot if the path is behind the robot.
Never exceeds max_speed / max_turn, which are the SAME limits the arrow keys and the simulator use
(common/config.py), so click-to-drive moves at the same speed as teleop.

Layout: ugv/common/follower.py
"""

import math


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PurePursuit:
    def __init__(self, max_speed, max_turn, lookahead=0.6, goal_tol=0.25):
        self.max_speed, self.max_turn = max_speed, max_turn
        self.lookahead, self.goal_tol = lookahead, goal_tol
        self.path = []
        self.seg = 0

    def set_path(self, path):
        self.path = list(path)
        self.seg = 0

    def _target(self, x, y):
        pts = self.path
        if len(pts) < 2:
            return pts[-1]
        # closest point on the path, searching forward from where we already are
        best = (float("inf"), self.seg, 0.0)
        for i in range(self.seg, len(pts) - 1):
            ax, ay = pts[i]
            bx, by = pts[i + 1]
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            if d < best[0] - 1e-9:
                best = (d, i, t)
            elif i > best[1] + 1 and d > best[0] + 2 * self.lookahead:
                break
        _, i, t = best
        self.seg = i
        # walk `lookahead` metres along the path from that point
        ax, ay = pts[i]
        bx, by = pts[i + 1]
        px, py = ax + t * (bx - ax), ay + t * (by - ay)
        remaining = self.lookahead
        while True:
            seg_len = math.hypot(bx - px, by - py)
            if seg_len >= remaining:
                f = remaining / seg_len if seg_len > 0 else 0.0
                return px + f * (bx - px), py + f * (by - py)
            remaining -= seg_len
            i += 1
            if i >= len(pts) - 1:
                return pts[-1]
            px, py = pts[i]
            bx, by = pts[i + 1]

    def update(self, x, y, yaw):
        """Returns (v, w, done)."""
        if not self.path:
            return 0.0, 0.0, True
        gx, gy = self.path[-1]
        dist = math.hypot(gx - x, gy - y)
        if dist < self.goal_tol:
            return 0.0, 0.0, True
        tx, ty = self._target(x, y)
        alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
        w = max(-self.max_turn, min(self.max_turn, 2.0 * alpha))
        if abs(alpha) > 1.0:                                  # target is off to the side/behind: turn first
            return 0.0, w, False
        v = self.max_speed * max(0.0, math.cos(alpha)) ** 2
        v = min(v, 0.15 + 1.0 * dist)                          # ease off near the goal
        v *= 1.0 - 0.5 * abs(w) / self.max_turn                # and in tight turns
        return max(0.0, v), w, False
