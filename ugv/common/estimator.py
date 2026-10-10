"""
Pose estimator: where is the robot RIGHT NOW (or when a lidar scan was taken)?

GNSS + IMU only arrive about once a second, but the robot moves between samples. We already know
what we told it to do (every cmd_vel command), so we run a small model of the robot (the visualiser's
"goal robot") and use ONLY ITS MOTION between the telemetry sample time and the time we care about
to carry the measured pose forward:

    pose(t) = measured pose at the sample time  +  how far the model moved since then

Every new GNSS/IMU sample corrects whatever the model got wrong, so errors never build up.

Layout: ugv/common/estimator.py
"""

import bisect
import math


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PoseEstimator:
    def __init__(self, hist_s=8.0):
        self.hist_s = hist_s
        self.ts, self.vals = [], []          # model history: times and (x, y, yaw)
        self.fix = None                      # (t_pos, x, y, t_yaw, yaw)

    def push_model(self, t, x, y, yaw):
        self.ts.append(t)
        self.vals.append((x, y, yaw))
        if len(self.ts) > 64 and t - self.ts[0] > self.hist_s * 1.5:
            k = bisect.bisect_left(self.ts, t - self.hist_s)
            del self.ts[:k], self.vals[:k]

    def set_fix(self, t_pos, x, y, t_yaw, yaw):
        self.fix = (t_pos, x, y, t_yaw, yaw)

    def reset_model(self):
        self.ts.clear()
        self.vals.clear()

    def has_fix(self):
        return self.fix is not None

    def fix_age(self, now):
        return None if self.fix is None else now - self.fix[0]

    def model_at(self, t):
        if not self.ts:
            return None
        if t <= self.ts[0]:
            return self.vals[0]
        if t >= self.ts[-1]:
            return self.vals[-1]
        i = bisect.bisect_right(self.ts, t)
        t0, t1 = self.ts[i - 1], self.ts[i]
        a = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        (x0, y0, w0), (x1, y1, w1) = self.vals[i - 1], self.vals[i]
        return x0 + (x1 - x0) * a, y0 + (y1 - y0) * a, wrap(w0 + wrap(w1 - w0) * a)

    def pose_at(self, t):
        """(x, y, yaw) at time t, or None before the first telemetry sample."""
        if self.fix is None:
            return None
        t_pos, fx, fy, t_yaw, fyaw = self.fix
        m_now, m_pos, m_yaw = self.model_at(t), self.model_at(t_pos), self.model_at(t_yaw)
        if m_now is None or m_pos is None or m_yaw is None:
            return fx, fy, fyaw
        # how far the model moved since the position sample, in the model's own body frame
        dx, dy = m_now[0] - m_pos[0], m_now[1] - m_pos[1]
        c, s = math.cos(-m_pos[2]), math.sin(-m_pos[2])
        bx, by = c * dx - s * dy, s * dx + c * dy
        # ...replayed from the measured heading at the position sample
        yaw_pos = fyaw + wrap(m_pos[2] - m_yaw[2])
        c, s = math.cos(yaw_pos), math.sin(yaw_pos)
        x = fx + c * bx - s * by
        y = fy + s * bx + c * by
        yaw = wrap(fyaw + wrap(m_now[2] - m_yaw[2]))
        return x, y, yaw
