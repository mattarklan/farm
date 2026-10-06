from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Dict, Deque, Tuple


@dataclass
class WindowConfig:
    max_size: int = 1000  # fallback
    max_age_seconds: int = 3600  # 1 hour default


class RollingWindowStore:
    def __init__(self, config: WindowConfig = WindowConfig()):
        self.config = config
        # sensor_id -> deque[(timestamp, value)]
        self.windows: Dict[str, Deque[Tuple[datetime, float]]] = {}

    def append(self, sensor_id: str, value: float, timestamp: datetime):
        if sensor_id not in self.windows:
            self.windows[sensor_id] = deque()
        window = self.windows[sensor_id]
        window.append((timestamp, value))
        # enforce size limit
        if len(window) > self.config.max_size:
            window.popleft()
        # enforce time limit
        self._prune_old(sensor_id)

    def get(self, sensor_id: str):
        return list(self.windows.get(sensor_id, []))

    def get_last_n(self, sensor_id: str, n: int):
        window = self.windows.get(sensor_id, [])
        return list(window)[-n:]

    def get_range(self, sensor_id: str, seconds: int, now: datetime):
        window = self.windows.get(sensor_id, [])
        cutoff = now.timestamp() - seconds
        return [
            (t, v)
            for (t, v) in window
            if t.timestamp() >= cutoff
        ]

    def _prune_old(self, sensor_id: str):
        window = self.windows[sensor_id]
        # datetime.utcnow() is naive but represents UTC wall-clock time.
        # Calling .timestamp() on a naive datetime makes Python assume
        # it's in the SYSTEM's local timezone and convert accordingly --
        # on a Perth (AWST, UTC+8) box that shifts the computed "now"
        # ~8h behind the real epoch, which pushes cutoff ~8h further
        # into the past than intended. In practice that means the
        # age-based prune (max_age_seconds, default 1h) almost never
        # actually fires -- only the size cap (max_size) still works,
        # so history sticks around far longer than the configured
        # retention. Using an explicit timezone-aware UTC datetime
        # makes .timestamp() correct regardless of the system's local
        # timezone.
        now = datetime.now(timezone.utc)
        cutoff = now.timestamp() - self.config.max_age_seconds
        while window and window[0][0].timestamp() < cutoff:
            window.popleft()
