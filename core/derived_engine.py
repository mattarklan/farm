"""
Computes derived per-sensor signals (slope, volatility) from rolling
window history.

Nothing you pasted defines this class — `pipeline.py`, `event_engine.py`,
and `constraint_engine.py` all reference a `derived_engine.compute(...)`
/ `derived[sensor_id]["slope"|"volatility"]`, but no derived_engine.py
was ever provided. This is a minimal, clearly-labeled placeholder that
covers just those two keys so the pipeline is actually runnable. Replace
it if a real implementation exists elsewhere.
"""
from __future__ import annotations

import statistics

from core.window_store import RollingWindowStore


class DerivedEngine:
    def __init__(self, window_store: RollingWindowStore, slope_window_n: int = 5):
        self.window_store = window_store
        self.slope_window_n = slope_window_n

    def compute(self, sensor_ids) -> dict:
        derived = {}
        for sensor_id in sensor_ids:
            points = self.window_store.get_last_n(sensor_id, self.slope_window_n)
            derived[sensor_id] = {
                "slope": self._slope(points),
                "volatility": self._volatility(points),
            }
        return derived

    @staticmethod
    def _slope(points) -> float:
        if len(points) < 2:
            return 0.0
        (t0, v0), (t1, v1) = points[0], points[-1]
        dt = (t1 - t0).total_seconds()
        if dt <= 0:
            return 0.0
        return (v1 - v0) / dt

    @staticmethod
    def _volatility(points) -> float:
        values = [v for _, v in points]
        if len(values) < 2:
            return 0.0
        return statistics.pstdev(values)
