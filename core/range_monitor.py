"""
Flags a sensor reading at or beyond its physically-possible range
(config/sensor_range_thresholds.yaml) -- e.g. soil moisture at exactly
0% or 100% usually means the sensor's pegged at its rail (disconnected/
shorted), not that the soil is actually bone dry or fully saturated.

Metrics with no entry in the config are skipped entirely -- no check is
applied, rather than guessing a bound that could cause false alarms.
"""
from __future__ import annotations

import uuid

from core.models import Event
from core.severity import Severity


class RangeMonitor:
    def __init__(self, ranges_config: dict, registry=None, severity: int = Severity.CRITICAL):
        self.ranges = ranges_config.get("ranges", {})
        self.registry = registry
        self.severity = severity

    def process(self, state: dict) -> list:
        events = []
        for sensor_id, sensor_state in state["sensors"].items():
            bounds = self._bounds_for(sensor_id)
            if bounds is None:
                continue
            value = sensor_state.value
            if value <= bounds["min"] or value >= bounds["max"]:
                sensor_state.confidence = 0.0  # physically impossible reading is untrustworthy -- same mechanism
                events.append(Event(
                    id=str(uuid.uuid4()),
                    event_type="SENSOR_OUT_OF_RANGE",
                    asset_id=self._asset_id_for(sensor_id),
                    sensor_id=sensor_id,
                    severity=self.severity,
                    confidence=1.0,
                    timestamp=state["timestamp"],
                    data={"value": value, "min": bounds["min"], "max": bounds["max"]},
                ))
        return events

    def _bounds_for(self, sensor_id: str):
        for metric, bounds in self.ranges.items():
            if sensor_id.endswith(f"_{metric}"):
                return bounds
        return None

    def _asset_id_for(self, sensor_id: str) -> str:
        if self.registry is not None:
            asset = self.registry.get_asset_from_sensor(sensor_id)
            if asset is not None:
                return asset.id
        return sensor_id.rsplit("_", 1)[0]
