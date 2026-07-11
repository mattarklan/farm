"""
Flags sensors that haven't reported a fresh reading recently enough --
config/staleness_thresholds.yaml sets how "recent" needs to be, per
metric (default 24h, tighter overrides for fast-changing metrics like
soil_moisture/battery voltage). Directly motivated by header_tank_1
showing an 8-hour-stale reading in real data while nothing noticed.
"""
from __future__ import annotations

import uuid
from datetime import datetime, timezone

from core.models import Event
from core.severity import Severity


class StalenessMonitor:
    def __init__(self, thresholds_config: dict, registry=None, severity: int = Severity.URGENT):
        self.default_max_age = thresholds_config.get("default_max_age_seconds", 86400)
        self.overrides = thresholds_config.get("overrides", {})
        self.registry = registry
        self.severity = severity

    def process(self, state: dict) -> list:
        now = state["timestamp"]
        if now.tzinfo is None:
            now = now.replace(tzinfo=timezone.utc)
        events = []

        for sensor_id, sensor_state in state["sensors"].items():
            if sensor_state.timestamp is None:
                continue
            ts = sensor_state.timestamp
            if ts.tzinfo is None:
                ts = ts.replace(tzinfo=timezone.utc)

            max_age = self._max_age_for(sensor_id)
            age_seconds = (now - ts).total_seconds()
            if age_seconds > max_age:
                sensor_state.confidence = 0.0  # stale reading is untrustworthy -- same mechanism as confidence_monitor
                events.append(Event(
                    id=str(uuid.uuid4()),
                    event_type="SENSOR_STALE",
                    asset_id=self._asset_id_for(sensor_id),
                    sensor_id=sensor_id,
                    severity=self.severity,
                    confidence=1.0,
                    timestamp=state["timestamp"],
                    data={"age_seconds": age_seconds, "max_age_seconds": max_age},
                ))

        return events

    def _max_age_for(self, sensor_id: str) -> float:
        for metric, max_age in self.overrides.items():
            if sensor_id.endswith(f"_{metric}"):
                return max_age
        return self.default_max_age

    def _asset_id_for(self, sensor_id: str) -> str:
        if self.registry is not None:
            asset = self.registry.get_asset_from_sensor(sensor_id)
            if asset is not None:
                return asset.id
        return sensor_id.rsplit("_", 1)[0]
