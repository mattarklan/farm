"""
Watches sensor health from two angles:

1. Zero confidence -- the ingestor's own MCU-reported "no legitimate
   signal" flag. Raised immediately, every cycle it's true.

2. Stuck values -- a sensor reporting the EXACT same value for many
   cycles running (derived_engine's volatility == 0 over its window).
   This is a heuristic, not a certainty: a genuinely stable reading
   (e.g. battery voltage overnight) can look identical to a frozen
   sensor. To cut down false positives, this only fires after the
   value has been flat for `stuck_cycles_threshold` consecutive cycles
   (default 10) -- tune this per how twitchy real sensors normally are.
   Reports every cycle it remains stuck; core/event_dedup.py (shared
   across all engines) is what turns that into "once, then periodic
   reminders" instead of spamming the log every cycle.

NOT built here: implausible-range checks (needs per-sensor-type
min/max, which nobody's defined yet) and implausible rate-of-change
checks (needs a per-sensor-type "physically possible per cycle" bound).
Both are easy to add once you have real numbers to set those bounds
with -- guessing at them now risks false alarms or missed real faults.
"""
from __future__ import annotations

import uuid

from core.models import Event
from core.severity import Severity

ZERO_CONFIDENCE_THRESHOLD = 0.0


class ConfidenceMonitor:
    def __init__(self, severity: int = Severity.CRITICAL, registry=None,
                 stuck_cycles_threshold: int = 10, stuck_severity: int = Severity.REVIEW):
        self.severity = severity
        self.registry = registry
        self.stuck_cycles_threshold = stuck_cycles_threshold
        self.stuck_severity = stuck_severity
        self._stuck_counters: dict[str, int] = {}

    def process(self, state: dict, derived: dict | None = None) -> list:
        derived = derived or {}
        events = []

        for sensor_id, sensor_state in state["sensors"].items():

            if sensor_state.confidence <= ZERO_CONFIDENCE_THRESHOLD:
                events.append(Event(
                    id=str(uuid.uuid4()),
                    event_type="SENSOR_CONFIDENCE_ZERO",
                    asset_id=self._asset_id_for(sensor_id),
                    sensor_id=sensor_id,
                    severity=self.severity,
                    confidence=1.0,
                    timestamp=state["timestamp"],
                    data={"reported_confidence": sensor_state.confidence},
                ))
                self._stuck_counters[sensor_id] = 0  # no legit signal -- stuck-check is meaningless here
                continue

            volatility = derived.get(sensor_id, {}).get("volatility")
            looks_stuck = volatility == 0 and sensor_state.value != 0  # a real "off" sensor legitimately reads 0
            if looks_stuck:
                self._stuck_counters[sensor_id] = self._stuck_counters.get(sensor_id, 0) + 1
            else:
                self._stuck_counters[sensor_id] = 0

            if self._stuck_counters.get(sensor_id, 0) >= self.stuck_cycles_threshold:
                # Downgrade confidence to 0 (not just raise an event): this
                # is what makes event_engine skip threshold checks on it
                # and goal_engine fall back to its timer, automatically --
                # same mechanism that already exists for ingest-reported
                # zero confidence, now driven by core's own finding too.
                sensor_state.confidence = 0.0
                events.append(Event(
                    id=str(uuid.uuid4()),
                    event_type="SENSOR_VALUE_STUCK",
                    asset_id=self._asset_id_for(sensor_id),
                    sensor_id=sensor_id,
                    severity=self.stuck_severity,
                    confidence=1.0,
                    timestamp=state["timestamp"],
                    data={"stuck_value": sensor_state.value, "cycles": self.stuck_cycles_threshold},
                ))

        return events

    def _asset_id_for(self, sensor_id: str) -> str:
        if self.registry is not None:
            asset = self.registry.get_asset_from_sensor(sensor_id)
            if asset is not None:
                return asset.id
        return sensor_id.rsplit("_", 1)[0]
