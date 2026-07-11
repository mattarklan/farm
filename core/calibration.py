"""
Records a manual calibration check -- e.g. you take a handheld soil
moisture reading and want to know how far the installed sensor has
drifted from it -- and flags drift as an event if it's outside
tolerance.

Deliberately simple and manual-trigger-only. No automatic/continuous
drift modelling here (that's the still-open "operator computes its own
confidence from trends" question from earlier -- genuinely unclear what
the right model is, especially with seasonal/temperature effects, so
not guessed at). This just answers: "I measured X by hand, the sensor
says Y right now -- is that within tolerance, and log it either way."

Usage (standalone, or wire into a small CLI/GUI form later):

    from core.calibration import CalibrationChecker
    checker = CalibrationChecker()
    event = checker.record_check("zone_b_soil_moisture", reference_value=0.28, state=current_state)
    if event:
        ...  # feed it into gui_export / decision_memory same as any other event
"""
from __future__ import annotations

import json
import uuid
from datetime import datetime, timezone
from pathlib import Path

from core.models import Event
from core.severity import Severity

DEFAULT_TOLERANCE = 0.05  # same units as the sensor's own value (e.g. 0.05 = 5% moisture)


class CalibrationChecker:
    def __init__(self, log_path: str = "config/calibration_log.json", default_tolerance: float = DEFAULT_TOLERANCE):
        self.log_path = Path(log_path)
        self.default_tolerance = default_tolerance

    def record_check(self, sensor_id: str, reference_value: float, state: dict,
                      tolerance: float | None = None) -> Event | None:
        """
        Compares `reference_value` (your handheld reading) against
        whatever the sensor currently reports in `state`. Always logs
        the check to calibration_log.json regardless of outcome, for a
        running record of how each sensor's drifted over time. Returns
        a SENSOR_CONFIDENCE_DRIFT Event only if the offset exceeds
        tolerance -- caller decides what to do with it (feed into
        gui_export/decision_memory, print it, whatever).
        """
        sensor_state = state["sensors"].get(sensor_id)
        if sensor_state is None:
            return None  # sensor not currently reporting -- nothing to compare against

        tolerance = self.default_tolerance if tolerance is None else tolerance
        offset = sensor_state.value - reference_value

        self._append({
            "sensor_id": sensor_id,
            "reference_value": reference_value,
            "sensor_value": sensor_state.value,
            "offset": offset,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        })

        if abs(offset) <= tolerance:
            return None

        return Event(
            id=str(uuid.uuid4()),
            event_type="SENSOR_CONFIDENCE_DRIFT",
            asset_id=sensor_id.rsplit("_", 1)[0],
            sensor_id=sensor_id,
            severity=Severity.REVIEW,
            confidence=1.0,
            timestamp=state["timestamp"],
            data={
                "reference_value": reference_value,
                "sensor_value": sensor_state.value,
                "offset": offset,
                "tolerance": tolerance,
            },
        )

    def _append(self, entry: dict):
        existing = []
        if self.log_path.exists():
            try:
                existing = json.loads(self.log_path.read_text())
            except (json.JSONDecodeError, OSError):
                existing = []
        existing.append(entry)
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_path.write_text(json.dumps(existing, indent=2))
