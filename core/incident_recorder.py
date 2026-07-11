"""
Permanent, never-pruned incident log -- deliberately independent of
ingestion's 90-day raw_telemetry purge and of gui_export.py's 30-day
action pruning. Motivating case: if dads_ute leaves the geofence, its
GPS trail and other telemetry need to survive on their own, since this
might end up being handed to police if the vehicle's stolen.

Triggered by any anomaly rule that declares `record_sensors: [...]` --
not vehicle-specific, so it's reusable for any future "this matters,
keep it regardless of normal retention" situation with zero new code,
just a YAML addition.

On first detecting the rule's condition, captures whatever window_store
history is already retained for each listed sensor (up to
window_store's own retention, 1h by default) -- so you get the leadup,
not just the moment it was first noticed -- then appends a fresh
snapshot every cycle the condition remains true. Records an end time
the moment the condition clears. Never deletes anything automatically.
"""
from __future__ import annotations

import json
import uuid
from pathlib import Path


class IncidentRecorder:
    def __init__(self, log_path: str = "config/incident_log.json", window_store=None):
        self.log_path = Path(log_path)
        self.window_store = window_store
        self.active: dict[str, str] = {}  # rule_id -> currently-open incident_id

    def update(self, rule_id: str, description: str, sensors: list, state: dict, active: bool):
        if active:
            self._record_active(rule_id, description, sensors, state)
        else:
            self._close_if_open(rule_id, state)

    def _record_active(self, rule_id, description, sensors, state):
        incidents = self._read()
        incident_id = self.active.get(rule_id)

        if incident_id is None:
            incident_id = str(uuid.uuid4())
            self.active[rule_id] = incident_id
            incidents.append({
                "incident_id": incident_id,
                "rule_id": rule_id,
                "description": description,
                "started_at": state["timestamp"].isoformat(),
                "ended_at": None,
                "history_at_start": {s: self._history_for(s) for s in sensors},
                "readings": [],
            })

        entry = next(i for i in incidents if i["incident_id"] == incident_id)
        entry["readings"].append({
            "timestamp": state["timestamp"].isoformat(),
            "values": {s: self._value_for(s, state) for s in sensors},
        })
        self._write(incidents)

    def _close_if_open(self, rule_id, state):
        incident_id = self.active.pop(rule_id, None)
        if incident_id is None:
            return
        incidents = self._read()
        for entry in incidents:
            if entry["incident_id"] == incident_id:
                entry["ended_at"] = state["timestamp"].isoformat()
        self._write(incidents)

    def _value_for(self, sensor_id, state):
        s = state["sensors"].get(sensor_id)
        return s.value if s else None

    def _history_for(self, sensor_id):
        if self.window_store is None:
            return []
        return [{"timestamp": t.isoformat(), "value": v} for t, v in self.window_store.get(sensor_id)]

    def _read(self) -> list:
        if not self.log_path.exists():
            return []
        try:
            return json.loads(self.log_path.read_text())
        except (json.JSONDecodeError, OSError):
            return []

    def _write(self, data: list):
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_path.write_text(json.dumps(data, indent=2, default=str))
