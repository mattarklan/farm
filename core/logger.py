"""
In-memory + file logger for farm decision cycles.

Two fixes from the pasted version:
  1. Imported `core.events.Event` / `core.rules.Action` — those modules
     are the binned legacy system. Switched to `core.models`, which is
     what event_engine.py / rule_engine.py actually produce.
  2. Used `asdict(state)`, which only works on dataclass instances.
     `state` is a plain dict everywhere else in this codebase
     (state_builder.py, pipeline.py, event_engine.py all treat it as
     one), so asdict() would've raised a TypeError the first time this
     ran for real. Serializes the dict directly instead.
"""
from __future__ import annotations

import json
from datetime import datetime
from typing import List

from core.models import Event, Action, SensorState


def json_serializable(obj):
    """Recursively convert non-JSON-serializable types."""
    if isinstance(obj, datetime):
        return obj.isoformat()
    if isinstance(obj, SensorState):
        return {
            "value": obj.value,
            "quality": obj.quality,
            "timestamp": obj.timestamp.isoformat() if obj.timestamp else None,
            "source": obj.source,
        }
    if isinstance(obj, dict):
        return {k: json_serializable(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [json_serializable(i) for i in obj]
    return obj


class Logger:
    """
    In-memory + file logger for farm decision cycles.
    Captures full system state transitions.
    """

    def __init__(self):
        self.history: List[dict] = []

    def log(self, state: dict, events: List[Event], actions: List[Action]):
        entry = {
            "timestamp": state["timestamp"].isoformat(),
            "state_snapshot": json_serializable(state),
            "events": [self._event_to_dict(e) for e in events],
            "actions": [self._action_to_dict(a) for a in actions],
        }
        self.history.append(entry)

    def save(self, path: str = "farm_log.json"):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.history, f, indent=2)

    def _event_to_dict(self, event: Event) -> dict:
        return {
            "id": event.id,
            "event_type": event.event_type,
            "asset_id": event.asset_id,
            "sensor_id": event.sensor_id,
            "severity": event.severity,
            "confidence": event.confidence,
            "timestamp": event.timestamp.isoformat(),
            "data": json_serializable(event.data),
        }

    def _action_to_dict(self, action: Action) -> dict:
        return {
            "id": action.id,
            "action_type": action.action_type,
            "target": action.target,
            "parameters": json_serializable(action.parameters),
            "priority": action.priority,
            "created_from_event": action.created_from_event,
            "timestamp": action.timestamp.isoformat(),
        }
