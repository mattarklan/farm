from dataclasses import dataclass, asdict, fields
from typing import List, Any
import json
from datetime import datetime
from core.events import Event
from core.rules import Action

@dataclass(slots=True)
class LogEntry:
    timestamp: str
    state_snapshot: dict
    events: List[dict]
    actions: List[dict]

def json_serializable(obj):
    """Recursively convert non-serializable types."""
    if isinstance(obj, datetime):
        return obj.isoformat()
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

    def log(self, state, events: List[Event], actions: List[Action]):
        entry = {
            "timestamp": state.timestamp.isoformat(),
            "state_snapshot": json_serializable(asdict(state)),
            "events": [self._event_to_dict(e) for e in events],
            "actions": [self._action_to_dict(a) for a in actions],
        }
        self.history.append(entry)

    def save(self, path: str = "farm_log.json"):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.history, f, indent=2)

    def _event_to_dict(self, event: Event) -> dict:
        return {
            "event_id": event.event_id,
            "zone": event.zone,
            "severity": event.severity,
            "confidence": event.confidence,
            "timestamp": event.timestamp.isoformat(),
            "data": json_serializable(event.data),
        }

    def _action_to_dict(self, action: Action) -> dict:
        return {
            "action_type": action.action_type,
            "zone": action.zone,
            "params": json_serializable(action.params),
        }