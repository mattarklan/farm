"""
Writes the machine-readable event/action history the GUI reads:
config/events.json and config/actions.json.

JSON, not YAML: both files are written and read entirely by code, never
hand-edited, and appended to every cycle -- JSON's simpler grammar
avoids YAML's whitespace-sensitivity and type-coercion surprises (e.g.
"no"/"on" parsing as booleans) for auto-generated data. YAML earns its
keep on the hand-edited config files (rules.yaml, constraints.yaml
etc.); this isn't one of those.

Actions older than ACTION_RETENTION_DAYS are pruned from actions.json on
every write. events.json is kept in full as the audit trail -- add
pruning there too later if it grows unwieldy.

Implementation note: this does a full read-modify-write of each file
every cycle, which is fine at farm scale on a 60s interval but won't
scale to a high-frequency or very long-running history without
switching to JSON Lines or a real DB -- flag if that becomes a problem.
"""
from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone
from pathlib import Path

ACTION_RETENTION_DAYS = 30
EVENT_RETENTION_DAYS_DEFAULT = 90
EVENT_RETENTION_DAYS_CRITICAL = 365  # severity 1 (CRITICAL) events kept a full year


class GuiExporter:
    def __init__(self, events_path: str = "config/events.json", actions_path: str = "config/actions.json"):
        self.events_path = Path(events_path)
        self.actions_path = Path(actions_path)

    def export(self, events, actions_with_status):
        """
        events: list[core.models.Event]
        actions_with_status: list[dict], each {"action": Action, "status": str, "reason": str | None (optional)}
        """
        self._append_events(events)
        self._append_actions(actions_with_status)

    def _append_events(self, events):
        existing = self._read(self.events_path)
        existing.extend(self._event_to_dict(e) for e in events)
        now = datetime.now(timezone.utc)
        existing = [e for e in existing if self._parse(e["timestamp"]) >= self._event_cutoff(e, now)]
        self._write(self.events_path, existing)

    @staticmethod
    def _event_cutoff(event_dict, now):
        # severity 1 = CRITICAL -- kept a full year; everything else 90 days
        days = EVENT_RETENTION_DAYS_CRITICAL if event_dict.get("severity") == 1 else EVENT_RETENTION_DAYS_DEFAULT
        return now - timedelta(days=days)

    def _append_actions(self, actions_with_status):
        existing = self._read(self.actions_path)
        existing.extend(self._action_to_dict(entry) for entry in actions_with_status)
        cutoff = datetime.now(timezone.utc) - timedelta(days=ACTION_RETENTION_DAYS)
        existing = [a for a in existing if self._parse(a["timestamp"]) >= cutoff]
        self._write(self.actions_path, existing)

    @staticmethod
    def _read(path: Path) -> list:
        if not path.exists():
            return []
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError):
            # Corrupt/unreadable GUI file -- don't crash the pipeline
            # over it, just start fresh. (This kind of thing is exactly
            # what the problem-only logger.py should be recording.)
            return []

    @staticmethod
    def _write(path: Path, data: list):
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, default=str)

    @staticmethod
    def _parse(ts: str) -> datetime:
        dt = datetime.fromisoformat(ts)
        return dt if dt.tzinfo else dt.replace(tzinfo=timezone.utc)

    @staticmethod
    def _event_to_dict(event) -> dict:
        return {
            "id": event.id,
            "event_type": event.event_type,
            "asset_id": event.asset_id,
            "sensor_id": event.sensor_id,
            "severity": event.severity,
            "confidence": event.confidence,
            "timestamp": event.timestamp.isoformat(),
            "data": event.data,
            "source": event.source,
            "proposal_id": event.proposal_id,
        }

    @staticmethod
    def _action_to_dict(entry: dict) -> dict:
        action = entry["action"]
        return {
            "id": action.id,
            "action_type": action.action_type,
            "target": action.target,
            "parameters": action.parameters,
            "priority": action.priority,
            "severity": action.severity,
            "status": entry["status"],
            "reason": entry.get("reason"),
            "created_from_event": action.created_from_event,
            "timestamp": action.timestamp.isoformat(),
            "source": action.source,
            "proposal_id": action.proposal_id,
        }
