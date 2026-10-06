"""
Tracks whether an executed action actually got acknowledged by its
target's MCU -- a layer BELOW anomaly_engine's action_effect_check.
This asks "did the command land at all"; action_effect_check asks "did
it produce the physical effect it was supposed to."

Three distinct fault classes fall out of this:
  FAILED   -- no ack within ack_timeout_seconds. Comms/MCU/wiring
              problem -- core doesn't know the command was ever
              received.
  REFUSED  -- ack received, but with a non-"ok" status -- the MCU is
              alive, got the command, and is explicitly declining it
              with a reason (e.g. a bore reporting "dry" instead of
              turning on). Known immediately, no need to wait for a
              timeout -- same channel as a normal ack, just a different
              value in it (see memo_ingestion_ack_interface.md).
  FAULTY   -- ack received with status "ok" (MCU applied the command),
              but a LATER anomaly_engine check (e.g.
              bore_1_running_no_flow) shows the physical effect never
              followed. Command landed and was applied, the hardware
              itself is the problem.

GUESSED INTERFACE -- no real ack signal exists in ingestion's API yet.
Assumes two sensors per target: f"{target}_last_ack_ts" (epoch
timestamp of the last command actually applied) and
f"{target}_ack_status" (string, "ok" or a reason code for a refusal).
Same "wired up now, real field later" pattern as
api_client.get_weather_offsets -- see the memo for the exact ask.

Registered for EVERY successfully-executed action -- on or off, goal
start/stop, instant rule, supply_engine tank routing, pause/resume,
startup safe-sweep -- regardless of when or why it was sent. An
actuator commanded off deserves the same confirmation as one commanded
on; core shouldn't assume a CLOSE_VALVE landed any more than it assumes
an OPEN_VALVE did.
"""
from __future__ import annotations

import uuid

from core.models import Event
from core.severity import Severity


class ActionConfirmationMonitor:
    def __init__(self, action_types_config: dict | None = None, ack_timeout_seconds: int = 30):
        self.action_types = (action_types_config or {}).get("action_types", {})
        self.ack_timeout_seconds = ack_timeout_seconds
        self._pending: dict[str, dict] = {}  # action_id -> {action, sent_at}

    def register(self, action, state):
        """Call right after executor.execute() succeeds. No-ops for
        value_type:none action_types (LOG_ONLY/TRIGGER_ALERT) -- those
        have no hardware channel or MCU at all, so there's nothing that
        could ever ack them; tracking them would just guarantee a false
        timeout every time, not a real check."""
        spec = self.action_types.get(action.action_type) or self.action_types.get(action.action_type.upper()) or {}
        if spec.get("value_type") == "none":
            return
        self._pending[action.id] = {"action": action, "sent_at": state["timestamp"]}

    def resolve(self, state) -> list:
        """Call once per cycle, BEFORE goal_engine.process() -- a
        no-ack/refusal needs to affect THIS cycle's goal decisions, same
        reasoning as the health checks running first in pipeline.py."""
        events = []
        still_pending = {}

        for action_id, entry in self._pending.items():
            action, sent_at = entry["action"], entry["sent_at"]
            ack_sensor_id = f"{action.target}_last_ack_ts"
            status_sensor_id = f"{action.target}_ack_status"
            ack_sensor = state["sensors"].get(ack_sensor_id)
            status_sensor = state["sensors"].get(status_sensor_id)

            acked = ack_sensor is not None and ack_sensor.confidence > 0 and ack_sensor.value >= sent_at.timestamp()

            if acked:
                status = status_sensor.value if status_sensor is not None and status_sensor.confidence > 0 else "ok"
                if status != "ok":
                    events.append(Event(
                        id=str(uuid.uuid4()),
                        event_type="ACTUATOR_COMMAND_REFUSED",
                        asset_id=action.target or "",
                        sensor_id=status_sensor_id,
                        severity=Severity.CRITICAL,
                        confidence=1.0,
                        timestamp=state["timestamp"],
                        data={
                            "action_id": action.id,
                            "action_type": action.action_type,
                            "reason": status,
                        },
                        source=action.source,
                        proposal_id=action.proposal_id,
                    ))
                continue  # acked either way (accepted or explicitly refused) -- drop from pending, nothing left to wait for

            elapsed = (state["timestamp"] - sent_at).total_seconds()
            if elapsed < self.ack_timeout_seconds:
                still_pending[action_id] = entry  # still waiting, not timed out yet
                continue

            events.append(Event(
                id=str(uuid.uuid4()),
                event_type="ACTION_NO_ACK",
                asset_id=action.target or "",
                sensor_id=ack_sensor_id,
                severity=Severity.CRITICAL,
                confidence=1.0,
                timestamp=state["timestamp"],
                data={
                    "action_id": action.id,
                    "action_type": action.action_type,
                    "sent_at": sent_at.isoformat(),
                    "ack_timeout_seconds": self.ack_timeout_seconds,
                },
                source=action.source,
                proposal_id=action.proposal_id,
            ))
            # dropped from pending either way -- one-shot per send, same
            # pattern as anomaly_engine's _pending_effect_checks

        self._pending = still_pending
        return events
