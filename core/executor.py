"""
Dispatches allowed Actions to their hardware-facing effect.

Generic and config-driven, per the observation that every action is
really just "set a bool or int channel on a target" -- the action_type
name is cosmetic (rules.yaml/GUI readability), not something the
hardware cares about. See config/action_types.yaml. Adding a new
action_type as the farm grows means editing that YAML, not this file.

This still only SIMULATES (prints) the hardware effect -- the real
MCU/relay comms layer isn't built, and plugging it in later means
changing only `_apply()` below, nothing about the dispatch logic.
"""
from __future__ import annotations

from datetime import datetime

from core.models import ActionResult


class Executor:
    def __init__(self, action_types_config: dict | None = None):
        self.action_types = (action_types_config or {}).get("action_types", {})

    def execute(self, actions):
        results = []
        for action in actions:
            spec = self.action_types.get(action.action_type) or self.action_types.get(action.action_type.upper())
            if spec is None:
                results.append(ActionResult(
                    action_id=action.id, success=False,
                    message=f"Unknown action_type '{action.action_type}' -- not in config/action_types.yaml",
                    timestamp=datetime.utcnow(),
                ))
                continue
            results.append(self._apply(action, spec))
        return results

    def _apply(self, action, spec: dict) -> ActionResult:
        value_type = spec.get("value_type")

        if value_type == "none":
            print(f"[EXECUTOR] {action.action_type} -> {action.target} (no hardware channel, simulated)")
            return ActionResult(action_id=action.id, success=True,
                                 message="No hardware effect (simulated)", timestamp=datetime.utcnow())

        if value_type == "bool":
            value = spec.get("value", True)
            print(f"[EXECUTOR] {action.target}.{spec['channel']} = {value}  (via {action.action_type}, simulated)")
            return ActionResult(action_id=action.id, success=True,
                                 message=f"{spec['channel']} set to {value} (simulated)", timestamp=datetime.utcnow())

        if value_type == "int":
            param_key = spec.get("param", spec["channel"])
            value = action.parameters.get(param_key)
            if value is None:
                return ActionResult(action_id=action.id, success=False,
                                     message=f"Missing required param '{param_key}' for {action.action_type}",
                                     timestamp=datetime.utcnow())
            print(f"[EXECUTOR] {action.target}.{spec['channel']} = {value}  (via {action.action_type}, simulated)")
            return ActionResult(action_id=action.id, success=True,
                                 message=f"{spec['channel']} set to {value} (simulated)", timestamp=datetime.utcnow())

        return ActionResult(action_id=action.id, success=False,
                             message=f"Unknown value_type '{value_type}' in action_types.yaml for {action.action_type}",
                             timestamp=datetime.utcnow())
