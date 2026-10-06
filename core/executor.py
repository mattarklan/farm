"""
Dispatches allowed Actions to their hardware-facing effect.

Generic and config-driven, per the observation that every action is
really just "set a bool or int channel on a target" -- the action_type
name is cosmetic (rules.yaml/GUI readability), not something the
hardware cares about. See config/action_types.yaml. Adding a new
action_type as the farm grows means editing that YAML, not this file.

This still only SIMULATES (prints) the hardware effect -- the real
MCU/relay comms layer is its own future project, this is the interface
layer it'll eventually sit behind. Until then, prints are the thing
someone's actually going to read to understand what core is doing, so
every one of them says not just WHAT is being simulated but WHAT LED TO
IT -- which rule/event/engine decided this needed to happen -- via
action.created_from_event/source/proposal_id, which were already being
set and stored but not surfaced here. Plugging in the real comms layer
later means changing only `_apply()`, nothing about the dispatch logic
or this reasoning.
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

    def _describe_origin(self, action) -> str:
        """What led to this action being sent -- printed alongside every
        simulated effect so the console log reads as a narrative, not
        just a stream of channel=value dumps."""
        origin = action.created_from_event or "unknown origin"
        if origin == "supply_engine":
            detail = "supply_engine tank routing / pause-resume decision"
        elif origin == "startup_safe_sweep":
            detail = "startup safe-sweep, forcing this actuator to its safe state on boot"
        elif isinstance(origin, str) and origin.startswith("schedule:"):
            detail = f"scheduled rule '{origin.split(':', 1)[1]}' (wall-clock trigger, no triggering event)"
        else:
            detail = f"triggered by event {origin}"
        if action.source:
            detail += f", source={action.source}"
        if action.proposal_id:
            detail += f", proposal={action.proposal_id}"
        return detail

    def _apply(self, action, spec: dict) -> ActionResult:
        value_type = spec.get("value_type")
        origin = self._describe_origin(action)

        if value_type == "none":
            print(f"[EXECUTOR] {action.action_type} on {action.target or '(no target)'} -- no hardware channel, nothing to simulate. Because: {origin}")
            return ActionResult(action_id=action.id, success=True,
                                 message="No hardware effect (simulated)", timestamp=datetime.utcnow())

        if value_type == "bool":
            value = spec.get("value", True)
            print(f"[EXECUTOR] {action.target}.{spec['channel']} = {value}  (via {action.action_type}, simulated). Because: {origin}")
            return ActionResult(action_id=action.id, success=True,
                                 message=f"{spec['channel']} set to {value} (simulated)", timestamp=datetime.utcnow())

        if value_type == "int":
            param_key = spec.get("param", spec["channel"])
            value = action.parameters.get(param_key)
            if value is None:
                return ActionResult(action_id=action.id, success=False,
                                     message=f"Missing required param '{param_key}' for {action.action_type}",
                                     timestamp=datetime.utcnow())
            print(f"[EXECUTOR] {action.target}.{spec['channel']} = {value}  (via {action.action_type}, simulated). Because: {origin}")
            return ActionResult(action_id=action.id, success=True,
                                 message=f"{spec['channel']} set to {value} (simulated)", timestamp=datetime.utcnow())

        return ActionResult(action_id=action.id, success=False,
                             message=f"Unknown value_type '{value_type}' in action_types.yaml for {action.action_type}",
                             timestamp=datetime.utcnow())
