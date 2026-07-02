"""
Manages interruptible, multi-cycle goals — e.g. "water zone B until soil
moisture > 0.3, or a constraint trips, or 30 minutes pass."

This is new. Nothing in the pasted codebase had a concept of an action
persisting across pipeline cycles — rule_engine.py is stateless (event
in, action out, done each cycle). GoalEngine adds the missing state:
`active_goals`, keyed by target, checked every cycle before new rules
are evaluated.

Rule shape this expects (see config/rules.yaml example):

    rules:
      - id: water_zone_b_until_moist
        kind: goal
        trigger:
          event_type: S1_SOIL_MOISTURE_LOW
        start_action:
          type: OPEN_VALVE
          target: zone_b
          params: {flow_rate_lpm: 10}
        stop_action:
          type: CLOSE_VALVE
          target: zone_b
        until:
          - type: sensor
            sensor: zone_b_soil_moisture
            op: gte
            value: 0.3
          - type: timer
            max_duration_minutes: 120
          - type: constraint
            sensor: zone_b_soil_moisture
            min: 0.0
            max: 1.0
        priority: 10

`rule_engine.py`'s existing rules (triggers/conditions/actions, no
"until") are untouched and keep working for instant actions — a rule in
config/rules.yaml is routed to GoalEngine only if it has `kind: goal`.

OPEN ITEM: "constraint"-type until-conditions here are a lightweight
per-sensor min/max check — a stand-in for the real "locked rules" /
constraints.yaml concept (battery voltage, tank level bounds etc.) you
described, which hasn't been built yet (ranked #3). Once that exists,
this should probably call into it instead of re-implementing its own
bound check.
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional

from core.models import Action, Event


@dataclass(slots=True)
class Goal:
    id: str
    rule_id: str
    target: str
    stop_action_type: str
    stop_params: dict
    until: list
    started_at: datetime
    started_from_event: str
    priority: int


class GoalEngine:
    def __init__(self, rule_config: dict):
        self.rules = [r for r in rule_config.get("rules", []) if r.get("kind") == "goal"]
        self.active_goals: dict[str, Goal] = {}  # keyed by target

    def process(self, events, state, derived):
        """
        Returns (start_actions, stop_actions, goal_events).

        Call every pipeline cycle, BEFORE rule_engine.process(). Stop
        checks run first (so a goal can end and free up its target in
        the same cycle a new one might want to claim it); new goals are
        then started from the incoming events, skipping any target
        that's already active.
        """
        stop_actions = []
        goal_events = []

        for target, goal in list(self.active_goals.items()):
            reason = self._check_until(goal, state, derived)
            if reason is not None:
                stop_actions.append(self._build_stop_action(goal))
                goal_events.append(self._build_goal_event(goal, reason, state))
                del self.active_goals[target]

        start_actions = []
        for event in events:
            for rule in self.rules:
                if not self._trigger_matches(rule, event):
                    continue

                target = rule["start_action"]["target"]
                if target in self.active_goals:
                    # already running — don't double-start, don't re-evaluate
                    continue

                goal = self._build_goal(rule, event)
                self.active_goals[target] = goal
                start_actions.append(self._build_start_action(rule, event))

        return start_actions, stop_actions, goal_events

    # -------------------------------------------------
    # TRIGGER MATCH
    # -------------------------------------------------
    def _trigger_matches(self, rule, event) -> bool:
        trigger = rule.get("trigger", {})
        if trigger.get("event_type") != event.event_type:
            return False
        if "zone_id" in trigger and trigger["zone_id"] != event.asset_id:
            return False
        if "asset_id" in trigger and trigger["asset_id"] != event.asset_id:
            return False
        return True

    # -------------------------------------------------
    # UNTIL-CONDITION CHECK — returns a reason string, or None if still running
    # -------------------------------------------------
    def _check_until(self, goal: Goal, state, derived) -> Optional[str]:
        elapsed_seconds = (datetime.utcnow() - goal.started_at).total_seconds()

        for cond in goal.until:
            kind = cond["type"]

            if kind == "sensor":
                sensor_state = state["sensors"].get(cond["sensor"])
                if sensor_state is None:
                    continue
                if self._compare(sensor_state.value, cond["op"], cond["value"]):
                    return f"goal_condition_met:{cond['sensor']}_{cond['op']}_{cond['value']}"

            elif kind == "timer":
                if elapsed_seconds >= cond["max_duration_minutes"] * 60:
                    return "timer_expired"

            elif kind == "constraint":
                sensor_state = state["sensors"].get(cond["sensor"])
                if sensor_state is None:
                    continue
                value = sensor_state.value
                lo = cond.get("min", float("-inf"))
                hi = cond.get("max", float("inf"))
                if value < lo or value > hi:
                    return f"constraint_violation:{cond['sensor']}={value}"

        return None

    @staticmethod
    def _compare(left, op, right) -> bool:
        if op == "lt":
            return left < right
        if op == "lte":
            return left <= right
        if op == "gt":
            return left > right
        if op == "gte":
            return left >= right
        if op == "eq":
            return left == right
        if op == "neq":
            return left != right
        return False

    # -------------------------------------------------
    # BUILDERS
    # -------------------------------------------------
    def _build_goal(self, rule, event) -> Goal:
        stop = rule["stop_action"]
        return Goal(
            id=str(uuid.uuid4()),
            rule_id=rule["id"],
            target=rule["start_action"]["target"],
            stop_action_type=stop["type"],
            stop_params=stop.get("params", {}),
            until=rule.get("until", []),
            started_at=datetime.utcnow(),
            started_from_event=event.id,
            priority=rule.get("priority", 1),
        )

    def _build_start_action(self, rule, event) -> Action:
        start = rule["start_action"]
        return Action(
            id=str(uuid.uuid4()),
            action_type=start["type"],
            target=start["target"],
            parameters=start.get("params", {}),
            priority=rule.get("priority", 1),
            created_from_event=event.id,
            timestamp=datetime.utcnow(),
        )

    def _build_stop_action(self, goal: Goal) -> Action:
        return Action(
            id=str(uuid.uuid4()),
            action_type=goal.stop_action_type,
            target=goal.target,
            parameters=goal.stop_params,
            priority=goal.priority,
            created_from_event=goal.started_from_event,
            timestamp=datetime.utcnow(),
        )

    def _build_goal_event(self, goal: Goal, reason: str, state) -> Event:
        return Event(
            id=str(uuid.uuid4()),
            event_type="GOAL_STOPPED",
            asset_id=goal.target,
            sensor_id="",
            severity=1,
            confidence=1.0,
            timestamp=state["timestamp"],
            data={
                "rule_id": goal.rule_id,
                "goal_id": goal.id,
                "reason": reason,
                "started_at": goal.started_at.isoformat(),
            },
        )
