"""
Manages interruptible, multi-cycle goals. Three ways a rule becomes a
goal (checked in this order by _infer_kind):

  1. Explicit: kind: goal, with its own `until` list.
  2. Auto-derived: an instant-shaped rule whose action has a
     `duration_minutes` param and no `until` -- treated as a goal with a
     single, primary, timer-only stop condition. This is what lets
     governance's existing schema (open_valve/start_pump + duration,
     no concept of "until") drive a goal without changing that schema.
  3. Scheduled: trigger.type == "schedule" (start_time/end_time,
     repeats daily) instead of an event trigger -- e.g. "grow lights
     18:00-06:00". Checked against wall-clock time each cycle, not
     against incoming Events.

STOP CONDITIONS -- primary vs fallback:
  Each `until` entry may set `primary: true`. The primary condition is
  "the goal achieved what it was for" (moisture hit target; light
  reached its scheduled end time) -- stopping via it is normal, no
  special flag. Any OTHER condition stopping the goal first (timer
  ran out, a constraint tripped) means the primary condition was never
  satisfied -- something didn't go as planned -- so that gets the goal's
  severity bumped up a notch and `data["expected"] = False` on the
  GOAL_STOPPED event, rather than being silently equivalent.

  If no condition is marked primary, the first one in the list is
  treated as primary by default.

TEMPLATES:
  "$event"          -> event.asset_id (targets) / event.sensor_id (until sensors)
  "$asset.<field>"  -> looks up <field> on the resolved target's
                        registry metadata (e.g. crops.yaml)

CONFIDENCE: a `sensor`/`constraint` until-condition whose sensor has
confidence <= 0 is skipped for that cycle (not met, not failed) so the
goal falls through to its other conditions (a timer, typically) as a
fallback. ConfidenceMonitor raises SENSOR_CONFIDENCE_ZERO separately.

AUDIT TRAIL: `source`/`proposal_id` on the rule (governance schema
fields) are copied onto every Action and Event this goal produces.
"""
from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from datetime import datetime, timedelta, time as dt_time
from typing import Optional

from core.models import Action, Event


def infer_is_goal(rule: dict) -> bool:
    """Shared with RuleEngine so both agree on which rules belong to which engine."""
    if rule.get("kind") == "goal":
        return True
    if "until" in rule:
        return True
    if rule.get("trigger", {}).get("type") == "schedule":
        return True
    start = rule.get("start_action") or (rule.get("actions") or [{}])[0]
    if start.get("params", {}).get("duration_minutes") is not None and "until" not in rule:
        return True
    return False


@dataclass(slots=True)
class Goal:
    id: str
    rule_id: str
    target: str
    action_type: str  # the START action type, e.g. OPEN_VALVE -- what's currently commanded on this target
    trigger_sensor_id: str
    stop_action_type: str
    stop_params: dict
    stop_severity: int
    until: list
    started_at: datetime
    started_from_event: str
    priority: int
    source: Optional[str] = None
    proposal_id: Optional[str] = None
    schedule_end_time: Optional[str] = None  # HH:MM, only set for schedule-triggered goals
    resources: list = field(default_factory=list)  # shared resource names this goal holds while active


class GoalEngine:
    def __init__(self, rule_config: dict, registry=None, action_types_config: dict | None = None):
        self.action_types = (action_types_config or {}).get("action_types", {})
        self.rules, _ = self._filter_active(rule_config.get("rules", []))
        self.registry = registry
        self.active_goals: dict[str, Goal] = {}
        self.last_stopped_at: dict[str, datetime] = {}
        self._resource_holders: dict[str, str] = {}   # resource_name -> target currently holding it
        self._pending: list = []                       # queued (key, rule, event, target, schedule_end_time), FIFO

    def reload(self, rule_config: dict):
        self.rules, skipped = self._filter_active(rule_config.get("rules", []))
        return skipped

    def _filter_active(self, rules):
        now = datetime.utcnow()
        active, skipped = [], []
        for r in rules:
            if not infer_is_goal(r):
                continue
            if "id" not in r:
                skipped.append((r, "missing id"))
                continue
            if not r.get("enabled", True):
                continue
            expires_at = r.get("expires_at")
            if expires_at:
                try:
                    if datetime.fromisoformat(expires_at) <= now:
                        continue
                except ValueError:
                    skipped.append((r, f"unparseable expires_at: {expires_at!r}"))
                    continue
            normalized = self._normalize(r)
            if normalized is None:
                skipped.append((r, "goal rule missing start_action/stop_action"))
                continue
            active.append(normalized)
        return active, skipped

    def _normalize(self, rule: dict) -> Optional[dict]:
        """Fills in `until`/`start_action`/`stop_action` for the
        auto-derived (duration_minutes, no explicit until) shape, so
        the rest of the engine only ever deals with one consistent
        goal-rule shape."""
        rule = dict(rule)
        if "start_action" not in rule:
            action_defs = rule.get("actions") or []
            if not action_defs:
                return None
            rule["start_action"] = action_defs[0]
        if "stop_action" not in rule:
            start_type = rule["start_action"]["type"]
            spec = self.action_types.get(start_type) or self.action_types.get(start_type.upper())
            stop_type = (spec or {}).get("stop_action_type") or _STOP_TYPE_FOR_START.get(start_type)
            if stop_type is None:
                return None  # no known stop counterpart -- can't safely auto-derive
            rule["stop_action"] = {"type": stop_type, "target": rule["start_action"].get("target", "$event")}
        if "until" not in rule:
            duration = rule["start_action"].get("params", {}).get("duration_minutes")
            if duration is None:
                return None
            rule["until"] = [{"type": "timer", "max_duration_minutes": duration, "primary": True}]
        return rule

    def _rule_still_active(self, rule_id: str) -> bool:
        return any(r["id"] == rule_id for r in self.rules)

    def get_active_action_type(self, target: str):
        """What action_type is currently commanded on this target, if any.
        Used by anomaly_engine.py's commanded_vs_sensor checks."""
        goal = self.active_goals.get(target)
        return goal.action_type if goal else None

    def process(self, events, state, derived):
        stop_actions = []
        goal_events = []

        for target, goal in list(self.active_goals.items()):
            result = self._check_until(goal, state, derived)
            if result is None and not self._rule_still_active(goal.rule_id):
                result = ("rule_deactivated", True)  # deliberate deactivation -- not "unexpected"
            if result is not None:
                reason, expected = result
                stop_actions.append(self._build_stop_action(goal))
                goal_events.append(self._build_goal_event(goal, reason, expected, state))
                self.last_stopped_at[target] = datetime.utcnow()
                self._release_resources(goal)
                del self.active_goals[target]

        # a stop above may have freed a resource a queued request was
        # waiting on -- give the queue first shot before anything new
        # tries to jump in ahead of it
        start_actions = self._drain_pending()

        # event-triggered goals
        for event in events:
            for rule in self.rules:
                if rule.get("trigger", {}).get("type") == "schedule":
                    continue  # handled separately below
                if not self._trigger_matches(rule, event):
                    continue
                self._maybe_start(rule, event, state, start_actions)

        # schedule-triggered goals -- checked every cycle against wall clock, no event needed
        now_t = state["timestamp"].time()
        for rule in self.rules:
            trigger = rule.get("trigger", {})
            if trigger.get("type") != "schedule":
                continue
            if not self._in_schedule_window(now_t, trigger["start_time"], trigger["end_time"]):
                continue
            target = rule["start_action"]["target"]
            if target == "$event":
                continue  # schedule rules must have a literal target, no triggering event to derive from
            if target in self.active_goals:
                continue
            self._maybe_start(rule, None, state, start_actions, schedule_end_time=trigger["end_time"])

        return start_actions, stop_actions, goal_events

    def _maybe_start(self, rule, event, state, start_actions, schedule_end_time=None):
        target = self._resolve(rule["start_action"]["target"], event)
        if target in self.active_goals:
            return
        min_interval_days = rule.get("min_interval_days")
        if min_interval_days is not None:
            last = self.last_stopped_at.get(target)
            if last and (datetime.utcnow() - last) < timedelta(days=min_interval_days):
                return

        resources = rule.get("resources", [])
        busy = [r for r in resources if self._resource_holders.get(r) not in (None, target)]
        if busy:
            self._enqueue(rule, event, target, schedule_end_time)
            return

        goal = self._build_goal(rule, event, target, schedule_end_time)
        self._claim_resources(goal)
        self.active_goals[target] = goal
        start_actions.append(self._build_start_action(rule, event, target))

    # -------------------------------------------------
    # RESOURCE QUEUE (FIFO) -- for rules that declare `resources: [...]`
    # (e.g. two zones sharing one bore/pump). A request whose resources
    # are all free starts immediately, same as before. One that isn't
    # queues instead of silently failing, and is served in arrival
    # order the moment the resource frees up -- no priority jumping,
    # per the "first come first served, no urgent case seen yet" call.
    # -------------------------------------------------
    def _enqueue(self, rule, event, target, schedule_end_time):
        key = (rule["id"], target)
        if any(k == key for k, *_ in self._pending):
            return  # already queued -- don't duplicate on every re-firing event
        self._pending.append((key, rule, event, target, schedule_end_time))

    def _drain_pending(self):
        start_actions = []
        still_pending = []
        for key, rule, event, target, schedule_end_time in self._pending:
            if target in self.active_goals:
                continue  # started some other way already -- drop from queue
            resources = rule.get("resources", [])
            busy = [r for r in resources if self._resource_holders.get(r) not in (None, target)]
            if busy:
                still_pending.append((key, rule, event, target, schedule_end_time))
                continue
            goal = self._build_goal(rule, event, target, schedule_end_time)
            self._claim_resources(goal)
            self.active_goals[target] = goal
            start_actions.append(self._build_start_action(rule, event, target))
        self._pending = still_pending
        return start_actions

    def _claim_resources(self, goal: "Goal"):
        for r in goal.resources:
            self._resource_holders[r] = goal.target

    def _release_resources(self, goal: "Goal"):
        for r in goal.resources:
            if self._resource_holders.get(r) == goal.target:
                del self._resource_holders[r]

    # -------------------------------------------------
    # SCHEDULE WINDOW
    # -------------------------------------------------
    @staticmethod
    def _parse_hhmm(s: str) -> dt_time:
        h, m = s.split(":")
        return dt_time(int(h), int(m))

    def _in_schedule_window(self, now_t: dt_time, start_s: str, end_s: str) -> bool:
        start, end = self._parse_hhmm(start_s), self._parse_hhmm(end_s)
        if start <= end:
            return start <= now_t < end
        return now_t >= start or now_t < end  # wraps past midnight (e.g. 18:00-06:00)

    # -------------------------------------------------
    # TEMPLATE RESOLUTION
    # -------------------------------------------------
    @staticmethod
    def _resolve(spec: str, event: Optional[Event]) -> str:
        if spec == "$event":
            return event.asset_id
        return spec

    def _resolve_asset_field(self, target: str, field_name: str):
        if self.registry is None:
            return None
        asset = getattr(self.registry, "assets", {}).get(target)
        if asset is None:
            return None
        return asset.metadata.get(field_name)

    def _resolve_value(self, value_spec, target):
        if isinstance(value_spec, str) and value_spec.startswith("$asset."):
            return self._resolve_asset_field(target, value_spec.split(".", 1)[1])
        return value_spec

    # -------------------------------------------------
    # TRIGGER MATCH (event-based only)
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
    # UNTIL-CONDITION CHECK -- returns (reason, expected) or None
    # -------------------------------------------------
    def _check_until(self, goal: Goal, state, derived):
        elapsed_seconds = (datetime.utcnow() - goal.started_at).total_seconds()
        conditions = goal.until
        primary_idx = next((i for i, c in enumerate(conditions) if c.get("primary")), 0)

        for i, cond in enumerate(conditions):
            kind = cond["type"]
            is_primary = (i == primary_idx)

            if kind == "sensor":
                sensor_id = goal.trigger_sensor_id if cond["sensor"] == "$event" else cond["sensor"]
                sensor_state = state["sensors"].get(sensor_id)
                if sensor_state is None or sensor_state.confidence <= 0:
                    continue
                target_value = self._resolve_value(cond["value"], goal.target)
                if target_value is None:
                    continue
                if self._compare(sensor_state.value, cond["op"], target_value):
                    return (f"goal_condition_met:{sensor_id}_{cond['op']}_{target_value}", is_primary)

            elif kind == "timer":
                if elapsed_seconds >= cond["max_duration_minutes"] * 60:
                    return ("timer_expired", is_primary)

            elif kind == "constraint":
                sensor_id = goal.trigger_sensor_id if cond["sensor"] == "$event" else cond["sensor"]
                sensor_state = state["sensors"].get(sensor_id)
                if sensor_state is None or sensor_state.confidence <= 0:
                    continue
                value = sensor_state.value
                lo, hi = cond.get("min", float("-inf")), cond.get("max", float("inf"))
                if value < lo or value > hi:
                    return (f"constraint_violation:{sensor_id}={value}", is_primary)

            elif kind == "schedule_end":
                if goal.schedule_end_time is not None:
                    now_t = state["timestamp"].time()
                    end = self._parse_hhmm(goal.schedule_end_time)
                    # crossing/at end time -- simple same-tick check, good enough at cycle granularity
                    if now_t.hour == end.hour and now_t.minute == end.minute:
                        return ("schedule_end_reached", is_primary)

        return None

    @staticmethod
    def _compare(left, op, right) -> bool:
        if op == "lt": return left < right
        if op == "lte": return left <= right
        if op == "gt": return left > right
        if op == "gte": return left >= right
        if op == "eq": return left == right
        if op == "neq": return left != right
        return False

    # -------------------------------------------------
    # BUILDERS
    # -------------------------------------------------
    def _build_goal(self, rule, event, target, schedule_end_time=None) -> Goal:
        start, stop = rule["start_action"], rule["stop_action"]
        return Goal(
            id=str(uuid.uuid4()),
            rule_id=rule["id"],
            target=target,
            action_type=start["type"],
            trigger_sensor_id=event.sensor_id if event else "",
            stop_action_type=stop["type"],
            stop_params=stop.get("params", {}),
            stop_severity=stop.get("severity", 3),
            until=rule.get("until", []),
            started_at=datetime.utcnow(),
            started_from_event=event.id if event else f"schedule:{rule['id']}",
            priority=rule.get("priority", 1),
            source=rule.get("source"),
            proposal_id=rule.get("proposal_id"),
            schedule_end_time=schedule_end_time,
            resources=rule.get("resources", []),
        )

    def _build_start_action(self, rule, event, target) -> Action:
        start = rule["start_action"]
        return Action(
            id=str(uuid.uuid4()),
            action_type=start["type"],
            target=target,
            parameters=start.get("params", {}),
            priority=rule.get("priority", 1),
            created_from_event=event.id if event else f"schedule:{rule['id']}",
            timestamp=datetime.utcnow(),
            severity=start.get("severity", 3),
            source=rule.get("source"),
            proposal_id=rule.get("proposal_id"),
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
            severity=goal.stop_severity,
            source=goal.source,
            proposal_id=goal.proposal_id,
        )

    def _build_goal_event(self, goal: Goal, reason: str, expected: bool, state) -> Event:
        severity = goal.stop_severity if expected else max(1, goal.stop_severity - 1)
        return Event(
            id=str(uuid.uuid4()),
            event_type="GOAL_STOPPED",
            asset_id=goal.target,
            sensor_id="",
            severity=severity,
            confidence=1.0,
            timestamp=state["timestamp"],
            data={
                "rule_id": goal.rule_id,
                "goal_id": goal.id,
                "reason": reason,
                "expected": expected,
                "started_at": goal.started_at.isoformat(),
            },
            source=goal.source,
            proposal_id=goal.proposal_id,
        )


# action_type -> its natural stop counterpart, used when auto-deriving a
# goal from a duration_minutes instant action with no explicit stop_action
_STOP_TYPE_FOR_START = {
    "OPEN_VALVE": "CLOSE_VALVE",
    "START_PUMP": "STOP_PUMP",
    "open_valve": "close_valve",
    "start_pump": "stop_pump",
}
