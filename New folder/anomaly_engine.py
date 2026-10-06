"""
Generic, YAML-driven cross-sensor / commanded-vs-actual checks
(config/anomaly_rules.yaml). Four reusable patterns so far:

  mass_balance         -- a level should track the direction implied by
                           inflow vs outflow (v1: direction only, not
                           exact volume).
  sensor_pair          -- sensor A meeting a condition implies sensor B
                           should too (e.g. bore running -> flow > 0).
  commanded_vs_sensor  -- an asset commanded on via an active goal
                           should show up on a related sensor.
  action_effect_check  -- an action executed should produce a measurable
                           change in a sensor within N seconds (e.g.
                           LOAD_SHEDDING should actually reduce current
                           draw) -- distinct from the other three: it's
                           triggered by an executed action, not
                           continuously re-evaluated every cycle.

New INSTANCES of these four are pure YAML. A genuinely new comparison
SHAPE needs one new case added here once; after that it's reusable.

sustained_seconds still gates mass_balance/sensor_pair/commanded_vs_sensor/
geofence (avoids flagging normal startup lag), but once past that
threshold, the mismatch is reported EVERY cycle it remains true --
core/event_dedup.py (shared across all engines) is what turns that into
"once, then periodic reminders" rather than each check re-implementing
its own one-shot suppression.
"""
from __future__ import annotations

import uuid
from datetime import datetime

from core.models import Event
from core.severity import Severity


class AnomalyEngine:
    def __init__(self, rules_config: dict, goal_engine=None, registry=None, incident_recorder=None, supply_engine=None):
        self.rules = rules_config.get("rules", [])
        self.goal_engine = goal_engine        # needed only for commanded_vs_sensor rules
        self.registry = registry              # needed only for geofence rules ($asset.<field> center)
        self.incident_recorder = incident_recorder  # needed only for rules with record_sensors
        self.supply_engine = supply_engine    # needed only for actuator_fault_matrix rules -- commanded-state provider, same role goal_engine plays for commanded_vs_sensor
        self._mismatch_since: dict[str, datetime] = {}
        self._pending_effect_checks: dict[str, dict] = {}  # rule_id -> {baseline, triggered_at}
        self._fault_matrix_since: dict[str, datetime] = {}  # f"{rule_id}:{outcome}" -> when THAT specific outcome started -- separate from _mismatch_since since this check has 4 possible outcomes, not one bool

    def process(self, state: dict, derived: dict) -> list:
        events = []
        for rule in self.rules:
            kind = rule.get("type")

            if kind == "action_effect_check":
                event = self._check_action_effect(rule, state)
                if event:
                    events.append(event)
                continue

            if kind == "actuator_fault_matrix":
                event = self._check_actuator_fault_matrix(rule, state, derived)
                if event:
                    events.append(event)
                continue

            if kind == "mass_balance":
                mismatch = self._check_mass_balance(rule, state, derived)
            elif kind == "sensor_pair":
                mismatch = self._check_sensor_pair(rule, state)
            elif kind == "commanded_vs_sensor":
                mismatch = self._check_commanded_vs_sensor(rule, state)
            elif kind == "geofence":
                mismatch = self._check_geofence(rule, state)
            else:
                continue  # unknown type -- skip rather than guess

            # Recording starts the instant the condition appears -- not
            # gated by sustained_seconds, so a brief false trigger still
            # gets captured.
            if self.incident_recorder is not None and rule.get("record_sensors"):
                self.incident_recorder.update(
                    rule["id"], rule.get("description", ""), rule["record_sensors"], state, active=mismatch
                )

            event = self._evaluate_sustained(rule, mismatch, state)
            if event:
                events.append(event)
        return events

    def on_action_executed(self, action, state):
        """
        Call once per successfully-executed action (pipeline.py, after
        executor.execute()). Snapshots a baseline for any
        action_effect_check rule watching this action_type/target, so
        process() can check back on it later.
        """
        for rule in self.rules:
            if rule.get("type") != "action_effect_check":
                continue
            if rule["trigger_action_type"] != action.action_type:
                continue
            target_filter = rule.get("trigger_target")
            if target_filter is not None and target_filter != action.target:
                continue
            sensor_state = state["sensors"].get(rule["check_sensor"])
            if sensor_state is None or sensor_state.confidence <= 0:
                continue
            self._pending_effect_checks[rule["id"]] = {
                "baseline": sensor_state.value,
                "triggered_at": state["timestamp"],
            }

    # -------------------------------------------------
    # SUSTAINED-MISMATCH TRACKING (shared by the 3 continuous rule types)
    # -------------------------------------------------
    def _evaluate_sustained(self, rule, mismatch: bool, state):
        rule_id = rule["id"]
        now = state["timestamp"]

        if not mismatch:
            self._mismatch_since.pop(rule_id, None)
            return None

        since = self._mismatch_since.setdefault(rule_id, now)
        elapsed = (now - since).total_seconds()
        if elapsed < rule.get("sustained_seconds", 0):
            return None

        return Event(
            id=str(uuid.uuid4()),
            event_type=rule.get("event_type", "ANOMALY_DETECTED"),
            asset_id=rule.get("asset") or rule.get("commanded_target") or "",
            sensor_id=rule.get("expect_sensor") or rule.get("level_sensor") or "",
            severity=rule.get("severity", Severity.URGENT),
            confidence=1.0,
            timestamp=now,
            data={"rule_id": rule_id, "description": rule.get("description", "")},
        )

    # -------------------------------------------------
    # RULE TYPE: action_effect_check
    # -------------------------------------------------
    def _check_action_effect(self, rule, state):
        pending = self._pending_effect_checks.get(rule["id"])
        if pending is None:
            return None

        elapsed = (state["timestamp"] - pending["triggered_at"]).total_seconds()
        if elapsed < rule["check_after_seconds"]:
            return None  # not time to judge yet

        sensor_state = state["sensors"].get(rule["check_sensor"])
        del self._pending_effect_checks[rule["id"]]  # one-shot per trigger, resolved either way

        if sensor_state is None or sensor_state.confidence <= 0:
            return None  # can't judge without a trustworthy reading

        baseline = pending["baseline"]
        actual_change = sensor_state.value - baseline
        op = rule["expect_change_op"]
        expected = rule["expect_change_value"]

        if op == "decreases_by":
            effective = actual_change <= -expected
        elif op == "increases_by":
            effective = actual_change >= expected
        elif op == "changes_by":
            effective = abs(actual_change) >= expected
        else:
            return None

        if effective:
            return None  # worked as expected -- no anomaly

        return Event(
            id=str(uuid.uuid4()),
            event_type=rule.get("event_type", "ACTION_EFFECT_MISMATCH"),
            asset_id=rule.get("asset") or rule.get("trigger_target") or "",
            sensor_id=rule["check_sensor"],
            severity=rule.get("severity", Severity.URGENT),
            confidence=1.0,
            timestamp=state["timestamp"],
            data={
                "rule_id": rule["id"],
                "description": rule.get("description", ""),
                "baseline": baseline,
                "actual": sensor_state.value,
                "expected_change": expected,
            },
        )

    # -------------------------------------------------
    # RULE TYPE: mass_balance
    # -------------------------------------------------
    def _check_mass_balance(self, rule, state, derived) -> bool:
        level = state["sensors"].get(rule["level_sensor"])
        inflow = state["sensors"].get(rule["inflow_sensor"])
        outflow = state["sensors"].get(rule["outflow_sensor"])
        if not (level and inflow and outflow):
            return False
        if min(level.confidence, inflow.confidence, outflow.confidence) <= 0:
            return False  # don't judge a relationship off an untrusted reading

        net_flow = inflow.value - outflow.value
        level_slope = derived.get(rule["level_sensor"], {}).get("slope", 0.0)

        flow_deadband = rule.get("flow_deadband", 0.5)
        slope_deadband = rule.get("slope_deadband", 0.001)

        if net_flow > flow_deadband and level_slope < -slope_deadband:
            return True   # net inflow, but level is actually dropping
        if net_flow < -flow_deadband and level_slope > slope_deadband:
            return True   # net outflow, but level is actually rising
        return False

    # -------------------------------------------------
    # RULE TYPE: actuator_fault_matrix
    # -------------------------------------------------
    def _check_actuator_fault_matrix(self, rule, state, derived):
        """
        level moving | flow moving | commanded | diagnosis
        -------------+-------------+-----------+-------------------------
        no           | no          | ON        | TANK_CLOGGED
        yes          | no          | ON        | FLOW_SENSOR_FAULTY
        no           | yes         | ON        | LEVEL_SENSOR_SUSPECT
        yes          | yes         | OFF       | TANK_LEAK
        anything else (including ON with both moving consistently, or
        OFF with neither moving) -- normal, no event.

        `commanded_state_provider` names which held reference to query
        (currently only "supply_engine" is wired up -- goal_engine could
        work too via the same get_active_action_type() interface, once
        something commands a target this check would apply to via goal_engine
        instead, e.g. zone valves once zones have their own flow sensors).
        Per-outcome sustained_seconds timing, not the shared
        _mismatch_since (this has 4 possible outcomes per rule, not one
        bool) -- avoids noise on brief readings same as every other
        sustained check in this file.
        """
        provider = getattr(self, rule.get("commanded_state_provider", "supply_engine"), None)
        if provider is None:
            return None

        level = state["sensors"].get(rule["level_sensor"])
        flow = state["sensors"].get(rule["flow_sensor"])
        if level is None or flow is None or level.confidence <= 0 or flow.confidence <= 0:
            return None  # don't judge off an untrusted or missing reading

        commanded_on = provider.get_active_action_type(rule["commanded_target"]) == rule["commanded_action_type"]
        level_slope = derived.get(rule["level_sensor"], {}).get("slope", 0.0)
        level_moving = abs(level_slope) > rule.get("level_slope_deadband", 0.001)
        flow_moving = flow.value > rule.get("flow_deadband", 0.5)

        outcome = None
        if commanded_on and not level_moving and not flow_moving:
            outcome = ("TANK_CLOGGED", Severity.URGENT)
        elif commanded_on and level_moving and not flow_moving:
            outcome = ("FLOW_SENSOR_FAULTY", Severity.REVIEW)
        elif commanded_on and not level_moving and flow_moving:
            outcome = ("LEVEL_SENSOR_SUSPECT", Severity.REVIEW)
        elif not commanded_on and level_moving and flow_moving:
            outcome = ("TANK_LEAK", Severity.CRITICAL)

        if outcome is None:
            # clear any in-progress timers for this rule -- condition's
            # not present right now, so nothing should be "sustained"
            for key in [k for k in self._fault_matrix_since if k.startswith(f"{rule['id']}:")]:
                del self._fault_matrix_since[key]
            return None

        event_type, severity = outcome
        key = f"{rule['id']}:{event_type}"
        since = self._fault_matrix_since.setdefault(key, state["timestamp"])
        sustained = rule.get("sustained_seconds", 300)
        if (state["timestamp"] - since).total_seconds() < sustained:
            return None  # not sustained long enough yet

        return Event(
            id=str(uuid.uuid4()),
            event_type=event_type,
            asset_id=rule.get("asset", rule["commanded_target"]),
            sensor_id=rule["level_sensor"],
            severity=severity,
            confidence=1.0,
            timestamp=state["timestamp"],
            data={
                "level": level.value,
                "flow": flow.value,
                "level_slope": level_slope,
                "commanded_on": commanded_on,
                "rule_id": rule["id"],
            },
        )

    # -------------------------------------------------
    # RULE TYPE: sensor_pair
    # -------------------------------------------------
    def _check_sensor_pair(self, rule, state) -> bool:
        cond = state["sensors"].get(rule["condition_sensor"])
        expect = state["sensors"].get(rule["expect_sensor"])
        if not (cond and expect):
            return False
        if cond.confidence <= 0 or expect.confidence <= 0:
            return False
        if not self._compare(cond.value, rule["condition_op"], rule["condition_value"]):
            return False  # precondition not met -- nothing to check this cycle
        return not self._compare(expect.value, rule["expect_op"], rule["expect_value"])

    # -------------------------------------------------
    # RULE TYPE: commanded_vs_sensor
    # -------------------------------------------------
    def _check_commanded_vs_sensor(self, rule, state) -> bool:
        if self.goal_engine is None:
            return False
        active_type = self.goal_engine.get_active_action_type(rule["commanded_target"])
        if active_type != rule["commanded_action_type"]:
            return False  # not currently commanded -- nothing to check
        expect = state["sensors"].get(rule["expect_sensor"])
        if expect is None or expect.confidence <= 0:
            return False
        return not self._compare(expect.value, rule["expect_op"], rule["expect_value"])

    # -------------------------------------------------
    # RULE TYPE: geofence
    # -------------------------------------------------
    def _check_geofence(self, rule, state) -> bool:
        lat_state = state["sensors"].get(rule["lat_sensor"])
        lon_state = state["sensors"].get(rule["lon_sensor"])
        if not (lat_state and lon_state):
            return False  # no position yet, or signal's dropped -- nothing to check
        if lat_state.confidence <= 0 or lon_state.confidence <= 0:
            return False

        center = self._resolve_center(rule)
        if center is None:
            return False

        distance_m = self._haversine_meters(lat_state.value, lon_state.value, center[0], center[1])
        return distance_m > rule["radius_meters"]

    def _resolve_center(self, rule):
        center = rule.get("center")
        if isinstance(center, (list, tuple)) and len(center) == 2:
            return center
        if isinstance(center, str) and center.startswith("$asset."):
            if self.registry is None:
                return None
            asset = self.registry.assets.get(rule["asset"])
            if asset is None:
                return None
            field = center.split(".", 1)[1]
            value = asset.metadata.get(field)
            if isinstance(value, (list, tuple)) and len(value) == 2:
                return value
        return None

    @staticmethod
    def _haversine_meters(lat1, lon1, lat2, lon2) -> float:
        from math import radians, sin, cos, asin, sqrt
        r = 6371000  # Earth radius, meters
        dlat, dlon = radians(lat2 - lat1), radians(lon2 - lon1)
        a = sin(dlat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
        return 2 * r * asin(sqrt(a))

    @staticmethod
    def _compare(left, op, right) -> bool:
        if op == "lt": return left < right
        if op == "lte": return left <= right
        if op == "gt": return left > right
        if op == "gte": return left >= right
        if op == "eq": return left == right
        if op == "neq": return left != right
        return False
