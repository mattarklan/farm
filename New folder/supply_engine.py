"""
Owns tank valve commands and orchestration -- scales to however many
tank/crop assets exist in the registry (currently 6 tanks/2 zones,
designed for up to ~20 of each per the growth plan), no hardcoded IDs
anywhere in this file. Fault DIAGNOSIS lives in anomaly_engine.py (see
below) -- this engine only decides what should be open and commands it.

TANK FAULT MATRIX lives in anomaly_engine.py as a new rule type
(actuator_fault_matrix) -- this engine only tracks and exposes its own
commanded state (get_active_action_type, below) for that check to read,
same pattern anomaly_engine already uses for goal_engine.

ZONE -> TANK PREFERENCE: a zone's registry `dependencies` list, filtered
to type=="tank" entries, IN ORDER -- first is primary, rest are
buffers, tried in the order listed. Different zones can order the same
tanks differently (e.g. tank_1 primary for zone_a, backup for zone_e).
This overloads list ORDER as meaning for the first time -- if
dependencies aren't deliberately ordered by preference when populated,
this logic won't do what's intended. Flagging since it's a real
assumption, not a small one.

FLOW MEASUREMENT LIMITATION, read this before touching the flow logic:
tanks have flow sensors, zones currently don't -- if two zones share a
tank, there's no way to split its flow reading between them. This
engine therefore works off AGGREGATE flow (sum across currently-open
tanks) vs AGGREGATE demand (10 L/min x active zone count), not true
per-zone attribution. Once each zone has its own flow sensor, this
should be revisited to compare each zone's OWN flow against its own
10L/min target directly, rather than working from tank-side aggregates
-- that'll make the pause/resume decision far more precise (right now
it can only say "the zones sharing this supply, collectively, aren't
getting enough," not which one specifically is short).

PAUSE/RESUME: calls goal_engine.pause_goal()/resume_goal() directly
(same "held reference" pattern anomaly_engine already uses for
commanded_vs_sensor checks) rather than routing through events -- this
needs to react within the same cycle, not one cycle late. Those two
methods are NEW on GoalEngine, added alongside this engine, not before
it existed.

UNAVAILABILITY: a tank counts as unavailable for routing if it's empty
(level <= 10%), its sensors are already untrusted (confidence <= 0,
same signal every other engine already respects), OR anomaly_engine's
actuator_fault_matrix flagged it (TANK_CLOGGED/TANK_LEAK/
FLOW_SENSOR_FAULTY/LEVEL_SENSOR_SUSPECT) -- read from fault_events,
which pipeline.py carries forward one cycle same as it already does for
goal_engine's fault_event conditions (anomaly_engine runs AFTER this
engine each cycle, so a same-cycle fault can't reach here any faster).
"""
from __future__ import annotations

import uuid

from core.models import Action, Event
from core.severity import Severity

FLOW_PER_ZONE_LMIN = 10.0   # placeholder per your figure -- tune once real flow data exists
EMPTY_LEVEL_PERCENT = 10.0

TANK_FAULT_EVENT_TYPES = {"TANK_CLOGGED", "TANK_LEAK", "FLOW_SENSOR_FAULTY", "LEVEL_SENSOR_SUSPECT"}


class SupplyEngine:
    def __init__(self, registry, goal_engine, action_types_config: dict | None = None):
        self.registry = registry
        self.goal_engine = goal_engine
        self.action_types = (action_types_config or {}).get("action_types", {})
        self._tank_commanded_open: dict[str, bool] = {}   # tank asset id -> bool, THIS engine's own command record -- source of truth anomaly_engine reads via get_active_action_type
        self._unavailable_notified: set[str] = set()        # zone ids already flagged SUPPLY_UNAVAILABLE this episode

    def get_active_action_type(self, target: str) -> str | None:
        """Mirrors goal_engine's method of the same name/shape -- lets
        anomaly_engine treat this as a commanded-state provider the same
        way it already treats goal_engine, for the new
        actuator_fault_matrix rule type."""
        return "OPEN_VALVE" if self._tank_commanded_open.get(target, False) else None

    # -------------------------------------------------
    # PUBLIC ENTRY POINT
    # -------------------------------------------------
    def process(self, state: dict, derived: dict, fault_events: list | None = None) -> tuple[list, list]:
        """Returns (tank_valve_actions, events). Tank valve actions still
        go through constraint_engine like everything else -- this
        engine decides WHAT should be open, not whether it's safe to;
        that's still constraint_engine's job (e.g. tank overflow rules
        still apply to a tank this engine wants to fill)."""
        fault_events = fault_events or []
        tanks = self._tank_assets()
        unavailable_tanks = self._unavailable_tanks(tanks, state, fault_events)
        return self._route_supply(tanks, unavailable_tanks, state)

    # -------------------------------------------------
    # REGISTRY HELPERS
    # -------------------------------------------------
    def _tank_assets(self) -> list:
        return [a for a in self.registry.assets.values() if a.type == "tank"]

    def _zone_assets(self) -> list:
        return [a for a in self.registry.assets.values() if a.type == "crop"]

    def _tank_preference_for_zone(self, zone_asset) -> list[str]:
        return [
            dep for dep in getattr(zone_asset, "dependencies", [])
            if (t := self.registry.get_asset(dep)) is not None and t.type == "tank"
        ]

    def _unavailable_tanks(self, tanks, state, fault_events) -> set[str]:
        faulted = {ev.asset_id for ev in fault_events if ev.event_type in TANK_FAULT_EVENT_TYPES}
        unavailable = set(faulted)
        for tank in tanks:
            level = state["sensors"].get(f"{tank.id}_level")
            if level is None or level.confidence <= 0 or level.value <= EMPTY_LEVEL_PERCENT:
                unavailable.add(tank.id)
        return unavailable

    # -------------------------------------------------
    # ROUTING + FLOW-FAIRNESS PAUSE/RESUME
    # -------------------------------------------------
    def _route_supply(self, tanks, unavailable_tanks, state):
        actions = []
        events = []

        active_zone_goals = [
            g for target, g in self.goal_engine.active_goals.items()
            if (z := self.registry.get_asset(target)) is not None and z.type == "crop"
        ]
        unpaused = [g for g in active_zone_goals if not getattr(g, "paused", False)]
        paused = [g for g in active_zone_goals if getattr(g, "paused", False)]

        needed_tanks: set[str] = set()
        for goal in unpaused:
            zone = self.registry.get_asset(goal.target)
            preference = self._tank_preference_for_zone(zone)
            available = [t for t in preference if t not in unavailable_tanks]
            if not available:
                if goal.target not in self._unavailable_notified:
                    self._unavailable_notified.add(goal.target)
                    events.append(self._supply_unavailable_event(goal.target, preference, state))
                continue
            self._unavailable_notified.discard(goal.target)
            needed_tanks.add(available[0])  # start with primary available tank

        total_flow = self._sum_flow(needed_tanks, state)
        demand = FLOW_PER_ZONE_LMIN * len(unpaused)

        # not enough flow yet -- bring in each zone's next available
        # buffer tank in turn until flow catches up or options run out
        if total_flow < demand:
            for goal in unpaused:
                if total_flow >= demand:
                    break
                zone = self.registry.get_asset(goal.target)
                preference = self._tank_preference_for_zone(zone)
                for tank_id in preference:
                    if tank_id in needed_tanks or tank_id in unavailable_tanks:
                        continue
                    needed_tanks.add(tank_id)
                    total_flow = self._sum_flow(needed_tanks, state)
                    break

        # still short after exhausting every available tank -- pause the
        # most recently started zone to free up flow for the others,
        # per "pause the latest zone's irrigation... go again after one
        # of the others has finished"
        if total_flow < demand and unpaused:
            newest = max(unpaused, key=lambda g: g.started_at)
            pause_action = self.goal_engine.pause_goal(newest.target, state)
            if pause_action is not None:
                actions.append(pause_action)
            events.append(Event(
                id=str(uuid.uuid4()), event_type="ZONE_IRRIGATION_PAUSED",
                asset_id=newest.target, sensor_id="", severity=Severity.INFO,
                confidence=1.0, timestamp=state["timestamp"],
                data={"reason": "insufficient aggregate flow across available tanks", "total_flow": total_flow, "demand": demand},
            ))

        # flow has headroom and something's waiting -- resume the
        # longest-paused zone. Recomputed against the tanks already
        # needed (not asking for more) to avoid immediately re-triggering
        # the same pause next cycle.
        elif paused and total_flow >= demand + FLOW_PER_ZONE_LMIN:
            oldest_paused = min(paused, key=lambda g: g.paused_since)
            resume_action = self.goal_engine.resume_goal(oldest_paused.target, state)
            if resume_action is not None:
                actions.append(resume_action)
            events.append(Event(
                id=str(uuid.uuid4()), event_type="ZONE_IRRIGATION_RESUMED",
                asset_id=oldest_paused.target, sensor_id="", severity=Severity.INFO,
                confidence=1.0, timestamp=state["timestamp"], data={},
            ))

        # emit valve actions for the diff between needed and currently-open
        currently_open = {tid for tid, open_ in self._tank_commanded_open.items() if open_}
        for tank_id in needed_tanks - currently_open:
            actions.append(self._tank_action(tank_id, "OPEN_VALVE", state))
            self._tank_commanded_open[tank_id] = True
        for tank_id in currently_open - needed_tanks:
            actions.append(self._tank_action(tank_id, "CLOSE_VALVE", state))
            self._tank_commanded_open[tank_id] = False

        return actions, events

    def _sum_flow(self, tank_ids, state) -> float:
        total = 0.0
        for tid in tank_ids:
            sensor = state["sensors"].get(f"{tid}_flow_out")
            if sensor is not None and sensor.confidence > 0:
                total += sensor.value
        return total

    def _tank_action(self, tank_id: str, action_type: str, state) -> Action:
        spec = self.action_types.get(action_type, {})
        return Action(
            id=str(uuid.uuid4()),
            action_type=action_type,
            target=tank_id,
            parameters={},
            priority=5,
            created_from_event="supply_engine",
            timestamp=state["timestamp"],
            severity=spec.get("severity", 3),
        )

    def _supply_unavailable_event(self, zone_id, preference, state) -> Event:
        return Event(
            id=str(uuid.uuid4()), event_type="SUPPLY_UNAVAILABLE",
            asset_id=zone_id, sensor_id="", severity=Severity.CRITICAL,
            confidence=1.0, timestamp=state["timestamp"],
            data={"tanks_checked": preference, "reason": "every tank in this zone's preference list is empty/faulty/leaking"},
        )
