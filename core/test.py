"""
Smoke tests for the pipeline, including the new GoalEngine behaviour:
a goal starts on its trigger event, stays active on a cycle where its
`until` conditions aren't met yet, and stops (with a GOAL_STOPPED event)
once its sensor condition is satisfied.

Uses a mocked ingestion API response rather than a live server. Your
original core/test.py (seeded/read via sensor_store.py directly) wasn't
part of what you pasted into this chat, so this is a fresh replacement.
"""
from __future__ import annotations

from unittest.mock import MagicMock

from core.api_client import IngestionAPIClient
from core.state_builder import StateBuilder
from core.event_engine import EventEngine
from core.rule_engine import RuleEngine
from core.goal_engine import GoalEngine
from core.constraint_engine import ConstraintEngine
from core.executor import Executor
from core.pipeline import Pipeline
from core.window_store import RollingWindowStore
from core.decision_memory import DecisionMemory
from core.derived_engine import DerivedEngine


EVENT_RULES = {
    "rules": [
        {
            "id": "E1",
            "event_id": "E1_BATTERY_LOW",
            "asset_type": "battery",
            "sensor_suffix": "_soc",
            "condition": {"op": "lt", "value": 20},
            "severity": 4,
        },
        {
            "id": "S1",
            "event_id": "S1_SOIL_MOISTURE_LOW",
            "asset_type": "crop",
            "sensor_suffix": "_soil_moisture",
            "condition": {"op": "lt", "value": 0.25},
            "severity": 3,
        },
    ]
}

ACTION_RULES = {
    "rules": [
        {
            "id": "water_zone_b_until_moist",
            "kind": "goal",
            "trigger": {"event_type": "S1_SOIL_MOISTURE_LOW", "zone_id": "zone_b"},
            "start_action": {"type": "OPEN_VALVE", "target": "zone_b", "params": {"flow_rate_lpm": 10}},
            "stop_action": {"type": "CLOSE_VALVE", "target": "zone_b"},
            "until": [
                {"type": "sensor", "sensor": "zone_b_soil_moisture", "op": "gte", "value": 0.3},
                {"type": "timer", "max_duration_minutes": 120},
            ],
            "priority": 10,
        },
    ]
}

CONSTRAINT_CONFIG = {"constraints": []}


def _fake_registry():
    registry = MagicMock()

    def get_asset_from_sensor(sensor_id: str):
        if "battery" in sensor_id:
            return MagicMock(id="battery_1", type="battery")
        return MagicMock(id="zone_b", type="crop")

    registry.get_asset_from_sensor.side_effect = get_asset_from_sensor
    return registry


def _build_pipeline():
    window_store = RollingWindowStore()
    return Pipeline(
        event_engine=EventEngine(_fake_registry(), EVENT_RULES),
        rule_engine=RuleEngine(ACTION_RULES),
        goal_engine=GoalEngine(ACTION_RULES),
        constraint_engine=ConstraintEngine(CONSTRAINT_CONFIG),
        executor=Executor(),
        derived_engine=DerivedEngine(window_store),
        window_store=window_store,
        decision_memory=DecisionMemory(),
    )


def _fake_state(moisture: float):
    api_client = MagicMock(spec=IngestionAPIClient)
    api_client.get_latest_state.return_value = {
        "ts": 1782996741,
        "assets": [
            {
                "asset_id": "zone_b",
                "metrics": {"soil_moisture": moisture},
                "confidence": {"soil_moisture": 1.0},
                "ts": 1782996741,
            },
            {
                "asset_id": "battery_1",
                "metrics": {"soc": 80.0},
                "confidence": {"soc": 1.0},
                "ts": 1782996741,
            },
        ],
    }
    return StateBuilder(api_client).build()


def test_goal_starts_persists_and_stops():
    pipeline = _build_pipeline()

    # cycle 1: moisture low -> event fires -> goal starts (valve opens)
    state = _fake_state(moisture=0.10)
    result = pipeline.run(state)
    assert any(a.action_type == "OPEN_VALVE" for a in result["allowed"]), "goal should have started"
    assert "zone_b" in pipeline.goal_engine.active_goals, "goal should now be active"

    # cycle 2: moisture still low -> goal keeps running, doesn't restart or duplicate
    state = _fake_state(moisture=0.15)
    result = pipeline.run(state)
    assert result["allowed"] == [] or all(a.action_type != "OPEN_VALVE" for a in result["allowed"]), \
        "goal already running - should not start a second time"
    assert "zone_b" in pipeline.goal_engine.active_goals

    # cycle 3: moisture crosses threshold -> goal stops (valve closes), GOAL_STOPPED event logged
    state = _fake_state(moisture=0.35)
    result = pipeline.run(state)
    assert any(a.action_type == "CLOSE_VALVE" for a in result["goal_stop_actions"]), "goal should have stopped"
    assert "zone_b" not in pipeline.goal_engine.active_goals, "goal should be cleared once stopped"
    assert any(e.event_type == "GOAL_STOPPED" for e in result["events"]), "stop should be logged as an event"

    print("OK - goal start/persist/stop lifecycle verified")


def test_original_smoke_still_passes():
    pipeline = _build_pipeline()
    state = _fake_state(moisture=0.5)  # moisture fine, only battery event should fire
    result = pipeline.run(state)
    event_types = [e.event_type for e in result["events"]]
    assert "E1_BATTERY_LOW" not in event_types  # battery at 80%, shouldn't fire
    print("OK - baseline pipeline still runs cleanly")


if __name__ == "__main__":
    test_goal_starts_persists_and_stops()
    test_original_smoke_still_passes()
