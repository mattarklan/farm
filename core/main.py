"""
Entrypoint: wires the operator pipeline together against the live
ingestion API and the real config/loader.py registry (via
registry_adapter.py, which wraps its plain-dict output into the object
interface the engines expect). Runs the decision cycle on a fixed
interval. rules.yaml is watched for changes (via RuleWatcher) and
reloaded in place the instant it changes -- no polling in the main loop.
"""
from __future__ import annotations

import threading
import time

import yaml

from core.api_client import IngestionAPIClient, IngestionAPIError
from core.state_builder import StateBuilder
from core.registry_adapter import load_registry
from core.event_engine import EventEngine
from core.confidence_monitor import ConfidenceMonitor
from core.staleness_monitor import StalenessMonitor
from core.range_monitor import RangeMonitor
from core.anomaly_engine import AnomalyEngine
from core.incident_recorder import IncidentRecorder
from core.event_dedup import EventDeduplicator
from core.rule_engine import RuleEngine
from core.goal_engine import GoalEngine
from core.constraint_engine import ConstraintEngine
from core.executor import Executor
from core.pipeline import Pipeline
from core.window_store import RollingWindowStore
from core.derived_engine import DerivedEngine
from core.gui_export import GuiExporter
from core.logger import Logger
from core.rule_watcher import RuleWatcher

INGESTION_API_BASE_URL = "http://localhost:8000"
EVENT_RULES_PATH = "config/event_rules.yaml"
ACTION_RULES_PATH = "config/rules.yaml"
CONSTRAINT_CONFIG_PATH = "config/constraints.yaml"
ACTION_TYPES_PATH = "config/action_types.yaml"
STALENESS_THRESHOLDS_PATH = "config/staleness_thresholds.yaml"
SENSOR_RANGE_THRESHOLDS_PATH = "config/sensor_range_thresholds.yaml"
ANOMALY_RULES_PATH = "config/anomaly_rules.yaml"
CYCLE_INTERVAL_SECONDS = 10  # TEST VALUE -- matches the test ingestor's 10s posting rate
# CYCLE_INTERVAL_SECONDS = 60  # DEPLOYED VALUE -- restore this for the real deployment (60s posting rate)


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_pipeline(registry) -> Pipeline:
    event_rules = _load_yaml(EVENT_RULES_PATH)
    action_rules = _load_yaml(ACTION_RULES_PATH)
    constraint_config = _load_yaml(CONSTRAINT_CONFIG_PATH)
    action_types_config = _load_yaml(ACTION_TYPES_PATH)
    staleness_config = _load_yaml(STALENESS_THRESHOLDS_PATH)
    range_config = _load_yaml(SENSOR_RANGE_THRESHOLDS_PATH)
    anomaly_config = _load_yaml(ANOMALY_RULES_PATH)

    window_store = RollingWindowStore()
    goal_engine = GoalEngine(action_rules, registry=registry, action_types_config=action_types_config)
    incident_recorder = IncidentRecorder(window_store=window_store)

    return Pipeline(
        event_engine=EventEngine(registry, event_rules),
        confidence_monitor=ConfidenceMonitor(registry=registry),
        staleness_monitor=StalenessMonitor(staleness_config, registry=registry),
        range_monitor=RangeMonitor(range_config, registry=registry),
        anomaly_engine=AnomalyEngine(anomaly_config, goal_engine=goal_engine, registry=registry,
                                      incident_recorder=incident_recorder),
        rule_engine=RuleEngine(action_rules),
        goal_engine=goal_engine,
        constraint_engine=ConstraintEngine(constraint_config, action_types_config),
        executor=Executor(action_types_config),
        derived_engine=DerivedEngine(window_store),
        window_store=window_store,
        event_deduplicator=EventDeduplicator(),
    )


def run_forever(interval_seconds: int = CYCLE_INTERVAL_SECONDS):
    logger = Logger()
    logger.startup()

    reload_requested = threading.Event()
    try:
        api_client = IngestionAPIClient(INGESTION_API_BASE_URL)
        registry = load_registry()
        pipeline = build_pipeline(registry)
        state_builder = StateBuilder(api_client)
        gui_exporter = GuiExporter()

        watcher = RuleWatcher(ACTION_RULES_PATH, on_change=reload_requested.set, logger=logger)
        watcher.start()

        while True:
            if reload_requested.is_set():
                reload_requested.clear()
                try:
                    new_action_rules = _load_yaml(ACTION_RULES_PATH)
                    skipped_rule = pipeline.rule_engine.reload(new_action_rules)
                    skipped_goal = pipeline.goal_engine.reload(new_action_rules)
                    for rule, why in skipped_rule + skipped_goal:
                        logger.data_error(f"skipped rule {rule.get('id', '<no id>')}: {why}")
                except Exception as exc:
                    # Bad file on disk -- keep running on the last-known-good
                    # rule set rather than losing everything mid-cycle.
                    logger.data_error(f"rules.yaml reload failed, keeping previous rules: {exc}")

            try:
                state = state_builder.build()
            except IngestionAPIError as exc:
                logger.api_unreachable(str(exc))
                time.sleep(interval_seconds)
                continue

            try:
                result = pipeline.run(state)
            except Exception as exc:  # a bad cycle should not crash core
                logger.cycle_error(exc)
                time.sleep(interval_seconds)
                continue

            actions_with_status = (
                [{"action": a, "status": "allowed"} for a in result["allowed"]]
                + [{"action": b["action"], "status": "blocked", "reason": b["reason"]} for b in result["blocked"]]
                + [{"action": a, "status": "delayed"} for a in result["delayed"]]
                + [{"action": a, "status": "goal_stop"} for a in result["goal_stop_actions"]]
            )
            gui_exporter.export(result["events"], actions_with_status)

            print(
                f"events={len(result['events'])} "
                f"allowed={len(result['allowed'])} "
                f"blocked={len(result['blocked'])} "
                f"delayed={len(result['delayed'])}"
            )
            time.sleep(interval_seconds)

    except KeyboardInterrupt:
        logger.shutdown(clean=True)
    except Exception as exc:
        logger.cycle_error(exc)
        logger.shutdown(clean=False)
        raise


if __name__ == "__main__":
    run_forever()
