"""
Entrypoint: wires the operator pipeline together against the live
ingestion API and the real config/loader.py registry (via
registry_adapter.py, which wraps its plain-dict output into the object
interface the engines expect). Runs the decision cycle on a fixed
interval. rules.yaml is watched for changes (via RuleWatcher) and
reloaded in place the instant it changes -- no polling in the main loop.
"""
from __future__ import annotations

import json
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path

import yaml

from core.api_client import IngestionAPIClient, IngestionAPIError
from core.state_builder import StateBuilder
from core.registry_adapter import load_registry
from core.models import Action
from core.event_engine import EventEngine
from core.confidence_monitor import ConfidenceMonitor
from core.staleness_monitor import StalenessMonitor
from core.range_monitor import RangeMonitor
from core.anomaly_engine import AnomalyEngine
from core.incident_recorder import IncidentRecorder
from core.event_dedup import EventDeduplicator
from core.rule_engine import RuleEngine
from core.goal_engine import GoalEngine
from core.supply_engine import SupplyEngine
from core.constraint_engine import ConstraintEngine
from core.executor import Executor
from core.action_confirmation import ActionConfirmationMonitor
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
HEARTBEAT_PATH = Path("logs/heartbeat.txt")  # PROCESS_SUPERVISION.md -- external supervisor watches this for hangs


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
    supply_engine = SupplyEngine(registry, goal_engine, action_types_config=action_types_config)
    incident_recorder = IncidentRecorder(window_store=window_store)

    return Pipeline(
        event_engine=EventEngine(registry, event_rules),
        confidence_monitor=ConfidenceMonitor(registry=registry),
        staleness_monitor=StalenessMonitor(staleness_config, registry=registry),
        range_monitor=RangeMonitor(range_config, registry=registry),
        anomaly_engine=AnomalyEngine(anomaly_config, goal_engine=goal_engine, registry=registry,
                                      incident_recorder=incident_recorder, supply_engine=supply_engine),
        rule_engine=RuleEngine(action_rules),
        goal_engine=goal_engine,
        supply_engine=supply_engine,
        constraint_engine=ConstraintEngine(constraint_config, action_types_config),
        executor=Executor(action_types_config),
        derived_engine=DerivedEngine(window_store),
        window_store=window_store,
        event_deduplicator=EventDeduplicator(),
        action_confirmation=ActionConfirmationMonitor(action_types_config=action_types_config),
    )


def force_safe_startup(registry, action_types_config: dict, executor: Executor, action_confirmation, logger: Logger):
    """
    Crash recovery, part 1: force every known actuator to its safe/off
    state unconditionally before the first normal cycle runs. Doesn't
    try to resume "goal X was 40% done" -- a restart can't verify
    physical state, so it doesn't guess at it. Bypasses constraint_engine
    and goal_engine entirely, same reasoning pipeline.py already uses
    for goal-stop actions: a safety action should never be blockable by
    the system it's protecting against.

    Reads registry.assets[*].actuators (see registry_adapter.py) -- a
    NEW field, empty until config/loader.py's real registry data
    includes it. No-ops with a logged note if nothing's there yet,
    rather than failing.

    Registers each safe action with action_confirmation too -- "every
    action should get an ack, regardless of when or why" applies to the
    startup sweep same as any normal-cycle action. Runs before the main
    loop starts, so the first ack resolution for these happens on the
    first normal cycle, same as any other pending ack.
    """
    action_types = (action_types_config or {}).get("action_types", {})
    swept = 0

    for asset in registry.assets.values():
        for actuator_type in getattr(asset, "actuators", []):
            spec = action_types.get(actuator_type) or action_types.get(actuator_type.upper())
            if spec is None:
                logger.data_error(f"startup safe-sweep: unknown action_type '{actuator_type}' on asset '{asset.id}' -- skipped")
                continue
            stop_type = spec.get("stop_action_type")
            if not stop_type:
                continue  # this actuator IS the safe/off action_type -- nothing to force

            safe_action = Action(
                id=str(uuid.uuid4()),
                action_type=stop_type,
                target=asset.id,
                parameters={},
                priority=0,
                created_from_event="startup_safe_sweep",
                timestamp=datetime.now(timezone.utc),
                severity=1,
            )
            result = executor.execute([safe_action])[0]
            swept += 1
            if result.success:
                if action_confirmation is not None:
                    action_confirmation.register(safe_action, {"timestamp": datetime.now(timezone.utc)})
            else:
                logger.data_error(f"startup safe-sweep: failed to force '{asset.id}' safe via {stop_type}: {result.message}")

    if swept == 0:
        logger.data_error("startup safe-sweep: no actuators found on any registry asset -- registry has no actuator data yet, nothing was swept")
    else:
        print(f"[STARTUP] forced {swept} actuator(s) to their safe state")


def reconstruct_last_stopped_at(actions_path: str, goal_engine: GoalEngine, action_types_config: dict):
    """
    Crash recovery, part 2: actions.json survives a crash (gui_export.py,
    30-day retention) even though goal_engine.last_stopped_at doesn't --
    that dict is in-memory only. Reconstructs it from the most recent
    "stop-shaped" action per target, so min_interval_days cooldowns
    (e.g. water_zone_tomato_a_weekly's 7-day gap) survive a restart
    instead of silently resetting to "never watered." Doesn't attempt to
    resume active goals themselves -- see force_safe_startup above for
    why that's deliberate.
    """
    action_types = (action_types_config or {}).get("action_types", {})
    stop_action_types = {spec["stop_action_type"] for spec in action_types.values() if spec.get("stop_action_type")}

    path = Path(actions_path)
    if not path.exists():
        return
    try:
        entries = json.loads(path.read_text())
    except (json.JSONDecodeError, OSError):
        return  # corrupt/unreadable -- fail quiet, same handling gui_export.py uses on its own read

    latest_stop_per_target: dict[str, datetime] = {}
    for entry in entries:
        if entry.get("action_type") not in stop_action_types or not entry.get("target"):
            continue
        ts = datetime.fromisoformat(entry["timestamp"])
        target = entry["target"]
        if target not in latest_stop_per_target or ts > latest_stop_per_target[target]:
            latest_stop_per_target[target] = ts

    goal_engine.last_stopped_at.update(latest_stop_per_target)
    if latest_stop_per_target:
        print(f"[STARTUP] reconstructed last_stopped_at for {len(latest_stop_per_target)} target(s) from actions.json")


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

        action_types_config = _load_yaml(ACTION_TYPES_PATH)
        force_safe_startup(registry, action_types_config, pipeline.executor, pipeline.action_confirmation, logger)
        reconstruct_last_stopped_at("config/actions.json", pipeline.goal_engine, action_types_config)

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

            HEARTBEAT_PATH.parent.mkdir(parents=True, exist_ok=True)
            HEARTBEAT_PATH.write_text(datetime.now(timezone.utc).isoformat())

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
