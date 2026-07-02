"""
Entrypoint: wires the operator pipeline together against the live
ingestion API and the canonical registry loader, and runs one decision
cycle.

ASSUMPTION FLAGGED: imports `farm.config.loader.RegistryLoader` on the
assumption its interface matches the old (now-deleted) core/loader.py's
Registry: `.load(path) -> Registry`, `Registry.get_asset_from_sensor(sensor_id)
-> Asset` with `.id` / `.type`. That file lives in the ingestion project
and hasn't been shown to me, so this hasn't been verified — if the real
interface differs, this is the only place that should need adjusting.

ALSO OPEN: the config file for ConstraintEngine (`constraints.yaml`
below) vs. the config file for RuleEngine (`rules.yaml`) — the handoff
doc explicitly leaves unresolved who writes config/rules.yaml and
whether the constraint engine's rules live in the same file. Kept them
as two separate paths here rather than guessing they should merge.
"""
from __future__ import annotations

import yaml

from farm.config.loader import RegistryLoader  # canonical loader, owned by the ingestion project

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
from core.logger import Logger

INGESTION_API_BASE_URL = "http://localhost:8000"
REGISTRY_CONFIG_PATH = "farm/config/registry"       # the 4 registry YAMLs
EVENT_RULES_PATH = "config/event_rules.yaml"        # rules for EventEngine
ACTION_RULES_PATH = "config/rules.yaml"             # rules for RuleEngine
CONSTRAINT_CONFIG_PATH = "config/constraints.yaml"  # rules for ConstraintEngine


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_pipeline(api_client: IngestionAPIClient) -> Pipeline:
    registry = RegistryLoader().load(REGISTRY_CONFIG_PATH)

    event_rules = _load_yaml(EVENT_RULES_PATH)
    action_rules = _load_yaml(ACTION_RULES_PATH)
    constraint_config = _load_yaml(CONSTRAINT_CONFIG_PATH)

    window_store = RollingWindowStore()

    return Pipeline(
        event_engine=EventEngine(registry, event_rules),
        rule_engine=RuleEngine(action_rules),
        goal_engine=GoalEngine(action_rules),
        constraint_engine=ConstraintEngine(constraint_config),
        executor=Executor(),
        derived_engine=DerivedEngine(window_store),
        window_store=window_store,
        decision_memory=DecisionMemory(),
    )


def run_once() -> dict:
    api_client = IngestionAPIClient(INGESTION_API_BASE_URL)
    pipeline = build_pipeline(api_client)
    state_builder = StateBuilder(api_client)
    logger = Logger()

    state = state_builder.build()
    result = pipeline.run(state)
    logger.log(state, result["events"], result["allowed"])

    return result


if __name__ == "__main__":
    result = run_once()
    print(
        f"events={len(result['events'])} "
        f"allowed={len(result['allowed'])} "
        f"blocked={len(result['blocked'])} "
        f"delayed={len(result['delayed'])}"
    )
