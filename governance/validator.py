from pathlib import Path
import yaml

REQUIRED_FIELDS = ("proposal_id", "intent", "ir")


def load_locked_rules(config_path: Path) -> dict:
    path = config_path / "locked_rules.yaml"
    if not path.exists():
        raise FileNotFoundError(
            f"locked_rules.yaml not found at {path}. "
            "This file is required for the governance engine to start."
        )
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def validate_schema(proposal: dict) -> tuple[bool, str]:
    for field in REQUIRED_FIELDS:
        if field not in proposal:
            return False, f"Missing field: {field}"
    ir = proposal["ir"]
    if "action" not in ir:
        return False, "Missing IR action"
    if "type" not in ir["action"]:
        return False, "Missing IR action.type"
    return True, "OK"


def validate_locked_rules(ir: dict, locked: dict) -> tuple[bool, str]:
    constraints = ir.get("constraints", {})
    runtime = constraints.get("max_runtime")

    # No runtime constraint supplied — check if one is required.
    water_limits = locked.get("water", {})
    max_allowed = water_limits.get("max_pump_runtime_seconds")

    if max_allowed is None:
        # No limit configured, nothing to enforce.
        return True, "OK"

    if runtime is None:
        # A limit exists but the proposal didn't set one — reject rather than silently skip.
        return False, "max_runtime constraint is required but was not provided"

    if runtime > max_allowed:
        return False, f"max_runtime {runtime}s exceeds limit of {max_allowed}s"

    return True, "OK"
