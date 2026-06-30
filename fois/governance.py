"""
fois/governance.py

Governance layer for FOIS crop opportunity proposals.

Responsibility:
    Accept or reject proposed rule changes from FOIS before they touch
    the live operator config. Three sequential gates:

    1. Validator     — does the proposal conform to the rule schema?
    2. Feasibility   — can the operator's current resources support it?
    3. Policy        — does it violate any standing operational rules?

    All three must pass. First failure stops processing and records a rejection.

Usage:
    governor = GovernanceLayer(schema_path, state, db)
    result = governor.evaluate_proposal(proposal_path)
"""

import os
import shutil
import logging
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import yaml

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Result dataclass (no external deps)
# ---------------------------------------------------------------------------

class GovernanceResult:
    def __init__(self, proposal_id: str, accepted: bool, gate: str, reasons: list[str]):
        self.proposal_id = proposal_id
        self.accepted = accepted
        self.gate = gate          # "validator" | "feasibility" | "policy" | "accepted"
        self.reasons = reasons
        self.decided_at = datetime.now(timezone.utc).isoformat()

    def to_dict(self) -> dict:
        return {
            "proposal_id": self.proposal_id,
            "accepted": self.accepted,
            "gate": self.gate,
            "reasons": self.reasons,
            "decided_at": self.decided_at,
        }

    def __repr__(self):
        status = "ACCEPTED" if self.accepted else f"REJECTED at {self.gate}"
        return f"<GovernanceResult {self.proposal_id} {status}>"


# ---------------------------------------------------------------------------
# Gate 1 — Validator
# ---------------------------------------------------------------------------

class ProposalValidator:
    """
    Validates that a proposal's rules conform to the operator rule schema.
    Uses the placeholder schema at config/schema/rule_schema.yaml.
    Replace schema logic here when the real schema is available.
    """

    REQUIRED_RULE_FIELDS = {
        "rule_id", "description", "enabled", "source",
        "proposal_id", "trigger", "conditions", "actions",
        "priority", "expires_at",
    }

    VALID_SOURCES = {"operator", "fois"}
    VALID_OPERATORS = {"lt", "lte", "gt", "gte", "eq", "neq"}
    VALID_ACTION_TYPES = {
        "open_valve", "close_valve", "start_pump", "stop_pump",
        "set_irrigation_schedule", "log_only", "trigger_alert",
    }

    def __init__(self, schema: dict):
        self.schema = schema
        self.fois_constraints = schema.get("fois_constraints", {})

    def validate(self, proposal: dict) -> tuple[bool, list[str]]:
        errors = []

        # Top-level proposal fields
        for field in ["proposal_id", "generated_at", "expires_at", "crop",
                      "resources", "proposed_rules"]:
            if field not in proposal:
                errors.append(f"Proposal missing required field: {field}")

        if errors:
            return False, errors

        rules = proposal.get("proposed_rules", [])
        if not rules:
            errors.append("Proposal contains no proposed_rules")
            return False, errors

        for i, rule in enumerate(rules):
            rule_errors = self._validate_rule(rule, i)
            errors.extend(rule_errors)

        return len(errors) == 0, errors

    def _validate_rule(self, rule: dict, index: int) -> list[str]:
        errors = []
        ref = rule.get("rule_id", f"rule[{index}]")

        # Required fields
        missing = self.REQUIRED_RULE_FIELDS - set(rule.keys())
        for f in missing:
            errors.append(f"{ref}: missing required field '{f}'")

        # Source must be fois for FOIS-generated rules
        if rule.get("source") not in self.VALID_SOURCES:
            errors.append(f"{ref}: invalid source '{rule.get('source')}'")

        if rule.get("source") == "fois" and not rule.get("proposal_id"):
            errors.append(f"{ref}: source=fois requires proposal_id")

        # Priority ceiling
        max_priority = self.fois_constraints.get("max_priority", 60)
        if isinstance(rule.get("priority"), int):
            # Lower number = higher priority. FOIS may not claim priority numbers
            # below (more important than) the ceiling.
            if rule["priority"] < max_priority:
                errors.append(
                    f"{ref}: priority {rule['priority']} is above FOIS ceiling of {max_priority} "
                    f"(lower number = higher priority — FOIS cannot claim high-priority slots below {max_priority})"
                )
        else:
            errors.append(f"{ref}: priority must be an integer")

        # Expiry required for FOIS rules
        if self.fois_constraints.get("expiry_required") and not rule.get("expires_at"):
            errors.append(f"{ref}: FOIS rules must have expires_at set")

        # Trigger
        trigger = rule.get("trigger", {})
        if not trigger.get("event_type"):
            errors.append(f"{ref}: trigger.event_type is required")

        # Zone restriction
        allowed_zones = self.fois_constraints.get("allowed_zone_ids", [])
        zone_id = trigger.get("zone_id")
        if zone_id and allowed_zones and zone_id not in allowed_zones:
            errors.append(f"{ref}: zone_id '{zone_id}' not in FOIS allowed zones")

        # Conditions
        for j, cond in enumerate(rule.get("conditions", [])):
            if not cond.get("sensor"):
                errors.append(f"{ref} condition[{j}]: missing sensor")
            if cond.get("operator") not in self.VALID_OPERATORS:
                errors.append(f"{ref} condition[{j}]: invalid operator '{cond.get('operator')}'")
            if "value" not in cond:
                errors.append(f"{ref} condition[{j}]: missing value")

        # Actions
        actions = rule.get("actions", [])
        if not actions:
            errors.append(f"{ref}: at least one action is required")

        forbidden = set(self.fois_constraints.get("forbidden_action_types", []))
        for j, action in enumerate(actions):
            atype = action.get("action_type")
            if atype not in self.VALID_ACTION_TYPES:
                errors.append(f"{ref} action[{j}]: unknown action_type '{atype}'")
            if atype in forbidden:
                errors.append(f"{ref} action[{j}]: action_type '{atype}' is forbidden for FOIS rules")
            if not action.get("target"):
                errors.append(f"{ref} action[{j}]: missing target")

        return errors


# ---------------------------------------------------------------------------
# Gate 2 — Feasibility
# ---------------------------------------------------------------------------

class FeasibilityChecker:
    """
    Checks whether the operator's current resource state can support the
    resource demands declared in the proposal.

    Reads from the state dict produced by StateBuilder.
    """

    # Thresholds — could be moved to config later
    MIN_TANK_HEADROOM_PERCENT = 20   # tank must have this much above minimum to commit
    MIN_BATTERY_SOC_PERCENT = 30     # battery must be above this to accept new power loads
    MAX_DAILY_POWER_ADDITION_KWH = 10.0  # don't accept proposals adding more than this

    def __init__(self, state: dict):
        self.state = state

    def check(self, proposal: dict) -> tuple[bool, list[str]]:
        failures = []
        resources = proposal.get("resources", {})

        self._check_water(resources.get("water", {}), failures)
        self._check_power(resources.get("power", {}), failures)

        return len(failures) == 0, failures

    def _check_water(self, water: dict, failures: list):
        daily = water.get("daily_litres", 0)
        if daily <= 0:
            return

        # Check tank levels from state
        # Expecting state to have sensor readings keyed by sensor_id
        sensors = self.state.get("sensors", {})

        tank_level = sensors.get("tank_1_level") or sensors.get("header_tank_level")
        if tank_level is not None:
            if tank_level < self.MIN_TANK_HEADROOM_PERCENT:
                failures.append(
                    f"Feasibility: tank level {tank_level}% is below minimum "
                    f"{self.MIN_TANK_HEADROOM_PERCENT}% required to accept water commitment"
                )
        else:
            # No tank data — warn but don't hard-fail (sensor may be offline)
            logger.warning("Feasibility: tank level sensor not available in state")

    def _check_power(self, power: dict, failures: list):
        daily_kwh = power.get("daily_kwh", 0)
        if daily_kwh <= 0:
            return

        sensors = self.state.get("sensors", {})

        # Battery SOC check
        battery_soc = sensors.get("battery_soc")
        if battery_soc is not None:
            if battery_soc < self.MIN_BATTERY_SOC_PERCENT:
                failures.append(
                    f"Feasibility: battery SOC {battery_soc}% is below minimum "
                    f"{self.MIN_BATTERY_SOC_PERCENT}% required to accept new power load"
                )

        # Power addition ceiling
        if daily_kwh > self.MAX_DAILY_POWER_ADDITION_KWH:
            failures.append(
                f"Feasibility: proposed daily power demand {daily_kwh} kWh exceeds "
                f"maximum addition of {self.MAX_DAILY_POWER_ADDITION_KWH} kWh"
            )


# ---------------------------------------------------------------------------
# Gate 3 — Policy
# ---------------------------------------------------------------------------

class PolicyChecker:
    """
    Checks whether the proposal violates standing operational policy.

    Policy rules are things the operator always enforces regardless of
    resource availability — seasonal restrictions, rotation requirements,
    active maintenance windows, etc.

    Extend this class as operational policy grows.
    """

    def __init__(self, state: dict, active_proposals: list[dict]):
        self.state = state
        self.active_proposals = active_proposals  # already-accepted proposals

    def check(self, proposal: dict) -> tuple[bool, list[str]]:
        failures = []

        # NOTE: zone conflict is no longer a hard rejection here.
        # A newer proposal for an already-committed zone is allowed to pass
        # this gate; GovernanceLayer._accept() handles superseding the old
        # accepted proposal for that zone. See find_superseded_proposal().
        self._check_proposal_not_expired(proposal, failures)
        self._check_no_maintenance_window(proposal, failures)

        return len(failures) == 0, failures

    def find_superseded_proposal(self, proposal: dict) -> Optional[dict]:
        """
        Return the currently-active accepted proposal for this proposal's
        zone, if one exists. Used by GovernanceLayer to retire the old
        proposal when a new one for the same zone is accepted.
        Only one proposal may be active per growing zone at a time.
        """
        zone_id = proposal.get("crop", {}).get("zone_id")
        if not zone_id:
            return None

        for active in self.active_proposals:
            if active.get("crop", {}).get("zone_id") == zone_id:
                return active
        return None

    def _check_proposal_not_expired(self, proposal: dict, failures: list):
        expires_at_str = proposal.get("expires_at")
        if not expires_at_str:
            return
        try:
            expires_at = datetime.fromisoformat(expires_at_str)
            if datetime.now(timezone.utc) > expires_at:
                failures.append(
                    f"Policy: proposal expired at {expires_at_str} and can no longer be accepted"
                )
        except ValueError:
            failures.append(f"Policy: could not parse expires_at '{expires_at_str}'")

    def _check_no_maintenance_window(self, proposal: dict, failures: list):
        """If the operator state flags an active maintenance window, block new proposals."""
        maintenance_active = self.state.get("maintenance_window_active", False)
        if maintenance_active:
            failures.append(
                "Policy: operator is in a maintenance window — no new proposals accepted"
            )


# ---------------------------------------------------------------------------
# GovernanceLayer — orchestrates all three gates
# ---------------------------------------------------------------------------

class GovernanceLayer:
    """
    Top-level governance orchestrator.

    Instantiate once per pipeline run with current state.
    Call evaluate_proposal() for each pending proposal file.
    """

    PENDING_DIR = "fois/proposals/pending"
    ACCEPTED_DIR = "fois/proposals/accepted"
    REJECTED_DIR = "fois/proposals/rejected"
    VERDICT_LOG = "fois/proposals/verdicts.yaml"

    SUPERSEDED_DIR = "fois/proposals/superseded"

    def __init__(self, schema_path: str, state: dict, active_proposals: Optional[list] = None):
        with open(schema_path) as f:
            schema = yaml.safe_load(f)

        self.validator = ProposalValidator(schema)
        self.state = state

        # Always load accepted proposals from disk as the source of truth.
        # Any active_proposals passed in are merged in addition (useful for
        # tests that want to simulate a proposal not yet flushed to disk),
        # but disk is authoritative — this is what closes the bug where
        # re-running governance didn't know about prior acceptances.
        self.active_proposals = self._load_active_proposals_from_disk()
        if active_proposals:
            existing_ids = {p.get("proposal_id") for p in self.active_proposals}
            for p in active_proposals:
                if p.get("proposal_id") not in existing_ids:
                    self.active_proposals.append(p)

    def _load_active_proposals_from_disk(self) -> list[dict]:
        accepted = []
        path = Path(self.ACCEPTED_DIR)
        if not path.exists():
            return accepted
        for f in sorted(path.glob("*.yaml")):
            try:
                with open(f) as fp:
                    proposal = yaml.safe_load(fp)
                    if proposal:
                        accepted.append(proposal)
            except Exception as e:
                logger.warning(f"Governance: could not load accepted proposal {f}: {e}")
        return accepted

    def evaluate_proposal(self, proposal_path: str) -> GovernanceResult:
        """Load a proposal file, run all three gates, move file, record verdict."""
        with open(proposal_path) as f:
            proposal = yaml.safe_load(f)

        proposal_id = proposal.get("proposal_id", Path(proposal_path).stem)

        logger.info(f"Governance: evaluating proposal {proposal_id}")

        # Gate 1 — Validate
        valid, errors = self.validator.validate(proposal)
        if not valid:
            return self._reject(proposal_id, proposal_path, "validator", errors)

        # Gate 2 — Feasibility
        feasible, errors = FeasibilityChecker(self.state).check(proposal)
        if not feasible:
            return self._reject(proposal_id, proposal_path, "feasibility", errors)

        # Gate 3 — Policy
        policy_checker = PolicyChecker(self.state, self.active_proposals)
        policy_ok, errors = policy_checker.check(proposal)
        if not policy_ok:
            return self._reject(proposal_id, proposal_path, "policy", errors)

        return self._accept(proposal_id, proposal_path, proposal, policy_checker)

    def evaluate_all_pending(self) -> list[GovernanceResult]:
        """Process every YAML file in the pending directory."""
        results = []
        pending = Path(self.PENDING_DIR)
        if not pending.exists():
            logger.warning(f"Governance: pending directory {self.PENDING_DIR} does not exist")
            return results

        for path in sorted(pending.glob("*.yaml")):
            result = self.evaluate_proposal(str(path))
            results.append(result)
            logger.info(f"Governance: {result}")

        return results

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _accept(self, proposal_id: str, proposal_path: str, proposal: dict,
                policy_checker: "PolicyChecker") -> GovernanceResult:
        # Check if this proposal supersedes an existing accepted proposal
        # for the same zone. Only one proposal may be active per zone.
        superseded = policy_checker.find_superseded_proposal(proposal)

        reasons = ["Passed all governance gates"]

        if superseded:
            superseded_id = superseded.get("proposal_id")
            zone_id = proposal.get("crop", {}).get("zone_id")
            self._retire_proposal(superseded_id)
            reasons.append(
                f"Superseded prior proposal '{superseded_id}' for zone '{zone_id}'"
            )
            logger.info(
                f"Governance: {proposal_id} supersedes {superseded_id} in zone {zone_id}"
            )
            # Remove the retired one from our in-memory active list so a
            # later proposal in this same batch doesn't try to supersede
            # something already gone.
            self.active_proposals = [
                p for p in self.active_proposals
                if p.get("proposal_id") != superseded_id
            ]

        result = GovernanceResult(
            proposal_id=proposal_id,
            accepted=True,
            gate="accepted",
            reasons=reasons,
        )
        self._move_proposal(proposal_path, self.ACCEPTED_DIR)
        self._append_verdict(result)

        # Add the newly-accepted proposal to the in-memory active list so
        # subsequent proposals evaluated in this same batch see it too.
        self.active_proposals.append(proposal)

        logger.info(f"Governance: ACCEPTED {proposal_id}")
        return result

    def _retire_proposal(self, proposal_id: str):
        """Move a superseded proposal out of accepted/ into superseded/."""
        accepted_path = Path(self.ACCEPTED_DIR) / f"{proposal_id}.yaml"
        if not accepted_path.exists():
            logger.warning(
                f"Governance: tried to retire {proposal_id} but file not found in {self.ACCEPTED_DIR}"
            )
            return
        Path(self.SUPERSEDED_DIR).mkdir(parents=True, exist_ok=True)
        dest = Path(self.SUPERSEDED_DIR) / accepted_path.name
        shutil.move(str(accepted_path), str(dest))
        logger.info(f"Governance: retired {proposal_id} → {self.SUPERSEDED_DIR}")

    def _reject(self, proposal_id: str, proposal_path: str, gate: str, reasons: list[str]) -> GovernanceResult:
        result = GovernanceResult(
            proposal_id=proposal_id,
            accepted=False,
            gate=gate,
            reasons=reasons,
        )
        self._move_proposal(proposal_path, self.REJECTED_DIR)
        self._append_verdict(result)
        logger.warning(f"Governance: REJECTED {proposal_id} at {gate}: {reasons}")
        return result

    def _move_proposal(self, proposal_path: str, dest_dir: str):
        Path(dest_dir).mkdir(parents=True, exist_ok=True)
        dest = Path(dest_dir) / Path(proposal_path).name
        shutil.move(proposal_path, str(dest))

    def _append_verdict(self, result: GovernanceResult):
        """Append verdict to a running YAML log — the learning signal for FOIS."""
        log_path = Path(self.VERDICT_LOG)
        log_path.parent.mkdir(parents=True, exist_ok=True)

        verdicts = []
        if log_path.exists():
            with open(log_path) as f:
                verdicts = yaml.safe_load(f) or []

        verdicts.append(result.to_dict())

        with open(log_path, "w") as f:
            yaml.dump(verdicts, f, default_flow_style=False, sort_keys=False)