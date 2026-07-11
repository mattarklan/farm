#taking weather forecast info and wanting to store it somewhere. it doesnt need to be store long term because its only for 7 days in the future not years. i want to be able to check what the forecast said about temp, rain, wind, against what the weather station on the roof actually recorded. for adjusting expectations on the forecast. if it says its going to rain tomorrow then the operator may decide not to irrigate today depending on the rule. would need to find out how much rain corresponds to how much soil moisture and how accurate the forecast is. then create a json for weather offset.
from pathlib import Path
from rules import Rule, Decision
from validator import load_locked_rules, validate_schema, validate_locked_rules
from conflicts import check_conflicts
from storage import load_rules, save_rules


class GovernanceEngine:
    def __init__(self):
        self.config_path = Path(__file__).parent.parent / "config"
        self.locked = load_locked_rules(self.config_path)
        self.rules_path = self.config_path / "rules.yaml"

    def evaluate(self, proposal: dict) -> Decision:
        # 1. Schema validation
        ok, reason = validate_schema(proposal)
        if not ok:
            return Decision("REJECTED", reason)

        ir = proposal["ir"]

        # 2. Locked rules validation
        ok, reason = validate_locked_rules(ir, self.locked)
        if not ok:
            return Decision("REJECTED", reason)

        # 3. Load current rules
        rules = load_rules(self.rules_path)

        # 4. Conflict detection — returns existing version if one exists
        ok, reason, existing_version = check_conflicts(ir, rules)
        new_version = existing_version + 1  # 1 for new rules, n+1 for updates

        if not ok:
            # Not a hard rejection — overwrite with incremented version
            pass  # fall through to upsert below

        # 5. Upsert rule
        rule_id = ir["action"]["type"]
        rule = {
            "rule_id": rule_id,
            "version": new_version,
            "data": ir,
            "source_proposal": proposal["proposal_id"],
        }
        rules[rule_id] = rule
        save_rules(self.rules_path, rules)

        action = "Updated" if existing_version > 0 else "Accepted"
        return Decision(
            status="APPROVED",
            reason=f"{action} by governance (version {new_version})",
            rule=Rule(
                rule_id=rule_id,
                version=new_version,
                data=ir,
                source_proposal=proposal["proposal_id"],
            ),
        )
