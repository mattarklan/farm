from pathlib import Path
import yaml
from engine import GovernanceEngine

# Reset rules before each run so tests are idempotent
rules_path = Path(__file__).parent.parent / "config" / "rules.yaml"
rules_path.write_text("")

engine = GovernanceEngine()

tests = [
    {
        "name": "Valid proposal — new rule",
        "proposal": {
            "proposal_id": "1",
            "intent": "fill tanks",
            "ir": {
                "action": {"type": "pump_fill"},
                "constraints": {"max_runtime": 1200},
            },
        },
        "expected": "APPROVED",
    },
    {
        "name": "Same action type — should increment version",
        "proposal": {
            "proposal_id": "2",
            "intent": "fill tanks with updated runtime",
            "ir": {
                "action": {"type": "pump_fill"},
                "constraints": {"max_runtime": 600},
            },
        },
        "expected": "APPROVED",
        "expected_version": 2,
    },
    {
        "name": "Missing action",
        "proposal": {
            "proposal_id": "3",
            "intent": "bad proposal",
            "ir": {},
        },
        "expected": "REJECTED",
    },
    {
        "name": "Runtime exceeds locked limit",
        "proposal": {
            "proposal_id": "4",
            "intent": "run pump forever",
            "ir": {
                "action": {"type": "pump_fill"},
                "constraints": {"max_runtime": 999999},
            },
        },
        "expected": "REJECTED",
    },
    {
        "name": "Missing max_runtime constraint",
        "proposal": {
            "proposal_id": "5",
            "intent": "pump with no runtime set",
            "ir": {
                "action": {"type": "pump_drain"},
                "constraints": {},
            },
        },
        "expected": "REJECTED",
    },
]

passed = 0
for t in tests:
    result = engine.evaluate(t["proposal"])
    status_ok = result.status == t["expected"]
    version_ok = True
    if "expected_version" in t:
        version_ok = result.rule is not None and result.rule.version == t["expected_version"]
    ok = status_ok and version_ok
    label = "PASS" if ok else "FAIL"
    print(f"{label}  {t['name']}")
    if not ok:
        print(f"       got status={result.status} reason={result.reason}", end="")
        if "expected_version" in t:
            got_v = result.rule.version if result.rule else "N/A"
            print(f" version={got_v}", end="")
        print()
    if ok:
        passed += 1

print(f"\n{passed}/{len(tests)} passed")
