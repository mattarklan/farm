#rule_engine.py
from datetime import datetime
from core.models import Action
from core.goal_engine import infer_is_goal
import uuid


class RuleEngine:
    def __init__(self, rule_config: dict):
        self.rules, _ = self._filter_active(rule_config.get("rules", []))

    def reload(self, rule_config: dict):
        """
        Swaps in a new rule set in place (called when rules.yaml changes
        on disk -- e.g. governance approved a new proposal). Doesn't
        recreate the engine, so nothing else about it resets.
        Returns a list of (rule, reason) for anything skipped, so the
        caller can log it.
        """
        self.rules, skipped = self._filter_active(rule_config.get("rules", []))
        return skipped

    @staticmethod
    def _filter_active(rules):
        now = datetime.utcnow()
        active, skipped = [], []
        for r in rules:
            if "id" not in r:
                skipped.append((r, "missing id"))
                continue
            if infer_is_goal(r):
                continue  # GoalEngine's job, not ours -- shared inference keeps the two in sync
            if not r.get("enabled", True):
                continue  # disabled, not an error -- just inactive
            expires_at = r.get("expires_at")
            if expires_at:
                try:
                    if datetime.fromisoformat(expires_at) <= now:
                        continue  # expired, not an error -- just inactive
                except ValueError:
                    skipped.append((r, f"unparseable expires_at: {expires_at!r}"))
                    continue
            active.append(r)
        return active, skipped

    def process(self, events, state):
        actions = []
        for event in events:
            for rule in self.rules:
                # -------------------------
                # 1. trigger match
                # -------------------------
                if event.event_type not in rule.get("triggers", []):
                    continue
                # -------------------------
                # 2. conditions (ALL must pass)
                # -------------------------
                if not self._evaluate_conditions(rule.get("conditions", []), state):
                    continue
                # -------------------------
                # 3. expand actions
                # -------------------------
                for action_def in rule.get("actions", []):
                    actions.append(
                        Action(
                            id=str(uuid.uuid4()),
                            action_type=action_def["type"],
                            target=action_def.get("zone") or action_def.get("target"),
                            parameters=action_def.get("params", {}),
                            priority=rule.get("priority", 1),
                            created_from_event=event.id,
                            timestamp=datetime.utcnow(),
                            severity=action_def.get("severity", 3),
                            source=rule.get("source"),
                            proposal_id=rule.get("proposal_id"),
                        )
                    )
        return self._resolve(actions)

    # -------------------------------------------------
    # CONDITION ENGINE (AND logic across list)
    # -------------------------------------------------
    def _evaluate_conditions(self, conditions, state):
        if not conditions:
            return True
        for c in conditions:
            left_value = self._resolve_path(c["left"], state)
            if left_value is None:
                return False  # missing sensor, untrusted reading, or bad path -- don't match, don't crash the cycle over it
            right_value = c["right"]
            op = c["op"]
            if not self._compare(left_value, op, right_value):
                return False
        return True

    # -------------------------------------------------
    # SIMPLE DOT-PATH RESOLVER
    # e.g. "sensors.weather_1_humidity.value" -- this IS how you write a
    # multi-sensor condition (e.g. "humidity < x AND temp > y") today:
    # each condition in the list independently dot-paths into state, so
    # a rule's `conditions` can check as many different sensors as it
    # wants, not just the one that triggered it. See
    # water_growing_zone_until_moist for the trigger-vs-condition split
    # this relies on: `triggers` picks WHICH event starts evaluation,
    # `conditions` is free to check anything else in state regardless
    # of what triggered it.
    #
    # Safe against a missing sensor_id (dict lookup) or an untrusted one
    # (confidence <= 0, checked the moment traversal lands on a
    # SensorState -- duck-typed via hasattr, not an isinstance check, so
    # it doesn't need to import SensorState here) -- either returns None,
    # which _evaluate_conditions treats as "doesn't match" rather than
    # raising and losing the whole cycle over one bad condition.
    # -------------------------------------------------
    def _resolve_path(self, path, state):
        parts = path.split(".")
        value = state
        for p in parts:
            if isinstance(value, dict):
                if p not in value:
                    return None
                value = value[p]
            else:
                if not hasattr(value, p):
                    return None
                value = getattr(value, p)
            if hasattr(value, "confidence") and hasattr(value, "value") and getattr(value, "confidence") <= 0:
                return None  # landed on an untrusted sensor reading mid-path -- stop here
        return value

    # -------------------------------------------------
    # COMPARISON ENGINE
    # -------------------------------------------------
    def _compare(self, left, op, right):
        if op == ">":
            return left > right
        if op == "<":
            return left < right
        if op == ">=":
            return left >= right
        if op == "<=":
            return left <= right
        if op == "==":
            return left == right
        if op == "!=":
            return left != right
        return False

    # -------------------------------------------------
    # DEDUP + PRIORITY RESOLUTION
    # -------------------------------------------------
    def _resolve(self, actions):
        actions.sort(key=lambda a: a.priority, reverse=True)
        seen = set()
        final = []
        for a in actions:
            key = (a.action_type, a.target)
            if key in seen:
                continue
            seen.add(key)
            final.append(a)
        return final
