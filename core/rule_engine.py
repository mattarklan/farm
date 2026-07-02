#rule_engine.py
from datetime import datetime
from core.models import Action
import uuid


class RuleEngine:
    def __init__(self, rule_config: dict):
        self.rules = rule_config.get("rules", [])

    def process(self, events, state):
        actions = []
        for event in events:
            for rule in self.rules:
                # goal-kind rules (start/stop/until) are GoalEngine's job
                if rule.get("kind") == "goal":
                    continue
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
                            target=action_def.get("zone"),
                            parameters=action_def.get("params", {}),
                            priority=rule.get("priority", 1),
                            created_from_event=event.id,
                            timestamp=datetime.utcnow()
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
            right_value = c["right"]
            op = c["op"]
            if not self._compare(left_value, op, right_value):
                return False
        return True

    # -------------------------------------------------
    # SIMPLE DOT-PATH RESOLVER
    # e.g. "energy.battery_soc_percent"
    # -------------------------------------------------
    def _resolve_path(self, path, state):
        parts = path.split(".")
        value = state
        for p in parts:
            value = value[p] if isinstance(value, dict) else getattr(value, p)
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
