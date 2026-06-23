from dataclasses import dataclass
from typing import List, Any
import operator

from core.events import Event
from core.state import FarmState


@dataclass(slots=True)
class Action:
    action_type: str
    zone: str | None
    params: dict


class RuleEngine:

    def __init__(self, rules_config: dict):
        self.rules = rules_config["rules"]

    def process(self, events: List[Event], state: FarmState) -> List[Action]:

        actions: List[Action] = []

        for event in events:

            for rule in self.rules:

                if event.event_id not in rule["triggers"]:
                    continue

                if not self._check_conditions(rule.get("conditions", []), state):
                    continue

                for action in rule["actions"]:
                    actions.append(
                        Action(
                            action_type=action["type"],
                            zone=action.get("zone"),
                            params=action.get("params", {})
                        )
                    )

        return actions

    def _check_conditions(self, conditions: list, state: FarmState) -> bool:

        for cond in conditions:

            left = self._resolve(cond["left"], state)
            op = cond["op"]
            right = cond["right"]

            if not self._compare(left, op, right):
                return False

        return True

    def _resolve(self, path: str, state: FarmState):

        # very small path resolver: "energy.battery_soc_percent"
        parts = path.split(".")

        obj = state
        for p in parts:
            obj = getattr(obj, p)

        return obj

    def _compare(self, left, op, right):

        ops = {
            "<": operator.lt,
            "<=": operator.le,
            ">": operator.gt,
            ">=": operator.ge,
            "==": operator.eq,
            "!=": operator.ne,
        }

        return ops[op](left, right)