from dataclasses import dataclass
from typing import List

from core.rules import Action
from core.state import FarmState


@dataclass(slots=True)
class ConstraintViolation:
    constraint_id: str
    action: Action
    reason: str


class ConstraintEngine:
    """
    Global safety filter.
    Takes proposed actions and removes or blocks unsafe ones.
    """

    def process(
        self,
        actions: List[Action],
        state: FarmState
    ) -> List[Action]:

        safe_actions: List[Action] = []

        for action in actions:

            # -------------------------
            # RULE 1: LOW BATTERY BLOCKS HIGH LOAD ACTIONS
            # -------------------------
            if state.energy.battery_soc_percent < 20:

                if action.action_type in (
                    "IRRIGATE_ZONE",
                    "SUSPEND_IRRIGATION_ALL"
                ):
                    # allow suspension but block irrigation
                    if action.action_type == "IRRIGATE_ZONE":
                        continue

            # -------------------------
            # RULE 2: CRITICAL TANK PROTECTION
            # -------------------------
            if state.water.tank_level_percent < 15:

                if action.action_type == "IRRIGATE_ZONE":
                    continue

            # -------------------------
            # RULE 3: DEFAULT PASS-THROUGH
            # -------------------------
            safe_actions.append(action)

        return safe_actions