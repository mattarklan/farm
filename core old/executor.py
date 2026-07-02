#executor.py
from datetime import datetime
from core.models import ActionResult


class Executor:

    def __init__(self):

        # action dispatch table
        self.handlers = {
            "LOAD_SHEDDING": self._load_shedding,
            "SUSPEND_IRRIGATION_ZONE": self._suspend_irrigation,
            "SHUT_OFF_VALVE": self._shut_off_valve,
        }


    def execute(self, actions):

        results = []

        for action in actions:

            handler = self.handlers.get(action.action_type)

            if handler is None:

                results.append(
                    ActionResult(
                        action_id=action.id,
                        success=False,
                        message=f"No handler for {action.action_type}",
                        timestamp=datetime.utcnow()
                    )
                )

                continue

            result = handler(action)
            results.append(result)

        return results


    # -----------------------------
    # HANDLERS (hardware boundary)
    # -----------------------------

    def _load_shedding(self, action):

        print(f"[EXECUTOR] LOAD SHEDDING → {action.parameters}")

        # placeholder for inverter / relay control

        return ActionResult(
            action_id=action.id,
            success=True,
            message="Load shedding executed (simulated)",
            timestamp=datetime.utcnow()
        )


    def _suspend_irrigation(self, action):

        print(f"[EXECUTOR] IRRIGATION SUSPENDED → {action.target}")

        # placeholder for valve control system

        return ActionResult(
            action_id=action.id,
            success=True,
            message="Irrigation suspended (simulated)",
            timestamp=datetime.utcnow()
        )


    def _shut_off_valve(self, action):

        print(f"[EXECUTOR] VALVE CLOSED → {action.target}")

        # placeholder for relay / solenoid control

        return ActionResult(
            action_id=action.id,
            success=True,
            message="Valve shut off (simulated)",
            timestamp=datetime.utcnow()
        )