from typing import List
from core.rules import Action


class Executor:

    def process(self, actions: List[Action]):

        for a in actions:

            if a.action_type == "IRRIGATE_ZONE":
                print(f"[IRRIGATION] Zone={a.zone} duration={a.params['duration_minutes']}min")

            elif a.action_type == "LOAD_SHEDDING":
                print(f"[ENERGY] Load shedding level={a.params['level']}")

            elif a.action_type == "SUSPEND_IRRIGATION_ALL":
                print("[IRRIGATION] ALL ZONES SUSPENDED")

            elif a.action_type == "ALERT":
                print(f"[ALERT] {a.params['message']}")

            else:
                print(f"[UNKNOWN ACTION] {a}")