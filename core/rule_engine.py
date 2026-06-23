# core/rule_engine.py


class RuleEngine:

    def __init__(self, rules):
        self.rules = rules or self._default_rules()


    def _default_rules(self):

        return [
            {
                "event_id": "E_TANK_LOW",
                "action": "SUSPEND_IRRIGATION_ZONE",
                "params": {"mode": "conservative"},
                "priority": 2
            },
            {
                "event_id": "E_BATTERY_LOW",
                "action": "LOAD_SHEDDING",
                "params": {"level": "medium"},
                "priority": 5
            },
            {
                "event_id": "E_FLOW_ANOMALY",
                "action": "SHUT_OFF_VALVE",
                "params": {}
            }
        ]


    def process(self, events, state):

        actions = []

        for event in events:

            for rule in self.rules:

                if rule["event_id"] == event["event_id"]:

                    actions.append({
                        "action": rule["action"],
                        "params": rule.get("params", {}),
                        "triggered_by": event,
                        "priority": rule.get("priority", 1)
                    })

        return self._resolve_conflicts(actions)


    def _resolve_conflicts(self, actions):

        # simple priority merge (higher wins)
        actions.sort(key=lambda x: x["priority"], reverse=True)

        seen = set()
        final = []

        for a in actions:

            key = a["action"]

            if key not in seen:
                final.append(a)
                seen.add(key)

        return final