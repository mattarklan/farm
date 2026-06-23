# core/test_rules.py

from core.rule_engine import RuleEngine


events = [
    {"event_id": "E_TANK_LOW", "sensor": "tank_1", "value": 10},
    {"event_id": "E_FLOW_ANOMALY", "sensor": "bore_1", "value": -1},
]

engine = RuleEngine(None)

actions = engine.process(events, state=None)

print(actions)