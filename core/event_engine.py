# core/event_engine.py

from datetime import datetime


class EventEngine:

    def __init__(self, rules):
        self.rules = rules


    def process(self, state):

        events = []

        sensors = state["sensors"]

        for sensor_id, sensor in sensors.items():

            value = sensor["value"]

            # --- BATTERY RULE EXAMPLE ---
            if "battery" in sensor_id and value < 20:

                events.append({
                    "event_id": "E_BATTERY_LOW",
                    "sensor": sensor_id,
                    "value": value,
                    "severity": 4,
                    "confidence": 0.95,
                    "timestamp": datetime.utcnow().isoformat()
                })

            # --- WATER LEVEL RULE ---
            if "tank" in sensor_id and "level" in sensor_id and value < 25:

                events.append({
                    "event_id": "E_TANK_LOW",
                    "sensor": sensor_id,
                    "value": value,
                    "severity": 3,
                    "confidence": 0.9,
                    "timestamp": datetime.utcnow().isoformat()
                })

            # --- FLOW ANOMALY (simple heuristic) ---
            if "flow" in sensor_id and value < 0:

                events.append({
                    "event_id": "E_FLOW_ANOMALY",
                    "sensor": sensor_id,
                    "value": value,
                    "severity": 2,
                    "confidence": 0.8,
                    "timestamp": datetime.utcnow().isoformat()
                })

        return events