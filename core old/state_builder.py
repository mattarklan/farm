from core.models import SensorState
from datetime import datetime

class StateBuilder:
    def build(self, db, registry):
        rows = db.latest_readings()
        state = {
            "timestamp": datetime.utcnow(),
            "sensors": {}
        }
        for r in rows:
            state["sensors"][r["sensor_id"]] = SensorState(
                value=r["value"],
                quality=r["quality"],
                timestamp=r["timestamp"],
                source=r["source"]
            )
        return state