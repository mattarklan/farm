from core.models import SensorState
from datetime import datetime


class StateBuilder:

    def build(self, registry):

        state = {
            "timestamp": datetime.utcnow(),
            "sensors": {}
        }

        for asset in registry.assets.values():

            for sensor in asset.sensors:

                state["sensors"][sensor] = SensorState(
                    value=0.0,
                    quality=1.0,
                    timestamp=datetime.utcnow(),
                    source="init"
                )

        return state