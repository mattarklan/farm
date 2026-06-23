# core/state_builder.py

class StateBuilder:

    def build(self, registry):

        state = {
            "sensors": {},
            "assets": {},
            "derived": {},
        }

        for asset_id, asset in registry.assets.items():

            state["assets"][asset_id] = {
                "type": asset.type,
                "status": "initializing"
            }

            for sensor in asset.sensors:

                state["sensors"][sensor] = {
                    "value": 0.0,
                    "quality": 1.0,
                    "last_update": None
                }

        return state