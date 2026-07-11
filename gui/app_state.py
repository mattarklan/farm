from dataclasses import dataclass, field
from typing import Optional, Callable, Any, Dict, List


@dataclass
class AssetState:

    asset_id: str
    name: str
    type: str

    gps: Optional[tuple] = None

    sensors: Dict[str, Any] = field(
        default_factory=dict
    )

    dependencies: List[str] = field(
        default_factory=list
    )

    last_update: Optional[int] = None



class AppState:

    def __init__(self):

        self._selected_asset = None


        self.system = {
            "battery_soc": 0,
            "battery_voltage": 0,
            "tank_1_level": 0,
            "temperature": 0,
            "rain_1d": 0,
            "rain_7d": 0,
            "rain_30d": 0,
            "wind": 0,
        }


        self._asset_registry = {}


        self._on_selected_change = []

        self._on_system_change = []

        self._on_asset_change = []



    # --------------------------------------------------------
    # Registry
    # --------------------------------------------------------

    def register_asset(
        self,
        asset
    ):

        self._asset_registry[
            asset.asset_id
        ] = asset



    def get_all_assets(
        self
    ):

        return list(
            self._asset_registry.values()
        )



    def get_asset(
        self,
        asset_id
    ):

        return self._asset_registry.get(
            asset_id
        )



    # --------------------------------------------------------
    # Sensor updates
    # --------------------------------------------------------

    def update_asset_sensors(
        self,
        asset_id,
        sensors,
        ts=None
    ):

        asset = self._asset_registry.get(
            asset_id
        )


        if not asset:
            return


        asset.sensors.update(
            sensors
        )


        asset.last_update = ts


        for cb in self._on_asset_change:

            cb(
                asset
            )



    # --------------------------------------------------------
    # Selection
    # --------------------------------------------------------

    @property
    def selected_asset(
        self
    ):

        return self._selected_asset



    def set_selected_asset(
        self,
        asset
    ):

        self._selected_asset = asset


        for cb in self._on_selected_change:

            cb(
                asset
            )



    def clear_selection(
        self
    ):

        self.set_selected_asset(
            None
        )



    def on_selected_change(
        self,
        callback
    ):

        self._on_selected_change.append(
            callback
        )



    # --------------------------------------------------------
    # Asset callbacks
    # --------------------------------------------------------

    def on_asset_change(
        self,
        callback
    ):

        self._on_asset_change.append(
            callback
        )



    # --------------------------------------------------------
    # System telemetry
    # --------------------------------------------------------

    def update_system(
        self,
        **kwargs
    ):

        self.system.update(
            kwargs
        )


        for cb in self._on_system_change:

            cb(
                self.system
            )



    def on_system_change(
        self,
        callback
    ):

        self._on_system_change.append(
            callback
        )