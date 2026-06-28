from dataclasses import dataclass, field
from typing import Optional, Callable, Any, Dict


@dataclass
class AssetState:
    asset_id: str
    name: str
    type: str
    gps: Optional[tuple] = None
    sensors: Dict[str, Any] = field(default_factory=dict)


class AppState:
    def __init__(self):
        self._selected_asset: Optional[AssetState] = None
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
        self._on_selected_change: list[Callable] = []
        self._on_system_change: list[Callable] = []
        self._on_refresh: list[Callable] = []

        # asset_id → AssetState, populated by scene at build time
        self._asset_registry: Dict[str, AssetState] = {}

    # ── asset registry ───────────────────────────────────────────

    def register_asset(self, asset_state: AssetState):
        self._asset_registry[asset_state.asset_id] = asset_state

    def update_asset_sensors(self, asset_id: str, sensors: Dict[str, Any]):
        if asset_id in self._asset_registry:
            self._asset_registry[asset_id].sensors.update(sensors)

    def update_asset_gps(self, asset_id: str, gps: tuple):
        if asset_id in self._asset_registry:
            self._asset_registry[asset_id].gps = gps

    # ── selected asset ───────────────────────────────────────────

    @property
    def selected_asset(self) -> Optional[AssetState]:
        return self._selected_asset

    def set_selected_asset(self, asset: Optional[AssetState]):
        self._selected_asset = asset
        for cb in self._on_selected_change:
            cb(asset)

    def on_selected_change(self, callback: Callable):
        self._on_selected_change.append(callback)

    def clear_selection(self):
        self.set_selected_asset(None)

    # ── system telemetry ─────────────────────────────────────────

    def update_system(self, **kwargs):
        self.system.update(kwargs)
        for cb in self._on_system_change:
            cb(self.system)

    def on_system_change(self, callback: Callable):
        self._on_system_change.append(callback)

    # ── refresh ──────────────────────────────────────────────────

    def on_refresh(self, callback: Callable):
        self._on_refresh.append(callback)

    def trigger_refresh(self):
        for cb in self._on_refresh:
            cb()