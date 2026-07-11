"""
Adapts config/loader.py's plain-dict registry into the lightweight
object interface event_engine.py / goal_engine.py / confidence_monitor.py
already expect: asset.id / asset.type / asset.metadata, and a
get_asset_from_sensor(sensor_id) reverse lookup.

Built after seeing the real config/loader.py: it returns plain dicts
(get_asset(id) -> dict, get_all_assets() -> {id: dict}) with no
sensor->asset reverse lookup at all. Rather than rewrite three existing
engine files to use dict access and build their own reverse index each,
this wraps the real loader's output once, so those files keep working
against the interface they were already written for.
"""
from __future__ import annotations

from config import loader as real_loader


class Asset:
    """Thin attribute-access wrapper over the real loader's plain dict shape."""
    def __init__(self, data: dict):
        self.id = data["id"]
        self.name = data.get("name", self.id)
        self.type = data.get("type")
        self.metrics = data.get("metrics", [])
        self.dependencies = data.get("dependencies", [])
        self.downstream = data.get("downstream", [])
        self.metadata = data.get("metadata", {})


class RegistryAdapter:
    def __init__(self):
        self.assets: dict[str, Asset] = {}
        self._sensor_index: dict[str, Asset] = {}
        self._build()

    def _build(self):
        raw_assets = real_loader.get_all_assets()  # {asset_id: {id, name, type, metrics, dependencies, metadata}}
        self.assets = {aid: Asset(data) for aid, data in raw_assets.items()}
        self._sensor_index.clear()
        for asset in self.assets.values():
            for metric in asset.metrics:
                # real loader strips the "{asset_id}_" prefix off each sensor
                # name into `metrics` -- rebuild the full sensor_id here,
                # matching the convention state_builder.py also uses
                self._sensor_index[f"{asset.id}_{metric}"] = asset

    def get_asset_from_sensor(self, sensor_id: str):
        return self._sensor_index.get(sensor_id)

    def get_asset(self, asset_id: str):
        return self.assets.get(asset_id)

    def reload(self):
        """The real loader caches internally. Call this if the registry
        YAMLs change at runtime and this adapter's index needs rebuilding
        to match -- not currently wired into anything automatically."""
        real_loader.load_registry(force_reload=True)
        self._build()


def load_registry() -> RegistryAdapter:
    return RegistryAdapter()
