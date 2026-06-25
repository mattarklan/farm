import yaml
from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class Asset:
    id: str
    name: str
    type: str
    sensors: list
    dependencies: list
    metadata: dict


class Registry:

    def __init__(self):

        self.assets = {}

        self.sensor_index = {}

    def build_indexes(self):

        self.sensor_index.clear()

        for asset in self.assets.values():

            for sensor in asset.sensors:

                self.sensor_index[sensor] = asset

    def get_asset_from_sensor(self, sensor_name):

        return self.sensor_index.get(sensor_name)


class RegistryLoader:

    def load(self, path: str) -> Registry:

        registry = Registry()

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        for category, assets in data.items():

            for asset_id, cfg in assets.items():

                self._validate(asset_id, cfg)

                asset = Asset(
                    id=cfg["id"],
                    name=cfg.get("name", asset_id),
                    type=cfg["type"],
                    sensors=cfg.get("sensors", []),
                    dependencies=cfg.get("dependencies", []),
                    metadata=cfg.get("metadata", {})
                )

                registry.assets[asset.id] = asset
        registry.build_indexes()
        return registry

    def _validate(self, asset_id, cfg):

        required = ["id", "type"]

        for r in required:
            if r not in cfg:
                raise ValueError(f"Missing {r} in {asset_id}")