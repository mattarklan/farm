import yaml
import os


class Asset:
    def __init__(self, id, name, type, gps=None, sensors=None):
        self.id = id
        self.name = name
        self.type = type
        self.gps = gps
        self.sensors = sensors or {}


YAML_FILES = [
    "../config/registry/water.yaml",
    "../config/registry/vehicles.yaml",
    "../config/registry/power.yaml",
    "../config/registry/crops.yaml",
]


def load_assets():
    assets = []
    for path in YAML_FILES:
        if not os.path.exists(path):
            continue
        with open(path, "r") as f:
            data = yaml.safe_load(f)
        if not data:
            continue
        for category, items in data.items():
            if not isinstance(items, dict):
                continue
            for key, item in items.items():
                if not isinstance(item, dict):
                    continue
                metadata = item.get("metadata", {}) or {}
                sensors = {s: None for s in item.get("sensors", [])}
                assets.append(
                    Asset(
                        id=item.get("id", key),
                        name=item.get("name", key),
                        type=item.get("type", "unknown"),
                        gps=metadata.get("gps"),
                        sensors=sensors,
                    )
                )
    return assets