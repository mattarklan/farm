import os
import yaml

CONFIG_DIR = os.path.join(os.path.dirname(__file__), "registry")

REGISTRY_FILES = ["water.yaml", "vehicles.yaml", "crops.yaml", "power.yaml"]

_cache = None


def _load_yaml(filename):
    path = os.path.join(CONFIG_DIR, filename)
    if not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def _flatten_assets(raw):
    """
    Each registry file has one top-level grouping key (e.g. "water", "power",
    "growing_zones") whose value is a dict of asset_id -> asset_def.
    Flatten that away since the grouping key itself isn't needed downstream.
    """
    assets = {}
    for _group_name, group in raw.items():
        if not isinstance(group, dict):
            continue
        for asset_id, asset_def in group.items():
            if asset_id in assets:
                raise ValueError(f"Duplicate asset id across registry files: {asset_id!r}")
            assets[asset_id] = asset_def
    return assets


def _split_sensor(sensor_name, asset_id):
    """Sensor names are stored as '{asset_id}_{metric}' by convention."""
    prefix = f"{asset_id}_"
    if sensor_name.startswith(prefix):
        return sensor_name[len(prefix):]
    return sensor_name  # didn't follow convention — keep as-is rather than fail


def _build_registry():
    assets = {}
    for filename in REGISTRY_FILES:
        raw = _load_yaml(filename)
        for asset_id, asset_def in _flatten_assets(raw).items():
            sensors_raw = asset_def.get("sensors", []) or []
            metrics = [_split_sensor(s, asset_id) for s in sensors_raw]
            assets[asset_id] = {
                "id": asset_id,
                "name": asset_def.get("name", asset_id),
                "type": asset_def.get("type"),
                "metrics": metrics,
                "dependencies": asset_def.get("dependencies", []) or [],
                "metadata": asset_def.get("metadata", {}) or {},
            }
    return assets


def load_registry(force_reload=False):
    global _cache
    if _cache is None or force_reload:
        _cache = _build_registry()
    return _cache


def get_asset(asset_id):
    return load_registry().get(asset_id)


def get_all_assets():
    return load_registry()


def get_metrics_for_asset(asset_id):
    asset = get_asset(asset_id)
    return asset["metrics"] if asset else []


def get_dependencies(asset_id):
    """Assets this asset is supplied/fed by (e.g. tank <- bore)."""
    asset = get_asset(asset_id)
    return asset["dependencies"] if asset else []


def get_assets_by_type(asset_type):
    return {aid: a for aid, a in load_registry().items() if a["type"] == asset_type}