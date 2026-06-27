import yaml
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


# ----------------------------------------------------------------------
# Domain model (UI-agnostic)
# ----------------------------------------------------------------------

@dataclass
class Asset:
    id: str
    name: str
    type: str
    gps: Optional[tuple] = None
    metadata: Optional[dict] = None
    sensors: Optional[list] = None
    dependencies: Optional[list] = None


# ----------------------------------------------------------------------
# Registry loader
# ----------------------------------------------------------------------

class AssetRegistry:
    """
    Loads raw YAML into clean Asset objects.

    This is the ONLY place that knows YAML structure exists.
    """

    def __init__(self, yaml_path: str):
        self.yaml_path = yaml_path
        self._assets: List[Asset] = []

    # ------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------

    def load(self):
        with open(self.yaml_path, "r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)

        self._assets = self._parse(raw)
        return self._assets

    def assets(self) -> List[Asset]:
        return self._assets

    def get(self, asset_id: str) -> Optional[Asset]:
        for a in self._assets:
            if a.id == asset_id:
                return a
        return None

    # ------------------------------------------------------------
    # Internal parsing
    # ------------------------------------------------------------

    def _parse(self, raw: Dict[str, Any]) -> List[Asset]:
        assets: List[Asset] = []

        for group_name, group_data in raw.items():
            # group_name = "water", "power", etc.

            for _, item in group_data.items():
                meta = item.get("metadata", {}) or {}

                gps = meta.get("gps")
                if gps:
                    gps = tuple(gps)

                asset = Asset(
                    id=item.get("id"),
                    name=item.get("name"),
                    type=item.get("type"),
                    gps=gps,
                    metadata=meta,
                    sensors=item.get("sensors"),
                    dependencies=item.get("dependencies"),
                )

                assets.append(asset)

        return assets