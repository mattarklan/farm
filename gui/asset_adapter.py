from .app_state import AssetState


def registry_to_assets(registry: dict):
    """
    Convert farm.config.loader.load_registry() output into AssetState
    objects used throughout the GUI.
    """

    assets = []

    for asset_id, asset in registry.items():

        metadata = asset.get("metadata", {}) or {}

        metrics = asset.get("metrics", []) or []

        sensors = {
            metric: None
            for metric in metrics
        }

        assets.append(

            AssetState(

                asset_id=asset["id"],

                name=asset.get(
                    "name",
                    asset_id,
                ),

                type=asset.get(
                    "type",
                    "unknown",
                ),

                gps=metadata.get(
                    "gps"
                ),

                sensors=sensors,

                dependencies=asset.get(
                    "dependencies",
                    [],
                ),

            )

        )

    return assets