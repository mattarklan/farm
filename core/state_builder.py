"""
Builds the in-memory `state` dict the pipeline runs on, from the live
ingestion API.

Real GET /state/latest response shape (confirmed against live data):

    {
      "ts": 1782996741,
      "assets": [
        {
          "asset_id": "tank_1",
          "metrics": {"level": 48.8, "flow_in": 2.68, "flow_out": 3.0},
          "confidence": {"level": 0.86, "flow_in": 0.91, "flow_out": 0.91},
          "ts": 1782996741
        },
        ...
      ]
    }

sensor_id is built as f"{asset_id}_{metric}", matching the registry's
{asset_id}_{metric} naming convention.

KNOWN DATA ISSUE (not fixed here, just passed through): header_tank_1
has been observed reporting both correctly-named metrics ("level",
"flow_in", "flow_out") AND mis-prefixed ones ("header_1_level" etc.) in
the same payload - this is the sensor-naming fallback bug flagged in the
ingestion handoff doc. Both come through as distinct sensor_ids
(header_tank_1_level and header_tank_1_header_1_level) rather than being
silently merged or deduplicated, since guessing which one is "right"
here could hide a real upstream data problem. Also observed:
header_tank_1's per-asset `ts` can lag well behind the other assets' -
no freshness/staleness check is applied, a stale reading is treated the
same as a fresh one.
"""
from __future__ import annotations

from datetime import datetime, timezone

from core.api_client import IngestionAPIClient
from core.models import SensorState


class StateBuilder:
    def __init__(self, api_client: IngestionAPIClient):
        self.api_client = api_client

    def build(self) -> dict:
        raw = self.api_client.get_latest_state()

        sensors: dict[str, SensorState] = {}
        for asset in raw.get("assets", []):
            asset_id = asset["asset_id"]
            metrics = asset.get("metrics", {})
            confidence = asset.get("confidence", {})
            asset_ts = asset.get("ts")

            for metric, value in metrics.items():
                sensor_id = f"{asset_id}_{metric}"
                sensors[sensor_id] = SensorState(
                    value=value,
                    quality=confidence.get(metric, 1.0),
                    timestamp=self._parse_ts(asset_ts) or datetime.now(timezone.utc),
                    source="api",
                )

        return {
            "timestamp": self._parse_ts(raw.get("ts")) or datetime.now(timezone.utc),
            "sensors": sensors,
        }

    @staticmethod
    def _parse_ts(ts) -> datetime | None:
        if ts is None:
            return None
        if isinstance(ts, (int, float)):
            return datetime.fromtimestamp(ts, tz=timezone.utc)
        if isinstance(ts, str):
            return datetime.fromisoformat(ts)
        return None
