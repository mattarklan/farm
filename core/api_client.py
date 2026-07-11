#api_client
"""
Thin HTTP client for the ingestion project's FastAPI service.

Per the handoff doc, this is the ONLY sanctioned way core/ touches
farm.db-backed data. Nothing else in core/ should import sqlite3 or
reference farm.db directly.

Confirmed live GET /state/latest response shape:

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

Parsing lives in state_builder.py.
"""
from __future__ import annotations

from datetime import datetime

import requests


class IngestionAPIError(RuntimeError):
    """Raised when a call to the ingestion API fails or returns an error status."""


class IngestionAPIClient:
    def __init__(self, base_url: str, timeout_seconds: float = 5.0):
        self.base_url = base_url.rstrip("/")
        self.timeout_seconds = timeout_seconds

    def get_latest_state(self) -> dict:
        """Raw JSON from GET /state/latest. Caller (StateBuilder) shapes it."""
        return self._get("/state/latest")

    def get_asset_context(self, asset_id: str) -> dict:
        """GET /state/context/{asset_id}."""
        return self._get(f"/state/context/{asset_id}")

    def post_event(self, asset_id: str, event_type: str, payload: dict,
                    ts: datetime | None = None) -> dict:
        """POST /events — the write-path for core to report decided events back."""
        body = {"asset_id": asset_id, "event_type": event_type, "payload": payload}
        if ts is not None:
            body["ts"] = int(ts.timestamp())
        return self._post("/events", body)

    def get_events(self, asset_id: str | None = None, since_ts: int | None = None) -> dict:
        params = {}
        if asset_id is not None:
            params["asset_id"] = asset_id
        if since_ts is not None:
            params["since_ts"] = since_ts
        return self._get("/events", params=params)

    def get_weather_offsets(self, zone_id: str | None = None, since: str | None = None) -> list:
        """
        Placeholder for the future weather-offset module (schema in
        config/weather_offset_schema.yaml). No real endpoint exists yet
        -- calling this before that module is built will raise
        IngestionAPIError like any other failed call. Wired up now so
        rules can build against a stable interface once real data
        starts flowing, instead of needing another rework later.
        """
        params = {}
        if zone_id is not None:
            params["zone_id"] = zone_id
        if since is not None:
            params["since"] = since
        return self._get("/weather/offsets", params=params)

    def _get(self, path: str, params: dict | None = None) -> dict:
        try:
            resp = requests.get(f"{self.base_url}{path}", params=params, timeout=self.timeout_seconds)
            resp.raise_for_status()
            return resp.json()
        except requests.RequestException as exc:
            raise IngestionAPIError(f"GET {path} failed: {exc}") from exc

    def _post(self, path: str, body: dict) -> dict:
        try:
            resp = requests.post(f"{self.base_url}{path}", json=body, timeout=self.timeout_seconds)
            resp.raise_for_status()
            return resp.json()
        except requests.RequestException as exc:
            raise IngestionAPIError(f"POST {path} failed: {exc}") from exc
