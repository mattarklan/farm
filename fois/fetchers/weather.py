"""Weather FORECAST fetcher, sourced from the Bureau of Meteorology.

Important: this is forecast-only. Current/actual conditions come from the
Jaycar weather station via ingestor, not from here - fois should never
duplicate what a sensor already measures. Core is expected to compare
these forecasts against ingestor's later observations to maintain its own
weather offset (e.g. "BOM says 20C, station usually reads 22C").

Note on the API: api.weather.bom.gov.au is an unofficial/reverse-engineered
endpoint (no public developer docs, no auth). It has historically been
reliable for personal/non-commercial use but BOM could change or restrict
it without notice - that's exactly the kind of thing the source-priority
fallback list in sources.yaml exists to handle. If this starts failing,
add a second source (e.g. Open-Meteo's BOM-model passthrough) rather than
patching around it here.
"""

from datetime import datetime, date, timezone
from typing import Any

import httpx

from .base import Fetcher
from ..models import Reading

API_BASE = "https://api.weather.bom.gov.au/v1"


class BomWeatherFetcher(Fetcher):
    domain = "weather"

    def fetch(self, source_cfg: dict, region_name: str, region_cfg: dict) -> Any:
        geohash = region_cfg.get("bom_geohash")
        if not geohash:
            raise ValueError(f"region {region_name!r} has no bom_geohash configured")

        url = f"{API_BASE}/locations/{geohash}/forecasts/daily"
        resp = httpx.get(url, timeout=15, headers={"User-Agent": "fois-farm-manager/1.0"})
        resp.raise_for_status()
        return resp.json()

    def normalize(self, raw: Any, source_cfg: dict, region_name: str) -> list[Reading]:
        fetched_at = datetime.now(timezone.utc)
        readings: list[Reading] = []

        for day in raw.get("data", []):
            valid_for = _parse_date(day.get("date"))
            if valid_for is None:
                continue

            common = dict(
                region=region_name,
                kind="forecast",
                source=source_cfg["name"],
                fetched_at=fetched_at,
                valid_for=valid_for,
                raw=day,
            )

            if day.get("temp_max") is not None:
                readings.append(Reading(domain=self.domain, item="temp_max",
                                         value=float(day["temp_max"]), unit="celsius", **common))
            if day.get("temp_min") is not None:
                readings.append(Reading(domain=self.domain, item="temp_min",
                                         value=float(day["temp_min"]), unit="celsius", **common))

            rain = day.get("rain") or {}
            if rain.get("chance") is not None:
                readings.append(Reading(domain=self.domain, item="rain_probability",
                                         value=float(rain["chance"]), unit="percent", **common))
            amount = rain.get("amount") or {}
            if amount.get("max") is not None:
                readings.append(Reading(domain=self.domain, item="rain_amount_max",
                                         value=float(amount["max"]), unit="mm", **common))

            uv = day.get("uv") or {}
            if uv.get("max_index") is not None:
                readings.append(Reading(domain=self.domain, item="uv_index_max",
                                         value=float(uv["max_index"]), unit="index", **common))

        return readings


def _parse_date(raw_date: str | None) -> date | None:
    if not raw_date:
        return None
    try:
        # BOM returns e.g. "2026-07-21T00:00:00Z" or "2026-07-21"
        return datetime.fromisoformat(raw_date.replace("Z", "+00:00")).date()
    except ValueError:
        return None
