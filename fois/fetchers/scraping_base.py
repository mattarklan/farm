"""Shared base for price fetchers that scrape links listed in
config/fois/links.yaml. Domain subclasses (see fertilizer_prices.py,
mulch_prices.py, seed_prices.py) just set `domain` - all the loading,
fetching, and parsing logic lives here so it's identical across domains.

Isolation happens per-link, not just per-fetcher: one bad URL in
links.yaml logs a warning and is skipped, it doesn't stop the other
links (including other suppliers for the same item) from being tried.
"""

import logging
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import httpx
import yaml

from .base import Fetcher
from ..models import Reading
from ..scraping import extract_price

log = logging.getLogger("fois.fetchers.scraping")

LINKS_PATH = Path(__file__).resolve().parent.parent.parent / "config" / "fois" / "links.yaml"


class ScrapingPriceFetcherBase(Fetcher):
    """Not used directly - subclass and set `domain`."""

    def fetch(self, source_cfg: dict, region_name: str, region_cfg: dict) -> Any:
        if not LINKS_PATH.exists():
            raise FileNotFoundError(f"links file not found: {LINKS_PATH}")
        with open(LINKS_PATH) as f:
            all_links = yaml.safe_load(f) or {}

        items = all_links.get(self.domain) or {}
        results = []

        for item, entries in items.items():
            for entry in (entries or []):
                name = entry.get("name", "unknown")
                url = entry.get("url")
                try:
                    resp = httpx.get(
                        url, timeout=15, follow_redirects=True,
                        headers={"User-Agent": "fois-farm-manager/1.0"},
                    )
                    resp.raise_for_status()
                    parser = entry.get("parser", "auto")
                    match = extract_price(resp.text, parser=parser)
                    if match is None:
                        log.warning(
                            "no price pattern matched for %s/%s (%s) at %s",
                            self.domain, item, name, url,
                        )
                        continue
                    results.append({
                        "item": item,
                        "name": name,
                        "url": url,
                        "unit": entry.get("unit", ""),
                        "value": match.value,
                        "confidence": match.confidence,
                        "strategy": match.strategy,
                    })
                except Exception as exc:
                    log.warning(
                        "fetch/parse failed for %s/%s (%s) at %s: %s",
                        self.domain, item, name, url, exc,
                    )
                    continue

        return results

    def normalize(self, raw: Any, source_cfg: dict, region_name: str) -> list[Reading]:
        fetched_at = datetime.now(timezone.utc)
        return [
            Reading(
                domain=self.domain,
                item=r["item"],
                region=region_name,
                value=r["value"],
                unit=r["unit"],
                currency="AUD",
                kind="observed",
                source=r["name"],
                fetched_at=fetched_at,
                confidence=r["confidence"],
                raw={"url": r["url"], "strategy": r["strategy"]},
            )
            for r in raw
        ]
