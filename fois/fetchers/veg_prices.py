"""Vegetable price fetcher - STUB.

TODO: wire up a real source (e.g. a regional wholesale market report,
a farmers' market API, or a scraped price list). See fertilizer_prices.py
for the same pattern once one domain is done - they're deliberately
near-identical so it's easy to replicate.

Until wired up, this returns an empty list. The scheduler treats an empty
list as "no data available from this source" and simply logs a warning
(via Fetcher.run) rather than failing - core will see is_stale=true or a
missing item for these until a real source is added.
"""

from typing import Any

from .base import Fetcher
from ..models import Reading


class VegPriceFetcher(Fetcher):
    domain = "veg_price"

    def fetch(self, source_cfg: dict, region_name: str, region_cfg: dict) -> Any:
        # TODO: replace with a real HTTP call once a source is chosen.
        return None

    def normalize(self, raw: Any, source_cfg: dict, region_name: str) -> list[Reading]:
        if raw is None:
            return []
        # TODO: map source's response shape to Reading objects, e.g.:
        # Reading(domain=self.domain, item="tomato", region=region_name,
        #         value=4.50, unit="kg", currency="AUD", kind="observed",
        #         source=source_cfg["name"], fetched_at=..., raw=raw)
        return []
