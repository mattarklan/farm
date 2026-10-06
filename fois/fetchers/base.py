"""Fetcher ABC. Each domain (weather, veg_price, ...) implements one or more
of these - one per source. The scheduler tries sources in priority order
and falls back on failure, so a single source going down never takes out
the whole domain, let alone other domains.
"""

import logging
from abc import ABC, abstractmethod
from typing import Any

from ..models import Reading

log = logging.getLogger("fois.fetchers")


class Fetcher(ABC):
    domain: str

    @abstractmethod
    def fetch(self, source_cfg: dict, region_name: str, region_cfg: dict) -> Any:
        """Hit the external source and return raw data. Raise on failure."""
        raise NotImplementedError

    @abstractmethod
    def normalize(self, raw: Any, source_cfg: dict, region_name: str) -> list[Reading]:
        """Turn raw data into a list of normalised Reading objects."""
        raise NotImplementedError

    def run(self, source_cfg: dict, region_name: str, region_cfg: dict) -> list[Reading]:
        """Fetch + normalize, with isolation: any failure is logged and
        results in an empty list rather than propagating up and breaking
        the scheduler run for other domains/sources/regions."""
        try:
            raw = self.fetch(source_cfg, region_name, region_cfg)
            readings = self.normalize(raw, source_cfg, region_name)
            log.info(
                "fetched %d readings for domain=%s source=%s region=%s",
                len(readings), self.domain, source_cfg.get("name"), region_name,
            )
            return readings
        except Exception as exc:
            log.warning(
                "fetch failed for domain=%s source=%s region=%s: %s",
                self.domain, source_cfg.get("name"), region_name, exc,
            )
            return []
