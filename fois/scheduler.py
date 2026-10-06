"""Runs each domain's fetchers, in priority order, falling back to the
next source on failure. Meant to be invoked periodically (e.g. daily cron,
or a simple loop in run_fois.py) - fois itself doesn't need to be "always
running" for scheduling purposes, since core/GUI read from the store via
the API regardless of when the last fetch happened (is_stale tells them
whether it's current enough to trust).
"""

import logging

from .config import Config
from .store import Store

log = logging.getLogger("fois.scheduler")


def run_domain(config: Config, store: Store, domain: str) -> int:
    """Run a domain's sources according to its configured mode.

    'fallback' (default, e.g. weather): sources are redundant alternatives
    for the same fact - try in priority order, stop per-region at first
    success.

    'all' (e.g. multi-supplier prices): sources are genuinely different
    values, not alternatives - every source is tried every time, and every
    successful reading is kept, regardless of what other sources returned.
    """
    mode = config.mode_for(domain)
    total_inserted = 0

    if mode == "all":
        for source_cfg in config.sources_for(domain):
            fetcher_cls = config.load_fetcher_class(source_cfg)
            fetcher = fetcher_cls()
            for region_name, region_cfg in config.regions.items():
                readings = fetcher.run(source_cfg, region_name, region_cfg)
                if readings:
                    total_inserted += store.insert_readings(readings)
        return total_inserted

    # mode == "fallback"
    succeeded_regions: set[str] = set()
    for source_cfg in config.sources_for(domain):
        fetcher_cls = config.load_fetcher_class(source_cfg)
        fetcher = fetcher_cls()

        for region_name, region_cfg in config.regions.items():
            if region_name in succeeded_regions:
                continue

            readings = fetcher.run(source_cfg, region_name, region_cfg)
            if readings:
                total_inserted += store.insert_readings(readings)
                succeeded_regions.add(region_name)

    return total_inserted


def run_all(config: Config | None = None, store: Store | None = None) -> dict[str, int]:
    """Run every configured domain once. Returns a dict of domain -> rows inserted."""
    config = config or Config()
    store = store or Store()

    results = {}
    for domain in config.domain_names():
        log.info("running domain=%s", domain)
        results[domain] = run_domain(config, store, domain)
    return results


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    outcome = run_all()
    for domain, count in outcome.items():
        print(f"{domain}: {count} readings inserted")
