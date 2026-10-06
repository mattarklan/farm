"""Fertilizer price fetcher - scrapes every link listed under
fertilizer_price in config/fois/links.yaml. See scraping_base.py for
the shared logic; this file just points it at the right domain.
"""

from .scraping_base import ScrapingPriceFetcherBase


class FertilizerPriceFetcher(ScrapingPriceFetcherBase):
    domain = "fertilizer_price"
