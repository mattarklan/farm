"""Seed price fetcher - scrapes every link listed under seed_price in
config/fois/links.yaml. Currently empty (no scrapable white lupin seed
source found - see the comment in links.yaml). Add a link there and
this starts working with no code changes needed.
"""

from .scraping_base import ScrapingPriceFetcherBase


class SeedPriceFetcher(ScrapingPriceFetcherBase):
    domain = "seed_price"
