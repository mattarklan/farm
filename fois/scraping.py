"""Price extraction from a fetched HTML page.

Several named strategies, each trying a subset of price patterns in
order of confidence, first match wins within that strategy. Which one
runs for a given link is picked by that link's `parser` field in
config/fois/links.yaml - "auto" (default) tries everything; the others
are narrower, for when "auto"'s low-confidence last-resort match picks
up the wrong number and you want to pin a link to something stricter.

This is a best-effort helper, not a guarantee - some sites render price
via JavaScript after load and won't have it in the raw HTML at all, no
regex fixes that. If a link isn't producing a reading, check the fetch
logs.

Note: written against page structure seen via web search and a
markdown-rendering fetch tool, not verified against raw HTML bytes from
the actual sites. Treat the first real run as a test, not a given.
"""

import logging
import re
from dataclasses import dataclass
from typing import Callable, Optional

log = logging.getLogger("fois.scraping")


@dataclass
class PriceMatch:
    value: float
    confidence: float
    strategy: str


# name -> (compiled pattern, confidence). Ordered high to low confidence
# within ALL_PATTERNS; individual parsers below select a subset.
_JSON_LD = ("json_ld_price", re.compile(r'"price"\s*:\s*"?(\d+(?:\.\d{1,2})?)"?', re.IGNORECASE), 0.9)
_META_ITEMPROP = ("meta_itemprop_price", re.compile(r'itemprop=["\']price["\'][^>]*content=["\'](\d+(?:\.\d{1,2})?)["\']', re.IGNORECASE), 0.9)
_PRICE_CLASS = ("price_class_dollar", re.compile(r'class="[^"]*\bprice\b[^"]*"[^$]{0,200}?\$\s?(\d+(?:,\d{3})*(?:\.\d{1,2})?)', re.IGNORECASE | re.DOTALL), 0.8)
_DATA_PRICE = ("data_price_attr", re.compile(r'data-price=["\'](\d+(?:\.\d{1,2})?)["\']', re.IGNORECASE), 0.75)
_FIRST_DOLLAR = ("first_dollar_amount", re.compile(r'\$\s?(\d+(?:,\d{3})*\.\d{2})'), 0.4)

ALL_PATTERNS = [_JSON_LD, _META_ITEMPROP, _PRICE_CLASS, _DATA_PRICE, _FIRST_DOLLAR]


def _run(html: str, patterns: list) -> Optional[PriceMatch]:
    for strategy_name, pattern, confidence in patterns:
        match = pattern.search(html)
        if match:
            raw_value = match.group(1).replace(",", "")
            try:
                value = float(raw_value)
            except ValueError:
                continue
            log.info("price extracted via %s: %s", strategy_name, value)
            return PriceMatch(value=value, confidence=confidence, strategy=strategy_name)
    return None


# The actual parser registry. `links.yaml`'s `parser` field looks up a
# name here. Add a new one (e.g. a real per-site parser once you've
# inspected a site's actual HTML) and point a link at it by name - no
# other wiring needed.
PARSERS: dict[str, Callable[[str], Optional[PriceMatch]]] = {
    "auto": lambda html: _run(html, ALL_PATTERNS),
    # Structured-data only - skips the risky low-confidence last-resort
    # scan. Use this if "auto" keeps grabbing the wrong $ amount on a
    # page that does have JSON-LD/meta price data.
    "structured_only": lambda html: _run(html, [_JSON_LD, _META_ITEMPROP, _DATA_PRICE]),
    "json_ld": lambda html: _run(html, [_JSON_LD]),
    "meta_price": lambda html: _run(html, [_META_ITEMPROP]),
    "woocommerce": lambda html: _run(html, [_PRICE_CLASS]),
}


def extract_price(html: str, parser: str = "auto") -> Optional[PriceMatch]:
    fn = PARSERS.get(parser)
    if fn is None:
        log.warning("unknown parser %r in links.yaml, falling back to 'auto'", parser)
        fn = PARSERS["auto"]
    return fn(html)
