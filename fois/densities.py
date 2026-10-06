"""Loads bulk-density estimates (kg per litre) from config/fois/links.yaml,
used only to compute an approximate price-per-tonne for volume-priced
products. See that file's `densities` section for sources/caveats on
each figure - they're generic published estimates, not measurements of
the specific products fois is scraping, and moisture/compaction can
shift them meaningfully. price_per_m3 (in units.py) doesn't need any of
this and is exact - prefer it when precision matters.
"""

from pathlib import Path
from typing import Optional

import yaml

LINKS_PATH = Path(__file__).resolve().parent.parent / "config" / "fois" / "links.yaml"

_cache: Optional[dict] = None


def _load() -> dict:
    global _cache
    if _cache is None:
        with open(LINKS_PATH) as f:
            data = yaml.safe_load(f) or {}
        _cache = data.get("densities", {})
    return _cache


def density_kg_per_litre(domain: str, item: str) -> Optional[float]:
    return _load().get(domain, {}).get(item)
