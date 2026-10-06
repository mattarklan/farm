"""Public API schema. This is the contract core/GUI code against.

Kept intentionally narrow and semantic - callers get normalised values,
never raw scraped/API payloads.
"""

from datetime import date, datetime
from typing import Optional
from pydantic import BaseModel


class Reading(BaseModel):
    domain: str
    item: str
    region: Optional[str] = None
    value: float
    unit: str
    currency: Optional[str] = None    # null for weather
    kind: str                         # "observed" | "forecast"
    valid_for: Optional[date] = None  # for forecasts: date the value is about
    source: str
    fetched_at: datetime
    is_stale: bool
    confidence: Optional[float] = None

    # Normalised for comparing prices across suppliers who list different
    # unit sizes. price_per_m3 is exact (litres -> m3 is fixed unit math).
    # price_per_tonne is exact for products already priced by weight, but
    # an ESTIMATE for volume-priced products - see
    # price_per_tonne_is_estimate, and config/fois/links.yaml's
    # `densities` section for the assumption behind each estimate. Both
    # are null when not computable (e.g. unit isn't a recognised
    # volume/mass format, or - for tonnes - no density is configured).
    price_per_m3: Optional[float] = None
    price_per_tonne: Optional[float] = None
    price_per_tonne_is_estimate: Optional[bool] = None
