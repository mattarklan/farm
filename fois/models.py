"""Internal data model used between fetchers and the store.

This is deliberately separate from fois/schemas.py (the FastAPI/Pydantic
response models). Keeping them separate means the on-disk/internal shape
can evolve without touching the public API contract, and vice versa.
"""

from dataclasses import dataclass, field
from datetime import date, datetime
from typing import Optional, Any


@dataclass
class Reading:
    domain: str                      # weather, veg_price, fertilizer_price, mulch_price, seed_price
    item: str                        # 'rain_probability', 'urea', 'tomato', ...
    value: float
    unit: str
    kind: str                        # "observed" | "forecast"
    source: str                      # which source produced this (e.g. "bom")
    fetched_at: datetime             # when fois fetched it
    region: Optional[str] = None
    currency: Optional[str] = None   # None for non-price domains (e.g. weather)
    valid_for: Optional[date] = None # for forecasts: the date the value is about
    confidence: Optional[float] = None
    raw: Optional[Any] = None        # original payload, stored as JSON for audit/debug
