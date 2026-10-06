"""fois API - narrow, semantic endpoints. Callers (core, GUI) get
normalised Reading objects with an is_stale flag computed at read time;
they never see raw scraped/API payloads.

Run with: uvicorn fois.api:app --reload
"""

import sqlite3
from datetime import datetime, timezone
from typing import Optional

from fastapi import FastAPI, HTTPException, Query

from .config import Config
from .store import Store
from .schemas import Reading as ReadingOut
from . import units, densities

app = FastAPI(title="fois", description="Farm information & pricing service")

_config = Config()
_store = Store()


def _row_to_reading(row: sqlite3.Row) -> ReadingOut:
    fetched_at = datetime.fromisoformat(row["fetched_at"])
    if fetched_at.tzinfo is None:
        fetched_at = fetched_at.replace(tzinfo=timezone.utc)

    staleness = _config.staleness_for(row["domain"])
    age = datetime.now(timezone.utc) - fetched_at
    is_stale = age > staleness

    # Normalised comparison fields - only meaningful for price domains,
    # not weather (row["unit"] there is "celsius"/"percent"/etc, which
    # units.parse_unit correctly won't recognise, so these stay null).
    per_m3 = units.price_per_m3(row["value"], row["unit"])

    density = densities.density_kg_per_litre(row["domain"], row["item"])
    per_tonne = units.price_per_tonne(row["value"], row["unit"], density_kg_per_litre=density)
    # Exact when the unit was already mass-based (kg/tonne); an estimate
    # whenever a density had to be used to get there from a volume unit.
    parsed_unit = units.parse_unit(row["unit"])
    per_tonne_is_estimate = (
        None if per_tonne is None
        else (parsed_unit.kind == "volume" if parsed_unit else None)
    )

    return ReadingOut(
        domain=row["domain"],
        item=row["item"],
        region=row["region"],
        value=row["value"],
        unit=row["unit"],
        currency=row["currency"],
        kind=row["kind"],
        valid_for=row["valid_for"],
        source=row["source"],
        fetched_at=fetched_at,
        is_stale=is_stale,
        confidence=row["confidence"],
        price_per_m3=per_m3,
        price_per_tonne=per_tonne,
        price_per_tonne_is_estimate=per_tonne_is_estimate,
    )


def _sort_key(reading: ReadingOut):
    """Cheapest-first by a comparable basis, not raw bag price (which is
    meaningless across different bag sizes). Prefers price_per_m3 (exact)
    over price_per_tonne (may be an estimate); readings with neither sort
    last rather than being silently dropped."""
    if reading.price_per_m3 is not None:
        return (0, reading.price_per_m3)
    if reading.price_per_tonne is not None:
        return (1, reading.price_per_tonne)
    return (2, 0.0)


@app.get("/health")
def health():
    return {"status": "ok"}


@app.get("/weather/forecast", response_model=list[ReadingOut])
def weather_forecast(
    region: Optional[str] = Query(None, description="Region name, e.g. 'home_farm'"),
    days: int = Query(7, ge=1, le=14),
):
    rows = _store.get_forecast("weather", region, days)
    return [_row_to_reading(r) for r in rows]


@app.get("/price/{domain}", response_model=list[ReadingOut])
def price_domain(domain: str, region: Optional[str] = None):
    _validate_price_domain(domain)
    rows = _store.get_latest(domain, region)
    readings = [_row_to_reading(r) for r in rows]
    return sorted(readings, key=lambda r: (r.item, _sort_key(r)))


@app.get("/price/{domain}/{item}", response_model=list[ReadingOut])
def price_item(domain: str, item: str, region: Optional[str] = None):
    """Latest price per supplier for one item, cheapest-per-m3 (or
    per-tonne if m3 isn't computable) first. Returns a list even when
    only one supplier is configured, since fois never decides which
    supplier "wins" - that's for core/GUI to do."""
    _validate_price_domain(domain)
    rows = _store.get_latest_for_item(domain, item, region)
    if not rows:
        raise HTTPException(status_code=404, detail=f"no reading for {domain}/{item}")
    readings = [_row_to_reading(r) for r in rows]
    return sorted(readings, key=_sort_key)


def _validate_price_domain(domain: str):
    price_domains = {d for d in _config.domain_names() if d != "weather"}
    if domain not in price_domains:
        raise HTTPException(status_code=404, detail=f"unknown price domain: {domain!r}")
