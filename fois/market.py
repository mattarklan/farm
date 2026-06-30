"""
fois/market.py

Market intelligence for FOIS.

Fetches wholesale vegetable price data from publicly available sources
and estimates what price a crop is likely to achieve at harvest time.

Sources (in priority order):
    1. ABARES / agriculture.gov.au datasets
    2. Perth Market Authority (PMA) — perthmarket.com.au
    3. Hort Innovation market reports
    4. Fallback: crop profile indicative prices

Price output is a distribution (P10/P50/P90) to reflect uncertainty.
The further the harvest date, the wider the distribution.

Usage:
    market = MarketClient(db_path="farm.db")
    price = market.get_price_forecast("Capsicum", harvest_month=6)
    print(price.p50_per_kg)
"""

import sqlite3
import json
import logging
import urllib.request
import urllib.error
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Optional

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class PriceForecast:
    crop: str
    harvest_month: int
    harvest_year: int
    fetched_at: str
    source: str

    p10_per_kg: float
    p50_per_kg: float
    p90_per_kg: float

    confidence: float           # 0–1. Higher when recent data available.
    notes: str = ""


# ---------------------------------------------------------------------------
# MarketClient
# ---------------------------------------------------------------------------

class MarketClient:

    PRICE_TABLE = "fois_market_prices"

    # Seasonality multipliers by month (1.0 = average)
    # Derived from historical Perth wholesale data patterns.
    # Capsicum and tomato follow similar WA seasonal curves.
    SEASONALITY = {
        "Capsicum": {
            1: 0.75, 2: 0.80, 3: 0.90, 4: 1.00,
            5: 1.15, 6: 1.25, 7: 1.30, 8: 1.20,
            9: 1.05, 10: 0.95, 11: 0.85, 12: 0.75,
        },
        "Tomato": {
            1: 0.70, 2: 0.75, 3: 0.85, 4: 0.95,
            5: 1.10, 6: 1.20, 7: 1.25, 8: 1.20,
            9: 1.05, 10: 0.95, 11: 0.85, 12: 0.70,
        },
        "Zucchini": {
            1: 0.80, 2: 0.85, 3: 0.90, 4: 1.00,
            5: 1.15, 6: 1.20, 7: 1.20, 8: 1.10,
            9: 1.00, 10: 0.90, 11: 0.85, 12: 0.80,
        },
    }

    # Base indicative prices per kg (fallback when no live data)
    BASE_PRICES = {
        "Capsicum": 3.20,
        "Tomato": 2.80,
        "Zucchini": 2.20,
    }

    def __init__(self, db_path: str = "farm.db"):
        self.db_path = db_path
        self._ensure_tables()

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def get_price_forecast(self, crop: str, harvest_month: int,
                           harvest_year: int) -> PriceForecast:
        """
        Return a price distribution for a crop at a given harvest month.
        Tries live data first, falls back to historical DB, then to
        built-in seasonality model.
        """
        now = datetime.now(timezone.utc).isoformat()

        # Try to get recent data from DB (written by previous fetch runs)
        cached = self._load_recent_price(crop, harvest_month)
        if cached:
            logger.debug(f"MarketClient: using cached price for {crop} month {harvest_month}")
            return cached

        # Try live fetch
        live = self._fetch_live_price(crop)
        if live:
            base = live
            source = "live_fetch"
            confidence = 0.75
        else:
            base = self.BASE_PRICES.get(crop, 2.50)
            source = "indicative_fallback"
            confidence = 0.45

        # Apply seasonality
        seasonality = self.SEASONALITY.get(crop, {})
        multiplier = seasonality.get(harvest_month, 1.0)
        adjusted = base * multiplier

        # Spread widens with distance to harvest
        months_ahead = self._months_until(harvest_month, harvest_year)
        spread_pct = min(0.40, 0.10 + months_ahead * 0.03)

        p50 = round(adjusted, 2)
        p10 = round(adjusted * (1 - spread_pct), 2)
        p90 = round(adjusted * (1 + spread_pct), 2)

        forecast = PriceForecast(
            crop=crop,
            harvest_month=harvest_month,
            harvest_year=harvest_year,
            fetched_at=now,
            source=source,
            p10_per_kg=p10,
            p50_per_kg=p50,
            p90_per_kg=p90,
            confidence=confidence,
            notes=f"Seasonality multiplier: {multiplier:.2f}x, {months_ahead} months ahead",
        )

        self._write_price(forecast)
        return forecast

    def get_farm_sale_history(self, crop: str) -> list[dict]:
        """
        Read historical farm sale prices from the DB.
        This is the highest-confidence data — actual prices the farm achieved.
        The operator's DecisionMemory or a dedicated sales log table feeds this.
        """
        conn = sqlite3.connect(self.db_path)
        try:
            rows = conn.execute("""
                SELECT sale_date, crop, price_per_kg, grade, kg_sold, notes
                FROM farm_sales
                WHERE crop = ?
                ORDER BY sale_date DESC
                LIMIT 50
            """, (crop,)).fetchall()

            return [
                {
                    "sale_date": r[0], "crop": r[1], "price_per_kg": r[2],
                    "grade": r[3], "kg_sold": r[4], "notes": r[5],
                }
                for r in rows
            ]
        except sqlite3.OperationalError:
            # farm_sales table doesn't exist yet — that's fine early on
            logger.debug("MarketClient: farm_sales table not found — no farm history yet")
            return []
        finally:
            conn.close()

    # ------------------------------------------------------------------
    # Live fetch
    # ------------------------------------------------------------------

    def _fetch_live_price(self, crop: str) -> Optional[float]:
        """
        Attempt to fetch current wholesale price from public sources.

        Primary target: ABARES agricultural commodity data
        Fallback: Perth Market Authority published reports

        These are best-effort. Returns None if unavailable.
        A None result falls back to built-in seasonality model.
        """

        # ABARES crop price indices — publicly available JSON
        # URL pattern may change — treat as fragile, always handle failure
        abares_url = (
            "https://www.agriculture.gov.au/abares/research-topics/"
            "agricultural-outlook/data"
        )

        # PMA publishes weekly market reports as PDFs — not easily parseable.
        # For now this is a stub. In production, scrape or use a paid data feed.
        # Return None to fall through to the seasonality model.

        logger.debug(f"MarketClient: live fetch for {crop} not yet implemented — using model")
        return None

    # ------------------------------------------------------------------
    # DB helpers
    # ------------------------------------------------------------------

    def _load_recent_price(self, crop: str, harvest_month: int,
                           max_age_days: int = 7) -> Optional[PriceForecast]:
        conn = sqlite3.connect(self.db_path)
        try:
            row = conn.execute("""
                SELECT crop, harvest_month, harvest_year, fetched_at, source,
                       p10_per_kg, p50_per_kg, p90_per_kg, confidence, notes
                FROM fois_market_prices
                WHERE crop = ? AND harvest_month = ?
                  AND fetched_at >= datetime('now', ? || ' days')
                ORDER BY fetched_at DESC LIMIT 1
            """, (crop, harvest_month, f"-{max_age_days}")).fetchone()

            if not row:
                return None

            return PriceForecast(
                crop=row[0], harvest_month=row[1], harvest_year=row[2],
                fetched_at=row[3], source=row[4] + "_cached",
                p10_per_kg=row[5], p50_per_kg=row[6], p90_per_kg=row[7],
                confidence=row[8], notes=row[9],
            )
        finally:
            conn.close()

    def _write_price(self, forecast: PriceForecast):
        conn = sqlite3.connect(self.db_path)
        try:
            conn.execute("""
                INSERT INTO fois_market_prices (
                    crop, harvest_month, harvest_year, fetched_at, source,
                    p10_per_kg, p50_per_kg, p90_per_kg, confidence, notes
                ) VALUES (?,?,?,?,?,?,?,?,?,?)
            """, (
                forecast.crop, forecast.harvest_month, forecast.harvest_year,
                forecast.fetched_at, forecast.source,
                forecast.p10_per_kg, forecast.p50_per_kg, forecast.p90_per_kg,
                forecast.confidence, forecast.notes,
            ))
            conn.commit()
        finally:
            conn.close()

    def _months_until(self, harvest_month: int, harvest_year: int) -> int:
        now = datetime.now(timezone.utc)
        months = (harvest_year - now.year) * 12 + (harvest_month - now.month)
        return max(0, months)

    def _ensure_tables(self):
        conn = sqlite3.connect(self.db_path)
        try:
            conn.executescript("""
                CREATE TABLE IF NOT EXISTS fois_market_prices (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    crop TEXT NOT NULL,
                    harvest_month INTEGER NOT NULL,
                    harvest_year INTEGER NOT NULL,
                    fetched_at TEXT NOT NULL,
                    source TEXT NOT NULL,
                    p10_per_kg REAL,
                    p50_per_kg REAL,
                    p90_per_kg REAL,
                    confidence REAL,
                    notes TEXT
                );
            """)
            conn.commit()
        finally:
            conn.close()
