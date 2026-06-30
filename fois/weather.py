"""
fois/weather.py

Weather intelligence for FOIS.

Two responsibilities:

1. Fetch a simple 7-day forecast from BOM (temperature, rainfall, wind).
   Writes the raw forecast into the DB so the operator can also read it.

2. Apply farm station bias correction.
   Reads historical BOM vs actual station readings from the DB to learn
   how much BOM tends to over/under-predict for this specific location.
   Returns a corrected forecast.

The operator never fetches weather itself. It just reads the DB.
FOIS is the sole external data gateway.

Usage:
    weather = WeatherClient(db_path="farm.db", bom_station_id="009053")
    forecast = weather.get_forecast()
    # forecast.days[0].max_temp_corrected etc.
"""

import sqlite3
import logging
import json
from dataclasses import dataclass, field
from datetime import datetime, date, timezone, timedelta
from typing import Optional

import urllib.request
import urllib.error

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# BOM station config
# Gingin: BOM station 009053 (Gingin Aero)
# Update this if your nearest station differs.
# BOM 7-day forecast product: IDW60901 for WA
# ---------------------------------------------------------------------------
BOM_FORECAST_URL = (
    "https://api.weather.bom.gov.au/v1/locations/{geohash}/forecasts/daily"
)

# Gingin geohash (approximate) — update to exact farm geohash for better accuracy
# Get yours from: https://geohash.softeng.co/ or the BOM location API
FARM_GEOHASH = "r6ht"   # ~Gingin area


# ---------------------------------------------------------------------------
# Data structures
# ---------------------------------------------------------------------------

@dataclass
class DayForecast:
    forecast_date: str          # ISO date YYYY-MM-DD
    fetched_at: str             # ISO datetime
    source: str                 # "bom_raw" | "bom_corrected"

    # Temperature
    max_temp_c: Optional[float] = None
    min_temp_c: Optional[float] = None
    max_temp_corrected: Optional[float] = None
    min_temp_corrected: Optional[float] = None

    # Rainfall
    rain_chance_pct: Optional[float] = None
    rain_amount_mm_p50: Optional[float] = None
    rain_amount_mm_corrected: Optional[float] = None

    # Wind
    wind_speed_kmh: Optional[float] = None
    wind_direction: Optional[str] = None

    # Bias correction metadata
    temp_bias_applied: Optional[float] = None   # degrees added
    rain_bias_applied: Optional[float] = None   # mm added


@dataclass
class WeatherForecast:
    generated_at: str
    geohash: str
    days: list[DayForecast] = field(default_factory=list)
    bias_correction_days_used: int = 0   # how many historical days informed the correction


# ---------------------------------------------------------------------------
# WeatherClient
# ---------------------------------------------------------------------------

class WeatherClient:

    FORECAST_TABLE = "fois_weather_forecast"
    BIAS_TABLE = "fois_weather_bias"

    def __init__(self, db_path: str = "farm.db", geohash: str = FARM_GEOHASH):
        self.db_path = db_path
        self.geohash = geohash
        self._ensure_tables()

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def get_forecast(self) -> WeatherForecast:
        """
        Fetch BOM forecast, apply bias correction, write to DB, return result.
        Falls back to last cached forecast if BOM is unreachable.
        """
        raw = self._fetch_bom()

        if raw is None:
            logger.warning("WeatherClient: BOM fetch failed — loading cached forecast")
            return self._load_cached_forecast()

        bias = self._compute_bias()
        forecast = self._apply_bias(raw, bias)
        self._write_forecast(forecast)
        return forecast

    def record_actual(self, obs_date: str, actual_max_c: float, actual_min_c: float,
                      actual_rain_mm: float):
        """
        Called after a day passes. Records actual vs BOM for bias learning.
        The farm weather station data in the DB feeds this automatically
        via update_bias_from_station_history().
        """
        conn = sqlite3.connect(self.db_path)
        try:
            conn.execute("""
                INSERT OR REPLACE INTO fois_weather_actuals
                    (obs_date, actual_max_c, actual_min_c, actual_rain_mm, recorded_at)
                VALUES (?, ?, ?, ?, ?)
            """, (obs_date, actual_max_c, actual_min_c, actual_rain_mm,
                  datetime.now(timezone.utc).isoformat()))
            conn.commit()
        finally:
            conn.close()

    def update_bias_from_station_history(self, lookback_days: int = 30):
        """
        Reads farm weather station readings from the existing sensor_readings
        table and compares against stored BOM forecasts to update bias estimates.

        This is the key integration point with the existing DB schema.
        Assumes station sensors are named: weather_station_temp_max,
        weather_station_temp_min, weather_station_rain_mm
        Adjust sensor IDs to match your actual water.yaml registry.
        """
        conn = sqlite3.connect(self.db_path)
        try:
            # Read recent station daily summaries from sensor_readings
            # This query assumes the existing sensor_store schema
            rows = conn.execute("""
                SELECT
                    date(timestamp) as obs_date,
                    sensor_id,
                    MAX(value) as daily_max,
                    MIN(value) as daily_min
                FROM sensor_readings
                WHERE sensor_id IN (
                    'weather_station_temp',
                    'weather_station_temp_max',
                    'weather_station_temp_min',
                    'weather_station_rain_mm'
                )
                AND timestamp >= datetime('now', ? || ' days')
                GROUP BY date(timestamp), sensor_id
                ORDER BY obs_date
            """, (f"-{lookback_days}",)).fetchall()

            if not rows:
                logger.debug("WeatherClient: no station history found for bias update")
                return

            # Group by date
            by_date: dict[str, dict] = {}
            for obs_date, sensor_id, daily_max, daily_min in rows:
                if obs_date not in by_date:
                    by_date[obs_date] = {}
                by_date[obs_date][sensor_id] = {"max": daily_max, "min": daily_min}

            # Compare against stored BOM forecasts and compute errors
            updated = 0
            for obs_date, station_data in by_date.items():
                bom_row = conn.execute("""
                    SELECT max_temp_c, min_temp_c, rain_amount_mm_p50
                    FROM fois_weather_forecast
                    WHERE forecast_date = ? AND source = 'bom_raw'
                    ORDER BY fetched_at DESC LIMIT 1
                """, (obs_date,)).fetchone()

                if not bom_row:
                    continue

                bom_max, bom_min, bom_rain = bom_row

                # Get station temp (try different sensor naming conventions)
                station_max = (
                    station_data.get("weather_station_temp_max", {}).get("max")
                    or station_data.get("weather_station_temp", {}).get("max")
                )
                station_min = (
                    station_data.get("weather_station_temp_min", {}).get("min")
                    or station_data.get("weather_station_temp", {}).get("min")
                )

                if station_max and bom_max:
                    temp_error = station_max - bom_max   # positive = BOM under-predicted
                    conn.execute("""
                        INSERT OR REPLACE INTO fois_weather_bias_log
                            (obs_date, variable, bom_value, actual_value, error, recorded_at)
                        VALUES (?, 'temp_max', ?, ?, ?, ?)
                    """, (obs_date, bom_max, station_max, temp_error,
                          datetime.now(timezone.utc).isoformat()))
                    updated += 1

            conn.commit()
            if updated:
                logger.info(f"WeatherClient: updated bias log with {updated} observation(s)")

        finally:
            conn.close()

    # ------------------------------------------------------------------
    # BOM fetch
    # ------------------------------------------------------------------

    def _fetch_bom(self) -> Optional[WeatherForecast]:
        url = BOM_FORECAST_URL.format(geohash=self.geohash)
        try:
            req = urllib.request.Request(
                url,
                headers={"User-Agent": "FOIS-FarmSystem/1.0"},
            )
            with urllib.request.urlopen(req, timeout=10) as resp:
                data = json.loads(resp.read().decode())

            return self._parse_bom_response(data)

        except urllib.error.URLError as e:
            logger.warning(f"WeatherClient: BOM request failed: {e}")
            return None
        except (json.JSONDecodeError, KeyError) as e:
            logger.warning(f"WeatherClient: BOM parse error: {e}")
            return None

    def _parse_bom_response(self, data: dict) -> WeatherForecast:
        now = datetime.now(timezone.utc).isoformat()
        forecast = WeatherForecast(generated_at=now, geohash=self.geohash)

        for day_data in data.get("data", []):
            try:
                day = DayForecast(
                    forecast_date=day_data.get("date", "")[:10],
                    fetched_at=now,
                    source="bom_raw",
                    max_temp_c=day_data.get("temp_max"),
                    min_temp_c=day_data.get("temp_min"),
                    rain_chance_pct=day_data.get("rain", {}).get("chance"),
                    rain_amount_mm_p50=day_data.get("rain", {}).get("amount", {}).get("median"),
                    wind_speed_kmh=day_data.get("wind", {}).get("speed_kilometre"),
                    wind_direction=day_data.get("wind", {}).get("direction"),
                )
                forecast.days.append(day)
            except (TypeError, AttributeError) as e:
                logger.warning(f"WeatherClient: skipping malformed day entry: {e}")
                continue

        logger.info(f"WeatherClient: fetched {len(forecast.days)}-day BOM forecast")
        return forecast

    # ------------------------------------------------------------------
    # Bias correction
    # ------------------------------------------------------------------

    def _compute_bias(self, lookback_days: int = 30) -> dict:
        """
        Compute rolling average forecast error from bias log.
        Returns: {"temp_max": float, "temp_min": float, "rain": float}
        """
        bias = {"temp_max": 0.0, "temp_min": 0.0, "rain": 0.0}
        conn = sqlite3.connect(self.db_path)
        try:
            rows = conn.execute("""
                SELECT variable, AVG(error) as mean_error, COUNT(*) as n
                FROM fois_weather_bias_log
                WHERE recorded_at >= datetime('now', ? || ' days')
                GROUP BY variable
            """, (f"-{lookback_days}",)).fetchall()

            for variable, mean_error, n in rows:
                if variable == "temp_max":
                    bias["temp_max"] = round(mean_error, 2)
                elif variable == "temp_min":
                    bias["temp_min"] = round(mean_error, 2)
                elif variable == "rain":
                    bias["rain"] = round(mean_error, 2)

                logger.debug(f"WeatherClient: bias {variable} = {mean_error:+.2f} (n={n})")

        finally:
            conn.close()

        return bias

    def _apply_bias(self, forecast: WeatherForecast, bias: dict) -> WeatherForecast:
        for day in forecast.days:
            if day.max_temp_c is not None:
                day.max_temp_corrected = round(day.max_temp_c + bias["temp_max"], 1)
                day.temp_bias_applied = bias["temp_max"]
            if day.min_temp_c is not None:
                day.min_temp_corrected = round(day.min_temp_c + bias["temp_min"], 1)
            if day.rain_amount_mm_p50 is not None:
                day.rain_amount_mm_corrected = max(
                    0.0, round(day.rain_amount_mm_p50 + bias["rain"], 1)
                )
                day.rain_bias_applied = bias["rain"]
            day.source = "bom_corrected"

        forecast.bias_correction_days_used = lookback_days if bias["temp_max"] != 0 else 0
        return forecast

    # ------------------------------------------------------------------
    # DB persistence
    # ------------------------------------------------------------------

    def _write_forecast(self, forecast: WeatherForecast):
        conn = sqlite3.connect(self.db_path)
        try:
            for day in forecast.days:
                conn.execute("""
                    INSERT INTO fois_weather_forecast (
                        forecast_date, fetched_at, source,
                        max_temp_c, min_temp_c, max_temp_corrected, min_temp_corrected,
                        rain_chance_pct, rain_amount_mm_p50, rain_amount_mm_corrected,
                        wind_speed_kmh, wind_direction,
                        temp_bias_applied, rain_bias_applied
                    ) VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                """, (
                    day.forecast_date, day.fetched_at, day.source,
                    day.max_temp_c, day.min_temp_c,
                    day.max_temp_corrected, day.min_temp_corrected,
                    day.rain_chance_pct, day.rain_amount_mm_p50,
                    day.rain_amount_mm_corrected,
                    day.wind_speed_kmh, day.wind_direction,
                    day.temp_bias_applied, day.rain_bias_applied,
                ))
            conn.commit()
            logger.info(f"WeatherClient: wrote {len(forecast.days)} forecast days to DB")
        finally:
            conn.close()

    def _load_cached_forecast(self) -> WeatherForecast:
        """Return the most recent forecast from DB if BOM is unreachable."""
        conn = sqlite3.connect(self.db_path)
        try:
            rows = conn.execute("""
                SELECT forecast_date, fetched_at, source,
                       max_temp_c, min_temp_c, max_temp_corrected, min_temp_corrected,
                       rain_chance_pct, rain_amount_mm_p50, rain_amount_mm_corrected,
                       wind_speed_kmh, wind_direction, temp_bias_applied, rain_bias_applied
                FROM fois_weather_forecast
                WHERE fetched_at = (SELECT MAX(fetched_at) FROM fois_weather_forecast)
                ORDER BY forecast_date
            """).fetchall()

            forecast = WeatherForecast(
                generated_at=datetime.now(timezone.utc).isoformat(),
                geohash=self.geohash,
            )
            for row in rows:
                day = DayForecast(
                    forecast_date=row[0], fetched_at=row[1], source=row[2] + "_cached",
                    max_temp_c=row[3], min_temp_c=row[4],
                    max_temp_corrected=row[5], min_temp_corrected=row[6],
                    rain_chance_pct=row[7], rain_amount_mm_p50=row[8],
                    rain_amount_mm_corrected=row[9],
                    wind_speed_kmh=row[10], wind_direction=row[11],
                    temp_bias_applied=row[12], rain_bias_applied=row[13],
                )
                forecast.days.append(day)

            logger.info(f"WeatherClient: loaded {len(forecast.days)} cached forecast days")
            return forecast
        finally:
            conn.close()

    def _ensure_tables(self):
        conn = sqlite3.connect(self.db_path)
        try:
            conn.executescript("""
                CREATE TABLE IF NOT EXISTS fois_weather_forecast (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    forecast_date TEXT NOT NULL,
                    fetched_at TEXT NOT NULL,
                    source TEXT NOT NULL,
                    max_temp_c REAL,
                    min_temp_c REAL,
                    max_temp_corrected REAL,
                    min_temp_corrected REAL,
                    rain_chance_pct REAL,
                    rain_amount_mm_p50 REAL,
                    rain_amount_mm_corrected REAL,
                    wind_speed_kmh REAL,
                    wind_direction TEXT,
                    temp_bias_applied REAL,
                    rain_bias_applied REAL
                );

                CREATE TABLE IF NOT EXISTS fois_weather_bias_log (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    obs_date TEXT NOT NULL,
                    variable TEXT NOT NULL,
                    bom_value REAL,
                    actual_value REAL,
                    error REAL,
                    recorded_at TEXT NOT NULL,
                    UNIQUE(obs_date, variable)
                );

                CREATE TABLE IF NOT EXISTS fois_weather_actuals (
                    obs_date TEXT PRIMARY KEY,
                    actual_max_c REAL,
                    actual_min_c REAL,
                    actual_rain_mm REAL,
                    recorded_at TEXT NOT NULL
                );
            """)
            conn.commit()
        finally:
            conn.close()


# Expose lookback_days in _apply_bias properly
_apply_bias_orig = WeatherClient._apply_bias
def _apply_bias_fixed(self, forecast, bias, lookback_days=30):
    for day in forecast.days:
        if day.max_temp_c is not None:
            day.max_temp_corrected = round(day.max_temp_c + bias["temp_max"], 1)
            day.temp_bias_applied = bias["temp_max"]
        if day.min_temp_c is not None:
            day.min_temp_corrected = round(day.min_temp_c + bias["temp_min"], 1)
        if day.rain_amount_mm_p50 is not None:
            day.rain_amount_mm_corrected = max(
                0.0, round(day.rain_amount_mm_p50 + bias["rain"], 1)
            )
            day.rain_bias_applied = bias["rain"]
        day.source = "bom_corrected"
    forecast.bias_correction_days_used = lookback_days if bias["temp_max"] != 0 else 0
    return forecast
WeatherClient._apply_bias = _apply_bias_fixed
