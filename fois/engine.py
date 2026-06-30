"""
fois/engine.py

The FOIS investor brain.

Reads:
    - Zone state and soil data from farm DB
    - Weather forecast (from fois/weather.py, already in DB)
    - Crop profiles (YAML files)
    - Market price forecasts (from fois/market.py)
    - Farm history (past yields, rotation records)

Produces:
    - Ranked list of CropOpportunity objects
    - Writes winning proposal(s) as YAML to fois/proposals/pending/
      for the governance layer to evaluate

This is Stage 1 (statistical / empirical).
Monte Carlo simulation is used for profit distributions.
No ML yet — that's Stage 2/3.

Usage:
    engine = FoisEngine(db_path="farm.db")
    opportunities = engine.run()
"""

import sqlite3
import random
import logging
import uuid
from dataclasses import dataclass, field
from datetime import datetime, date, timezone, timedelta
from pathlib import Path
from typing import Optional

import yaml

from fois.weather import WeatherClient, WeatherForecast
from fois.market import MarketClient

logger = logging.getLogger(__name__)

CROP_PROFILES_DIR = Path("fois/crop_profiles")
PROPOSALS_PENDING_DIR = Path("fois/proposals/pending")
MONTE_CARLO_ITERATIONS = 1000


# ---------------------------------------------------------------------------
# Zone snapshot — read from DB
# ---------------------------------------------------------------------------

@dataclass
class ZoneSnapshot:
    zone_id: str
    area_sqm: float
    soil_ph: Optional[float] = None
    soil_ec: Optional[float] = None
    soil_salinity_risk: Optional[str] = None
    current_crop: Optional[str] = None
    last_crop: Optional[str] = None
    last_planted: Optional[str] = None   # ISO date
    last_harvested: Optional[str] = None
    avg_soil_moisture: Optional[float] = None   # from recent sensor readings


# ---------------------------------------------------------------------------
# Crop Opportunity — the primary output object
# ---------------------------------------------------------------------------

@dataclass
class CropOpportunity:
    proposal_id: str
    crop: str
    cultivar: str
    zone_id: str
    zone_area_sqm: float
    planting_date: str
    harvest_date: str

    # Economic
    p10_net_profit: float
    p50_net_profit: float
    p90_net_profit: float
    expected_roi: float
    risk_level: str           # low | moderate | high

    # Resource demands
    daily_water_litres: float
    peak_water_litres: float
    season_total_water_litres: float
    daily_power_kwh: float
    peak_power_kw: float

    # Market
    price_p10: float
    price_p50: float
    price_p90: float
    price_confidence: float

    # Quality
    premium_grade_pct: float
    expected_yield_kg_p50: float

    # Meta
    confidence: float
    reasons: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    generated_at: str = ""
    model_version: str = "0.1.0"

    def utility_score(self) -> float:
        """
        Risk-adjusted ranking score.
        Blends P50 profit with downside protection (P10) and confidence.
        Higher is better.
        """
        downside_weight = 0.3
        confidence_weight = 0.15
        base = self.p50_net_profit * (1 - downside_weight) + self.p10_net_profit * downside_weight
        return base * (1 + confidence_weight * self.confidence)


# ---------------------------------------------------------------------------
# FoisEngine
# ---------------------------------------------------------------------------

class FoisEngine:

    def __init__(self, db_path: str = "farm.db", top_n: int = 3):
        self.db_path = db_path
        self.top_n = top_n
        self.weather_client = WeatherClient(db_path=db_path)
        self.market_client = MarketClient(db_path=db_path)
        self._ensure_tables()

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def run(self) -> list[CropOpportunity]:
        """
        Full FOIS run:
        1. Refresh weather
        2. Load zones
        3. Load crop profiles
        4. Score all (zone, crop) combinations
        5. Rank by utility score
        6. Write top proposals to pending/
        7. Return ranked list
        """
        logger.info("FOIS engine: starting run")

        # Step 1: Weather
        logger.info("FOIS: fetching weather forecast")
        self.weather_client.update_bias_from_station_history()
        forecast = self.weather_client.get_forecast()

        # Step 2: Zones
        zones = self._load_zones()
        if not zones:
            logger.warning("FOIS: no zones found in DB — nothing to evaluate")
            return []

        # Step 3: Crop profiles
        profiles = self._load_crop_profiles()
        if not profiles:
            logger.warning("FOIS: no crop profiles found")
            return []

        # Step 4: Score all combinations
        opportunities = []
        today = date.today()

        for zone in zones:
            for crop_name, profile in profiles.items():
                opp = self._evaluate(zone, profile, forecast, today)
                if opp is not None:
                    opportunities.append(opp)

        # Step 5: Rank
        opportunities.sort(key=lambda o: o.utility_score(), reverse=True)
        ranked = opportunities[:self.top_n]

        # Step 6: Write proposals
        for opp in ranked:
            self._write_proposal(opp)

        # Step 7: Log summary to DB
        self._log_run(ranked)

        logger.info(f"FOIS engine: run complete. {len(ranked)} proposal(s) written.")
        for i, opp in enumerate(ranked):
            logger.info(
                f"  #{i+1}: {opp.crop} in {opp.zone_id} — "
                f"P50 profit ${opp.p50_net_profit:,.0f}, confidence {opp.confidence:.2f}"
            )

        return ranked

    # ------------------------------------------------------------------
    # Zone loading
    # ------------------------------------------------------------------

    def _load_zones(self) -> list[ZoneSnapshot]:
        """
        Load zone definitions and recent sensor state from the DB.
        Tries the fois_zones table first (FOIS-managed zone registry),
        then falls back to reading zone assets from the operator's registry.
        """
        conn = sqlite3.connect(self.db_path)
        zones = []
        try:
            # Try FOIS zone table
            try:
                rows = conn.execute("""
                    SELECT zone_id, area_sqm, soil_ph, soil_ec,
                           soil_salinity_risk, current_crop,
                           last_crop, last_planted, last_harvested
                    FROM fois_zones
                """).fetchall()

                for row in rows:
                    z = ZoneSnapshot(
                        zone_id=row[0], area_sqm=row[1] or 500.0,
                        soil_ph=row[2], soil_ec=row[3],
                        soil_salinity_risk=row[4],
                        current_crop=row[5],
                        last_crop=row[6], last_planted=row[7],
                        last_harvested=row[8],
                    )
                    # Enrich with recent soil moisture
                    z.avg_soil_moisture = self._get_recent_soil_moisture(conn, z.zone_id)
                    zones.append(z)

            except sqlite3.OperationalError:
                # fois_zones table doesn't exist yet — use hardcoded known zones
                logger.info("FOIS: fois_zones table not found — using default zone list")
                zones = self._default_zones(conn)

        finally:
            conn.close()

        logger.info(f"FOIS: loaded {len(zones)} zone(s)")
        return zones

    def _default_zones(self, conn) -> list[ZoneSnapshot]:
        """
        Fallback zone list matching the operator's water.yaml registry.
        Replace with fois_zones table population as the system matures.
        """
        default_zone_ids = [
            ("zone_capsicum_a", 500.0),
            ("zone_capsicum_b", 500.0),
            ("zone_tomato_a", 600.0),
            ("zone_tomato_b", 600.0),
        ]
        zones = []
        for zone_id, area in default_zone_ids:
            z = ZoneSnapshot(zone_id=zone_id, area_sqm=area)
            z.avg_soil_moisture = self._get_recent_soil_moisture(conn, zone_id)
            zones.append(z)
        return zones

    def _get_recent_soil_moisture(self, conn, zone_id: str) -> Optional[float]:
        try:
            sensor_id = f"{zone_id}_soil_moisture"
            row = conn.execute("""
                SELECT AVG(value) FROM sensor_readings
                WHERE sensor_id = ?
                AND timestamp >= datetime('now', '-7 days')
            """, (sensor_id,)).fetchone()
            return row[0] if row and row[0] is not None else None
        except sqlite3.OperationalError:
            return None

    # ------------------------------------------------------------------
    # Crop profile loading
    # ------------------------------------------------------------------

    def _load_crop_profiles(self) -> dict[str, dict]:
        profiles = {}
        for path in CROP_PROFILES_DIR.glob("*.yaml"):
            try:
                with open(path) as f:
                    profile = yaml.safe_load(f)
                crop_name = profile.get("crop")
                if crop_name:
                    profiles[crop_name] = profile
                    logger.debug(f"FOIS: loaded crop profile: {crop_name}")
            except Exception as e:
                logger.warning(f"FOIS: could not load crop profile {path}: {e}")
        return profiles

    # ------------------------------------------------------------------
    # Opportunity evaluation
    # ------------------------------------------------------------------

    def _evaluate(self, zone: ZoneSnapshot, profile: dict,
                  forecast: WeatherForecast, today: date) -> Optional[CropOpportunity]:
        """
        Evaluate a single (zone, crop) combination.
        Returns None if the combination is filtered out.
        """
        crop_name = profile.get("crop")
        reasons = []
        warnings = []

        # ---- Filter 1: Climate suitability ----
        suitability = profile.get("gingin_suitability", "unknown")
        if suitability == "unsuitable":
            logger.debug(f"FOIS: {crop_name} filtered — unsuitable for Gingin climate")
            return None

        # ---- Filter 2: Frost risk ----
        frost_months = profile.get("climate", {}).get("frost_risk_months", [])
        planting_window = self._best_planting_window(profile, today)
        if planting_window is None:
            logger.debug(f"FOIS: {crop_name} filtered — no valid planting window from today")
            return None

        planting_date, harvest_date, window_label = planting_window
        planting_month = planting_date.month

        if planting_month in frost_months:
            logger.debug(f"FOIS: {crop_name} filtered — planting month {planting_month} in frost risk window")
            return None

        # ---- Filter 3: Rotation check ----
        do_not_follow = profile.get("rotation", {}).get("do_not_follow", [])
        if zone.last_crop and zone.last_crop in do_not_follow:
            warnings.append(f"Rotation warning: last crop was {zone.last_crop}")
            # Don't filter — warn and reduce confidence

        if zone.current_crop:
            warnings.append(f"Zone currently has {zone.current_crop} — check harvest timing")

        # ---- Filter 4: Soil pH ----
        soil_ph_min = profile.get("soil", {}).get("ph_min", 5.5)
        soil_ph_max = profile.get("soil", {}).get("ph_max", 7.5)
        if zone.soil_ph:
            if not (soil_ph_min <= zone.soil_ph <= soil_ph_max):
                warnings.append(f"Soil pH {zone.soil_ph} outside optimal {soil_ph_min}–{soil_ph_max}")

        # ---- Filter 5: Near-term weather ----
        weather_ok, weather_reason = self._check_weather_suitability(profile, forecast)
        if not weather_ok:
            warnings.append(weather_reason)
            # Still propose — just warn. Operator constraints handle day-to-day.

        # ---- Build resource profile ----
        area = zone.area_sqm
        daily_litres_per_sqm = profile.get("water", {}).get("daily_litres_per_sqm", 5.0)
        season_mm = profile.get("water", {}).get("season_mm_equivalent", 500)

        daily_water = area * daily_litres_per_sqm
        peak_water = daily_water * 1.4   # peak ~40% above average
        season_water = area * season_mm

        # Power estimate: drip irrigation pump ~0.5 kWh per 1000L
        daily_kwh = daily_water / 1000 * 0.5
        peak_kw = peak_water / 1000 * 0.5 / 2   # peak over 2-hour window

        # ---- Market price ----
        price_forecast = self.market_client.get_price_forecast(
            crop_name, harvest_date.month, harvest_date.year
        )

        # ---- Yield estimate (Monte Carlo) ----
        yield_model = profile.get("yield", {})
        p50_yield = yield_model.get("expected_kg_per_sqm_p50", 5.0) * area
        p10_yield = yield_model.get("expected_kg_per_sqm_p10", 3.0) * area
        p90_yield = yield_model.get("expected_kg_per_sqm_p90", 8.0) * area

        premium_pct = yield_model.get("premium_grade_pct_p50", 60) / 100
        quality_multiplier = profile.get("market", {}).get("quality_premium_multiplier", 1.25)

        p10_profit, p50_profit, p90_profit = self._monte_carlo_profit(
            area=area,
            p10_yield=p10_yield, p50_yield=p50_yield, p90_yield=p90_yield,
            price_p10=price_forecast.p10_per_kg,
            price_p50=price_forecast.p50_per_kg,
            price_p90=price_forecast.p90_per_kg,
            premium_pct=premium_pct,
            quality_multiplier=quality_multiplier,
            daily_water=daily_water,
            season_days=(harvest_date - planting_date).days,
        )

        # ---- Confidence ----
        confidence = self._compute_confidence(
            zone=zone,
            profile=profile,
            price_confidence=price_forecast.confidence,
            has_warnings=len(warnings) > 0,
        )

        # ---- Reasons ----
        if suitability == "high":
            reasons.append(f"High climate suitability for Gingin")
        if price_forecast.p50_per_kg > self.BASE_PRICE_THRESHOLD(crop_name):
            reasons.append(f"Favourable harvest-time price outlook (${price_forecast.p50_per_kg:.2f}/kg)")
        if not warnings:
            reasons.append("No rotation or soil concerns")
        reasons.append(f"{window_label} planting window")

        # ---- Risk level ----
        risk_level = self._assess_risk(warnings, confidence, p10_profit, p50_profit)

        # ---- Pick cultivar ----
        cultivars = profile.get("cultivars", [])
        cultivar = cultivars[0]["name"] if cultivars else "Standard"

        roi = p50_profit / max(1, p50_yield * price_forecast.p50_per_kg * 0.4)

        return CropOpportunity(
            proposal_id=f"prop_{datetime.now(timezone.utc).strftime('%Y%m%d')}_{uuid.uuid4().hex[:6]}",
            crop=crop_name,
            cultivar=cultivar,
            zone_id=zone.zone_id,
            zone_area_sqm=area,
            planting_date=planting_date.isoformat(),
            harvest_date=harvest_date.isoformat(),
            p10_net_profit=round(p10_profit),
            p50_net_profit=round(p50_profit),
            p90_net_profit=round(p90_profit),
            expected_roi=round(roi, 2),
            risk_level=risk_level,
            daily_water_litres=round(daily_water),
            peak_water_litres=round(peak_water),
            season_total_water_litres=round(season_water),
            daily_power_kwh=round(daily_kwh, 2),
            peak_power_kw=round(peak_kw, 2),
            price_p10=price_forecast.p10_per_kg,
            price_p50=price_forecast.p50_per_kg,
            price_p90=price_forecast.p90_per_kg,
            price_confidence=price_forecast.confidence,
            premium_grade_pct=premium_pct * 100,
            expected_yield_kg_p50=round(p50_yield),
            confidence=round(confidence, 2),
            reasons=reasons,
            warnings=warnings,
            generated_at=datetime.now(timezone.utc).isoformat(),
        )

    # ------------------------------------------------------------------
    # Monte Carlo profit simulation
    # ------------------------------------------------------------------

    def _monte_carlo_profit(self, area, p10_yield, p50_yield, p90_yield,
                            price_p10, price_p50, price_p90,
                            premium_pct, quality_multiplier,
                            daily_water, season_days) -> tuple[float, float, float]:
        """
        Simulate profit distribution using triangular distributions for
        yield and price uncertainty. Stage 1 approach.
        """
        profits = []
        rng = random.Random(42)   # deterministic seed for reproducibility

        # Cost model (simple empirical estimates)
        water_cost_per_kl = 0.80         # $/kL pumping cost
        fertiliser_cost_per_sqm = 0.45
        chemical_cost_per_sqm = 0.30
        labour_cost_per_sqm = 1.20
        misc_cost_per_sqm = 0.25

        total_fixed_costs = area * (
            fertiliser_cost_per_sqm + chemical_cost_per_sqm +
            labour_cost_per_sqm + misc_cost_per_sqm
        )
        water_cost = (daily_water * season_days / 1000) * water_cost_per_kl

        total_costs = total_fixed_costs + water_cost

        for _ in range(MONTE_CARLO_ITERATIONS):
            # Sample yield from triangular distribution
            yield_kg = rng.triangular(p10_yield, p90_yield, p50_yield)

            # Sample price
            price = rng.triangular(price_p10, price_p90, price_p50)

            # Quality mix
            premium_kg = yield_kg * premium_pct
            standard_kg = yield_kg * (1 - premium_pct)
            revenue = (premium_kg * price * quality_multiplier) + (standard_kg * price)

            profit = revenue - total_costs
            profits.append(profit)

        profits.sort()
        n = len(profits)
        return (
            profits[int(n * 0.10)],
            profits[int(n * 0.50)],
            profits[int(n * 0.90)],
        )

    # ------------------------------------------------------------------
    # Supporting methods
    # ------------------------------------------------------------------

    def _best_planting_window(self, profile: dict,
                               today: date) -> Optional[tuple[date, date, str]]:
        """Find the next valid planting window from today."""
        windows = profile.get("planting_windows", [])
        if not windows:
            return None

        candidates = []
        for window in windows:
            start_month = window["start_month"]
            end_month = window["end_month"]
            offset_weeks = window.get("harvest_offset_weeks", 16)
            label = window.get("label", "")

            # Try this year and next year
            for year in [today.year, today.year + 1]:
                try:
                    planting = date(year, start_month, 1)
                    if planting < today:
                        # Try end of window month
                        planting = date(year, end_month, 15)
                    if planting < today:
                        continue
                    harvest = planting + timedelta(weeks=offset_weeks)
                    candidates.append((planting, harvest, label))
                except ValueError:
                    continue

        if not candidates:
            return None

        # Return soonest
        candidates.sort(key=lambda x: x[0])
        return candidates[0]

    def _check_weather_suitability(self, profile: dict,
                                    forecast: WeatherForecast) -> tuple[bool, str]:
        """Check 7-day forecast against crop temperature limits."""
        temp_max = profile.get("climate", {}).get("temp_max_c", 40)

        for day in forecast.days[:7]:
            check_temp = day.max_temp_corrected or day.max_temp_c
            if check_temp and check_temp > temp_max:
                return False, f"Forecast max {check_temp}°C exceeds crop limit {temp_max}°C"

        return True, ""

    def _compute_confidence(self, zone: ZoneSnapshot, profile: dict,
                             price_confidence: float, has_warnings: bool) -> float:
        confidence = 0.70   # base

        # Boost for good suitability rating
        if profile.get("gingin_suitability") == "high":
            confidence += 0.10

        # Boost for soil data available
        if zone.soil_ph:
            confidence += 0.05

        # Boost for moisture data
        if zone.avg_soil_moisture:
            confidence += 0.03

        # Blend in price confidence
        confidence = confidence * 0.7 + price_confidence * 0.3

        # Penalty for warnings
        if has_warnings:
            confidence -= 0.08

        return max(0.1, min(1.0, confidence))

    def _assess_risk(self, warnings: list, confidence: float,
                     p10: float, p50: float) -> str:
        downside_ratio = p10 / p50 if p50 > 0 else 0
        if confidence > 0.75 and downside_ratio > 0.5 and not warnings:
            return "low"
        elif confidence > 0.55 and downside_ratio > 0.3:
            return "moderate"
        else:
            return "high"

    def BASE_PRICE_THRESHOLD(self, crop: str) -> float:
        thresholds = {"Capsicum": 2.80, "Tomato": 2.40, "Zucchini": 1.90}
        return thresholds.get(crop, 2.00)

    # ------------------------------------------------------------------
    # Proposal YAML writer
    # ------------------------------------------------------------------

    def _write_proposal(self, opp: CropOpportunity):
        """
        Write a CropOpportunity as a governance-ready proposal YAML.
        Generates the proposed_rules section from the crop profile and zone.
        """
        PROPOSALS_PENDING_DIR.mkdir(parents=True, exist_ok=True)

        harvest_date = opp.harvest_date
        expires_at = opp.planting_date  # proposal expires if not accepted before planting

        # Generate irrigation rules for this zone/crop combination
        proposed_rules = self._generate_rules(opp, harvest_date)

        proposal = {
            "proposal_id": opp.proposal_id,
            "generated_at": opp.generated_at,
            "expires_at": f"{opp.planting_date}T00:00:00Z",
            "fois_model_version": opp.model_version,
            "confidence": opp.confidence,

            "crop": {
                "name": opp.crop,
                "cultivar": opp.cultivar,
                "zone_id": opp.zone_id,
                "planting_date": opp.planting_date,
                "harvest_date": opp.harvest_date,
            },

            "economics": {
                "expected_net_profit": {
                    "p10": opp.p10_net_profit,
                    "p50": opp.p50_net_profit,
                    "p90": opp.p90_net_profit,
                },
                "expected_roi": opp.expected_roi,
                "risk_level": opp.risk_level,
                "price_per_kg": {
                    "p10": opp.price_p10,
                    "p50": opp.price_p50,
                    "p90": opp.price_p90,
                },
            },

            "resources": {
                "water": {
                    "daily_litres": opp.daily_water_litres,
                    "peak_litres": opp.peak_water_litres,
                    "season_total_litres": opp.season_total_water_litres,
                },
                "power": {
                    "daily_kwh": opp.daily_power_kwh,
                    "peak_kw": opp.peak_power_kw,
                    "season_total_kwh": round(opp.daily_power_kwh * 
                                              (datetime.fromisoformat(opp.harvest_date) - 
                                               datetime.fromisoformat(opp.planting_date)).days),
                },
            },

            "yield_forecast": {
                "expected_kg_p50": opp.expected_yield_kg_p50,
                "premium_grade_pct": opp.premium_grade_pct,
            },

            "reasons": opp.reasons,
            "warnings": opp.warnings,

            "proposed_rules": proposed_rules,
        }

        path = PROPOSALS_PENDING_DIR / f"{opp.proposal_id}.yaml"
        with open(path, "w") as f:
            yaml.dump(proposal, f, default_flow_style=False, sort_keys=False, allow_unicode=True)

        logger.info(f"FOIS: wrote proposal {opp.proposal_id} → {path}")

    def _generate_rules(self, opp: CropOpportunity, harvest_date: str) -> list[dict]:
        """
        Generate operator-schema-compliant irrigation rules for this crop/zone.
        These are the rules the governance layer will validate.
        """
        zone = opp.zone_id
        crop_lower = opp.crop.lower()
        expires = f"{harvest_date}T23:59:00Z"
        pid = opp.proposal_id

        # Flow rate estimate from daily volume and 2x irrigation windows
        flow_rate = round(opp.daily_water_litres / (2 * 30 * 60), 1)  # L/min over 2x 30-min windows
        duration = 30

        rules = [
            {
                "rule_id": f"irrigate_{zone}_morning_{crop_lower}",
                "description": f"Morning irrigation for {opp.crop} in {zone} — {pid}",
                "enabled": True,
                "source": "fois",
                "proposal_id": pid,
                "priority": 70,
                "expires_at": expires,
                "trigger": {
                    "event_type": f"E_ZONE_{zone.upper()}_MOISTURE_LOW",
                    "zone_id": zone,
                },
                "conditions": [
                    {"sensor": f"{zone}_soil_moisture", "operator": "lt", "value": 45, "unit": "percent"},
                    {"sensor": "tank_1_level", "operator": "gt", "value": 20, "unit": "percent"},
                ],
                "actions": [
                    {
                        "action_type": "open_valve",
                        "target": f"{zone}_valve",
                        "params": {"duration_minutes": duration, "flow_rate_lpm": flow_rate},
                    }
                ],
            },
            {
                "rule_id": f"irrigate_{zone}_evening_{crop_lower}",
                "description": f"Evening top-up for {opp.crop} in {zone} — {pid}",
                "enabled": True,
                "source": "fois",
                "proposal_id": pid,
                "priority": 70,
                "expires_at": expires,
                "trigger": {
                    "event_type": f"E_ZONE_{zone.upper()}_MOISTURE_LOW",
                    "zone_id": zone,
                },
                "conditions": [
                    {"sensor": f"{zone}_soil_moisture", "operator": "lt", "value": 40, "unit": "percent"},
                    {"sensor": "tank_1_level", "operator": "gt", "value": 25, "unit": "percent"},
                ],
                "actions": [
                    {
                        "action_type": "open_valve",
                        "target": f"{zone}_valve",
                        "params": {"duration_minutes": 15, "flow_rate_lpm": flow_rate},
                    }
                ],
            },
            {
                "rule_id": f"heat_protect_{zone}_{crop_lower}",
                "description": f"Close valve during heat stress for {opp.crop} in {zone} — {pid}",
                "enabled": True,
                "source": "fois",
                "proposal_id": pid,
                "priority": 65,
                "expires_at": expires,
                "trigger": {
                    "event_type": "E_HEAT_STRESS_WARNING",
                    "zone_id": zone,
                },
                "conditions": [
                    {"sensor": "weather_station_temp", "operator": "gt", "value": 36, "unit": "celsius"},
                ],
                "actions": [
                    {"action_type": "close_valve", "target": f"{zone}_valve"},
                    {
                        "action_type": "log_only",
                        "target": f"{zone}_valve",
                        "params": {"message": f"Irrigation suspended — heat stress, {opp.crop}"},
                    },
                ],
            },
        ]

        return rules

    # ------------------------------------------------------------------
    # Run logging
    # ------------------------------------------------------------------

    def _log_run(self, opportunities: list[CropOpportunity]):
        conn = sqlite3.connect(self.db_path)
        try:
            for opp in opportunities:
                conn.execute("""
                    INSERT INTO fois_run_log (
                        proposal_id, generated_at, crop, zone_id,
                        p50_net_profit, confidence, risk_level, model_version
                    ) VALUES (?,?,?,?,?,?,?,?)
                """, (
                    opp.proposal_id, opp.generated_at, opp.crop, opp.zone_id,
                    opp.p50_net_profit, opp.confidence, opp.risk_level, opp.model_version,
                ))
            conn.commit()
        finally:
            conn.close()

    def _ensure_tables(self):
        conn = sqlite3.connect(self.db_path)
        try:
            conn.executescript("""
                CREATE TABLE IF NOT EXISTS fois_zones (
                    zone_id TEXT PRIMARY KEY,
                    area_sqm REAL,
                    soil_ph REAL,
                    soil_ec REAL,
                    soil_salinity_risk TEXT,
                    current_crop TEXT,
                    last_crop TEXT,
                    last_planted TEXT,
                    last_harvested TEXT,
                    updated_at TEXT
                );

                CREATE TABLE IF NOT EXISTS fois_run_log (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    proposal_id TEXT,
                    generated_at TEXT,
                    crop TEXT,
                    zone_id TEXT,
                    p50_net_profit REAL,
                    confidence REAL,
                    risk_level TEXT,
                    model_version TEXT
                );
            """)
            conn.commit()
        finally:
            conn.close()
