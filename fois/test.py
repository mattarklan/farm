"""
fois/test.py

Manual integration test for the full FOIS pipeline.

Run from the farm/ directory:
    python -m fois.test

What this does:
    1. Seeds the DB with minimal zone and sensor data
    2. Runs the FOIS engine (skips real BOM fetch — uses fallback)
    3. Prints ranked opportunities
    4. Runs governance on any proposals written
    5. Prints verdicts

This mirrors what core/test.py does for the operator.
"""

import sys
import logging
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("fois.test")

DB_PATH = "farm.db"


def seed_db():
    """Seed minimal data so the engine has something to work with."""
    conn = sqlite3.connect(DB_PATH)

    # Ensure sensor_readings table exists (operator owns this, but we need it)
    conn.executescript("""
        CREATE TABLE IF NOT EXISTS sensor_readings (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            sensor_id TEXT NOT NULL,
            value REAL NOT NULL,
            timestamp TEXT NOT NULL
        );
    """)

    now = datetime.now(timezone.utc).isoformat()

    # Seed soil moisture readings for each zone
    zone_sensor_data = [
        ("zone_capsicum_a_soil_moisture", 38.0),
        ("zone_capsicum_b_soil_moisture", 55.0),
        ("zone_tomato_a_soil_moisture", 42.0),
        ("zone_tomato_b_soil_moisture", 60.0),
        ("tank_1_level", 72.0),
        ("battery_soc", 85.0),
        ("weather_station_temp", 24.5),
    ]

    for sensor_id, value in zone_sensor_data:
        conn.execute(
            "INSERT INTO sensor_readings (sensor_id, value, timestamp) VALUES (?,?,?)",
            (sensor_id, value, now)
        )

    # Seed fois_zones with the two known growing areas
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
    """)

    zones = [
        ("zone_capsicum_a", 500.0, 6.4, 0.8, "low",  None,       "Tomato",   "2025-09-01", "2026-01-15"),
        ("zone_capsicum_b", 500.0, 6.6, 0.7, "low",  None,       "Zucchini", "2025-11-01", "2026-02-28"),
        ("zone_tomato_a",   600.0, 6.2, 1.1, "low",  None,       "Capsicum", "2025-08-15", "2025-12-20"),
        ("zone_tomato_b",   600.0, 6.8, 0.9, "low",  None,       None,       None,          None),
    ]

    for zone in zones:
        conn.execute("""
            INSERT OR REPLACE INTO fois_zones
                (zone_id, area_sqm, soil_ph, soil_ec, soil_salinity_risk,
                 current_crop, last_crop, last_planted, last_harvested, updated_at)
            VALUES (?,?,?,?,?,?,?,?,?,?)
        """, (*zone, now))

    conn.commit()
    conn.close()
    logger.info("Seed data written to DB")


def run_engine():
    from fois.engine import FoisEngine
    engine = FoisEngine(db_path=DB_PATH, top_n=3)
    opportunities = engine.run()
    return opportunities


def run_governance():
    from fois.governance import GovernanceLayer

    mock_state = {
        "sensors": {
            "tank_1_level": 72.0,
            "battery_soc": 85.0,
        },
        "maintenance_window_active": False,
    }

    governor = GovernanceLayer(
        schema_path="config/rule_schema.yaml",
        state=mock_state,
        active_proposals=[],
    )
    return governor.evaluate_all_pending()


def print_opportunities(opportunities):
    print("\n" + "="*60)
    print("FOIS — RANKED CROP OPPORTUNITIES")
    print("="*60)

    if not opportunities:
        print("  No opportunities generated.")
        return

    for i, opp in enumerate(opportunities):
        print(f"\n  #{i+1}  {opp.crop} ({opp.cultivar})  →  {opp.zone_id}")
        print(f"       Plant: {opp.planting_date}   Harvest: {opp.harvest_date}")
        print(f"       Profit P10/P50/P90: ${opp.p10_net_profit:,.0f} / "
              f"${opp.p50_net_profit:,.0f} / ${opp.p90_net_profit:,.0f}")
        print(f"       Price/kg  P50: ${opp.price_p50:.2f}  (confidence: {opp.price_confidence:.0%})")
        print(f"       Water: {opp.daily_water_litres:,.0f} L/day   "
              f"Power: {opp.daily_power_kwh:.1f} kWh/day")
        print(f"       Risk: {opp.risk_level}   Confidence: {opp.confidence:.0%}")
        print(f"       Utility score: {opp.utility_score():,.0f}")
        if opp.reasons:
            print(f"       Reasons:")
            for r in opp.reasons:
                print(f"         · {r}")
        if opp.warnings:
            print(f"       Warnings:")
            for w in opp.warnings:
                print(f"         ⚠ {w}")


def print_verdicts(results):
    print("\n" + "="*60)
    print("GOVERNANCE — VERDICTS")
    print("="*60)

    if not results:
        print("  No proposals evaluated.")
        return

    for r in results:
        status = "✓ ACCEPTED" if r.accepted else f"✗ REJECTED (at {r.gate})"
        print(f"\n  {r.proposal_id}: {status}")
        for reason in r.reasons:
            print(f"    · {reason}")


if __name__ == "__main__":
    print("\nFOIS integration test")
    print("─" * 40)

    print("\n[1] Seeding DB...")
    seed_db()

    print("\n[2] Running FOIS engine...")
    opportunities = run_engine()
    print_opportunities(opportunities)

    print("\n[3] Running governance on pending proposals...")
    verdicts = run_governance()
    print_verdicts(verdicts)

    print("\nDone.\n")
