"""SQLite-backed store for fois.

Mirrors ingestor's storage pattern so core/GUI integration is consistent
across both services. Append-only: rows are never updated or deleted by
normal operation (see prune_older_than for retention housekeeping),
because core needs full forecast history to calibrate against later
observed values (e.g. weather offset correction).
"""

import json
import sqlite3
from contextlib import closing
from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional

from .models import Reading

DEFAULT_DB_PATH = Path(__file__).resolve().parent / "fois.sqlite3"

SCHEMA = """
CREATE TABLE IF NOT EXISTS readings (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    domain TEXT NOT NULL,
    item TEXT NOT NULL,
    region TEXT,
    value REAL NOT NULL,
    unit TEXT NOT NULL,
    currency TEXT,
    kind TEXT NOT NULL,
    valid_for TEXT,
    source TEXT NOT NULL,
    fetched_at TEXT NOT NULL,
    confidence REAL,
    raw TEXT
);

CREATE INDEX IF NOT EXISTS idx_readings_domain_item
    ON readings (domain, item, region, fetched_at);

CREATE INDEX IF NOT EXISTS idx_readings_valid_for
    ON readings (domain, item, region, valid_for);
"""


class Store:
    def __init__(self, db_path: Path = DEFAULT_DB_PATH):
        self.db_path = Path(db_path)
        with closing(self._connect()) as conn:
            conn.executescript(SCHEMA)
            conn.commit()

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        return conn

    # ---------------------------------------------------------------
    # Writes
    # ---------------------------------------------------------------

    def insert_readings(self, readings: list[Reading]) -> int:
        """Insert readings. Append-only - never updates existing rows."""
        if not readings:
            return 0
        rows = [
            (
                r.domain,
                r.item,
                r.region,
                r.value,
                r.unit,
                r.currency,
                r.kind,
                r.valid_for.isoformat() if r.valid_for else None,
                r.source,
                r.fetched_at.isoformat(),
                r.confidence,
                json.dumps(r.raw) if r.raw is not None else None,
            )
            for r in readings
        ]
        with closing(self._connect()) as conn:
            conn.executemany(
                """
                INSERT INTO readings
                    (domain, item, region, value, unit, currency, kind,
                     valid_for, source, fetched_at, confidence, raw)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                rows,
            )
            conn.commit()
        return len(rows)

    def prune_older_than(self, days: int = 90) -> int:
        """Housekeeping: delete readings older than N days. Not called automatically."""
        cutoff = (datetime.utcnow() - timedelta(days=days)).isoformat()
        with closing(self._connect()) as conn:
            cur = conn.execute("DELETE FROM readings WHERE fetched_at < ?", (cutoff,))
            conn.commit()
            return cur.rowcount

    # ---------------------------------------------------------------
    # Reads
    # ---------------------------------------------------------------

    def get_latest(self, domain: str, region: Optional[str] = None) -> list[sqlite3.Row]:
        """Latest reading per (item, source). For multi-supplier domains this
        deliberately returns one row per supplier, not one row per item -
        different suppliers' prices are different facts, not fallbacks for
        each other, so neither should clobber the other. NOT ordered by
        raw value here - bag sizes differ across suppliers, so raw price
        isn't a meaningful "cheapest" order. See api.py, which sorts by
        normalised price-per-m3/tonne instead."""
        query = """
            SELECT r.* FROM readings r
            INNER JOIN (
                SELECT item, source, MAX(fetched_at) AS max_fetched
                FROM readings
                WHERE domain = ? AND (? IS NULL OR region = ?)
                GROUP BY item, source
            ) latest
            ON r.item = latest.item AND r.source = latest.source
                AND r.fetched_at = latest.max_fetched
            WHERE r.domain = ? AND (? IS NULL OR r.region = ?)
        """
        params = (domain, region, region, domain, region, region)
        with closing(self._connect()) as conn:
            return conn.execute(query, params).fetchall()

    def get_latest_for_item(self, domain: str, item: str, region: Optional[str] = None) -> list[sqlite3.Row]:
        """Latest reading per source for a single item - i.e. every
        supplier's current price for this item. NOT ordered by raw value
        here for the same reason as get_latest() above - see api.py."""
        query = """
            SELECT r.* FROM readings r
            INNER JOIN (
                SELECT source, MAX(fetched_at) AS max_fetched
                FROM readings
                WHERE domain = ? AND item = ? AND (? IS NULL OR region = ?)
                GROUP BY source
            ) latest
            ON r.source = latest.source AND r.fetched_at = latest.max_fetched
            WHERE r.domain = ? AND r.item = ? AND (? IS NULL OR r.region = ?)
        """
        params = (domain, item, region, region, domain, item, region, region)
        with closing(self._connect()) as conn:
            return conn.execute(query, params).fetchall()

    def get_forecast(self, domain: str, region: Optional[str], days: int) -> list[sqlite3.Row]:
        """Most recent forecast fetch, for each valid_for date within the window,
        for a forecast-type domain (e.g. weather)."""
        today = datetime.utcnow().date().isoformat()
        horizon = (datetime.utcnow().date() + timedelta(days=days)).isoformat()
        query = """
            SELECT r.* FROM readings r
            INNER JOIN (
                SELECT item, valid_for, MAX(fetched_at) AS max_fetched
                FROM readings
                WHERE domain = ? AND kind = 'forecast'
                    AND (? IS NULL OR region = ?)
                    AND valid_for BETWEEN ? AND ?
                GROUP BY item, valid_for
            ) latest
            ON r.item = latest.item
                AND r.valid_for = latest.valid_for
                AND r.fetched_at = latest.max_fetched
            WHERE r.domain = ? AND (? IS NULL OR r.region = ?)
            ORDER BY r.valid_for ASC
        """
        params = (domain, region, region, today, horizon, domain, region, region)
        with closing(self._connect()) as conn:
            return conn.execute(query, params).fetchall()
