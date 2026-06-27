import sqlite3
import json

from pathlib import Path

from .schema import SCHEMA


class Database:

    def __init__(self, filename="farm.db"):

        self.path = Path(filename)

        self.conn = sqlite3.connect(self.path)

        self.conn.row_factory = sqlite3.Row

        self.conn.executescript(SCHEMA)

        self.conn.commit()


    def close(self):
        self.conn.close()


    # -----------------------------
    # SENSOR REGISTRY
    # -----------------------------

    def register_sensor(
            self,
            sensor_id,
            asset_id,
            sensor_type,
            units="",
            expected_interval=60,
            stale_after=300):

        self.conn.execute(

            """
            INSERT OR REPLACE INTO sensors
            VALUES (?, ?, ?, ?, ?, ?, 1)
            """,

            (
                sensor_id,
                asset_id,
                sensor_type,
                units,
                expected_interval,
                stale_after,
            ),
        )

        self.conn.commit()


    # -----------------------------
    # SENSOR VALUES
    # -----------------------------

    def insert_sensor_reading(

            self,
            timestamp,
            sensor_id,
            value,
            quality="GOOD",
            source="SIM",
            sequence=None):

        self.conn.execute(

            """
            INSERT INTO sensor_readings
            (
                timestamp,
                sensor_id,
                value,
                quality,
                source,
                sequence
            )

            VALUES (?, ?, ?, ?, ?, ?)
            """,

            (
                timestamp,
                sensor_id,
                value,
                quality,
                source,
                sequence,
            ),
        )

        self.conn.commit()


    # -----------------------------
    # LATEST VALUE
    # -----------------------------

    def latest_reading(self, sensor_id):

        row = self.conn.execute(

            """
            SELECT *

            FROM sensor_readings

            WHERE sensor_id=?

            ORDER BY timestamp DESC

            LIMIT 1
            """,

            (sensor_id,),

        ).fetchone()

        return dict(row) if row else None


    # -----------------------------
    # ALL LATEST VALUES
    # -----------------------------

    def latest_readings(self):

        rows = self.conn.execute(

            """
            SELECT r.*

            FROM sensor_readings r

            JOIN
            (
                SELECT
                    sensor_id,
                    MAX(timestamp) latest

                FROM sensor_readings

                GROUP BY sensor_id

            ) latest

            ON r.sensor_id=latest.sensor_id

            AND r.timestamp=latest.latest
            """

        ).fetchall()

        return [dict(r) for r in rows]


    # -----------------------------
    # HISTORY
    # -----------------------------

    def history(self, sensor_id, limit=100):

        rows = self.conn.execute(

            """
            SELECT *

            FROM sensor_readings

            WHERE sensor_id=?

            ORDER BY timestamp DESC

            LIMIT ?
            """,

            (
                sensor_id,
                limit,
            ),

        ).fetchall()

        return [dict(r) for r in rows]