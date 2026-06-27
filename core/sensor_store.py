import sqlite3
from datetime import datetime

class SensorStore:
    def __init__(self, db_path: str = "farm.db"):
        self.conn = sqlite3.connect(db_path)
        self._init_db()

    def _init_db(self):
        self.conn.execute("""
            CREATE TABLE IF NOT EXISTS sensor_readings (
                id        INTEGER PRIMARY KEY AUTOINCREMENT,
                sensor_id TEXT    NOT NULL,
                value     REAL    NOT NULL,
                timestamp TEXT    NOT NULL
            )
        """)
        self.conn.commit()

    def latest_readings(self):
        cursor = self.conn.execute("""
            SELECT sensor_id, value, timestamp
            FROM sensor_readings
            WHERE id IN (
                SELECT MAX(id) FROM sensor_readings GROUP BY sensor_id
            )
        """)
        rows = cursor.fetchall()
        return [
            {
                "sensor_id": row[0],
                "value": row[1],
                "quality": 1.0,
                "timestamp": datetime.fromisoformat(row[2]),
                "source": "db"
            }
            for row in rows
        ]

    def record(self, sensor_id: str, value: float, timestamp: datetime):
        self.conn.execute(
            "INSERT INTO sensor_readings (sensor_id, value, timestamp) VALUES (?, ?, ?)",
            (sensor_id, value, timestamp.isoformat())
        )
        self.conn.commit()

    def close(self):
        self.conn.close()