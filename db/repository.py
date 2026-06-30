from farm.db.database import execute, fetchall, fetchone


# ----------------------------
# RAW TELEMETRY
# ----------------------------
def insert_raw(asset_id, metric, value, confidence, quality, ts):
    execute("""
        INSERT INTO raw_telemetry
        (ts, asset_id, metric, value, confidence, quality)
        VALUES (?, ?, ?, ?, ?, ?)
    """, (ts, asset_id, metric, value, confidence, quality))


# ----------------------------
# LATEST STATE (UPSERT)
# ----------------------------
def upsert_latest(asset_id, metric, value, confidence, quality, ts):
    execute("""
        INSERT INTO latest_values
        (asset_id, metric, value, confidence, quality, ts)
        VALUES (?, ?, ?, ?, ?, ?)
        ON CONFLICT(asset_id, metric) DO UPDATE SET
            value=excluded.value,
            confidence=excluded.confidence,
            quality=excluded.quality,
            ts=excluded.ts
    """, (asset_id, metric, value, confidence, quality, ts))


# ----------------------------
# READ LATEST SNAPSHOT
# ----------------------------
def get_latest_all():
    return fetchall("""
        SELECT *
        FROM latest_values
    """)


def get_latest_asset(asset_id):
    return fetchall("""
        SELECT *
        FROM latest_values
        WHERE asset_id = ?
    """, (asset_id,))


# ----------------------------
# RAW HISTORY (for debugging / analysis)
# ----------------------------
def get_raw_asset(asset_id, limit=100):
    return fetchall("""
        SELECT *
        FROM raw_telemetry
        WHERE asset_id = ?
        ORDER BY ts DESC
        LIMIT ?
    """, (asset_id, limit))


# ----------------------------
# SIMPLE TIME RANGE QUERY (for LLM / engine)
# ----------------------------
def get_metrics(asset_id, metric, since_ts):
    return fetchall("""
        SELECT ts, value, confidence, quality
        FROM raw_telemetry
        WHERE asset_id = ?
          AND metric = ?
          AND ts >= ?
        ORDER BY ts ASC
    """, (asset_id, metric, since_ts))