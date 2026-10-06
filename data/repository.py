from farm.data.database import execute, fetchall, fetchone

def insert_raw(asset_id, metric, value, confidence, ts):
    execute("""
        INSERT INTO raw_telemetry (ts, asset_id, metric, value, confidence)
        VALUES (?, ?, ?, ?, ?)
    """, (ts, asset_id, metric, value, confidence))

def upsert_latest(asset_id, metric, value, confidence, ts):
    execute("""
        INSERT INTO latest_values (asset_id, metric, value, confidence, ts)
        VALUES (?, ?, ?, ?, ?)
        ON CONFLICT(asset_id, metric) DO UPDATE SET
            value=excluded.value,
            confidence=excluded.confidence,
            ts=excluded.ts
    """, (asset_id, metric, value, confidence, ts))

def get_latest_all():
    return fetchall("SELECT * FROM latest_values")

def get_latest_asset(asset_id):
    return fetchall("SELECT * FROM latest_values WHERE asset_id = ?", (asset_id,))

def get_raw_asset(asset_id, limit=100):
    return fetchall("""
        SELECT * FROM raw_telemetry
        WHERE asset_id = ? ORDER BY ts DESC LIMIT ?
    """, (asset_id, limit))

def get_metrics(asset_id, metric, since_ts):
    return fetchall("""
        SELECT ts, value, confidence FROM raw_telemetry
        WHERE asset_id = ? AND metric = ? AND ts >= ?
        ORDER BY ts ASC
    """, (asset_id, metric, since_ts))

# ----------------------------
# EVENTS (placeholder — core writes, ingestion just stores)
# shape TBD pending core/events.py vs events_engine.py cleanup
# ----------------------------
def insert_event(asset_id, event_type, payload_json, ts):
    execute("""
        INSERT INTO events (ts, asset_id, event_type, payload)
        VALUES (?, ?, ?, ?)
    """, (ts, asset_id, event_type, payload_json))

def get_events(asset_id=None, since_ts=0):
    if asset_id:
        return fetchall("""
            SELECT * FROM events WHERE asset_id = ? AND ts >= ? ORDER BY ts ASC
        """, (asset_id, since_ts))
    return fetchall("SELECT * FROM events WHERE ts >= ? ORDER BY ts ASC", (since_ts,))

# ----------------------------
# HOURLY / DAILY SUMMARIES
# ----------------------------
def compute_hourly_summary(hour_start, hour_end):
    return fetchall("""
        SELECT asset_id, metric, MIN(value) as min, MAX(value) as max,
               AVG(value) as avg, COUNT(*) as count
        FROM raw_telemetry
        WHERE ts >= ? AND ts < ?
        GROUP BY asset_id, metric
    """, (hour_start, hour_end))

def upsert_hourly_summary(hour, asset_id, metric, min_v, max_v, avg_v, count):
    execute("""
        INSERT INTO hourly_summary (hour, asset_id, metric, min, max, avg, count)
        VALUES (?, ?, ?, ?, ?, ?, ?)
        ON CONFLICT(hour, asset_id, metric) DO UPDATE SET
            min=excluded.min, max=excluded.max, avg=excluded.avg, count=excluded.count
    """, (hour, asset_id, metric, min_v, max_v, avg_v, count))

def compute_daily_summary(day_start, day_end):
    return fetchall("""
        SELECT asset_id, metric, MIN(value) as min, MAX(value) as max,
               AVG(value) as avg, COUNT(*) as count
        FROM raw_telemetry
        WHERE ts >= ? AND ts < ?
        GROUP BY asset_id, metric
    """, (day_start, day_end))

def upsert_daily_summary(day, asset_id, metric, min_v, max_v, avg_v, count):
    execute("""
        INSERT INTO daily_summary (day, asset_id, metric, min, max, avg, count)
        VALUES (?, ?, ?, ?, ?, ?, ?)
        ON CONFLICT(day, asset_id, metric) DO UPDATE SET
            min=excluded.min, max=excluded.max, avg=excluded.avg, count=excluded.count
    """, (day, asset_id, metric, min_v, max_v, avg_v, count))

def get_hourly_summary(asset_id, since_ts=0):
    return fetchall("""
        SELECT * FROM hourly_summary WHERE asset_id=? AND hour >= ? ORDER BY hour ASC
    """, (asset_id, since_ts))

def get_daily_summary(asset_id, since_ts=0):
    return fetchall("""
        SELECT * FROM daily_summary WHERE asset_id=? AND day >= ? ORDER BY day ASC
    """, (asset_id, since_ts))

def purge_raw_before(cutoff_ts):
    execute("DELETE FROM raw_telemetry WHERE ts < ?", (cutoff_ts,))

# ----------------------------
# DERIVED STATE (written by core — vehicle_status, vehicle_hours,
# last_visit, etc. Kept separate from raw_telemetry since these aren't
# sensor readings and aren't guaranteed numeric.)
# ----------------------------
def upsert_derived_state(asset_id, metric, value, ts, computed_by="core"):
    execute("""
        INSERT INTO derived_state (asset_id, metric, value, ts, computed_by)
        VALUES (?, ?, ?, ?, ?)
        ON CONFLICT(asset_id, metric) DO UPDATE SET
            value=excluded.value,
            ts=excluded.ts,
            computed_by=excluded.computed_by
    """, (asset_id, metric, value, ts, computed_by))

def get_derived_state_asset(asset_id):
    return fetchall("SELECT * FROM derived_state WHERE asset_id = ?", (asset_id,))

def get_derived_state_all():
    return fetchall("SELECT * FROM derived_state")

# ----------------------------
# ACTUATOR ACKS (last_ack_ts, ack_status) — MCU-sourced, written by
# ingestion same as any other telemetry. NOT rolled up by aggregator.py.
# ----------------------------
def upsert_actuator_ack(asset_id, metric, value, confidence, ts):
    execute("""
        INSERT INTO actuator_ack (asset_id, metric, value, confidence, ts)
        VALUES (?, ?, ?, ?, ?)
        ON CONFLICT(asset_id, metric) DO UPDATE SET
            value=excluded.value,
            confidence=excluded.confidence,
            ts=excluded.ts
    """, (asset_id, metric, value, confidence, ts))

def get_actuator_ack_asset(asset_id):
    return fetchall("SELECT * FROM actuator_ack WHERE asset_id = ?", (asset_id,))

def get_actuator_ack_all():
    return fetchall("SELECT * FROM actuator_ack")