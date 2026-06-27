SCHEMA = """

PRAGMA journal_mode=WAL;

CREATE TABLE IF NOT EXISTS sensors (

    sensor_id TEXT PRIMARY KEY,

    asset_id TEXT NOT NULL,

    sensor_type TEXT NOT NULL,

    units TEXT,

    expected_interval INTEGER,

    stale_after INTEGER,

    enabled INTEGER DEFAULT 1

);


CREATE TABLE IF NOT EXISTS sensor_readings (

    id INTEGER PRIMARY KEY AUTOINCREMENT,

    timestamp TEXT NOT NULL,

    sensor_id TEXT NOT NULL,

    value REAL,

    quality TEXT DEFAULT 'GOOD',

    source TEXT,

    sequence INTEGER,

    FOREIGN KEY(sensor_id) REFERENCES sensors(sensor_id)

);


CREATE INDEX IF NOT EXISTS idx_sensor_time
ON sensor_readings(sensor_id, timestamp);



CREATE TABLE IF NOT EXISTS events (

    id INTEGER PRIMARY KEY AUTOINCREMENT,

    timestamp TEXT,

    event_type TEXT,

    asset_id TEXT,

    sensor_id TEXT,

    severity INTEGER,

    confidence REAL,

    json_data TEXT

);


CREATE TABLE IF NOT EXISTS actions (

    id INTEGER PRIMARY KEY AUTOINCREMENT,

    timestamp TEXT,

    action_type TEXT,

    target TEXT,

    parameters TEXT,

    approved INTEGER

);


CREATE TABLE IF NOT EXISTS executions (

    id INTEGER PRIMARY KEY AUTOINCREMENT,

    timestamp TEXT,

    action_type TEXT,

    success INTEGER,

    message TEXT

);


CREATE TABLE IF NOT EXISTS system_cycles (

    cycle_id INTEGER PRIMARY KEY AUTOINCREMENT,

    timestamp TEXT,

    duration_ms REAL,

    event_count INTEGER,

    action_count INTEGER,

    blocked_count INTEGER

);

"""