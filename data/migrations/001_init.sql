CREATE TABLE IF NOT EXISTS raw_telemetry (
    ts INTEGER NOT NULL,
    asset_id TEXT NOT NULL,
    metric TEXT NOT NULL,
    value REAL,
    confidence REAL
);

CREATE INDEX IF NOT EXISTS idx_raw_telemetry_asset_metric_ts
    ON raw_telemetry (asset_id, metric, ts);

CREATE TABLE IF NOT EXISTS latest_values (
    asset_id TEXT NOT NULL,
    metric TEXT NOT NULL,
    value REAL,
    confidence REAL,
    ts INTEGER NOT NULL,
    PRIMARY KEY (asset_id, metric)
);