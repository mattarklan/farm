CREATE TABLE IF NOT EXISTS hourly_summary (
    hour INTEGER NOT NULL,
    asset_id TEXT NOT NULL,
    metric TEXT NOT NULL,
    min REAL,
    max REAL,
    avg REAL,
    count INTEGER,
    PRIMARY KEY (hour, asset_id, metric)
);

CREATE INDEX IF NOT EXISTS idx_hourly_summary_asset_metric
    ON hourly_summary (asset_id, metric, hour);

CREATE TABLE IF NOT EXISTS daily_summary (
    day INTEGER NOT NULL,
    asset_id TEXT NOT NULL,
    metric TEXT NOT NULL,
    min REAL,
    max REAL,
    avg REAL,
    count INTEGER,
    PRIMARY KEY (day, asset_id, metric)
);

CREATE INDEX IF NOT EXISTS idx_daily_summary_asset_metric
    ON daily_summary (asset_id, metric, day);