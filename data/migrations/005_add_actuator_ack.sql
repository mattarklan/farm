
CREATE TABLE IF NOT EXISTS actuator_ack (
    asset_id TEXT NOT NULL,
    metric TEXT NOT NULL,       -- 'last_ack_ts' or 'ack_status'
    value TEXT,
    confidence REAL,
    ts INTEGER NOT NULL,
    PRIMARY KEY (asset_id, metric)
);

CREATE INDEX IF NOT EXISTS idx_actuator_ack_asset
    ON actuator_ack (asset_id);
