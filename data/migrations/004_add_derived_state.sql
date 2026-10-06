-- Derived / inferred facts (e.g. vehicle_status, vehicle_hours, last_visit)
-- computed by core from raw sensor history — never written by the simulator
-- or ingestion directly, and deliberately kept OUT of raw_telemetry so the
-- hourly/daily rollups in aggregator.py never have to deal with non-numeric
-- or non-sensor values.
CREATE TABLE IF NOT EXISTS derived_state (
    asset_id TEXT NOT NULL,
    metric TEXT NOT NULL,
    value TEXT,
    ts INTEGER NOT NULL,
    computed_by TEXT DEFAULT 'core',
    PRIMARY KEY (asset_id, metric)
);

CREATE INDEX IF NOT EXISTS idx_derived_state_asset
    ON derived_state (asset_id);
