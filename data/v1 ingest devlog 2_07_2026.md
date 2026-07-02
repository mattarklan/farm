# Ingestion Project — Handoff Summary

Status: **Complete and tested end-to-end.** This project owns `farm.db` exclusively.
Everything else (core, FOIS, GUI) must read/write through the API — never touch
`farm.db` directly.

## Final file structure

```
farm/
  api/
    api.py                  # FastAPI app — the ONLY sanctioned access point to farm.db
  config/
    loader.py                # canonical registry loader — reads farm/config/registry/*.yaml
    registry/
      water.yaml              # tanks, header tank, bores
      vehicles.yaml            # tractor
      crops.yaml                # growing zones
      power.yaml                  # battery
  data/
    database.py               # sqlite connection helpers (WAL mode), DB_PATH is module-relative
    repository.py             # the ONLY file allowed to contain SQL
    migrate.py                # schema_version-based migration runner
    migrations/
      001_init.sql             # raw_telemetry, latest_values
      002_add_events.sql        # events table
      003_add_summaries.sql      # hourly_summary, daily_summary
    ingest.py                 # MQTT subscriber -> writes raw + latest via repository.py
    simulator.py               # MQTT publisher — fakes sensor data for testing (NOT for prod)
    aggregator.py              # hourly/daily rollups + 90-day raw_telemetry retention purge
    farm.db                   # sole owner: this project. nobody else opens this file.
```

Deleted/superseded during this project: old `farm/db/` folder (renamed to `farm/data/`),
`devices.yaml` (replaced by the 4 registry YAMLs), `schema.json` (replaced by SQL migrations),
`snapshot.py` (replaced by `aggregator.py` + `/trends/*` API endpoints — the old 5-minute
rising/falling/stable trend logic was judged not useful and dropped).

## Schema (current, confirmed working)

```sql
raw_telemetry(ts INTEGER, asset_id TEXT, metric TEXT, value REAL, confidence REAL)
  -- indexed on (asset_id, metric, ts)
  -- rows older than 90 days are purged daily by aggregator.py

latest_values(asset_id TEXT, metric TEXT, value REAL, confidence REAL, ts INTEGER,
               PRIMARY KEY(asset_id, metric))
  -- one row per asset+metric, upserted on every message

events(id INTEGER PRIMARY KEY AUTOINCREMENT, ts INTEGER, asset_id TEXT,
       event_type TEXT, payload TEXT)
  -- PLACEHOLDER SHAPE ONLY. Ingestion does not generate events — core does.
  -- Ingestion just stores whatever core sends via POST /events.
  -- Real shape/fields still TBD pending core's events.py vs events_engine.py cleanup.

hourly_summary(hour INTEGER, asset_id TEXT, metric TEXT, min REAL, max REAL,
                avg REAL, count INTEGER, PRIMARY KEY(hour, asset_id, metric))
  -- rolled up once per hour from raw_telemetry, hour boundaries = UTC epoch hour

daily_summary(day INTEGER, asset_id TEXT, metric TEXT, min REAL, max REAL,
               avg REAL, count INTEGER, PRIMARY KEY(day, asset_id, metric))
  -- rolled up once per day from raw_telemetry
  -- day boundary = LOCAL MIDNIGHT, AWST (UTC+8, no DST), NOT UTC midnight
  -- confirmed via testing: day_start lands on e.g. "Wed Jul 1 00:00:00 2026" local time
```

Key conventions locked in:
- **Sensor naming**: every sensor in the registry YAMLs is named `{asset_id}_{metric}`
  (e.g. `tank_1_level`). The loader strips the `{asset_id}_` prefix to get the bare
  metric name (`level`) before it ever reaches the DB. If a sensor name doesn't start
  with its parent asset's id, the loader silently keeps it as-is rather than failing —
  this bit us once already (`header_tank_1` had sensors prefixed `header_1_` instead of
  `header_tank_1_`, fixed by renaming in the YAML). Worth having core's team know this
  fallback exists in case a future registry edit trips it again.
- **confidence**: currently a static placeholder from the ingest side (`DEFAULT_CONFIDENCE
  = 1.0`), and simulator fakes a random plausible-looking value. Real confidence scoring
  (e.g. detecting a degrading/miscalibrated sensor) is an unsolved problem — it likely
  needs cross-referencing against other sensors or historical baselines, not something a
  sensor can self-report. Not solved, not urgent, flagged for whoever eventually owns
  sensor-health logic (possibly core, possibly a future dedicated piece).
- **quality**: dropped entirely. It was going to be a derived label ("good"/"bad") from
  confidence, decided this was redundant with confidence itself and better computed at
  read-time by a consumer if ever needed, rather than stored.
- **dependencies** (in water.yaml, e.g. tank_1 depends on bore_1/bore_2): means "this
  tank is physically supplied by this bore." Not yet consumed by any code — just data
  sitting in the registry for whenever something (probably core) wants to reason about
  supply chains, e.g. "bore_1 failed, which tanks are affected."

## API endpoints (all tested working)

- `GET /state/latest` — all assets, current values
- `GET /state/asset/{asset_id}` — one asset, 404 if not found (confirmed working)
- `GET /state/context/{asset_id}` — same as above, slightly different shape for future LLM/engine use
- `GET /trends/hourly/{asset_id}?since_ts=` — hourly_summary rows
- `GET /trends/daily/{asset_id}?since_ts=` — daily_summary rows
- `POST /events` — body `{"asset_id", "event_type", "payload": {}, "ts": optional}` — write path for core
- `GET /events?asset_id=&since_ts=` — read events back

Requires `fastapi` + `uvicorn` installed and running permanently (`uvicorn farm.api.api:app`)
alongside `ingest.py`/`aggregator.py`/Mosquitto for the system to actually be queryable —
none of these are dev-only tools, they're the standing services the whole architecture runs on.

## Testing done (all passed)

Migrations apply cleanly and idempotently (fixed a version-off-by-one bug where migration
001 was silently skipped — starting `schema_version` was seeded at 1 instead of 0). Full
MQTT round trip confirmed (simulator -> Mosquitto -> ingest -> raw_telemetry + latest_values,
verified against raw `mosquitto_sub` traffic directly). Hourly/daily rollup math confirmed
correct against known seeded values. AWST day-boundary confirmed correct via `time.ctime()`
inspection. 90-day retention purge confirmed to delete only rows older than cutoff. All API
endpoints tested via `/docs`, including the 404 path and the events post->fetch round trip.

## Known open items / not yet built

- No real hourly/daily aggregation has run on real clock boundaries yet — only tested via
  a seed script calling the rollup functions directly with fabricated timestamps. The actual
  `aggregator.py` `run()` loop (sleeps, wakes hourly/daily) hasn't been soak-tested over real time.
- No supervisor/process-management setup yet — currently everything is run manually in
  separate terminal windows (ingest, simulator, aggregator, uvicorn, Mosquitto). Fine for
  dev, will need something more robust eventually (services, a process manager, etc.) for
  a real deployment.
- `power.yaml` only has one asset (`battery_1`) — drafted, not confirmed against real hardware.
- Confidence scoring (see above) is unsolved.
- `events` table shape is a placeholder pending core's own event-type decisions.

---

# What's needed for the `core/` chat

## Files needed from you (core's actual code)
Paste these one at a time, same as this project:
- `core/state_builder.py` — currently builds state from direct SQLite access (`sensor_store.py`);
  needs to be reworked to call `GET /state/latest` / `GET /state/context/{asset_id}` instead.
- `core/sensor_store.py` — old direct-SQLite scaffolding, expected to be deleted/gutted.
- `core/loader.py` — old registry loader, expected to be retired in favor of `farm/config/loader.py`.
- `core/events.py` **and** `core/events_engine.py` — you weren't sure which is which; both needed
  so we can figure out which one is meant to decide "this is an event" (sensor unresponsive,
  battery low, moisture low, bore running with no flow, etc.) and shape the real `events` table/
  API payload around whatever it actually produces.
- `core/event_engine.py` / `core/rule_engine.py` / `core/constraint_engine.py` / `core/executor.py`
  / `core/pipeline.py` — whichever of these exist and are relevant; even if unchanged, useful
  for context on what shape `state_builder.py`'s output needs to stay in, since downstream
  code shouldn't need to change if the swap from DB-shape to API-shape is done cleanly.
- `core/test.py` — currently seeds/reads via `sensor_store.py` directly, will need pointing at
  the API or a mocked API response instead.

## Reference material worth pasting into the new chat's first message
- This handoff document.
- The final schema block above (so nothing needs re-deriving from scratch).
- The list of API endpoints above.
- The original guideline doc's `core/ (operator)` section specifically (the bullet list about
  deleting `sensor_store.py`, retiring `core/loader.py`, `state_builder.py` changing from
  DB-rows to API-response, etc.) if you still have it — saves re-explaining the intent.

## Still-open architectural questions (not ingestion's problem, but will come up)
- Who writes `config/rules.yaml` — standalone governance, or FOIS's accepted proposals, or both,
  and how does `core/rule_engine.py` consume it. Unresolved from the original guideline.
- `farm_sales` — does this live in FOIS's own store, or in `farm.db` as an exception written by
  core's executor via `repository.py`, since "a sale happened" is arguably an operator/executor
  event rather than a FOIS concern. Unresolved.
- FOIS needs its own storage (fais.db or YAML/JSON files) since it's no longer allowed to touch
  `farm.db` — not blocking core's work, but will come up eventually.
