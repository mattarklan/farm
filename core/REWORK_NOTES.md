# core/ rework notes

## What's in this folder

Verified importable and runnable — `python3 test.py` (from one level above
`core/`, i.e. `PYTHONPATH=. python3 core/test.py`) passes.

| File | Status |
|---|---|
| `models.py` | unchanged |
| `api_client.py` | **new** — talks to the ingestion API |
| `state_builder.py` | **reworked** — builds state from the API, not SQLite |
| `event_engine.py` | unchanged |
| `rule_engine.py` | unchanged |
| `constraint_engine.py` | unchanged (see open issue below) |
| `executor.py` | unchanged |
| `decision_memory.py` | unchanged (deduped — you'd pasted it twice) |
| `window_store.py` | unchanged |
| `derived_engine.py` | **new** — nothing you pasted defined this, but `pipeline.py` depends on it |
| `pipeline.py` | one bug fixed (below) |
| `logger.py` | fixed — was importing from the binned modules |
| `main.py` | **new** — wiring/entrypoint, replaces `engine.py`'s job |
| `test.py` | **new** — your old one wasn't part of what you pasted |

## Binned (not carried over)

- `engine.py`, `events.py`, `rules.py`, `sensor_store.py`

These formed a second, older orchestrator that doesn't match the handoff
doc's architecture: dataclass-style `FarmState`, `EventEngine.process(state)`
with no `derived` argument, `ConstraintEngine.process()` / `Executor.process()`
(different method names to the ones `pipeline.py` actually calls). The
handoff doc explicitly names `event_engine.py` / `rule_engine.py` /
`constraint_engine.py` / `executor.py` / `pipeline.py` as the real,
unaffected-by-the-API-swap system, and `pipeline.py`'s method calls only
line up with that set — so that's what this rework is built on.

- `core/loader.py` — per the handoff doc, retired in favor of the
  ingestion project's canonical loader.
- `core/state.py` (`SystemState` dataclass) — unused by the real pipeline;
  `state` is a plain dict everywhere it's actually consumed.

## Bugs fixed along the way (not just moved)

1. **`pipeline.py`**: `decision_memory.record_action()` was being called on
   `executor.execute()`'s return value (`ActionResult`: `action_id`,
   `success`, `message`, `timestamp`) but `record_action()` reads
   `action.action_type` / `action.target`, which only exist on `Action`.
   This would've raised `AttributeError` the first time any action actually
   executed. Now records from `allowed` (the `Action` list) instead.

2. **`logger.py`**: imported `core.events.Event` / `core.rules.Action` —
   the binned legacy types — instead of `core.models`. Also called
   `asdict(state)`, which requires a dataclass instance; `state` is a plain
   dict everywhere else. Both fixed.

3. **`state_builder.py`**: added a fallback to "now" when the API doesn't
   supply a sensor timestamp, since `window_store.append()` crashes on
   `None`.

## Flagged, not fixed — needs your input

1. **`constraint_engine._check_raw()` reads `state.get("water", {})` /
   `state.get("energy", {})`** — a nested-by-domain shape. The actual
   `state` dict (from `state_builder.py`, and what `event_engine.py`
   consumes) is flat: `state["sensors"]["tank_1_level"]`, etc. As written,
   `_check_raw` will never find those keys and will silently never fire on
   raw-state violations — it'll just fall through to the risk model. I
   didn't rewrite this because I don't know what shape you actually want
   raw constraint conditions to check against (e.g. `condition: {"sensor":
   "battery_1_soc", "op": "lt", "value": 20}` would be a natural fit for
   the flat shape, but that's a guess). Left as-is; flagging rather than
   guessing.

2. **`api_client.py` / `state_builder.py` assume `GET /state/latest`
   returns `{asset_id: {metric: {value, confidence, ts}}}`.** Not
   confirmed against the real response — the handoff doc only describes
   it as "all assets, current values." If the real shape differs, only
   `StateBuilder.build()` needs to change.

3. **`main.py` imports `farm.config.loader.RegistryLoader`** on the
   assumption it returns something with `.get_asset_from_sensor(sensor_id)
   -> Asset(.id, .type)` — matching the deleted `core/loader.py`'s
   interface, since that's what `event_engine.py` needs. Never seen the
   real file, so unverified.

4. **`ConstraintEngine`'s config vs. `RuleEngine`'s config** — kept as two
   separate files (`config/constraints.yaml`, `config/rules.yaml`) in
   `main.py` rather than guessing they share one file. The handoff doc
   explicitly leaves "who writes `config/rules.yaml`" unresolved — this is
   the same open thread, just visible from core's side too.

5. **`SensorState.quality`** now holds the API's `confidence` value (the
   ingestion side dropped `quality` entirely as redundant with
   `confidence`). Reusing the field name that already existed rather than
   adding a new one — worth revisiting if `quality` and `confidence` are
   meant to diverge later.
