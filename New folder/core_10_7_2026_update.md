# Core Devlog Update — 10/07/2026

Addendum to `core_8_7_2026_update.md`. Session covered real registry data
for the first time (`loader.py` + all five `registry/*.yaml` files) --
several earlier guesses turned out wrong, corrected below rather than
quietly overwritten.

---

## 1. Fixes landed this session

- **Event ID collisions** (`event_engine.py` used the static rule id
  instead of a UUID) -- fixed.
- **Dead battery-risk check** (`constraint_engine._compute_risk` read a
  nested `state["energy"]` key that never existed) -- fixed, now reads
  real `_soc` sensors and respects confidence.
- **`window_store` timezone bug** -- `datetime.utcnow()` fed into
  `.timestamp()` was silently defeating the 1h retention window on an
  AWST box. Fixed to explicit `timezone.utc`.
- **`ACTION_BLOCKED` events** now carry the real triggering sensor/value
  (e.g. actual battery voltage), not just a static reason string.
- **Per-target constraint scoping** added (`constraints.yaml` rules can
  now apply to one specific target, not just one action_type globally).
- **Tank_1 overflow constraint** added.
- **Every `action_type` has a defined fallback** (`action_types.yaml`),
  default "retry after 5 min," some overridden shorter for off-actions.
- **Heartbeat write** added to `main.py`'s loop (`logs/heartbeat.txt`),
  per `PROCESS_SUPERVISION.md`.
- **Ack/faulty fault classification** built: `action_confirmation.py`
  (new) tracks whether a goal-start action got a hardware ack; `pipeline.py`
  now carries anomaly/blocked events forward one cycle so `goal_engine`
  can actually see them (it couldn't before -- see mistakes below);
  `goal_engine.py` gained a `fault_event` until-condition type and
  `_schedule_fallback`, consuming `action_types.yaml`'s fallback field
  for real for the first time.
- **`rules.yaml`**: `load_shed_on_low_battery` converted from a one-shot
  rule to a goal -- it previously had no way to ever turn back off
  (`LOAD_SHEDDING` had no `stop_action_type`; added `RESTORE_LOAD`).
  Added `fill_tank_1_from_bore`, the first rule exercising the new
  fault_event/fallback machinery.
- **`loader.py`**: fixed `downstream` -- documented as "already live"
  in the earlier devlog, but never actually extracted from `asset_def`
  in `_build_registry()`. `asset.downstream` has been silently `[]`
  regardless of real registry content until this fix. No known
  functional impact yet (nothing in `core/` reads `.downstream`), but
  would have quietly broken the dependency-chain walker whenever that
  gets built. Also added `actuators` extraction, symmetric with
  `metrics`/`dependencies`.
- **Registry actuators populated for real**: every tank (`OPEN_VALVE`),
  both bores (`START_PUMP`), and both zones (`OPEN_VALVE`) in
  `water.yaml`/`crops.yaml`. `main.py`'s `force_safe_startup` (built
  last session, previously a no-op with nothing to sweep) can now
  actually do something on these seven assets.
- **Soil-temp cross-reference** -- `confidence_monitor.py` now annotates
  `SENSOR_CONFIDENCE_ZERO`/`SENSOR_VALUE_STUCK` events with
  `temp_still_responsive` (True/False/None) by checking the sibling
  `_soil_temperature` sensor's volatility. Diagnostic only, deliberately
  never restores trust in the moisture reading itself.
- **Leak detection rebuilt** (`tank_1_leak_check`, `anomaly_engine.py`'s
  new `tank_balance` type) -- the old `mass_balance` rule needed a
  `tank_1_flow_in` sensor that doesn't exist on ANY tank (confirmed
  against `water.yaml` -- no tank has an inflow sensor at all, by
  design). Rebuilt to infer "is inflow happening" from whether core has
  `bore_1` actively commanded on, instead of measuring it -- works
  specifically because bore_1's link to tank_1 is confirmed permanent.
  Scoped to tank_1 only; tank_2/3 need the tank-selection design (see
  Outstanding, below) before the same approach can extend to them.
- **Bore sensor name fixes**: `bore_1_status` and `bore_1_power_draw`
  don't exist -- real sensors are `bore_1_flow`/`bore_1_current`.
  `bore_1_dry_or_faulty` now infers "running" from current draw instead
  of a dedicated status flag that was never real.
- **`pump_1_running_no_flow` renamed to `bore_1_running_no_flow`** and
  retargeted to `bore_1` -- it was about the bore all along, `pump_1`
  never existed as a separate target.

---

## 2. Mistakes / changed my mind -- flagged explicitly, not buried

- **Guessed `pump_1` as bore_1's actuator target.** Wrong -- confirmed
  the real target is `bore_1` itself. Corrected in `constraints.yaml`
  and `anomaly_rules.yaml`.
- **Guessed `bore_2` as tank_1's fallback water source** for
  `fill_tank_1_from_bore`'s retarget fallback. Wrong -- bore_2 is
  permanently paired with a different tank elsewhere on the property,
  confirmed against `water.yaml`'s `dependencies`. Reverted to no
  rule-level fallback (falls through to the plain retry default);
  corrected the rule's comment, which had reasoned "there's no second
  bore" -- there is, it's just not this tank's.
- **Built a guessed current-draw band** (`bore_1_dry_run_current`) to
  infer dry-well running from raw amperage. Removed entirely once it
  became clear the bore's own MCU needs to own dry-cutoff detection
  itself (protecting the pump in seconds, faster than core's cycle),
  making the guess both slower and redundant with the real signal once
  it exists. Replaced with a documented interface note instead of code.
- **Dropped `mass_balance` for tank_1 in favor of a new `tank_balance`
  type** once it was confirmed no tank has (or will have) a literal
  inflow sensor -- inflow is meant to be inferred from bore state +
  rain, not measured. `mass_balance` itself wasn't deleted (kept in
  case a literal inflow sensor is ever real somewhere), but tank_1 now
  uses the new type.
- **Assumed `goal_engine`/`pipeline.py` had some feedback path for
  "did my action actually work"** in earlier turns before checking --
  it didn't. `anomaly_engine`/`constraint_engine` both run AFTER
  `goal_engine` every cycle, so a same-cycle fault couldn't reach it at
  all. Fixed by carrying events forward one cycle rather than
  reordering the pipeline (reordering health-checks-before-goal-engine
  is explicitly load-bearing per the original devlog; this doesn't
  touch that).
- **Assumed a single shared "active tank" state was the likely model**
  before asking. Actually per-zone: zones have independent tank
  assignments, tank valves can legitimately be open concurrently for
  different zones, and cross-flow between tanks is a distinct,
  deliberate operation (opening two tank valves at once), not the same
  thing as "the currently active tank." Glad I asked before building
  the wrong state machine.

---

## 3. Outstanding -- genuinely not built, needs more info or is just next

- **Tank/zone selection state machine.** This is the big one. Confirmed
  this session: each tank and each zone has its own valve actuator
  (added to the registry); bore_1 is permanently fixed to tank_1;
  cross-flow between tanks is done by opening two tanks' valves
  together; tanks 1/2/3 are "linked" with sequential fallback ("if it
  goes empty, use the next one"); multiple zones can draw from
  different tanks simultaneously. NOT yet built: any actual rule/goal
  that selects which tank supplies which zone, detects a tank going
  empty and switches to the next, or triggers cross-flow. This needs
  its own design pass, not a bolt-on to what exists -- deliberately not
  guessed at this session.
- **`event_rules.yaml`** -- still not shared. Needed to confirm the S1
  fix landed, add `TK1_TANK_LEVEL_LOW` (referenced by
  `fill_tank_1_from_bore`, doesn't exist yet), and spot-check other
  thresholds.
- **MCU dry-cutoff report field** -- interface documented
  (`anomaly_rules.yaml`), not implemented (nothing to implement against
  yet -- this is on the MCU/ingestion side, not core).
- **Real ack field** (`action_confirmation.py`'s `{target}_last_ack_ts`)
  -- still a guessed placeholder, same status as last update.
- **`supervisor.py`** -- the actual watchdog process. Heartbeat write
  exists; nothing watches it yet.
- **Irrigation trend prediction** -- design agreed (predicted duration
  only sets the goal's starting timer, existing `until` conditions stay
  the real safety net), not built. Blocked on ingestion's
  `/trends/daily` per `PHASE_2_TREND_DETECTION.md`.
- **tank_2/tank_3 leak checks** -- blocked on the tank-selection design
  above; can't infer "is inflow happening" for a tank whose supply
  source isn't fixed the way bore_1→tank_1 is.
