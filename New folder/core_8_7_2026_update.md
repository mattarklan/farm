# Core Devlog Update — 08/07/2026

Addendum to `core_6_7_2026.txt` — that doc still stands as the full
reference; this is today's session's decisions only, to be folded in
once the items below are actually built (several are blocked on config
files / an outstanding doc, see section 6).

---

## 1. Action confirmation + fallback actions

Two separate fault classes need to be tracked separately, not
conflated:

- **Comms failure** — the action was sent, but the sensor/MCU never
  sends back a "job done" acknowledgment at all. This means we don't
  even know if the command was received, let alone whether it worked.
  No such ack round-trip exists today — `executor.py` only simulates
  (prints) the hardware effect and reports success locally; there's no
  real hardware confirmation loop yet. This is a genuinely new
  interface, not a code fix (see open questions, section 7).
- **Effect failure** — the ack came back fine (MCU's alive, command
  received), but the physical effect didn't happen (pump ran, wiring's
  broken so nothing moved; or a genuine dry-run). This is what
  `anomaly_engine`'s `action_effect_check` already covers, and needs no
  change to detect. Its existing `check_after_seconds` delay already
  accounts for the fact that a flow sensor won't read anything for a
  few seconds/minutes after a pump starts, since the water hasn't
  arrived yet — that was already correct, just confirming it.

**Fallback action, two layers:**

- **Default/placeholder** — every `action_type` gets a fallback even if
  nobody's defined one: wait 5 minutes, retry the same action. This is
  the safety-net floor, not a real strategy — nothing should ever have
  *zero* fallback.
- **Per-rule override** — a goal's `start_action` can declare its own
  specific fallback that takes priority over the default. Concrete
  case: tank_1's valve fails to open (or tank_1's level is already too
  low to be worth trying) → fallback routes to tank_2 instead of
  retrying the same broken target.

**Goal engine wiring implication:** on start-action failure, a goal
needs a new intermediate state — "attempting fallback" — rather than
either running normally or not existing. Haven't touched `goal_engine.py`
for this yet; want `action_types.yaml`/`rules.yaml` in hand first so the
fallback field's real shape is designed against real data, not guessed.

---

## 2. Crash recovery

Confirmed plan from last round stands: force every known target to its
safe/off state unconditionally on startup (don't try to resume
in-flight state), and use `actions.json` for bookkeeping — reconstructing
`last_stopped_at` so `min_interval_days` cooldowns survive a crash —
not for resuming live goals.

Crash recovery doc is coming but was written against a previous version
of core — treating it as inspiration/context only, not a spec to
implement verbatim. Will reconcile once it's shared.

---

## 3. Bore dry-run detection

Guessed current-draw band (dry-run vs failed vs normal) is fine to ship
now as an explicit placeholder — not waiting on real historical data
first. Will be clearly labeled `PLACEHOLDER — NEEDS VALIDATION` in
`anomaly_rules.yaml`/code comments so nobody mistakes a guessed band
for a calibrated one later.

---

## 4. Soil moisture / soil temp cross-reference

Confirmed framing from last round: soil temp is a diagnostic aid, not
true redundancy — it can't reconstruct a moisture value the way a
second flow sensor can stand in for level. Still worth wiring in: if a
moisture reading looks stuck/suspect but temp still shows normal
diurnal swing, that's evidence the probe's power/wiring is fine and
only the moisture element itself is bad. Proposed as an annotation on
the existing `SENSOR_VALUE_STUCK`/`SENSOR_CONFIDENCE_ZERO` event
(e.g. `data["temp_still_responsive"]`) rather than a confidence
override — it's context for a human, not grounds to auto-restore trust
in the moisture reading.

---

## 5. Irrigation trend prediction

Confirmed: predicted duration (previous days cross-referenced with
temperature) only sets the goal's *starting* timer/target. The existing
reactive `until` conditions — moisture threshold, safety timer,
constraints — remain the actual stop mechanism, unchanged. A bad
prediction degrades to "watered for the wrong initial duration," never
"lost the safety net." Still blocked on ingestion's `/trends/daily`
being real (per `PHASE_2_TREND_DETECTION.md`) — `window_store`'s 1h
retention doesn't hold enough history for day-over-day comparison
regardless.

---

## 6. Confirmed fixes

- **S1 threshold** — fixed (was incorrectly set to `55`, off by two
  orders of magnitude against the 0–1 fractional scale sensors actually
  report on; this is what caused `S1_SOIL_MOISTURE_LOW` to fire at 0.39).
- **`ACTION_BLOCKED` events need the real triggering value** — e.g. the
  actual battery voltage reading, not just the static `reason` string
  from `constraints.yaml`. Plan (from last round) stands: thread the
  triggering sensor_id/value through `_check_raw` → `_evaluate_rule` →
  `evaluate()` → `pipeline._blocked_event`. Not yet implemented — needs
  the real `constraints.yaml` to match against actual rule shapes rather
  than guessed ones.

---

## 7. Open questions (need your call, not just files)

- **Job-done ack interface**: does ingestion have — or plan to have — a
  command/acknowledgment endpoint at all? Right now there's no real
  hardware round-trip anywhere in core (`executor.py` simulates and
  reports success locally). If this doesn't exist on ingestion's side
  yet, it's a new cross-team item, same shape as the existing
  `INGESTION_REQUESTS.md` asks.
- **Timeout relationship**: is "no job-done received" judged against the
  same 5-minute window as the default retry fallback, or is comms
  timeout meant to be shorter/separate from the retry delay? These are
  two different clocks conceptually (how long to wait for an ack vs.
  how long to wait before retrying) and I don't want to collapse them
  into one number without checking.

---

## 8. Files requested next

- `config/action_types.yaml` — to add the default fallback field and
  see real action_type names for the per-rule override design.
- `config/constraints.yaml` — to implement the real battery-voltage
  (and other) triggering-value fix, and to write the real tank_1/tank_2
  overflow constraint.
- `config/anomaly_rules.yaml` — to add the bore dry-run placeholder band
  and the soil-temp cross-reference annotation.
- `config/rules.yaml` — specifically the tank_1/tank_2 and bore-related
  goal rules, to design the per-rule fallback field against real
  targets rather than a guess.
- `config/event_rules.yaml` — to confirm the S1 fix landed as intended
  and check for any other thresholds worth sanity-checking the same way.
- `config/sensor_range_thresholds.yaml` — for the bore dry-run band
  placeholder, if bore current draw already has an entry there.
- The crash recovery doc, whenever it's ready (inspiration-only, per
  section 2).
