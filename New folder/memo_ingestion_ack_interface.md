# Memo: Actuator Acknowledgment Interface — Needed From Ingestion

**To:** Ingestion team
**From:** Core
**Re:** One new interface — covers several open items at once

## The ask

A way for core to know a command actually reached an actuator's MCU and
what it did with it. One mechanism, two fields, applies to **every**
actuator in the registry (tank valves, bore pumps, zone valves) — not
bore-specific.

```
{target}_last_ack_ts    epoch seconds — last command this actuator's
                         MCU actually applied. Same shape as any other
                         metric (GET /state/latest, per-asset, with a
                         confidence entry same as everything else).

{target}_ack_status     string, companion to the above. "ok" for a
                         normal ack. Anything else means the actuator
                         explicitly declined the command and is telling
                         us why — e.g. "dry" when a bore refuses to
                         start because it's detected a dry well. Open-
                         ended: define whatever reason codes make sense
                         on your side, core treats anything non-"ok" as
                         "needs a human's attention," so you don't need
                         another round-trip with us to add a new one.
```

## Why this is the priority ask

Right now core cannot tell apart three genuinely different problems:
command never received (comms/MCU dead), command received but refused
(e.g. dry well), command received and applied but the physical result
never showed up (e.g. no flow, no power draw, indicates dry well or faulty pump). All three need different responses, and
core's retry/fallback logic is already built and waiting to consume
this — this is the single highest-leverage thing to ship, since it
unblocks that distinction across every actuator at once, not one at a
time.

## One thing that makes this easier on your end

MCUs are free to send the ack more than once (e.g. for redundancy) —
core reads `_last_ack_ts` as a value to compare against, not a one-shot
event, so duplicate sends are harmless on our side. No need to
de-duplicate before sending.

## Not needed / already resolved

Bore current/flow sensor naming (`bore_1_current`, `bore_1_flow`) is
already confirmed working against what's live — no change needed
there.
