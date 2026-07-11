"""
Shared dedup/reminder/counting layer -- every event from every engine
passes through this before reaching events.json. This is deliberately
the ONE place that decides "how often does a human need to see this,"
so individual engines (event_engine, confidence_monitor, anomaly_engine,
...) don't each need their own one-shot/suppression logic -- they just
report "is this true right now" every cycle, honestly, every time.

Key = (event_type, asset_id, sensor_id) -- "the same underlying problem."

Behaviour:
  - First occurrence of a key -> emitted immediately, starts a new
    "episode," increments today's occurrence count for that key.
  - While the same key keeps recurring (episode ongoing) -> suppressed,
    except a reminder is let through every `reminder_seconds` (default
    300s/5min) so a long-running problem doesn't vanish from the log
    for hours -- this is also what makes a permanently-stuck sensor
    keep reporting itself instead of going silent after the first flag.
  - If a key stops appearing for at least `cooldown_seconds` (default
    300s, same number reused deliberately -- flag if you want these to
    differ) before reappearing, that counts as a genuinely NEW episode
    -- the daily occurrence count increments again.
  - Every emitted event gets `data["occurrences_today"]` (episodes
    started since local AWST midnight, resets daily) and
    `data["is_reminder"]` (False for the first-fire, True for a repeat)
    -- what the GUI can show as e.g. "x10" next to an event.

Whether a human has actually SEEN/acknowledged an event is a GUI-side
concern, not built here -- core just guarantees the data (id,
timestamp, severity, occurrences_today) a GUI needs to build its own
"remind me about anything unchecked" logic on top of.
"""
from __future__ import annotations

from datetime import datetime, timedelta, timezone

AWST_OFFSET = timedelta(hours=8)  # UTC+8, no DST -- matches the ingestion day-boundary convention


class EventDeduplicator:
    def __init__(self, reminder_seconds: int = 300, cooldown_seconds: int = 300):
        self.reminder_seconds = reminder_seconds
        self.cooldown_seconds = cooldown_seconds
        self._last_seen: dict[tuple, datetime] = {}
        self._last_emitted: dict[tuple, datetime] = {}
        self._episode_active: dict[tuple, bool] = {}
        self._day_counts: dict[tuple, int] = {}
        self._count_day: dict[tuple, object] = {}

    def process(self, events: list, now: datetime) -> list:
        if now.tzinfo is None:
            now = now.replace(tzinfo=timezone.utc)

        seen_this_cycle = set()
        output = []

        for event in events:
            key = (event.event_type, event.asset_id, event.sensor_id)
            seen_this_cycle.add(key)

            last_seen = self._last_seen.get(key)
            episode_active = self._episode_active.get(key, False)
            gap_too_long = last_seen is not None and (now - last_seen).total_seconds() > self.cooldown_seconds
            is_new_episode = not episode_active or gap_too_long

            if is_new_episode:
                self._episode_active[key] = True
                self._bump_daily_count(key, now)
                event.data["occurrences_today"] = self._day_counts[key]
                event.data["is_reminder"] = False
                self._last_emitted[key] = now
                output.append(event)
            else:
                last_emitted = self._last_emitted.get(key)
                due_for_reminder = last_emitted is None or (now - last_emitted).total_seconds() >= self.reminder_seconds
                if due_for_reminder:
                    event.data["occurrences_today"] = self._day_counts.get(key, 1)
                    event.data["is_reminder"] = True
                    self._last_emitted[key] = now
                    output.append(event)
                # else: same ongoing episode, too soon for a reminder -- suppressed

            self._last_seen[key] = now

        # keys that went quiet long enough are no longer "active" -- the
        # next time they appear, is_new_episode above will correctly
        # treat it as a fresh episode rather than a continuation
        for key, active in list(self._episode_active.items()):
            if not active or key in seen_this_cycle:
                continue
            last_seen = self._last_seen.get(key)
            if last_seen and (now - last_seen).total_seconds() > self.cooldown_seconds:
                self._episode_active[key] = False

        return output

    def _bump_daily_count(self, key, now: datetime):
        local_date = (now + AWST_OFFSET).date()
        if self._count_day.get(key) != local_date:
            self._count_day[key] = local_date
            self._day_counts[key] = 0
        self._day_counts[key] += 1
