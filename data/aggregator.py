import time

from farm.data.repository import (
    compute_hourly_summary, upsert_hourly_summary,
    compute_daily_summary, upsert_daily_summary,
    purge_raw_before,
)

RETENTION_DAYS = 90
CHECK_INTERVAL_SEC = 60
LOCAL_UTC_OFFSET_SEC = 8 * 3600  # AWST, no DST — adjust if farm is elsewhere


def hour_bounds(ts):
    hour_start = ts - (ts % 3600)
    return hour_start, hour_start + 3600


def local_day_bounds(ts):
    local_ts = ts + LOCAL_UTC_OFFSET_SEC
    day_start_local = local_ts - (local_ts % 86400)
    day_start_utc = day_start_local - LOCAL_UTC_OFFSET_SEC
    return day_start_utc, day_start_utc + 86400


def run_hourly_rollup(now):
    hour_start, hour_end = hour_bounds(now - 3600)  # last COMPLETED hour
    rows = compute_hourly_summary(hour_start, hour_end)
    for r in rows:
        upsert_hourly_summary(hour_start, r["asset_id"], r["metric"], r["min"], r["max"], r["avg"], r["count"])
    print(f"[aggregator] hourly rollup for {hour_start}: {len(rows)} rows")


def run_daily_rollup(now):
    day_start, day_end = local_day_bounds(now - 86400)  # last COMPLETED local day
    rows = compute_daily_summary(day_start, day_end)
    for r in rows:
        upsert_daily_summary(day_start, r["asset_id"], r["metric"], r["min"], r["max"], r["avg"], r["count"])
    print(f"[aggregator] daily rollup for {day_start}: {len(rows)} rows")


def run_retention(now):
    cutoff = now - (RETENTION_DAYS * 86400)
    purge_raw_before(cutoff)
    print(f"[aggregator] purged raw_telemetry older than {cutoff}")


def run():
    last_hour_done = None
    last_day_done = None

    while True:
        now = int(time.time())
        current_hour = now // 3600
        current_local_day = (now + LOCAL_UTC_OFFSET_SEC) // 86400

        if current_hour != last_hour_done:
            run_hourly_rollup(now)
            last_hour_done = current_hour

        if current_local_day != last_day_done:
            run_daily_rollup(now)
            run_retention(now)
            last_day_done = current_local_day

        time.sleep(CHECK_INTERVAL_SEC)


if __name__ == "__main__":
    run()