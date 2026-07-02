# check_db.py — throwaway inspection script
import sqlite3
from farm.data.aggregator import local_day_bounds
import time

c = sqlite3.connect("farm/data/farm.db")

print("--- low-value tank_1 rows (looking for daily seed data) ---")
rows = c.execute("SELECT ts, value FROM raw_telemetry WHERE asset_id='tank_1' AND value < 45").fetchall()
print(rows)

print("--- current local_day_bounds(now - 86400) ---")
now = int(time.time())
day_start, day_end = local_day_bounds(now - 86400)
print(day_start, day_end)
print("as local time:", time.ctime(day_start), "to", time.ctime(day_end))