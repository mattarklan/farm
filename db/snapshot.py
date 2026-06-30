import time
import json

from farm.db.repository import (
    get_latest_all,
    get_metrics
)


SNAPSHOT_INTERVAL_SEC = 300  # 5 minutes


# ----------------------------
# Trend calculation
# ----------------------------
def compute_trend(values):
    if len(values) < 2:
        return "stable"

    first = values[0]["value"]
    last = values[-1]["value"]

    if last > first * 1.02:
        return "rising"
    if last < first * 0.98:
        return "falling"
    return "stable"


# ----------------------------
# Build snapshot per asset
# ----------------------------
def build_snapshot():
    latest_rows = get_latest_all()

    assets = {}

    # group latest values
    for r in latest_rows:
        asset_id = r["asset_id"]
        metric = r["metric"]

        assets.setdefault(asset_id, {
            "asset_id": asset_id,
            "metrics": {},
            "confidence": {},
            "quality": {},
            "ts": r["ts"]
        })

        assets[asset_id]["metrics"][metric] = r["value"]
        assets[asset_id]["confidence"][metric] = r["confidence"]
        assets[asset_id]["quality"][metric] = r["quality"]

    snapshots = []

    now = int(time.time())
    since = now - 86400  # 24h window for trends

    for asset_id, state in assets.items():

        # trend per metric
        trends = {}

        for metric in state["metrics"].keys():
            history = get_metrics(asset_id, metric, since)
            history = [dict(h) for h in history]

            trends[metric] = compute_trend(history)

        snapshot = {
            "asset_id": asset_id,
            "ts": now,
            "metrics": state["metrics"],
            "confidence": state["confidence"],
            "quality": state["quality"],
            "trends": trends
        }

        snapshots.append(snapshot)

    return snapshots


# ----------------------------
# Run loop
# ----------------------------
def run():
    while True:
        snapshots = build_snapshot()

        # for now: just print
        # later: store in DB via repository
        print(f"[snapshot] generated {len(snapshots)} assets")

        for s in snapshots[:2]:
            print(json.dumps(s, indent=2))

        time.sleep(SNAPSHOT_INTERVAL_SEC)


if __name__ == "__main__":
    run()