from fastapi import FastAPI, HTTPException

from farm.db.repository import get_latest_asset
from farm.data.snapshot import build_snapshot

app = FastAPI(title="Farm State API")


# ----------------------------
# helpers
# ----------------------------
def get_latest_snapshot():
    return build_snapshot()


def find_asset_snapshot(asset_id: str):
    snapshots = build_snapshot()

    for s in snapshots:
        if s["asset_id"] == asset_id:
            return s

    return None


# ----------------------------
# full farm state
# ----------------------------
@app.get("/state/latest")
def state_latest():
    snapshots = get_latest_snapshot()

    return {
        "ts": snapshots[0]["ts"] if snapshots else None,
        "assets": snapshots
    }


# ----------------------------
# single asset state
# ----------------------------
@app.get("/state/asset/{asset_id}")
def state_asset(asset_id: str):
    snapshot = find_asset_snapshot(asset_id)

    if not snapshot:
        raise HTTPException(status_code=404, detail="asset not found")

    return snapshot


# ----------------------------
# LLM / decision engine context
# ----------------------------
@app.get("/state/context/{asset_id}")
def state_context(asset_id: str):
    snapshot = find_asset_snapshot(asset_id)

    if not snapshot:
        raise HTTPException(status_code=404, detail="asset not found")

    # compact form: remove redundancy, keep signal
    context = {
        "asset_id": snapshot["asset_id"],
        "metrics": snapshot["metrics"],
        "trends": snapshot["trends"],
        "confidence": snapshot["confidence"],
        "quality": snapshot["quality"],
        "ts": snapshot["ts"]
    }

    return context