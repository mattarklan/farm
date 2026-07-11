#api.py
import json
import time

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

from farm.data.repository import (
    get_latest_all, insert_event, get_events, get_hourly_summary, get_daily_summary,
)

app = FastAPI(title="Farm State API")


def build_state():
    rows = get_latest_all()
    assets = {}
    for r in rows:
        a = assets.setdefault(r["asset_id"], {
            "asset_id": r["asset_id"], "metrics": {}, "confidence": {}, "ts": r["ts"]
        })
        a["metrics"][r["metric"]] = r["value"]
        a["confidence"][r["metric"]] = r["confidence"]
    return list(assets.values())


def find_asset_state(asset_id: str):
    for s in build_state():
        if s["asset_id"] == asset_id:
            return s
    return None


@app.get("/state/latest")
def state_latest():
    states = build_state()
    return {"ts": states[0]["ts"] if states else None, "assets": states}


@app.get("/state/asset/{asset_id}")
def state_asset(asset_id: str):
    state = find_asset_state(asset_id)
    if not state:
        raise HTTPException(status_code=404, detail="asset not found")
    return state


@app.get("/state/context/{asset_id}")
def state_context(asset_id: str):
    state = find_asset_state(asset_id)
    if not state:
        raise HTTPException(status_code=404, detail="asset not found")
    return state


@app.get("/trends/hourly/{asset_id}")
def trends_hourly(asset_id: str, since_ts: int = 0):
    return [dict(r) for r in get_hourly_summary(asset_id, since_ts)]


@app.get("/trends/daily/{asset_id}")
def trends_daily(asset_id: str, since_ts: int = 0):
    return [dict(r) for r in get_daily_summary(asset_id, since_ts)]


class EventIn(BaseModel):
    asset_id: str
    event_type: str
    payload: dict = {}
    ts: int | None = None


@app.post("/events")
def post_event(event: EventIn):
    ts = event.ts or int(time.time())
    insert_event(event.asset_id, event.event_type, json.dumps(event.payload), ts)
    return {"status": "ok"}


@app.get("/events")
def get_events_endpoint(asset_id: str = None, since_ts: int = 0):
    return [dict(r) for r in get_events(asset_id, since_ts)]