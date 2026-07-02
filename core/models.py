from dataclasses import dataclass
from datetime import datetime
from typing import Any, Optional


# -------------------------
# STATE
# -------------------------
@dataclass(slots=True)
class SensorState:
    value: float = 0.0
    quality: float = 1.0
    timestamp: Optional[datetime] = None
    source: str = "sim"


# -------------------------
# EVENTS
# -------------------------
@dataclass(slots=True)
class Event:
    id: str
    event_type: str
    asset_id: str
    sensor_id: str
    severity: int
    confidence: float
    timestamp: datetime
    data: dict


# -------------------------
# ACTIONS
# -------------------------
@dataclass(slots=True)
class Action:
    id: str
    action_type: str
    target: Optional[str]
    parameters: dict
    priority: int
    created_from_event: str
    timestamp: datetime


@dataclass(slots=True)
class ActionResult:
    action_id: str
    success: bool
    message: str
    timestamp: datetime
