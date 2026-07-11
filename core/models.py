from dataclasses import dataclass
from datetime import datetime
from typing import Any, Optional


# -------------------------
# STATE
# -------------------------
@dataclass(slots=True)
class SensorState:
    value: float = 0.0
    confidence: float = 1.0
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
    source: Optional[str] = None       # "operator" | "fois" | None -- which rule/proposal caused this
    proposal_id: Optional[str] = None  # governance proposal id, if source == "fois"


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
    severity: int = 3  # 1=CRITICAL..4=INFO -- tags the event raised if this action gets blocked/fails
    source: Optional[str] = None
    proposal_id: Optional[str] = None


@dataclass(slots=True)
class ActionResult:
    action_id: str
    success: bool
    message: str
    timestamp: datetime
