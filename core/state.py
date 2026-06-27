from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Any


@dataclass
class SystemState:
    timestamp: datetime
    sensors: Dict[str, Any]
    derived: Dict[str, Any] = None