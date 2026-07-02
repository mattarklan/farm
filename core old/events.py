#events.py
from dataclasses import dataclass
from typing import List
from datetime import datetime

from core.state import FarmState


@dataclass(slots=True)
class Event:
    event_id: str
    zone: str | None
    severity: int
    confidence: float
    timestamp: datetime
    data: dict


class EventEngine:
    """
    Converts raw state into discrete events.
    This is deterministic logic only (no YAML yet).
    """

    def process(self, state: FarmState) -> List[Event]:

        events: List[Event] = []

        # -------------------------
        # WATER EVENTS
        # -------------------------

        if state.water.tank_level_percent < 20:
            events.append(Event(
                event_id="W1_TANK_LEVEL_LOW",
                zone=None,
                severity=3,
                confidence=0.95,
                timestamp=state.timestamp,
                data={"tank_level": state.water.tank_level_percent}
            ))

        # -------------------------
        # ENERGY EVENTS
        # -------------------------

        if state.energy.battery_soc_percent < 20:
            events.append(Event(
                event_id="E1_BATTERY_LOW",
                zone=None,
                severity=4,
                confidence=0.98,
                timestamp=state.timestamp,
                data={"soc": state.energy.battery_soc_percent}
            ))

        # -------------------------
        # SOIL EVENTS
        # -------------------------

        for zone_name, zone in state.zones.items():

            if zone.soil_moisture < 0.25:

                events.append(Event(
                    event_id="S1_SOIL_MOISTURE_LOW",
                    zone=zone_name,
                    severity=3,
                    confidence=0.9 * zone.sensor_trust,
                    timestamp=state.timestamp,
                    data={
                        "moisture": zone.soil_moisture
                    }
                ))

        return events