from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict


@dataclass(slots=True)
class WaterState:
    """Current water system measurements."""

    tank_level_percent: float = 100.0
    bore_flow_lpm: float = 0.0
    pressure_kpa: float = 0.0
    irrigation_flow_lpm: float = 0.0


@dataclass(slots=True)
class EnergyState:
    """Current electrical system measurements."""

    battery_soc_percent: float = 100.0
    solar_power_w: float = 0.0
    inverter_load_w: float = 0.0


@dataclass(slots=True)
class ZoneState:
    """Measurements for a single growing zone."""

    name: str

    soil_moisture: float = 0.0
    soil_temperature_c: float = 20.0

    air_temperature_c: float = 20.0
    humidity_percent: float = 50.0

    sensor_trust: float = 1.0


@dataclass(slots=True)
class FarmState:
    """Entire farm state."""

    timestamp: datetime = field(default_factory=datetime.now)

    water: WaterState = field(default_factory=WaterState)

    energy: EnergyState = field(default_factory=EnergyState)

    zones: Dict[str, ZoneState] = field(default_factory=dict)

    def get_zone(self, name: str) -> ZoneState:

        if name not in self.zones:
            self.zones[name] = ZoneState(name=name)

        return self.zones[name]

    def update_timestamp(self):

        self.timestamp = datetime.now()