from dataclasses import dataclass
from math import sqrt
from typing import Optional

@dataclass
class LocationGlobal:
    lat: float
    lon: float
    alt: Optional[float] = None

@dataclass
class LocationGlobalRelative:
    lat: float
    lon: float
    alt: Optional[float] = None

@dataclass
class LocationLocal:
    north: float
    east: float
    down: float

@dataclass
class Battery:
    voltage: float  # in Volts
    current: Optional[float]  # in Amps
    level: Optional[int]      # battery level %

    def __init__(self, voltage: float, current: int, level: int):
        self.voltage = voltage / 1000.0
        self.current = None if current == -1 else current / 100.0
        self.level = None if level == -1 else level

@dataclass
class PositionAccuracy:
    horizontal: Optional[float] # in metres
    vertical: Optional[float]   # in metres

    def __init__(self, h_acc_mm: int, v_acc_mm: int):
        # GPS_RAW_INT reports millimetres, 0 means "not provided"
        if h_acc_mm > 0:
            self.horizontal = h_acc_mm / 1000.0
        else:
            self.horizontal = None

        if v_acc_mm > 0:
            self.vertical = v_acc_mm / 1000.0
        else:
            self.vertical = None

@dataclass
class Velocity:
    north: float
    east: float
    down: float

    def magnitude(self) -> float:
        return sqrt(self.north ** 2 + self.east ** 2 + self.down ** 2)
