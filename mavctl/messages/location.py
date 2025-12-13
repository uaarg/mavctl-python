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
class Velocity:
    north: float
    east: float
    down: float

    def magnitude(self) -> float:
        """Compute the 3D magnitude of the velocity vector."""
        return sqrt(self.north ** 2 + self.east ** 2 + self.down ** 2)
