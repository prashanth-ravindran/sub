"""Waypoint mission definition and validation."""

from dataclasses import dataclass
import math

from .guidance import latlon_to_ne


@dataclass(frozen=True)
class Mission:
    start_lat: float = 12.9716
    start_lon: float = 80.2209
    target_lat: float = 12.9752
    target_lon: float = 80.2246
    depth_m: float = 10.0
    speed_mps: float = 1.5
    arrival_radius_m: float = 15.0

    def validate(self):
        values = (self.start_lat, self.start_lon, self.target_lat,
                  self.target_lon, self.depth_m, self.speed_mps,
                  self.arrival_radius_m)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Mission values must be finite")
        if not (-90 < self.start_lat < 90 and -90 < self.target_lat < 90):
            raise ValueError("Latitude must be strictly between -90 and 90 degrees")
        if not (-180 <= self.start_lon <= 180 and -180 <= self.target_lon <= 180):
            raise ValueError("Longitude must be between -180 and 180 degrees")
        if self.depth_m <= 0 or self.speed_mps <= 0 or self.arrival_radius_m <= 0:
            raise ValueError("Depth, speed and arrival radius must be positive")
        if math.hypot(*self.target_ne()) <= self.arrival_radius_m:
            raise ValueError("Target must be outside the arrival radius")

    def target_ne(self):
        return latlon_to_ne(
            self.target_lat, self.target_lon, self.start_lat, self.start_lon
        )

