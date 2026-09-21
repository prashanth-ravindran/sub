"""Flat-Earth NED displacement to latitude/longitude using a fixed local scale."""

import math


# Used only to set the metres-to-degrees scale, not to curve the NED frame. Flat earth model only
EARTH_RADIUS_M = 6_371_000.0


def ned_to_geodetic(
    north_m: float, east_m: float, latitude_origin_deg: float = 13.0,
    longitude_origin_deg: float = 80.0,
) -> tuple[float, float]:
    """Return latitude/longitude in degrees; depth is the unchanged NED down value."""
    values = (north_m, east_m, latitude_origin_deg, longitude_origin_deg)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("Geographic coordinates must be finite")
    if not -90 < latitude_origin_deg < 90 or not -180 <= longitude_origin_deg <= 180:
        raise ValueError("Origin must have latitude strictly between ±90 and longitude within ±180")
    # ponytail: fixed flat-Earth scale at the origin; use geodesy for long paths.
    latitude = latitude_origin_deg + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = longitude_origin_deg + math.degrees(
        east_m / EARTH_RADIUS_M / math.cos(math.radians(latitude_origin_deg))
    )
    if not -90 <= latitude <= 90 or not math.isfinite(longitude):
        raise ValueError("Displacement exceeds the local geographic model's domain")
    return latitude, (longitude + 180) % 360 - 180
