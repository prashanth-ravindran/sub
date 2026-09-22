"""Flat-earth waypoint guidance in the local NED frame."""

import math


EARTH_RADIUS_M = 6_371_000.0


def clamp(value, low, high):
    return min(max(value, low), high)


def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def latlon_to_ne(lat, lon, lat0, lon0):
    north = EARTH_RADIUS_M * math.radians(lat - lat0)
    east = EARTH_RADIUS_M * math.cos(math.radians(lat0)) * math.radians(lon - lon0)
    return north, east


def waypoint_error(target_north, target_east, north, east):
    delta_n, delta_e = target_north - north, target_east - east
    return math.atan2(delta_e, delta_n), math.hypot(delta_n, delta_e)

