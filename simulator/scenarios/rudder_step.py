"""Accelerate from rest, then command a three-degree starboard rudder step."""

from .surge_step import surge_step


def rudder_step(time_s: float) -> tuple[float, float, float]:
    return (surge_step(time_s)[0], 0.0, 3.0 if time_s >= 10.0 else 0.0)
