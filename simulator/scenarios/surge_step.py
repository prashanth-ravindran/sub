"""A 90 N propeller thrust step at two simulated seconds with default parameters."""


def surge_step(time_s: float) -> tuple[float, float, float]:
    return (1800.0 if time_s >= 2.0 else 0.0, 0.0, 0.0)
