"""Illustrative vehicle parameters, expressed in SI units."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike


def _vector(value: ArrayLike, size: int, name: str) -> np.ndarray:
    """Validate a single finite vector without modifying the input."""
    vector = np.asarray(value, dtype=float)
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must be a finite vector with shape ({size},)")
    return vector


@dataclass(frozen=True)
class VehicleParameters:
    """Principal inertias about CG; CB is directly above CG, and B = W.

    """

    mass_kg: float = 84.0
    # roll moment, pitch moment, yaw moment
    inertia_kg_m2: tuple[float, float, float] = (0.7, 25.0, 25.0)
    # Center of buoyancy is 2 cm away from center of mass
    cb_height_m: float = 0.02
    gravity_mps2: float = 9.8
    # Order: surge, sway, heave, roll, pitch, yaw. Translational added mass
    # is in kg; rotational added inertia is in kg*m^2.
    added_mass: tuple[float, ...] = (5.0, 80.0, 80.0, 0.1, 10.0, 10.0)
    # Force/moment per velocity and per squared velocity, respectively.
    linear_damping: tuple[float, ...] = (2.0, 200.0, 200.0, 0.5, 400.0, 400.0)
    quadratic_damping: tuple[float, ...] = (10.0, 200.0, 200.0, 0.1, 50.0, 50.0)
    water_density_kg_m3: float = 1025.0
    propeller_thrust_coefficient: float = 0.1  # N / (rev/s)^2
    elevator_area_m2: float = 0.02  # Combined area of the pair.
    rudder_area_m2: float = 0.02
    fin_lift_slope_per_rad: float = 4.0
    fin_x_m: float = -0.8

    def __post_init__(self) -> None:
        for name in (
            "mass_kg", "cb_height_m", "gravity_mps2", "water_density_kg_m3",
            "propeller_thrust_coefficient", "elevator_area_m2", "rudder_area_m2",
            "fin_lift_slope_per_rad", "fin_x_m",
        ):
            value = np.asarray(getattr(self, name), dtype=float)
            if value.shape != () or not np.isfinite(value):
                raise ValueError(f"{name} must be a finite scalar")
            value = float(value)
            if name == "cb_height_m":
                if value < 0:
                    raise ValueError("cb_height_m must be nonnegative")
            elif name == "fin_x_m":
                if value >= 0:
                    raise ValueError("fin_x_m must be negative (aft of CG)")
            elif value <= 0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)

        inertia = _vector(self.inertia_kg_m2, 3, "inertia_kg_m2")
        if np.any(inertia <= 0):
            raise ValueError("Principal inertias must be positive")
        object.__setattr__(self, "inertia_kg_m2", tuple(float(i) for i in inertia))

        for name in ("added_mass", "linear_damping", "quadratic_damping"):
            values = _vector(getattr(self, name), 6, name)
            if np.any(values < 0):
                raise ValueError(f"{name} must be nonnegative")
            object.__setattr__(self, name, tuple(float(v) for v in values))
