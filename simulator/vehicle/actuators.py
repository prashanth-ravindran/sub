"""Surge propeller and symmetric aft elevator/rudder pairs."""

import numpy as np
from numpy.typing import ArrayLike

from common.messages import (
    MAX_ELEVATOR_DEG, MAX_PROPELLER_RPM, MAX_RUDDER_DEG,
    MIN_ELEVATOR_DEG, MIN_PROPELLER_RPM, MIN_RUDDER_DEG,
)
from .parameters import VehicleParameters, _vector


def actuator_forces(
    nu: ArrayLike, actuators: ArrayLike, parameters: VehicleParameters
) -> np.ndarray:
    """Map (RPM, elevator_deg, rudder_deg) to BODY [X, Y, Z, K, M, N].

    Positive elevator pitches the nose up; positive rudder yaws to starboard.
    Surface areas represent both fins in each pair, so do not double the force.
    """
    velocity = _vector(nu, 6, "nu")
    rpm, elevator, rudder = _vector(actuators, 3, "actuators")
    if not (
        MIN_PROPELLER_RPM <= rpm <= MAX_PROPELLER_RPM
        and MIN_ELEVATOR_DEG <= elevator <= MAX_ELEVATOR_DEG
        and MIN_RUDDER_DEG <= rudder <= MAX_RUDDER_DEG
    ):
        raise ValueError("Actuators must be 0–3000 RPM and within ±20 degrees")
    thrust = parameters.propeller_thrust_coefficient * (rpm / 60.0) ** 2
    # ponytail: forward-flow, small-angle fins; add incidence/stall/wash for higher fidelity.
    lift = 0.5 * parameters.water_density_kg_m3 * max(velocity[0], 0.0) ** 2
    lift *= parameters.fin_lift_slope_per_rad
    z_force = lift * parameters.elevator_area_m2 * np.deg2rad(elevator)
    y_force = -lift * parameters.rudder_area_m2 * np.deg2rad(rudder)
    return np.array([
        thrust, y_force, z_force, 0.0,
        -parameters.fin_x_m * z_force, parameters.fin_x_m * y_force,
    ])
