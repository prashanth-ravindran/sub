"""Neutral buoyancy and restoring moments with CB directly above CG."""

import numpy as np
from numpy.typing import ArrayLike

from simulator.scratch.fossen import submerged_ellipsoid

from .kinematics import body_to_ned_matrix
from .parameters import VehicleParameters


def restoring_vector(
    quaternion: ArrayLike, parameters: VehicleParameters, depth_m: float
) -> np.ndarray:
    """Return Fossen's g, the negative of physical weight/buoyancy forces.

    """
    rotation = body_to_ned_matrix(quaternion)
    fraction, cb_body = submerged_ellipsoid(
        depth_m, rotation[2], parameters.hull_length_m,
        parameters.mass_kg / parameters.water_density_kg_m3,
        -parameters.cb_height_m,
    )
    # Calculate weight.
    weight_ned = np.array([0.0, 0.0, parameters.mass_kg * parameters.gravity_mps2])
    # weight into the vehicle’s coordinate frame.
    weight_body = rotation.T @ weight_ned
    buoyancy_body = -weight_body * fraction
    # Torque = lever arm × force
    moment_body = np.cross(cb_body, buoyancy_body)
    # -ve as it acts against a induced roll
    return -np.concatenate((weight_body + buoyancy_body, moment_body))
