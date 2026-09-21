"""Neutral buoyancy and restoring moments with CB directly above CG."""

import numpy as np
from numpy.typing import ArrayLike

from .kinematics import body_to_ned_matrix
from .parameters import VehicleParameters


def restoring_vector(
    quaternion: ArrayLike, parameters: VehicleParameters
) -> np.ndarray:
    """Return Fossen's g, the negative of physical weight/buoyancy forces.

    g belongs on the left of M nu_dot + C nu + g = tau. The BODY origin
    is CG, so weight has no lever arm. Neutral buoyancy holds at all attitudes.
    """
    rotation = body_to_ned_matrix(quaternion)
    # Calculate weight.
    weight_ned = np.array([0.0, 0.0, parameters.mass_kg * parameters.gravity_mps2])
    # weight into the vehicle’s coordinate frame.
    weight_body = rotation.T @ weight_ned
    # This implements neutral buoyancy
    buoyancy_body = -weight_body
    cb_body = np.array([0.0, 0.0, -parameters.cb_height_m])
    # Torque = lever arm × force
    moment_body = np.cross(cb_body, buoyancy_body)
    # -ve as it acts against a induced roll
    return -np.concatenate((weight_body + buoyancy_body, moment_body))
