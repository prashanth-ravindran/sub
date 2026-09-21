"""Rigid-body and added-mass Coriolis/centripetal terms."""

import numpy as np
from numpy.typing import ArrayLike

from .parameters import VehicleParameters, _vector


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])


def coriolis_matrix(nu: ArrayLike, parameters: VehicleParameters) -> np.ndarray:
    """Return C_RB at CG, with C @ nu = [m omega x v, omega x (I omega)]."""
    omega = _vector(nu, 6, "nu")[3:]
    angular_momentum = np.asarray(parameters.inertia_kg_m2) * omega
    coriolis = np.zeros((6, 6))
    coriolis[:3, :3] = parameters.mass_kg * _skew(omega)
    coriolis[3:, 3:] = -_skew(angular_momentum)
    return coriolis


def added_mass_coriolis_matrix(nu: ArrayLike, parameters: VehicleParameters) -> np.ndarray:
    """Return skew-symmetric C_A using momenta from the positive M_A matrix."""
    momentum = np.asarray(parameters.added_mass) * _vector(nu, 6, "nu")
    linear = -_skew(momentum[:3])
    coriolis = np.zeros((6, 6))
    coriolis[:3, 3:] = linear
    coriolis[3:, :3] = linear
    coriolis[3:, 3:] = -_skew(momentum[3:])
    return coriolis
