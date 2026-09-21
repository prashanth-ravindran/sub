"""Positive diagonal hydrodynamic added mass and inertia."""

import numpy as np

from .parameters import VehicleParameters


def added_mass_matrix(parameters: VehicleParameters) -> np.ndarray:
    """Return M_A in [u, v, w, p, q, r] order; coefficients are positive masses."""
    return np.diag(parameters.added_mass)
