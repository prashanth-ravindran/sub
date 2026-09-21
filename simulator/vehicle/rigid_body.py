"""Rigid-body inertia at the centre of gravity, along principal axes."""

import numpy as np

from .parameters import VehicleParameters


def mass_matrix(parameters: VehicleParameters) -> np.ndarray:
    """Return M_RB for BODY velocity [u, v, w, p, q, r]."""
    # ponytail: principal axes at CG; add offset coupling if the origin moves.
    return np.diag([parameters.mass_kg] * 3 + list(parameters.inertia_kg_m2))
