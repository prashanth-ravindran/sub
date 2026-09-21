"""Linear and quadratic drag in still water."""

import numpy as np
from numpy.typing import ArrayLike

from .parameters import VehicleParameters, _vector


def damping_vector(nu: ArrayLike, parameters: VehicleParameters) -> np.ndarray:
    """Return D(nu) @ nu on Fossen's left-hand side; physical drag is its negative."""
    velocity = _vector(nu, 6, "nu")
    return (
        np.asarray(parameters.linear_damping)
        + np.asarray(parameters.quadratic_damping) * np.abs(velocity)
    ) * velocity
