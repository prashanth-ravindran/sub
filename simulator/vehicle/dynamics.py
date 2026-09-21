"""Full still-water vehicle derivative, independent of integration and IPC."""

import numpy as np
from numpy.typing import ArrayLike

from .added_mass import added_mass_matrix
from .coriolis import added_mass_coriolis_matrix, coriolis_matrix
from .damping import damping_vector
from .hydrostatics import restoring_vector
from .kinematics import kinematic_rates
from .parameters import VehicleParameters, _vector
from .rigid_body import mass_matrix


def state_derivative(
    state: ArrayLike, tau: ArrayLike, parameters: VehicleParameters
) -> np.ndarray:
    """Return derivatives of [N, E, D, qw, qx, qy, qz, u, v, w, p, q, r].

    tau = [X, Y, Z, K, M, N] is an applied BODY force/moment vector in N
    and N*m, not actuator deflections or RPM. Inputs are never modified.
    The integrator must normalize the quaternion after completed steps.
    """
    state = _vector(state, 13, "state")
    tau = _vector(tau, 6, "tau")
    quaternion, nu = state[3:7], state[7:]
    position_dot, quaternion_dot = kinematic_rates(quaternion, nu)
    mass = mass_matrix(parameters) + added_mass_matrix(parameters)
    coriolis = coriolis_matrix(nu, parameters) + added_mass_coriolis_matrix(nu, parameters)
    rhs = tau - coriolis @ nu - damping_vector(nu, parameters)
    rhs -= restoring_vector(quaternion, parameters)
    nu_dot = np.linalg.solve(mass, rhs)
    return np.concatenate((position_dot, quaternion_dot, nu_dot))
