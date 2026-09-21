import numpy as np
import pytest

from simulator.vehicle.dynamics import state_derivative
from simulator.vehicle.kinematics import quaternion_from_euler
from simulator.vehicle.parameters import VehicleParameters


def rigid_body_parameters(**kwargs):
    return VehicleParameters(
        added_mass=(0,) * 6, linear_damping=(0,) * 6, quadratic_damping=(0,) * 6,
        **kwargs,
    )


def test_level_rest_is_equilibrium():
    state = np.zeros(13)
    state[2], state[3] = 10, 1
    derivative = state_derivative(state, np.zeros(6), VehicleParameters())
    np.testing.assert_array_equal(derivative, np.zeros(13))


def test_applied_forces_and_moments_give_expected_accelerations_without_mutation():
    state = np.zeros(13)
    state[3] = 1
    tau = np.array([84, 168, 252, 0.7, 50, 75])
    original_state, original_tau = state.copy(), tau.copy()
    derivative = state_derivative(state, tau, rigid_body_parameters())
    np.testing.assert_allclose(derivative, [0] * 7 + [1, 2, 3, 1, 2, 3])
    np.testing.assert_array_equal(state, original_state)
    np.testing.assert_array_equal(tau, original_tau)


def test_unforced_rotating_body_has_newton_euler_coupling():
    parameters = rigid_body_parameters(inertia_kg_m2=(3, 4, 5), cb_height_m=0)
    state = np.array([0, 0, 10, 1, 0, 0, 0, 2, -0.3, 0.7, 0.2, -0.4, 0.5])
    velocity, omega = state[7:10], state[10:]
    inertia = np.asarray(parameters.inertia_kg_m2)
    expected_acceleration = np.r_[
        -np.cross(omega, velocity),
        -np.cross(omega, inertia * omega) / inertia,
    ]
    derivative = state_derivative(state, np.zeros(6), parameters)
    np.testing.assert_allclose(derivative[:3], velocity)
    np.testing.assert_allclose(derivative[3:7], [0, *(0.5 * omega)])
    np.testing.assert_allclose(derivative[7:], expected_acceleration, atol=1e-14)


def test_tilted_rest_accelerates_toward_level():
    parameters = rigid_body_parameters()
    roll, pitch = 0.1, -0.2
    state = np.zeros(13)
    state[3:7] = quaternion_from_euler(roll, pitch, 0.7)
    derivative = state_derivative(state, np.zeros(6), parameters)
    stiffness = parameters.mass_kg * parameters.gravity_mps2 * parameters.cb_height_m
    np.testing.assert_allclose(derivative[:10], np.zeros(10), atol=1e-14)
    np.testing.assert_allclose(derivative[10:], [
        -stiffness * np.cos(pitch) * np.sin(roll) / parameters.inertia_kg_m2[0],
        -stiffness * np.sin(pitch) / parameters.inertia_kg_m2[1],
        0,
    ], atol=1e-13)


@pytest.mark.parametrize("invalid", [
    "state_shape", "state_nonfinite", "zero_quaternion", "tau_shape", "tau_nonfinite",
])
def test_invalid_state_and_force_inputs_are_rejected(invalid):
    state = np.zeros(13)
    state[3] = 1
    tau = np.zeros(6)
    if invalid == "state_shape":
        state = state[:, None]
    elif invalid == "state_nonfinite":
        state[7] = np.nan
    elif invalid == "zero_quaternion":
        state[3] = 0
    elif invalid == "tau_shape":
        tau = tau[:5]
    else:
        tau[0] = np.inf
    with pytest.raises(ValueError):
        state_derivative(state, tau, VehicleParameters())
