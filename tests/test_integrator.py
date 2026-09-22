import numpy as np
import pytest

from simulator.simulation.integrator import rk4_step, step_vehicle
from simulator.vehicle.parameters import VehicleParameters
from simulator.ipc.messages import state_message


def test_rk4_fourth_order_convergence_and_non_autonomous_rhs():
    errors = []
    for dt in (0.2, 0.1, 0.05):
        initial = np.array([1.0])
        state = initial.copy()
        for step in range(round(1 / dt)):
            state = rk4_step(lambda t, x: x, step * dt, state, dt)
        errors.append(abs(state[0] - np.e))
        np.testing.assert_array_equal(initial, [1.0])
    assert 14 < errors[0] / errors[1] < 18
    assert 14 < errors[1] / errors[2] < 18
    np.testing.assert_allclose(rk4_step(lambda t, x: np.array([t**3]), 0, [0], 1), [0.25])


def test_vehicle_recomputes_fin_forces_at_intermediate_speeds_and_normalizes():
    state = np.zeros(13)
    state[2], state[3] = 50, 1
    result = step_vehicle(state, [1800, 5, 0], 0, 0.01, VehicleParameters())
    # At the initial zero speed there is no fin force; later RK stages have flow.
    assert result[11] > 0
    assert result[7] == pytest.approx(90 / 89 * 0.01, rel=0.001)
    assert np.linalg.norm(result[3:7]) == pytest.approx(1, abs=1e-14)
    np.testing.assert_array_equal(state, [0, 0, 50, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])


def test_vehicle_neutral_rest_and_free_rotation_energy():
    state = np.zeros(13)
    state[2], state[3] = 50, 1
    np.testing.assert_array_equal(step_vehicle(state, [0, 0, 0], 0, 0.01, VehicleParameters()), state)
    parameters = VehicleParameters(cb_height_m=0, linear_damping=(0,) * 6, quadratic_damping=(0,) * 6)
    inertia = np.array([84, 84, 84, 0.7, 25, 25]) + parameters.added_mass
    state[7:] = [1, -0.2, 0.3, 0.1, -0.05, 0.02]
    energy = 0.5 * np.sum(inertia * state[7:]**2)
    for step in range(200):
        state = step_vehicle(state, [0, 0, 0], step * 0.01, 0.01, parameters)
    assert 0.5 * np.sum(inertia * state[7:]**2) == pytest.approx(energy, rel=1e-8)
    assert np.linalg.norm(state[3:7]) == pytest.approx(1, abs=1e-14)


@pytest.mark.parametrize("dt", [0, -0.01, np.inf, np.nan])
def test_invalid_timesteps_are_rejected(dt):
    with pytest.raises(ValueError):
        rk4_step(lambda t, x: x, 0, [1], dt)


def test_nonfinite_derivatives_fail_clearly():
    with pytest.raises(ValueError, match="derivative"):
        rk4_step(lambda t, x: [np.nan], 0, [1], 0.01)


def test_cg_starts_at_waterline_and_can_cross_it_without_clamping():
    state = np.zeros(13)
    state[3] = 1
    parameters = VehicleParameters()
    sinking = step_vehicle(state, [0, 0, 0], 0, 0.01, parameters)
    assert sinking[2] > 0
    assert sinking[9] > 0
    np.testing.assert_array_equal(sinking[[0, 1, 7, 8, 10, 11, 12]], np.zeros(7))
    state[9] = -1.0
    rising = step_vehicle(state, [0, 0, 0], 0, 0.01, parameters)
    assert rising[2] < 0
    assert rising[9] < 0
    message = state_message(rising, 1, 0.01)
    assert message["depth_m"] == message["position"]["down_m"] == rising[2]
    for step in range(1, 501):
        rising = step_vehicle(rising, [0, 0, 0], step * 0.01, 0.01, parameters)
    assert rising[2] > 0
    assert np.all(np.isfinite(rising))
