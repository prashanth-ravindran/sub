import numpy as np
import pytest

from simulator.scenarios import SCENARIOS
from simulator.simulation.integrator import step_vehicle
from simulator.vehicle.kinematics import euler_from_quaternion
from simulator.vehicle.parameters import VehicleParameters


@pytest.mark.parametrize("name", SCENARIOS)
def test_scenario_steps_transients_and_timestep_convergence(name):
    scenario = SCENARIOS[name]
    assert scenario(0) == (0, 0, 0)
    assert scenario(1.99) == (0, 0, 0)
    assert scenario(2) == (1800, 0, 0)
    assert scenario(9.99) == (1800, 0, 0)
    assert scenario(10) == {"surge_step": (1800, 0, 0), "elevator_step": (1800, 1, 0), "rudder_step": (1800, 0, 3)}[name]
    results = []
    parameters = VehicleParameters()
    for dt in (0.02, 0.01):
        state = np.zeros(13)
        state[2], state[3] = 50, 1
        for step in range(round(30 / dt)):
            time_s = step * dt
            state = step_vehicle(state, scenario(time_s), time_s, dt, parameters)
        assert np.linalg.norm(state[3:7]) == pytest.approx(1, abs=1e-14)
        results.append(state)
    np.testing.assert_allclose(results[0], results[1], atol=2e-5, rtol=2e-5)
    roll, pitch, yaw = euler_from_quaternion(state[3:7])
    if name == "surge_step":
        terminal_speed = (-2 + np.sqrt(4 + 4 * 10 * 90)) / 20
        assert state[7] == pytest.approx(terminal_speed, rel=1e-5)
        assert state[0] > 70
        np.testing.assert_allclose(state[[1, 2]], [0, 50], atol=1e-12)
    elif name == "elevator_step":
        assert pitch > 0.1
        assert 0 < state[2] < 45
        assert state[9] > 0  # Tail force acts down while the pitched vehicle rises.
    else:
        assert yaw > 0.1
        assert state[1] > 20
        assert state[8] < 0
        assert state[2] == pytest.approx(50)
    assert abs(roll) < 1e-12
