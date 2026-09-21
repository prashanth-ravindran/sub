import numpy as np

from simulator.simulation.integrator import step_vehicle
from simulator.vehicle.damping import damping_vector
from simulator.vehicle.kinematics import quaternion_from_euler, euler_from_quaternion
from simulator.vehicle.parameters import VehicleParameters


def test_damping_opposes_velocity_and_dissipates_energy_in_both_directions():
    parameters = VehicleParameters()
    for velocity in np.random.default_rng(42).normal(size=(20, 6)):
        damping = damping_vector(velocity, parameters)
        assert np.all(velocity * damping >= 0)
        assert velocity @ damping > 0
        np.testing.assert_allclose(damping_vector(-velocity, parameters), -damping)
    np.testing.assert_array_equal(damping_vector(np.zeros(6), parameters), np.zeros(6))


def test_unforced_surge_slows_and_roll_disturbance_settles():
    parameters = VehicleParameters()
    state = np.zeros(13)
    state[2], state[3], state[7] = 50, 1, 3
    for step in range(500):
        previous = state[7]
        state = step_vehicle(state, [0, 0, 0], step * 0.02, 0.02, parameters)
        assert 0 < state[7] < previous
    state = np.zeros(13)
    state[2], state[3:7] = 50, quaternion_from_euler(0.2, 0, 0)
    for step in range(1000):
        state = step_vehicle(state, [0, 0, 0], step * 0.02, 0.02, parameters)
    assert abs(euler_from_quaternion(state[3:7])[0]) < 0.001
    assert abs(state[10]) < 0.005
