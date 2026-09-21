import numpy as np
import pytest

from simulator.vehicle.hydrostatics import restoring_vector
from simulator.vehicle.kinematics import quaternion_from_euler
from simulator.vehicle.parameters import VehicleParameters


@pytest.mark.parametrize("roll, pitch, yaw", [
    (0, 0, 0), (0, 0, 1.2), (0.1, 0, 0), (-0.1, 0, 0),
    (0, 0.2, 0), (0, -0.2, 0), (0.3, -0.4, 1.1),
    (0.3, np.pi / 2, -0.4), (0.3, -np.pi / 2, -0.4),
])
def test_restoring_matches_fossen_neutral_buoyancy_formula(roll, pitch, yaw):
    parameters = VehicleParameters()
    quat = quaternion_from_euler(roll, pitch, yaw)
    restoring = restoring_vector(quat, parameters)
    stiffness = parameters.mass_kg * parameters.gravity_mps2 * parameters.cb_height_m
    expected = [
        0, 0, 0,
        stiffness * np.cos(pitch) * np.sin(roll),
        stiffness * np.sin(pitch),
        0,
    ]
    np.testing.assert_allclose(restoring, expected, atol=1e-13)
    # The physical hydrostatic torque is -g, opposing small roll/pitch disturbances.
    assert -restoring[3] * roll <= 1e-13
    assert -restoring[4] * pitch <= 1e-13


def test_coincident_cg_and_cb_have_no_restoring_moment():
    parameters = VehicleParameters(cb_height_m=0)
    quat = quaternion_from_euler(0.7, -0.3, 1.5)
    np.testing.assert_array_equal(restoring_vector(quat, parameters), np.zeros(6))
