import numpy as np
import pytest

from simulator.vehicle.actuators import actuator_forces
from simulator.vehicle.parameters import VehicleParameters


def test_propeller_and_fin_units_signs_pairs_and_speed_scaling():
    parameters = VehicleParameters()
    command = [1800, 5, 3]
    force = actuator_forces([3, 0, 0, 0, 0, 0], command, parameters)
    expected_z = 0.5 * 1025 * 9 * 0.02 * 4 * np.deg2rad(5)
    expected_y = -0.5 * 1025 * 9 * 0.02 * 4 * np.deg2rad(3)
    np.testing.assert_allclose(force, [90, expected_y, expected_z, 0, 0.8 * expected_z, -0.8 * expected_y])
    np.testing.assert_allclose(
        actuator_forces([6, 0, 0, 0, 0, 0], command, parameters)[1:], 4 * force[1:],
    )
    for speed in (0, -3):
        np.testing.assert_array_equal(
            actuator_forces([speed, 0, 0, 0, 0, 0], command, parameters), [90, 0, 0, 0, 0, 0],
        )


@pytest.mark.parametrize("command", [[-1, 0, 0], [3001, 0, 0], [0, 21, 0], [0, 0, -21], [0, np.nan, 0], [0, 0]])
def test_invalid_actuator_commands_are_rejected(command):
    with pytest.raises(ValueError):
        actuator_forces(np.zeros(6), command, VehicleParameters())
