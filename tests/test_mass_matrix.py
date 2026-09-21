from dataclasses import FrozenInstanceError

import numpy as np
import pytest

from simulator.vehicle.parameters import VehicleParameters
from simulator.vehicle.rigid_body import mass_matrix
from simulator.vehicle.added_mass import added_mass_matrix


def test_mass_matrix_is_symmetric_positive_definite_with_expected_units():
    matrix = mass_matrix(VehicleParameters())
    np.testing.assert_array_equal(matrix, np.diag([84, 84, 84, 0.7, 25, 25]))
    np.testing.assert_array_equal(matrix, matrix.T)
    assert np.all(np.linalg.eigvalsh(matrix) > 0)


def test_parameters_are_adjustable_and_immutable():
    inertia = [3, 4, 5]
    parameters = VehicleParameters(mass_kg=10, inertia_kg_m2=inertia)
    inertia[0] = 99
    np.testing.assert_array_equal(np.diag(mass_matrix(parameters)), [10, 10, 10, 3, 4, 5])
    with pytest.raises(FrozenInstanceError):
        parameters.mass_kg = 20


def test_total_mass_is_symmetric_positive_definite():
    parameters = VehicleParameters()
    matrix = mass_matrix(parameters) + added_mass_matrix(parameters)
    np.testing.assert_allclose(np.diag(matrix), [89, 164, 164, 0.8, 35, 35])
    np.testing.assert_array_equal(matrix, matrix.T)
    assert np.all(np.linalg.eigvalsh(matrix) > 0)
    original = [5, 80, 80, 0.1, 10, 10]
    parameters = VehicleParameters(added_mass=original)
    original[0] = 999
    assert parameters.added_mass[0] == 5


@pytest.mark.parametrize("field, value", [
    ("mass_kg", 0), ("mass_kg", -1), ("mass_kg", np.nan),
    ("mass_kg", [84]), ("gravity_mps2", 0), ("gravity_mps2", np.inf),
    ("cb_height_m", -0.01), ("cb_height_m", np.nan),
    ("inertia_kg_m2", [0, 25, 25]), ("inertia_kg_m2", [-0.7, 25, 25]),
    ("inertia_kg_m2", [0.7, 25]), ("inertia_kg_m2", [0.7, np.inf, 25]),
    ("added_mass", [-1, 0, 0, 0, 0, 0]), ("added_mass", [1] * 5),
    ("linear_damping", [np.nan] * 6), ("quadratic_damping", [-1] * 6),
    ("water_density_kg_m3", 0), ("propeller_thrust_coefficient", -1),
    ("elevator_area_m2", 0), ("rudder_area_m2", np.inf),
    ("fin_lift_slope_per_rad", 0), ("fin_x_m", 0.8),
])
def test_invalid_physical_parameters_are_rejected(field, value):
    with pytest.raises(ValueError):
        VehicleParameters(**{field: value})
