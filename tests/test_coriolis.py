import numpy as np
import pytest

from simulator.vehicle.coriolis import added_mass_coriolis_matrix, coriolis_matrix
from simulator.vehicle.parameters import VehicleParameters


@pytest.mark.parametrize("nu", [
    [0, 0, 0, 0, 0, 0],
    [1.2, -0.4, 0.1, 0.3, -0.2, 0.5],
    [-0.3, 2, -1, -0.5, 0.7, -0.2],
])
def test_coriolis_matches_newton_euler_and_does_no_work(nu):
    parameters = VehicleParameters(mass_kg=10, inertia_kg_m2=(3, 4, 5))
    nu = np.asarray(nu)
    matrix = coriolis_matrix(nu, parameters)
    momentum = parameters.mass_kg * nu[:3]
    angular_momentum = np.asarray(parameters.inertia_kg_m2) * nu[3:]
    expected = np.r_[np.cross(nu[3:], momentum), np.cross(nu[3:], angular_momentum)]
    np.testing.assert_allclose(matrix @ nu, expected, atol=1e-14)
    np.testing.assert_array_equal(matrix + matrix.T, np.zeros((6, 6)))
    assert nu @ matrix @ nu == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("nu", [[0] * 5, np.zeros((6, 1)), [0, 0, np.nan, 0, 0, 0]])
def test_invalid_body_velocity_is_rejected(nu):
    with pytest.raises(ValueError):
        coriolis_matrix(nu, VehicleParameters())


def test_added_mass_coriolis_matches_momentum_cross_products_and_does_no_work():
    parameters = VehicleParameters(added_mass=(5, 70, 80, 0.1, 10, 12))
    for nu in np.random.default_rng(7).normal(size=(20, 6)):
        momentum = np.asarray(parameters.added_mass) * nu
        expected = np.r_[
            np.cross(nu[3:], momentum[:3]),
            np.cross(nu[:3], momentum[:3]) + np.cross(nu[3:], momentum[3:]),
        ]
        added = added_mass_coriolis_matrix(nu, parameters)
        np.testing.assert_allclose(added @ nu, expected, atol=1e-12)
        total = coriolis_matrix(nu, parameters) + added
        np.testing.assert_array_equal(total + total.T, np.zeros((6, 6)))
        assert nu @ total @ nu == pytest.approx(0, abs=1e-10)
