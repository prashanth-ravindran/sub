import numpy as np
import pytest

from simulator.scratch.fossen import submerged_ellipsoid
from simulator.vehicle.hydrostatics import restoring_vector
from simulator.vehicle.kinematics import body_to_ned_matrix, quaternion_from_euler
from simulator.vehicle.parameters import VehicleParameters


@pytest.mark.parametrize("roll, pitch, yaw", [
    (0, 0, 0), (0, 0, 1.2), (0.1, 0, 0), (-0.1, 0, 0),
    (0, 0.2, 0), (0, -0.2, 0), (0.3, -0.4, 1.1),
    (0.3, np.pi / 2, -0.4), (0.3, -np.pi / 2, -0.4),
])
def test_restoring_matches_fossen_neutral_buoyancy_formula(roll, pitch, yaw):
    parameters = VehicleParameters()
    quat = quaternion_from_euler(roll, pitch, yaw)
    restoring = restoring_vector(quat, parameters, 10.0)
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
    np.testing.assert_array_equal(restoring_vector(quat, parameters, 10.0), np.zeros(6))


@pytest.mark.parametrize("depth, fraction", [
    (-2.0, 0.0), (-1.0, 0.0), (-0.5, 5 / 32),
    (0.0, 0.5), (0.5, 27 / 32), (1.0, 1.0), (2.0, 1.0),
])
def test_submerged_volume_matches_spherical_caps(depth, fraction):
    actual, center = submerged_ellipsoid(depth, [0, 0, 1], 2.0, 4 * np.pi / 3, 0.0)
    assert actual == pytest.approx(fraction)
    if depth == 0:
        np.testing.assert_allclose(center, [0, 0, 3 / 8])


def test_surface_buoyancy_moment_uses_submerged_centroid_and_attitude():
    parameters = VehicleParameters(cb_height_m=0)
    quat = quaternion_from_euler(0.0, 0.3, 0.7)
    rotation = body_to_ned_matrix(quat)
    down = rotation[2]
    fraction, center = submerged_ellipsoid(
        0.0, down, parameters.hull_length_m,
        parameters.mass_kg / parameters.water_density_kg_m3, 0.0,
    )
    assert fraction == pytest.approx(0.5)
    assert center[0] < 0 < center[2]
    weight = parameters.mass_kg * parameters.gravity_mps2
    physical = -restoring_vector(quat, parameters, 0.0)
    np.testing.assert_allclose(rotation @ physical[:3], [0, 0, weight / 2], atol=1e-12)
    np.testing.assert_allclose(physical[3:], np.cross(center, -weight / 2 * down))
    assert physical[4] < 0
    level = -restoring_vector([1, 0, 0, 0], parameters, 0.2)
    tilted = -restoring_vector(quat, parameters, 0.2)
    np.testing.assert_array_equal(level[:3], [0, 0, 0])
    assert (rotation @ tilted[:3])[2] > 0


def test_surface_hydrostatics_are_continuous_at_dry_and_wet_limits():
    parameters = VehicleParameters()
    quat = quaternion_from_euler(0.2, -0.4, 0.7)
    rotation = body_to_ned_matrix(quat)
    half_length = parameters.hull_length_m / 2
    radius_squared = 3 * parameters.mass_kg / (4 * np.pi * parameters.water_density_kg_m3 * half_length)
    extent = np.sqrt(np.dot(rotation[2]**2, [half_length**2, radius_squared, radius_squared]))
    center_offset = parameters.cb_height_m * rotation[2, 2]
    for boundary in (center_offset - extent, center_offset + extent):
        np.testing.assert_allclose(
            restoring_vector(quat, parameters, boundary - 1e-8),
            restoring_vector(quat, parameters, boundary + 1e-8), atol=1e-10,
        )
    weight_body = rotation[2] * parameters.mass_kg * parameters.gravity_mps2
    np.testing.assert_allclose(-restoring_vector(quat, parameters, -2.0), [*weight_body, 0, 0, 0])


@pytest.mark.parametrize("depth", [np.nan, np.inf, -np.inf])
def test_surface_hydrostatics_reject_nonfinite_depth(depth):
    with pytest.raises(ValueError, match="finite"):
        restoring_vector([1, 0, 0, 0], VehicleParameters(), depth)
