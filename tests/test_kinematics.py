import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from simulator.vehicle.kinematics import (
    body_to_ned_matrix,
    euler_from_quaternion,
    kinematic_rates,
    normalize_quaternion,
    quaternion_from_euler,
)


@pytest.mark.parametrize("angles, body, ned", [
    ((0, 0, 0), (1, 2, 3), (1, 2, 3)),
    ((0, 0, np.pi / 2), (1, 0, 0), (0, 1, 0)),
    ((0, np.pi / 2, 0), (1, 0, 0), (0, 0, -1)),
    ((np.pi / 2, 0, 0), (0, 1, 0), (0, 0, 1)),
])
def test_body_axes_and_position_rates(angles, body, ned):
    quat = quaternion_from_euler(*angles)
    position_dot, quat_dot = kinematic_rates(quat, [*body, 0, 0, 0])
    np.testing.assert_allclose(position_dot, ned, atol=1e-14)
    np.testing.assert_array_equal(quat_dot, np.zeros(4))


def test_mixed_rotation_order_and_round_trips():
    angles = (np.pi / 2, np.pi / 6, np.pi / 2)
    quat = quaternion_from_euler(*angles)
    rotation = body_to_ned_matrix(quat)
    # Rz(90 degrees) Ry(30 degrees) Rx(90 degrees), evaluated independently.
    expected = [[0, 0, 1], [np.sqrt(3) / 2, 0.5, 0], [-0.5, np.sqrt(3) / 2, 0]]
    np.testing.assert_allclose(rotation, expected, atol=1e-14)
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-14)
    assert np.linalg.det(rotation) == pytest.approx(1.0)
    np.testing.assert_allclose(euler_from_quaternion(quat), angles, atol=1e-14)
    np.testing.assert_allclose(body_to_ned_matrix(-quat), rotation, atol=1e-14)


@pytest.mark.parametrize("scale", [1.0, -2.0, 1e300, 1e-300])
def test_quaternion_normalization_handles_scale(scale):
    quat = np.array([1.0, -0.5, 0.25, -0.125])
    normalized = normalize_quaternion(quat * scale)
    np.testing.assert_allclose(normalized, np.sign(scale) * quat / np.linalg.norm(quat))
    assert np.linalg.norm(normalized) == pytest.approx(1.0)


def test_quaternion_rate_matches_body_rotation_and_preserves_norm():
    quat = quaternion_from_euler(0.2, -0.3, 0.4)
    omega = np.array([0.3, -0.2, 0.5])
    _, derivative = kinematic_rates(quat, [0, 0, 0, *omega])
    dt = 1e-7
    rotation = Rotation.from_quat(quat, scalar_first=True)
    advanced = (rotation * Rotation.from_rotvec(omega * dt)).as_quat(scalar_first=True)
    np.testing.assert_allclose(derivative, (advanced - quat) / dt, atol=2e-8, rtol=0)
    assert quat @ derivative == pytest.approx(0.0, abs=1e-14)


@pytest.mark.parametrize("pitch", [-np.pi / 2, np.pi / 2])
def test_vertical_pitch_has_finite_quaternion_rates(pitch):
    quat = quaternion_from_euler(0.3, pitch, -0.4)
    rates = kinematic_rates(quat, [2, -1, 0.5, 0.1, -0.2, 0.3])
    assert all(np.all(np.isfinite(rate)) for rate in rates)
    with pytest.warns(UserWarning, match="Gimbal lock"):
        angles = euler_from_quaternion(quat)
    np.testing.assert_allclose(
        body_to_ned_matrix(quaternion_from_euler(*angles)),
        body_to_ned_matrix(quat), atol=1e-14,
    )


@pytest.mark.parametrize("quat", [
    [0, 0, 0, 0], [1, 0, 0], [[1, 0], [0, 0]],
    [1, np.nan, 0, 0], [1, 0, np.inf, 0],
])
def test_invalid_quaternion_is_rejected(quat):
    with pytest.raises(ValueError):
        normalize_quaternion(quat)


def test_nonfinite_euler_angle_is_rejected():
    with pytest.raises(ValueError):
        quaternion_from_euler(0, np.nan, 0)
