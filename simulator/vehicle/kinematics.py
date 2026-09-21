"""BODY (forward, starboard, down) to NED kinematics.

Attitude uses Hamilton quaternions [qw, qx, qy, qz] rotating BODY to NED.
Euler angles are roll, pitch, yaw in radians: R = Rz(yaw) Ry(pitch) Rx(roll).
"""

import numpy as np
from numpy.typing import ArrayLike
from scipy.spatial.transform import Rotation

from .parameters import _vector


def normalize_quaternion(quaternion: ArrayLike) -> np.ndarray:
    """Return a unit quaternion; reject zero, malformed, or nonfinite inputs."""
    quat = _vector(quaternion, 4, "quaternion")
    scale = np.max(np.abs(quat))
    if scale == 0:
        raise ValueError("quaternion must have nonzero norm")
    # Scale first so finite, very large or small inputs can be normalized.
    quat = quat / scale
    return quat / np.linalg.norm(quat)


def quaternion_from_euler(
    roll_rad: float, pitch_rad: float, yaw_rad: float
) -> np.ndarray:
    """Convert roll, pitch, yaw in radians to a BODY-to-NED quaternion."""
    angles = _vector([roll_rad, pitch_rad, yaw_rad], 3, "Euler angles")
    return Rotation.from_euler("xyz", angles).as_quat(scalar_first=True)


def euler_from_quaternion(quaternion: ArrayLike) -> np.ndarray:
    """Return roll, pitch, yaw in radians, retaining SciPy's gimbal-lock warning."""
    quat = normalize_quaternion(quaternion)
    return Rotation.from_quat(quat, scalar_first=True).as_euler("xyz")


def body_to_ned_matrix(quaternion: ArrayLike) -> np.ndarray:
    """Return R such that velocity_ned = R @ velocity_body; R.T is its inverse."""
    quat = normalize_quaternion(quaternion)
    return Rotation.from_quat(quat, scalar_first=True).as_matrix()


def kinematic_rates(
    quaternion: ArrayLike, nu: ArrayLike
) -> tuple[np.ndarray, np.ndarray]:
    """Return NED position rate (3,) and quaternion rate (4,).

    nu = [u, v, w, p, q, r] contains BODY velocities in m/s and rad/s.
    BODY angular rates are not Euler-angle derivatives.
    """
    quat = normalize_quaternion(quaternion)
    velocity = _vector(nu, 6, "nu")
    omega = velocity[3:]
    position_dot = body_to_ned_matrix(quat) @ velocity[:3]
    quaternion_dot = 0.5 * np.concatenate(
        ([-quat[1:] @ omega], quat[0] * omega + np.cross(quat[1:], omega))
    )
    return position_dot, quaternion_dot
