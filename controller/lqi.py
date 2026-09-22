"""Integral-augmented LQR, including scratch anti-windup behavior."""

import math

import numpy as np

from .actuators import saturate
from .lqr import WaypointLQRController


class WaypointLQIController(WaypointLQRController):
    def __init__(self, mission, dt):
        super().__init__(mission, dt)
        self.dt = dt
        self.tracking_indices = [0, 4, 3]
        self.integral = np.zeros(3)
        self.integral_limits = np.array([6.0, 20.0, 2 * math.pi])
        tracking = np.eye(10)[self.tracking_indices]
        self.A_d = np.block([
            [self.A_d, np.zeros((10, 3))],
            [dt * tracking, np.eye(3)],
        ])
        self.B_d = np.vstack((self.B_d, np.zeros((3, 3))))
        self.Q[0, 0] = 1.0
        self.Q[2, 2] = 1.0 / math.radians(1.5)**2
        integral_scales = np.array([30.0, 5.0, math.radians(360)])
        self.Q = np.diag(np.r_[np.diag(self.Q), 1.0 / integral_scales**2])
        self._compute_gain()

    def actuator_command(self, error):
        raw = self.trim_command - self.K @ np.r_[error, self.integral]
        saturated = saturate(raw)
        increment = self.dt * error[self.tracking_indices]
        input_increment = -self.K[:, 10:] * increment
        pushes_saturation = np.any(
            np.sign(raw - saturated)[:, None] * input_increment > 1e-9, axis=0
        )
        self.integral = np.clip(
            self.integral + np.where(pushes_saturation, 0.0, increment),
            -self.integral_limits, self.integral_limits,
        )
        return saturated
