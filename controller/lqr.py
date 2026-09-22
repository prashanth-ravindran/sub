"""Discrete waypoint LQR with the scratch model's weights and limits."""

import math

import numpy as np
from scipy.linalg import solve_discrete_are

from .actuators import saturate
from .guidance import waypoint_error, wrap_pi
from .linearization import linearize


class WaypointLQRController:
    def __init__(self, mission, dt):
        self.mission = mission
        self.target_north, self.target_east = mission.target_ne()
        self.A, self.B, self.A_d, self.B_d, self.trim_command = linearize(mission, dt)
        error_scales = np.array([
            5.0, math.radians(10), math.radians(15), math.radians(30),
            0.3, 0.5, 0.5, math.radians(10), math.radians(15), math.radians(15),
        ])
        self.Q = np.diag(1.0 / error_scales**2)
        self.R = np.diag(1.0 / np.array([800.0, 20.0, 20.0])**2)
        self._compute_gain()

    def _compute_gain(self):
        riccati = solve_discrete_are(self.A_d, self.B_d, self.Q, self.R)
        self.K = np.linalg.solve(
            self.R + self.B_d.T @ riccati @ self.B_d,
            self.B_d.T @ riccati @ self.A_d,
        )

    def actuator_command(self, error):
        return saturate(self.trim_command - self.K @ error)

    def command(self, telemetry):
        north, east, depth = telemetry["position"]
        roll, pitch, yaw = telemetry["euler"]
        heading_setpoint, distance = waypoint_error(
            self.target_north, self.target_east, north, east
        )
        error = np.array([
            depth - self.mission.depth_m, roll, pitch,
            wrap_pi(yaw - heading_setpoint), *telemetry["velocity"],
        ])
        error[4] -= self.mission.speed_mps
        rpm, elevator, rudder = self.actuator_command(error)
        return {
            "rpm": float(rpm),
            "elevator_deg": float(elevator),
            "rudder_deg": float(rudder),
            "depth_setpoint": self.mission.depth_m,
            "pitch_setpoint": 0.0,
            "heading_setpoint": heading_setpoint,
            "speed_setpoint": self.mission.speed_mps,
            "distance": distance,
            "arrived": distance <= self.mission.arrival_radius_m,
        }

