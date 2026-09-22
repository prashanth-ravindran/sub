"""The scratch demonstration's cascaded waypoint PID controller."""

import math

from .actuators import trim_rpm
from .guidance import clamp, waypoint_error, wrap_pi


class PID:
    def __init__(self, kp, ki, kd, output_limit, integral_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.integral = 0.0

    def update(self, error, measurement_rate, dt):
        self.integral = clamp(
            self.integral + error * dt, -self.integral_limit, self.integral_limit
        )
        output = self.kp * error + self.ki * self.integral - self.kd * measurement_rate
        return clamp(output, -self.output_limit, self.output_limit)


class WaypointPIDController:
    def __init__(self, mission, dt):
        self.mission = mission
        self.dt = dt
        self.target_north, self.target_east = mission.target_ne()
        self.depth_pid = PID(0.040, 0.00035, 0.12, math.radians(22), 10.0)
        self.pitch_pid = PID(45.0, 1.0, 16.0, 20.0, 3.0)
        self.heading_pid = PID(28.0, 0.55, 18.0, 20.0, 3.0)
        self.speed_pid = PID(320.0, 35.0, 45.0, 1200.0, 20.0)

    def command(self, telemetry):
        north, east, depth = telemetry["position"]
        _, pitch, yaw = telemetry["euler"]
        u, _, _, _, q_rate, r_rate = telemetry["velocity"]
        depth_rate = telemetry["depth_rate"]

        heading_setpoint, distance = waypoint_error(
            self.target_north, self.target_east, north, east
        )
        depth_error = self.mission.depth_m - depth
        dive_pitch = self.depth_pid.update(depth_error, depth_rate, self.dt)
        pitch_setpoint = -dive_pitch
        elevator = self.pitch_pid.update(pitch_setpoint - pitch, q_rate, self.dt)
        rudder = self.heading_pid.update(wrap_pi(heading_setpoint - yaw), r_rate, self.dt)
        rpm = trim_rpm(self.mission.speed_mps) + self.speed_pid.update(
            self.mission.speed_mps - u, telemetry["surge_accel"], self.dt
        )
        return {
            "rpm": clamp(rpm, 0.0, 3000.0),
            "elevator_deg": clamp(elevator, -20.0, 20.0),
            "rudder_deg": clamp(rudder, -20.0, 20.0),
            "depth_setpoint": self.mission.depth_m,
            "pitch_setpoint": pitch_setpoint,
            "heading_setpoint": heading_setpoint,
            "speed_setpoint": self.mission.speed_mps,
            "distance": distance,
            "arrived": distance <= self.mission.arrival_radius_m,
        }
