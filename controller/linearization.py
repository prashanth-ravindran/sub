"""Exact numerical plant linearization from the scratch controller."""

import math

import numpy as np
from scipy.signal import cont2discrete

from simulator.scratch.fossen import AUV

from .actuators import trim_rpm


def linearize(mission, dt):
    mission.validate()
    if not (math.isfinite(dt) and 0 < dt <= 0.1):
        raise ValueError("dt must be finite and in (0, 0.1] seconds")
    vehicle = AUV()
    trim = trim_rpm(mission.speed_mps, vehicle.p)
    if not 0 < trim < 3000:
        raise ValueError("Cruise speed must require strictly between 0 and 3000 RPM")
    trim_command = np.array([trim, 0.0, 0.0])
    trim_state = np.zeros(13)
    trim_state[2] = mission.depth_m
    trim_state[3] = 1.0
    trim_state[7] = mission.speed_mps
    state_map = np.zeros((13, 10))
    state_map[2, 0] = 1.0
    state_map[4:7, 1:4] = 0.5 * np.eye(3)
    state_map[7:13, 4:10] = np.eye(6)

    def reduced_derivative(error, actuators):
        derivative = vehicle.derivative(
            trim_state + state_map @ error,
            dict(zip(("rpm", "elevator_deg", "rudder_deg"), actuators)),
        )
        return np.r_[derivative[2], 2.0 * derivative[4:7], derivative[7:13]]

    epsilon = 1e-5
    A = np.column_stack([
        (reduced_derivative(delta, trim_command)
         - reduced_derivative(-delta, trim_command)) / (2 * epsilon)
        for delta in epsilon * np.eye(10)
    ])
    input_steps = np.array([min(1e-3, trim / 2, (3000 - trim) / 2), 1e-3, 1e-3])
    B = np.column_stack([
        (reduced_derivative(np.zeros(10), trim_command + delta)
         - reduced_derivative(np.zeros(10), trim_command - delta)) / (2 * step)
        for delta, step in zip(np.diag(input_steps), input_steps)
    ])
    A_d, B_d, _, _, _ = cont2discrete(
        (A, B, np.eye(10), np.zeros((10, 3))), dt,
    )
    return A, B, A_d, B_d, trim_command

