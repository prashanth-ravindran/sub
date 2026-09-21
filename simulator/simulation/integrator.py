"""Classical fourth-order Runge–Kutta with fixed actuator commands per step."""

from collections.abc import Callable

import numpy as np
from numpy.typing import ArrayLike

from simulator.vehicle.actuators import actuator_forces
from simulator.vehicle.dynamics import state_derivative
from simulator.vehicle.kinematics import normalize_quaternion
from simulator.vehicle.parameters import VehicleParameters, _vector


def rk4_step(
    rhs: Callable, time_s: float, state: ArrayLike, dt: float
) -> np.ndarray:
    """Advance x_dot = rhs(t, x) by positive dt without modifying the input."""
    if not np.isfinite(time_s) or not np.isfinite(dt) or dt <= 0:
        raise ValueError("time_s must be finite and dt must be finite and positive")
    state = np.array(state, dtype=float, copy=True)
    if state.ndim != 1 or state.size == 0:
        raise ValueError("state must be a nonempty vector")
    state = _vector(state, state.size, "state")
    k1 = _vector(rhs(time_s, state), state.size, "derivative")
    k2 = _vector(rhs(time_s + dt / 2, state + dt * k1 / 2), state.size, "derivative")
    k3 = _vector(rhs(time_s + dt / 2, state + dt * k2 / 2), state.size, "derivative")
    k4 = _vector(rhs(time_s + dt, state + dt * k3), state.size, "derivative")
    return _vector(state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6, state.size, "next state")


def step_vehicle(
    state: ArrayLike, actuators: ArrayLike, time_s: float, dt: float,
    parameters: VehicleParameters,
) -> np.ndarray:
    """Integrate one submerged-vehicle step, recomputing forces at each RK stage."""
    state = _vector(state, 13, "state")
    if state[2] < 0:
        raise ValueError("Vehicle is above the water surface")
    actuators = _vector(actuators, 3, "actuators").copy()

    def rhs(t, intermediate):
        tau = actuator_forces(intermediate[7:], actuators, parameters)
        return state_derivative(intermediate, tau, parameters)

    with np.errstate(over="raise", invalid="raise", divide="raise"):
        result = rk4_step(rhs, time_s, state, dt)
    result[3:7] = normalize_quaternion(result[3:7])
    if result[2] < 0:
        raise ValueError("Vehicle crossed the water surface; submerged model no longer applies")
    return result
