"""The scratch controller's trim and actuator saturation rules."""

import math

import numpy as np

from simulator.scratch.fossen import P


def trim_rpm(speed_mps, parameters=None):
    p = P() if parameters is None else parameters
    drag = p.linear_u * speed_mps + p.quad_u * speed_mps**2
    return 60.0 * math.sqrt(drag / p.prop_k)


def saturate(values):
    return np.clip(values, [0.0, -20.0, -20.0], [3000.0, 20.0, 20.0])

