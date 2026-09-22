"""Defaults shared by the controller service and its UI."""

from dataclasses import asdict
import math

from .mission import Mission

SIMULATOR_HZ = 100.0
CONTROL_DT = 0.05
DURATION_S = 450.0
INITIAL_DEPTH_M = 0.0
CONTROLLER_TIMEOUT_S = 10.0
COMMAND_VALIDITY_NS = 5_000_000_000
API_PORT = 8766

CONTROLLERS = ("pid", "lqr", "lqi")


def run_defaults():
    return {"mission": asdict(Mission()), "controller_type": "pid", "duration_s": DURATION_S}


def validate_run(payload):
    if not isinstance(payload, dict):
        raise ValueError("Request body must be a JSON object")
    if payload.keys() - {"mission", "controller_type", "duration_s"}:
        raise ValueError("Unknown run setting")
    mission_data = payload.get("mission", {})
    if not isinstance(mission_data, dict):
        raise ValueError("mission must be an object")
    defaults = asdict(Mission())
    if mission_data.keys() - defaults.keys():
        raise ValueError("Unknown mission field")
    for name, value in mission_data.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"{name} must be a finite number")
        defaults[name] = float(value)
    mission = Mission(**defaults)
    mission.validate()
    controller_type = payload.get("controller_type", "pid")
    if controller_type not in CONTROLLERS:
        raise ValueError("controller_type must be pid, lqr, or lqi")
    duration_s = payload.get("duration_s", DURATION_S)
    if isinstance(duration_s, bool) or not isinstance(duration_s, (int, float)) or not math.isfinite(duration_s) or duration_s <= 0:
        raise ValueError("duration_s must be finite and positive")
    if not math.isfinite(duration_s * SIMULATOR_HZ):
        raise ValueError("duration_s produces too many simulator steps")
    return mission, controller_type, float(duration_s)
