"""The modular control laws must retain the scratch script's behavior."""

import math

import numpy as np
import pytest

from controller.ipc import telemetry_from_message
from controller.config import validate_run
from controller.lqi import WaypointLQIController
from controller.lqr import WaypointLQRController
from controller.mission import Mission
from controller.pid import WaypointPIDController
from controller.scratch.controller import (
    Mission as ReferenceMission,
    WaypointLQIController as ReferenceLQI,
    WaypointLQRController as ReferenceLQR,
    WaypointPIDController as ReferencePID,
    telemetry_from_state,
)
from simulator.ipc.messages import state_message
from simulator.vehicle.kinematics import quaternion_from_euler


@pytest.mark.parametrize("actual,reference", [
    (WaypointPIDController, ReferencePID),
    (WaypointLQRController, ReferenceLQR),
    (WaypointLQIController, ReferenceLQI),
])
def test_controller_commands_match_scratch_over_multiple_updates(actual, reference):
    controller = actual(Mission(), 0.05)
    original = reference(ReferenceMission(), 0.05)
    if hasattr(controller, "K"):
        np.testing.assert_array_equal(controller.K, original.K)
        np.testing.assert_array_equal(controller.A_d, original.A_d)
        np.testing.assert_array_equal(controller.B_d, original.B_d)
    for time_s in (0.0, 0.05, 0.10, 0.15):
        telemetry = {
            "position": [time_s, 0.2 * time_s, 0.1 + time_s],
            "euler": (0.01, -0.02, 0.03),
            "velocity": [0.7 + time_s, 0.01, -0.02, 0.01, -0.02, 0.03],
            "depth_rate": -0.03, "surge_accel": 0.2,
        }
        result, expected = controller.command(telemetry), original.command(telemetry)
        assert result == pytest.approx(expected)
    if isinstance(controller, WaypointLQIController):
        np.testing.assert_array_equal(controller.integral, original.integral)


def test_ipc_telemetry_matches_scratch_body_to_ned_conversion():
    state = np.zeros(13)
    state[:3] = [10, 20, 5]
    state[3:7] = quaternion_from_euler(0.1, -0.2, 0.3)
    state[7:] = [1.2, -0.1, 0.2, 0.02, -0.03, 0.04]
    packet = state_message(state, 5, 0.05)
    actual = telemetry_from_message(packet, previous_u=1.1, dt=0.05)
    expected = telemetry_from_state(state, previous_u=1.1, dt=0.05)
    np.testing.assert_allclose(actual["position"], expected["position"], atol=1e-12)
    np.testing.assert_allclose(actual["euler"], expected["euler"], atol=1e-12)
    np.testing.assert_allclose(actual["velocity"], expected["velocity"], atol=1e-12)
    assert math.isclose(actual["depth_rate"], expected["depth_rate"], abs_tol=1e-12)
    assert math.isclose(actual["surge_accel"], expected["surge_accel"], abs_tol=1e-12)


@pytest.mark.parametrize("payload", [
    {"controller_type": "unknown"},
    {"duration_s": True},
    {"duration_s": 0},
    {"mission": {"target_lat": 91}},
    {"mission": {"arrival_radius_m": 1e9}},
    {"mission": {"speed_mps": 0}},
    {"mission": {"start_lon": float("nan")}},
    {"simulator_frequency_hz": 50},
    {"simulator_frequency_hz": True},
    {"simulator_frequency_hz": "60"},
])
def test_invalid_controller_runs_are_rejected_before_start(payload):
    with pytest.raises(ValueError):
        validate_run(payload)
