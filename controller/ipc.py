"""The controller's only simulator communication path."""

import math

from common.ipc import UnixSeqPacketClient
from common.messages import (
    make_actuator_command, make_simulation_stop, validate_vehicle_state_or_raise,
)

from .config import COMMAND_VALIDITY_NS


def telemetry_from_message(message, previous_u=0.0, dt=0.05):
    """Adapt the version-1 state packet to the scratch controller input."""
    validate_vehicle_state_or_raise(message)
    pos = message["position"]
    attitude = message["orientation"]
    linear = message["velocity_body"]
    angular = message["angular_velocity_body"]
    roll, pitch, yaw = (attitude[key] for key in ("roll_rad", "pitch_rad", "yaw_rad"))
    u, v, w = (linear[key] for key in ("u_mps", "v_mps", "w_mps"))
    p, q, r = (angular[key] for key in ("p_radps", "q_radps", "r_radps"))
    depth_rate = -math.sin(pitch) * u + math.cos(pitch) * (
        math.sin(roll) * v + math.cos(roll) * w
    )
    return {
        "position": [pos["north_m"], pos["east_m"], pos["down_m"]],
        "euler": (roll, pitch, yaw),
        "velocity": [u, v, w, p, q, r],
        "depth_rate": depth_rate,
        "surge_accel": (u - previous_u) / dt,
    }


class ControllerLink:
    def __init__(self, socket_path):
        self.client = UnixSeqPacketClient(socket_path, retry_interval=0.01)
        self.command_sequence = 0

    def connect(self, timeout_s=10.0):
        self.client.connect(timeout_s=timeout_s)

    def receive_latest(self):
        message = self.client.receive_latest()
        if message is not None:
            validate_vehicle_state_or_raise(message)
        return message

    def send_command(self, command, state_sequence):
        packet = make_actuator_command(
            sequence=self.command_sequence,
            state_sequence=state_sequence,
            propeller_rpm=command["rpm"],
            elevator_deg=command["elevator_deg"],
            rudder_deg=command["rudder_deg"],
            validity_ns=COMMAND_VALIDITY_NS,
        )
        self.client.send(packet)
        self.command_sequence += 1

    def send_stop(self, state_sequence):
        self.client.send(make_simulation_stop(state_sequence=state_sequence))

    def close(self):
        self.client.close()

