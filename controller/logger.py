"""Per-control-cycle state, guidance and actuator CSV."""

import csv
import math


FIELDS = [
    "time_s", "state_sequence", "north_m", "east_m", "latitude_deg", "longitude_deg",
    "depth_setpoint_m", "depth_m", "heading_setpoint_deg", "heading_deg",
    "speed_setpoint_mps", "surge_speed_mps", "pitch_setpoint_deg", "pitch_deg",
    "roll_deg", "distance_to_target_m", "rpm", "elevator_deg", "rudder_deg",
    "v_mps", "w_mps", "p_radps", "q_radps", "r_radps",
]


def log_row(state, command):
    pos, geo = state["position"], state["geodetic"]
    angles, velocity, rates = (
        state["orientation"], state["velocity_body"], state["angular_velocity_body"]
    )
    return {
        "time_s": state["simulation_time_s"],
        "state_sequence": state["sequence"],
        "north_m": pos["north_m"], "east_m": pos["east_m"],
        "latitude_deg": geo["latitude_deg"], "longitude_deg": geo["longitude_deg"],
        "depth_setpoint_m": command["depth_setpoint"], "depth_m": state["depth_m"],
        "heading_setpoint_deg": math.degrees(command["heading_setpoint"]) % 360.0,
        "heading_deg": math.degrees(angles["yaw_rad"]) % 360.0,
        "speed_setpoint_mps": command["speed_setpoint"],
        "surge_speed_mps": velocity["u_mps"],
        "pitch_setpoint_deg": math.degrees(command["pitch_setpoint"]),
        "pitch_deg": math.degrees(angles["pitch_rad"]),
        "roll_deg": math.degrees(angles["roll_rad"]),
        "distance_to_target_m": command["distance"],
        "rpm": command["rpm"], "elevator_deg": command["elevator_deg"],
        "rudder_deg": command["rudder_deg"],
        "v_mps": velocity["v_mps"], "w_mps": velocity["w_mps"],
        "p_radps": rates["p_radps"], "q_radps": rates["q_radps"], "r_radps": rates["r_radps"],
    }


class RunLogger:
    def __init__(self, path):
        self.file = open(path, "x", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()

    def write(self, state, command):
        self.writer.writerow(log_row(state, command))

    def close(self):
        self.file.close()

