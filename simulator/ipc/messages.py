"""Translate the internal quaternion state to the existing version-1 schema."""

from common.messages import make_vehicle_state
from simulator.navigation.geodetic import ned_to_geodetic
from simulator.vehicle.kinematics import euler_from_quaternion
from simulator.vehicle.parameters import _vector


def state_message(state, sequence, simulation_time_s, latitude=13.0, longitude=80.0):
    state = _vector(state, 13, "state")
    north, east, down = map(float, state[:3])
    lat, lon = ned_to_geodetic(north, east, latitude, longitude)
    roll, pitch, yaw = map(float, euler_from_quaternion(state[3:7]))
    u, v, w, p, q, r = map(float, state[7:])
    return make_vehicle_state(
        sequence=sequence, simulation_time_s=simulation_time_s,
        north_m=north, east_m=east, down_m=down,
        latitude_deg=lat, longitude_deg=lon,
        roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw,
        u_mps=u, v_mps=v, w_mps=w, p_radps=p, q_radps=q, r_radps=r,
        depth_m=down,
    )


def csv_row(message):
    """Flatten state fields, preserving their unit-bearing protocol names."""
    row = {key: message[key] for key in ("sequence", "timestamp_ns", "simulation_time_s", "depth_m")}
    for key in ("position", "geodetic", "orientation", "velocity_body", "angular_velocity_body"):
        row.update(message[key])
    return row
