#!/usr/bin/env python3
"""Self-contained 6-DOF AUV + waypoint PID controller demonstration.

The simulator and controller run as separate OS processes.  They exchange
telemetry and actuator commands over a bidirectional multiprocessing Pipe,
which keeps the controller boundary explicit while retaining a one-file demo.

Install:  python -m pip install numpy matplotlib
Run:      python auv_pid_demo.py
Check:    python auv_pid_demo.py --self-test

Frames: NED (+north, +east, +down), BODY (+forward, +starboard, +down).
"""

import argparse
import csv
import math
import multiprocessing as mp
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from simulator.scratch.fossen import AUV, P, ned_to_latlon, q_to_R, q_to_euler


EARTH_RADIUS_M = 6_371_000.0


def clamp(value, low, high):
    return min(max(value, low), high)


def wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def latlon_to_ne(lat, lon, lat0, lon0):
    """Flat-Earth latitude/longitude to local north/east metres."""
    north = EARTH_RADIUS_M * math.radians(lat - lat0)
    east = EARTH_RADIUS_M * math.cos(math.radians(lat0)) * math.radians(lon - lon0)
    return north, east


@dataclass(frozen=True)
class Mission:
    start_lat: float = 12.9716
    start_lon: float = 80.2209
    target_lat: float = 12.9752
    target_lon: float = 80.2246
    depth_m: float = 10.0
    speed_mps: float = 1.5
    arrival_radius_m: float = 15.0

    def validate(self):
        values = (self.start_lat, self.start_lon, self.target_lat,
                  self.target_lon, self.depth_m, self.speed_mps,
                  self.arrival_radius_m)
        if not all(math.isfinite(v) for v in values):
            raise ValueError("Mission values must be finite")
        if not (-90 < self.start_lat < 90 and -90 < self.target_lat < 90):
            raise ValueError("Latitude must be strictly between -90 and 90 degrees")
        if not (-180 <= self.start_lon <= 180 and -180 <= self.target_lon <= 180):
            raise ValueError("Longitude must be between -180 and 180 degrees")
        if self.depth_m <= 0 or self.speed_mps <= 0 or self.arrival_radius_m <= 0:
            raise ValueError("Depth, speed and arrival radius must be positive")
        if math.hypot(*self.target_ne()) <= self.arrival_radius_m:
            raise ValueError("Target must be outside the arrival radius")

    def target_ne(self):
        return latlon_to_ne(
            self.target_lat, self.target_lon, self.start_lat, self.start_lon
        )


class PID:
    """PID with output/integral limits and derivative on measurement."""

    def __init__(self, kp, ki, kd, output_limit, integral_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.integral = 0.0

    def update(self, error, measurement_rate, dt):
        self.integral = clamp(
            self.integral + error*dt, -self.integral_limit, self.integral_limit
        )
        output = self.kp*error + self.ki*self.integral - self.kd*measurement_rate
        return clamp(output, -self.output_limit, self.output_limit)


class WaypointPIDController:
    """Cascaded depth/pitch plus heading and surge-speed PID loops.

    Gain selection is deliberately transparent: feed-forward balances the
    model's steady surge drag, then modest PID corrections remove error.
    Depth PID requests a bounded pitch; pitch PD turns that into elevator.
    All actuator outputs are saturated to the supplied model's limits.
    """

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

        delta_n = self.target_north - north
        delta_e = self.target_east - east
        heading_setpoint = math.atan2(delta_e, delta_n)
        distance = math.hypot(delta_n, delta_e)

        depth_error = self.mission.depth_m - depth
        dive_pitch = self.depth_pid.update(depth_error, depth_rate, self.dt)
        pitch_setpoint = -dive_pitch  # negative pitch is nose-down in NED
        elevator = self.pitch_pid.update(pitch_setpoint - pitch, q_rate, self.dt)

        heading_error = wrap_pi(heading_setpoint - yaw)
        rudder = self.heading_pid.update(heading_error, r_rate, self.dt)

        speed_error = self.mission.speed_mps - u
        p = P()
        drag = p.linear_u*self.mission.speed_mps + p.quad_u*self.mission.speed_mps**2
        feed_forward_rpm = 60.0 * math.sqrt(drag / p.prop_k)
        rpm = feed_forward_rpm + self.speed_pid.update(
            speed_error, telemetry["surge_accel"], self.dt
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


def controller_process(connection, mission, dt):
    """IPC worker: telemetry in, actuator command out, until None is sent."""
    controller = WaypointPIDController(mission, dt)
    try:
        while True:
            telemetry = connection.recv()
            if telemetry is None:
                break
            connection.send(controller.command(telemetry))
    except EOFError:
        pass
    finally:
        connection.close()


FIELDS = [
    "time_s", "north_m", "east_m", "latitude_deg", "longitude_deg",
    "depth_setpoint_m", "depth_m", "heading_setpoint_deg", "heading_deg",
    "speed_setpoint_mps", "surge_speed_mps", "pitch_setpoint_deg",
    "pitch_deg", "roll_deg", "distance_to_target_m", "rpm",
    "elevator_deg", "rudder_deg",
]


def telemetry_from_state(state, previous_u, dt):
    roll, pitch, yaw = q_to_euler(state[3:7])
    ned_velocity = q_to_R(state[3:7]) @ state[7:10]
    return {
        "position": state[:3].tolist(),
        "euler": (roll, pitch, yaw),
        "velocity": state[7:13].tolist(),
        "depth_rate": float(ned_velocity[2]),
        "surge_accel": float((state[7] - previous_u) / dt),
    }


def log_row(t, state, command, mission):
    roll, pitch, yaw = q_to_euler(state[3:7])
    lat, lon = ned_to_latlon(state[0], state[1], mission.start_lat, mission.start_lon)
    return [
        t, state[0], state[1], lat, lon,
        command["depth_setpoint"], state[2],
        math.degrees(command["heading_setpoint"]) % 360.0,
        math.degrees(yaw) % 360.0,
        command["speed_setpoint"], state[7],
        math.degrees(command["pitch_setpoint"]), math.degrees(pitch),
        math.degrees(roll), command["distance"], command["rpm"],
        command["elevator_deg"], command["rudder_deg"],
    ]


def run_closed_loop(mission, duration, dt):
    """Run simulator in this process and controller in an IPC worker."""
    mission.validate()
    if not (math.isfinite(duration) and duration > 0):
        raise ValueError("Duration must be finite and positive")
    if not (math.isfinite(dt) and 0 < dt <= 0.1):
        raise ValueError("dt must be finite and in (0, 0.1] seconds")

    context = mp.get_context("spawn")
    simulator_end, controller_end = context.Pipe(duplex=True)
    worker = context.Process(
        target=controller_process, args=(controller_end, mission, dt),
        name="auv-pid-controller",
    )
    worker.start()
    controller_end.close()

    vehicle = AUV()
    state = np.zeros(13)
    state[2] = 0.1
    state[3] = 1.0
    previous_u = 0.0
    rows = []
    arrived = False

    try:
        for step in range(math.ceil(duration / dt)):
            simulator_end.send(telemetry_from_state(state, previous_u, dt))
            if not simulator_end.poll(5.0):
                raise RuntimeError("Controller did not answer over IPC")
            command = simulator_end.recv()
            if command["arrived"]:
                arrived = True
                rows.append(log_row(step*dt, state, command, mission))
                break
            previous_u = float(state[7])
            state = vehicle.step_rk4(state, command, dt)
            rows.append(log_row((step + 1)*dt, state, command, mission))
    finally:
        if worker.is_alive():
            try:
                simulator_end.send(None)
            except (BrokenPipeError, EOFError):
                pass
        simulator_end.close()
        worker.join(timeout=5.0)
        if worker.is_alive():
            worker.terminate()
            worker.join()

    if not rows:
        raise RuntimeError("Simulation produced no samples")
    return rows, arrived


def write_csv(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(FIELDS)
        writer.writerows(rows)


def plot_results(rows, mission):
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("Plotting requires matplotlib: python -m pip install matplotlib") from exc

    data = {name: np.asarray([row[i] for row in rows]) for i, name in enumerate(FIELDS)}
    t = data["time_s"]
    fig, axes = plt.subplots(3, 3, figsize=(15, 11), constrained_layout=True)

    def time_plot(ax, actual, commanded, ylabel, title):
        ax.plot(t, commanded, "k--", label="commanded")
        ax.plot(t, actual, label="actual")
        ax.set(xlabel="Time [s]", ylabel=ylabel, title=title)
        ax.grid(True)
        ax.legend()

    time_plot(axes[0, 0], data["depth_m"], data["depth_setpoint_m"], "Depth [m]", "Depth")
    time_plot(axes[0, 1], data["heading_deg"], data["heading_setpoint_deg"], "Heading [deg]", "Heading")
    time_plot(axes[0, 2], data["surge_speed_mps"], data["speed_setpoint_mps"], "u [m/s]", "Surge speed")
    time_plot(axes[1, 0], data["pitch_deg"], data["pitch_setpoint_deg"], "Pitch [deg]", "Pitch")

    axes[1, 1].plot(t, data["roll_deg"])
    axes[1, 1].set(xlabel="Time [s]", ylabel="Roll [deg]", title="Roll")
    axes[1, 1].grid(True)

    for ax, field, label, title in (
        (axes[1, 2], "rpm", "Propeller [RPM]", "Propeller command"),
        (axes[2, 0], "elevator_deg", "Elevator [deg]", "Elevator command"),
        (axes[2, 1], "rudder_deg", "Rudder [deg]", "Rudder command"),
    ):
        ax.plot(t, data[field])
        ax.set(xlabel="Time [s]", ylabel=label, title=title)
        ax.grid(True)

    target_north, target_east = mission.target_ne()
    axes[2, 2].plot(data["east_m"], data["north_m"], label="vehicle track")
    axes[2, 2].plot(0.0, 0.0, "go", label="start")
    axes[2, 2].plot(target_east, target_north, "r*", ms=12, label="target")
    axes[2, 2].set(xlabel="East [m]", ylabel="North [m]", title="Horizontal trajectory")
    axes[2, 2].axis("equal")
    axes[2, 2].grid(True)
    axes[2, 2].legend()

    fig.suptitle("6-DOF AUV waypoint PID demonstration", fontsize=15)
    plt.show()
    plt.close(fig)


def self_test():
    assert abs(wrap_pi(3*math.pi) + math.pi) < 1e-12
    north, east = latlon_to_ne(0.0, 1.0, 0.0, 0.0)
    assert abs(north) < 1e-12 and 111_000 < east < 112_000
    mission = Mission(target_lat=12.9716, target_lon=80.2220)
    mission.validate()
    heading = math.atan2(mission.target_ne()[1], mission.target_ne()[0])
    assert abs(math.degrees(heading) - 90.0) < 1e-6
    print("Self-test passed")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--start-lat", type=float, default=12.9716)
    parser.add_argument("--start-lon", type=float, default=80.2209)
    parser.add_argument("--target-lat", type=float, default=12.9752)
    parser.add_argument("--target-lon", type=float, default=80.2246)
    parser.add_argument("--depth", type=float, default=10.0, help="Transit depth [m]")
    parser.add_argument("--speed", type=float, default=1.5, help="Transit speed [m/s]")
    parser.add_argument("--arrival-radius", type=float, default=15.0, help="Arrival radius [m]")
    parser.add_argument("--duration", type=float, default=450.0, help="Maximum simulated seconds")
    parser.add_argument("--dt", type=float, default=0.05, help="Simulation/control period [s]")
    parser.add_argument("--output-prefix", default="auv_pid_results")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.self_test:
        self_test()
        return

    mission = Mission(
        args.start_lat, args.start_lon, args.target_lat, args.target_lon,
        args.depth, args.speed, args.arrival_radius,
    )
    target_north, target_east = mission.target_ne()
    distance = math.hypot(target_north, target_east)
    bearing = math.degrees(math.atan2(target_east, target_north)) % 360.0
    print(f"Mission: {distance:.1f} m at initial bearing {bearing:.1f} deg")

    rows, arrived = run_closed_loop(mission, args.duration, args.dt)
    prefix = Path(args.output_prefix)
    csv_path = prefix.with_suffix(".csv")
    write_csv(csv_path, rows)
    plot_results(rows, mission)

    final = dict(zip(FIELDS, rows[-1]))
    status = "ARRIVED" if arrived else "TIME LIMIT"
    print(f"{status}: t={final['time_s']:.1f} s, range={final['distance_to_target_m']:.1f} m")
    print(f"Final: depth={final['depth_m']:.2f} m, heading={final['heading_deg']:.1f} deg, u={final['surge_speed_mps']:.2f} m/s")
    print(f"Wrote {csv_path}")


if __name__ == "__main__":
    mp.freeze_support()
    main()
