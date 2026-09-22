#!/usr/bin/env python3
"""Self-contained waypoint PID controller + LQR controller. Uses the 6 DOF AUV model 
from the simulator package

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


class WaypointLQRController:
    def __init__(self, mission, dt):
        from scipy.signal import cont2discrete

        mission.validate()
        if not (math.isfinite(dt) and 0 < dt <= 0.1):
            raise ValueError("dt must be finite and in (0, 0.1] seconds")
        self.mission = mission
        self.target_north, self.target_east = mission.target_ne()
        vehicle = AUV()
        p = vehicle.p
        drag = p.linear_u*mission.speed_mps + p.quad_u*mission.speed_mps**2
        trim_rpm = 60.0 * math.sqrt(drag / p.prop_k)
        if not 0 < trim_rpm < 3000:
            raise ValueError("Cruise speed must require strictly between 0 and 3000 RPM")
        self.trim_command = np.array([trim_rpm, 0.0, 0.0])
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
            return np.r_[derivative[2], 2.0*derivative[4:7], derivative[7:13]]

        epsilon = 1e-5
        self.A = np.column_stack([
            (reduced_derivative(delta, self.trim_command)
             - reduced_derivative(-delta, self.trim_command)) / (2*epsilon)
            for delta in epsilon * np.eye(10)
        ])
        input_steps = np.array([min(1e-3, trim_rpm/2, (3000-trim_rpm)/2), 1e-3, 1e-3])
        self.B = np.column_stack([
            (reduced_derivative(np.zeros(10), self.trim_command + delta)
             - reduced_derivative(np.zeros(10), self.trim_command - delta)) / (2*step)
            for delta, step in zip(np.diag(input_steps), input_steps)
        ])
        self.A_d, self.B_d, _, _, _ = cont2discrete(
            (self.A, self.B, np.eye(10), np.zeros((10, 3))), dt,
        )
        error_scales = np.array([
            5.0, math.radians(10), math.radians(15), math.radians(30),
            0.3, 0.5, 0.5, math.radians(10), math.radians(15), math.radians(15),
        ])
        self.Q = np.diag(1.0 / error_scales**2)
        self.R = np.diag(1.0 / np.array([800.0, 20.0, 20.0])**2)
        self._compute_gain()

    def _compute_gain(self):
        from scipy.linalg import solve_discrete_are

        riccati = solve_discrete_are(self.A_d, self.B_d, self.Q, self.R)
        self.K = np.linalg.solve(
            self.R + self.B_d.T @ riccati @ self.B_d,
            self.B_d.T @ riccati @ self.A_d,
        )

    def actuator_command(self, error):
        return np.clip(
            self.trim_command - self.K @ error,
            [0.0, -20.0, -20.0], [3000.0, 20.0, 20.0],
        )

    def command(self, telemetry):
        north, east, depth = telemetry["position"]
        roll, pitch, yaw = telemetry["euler"]
        delta_n = self.target_north - north
        delta_e = self.target_east - east
        heading_setpoint = math.atan2(delta_e, delta_n)
        distance = math.hypot(delta_n, delta_e)
        error = np.array([
            depth - self.mission.depth_m, roll, pitch,
            wrap_pi(yaw - heading_setpoint), *telemetry["velocity"],
        ])
        error[4] -= self.mission.speed_mps
        rpm, elevator, rudder = self.actuator_command(error)
        return {
            "rpm": float(rpm),
            "elevator_deg": float(elevator),
            "rudder_deg": float(rudder),
            "depth_setpoint": self.mission.depth_m,
            "pitch_setpoint": 0.0,
            "heading_setpoint": heading_setpoint,
            "speed_setpoint": self.mission.speed_mps,
            "distance": distance,
            "arrived": distance <= self.mission.arrival_radius_m,
        }


class WaypointLQIController(WaypointLQRController):
    def __init__(self, mission, dt):
        super().__init__(mission, dt)
        self.dt = dt
        self.tracking_indices = [0, 4, 3]
        self.integral = np.zeros(3)
        self.integral_limits = np.array([6.0, 20.0, 2*math.pi])
        tracking = np.eye(10)[self.tracking_indices]
        self.A_d = np.block([
            [self.A_d, np.zeros((10, 3))],
            [dt*tracking, np.eye(3)],
        ])
        self.B_d = np.vstack((self.B_d, np.zeros((3, 3))))
        self.Q[0, 0] = 1.0
        self.Q[2, 2] = 1.0 / math.radians(1.5)**2
        integral_scales = np.array([30.0, 5.0, math.radians(360)])
        self.Q = np.diag(np.r_[np.diag(self.Q), 1.0 / integral_scales**2])
        self._compute_gain()

    def actuator_command(self, error):
        raw = self.trim_command - self.K @ np.r_[error, self.integral]
        saturated = np.clip(raw, [0.0, -20.0, -20.0], [3000.0, 20.0, 20.0])
        increment = self.dt * error[self.tracking_indices]
        input_increment = -self.K[:, 10:] * increment
        pushes_saturation = np.any(np.sign(raw - saturated)[:, None] * input_increment > 1e-9, axis=0)
        self.integral = np.clip(
            self.integral + np.where(pushes_saturation, 0.0, increment),
            -self.integral_limits, self.integral_limits,
        )
        return saturated


def controller_process(connection, controller):
    """IPC worker: telemetry in, actuator command out, until None is sent."""
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


def run_closed_loop(mission, duration, dt, controller_type="pid"):
    """Run simulator in this process and controller in an IPC worker."""
    mission.validate()
    if not (math.isfinite(duration) and duration > 0):
        raise ValueError("Duration must be finite and positive")
    if not (math.isfinite(dt) and 0 < dt <= 0.1):
        raise ValueError("dt must be finite and in (0, 0.1] seconds")
    if controller_type not in ("pid", "lqr", "lqi"):
        raise ValueError("Controller must be 'pid', 'lqr', or 'lqi'")
    controller_class = {
        "pid": WaypointPIDController, "lqr": WaypointLQRController, "lqi": WaypointLQIController,
    }[controller_type]
    controller = controller_class(mission, dt)

    context = mp.get_context("spawn")
    simulator_end, controller_end = context.Pipe(duplex=True)
    worker = context.Process(
        target=controller_process, args=(controller_end, controller),
        name=f"auv-{controller_type}-controller",
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


def plot_results(rows, mission, controller_type="pid"):
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

    fig.suptitle(f"6-DOF AUV waypoint {controller_type.upper()} demonstration", fontsize=15)
    plt.show()
    plt.close(fig)


def self_test(controller_type="pid"):
    assert abs(wrap_pi(3*math.pi) + math.pi) < 1e-12
    north, east = latlon_to_ne(0.0, 1.0, 0.0, 0.0)
    assert abs(north) < 1e-12 and 111_000 < east < 112_000
    mission = Mission(target_lat=12.9716, target_lon=80.2220)
    mission.validate()
    heading = math.atan2(mission.target_ne()[1], mission.target_ne()[0])
    assert abs(math.degrees(heading) - 90.0) < 1e-6
    if controller_type in ("lqr", "lqi"):
        mission = Mission()
        controller_class = WaypointLQIController if controller_type == "lqi" else WaypointLQRController
        controller = controller_class(mission, 0.05)
        assert np.max(np.abs(np.linalg.eigvals(controller.A_d - controller.B_d @ controller.K))) < 1.0
        rows, arrived = run_closed_loop(mission, 450.0, 0.05, controller_type)
        data = np.asarray(rows)
        assert arrived and np.all(np.isfinite(data))
        assert np.all(data[:, FIELDS.index("depth_m")] >= 0)
        assert np.all((data[:, -3] >= 0) & (data[:, -3] <= 3000))
        assert np.all(np.abs(data[:, -2:]) <= 20)
        final = dict(zip(FIELDS, rows[-1]))
        assert abs(final["depth_m"] - mission.depth_m) < 0.1
        assert abs(final["surge_speed_mps"] - mission.speed_mps) < 0.05
    if controller_type == "lqi":
        assert np.max(data[:, FIELDS.index("depth_m")]) - mission.depth_m < 0.1
        assert np.max(np.abs(data[:, FIELDS.index("pitch_deg")])) < 23.0
        settled_depth = data[data[:, FIELDS.index("time_s")] >= 65, FIELDS.index("depth_m")]
        assert np.all(np.abs(settled_depth - mission.depth_m) < 0.1)
        for axis in controller.tracking_indices:
            for sign in (-1, 1):
                controller.integral[:] = 0
                error = np.zeros(10)
                error[axis] = sign * 3.0
                for _ in range(10):
                    controller.actuator_command(error)
                np.testing.assert_array_equal(controller.integral, 0)

        controller.integral[:] = [0.0, -20.0, 0.0]
        error = np.zeros(10)
        error[4] = 0.1
        assert controller.actuator_command(error)[0] == 3000.0
        assert controller.integral[1] > -20.0

        mission = Mission(target_lat=13.1, target_lon=80.2209)
        controller = WaypointLQIController(mission, 0.1)
        vehicle = AUV(P(quad_u=2*P().quad_u))
        state = np.zeros(13)
        state[2], state[3], state[7] = mission.depth_m, 1.0, mission.speed_mps
        for _ in range(1800):
            command = controller.command(telemetry_from_state(state, state[7], 0.1))
            command["elevator_deg"] = clamp(command["elevator_deg"] + 2.0, -20, 20)
            command["rudder_deg"] = clamp(command["rudder_deg"] + 1.0, -20, 20)
            state = vehicle.step_rk4(state, command, 0.1)
            assert np.all(np.abs(controller.integral) <= controller.integral_limits)
        heading = math.atan2(controller.target_east - state[1], controller.target_north - state[0])
        assert abs(state[2] - mission.depth_m) < 1e-3
        assert abs(state[7] - mission.speed_mps) < 1e-3
        assert abs(wrap_pi(q_to_euler(state[3:7])[2] - heading)) < 1e-3
    print("Self-test passed")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", choices=("pid", "lqr", "lqi"), default="pid")
    parser.add_argument("--start-lat", type=float, default=12.9716)
    parser.add_argument("--start-lon", type=float, default=80.2209)
    parser.add_argument("--target-lat", type=float, default=12.9752)
    parser.add_argument("--target-lon", type=float, default=80.2246)
    parser.add_argument("--depth", type=float, default=10.0, help="Transit depth [m]")
    parser.add_argument("--speed", type=float, default=1.5, help="Transit speed [m/s]")
    parser.add_argument("--arrival-radius", type=float, default=15.0, help="Arrival radius [m]")
    parser.add_argument("--duration", type=float, default=450.0, help="Maximum simulated seconds")
    parser.add_argument("--dt", type=float, default=0.05, help="Simulation/control period [s]")
    parser.add_argument("--output-prefix")
    parser.add_argument("--self-test", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.self_test:
        self_test(args.controller)
        return

    mission = Mission(
        args.start_lat, args.start_lon, args.target_lat, args.target_lon,
        args.depth, args.speed, args.arrival_radius,
    )
    target_north, target_east = mission.target_ne()
    distance = math.hypot(target_north, target_east)
    bearing = math.degrees(math.atan2(target_east, target_north)) % 360.0
    print(f"Mission: {distance:.1f} m at initial bearing {bearing:.1f} deg")

    rows, arrived = run_closed_loop(mission, args.duration, args.dt, args.controller)
    prefix = Path(args.output_prefix or f"auv_{args.controller}_results")
    csv_path = prefix.with_suffix(".csv")
    write_csv(csv_path, rows)
    plot_results(rows, mission, args.controller)

    final = dict(zip(FIELDS, rows[-1]))
    status = "ARRIVED" if arrived else "TIME LIMIT"
    print(f"{status}: t={final['time_s']:.1f} s, range={final['distance_to_target_m']:.1f} m")
    print(f"Final: depth={final['depth_m']:.2f} m, heading={final['heading_deg']:.1f} deg, u={final['surge_speed_mps']:.2f} m/s")
    print(f"Wrote {csv_path}")


if __name__ == "__main__":
    mp.freeze_support()
    main()
