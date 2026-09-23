"""Regenerate the simulator and controller README figures from current runs."""

import math
from pathlib import Path
import tempfile
import time

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
import pandas as pd

from controller.controller import RunManager
from controller.guidance import wrap_pi
from controller.lqi import WaypointLQIController
from controller.lqr import WaypointLQRController
from controller.mission import Mission
from controller.scratch.controller import telemetry_from_state
from simulator.scratch.fossen import AUV, P, q_to_euler, submerged_ellipsoid
from simulator.simulation.simulator import run_simulator
from simulator.vehicle.parameters import VehicleParameters


ROOT = Path(__file__).resolve().parents[1]
COLORS = {"pid": "#d55e00", "lqr": "#0072b2", "lqi": "#009e73"}


def save(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    print(path.relative_to(ROOT))


def simulator_figures(directory):
    runs = {}
    for name, depth in (
        ("surge_step", 50.0),
        ("elevator_step", 50.0),
        ("rudder_step", 50.0),
        ("surface", 0.0),
    ):
        path = directory / f"{name}.csv"
        run_simulator(
            scenario="surge_step" if name == "surface" else name,
            initial_depth=depth,
            duration_s=30.0,
            frequency_hz=100.0,
            socket_path=str(directory / f"{name}.sock"),
            output=path,
        )
        runs[name] = pd.read_csv(path)

    fig, axes = plt.subplots(1, 3, figsize=(14, 3.7), constrained_layout=True)
    items = (
        ("surge_step", "u_mps", 2.0, "Surge: 1800 RPM", "Surge speed (m/s)", 1.0),
        ("elevator_step", "pitch_rad", 10.0, "Elevator: +1°", "Pitch (deg)", 180 / math.pi),
        ("rudder_step", "yaw_rad", 10.0, "Rudder: +3°", "Yaw (deg)", 180 / math.pi),
    )
    for ax, (name, field, step, title, ylabel, scale) in zip(axes, items):
        frame = runs[name]
        ax.plot(frame.simulation_time_s, frame[field] * scale, color="#0072b2", lw=2)
        ax.axvline(step, color="#d55e00", ls="--", lw=1.5, label="command step")
        ax.set(title=title, xlabel="Simulated time (s)", ylabel=ylabel, xlim=(0, 30))
        ax.grid(alpha=0.25)
    axes[0].legend(loc="lower right")
    save(fig, ROOT / "simulator/figures/step_responses.png")

    params = VehicleParameters()
    depth_grid = np.linspace(-0.2, 0.3, 501)
    fraction = [
        submerged_ellipsoid(
            depth, np.array([0.0, 0.0, 1.0]), params.hull_length_m,
            params.mass_kg / params.water_density_kg_m3, -params.cb_height_m,
        )[0]
        for depth in depth_grid
    ]
    surface = runs["surface"]
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.9), constrained_layout=True)
    axes[0].plot(surface.simulation_time_s, surface.depth_m, color="#0072b2", lw=2)
    axes[0].axvline(2.0, color="#d55e00", ls="--", label="1800 RPM step")
    axes[0].set(title="Surface-start surge scenario", xlabel="Simulated time (s)", ylabel="CG depth (m)", xlim=(0, 30))
    axes[0].legend()
    axes[1].plot(depth_grid, fraction, color="#009e73", lw=2)
    axes[1].axvline(0.0, color="#d55e00", ls="--", label="CG at waterline")
    axes[1].set(title="Level-hull buoyancy model", xlabel="CG depth (m)", ylabel="Submerged volume fraction", ylim=(-0.03, 1.03))
    axes[1].legend()
    for ax in axes:
        ax.grid(alpha=0.25)
    assert surface.depth_m.iloc[0] > 0 and fraction[0] == 0 and fraction[-1] == 1
    save(fig, ROOT / "simulator/figures/surface_buoyancy.png")


def controller_figures():
    runs = {}
    manager = RunManager()
    try:
        for mode in COLORS:
            run = manager.start({"controller_type": mode, "simulator_frequency_hz": 100})
            deadline = time.monotonic() + 240
            while time.monotonic() < deadline:
                status = manager.status(run["run_id"])
                if status["status"] != "running":
                    break
                time.sleep(0.05)
            assert status["status"] == "completed" and status["completion_reason"] == "goal", status
            runs[mode] = pd.read_csv(manager.result_path(run["run_id"]))
            frame = runs[mode]
            assert frame.depth_m.iloc[0] == 0 and frame.distance_to_target_m.iloc[-1] <= 15
            assert frame.rpm.between(0, 3000).all()
            assert frame.elevator_deg.abs().le(20).all() and frame.rudder_deg.abs().le(20).all()
            print(f"{mode}: arrived at {frame.time_s.iloc[-1]:.2f} s")
    finally:
        manager.close()

    fig, axes = plt.subplots(2, 2, figsize=(11, 7.5), constrained_layout=True)
    signals = (
        (axes[0, 0], "depth_m", "depth_setpoint_m", "Depth", "Depth (m)"),
        (axes[0, 1], "heading_deg", "heading_setpoint_deg", "Heading", "Heading (deg)"),
        (axes[1, 0], "surge_speed_mps", "speed_setpoint_mps", "Surge speed", "Speed (m/s)"),
    )
    for ax, actual, desired, title, ylabel in signals:
        for mode, frame in runs.items():
            ax.plot(frame.time_s, frame[actual], color=COLORS[mode], lw=1.8, label=mode.upper())
        reference = runs["pid"]
        ax.plot(reference.time_s, reference[desired], color="black", ls="--", lw=1.2, label="requested")
        ax.set(title=title, xlabel="Simulated time (s)", ylabel=ylabel, xlim=(0, 380))
        ax.grid(alpha=0.25)
    axes[0, 0].legend(ncol=2)
    target_north, target_east = Mission().target_ne()
    track = axes[1, 1]
    for mode, frame in runs.items():
        track.plot(frame.east_m, frame.north_m, color=COLORS[mode], lw=1.8, label=mode.upper())
    track.add_patch(Circle((target_east, target_north), Mission().arrival_radius_m, fill=False, color="#777777", ls="--"))
    track.plot(0, 0, "ko", ms=4, label="start")
    track.plot(target_east, target_north, "k*", ms=10, label="target")
    track.set(title="Horizontal waypoint track", xlabel="East (m)", ylabel="North (m)")
    track.set_aspect("equal", adjustable="box")
    track.grid(alpha=0.25)
    track.legend(loc="upper left")
    save(fig, ROOT / "controller/figures/mission_tracking.png")

    frame = runs["pid"]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    signals = (
        (axes[0, 0], "pitch_deg", "Pitch (deg)", "PID pitch"),
        (axes[0, 1], "rpm", "RPM", "PID propeller"),
        (axes[1, 0], "elevator_deg", "Deflection (deg)", "PID elevator"),
        (axes[1, 1], "rudder_deg", "Deflection (deg)", "PID rudder"),
    )
    for ax, field, ylabel, title in signals:
        ax.plot(frame.time_s, frame[field], color="#d55e00", lw=1.7, label="actual / command")
        ax.set(title=title, xlabel="Simulated time (s)", ylabel=ylabel, xlim=(0, 380))
        ax.grid(alpha=0.25)
    axes[0, 0].plot(frame.time_s, frame.pitch_setpoint_deg, color="black", ls="--", lw=1.2, label="requested")
    axes[0, 0].legend()
    save(fig, ROOT / "controller/figures/pid_actuators.png")


def bias_figure():
    mission = Mission(target_lat=13.1, target_lon=80.2209)
    runs = {}
    for mode, controller_class in (
        ("lqr", WaypointLQRController),
        ("lqi", WaypointLQIController),
    ):
        controller = controller_class(mission, 0.1)
        vehicle = AUV(P(quad_u=2 * P().quad_u))
        state = np.zeros(13)
        state[2], state[3], state[7] = mission.depth_m, 1.0, mission.speed_mps
        errors = []
        for step in range(1801):
            heading = math.atan2(controller.target_east - state[1], controller.target_north - state[0])
            errors.append((
                step * 0.1,
                state[2] - mission.depth_m,
                state[7] - mission.speed_mps,
                math.degrees(wrap_pi(q_to_euler(state[3:7])[2] - heading)),
            ))
            if step == 1800:
                break
            command = controller.command(telemetry_from_state(state, state[7], 0.1))
            command["elevator_deg"] = float(np.clip(command["elevator_deg"] + 2.0, -20, 20))
            command["rudder_deg"] = float(np.clip(command["rudder_deg"] + 1.0, -20, 20))
            state = vehicle.step_rk4(state, command, 0.1)
        runs[mode] = np.array(errors)
    assert np.max(np.abs(runs["lqi"][-1, 1:])) < 0.001
    assert np.max(np.abs(runs["lqr"][-1, 1:])) > 0.1

    fig, axes = plt.subplots(1, 3, figsize=(14, 3.7), constrained_layout=True)
    for ax, index, title, ylabel in zip(
        axes,
        (1, 2, 3),
        ("Depth error", "Surge-speed error", "Heading error"),
        ("Depth − target (m)", "Speed − target (m/s)", "Yaw − bearing (deg)"),
    ):
        for mode, data in runs.items():
            ax.plot(data[:, 0], data[:, index], color=COLORS[mode], lw=1.8, label=mode.upper())
        ax.axhline(0, color="black", ls="--", lw=1)
        ax.set(title=title, xlabel="Simulated time (s)", ylabel=ylabel, xlim=(0, 180))
        ax.grid(alpha=0.25)
    axes[0].legend()
    save(fig, ROOT / "controller/figures/lqr_lqi_bias.png")


def main():
    plt.rcParams.update({"font.size": 11, "axes.titlesize": 12})
    with tempfile.TemporaryDirectory(prefix="auv-readme-plots-") as directory:
        simulator_figures(Path(directory))
    controller_figures()
    bias_figure()


if __name__ == "__main__":
    main()
