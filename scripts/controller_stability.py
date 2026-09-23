"""Reproduce the controller README's local poles and cruise-recovery figures.

Run from the repository root: python -m scripts.controller_stability
This offline experiment freezes waypoint bearing at north; it uses the production
control laws, telemetry conversion, and nonlinear RK4 vehicle integrator.
"""

import math

import numpy as np

from controller.config import CONTROL_DT, SIMULATOR_HZ
from controller.ipc import telemetry_from_message
from controller.linearization import linearize
from controller.lqi import WaypointLQIController
from controller.lqr import WaypointLQRController
from controller.mission import Mission
from controller.pid import WaypointPIDController
from scripts.generate_readme_plots import COLORS, ROOT, plt, save
from simulator.ipc.messages import state_message
from simulator.simulation.integrator import step_vehicle
from simulator.vehicle.kinematics import euler_from_quaternion, quaternion_from_euler
from simulator.vehicle.parameters import VehicleParameters


CLASSES = {"pid": WaypointPIDController, "lqr": WaypointLQRController, "lqi": WaypointLQIController}
DURATION_S = 300.0
# [depth, roll, pitch, yaw, surge, sway, heave, roll/pitch/yaw rates].
PERTURBATION = np.array([0.2, *np.radians([0.5, 1.0, 2.0]), 0.05, 0, 0, 0, 0, 0])


def cruise_model(mode):
    """Return the controller and a deterministic augmented-state update F(z)."""
    mission = Mission()
    controller = CLASSES[mode](mission, CONTROL_DT)
    controller.target_north, controller.target_east = 1000.0, 0.0
    parameters = VehicleParameters()
    _, _, ad, bd, trim = linearize(mission, CONTROL_DT)
    dimension = 10 + {"pid": 5, "lqr": 0, "lqi": 3}[mode]

    def step(z, nonlinear=False):
        # Reload memory on every call: finite-difference evaluations must not
        # inherit the integrals left by a preceding evaluation.
        if mode == "pid":
            loops = (controller.depth_pid, controller.pitch_pid,
                     controller.heading_pid, controller.speed_pid)
            for loop, value in zip(loops, z[10:14]):
                loop.integral = value
        elif mode == "lqi":
            controller.integral = z[10:].copy()

        state = np.zeros(13)
        state[2] = mission.depth_m + z[0]
        state[3:7] = quaternion_from_euler(*z[1:4])
        state[7:] = z[4:10]
        state[7] += mission.speed_mps
        previous_u = mission.speed_mps + (z[14] if mode == "pid" else z[4])
        telemetry = telemetry_from_message(state_message(state, 0, 0.0), previous_u, CONTROL_DT)
        # N/E are deliberately zero here: this fixes heading at zero and removes
        # horizontal translation from a cruise-regulation (not waypoint) test.
        command = controller.command(telemetry)
        actuators = np.array([command["rpm"], command["elevator_deg"], command["rudder_deg"]])
        memory = np.empty(0)
        if mode == "pid":
            memory = np.r_[[loop.integral for loop in loops], z[4]]
        elif mode == "lqi":
            memory = controller.integral.copy()

        if nonlinear:
            for k in range(round(CONTROL_DT * SIMULATOR_HZ)):
                state = step_vehicle(state, actuators, k / SIMULATOR_HZ,
                                     1 / SIMULATOR_HZ, parameters)
            error = np.r_[state[2] - mission.depth_m,
                          euler_from_quaternion(state[3:7]), state[7:]]
            error[4] -= mission.speed_mps
        else:
            error = ad @ z[:10] + bd @ (actuators - trim)
        return np.r_[error, memory], actuators

    return controller, dimension, step


def jacobian(step, dimension, epsilon=1e-5, nonlinear=False):
    return np.column_stack([
        (step(delta, nonlinear)[0] - step(-delta, nonlinear)[0]) / (2 * epsilon)
        for delta in epsilon * np.eye(dimension)
    ])


def analyze(mode):
    controller, dimension, step = cruise_model(mode)
    for nonlinear in (False, True):
        np.testing.assert_allclose(step(np.zeros(dimension), nonlinear)[0], 0, atol=1e-12)
    matrix = jacobian(step, dimension)
    np.testing.assert_allclose(matrix, jacobian(step, dimension, epsilon=1e-4), atol=1e-7)
    if mode != "pid":
        # Independently checks LQI's update order and all added integral states.
        np.testing.assert_allclose(matrix, controller.A_d - controller.B_d @ controller.K, atol=1e-7)
    sampled_matrix = jacobian(step, dimension, nonlinear=True)
    poles = np.linalg.eigvals(matrix)
    radius = np.max(np.abs(poles))
    sampled_radius = np.max(np.abs(np.linalg.eigvals(sampled_matrix)))
    assert radius < 1 and sampled_radius < 1
    assert abs(radius - sampled_radius) < 1e-6
    print(f"{mode.upper()}: {dimension} states; rho={radius:.10f}; "
          f"RK4 rho={sampled_radius:.10f}; "
          f"dominant-mode time constant={-CONTROL_DT / np.log(radius):.2f} s", flush=True)

    count = round(DURATION_S / CONTROL_DT)
    history = np.zeros((count + 1, dimension))
    linear = np.zeros_like(history)
    history[0, :10] = linear[0, :10] = PERTURBATION
    commands = np.empty((count, 3))
    for k in range(count):
        history[k + 1], commands[k] = step(history[k], nonlinear=True)
        linear[k + 1] = matrix @ linear[k]
    assert np.isfinite(history).all()
    assert (commands > [0, -20, -20]).all() and (commands < [3000, 20, 20]).all()
    if mode == "pid":
        assert (np.abs(history[:, 10:14]) < [10, 3, 3, 20]).all()
    elif mode == "lqi":
        assert (np.abs(history[:, 10:]) < controller.integral_limits).all()
    assert np.max(np.abs(history[-1, :5] / PERTURBATION[:5])) < 0.05
    print(f"{mode.upper()}: final [depth m, roll/pitch/yaw rad, speed m/s] errors "
          f"{history[-1, :5]}; actuator min={commands.min(axis=0)}, "
          f"max={commands.max(axis=0)}; no actuator or integral limits reached", flush=True)
    return {"poles": poles, "history": history, "linear": linear, "commands": commands}


def pole_figure(runs):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    angle = np.linspace(-math.pi, math.pi, 2000)
    for ax in axes:
        ax.plot(np.cos(angle), np.sin(angle), "k--", lw=1, label="Unit circle")
        ax.axhline(0, color="#aaaaaa", lw=0.7)
        ax.axvline(0, color="#aaaaaa", lw=0.7)
        ax.set(xlabel="Real part", ylabel="Imaginary part")
        ax.grid(alpha=0.2)
        for (mode, run), marker in zip(runs.items(), ("x", "o", "+")):
            poles = run["poles"]
            ax.plot(poles.real, poles.imag, linestyle="none", marker=marker,
                    markerfacecolor="none", ms=8, mew=1.6, color=COLORS[mode],
                    label=f"{mode.upper()}  max |λ| = {np.max(np.abs(poles)):.7f}")
    axes[0].set(title="Closed-loop poles: all modes", xlim=(-1.1, 1.1), ylim=(-1.1, 1.1))
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].legend(loc="upper left", fontsize=9)
    axes[1].set(title="Zoom: slow modes near +1", xlim=(0.990, 1.0005), ylim=(-0.003, 0.003))
    axes[1].ticklabel_format(useOffset=False)
    fig.suptitle("Fixed-reference cruise: 10 m depth, 1.5 m/s, 20 Hz control")
    save(fig, ROOT / "controller/figures/stability_poles.png")


def recovery_figure(runs):
    t = np.arange(round(DURATION_S / CONTROL_DT) + 1) * CONTROL_DT
    fig, axes = plt.subplots(2, 3, figsize=(13, 8), constrained_layout=True)
    signals = ((0, 1, "Depth error (m)"), (2, 180 / math.pi, "Pitch (deg)"),
               (3, 180 / math.pi, "Heading error (deg)"), (4, 1, "Surge-speed error (m/s)"),
               (1, 180 / math.pi, "Roll (deg)"))
    for ax, (index, scale, title) in zip(axes.flat, signals):
        for mode, run in runs.items():
            ax.plot(t, run["history"][:, index] * scale, color=COLORS[mode], lw=1.7, label=mode.upper())
            ax.plot(t, run["linear"][:, index] * scale, color=COLORS[mode], lw=1, ls="--")
        ax.axhline(0, color="#555555", lw=0.7)
        ax.set(title=title, xlabel="Simulated time (s)", xlim=(0, DURATION_S))
        ax.grid(alpha=0.2)
    ax = axes[1, 2]
    for mode, run in runs.items():
        for name, linestyle in (("history", "-"), ("linear", "--")):
            error = np.linalg.norm(run[name][:, :5] / PERTURBATION[:5], axis=1) / math.sqrt(5)
            ax.semilogy(t, np.maximum(error, 1e-12), color=COLORS[mode], ls=linestyle, lw=1.5)
    ax.set(title="Normalized tracking-error magnitude", xlabel="Simulated time (s)",
           ylabel="E(t) / E(0), logarithmic scale", xlim=(0, DURATION_S))
    ax.grid(alpha=0.2, which="both")
    axes[0, 0].legend()
    fig.suptitle("Small-disturbance recovery at cruise\nSolid: nonlinear RK4 simulation; dashed: local linear prediction")
    save(fig, ROOT / "controller/figures/stability_recovery.png")


def actuator_figure(runs):
    t = np.arange(round(DURATION_S / CONTROL_DT)) * CONTROL_DT
    fig, axes = plt.subplots(3, 2, figsize=(12, 9), constrained_layout=True)
    for index, (label, limits) in enumerate((("Propeller (RPM)", (0, 3000)),
                                           ("Elevator (deg)", (-20, 20)),
                                           ("Rudder (deg)", (-20, 20)))):
        for column, ax in enumerate(axes[index]):
            for mode, run in runs.items():
                ax.plot(t, run["commands"][:, index], color=COLORS[mode], lw=1.6, label=mode.upper())
            if column == 0:
                for limit in limits:
                    ax.axhline(limit, color="#555555", ls="--", lw=1)
            ax.set(ylabel=label, xlabel="Simulated time (s)",
                   xlim=(0, DURATION_S if column == 0 else 30))
            ax.grid(alpha=0.2)
    axes[0, 0].set_title("Complete run with actuator limits")
    axes[0, 1].set_title("First 30 seconds, expanded command scale")
    axes[0, 0].legend(loc="upper right", ncol=3)
    fig.suptitle("Commands during the same nonlinear disturbance-recovery experiment")
    save(fig, ROOT / "controller/figures/stability_actuators.png")


def main():
    plt.rcParams.update({"font.size": 10, "axes.titlesize": 11})
    runs = {mode: analyze(mode) for mode in CLASSES}
    pole_figure(runs)
    recovery_figure(runs)
    actuator_figure(runs)


if __name__ == "__main__":
    main()
