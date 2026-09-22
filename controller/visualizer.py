"""Post-run plots for the controller UI."""

import numpy as np
import matplotlib.pyplot as plt


def plot_results(rows, mission, controller_type):
    """Return the scratch demonstration's nine-panel result figure."""
    data = {name: np.asarray([float(row[name]) for row in rows]) for name in (
        "time_s", "depth_m", "depth_setpoint_m", "heading_deg", "heading_setpoint_deg",
        "surge_speed_mps", "speed_setpoint_mps", "pitch_deg", "pitch_setpoint_deg",
        "roll_deg", "rpm", "elevator_deg", "rudder_deg", "east_m", "north_m",
    )}
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
    return fig
