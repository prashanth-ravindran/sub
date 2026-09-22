"""Streamlit client for completed Vehicle Simulator runs."""

from io import BytesIO
import json
import os
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import streamlit as st


API_URL = os.environ.get("VEHICLE_SIMULATOR_API_URL", "http://127.0.0.1:8765").rstrip("/")
AXES = ("surge", "sway", "heave", "roll", "pitch", "yaw")


def api(method, path, payload=None, *, csv=False):
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Content-Type": "application/json"} if body is not None else {}
    request = Request(API_URL + path, data=body, headers=headers, method=method)
    try:
        with urlopen(request, timeout=10) as response:
            data = response.read()
    except HTTPError as exc:
        try:
            detail = json.load(exc).get("error", exc.reason)
        except (ValueError, AttributeError):
            detail = exc.reason
        raise RuntimeError(str(detail)) from exc
    except URLError as exc:
        raise RuntimeError(f"Cannot reach the simulator API at {API_URL}: {exc.reason}") from exc
    return data if csv else json.loads(data)


@st.cache_data(max_entries=1, show_spinner="Loading completed run…")
def load_result(run_id):
    csv_bytes = api("GET", f"/api/runs/{run_id}/csv", csv=True)
    return csv_bytes, pd.read_csv(BytesIO(csv_bytes))


def parameter_inputs(defaults):
    """Render every current VehicleParameters field using API-supplied values."""
    selected = {}
    scalars = [(name, value) for name, value in defaults.items() if not isinstance(value, list)]
    vectors = [(name, value) for name, value in defaults.items() if isinstance(value, list)]
    columns = st.columns(2)
    for index, (name, value) in enumerate(scalars):
        with columns[index % 2]:
            selected[name] = st.number_input(name, value=float(value), format="%.6f", key=f"param_{name}")
    for name, values in vectors:
        st.markdown(f"**{name}**")
        labels = ("roll", "pitch", "yaw") if name == "inertia_kg_m2" else AXES
        selected[name] = []
        for start in range(0, len(values), 3):
            columns = st.columns(3)
            for offset, value in enumerate(values[start:start + 3]):
                index = start + offset
                with columns[offset]:
                    selected[name].append(st.number_input(
                        labels[index], value=float(value), format="%.6f",
                        key=f"param_{name}_{index}",
                    ))
    return selected


@st.fragment(run_every="1s")
def watch_run(run_id):
    try:
        status = api("GET", f"/api/runs/{run_id}")
    except RuntimeError as exc:
        st.error(str(exc))
        return
    if status["status"] == "running":
        st.info("Simulation running. Charts will appear when it finishes.")
        return
    st.session_state["active_run_id"] = None
    st.session_state["finished_run"] = status
    st.rerun(scope="app")


def show_result(status):
    if status["status"] == "failed":
        st.error(f"Simulation failed: {status['error']}")
        return
    run_id = status["run_id"]
    try:
        csv_bytes, frame = load_result(run_id)
    except RuntimeError as exc:
        st.error(str(exc))
        return
    stats = status["stats"]
    st.subheader("Run result")
    st.caption(f"Completed by {status['completion_reason']} · {stats['steps']:,} steps")
    summary = st.columns(3)
    summary[0].metric("Simulated time", f"{stats['simulation_time_s']:.2f} s")
    summary[1].metric("Wall time", f"{stats['wall_time_s']:.2f} s")
    summary[2].metric("Achieved rate", f"{stats['achieved_hz']:.1f} Hz")
    st.download_button(
        "Download state CSV", csv_bytes, file_name=f"vehicle-{run_id}.csv",
        mime="text/csv",
    )

    if frame.empty:
        st.info("The run ended before its first simulation step.")
        return
    chart_frame = frame
    if len(frame) > 5000:
        chart_frame = frame.iloc[np.linspace(0, len(frame) - 1, 5000, dtype=int)]
        st.caption("Charts show 5,000 samples; the CSV contains every step.")
    final = frame.iloc[-1]
    position = st.columns(4)
    position[0].metric("North", f"{final['north_m']:.2f} m")
    position[1].metric("East", f"{final['east_m']:.2f} m")
    position[2].metric("Depth", f"{final['depth_m']:.2f} m")
    position[3].metric("Forward speed", f"{final['u_mps']:.2f} m/s")
    st.caption(f"Final location: {final['latitude_deg']:.6f}°, {final['longitude_deg']:.6f}°")

    st.subheader("Depth")
    st.line_chart(chart_frame, x="simulation_time_s", y="depth_m", x_label="Simulation time (s)", y_label="Depth (m)")
    angles = pd.DataFrame({
        "simulation_time_s": chart_frame["simulation_time_s"],
        "Roll (deg)": np.degrees(chart_frame["roll_rad"]),
        "Pitch (deg)": np.degrees(chart_frame["pitch_rad"]),
        "Yaw (deg)": np.degrees(chart_frame["yaw_rad"]),
    })
    st.subheader("Attitude")
    st.line_chart(angles, x="simulation_time_s", y=["Roll (deg)", "Pitch (deg)", "Yaw (deg)"], x_label="Simulation time (s)", y_label="Angle (deg)")
    st.subheader("Body velocity")
    st.line_chart(chart_frame, x="simulation_time_s", y=["u_mps", "v_mps", "w_mps"], x_label="Simulation time (s)", y_label="Velocity (m/s)")
    st.subheader("North–East track")
    figure, axes = plt.subplots()
    axes.plot(chart_frame["east_m"], chart_frame["north_m"])
    axes.scatter([frame["east_m"].iloc[0], frame["east_m"].iloc[-1]],
                 [frame["north_m"].iloc[0], frame["north_m"].iloc[-1]])
    axes.set_xlabel("East (m)")
    axes.set_ylabel("North (m)")
    axes.set_aspect("equal", adjustable="datalim")
    st.pyplot(figure)
    plt.close(figure)


st.set_page_config(page_title="Vehicle Simulator", layout="wide")
st.title("Vehicle Simulator")
st.caption("Choose a scenario and vehicle parameters, then review the finished run.")

try:
    config = api("GET", "/api/config")
except RuntimeError as exc:
    st.error(str(exc))
    st.stop()

defaults = config["run_defaults"]
scenarios = config["scenarios"]
with st.form("simulation_settings"):
    st.subheader("Simulation")
    choices = scenarios + ["External controller"]
    scenario = st.selectbox("Input source", choices, index=choices.index(defaults["scenario"]))
    a, b, c = st.columns(3)
    with a:
        duration_s = st.number_input("Time horizon (s)", min_value=0.01, value=float(defaults["duration_s"]))
        initial_depth = st.number_input("Initial depth (m)", min_value=0.0, value=float(defaults["initial_depth"]))
    with b:
        frequency_hz = st.number_input("Simulation rate (Hz)", min_value=0.01, value=float(defaults["frequency_hz"]))
        latitude = st.number_input("Origin latitude (deg)", value=float(defaults["latitude"]), format="%.6f")
    with c:
        fast = st.checkbox("Accelerated time", value=bool(defaults["fast"]))
        longitude = st.number_input("Origin longitude (deg)", value=float(defaults["longitude"]), format="%.6f")
    with st.expander("Vehicle parameters", expanded=False):
        st.caption("Units are in field names or the simulator documentation. Six-element vectors use surge, sway, heave, roll, pitch, yaw order; inertia uses roll, pitch, yaw.")
        parameters = parameter_inputs(config["vehicle_parameters"])
    submitted = st.form_submit_button("Start simulation", disabled=bool(st.session_state.get("active_run_id")))

if submitted:
    request = {
        "scenario": None if scenario == "External controller" else scenario,
        "duration_s": duration_s,
        "frequency_hz": frequency_hz,
        "fast": fast,
        "initial_depth": initial_depth,
        "latitude": latitude,
        "longitude": longitude,
        "vehicle_parameters": parameters,
    }
    try:
        started = api("POST", "/api/runs", request)
    except RuntimeError as exc:
        st.error(str(exc))
    else:
        st.session_state["active_run_id"] = started["run_id"]
        st.session_state["finished_run"] = None
        st.rerun()

if st.session_state.get("active_run_id"):
    watch_run(st.session_state["active_run_id"])
elif st.session_state.get("finished_run"):
    show_result(st.session_state["finished_run"])
