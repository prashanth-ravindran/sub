"""Separate-process Streamlit client for waypoint controller runs."""

from io import BytesIO
import json
import os
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import streamlit as st

from controller.mission import Mission
from controller.visualizer import plot_results


API_URL = os.environ.get("AUV_CONTROLLER_API_URL", "http://127.0.0.1:8766").rstrip("/")


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
        raise RuntimeError(f"Cannot reach the controller API at {API_URL}: {exc.reason}") from exc
    return data if csv else json.loads(data)


@st.cache_data(max_entries=2, show_spinner="Loading completed run…")
def load_results(run_id):
    control_csv = api("GET", f"/api/runs/{run_id}/csv", csv=True)
    state_csv = api("GET", f"/api/runs/{run_id}/state-csv", csv=True)
    return control_csv, state_csv, pd.read_csv(BytesIO(control_csv))


def latest_metrics(latest):
    if latest is None:
        st.info("Waiting for the first simulator state.")
        return
    state, command = latest["state"], latest["command"]
    position, velocity = state["position"], state["velocity_body"]
    with st.container(horizontal=True):
        st.metric("Simulation time", f"{state['simulation_time_s']:.2f} s")
        st.metric("Distance to target", f"{command['distance']:.1f} m")
        st.metric("Depth", f"{state['depth_m']:.2f} m")
        st.metric("Forward speed", f"{velocity['u_mps']:.2f} m/s")
    with st.container(horizontal=True):
        st.metric("Propeller", f"{command['rpm']:.0f} RPM")
        st.metric("Elevator", f"{command['elevator_deg']:.2f}°")
        st.metric("Rudder", f"{command['rudder_deg']:.2f}°")
    st.caption(f"NED: north {position['north_m']:.2f} m, east {position['east_m']:.2f} m")
    with st.expander("Latest simulator state and controller output", expanded=True):
        st.caption(f"Sample {state['sequence']:,} · Refreshes every second while running.")
        vehicle, control = st.columns(2)
        with vehicle:
            st.markdown("**Vehicle state**")
            st.table({
                "Simulation time": f"{state['simulation_time_s']:.2f} s",
                "North": f"{position['north_m']:.2f} m",
                "East": f"{position['east_m']:.2f} m",
                "Down (NED)": f"{position['down_m']:.2f} m",
                "Depth": f"{state['depth_m']:.2f} m",
                "Latitude": f"{state['geodetic']['latitude_deg']:.6f}°",
                "Longitude": f"{state['geodetic']['longitude_deg']:.6f}°",
                "Roll": f"{np.degrees(state['orientation']['roll_rad']):.2f}°",
                "Pitch": f"{np.degrees(state['orientation']['pitch_rad']):.2f}°",
                "Yaw": f"{np.degrees(state['orientation']['yaw_rad']):.2f}°",
                "Surge velocity (u)": f"{velocity['u_mps']:.3f} m/s",
                "Sway velocity (v)": f"{velocity['v_mps']:.3f} m/s",
                "Heave velocity (w)": f"{velocity['w_mps']:.3f} m/s",
                "Roll rate (p)": f"{state['angular_velocity_body']['p_radps']:.4f} rad/s",
                "Pitch rate (q)": f"{state['angular_velocity_body']['q_radps']:.4f} rad/s",
                "Yaw rate (r)": f"{state['angular_velocity_body']['r_radps']:.4f} rad/s",
            }, border="horizontal")
        with control:
            st.markdown("**Controller output**")
            st.table({
                "Propeller": f"{command['rpm']:.0f} RPM",
                "Elevator": f"{command['elevator_deg']:.2f}°",
                "Rudder": f"{command['rudder_deg']:.2f}°",
                "Depth setpoint": f"{command['depth_setpoint']:.2f} m",
                "Speed setpoint": f"{command['speed_setpoint']:.2f} m/s",
                "Pitch setpoint": f"{np.degrees(command['pitch_setpoint']):.2f}°",
                "Heading setpoint": f"{np.degrees(command['heading_setpoint']) % 360:.2f}°",
                "Distance to target": f"{command['distance']:.2f} m",
                "Arrived": "Yes" if command['arrived'] else "No",
            }, border="horizontal")


def process_status(status):
    pid = status.get("simulator_pid")
    with st.container(horizontal=True):
        with st.container():
            st.metric("Controller service PID", str(status.get("controller_pid", "Unavailable")))
            st.caption("Service stays running between missions.")
        with st.container():
            st.metric("Simulator PID", str(pid) if pid is not None else "—")
            if status.get("simulator_running"):
                st.caption("Running · transient process")
            elif pid is not None:
                st.caption("Exited · PID retained as a record of this run")
            elif status.get("status") == "running":
                st.caption("Simulator process: starting…")
            else:
                st.caption("No simulator process started.")
    st.caption(
        "The simulator is transient: each mission starts a separate process that exits when "
        "the run completes or is stopped. The next mission starts a new simulator process."
    )
    if "controller_pid" not in status:
        st.caption("Restart the controller service after this run to enable both PID displays.")


@st.fragment(run_every="1s")
def watch_run(run_id):
    try:
        status = api("GET", f"/api/runs/{run_id}")
    except RuntimeError as exc:
        st.error(str(exc))
        return
    if status["status"] == "running":
        process_status(status)
        st.info("Controller and simulator are running. Charts appear after completion.")
        latest_metrics(status["latest"])
        if st.button("Stop run"):
            try:
                api("POST", f"/api/runs/{run_id}/stop")
            except RuntimeError as exc:
                st.error(str(exc))
        return
    st.session_state["active_run_id"] = None
    st.session_state["finished_run"] = status
    st.rerun(scope="app")


def show_result(status):
    process_status(status)
    if status["status"] == "failed":
        st.error(f"Run failed: {status['error']}")
        if status["latest"]:
            latest_metrics(status["latest"])
        return
    st.subheader("Run result")
    st.caption(f"Completed by {status['completion_reason']}")
    latest_metrics(status["latest"])
    try:
        control_csv, state_csv, frame = load_results(status["run_id"])
    except RuntimeError as exc:
        st.error(str(exc))
        return
    with st.container(horizontal=True):
        st.download_button("Download controller CSV", control_csv,
                           file_name=f"controller-{status['run_id']}.csv", mime="text/csv")
        st.download_button(f"Download {status['simulator_frequency_hz']} Hz state CSV", state_csv,
                           file_name=f"vehicle-{status['run_id']}.csv", mime="text/csv")
    if frame.empty:
        st.info("No control samples were recorded.")
        return
    chart_frame = frame if len(frame) <= 5000 else frame.iloc[
        np.linspace(0, len(frame) - 1, 5000, dtype=int)
    ]
    if len(chart_frame) < len(frame):
        st.caption("Charts show 5,000 samples; the downloads contain the full logs.")
    figure = plot_results(chart_frame.to_dict("records"), Mission(**status["mission"]),
                          status["controller_type"])
    st.pyplot(figure, width="stretch")
    plt.close(figure)
    with st.expander("Recorded state and control samples"):
        st.dataframe(frame, hide_index=True)


st.set_page_config(page_title="AUV Controller", layout="wide")
st.title("AUV Controller")
st.caption("Run waypoint PID, LQR, or LQI against the independent vehicle simulator.")

try:
    config = api("GET", "/api/config")
except RuntimeError as exc:
    st.error(str(exc))
    st.stop()

defaults = config["run_defaults"]
mission = defaults["mission"]
with st.form("mission_settings"):
    st.subheader("Mission")
    a, b = st.columns(2)
    with a:
        start_lat = st.number_input("Start latitude (deg)", value=float(mission["start_lat"]), format="%.6f")
        start_lon = st.number_input("Start longitude (deg)", value=float(mission["start_lon"]), format="%.6f")
        depth_m = st.number_input("Target depth (m)", min_value=0.01, value=float(mission["depth_m"]))
    with b:
        target_lat = st.number_input("Target latitude (deg)", value=float(mission["target_lat"]), format="%.6f")
        target_lon = st.number_input("Target longitude (deg)", value=float(mission["target_lon"]), format="%.6f")
        speed_mps = st.number_input("Target speed (m/s)", min_value=0.01, value=float(mission["speed_mps"]))
    c, d, e, f = st.columns(4)
    with c:
        arrival_radius_m = st.number_input("Arrival radius (m)", min_value=0.01,
                                           value=float(mission["arrival_radius_m"]))
    with d:
        controller_type = st.selectbox("Controller", config["controllers"])
    with e:
        duration_s = st.number_input("Time horizon (s)", min_value=0.01,
                                     value=float(defaults["duration_s"]))
    with f:
        simulator_frequency_hz = st.selectbox(
            "Simulation rate (Hz)", config["simulator_frequencies_hz"],
            index=config["simulator_frequencies_hz"].index(defaults["simulator_frequency_hz"]),
        )
    submitted = st.form_submit_button("Start controller run",
                                      disabled=bool(st.session_state.get("active_run_id")))

if submitted:
    request = {
        "mission": {
            "start_lat": start_lat, "start_lon": start_lon,
            "target_lat": target_lat, "target_lon": target_lon,
            "depth_m": depth_m, "speed_mps": speed_mps,
            "arrival_radius_m": arrival_radius_m,
        },
        "controller_type": controller_type, "duration_s": duration_s,
        "simulator_frequency_hz": simulator_frequency_hz,
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
else:
    process_status(config)
