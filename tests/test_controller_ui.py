"""The controller Streamlit page renders without owning a simulator socket."""

from dataclasses import asdict
import csv
import io
import json
from pathlib import Path
import urllib.request

from streamlit.testing.v1 import AppTest

from controller.mission import Mission
from controller.logger import FIELDS
from simulator.ipc.messages import state_message

import numpy as np


def test_controller_ui_renders_mission_form(monkeypatch):
    config = {
        "run_defaults": {"mission": asdict(Mission()), "controller_type": "pid", "duration_s": 450,
                         "simulator_frequency_hz": 100},
        "controllers": ["pid", "lqr", "lqi"],
        "control_period_s": 0.05, "simulator_frequency_hz": 100,
        "simulator_frequencies_hz": [60, 80, 100],
        "controller_pid": 12345,
    }

    submitted = []

    def fake_urlopen(request, timeout):
        if request.full_url.endswith("/api/config"):
            return io.BytesIO(json.dumps(config).encode())
        if request.full_url.endswith("/api/runs"):
            submitted.append(json.loads(request.data))
            return io.BytesIO(json.dumps({"run_id": "a" * 32}).encode())
        if request.full_url.endswith("/api/runs/" + "a" * 32):
            return io.BytesIO(json.dumps({
                "status": "running", "latest": None, "controller_pid": 12345,
                "simulator_pid": None, "simulator_running": False,
            }).encode())
        raise AssertionError(request.full_url)

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    app = AppTest.from_file(str(Path(__file__).resolve().parents[1] / "ui/controller_app.py"))
    app.run(timeout=10)
    assert not app.exception
    assert app.title[0].value == "AUV Controller"
    assert list(app.selectbox[0].options) == ["pid", "lqr", "lqi"]
    assert list(app.selectbox[1].options) == ["60", "80", "100"]
    assert app.selectbox[1].value == 100
    assert app.metric[0].label == "Controller service PID"
    assert app.metric[0].value == "12345"
    assert app.metric[1].value == "—"
    app.selectbox[1].set_value(60)
    next(item for item in app.button if item.label == "Start controller run").click()
    app.run(timeout=10)
    assert not app.exception
    assert submitted[0]["simulator_frequency_hz"] == 60


def test_controller_ui_refreshes_telemetry_then_charts_completed_run(monkeypatch):
    mission = Mission()
    config = {
        "run_defaults": {"mission": asdict(mission), "controller_type": "pid", "duration_s": 450,
                         "simulator_frequency_hz": 100},
        "controllers": ["pid", "lqr", "lqi"],
        "simulator_frequencies_hz": [60, 80, 100],
    }
    row = {field: 0.0 for field in FIELDS}
    row.update({"depth_m": 0.1, "depth_setpoint_m": 10.0, "distance_to_target_m": 500.0})
    text = io.StringIO()
    writer = csv.DictWriter(text, fieldnames=FIELDS)
    writer.writeheader()
    writer.writerow(row)
    state = np.zeros(13)
    state[2], state[3] = 0.1, 1.0
    status = {
        "status": "running", "completion_reason": None, "run_id": "a" * 32,
        "simulator_pid": None,
        "controller_pid": 12345, "simulator_running": False,
        "simulator_frequency_hz": 100,
        "mission": asdict(mission), "controller_type": "pid",
        "latest": {"state": state_message(state, 0, 0.0), "command": {
            "rpm": 1000.0, "elevator_deg": 0.0, "rudder_deg": 0.0, "distance": 500.0,
            "depth_setpoint": 10.0, "speed_setpoint": 1.5,
            "pitch_setpoint": -np.pi / 12, "heading_setpoint": np.pi / 2,
            "arrived": False,
        }},
    }

    def fake_urlopen(request, timeout):
        if request.full_url.endswith("/api/config"):
            return io.BytesIO(json.dumps(config).encode())
        if request.full_url.endswith(f"/api/runs/{status['run_id']}"):
            return io.BytesIO(json.dumps(status).encode())
        if request.full_url.endswith("/state-csv"):
            return io.BytesIO(b"sequence\n0\n")
        if request.full_url.endswith("/csv"):
            return io.BytesIO(text.getvalue().encode())
        raise AssertionError(request.full_url)

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    app = AppTest.from_file(str(Path(__file__).resolve().parents[1] / "ui/controller_app.py"))
    app.session_state["active_run_id"] = status["run_id"]
    app.run(timeout=10)
    assert not app.exception
    assert not app.json
    assert any("Simulator process: starting" in item.value for item in app.caption)
    assert app.table[0].value.loc["Depth"].iloc[0] == "0.10 m"
    assert app.table[1].value.loc["Pitch setpoint"].iloc[0] == "-15.00°"
    assert app.table[1].value.loc["Heading setpoint"].iloc[0] == "90.00°"
    assert app.table[1].value.loc["Arrived"].iloc[0] == "No"

    state[2] = 4.25
    state[3:7] = [np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)]
    state[8], state[12] = -0.125, 0.0123
    status["latest"]["state"] = state_message(state, 100, 1.0)
    status["latest"]["command"]["rpm"] = 1200.0
    status["simulator_pid"] = 43210
    status["simulator_running"] = True
    app.run(timeout=10)
    assert not app.exception
    assert app.table[0].value.loc["Depth"].iloc[0] == "4.25 m"
    assert app.table[0].value.loc["Yaw"].iloc[0] == "90.00°"
    assert app.table[0].value.loc["Sway velocity (v)"].iloc[0] == "-0.125 m/s"
    assert app.table[0].value.loc["Yaw rate (r)"].iloc[0] == "0.0123 rad/s"
    assert app.table[1].value.loc["Propeller"].iloc[0] == "1200 RPM"
    assert app.metric[0].value == "12345"
    assert app.metric[1].label == "Simulator PID"
    assert app.metric[1].value == "43210"
    assert any("Running · transient process" == item.value for item in app.caption)

    status.update(status="completed", completion_reason="horizon", simulator_running=False)
    app.run(timeout=10)
    assert not app.exception
    assert any("Run result" in item.value for item in app.subheader)
    assert app.table[0].value.loc["Depth"].iloc[0] == "4.25 m"
    assert app.metric[0].value == "12345"
    assert app.metric[1].value == "43210"
    assert any("Exited · PID retained" in item.value for item in app.caption)
    assert any("each mission starts a separate process" in item.value for item in app.caption)
