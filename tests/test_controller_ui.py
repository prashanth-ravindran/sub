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
        "run_defaults": {"mission": asdict(Mission()), "controller_type": "pid", "duration_s": 450},
        "controllers": ["pid", "lqr", "lqi"],
        "control_period_s": 0.05, "simulator_frequency_hz": 100,
    }

    def fake_urlopen(request, timeout):
        assert request.full_url.endswith("/api/config")
        return io.BytesIO(json.dumps(config).encode())

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    app = AppTest.from_file(str(Path(__file__).resolve().parents[1] / "ui/controller_app.py"))
    app.run(timeout=10)
    assert not app.exception
    assert app.title[0].value == "AUV Controller"
    assert list(app.selectbox[0].options) == ["pid", "lqr", "lqi"]


def test_controller_ui_charts_completed_state_and_control_run(monkeypatch):
    mission = Mission()
    config = {
        "run_defaults": {"mission": asdict(mission), "controller_type": "pid", "duration_s": 450},
        "controllers": ["pid", "lqr", "lqi"],
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
        "status": "completed", "completion_reason": "horizon", "run_id": "a" * 32,
        "mission": asdict(mission), "controller_type": "pid",
        "latest": {"state": state_message(state, 0, 0.0), "command": {
            "rpm": 1000.0, "elevator_deg": 0.0, "rudder_deg": 0.0, "distance": 500.0,
        }},
    }

    def fake_urlopen(request, timeout):
        if request.full_url.endswith("/api/config"):
            return io.BytesIO(json.dumps(config).encode())
        if request.full_url.endswith("/state-csv"):
            return io.BytesIO(b"sequence\n0\n")
        if request.full_url.endswith("/csv"):
            return io.BytesIO(text.getvalue().encode())
        raise AssertionError(request.full_url)

    monkeypatch.setattr(urllib.request, "urlopen", fake_urlopen)
    app = AppTest.from_file(str(Path(__file__).resolve().parents[1] / "ui/controller_app.py"))
    app.session_state["finished_run"] = status
    app.run(timeout=10)
    assert not app.exception
    assert any("Run result" in item.value for item in app.subheader)
