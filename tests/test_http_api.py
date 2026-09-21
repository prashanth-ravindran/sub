"""Exercise the separate HTTP control plane against real simulator runs."""

import csv
from io import StringIO
import json
from pathlib import Path
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest
from streamlit.testing.v1 import AppTest

from simulator.http_api import create_server


@pytest.fixture
def api_server(tmp_path):
    server = create_server(port=0, socket_path=str(tmp_path / "sim.sock"))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}"
    finally:
        server.shutdown()
        worker.join(timeout=3)
        server.manager.close()
        server.server_close()


def call(base, method, path, payload=None, *, raw=False):
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = Request(base + path, data=data, method=method)
    try:
        response = urlopen(request, timeout=5)
    except HTTPError as exc:
        response = exc
    with response:
        body = response.read()
        return response.status, body if raw else json.loads(body)


def wait_for_run(base, run_id):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        code, status = call(base, "GET", f"/api/runs/{run_id}")
        assert code == 200
        if status["status"] != "running":
            return status
        time.sleep(0.02)
    pytest.fail("Simulator run did not finish")


def test_http_runs_apply_parameter_overrides_and_export_results(api_server):
    code, config = call(api_server, "GET", "/api/config")
    assert code == 200
    assert config["run_defaults"]["frequency_hz"] == 100
    assert config["vehicle_parameters"]["mass_kg"] == 84

    speeds = []
    for mass in (84, 168):
        code, started = call(api_server, "POST", "/api/runs", {
            "scenario": "surge_step", "duration_s": 2.2,
            "vehicle_parameters": {"mass_kg": mass},
        })
        assert code == 202
        assert started["vehicle_parameters"]["mass_kg"] == mass
        run_id = started["run_id"]
        status = wait_for_run(api_server, run_id)
        assert status["status"] == "completed", status["error"]
        assert status["completion_reason"] == "horizon"
        assert status["stats"]["steps"] == 220
        code, data = call(api_server, "GET", f"/api/runs/{run_id}/csv", raw=True)
        assert code == 200
        rows = list(csv.DictReader(StringIO(data.decode("utf-8"))))
        assert len(rows) == 220
        speeds.append(float(rows[-1]["u_mps"]))
    assert speeds[0] > speeds[1] > 0


def test_goal_signal_ends_a_run_and_prevents_concurrent_start(api_server):
    code, started = call(api_server, "POST", "/api/runs", {
        "scenario": None, "duration_s": 30, "fast": False,
    })
    assert code == 202
    run_id = started["run_id"]
    code, error = call(api_server, "POST", "/api/runs", {})
    assert code == 409
    assert "already running" in error["error"]
    code, error = call(api_server, "GET", f"/api/runs/{run_id}/csv")
    assert code == 409
    assert "after" in error["error"]
    code, _ = call(api_server, "POST", f"/api/runs/{run_id}/complete")
    assert code == 202
    status = wait_for_run(api_server, run_id)
    assert status["status"] == "completed", status["error"]
    assert status["completion_reason"] == "goal"
    assert status["stats"]["simulation_time_s"] < 30


@pytest.mark.parametrize("payload", [
    {"duration_s": 0},
    {"frequency_hz": True},
    {"scenario": "missing"},
    {"vehicle_parameters": {"mass_kg": -1}},
    {"vehicle_parameters": {"added_mass": [1, 2]}},
    {"vehicle_parameters": {"unknown": 2}},
    {"vehicle_parameters": {"mass_kg": 10 ** 400}},
    {"socket_path": "/tmp/other.sock"},
])
def test_http_rejects_invalid_run_configuration(api_server, payload):
    code, error = call(api_server, "POST", "/api/runs", payload)
    assert code == 400
    assert error["error"]


def test_streamlit_ui_starts_and_charts_a_finished_run(api_server, monkeypatch):
    monkeypatch.setenv("VEHICLE_SIMULATOR_API_URL", api_server)
    app = AppTest.from_file(str(Path(__file__).resolve().parents[1] / "ui/simulator_app.py")).run(timeout=20)
    assert [item.value for item in app.title] == ["Vehicle Simulator"]
    assert not app.exception
    next(item for item in app.number_input if item.label == "Time horizon (s)").set_value(0.1)
    next(item for item in app.number_input if item.label == "mass_kg").set_value(90.0)
    next(item for item in app.button if item.label == "Start simulation").click()
    app.run(timeout=20)
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline and not app.session_state.get("finished_run"):
        app.run(timeout=20)
    assert app.session_state["finished_run"]["status"] == "completed"
    assert app.session_state["finished_run"]["vehicle_parameters"]["mass_kg"] == 90
    assert len(app.get("vega_lite_chart")) == 3
    assert not app.exception
