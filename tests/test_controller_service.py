"""Synchronized IPC and controller process integration."""

import csv
import json
import threading
import time
from urllib.request import Request, urlopen

import pytest

from controller.controller import RunManager
from controller.http_api import create_server
from controller.ipc import ControllerLink
from simulator.simulation.simulator import run_simulator


def test_accelerated_simulator_waits_for_boundary_command_and_ipc_stop(tmp_path):
    socket_path = str(tmp_path / "sync.sock")
    result = {}

    def run():
        result["stats"] = run_simulator(
            duration_s=1, fast=True, synchronize_controller=True,
            socket_path=socket_path, controller_timeout_s=3,
        )

    worker = threading.Thread(target=run)
    worker.start()
    link = ControllerLink(socket_path)
    try:
        link.connect(timeout_s=3)
        deadline = time.monotonic() + 3
        first = None
        while first is None and time.monotonic() < deadline:
            first = link.receive_latest()
            time.sleep(0.001)
        assert first["sequence"] == 0
        time.sleep(0.03)
        assert link.receive_latest() is None
        link.send_command({"rpm": 1800, "elevator_deg": 0, "rudder_deg": 0}, 999)
        time.sleep(0.03)
        assert link.receive_latest() is None
        link.send_command({"rpm": 1800, "elevator_deg": 0, "rudder_deg": 0}, 0)
        fifth = None
        while (fifth is None or fifth["sequence"] != 5) and time.monotonic() < deadline:
            latest = link.receive_latest()
            if latest is not None:
                fifth = latest
            time.sleep(0.001)
        assert fifth["sequence"] == 5
        assert fifth["simulation_time_s"] == pytest.approx(0.05)
        link.send_stop(5)
        worker.join(timeout=3)
        assert not worker.is_alive()
        assert result["stats"]["steps"] == 5
        assert result["stats"]["published"] == 6
        assert result["stats"]["invalid_commands"] == 1
    finally:
        link.close()
        worker.join(timeout=3)


def test_synchronized_run_times_out_without_a_controller(tmp_path):
    socket_path = tmp_path / "unconnected.sock"
    with pytest.raises(TimeoutError, match="did not connect"):
        run_simulator(
            duration_s=1, fast=True, synchronize_controller=True,
            socket_path=str(socket_path), controller_timeout_s=0.05,
        )
    assert not socket_path.exists()


def test_controller_http_run_reaches_horizon_and_exports_both_logs():
    server = create_server(port=0)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    base = f"http://127.0.0.1:{server.server_port}"

    def request(method, path, payload=None):
        body = None if payload is None else json.dumps(payload).encode()
        with urlopen(Request(base + path, data=body, method=method), timeout=5) as response:
            return response.read()

    try:
        config = json.loads(request("GET", "/api/config"))
        assert config["controllers"] == ["pid", "lqr", "lqi"]
        started = json.loads(request("POST", "/api/runs", {"duration_s": 0.15}))
        run_id = started["run_id"]
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            status = json.loads(request("GET", f"/api/runs/{run_id}"))
            if status["status"] != "running":
                break
            time.sleep(0.02)
        assert status["status"] == "completed", status
        assert status["completion_reason"] == "horizon"
        assert status["latest"]["state"]["sequence"] == 15
        control_csv = request("GET", f"/api/runs/{run_id}/csv").decode().splitlines()
        state_csv = request("GET", f"/api/runs/{run_id}/state-csv").decode().splitlines()
        assert [int(row["state_sequence"]) for row in csv.DictReader(control_csv)] == [0, 5, 10, 15]
        assert len(list(csv.DictReader(state_csv))) == 15
    finally:
        server.shutdown()
        server.manager.close()
        server.server_close()
        worker.join(timeout=2)


@pytest.mark.parametrize("controller_type", ["pid", "lqr", "lqi"])
def test_controller_service_stops_simulator_on_goal(controller_type):
    manager = RunManager()
    try:
        started = manager.start({
            "controller_type": controller_type,
            "duration_s": 5,
            "mission": {
                "target_lat": 12.971602,
                "target_lon": 80.2209,
                "arrival_radius_m": 0.1,
            },
        })
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            status = manager.status(started["run_id"])
            if status["status"] != "running":
                break
            time.sleep(0.02)
        assert status["status"] == "completed", status
        assert status["completion_reason"] == "goal"
        assert status["latest"]["state"]["simulation_time_s"] < 5
    finally:
        manager.close()


def test_manual_stop_before_first_state_finishes_cleanly():
    manager = RunManager()
    try:
        started = manager.start({"duration_s": 100})
        manager.stop(started["run_id"])
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            status = manager.status(started["run_id"])
            if status["status"] != "running":
                break
            time.sleep(0.02)
        assert status["status"] == "completed", status
        assert status["completion_reason"] == "stopped"
        assert manager.result_path(started["run_id"]).exists()
    finally:
        manager.close()
