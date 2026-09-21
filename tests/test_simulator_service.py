import csv
import json
from pathlib import Path
import select
import signal
import socket
import subprocess
import sys
import time

import pytest

from common.ipc import MAX_MESSAGE_SIZE
from common.messages import make_actuator_command, validate_vehicle_state_or_raise
from simulator.simulation.simulator import run_simulator


ROOT = Path(__file__).resolve().parents[1]


def test_service_runs_without_controller_and_exports_csv(tmp_path):
    path, output = tmp_path / "s", tmp_path / "state.csv"
    stats = run_simulator(socket_path=str(path), output=output, duration_s=0.1)
    assert stats["steps"] == 10
    assert stats["simulation_time_s"] == pytest.approx(0.1)
    assert stats["published"] == 0
    assert not path.exists()
    with output.open() as file:
        rows = list(csv.DictReader(file))
    assert len(rows) == 10
    assert float(rows[-1]["simulation_time_s"]) == pytest.approx(0.1)
    assert float(rows[-1]["latitude_deg"]) == 13
    assert all(float(row["depth_m"]) == 50 and float(row["u_mps"]) == 0 for row in rows)
    with pytest.raises(FileExistsError):
        run_simulator(socket_path=str(path), output=output, duration_s=0.1)
    assert not path.exists()


def test_accelerated_scenario_uses_simulation_time_without_a_controller(tmp_path):
    path, output = tmp_path / "s", tmp_path / "surge.csv"
    stats = run_simulator(
        socket_path=str(path), output=output, scenario="surge_step",
        duration_s=2.1,
    )
    assert stats["steps"] == 210
    with output.open() as file:
        rows = list(csv.DictReader(file))
    assert all(float(row["u_mps"]) == 0 for row in rows[:200])
    assert float(rows[-1]["u_mps"]) > 0.09
    assert float(rows[-1]["simulation_time_s"]) == pytest.approx(2.1)
    assert not path.exists()


@pytest.mark.parametrize("signum, pacing", [(signal.SIGINT, []), (signal.SIGTERM, ["--real-time"])])
def test_service_accepts_commands_expires_them_and_cleans_up_on_signal(tmp_path, signum, pacing):
    path = tmp_path / "s"
    process = subprocess.Popen(
        [sys.executable, "-m", "simulator", "--socket", str(path), *pacing],
        cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    )
    try:
        assert select.select([process.stdout], [], [], 10)[0], "Service did not start"
        startup = process.stdout.readline()
        assert "listening" in startup
        assert ("real-time" if pacing else "accelerated") in startup
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as client:
            client.settimeout(2)
            client.connect(str(path))
            command = make_actuator_command(
                sequence=1, propeller_rpm=1800, elevator_deg=0, rudder_deg=0,
                validity_ns=200_000_000,
            )
            client.send(json.dumps(command).encode())
            samples = []
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                message = json.loads(client.recv(MAX_MESSAGE_SIZE))
                validate_vehicle_state_or_raise(message)
                samples.append(message)
                if message["timestamp_ns"] > command["valid_until_ns"] + 200_000_000:
                    break
            speeds = [sample["velocity_body"]["u_mps"] for sample in samples]
            assert max(speeds) > 0.01
            assert speeds[-1] < max(speeds)
            assert samples[-1]["simulation_time_s"] > samples[0]["simulation_time_s"]
        process.send_signal(signum)
        stdout, stderr = process.communicate(timeout=3)
        assert process.returncode == 0, stderr
        assert json.loads(stdout.strip())["steps"] > 0
        assert not path.exists()
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=3)


@pytest.mark.parametrize("kwargs", [
    {"frequency_hz": 0}, {"duration_s": -1}, {"duration_s": float("nan")},
    {"initial_depth": -1}, {"latitude": 90}, {"scenario": "unknown"},
    {"frequency_hz": 1e308, "duration_s": 1e308},
])
def test_invalid_configuration_fails_before_creating_socket(tmp_path, kwargs):
    path = tmp_path / "s"
    with pytest.raises(ValueError):
        run_simulator(socket_path=str(path), **kwargs)
    assert not path.exists()
