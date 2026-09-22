"""Own one closed-loop mission, its simulator process and its IPC connection."""

from dataclasses import asdict
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
from uuid import uuid4

from .config import (
    CONTROL_DT, CONTROLLER_TIMEOUT_S, INITIAL_DEPTH_M, SIMULATOR_HZ,
    validate_run,
)
from .ipc import ControllerLink, telemetry_from_message
from .logger import RunLogger
from .lqi import WaypointLQIController
from .lqr import WaypointLQRController
from .pid import WaypointPIDController


CONTROLLER_CLASSES = {
    "pid": WaypointPIDController,
    "lqr": WaypointLQRController,
    "lqi": WaypointLQIController,
}


class ConflictError(Exception):
    pass


class RunManager:
    def __init__(self):
        self._directory = tempfile.TemporaryDirectory(prefix="auv-controller-")
        self._lock = threading.Lock()
        self._run = None
        self._worker = None

    @staticmethod
    def _public(run):
        return {key: run[key] for key in (
            "run_id", "status", "completion_reason", "error", "mission",
            "controller_type", "duration_s", "latest",
        )}

    def start(self, payload):
        mission, kind, duration_s = validate_run(payload)
        with self._lock:
            if self._worker is not None and self._worker.is_alive():
                raise ConflictError("A controller run is already active")
            if self._run is not None:
                for name in ("controller_csv", "simulator_csv"):
                    self._run[name].unlink(missing_ok=True)
            run_id = uuid4().hex
            directory = Path(self._directory.name)
            run = {
                "run_id": run_id, "status": "running", "completion_reason": None,
                "error": None, "mission": asdict(mission), "controller_type": kind,
                "duration_s": duration_s, "latest": None,
                "controller_csv": directory / f"{run_id}-controller.csv",
                "simulator_csv": directory / f"{run_id}-simulator.csv",
                "socket_path": directory / f"{run_id}.sock",
                "stop_event": threading.Event(), "process": None,
            }
            self._run = run
            self._worker = threading.Thread(target=self._execute, args=(run, mission), daemon=True)
            self._worker.start()
            return self._public(run)

    def _get(self, run_id):
        if self._run is None or self._run["run_id"] != run_id:
            raise KeyError("Run not found")
        return self._run

    def status(self, run_id):
        with self._lock:
            return self._public(self._get(run_id))

    def stop(self, run_id):
        with self._lock:
            run = self._get(run_id)
            if run["status"] != "running":
                raise ConflictError("Run has already ended")
            run["stop_event"].set()
            return self._public(run)

    def result_path(self, run_id, kind="controller"):
        with self._lock:
            run = self._get(run_id)
            if run["status"] != "completed":
                raise ConflictError("Results are available after a successful run")
            return run[f"{kind}_csv"]

    @staticmethod
    def _next_state(link, process, deadline, control_steps, max_steps):
        while time.monotonic() < deadline:
            state = link.receive_latest()
            if state is not None and (
                state["sequence"] == 0 or state["sequence"] % control_steps == 0
                or state["sequence"] == max_steps
            ):
                return state
            if process.poll() is not None:
                raise RuntimeError(f"Simulator exited with status {process.returncode}")
            time.sleep(0.001)
        raise TimeoutError("No simulator state at the next control boundary")

    @staticmethod
    def _simulator_command(run, mission):
        return [
            sys.executable, "-m", "simulator", "--fast", "--sync-controller",
            "--frequency", str(SIMULATOR_HZ), "--control-period", str(CONTROL_DT),
            "--controller-timeout", str(CONTROLLER_TIMEOUT_S),
            "--duration", str(run["duration_s"]),
            "--initial-depth", str(INITIAL_DEPTH_M),
            "--latitude", str(mission.start_lat), "--longitude", str(mission.start_lon),
            "--socket", str(run["socket_path"]), "--output", str(run["simulator_csv"]),
        ]

    def _execute(self, run, mission):
        process = link = logger = None
        try:
            controller = CONTROLLER_CLASSES[run["controller_type"]](mission, CONTROL_DT)
            process = subprocess.Popen(
                self._simulator_command(run, mission),
                cwd=Path(__file__).resolve().parent.parent,
                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            )
            with self._lock:
                run["process"] = process
            link = ControllerLink(str(run["socket_path"]))
            link.connect(timeout_s=CONTROLLER_TIMEOUT_S)
            logger = RunLogger(run["controller_csv"])
            previous_sequence, previous_u = -1, 0.0
            control_steps = round(SIMULATOR_HZ * CONTROL_DT)
            max_steps = math.ceil(run["duration_s"] * SIMULATOR_HZ - 1e-12)
            while True:
                state = self._next_state(
                    link, process, time.monotonic() + CONTROLLER_TIMEOUT_S,
                    control_steps, max_steps,
                )
                sequence = state["sequence"]
                if sequence <= previous_sequence:
                    raise ValueError("Out-of-order simulator state")
                if previous_sequence >= 0 and sequence - previous_sequence > control_steps:
                    raise ValueError("Missed synchronized control boundary")
                telemetry = telemetry_from_message(state, previous_u, CONTROL_DT)
                command = controller.command(telemetry)
                logger.write(state, command)
                with self._lock:
                    run["latest"] = {
                        "state": state, "command": command,
                    }
                if command["arrived"]:
                    reason = "goal"
                elif run["stop_event"].is_set():
                    reason = "stopped"
                elif state["simulation_time_s"] + 1e-12 >= run["duration_s"]:
                    reason = "horizon"
                else:
                    reason = None
                if reason is not None:
                    link.send_stop(sequence)
                    break
                link.send_command(command, sequence)
                previous_sequence = sequence
                previous_u = telemetry["velocity"][0]
            _, stderr = process.communicate(timeout=CONTROLLER_TIMEOUT_S)
            if process.returncode != 0:
                raise RuntimeError(stderr.decode(errors="replace").strip() or "Simulator failed")
            logger.close()
            logger = None
            with self._lock:
                run["status"] = "completed"
                run["completion_reason"] = reason
        except Exception as exc:
            with self._lock:
                run["status"] = "failed"
                run["error"] = str(exc)
        finally:
            if link is not None:
                link.close()
            if logger is not None:
                logger.close()
            if process is not None and process.poll() is None:
                process.terminate()
                try:
                    process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()

    def close(self):
        with self._lock:
            if self._run is not None and self._run["status"] == "running":
                self._run["stop_event"].set()
            worker = self._worker
            process = None if self._run is None else self._run["process"]
        if worker is not None:
            worker.join(timeout=CONTROLLER_TIMEOUT_S + 5)
            if worker.is_alive() and process is not None and process.poll() is None:
                process.terminate()
                worker.join(timeout=5)
        self._directory.cleanup()
