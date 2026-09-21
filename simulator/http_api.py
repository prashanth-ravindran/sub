"""Local HTTP control plane for the independent vehicle simulator service."""

import argparse
from dataclasses import asdict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import tempfile
import threading
import traceback
from urllib.parse import urlsplit
from uuid import uuid4

from simulator.navigation.geodetic import ned_to_geodetic
from simulator.scenarios import SCENARIOS
from simulator.simulation.clock import SimulationClock
from simulator.simulation.simulator import run_simulator
from simulator.vehicle.parameters import VehicleParameters


RUN_DEFAULTS = {
    "scenario": "surge_step",
    "duration_s": 30.0,
    "frequency_hz": 100.0,
    "fast": True,
    "initial_depth": 50.0,
    "latitude": 13.0,
    "longitude": 80.0,
}
RUN_ROUTE = re.compile(r"/api/runs/([0-9a-f]{32})(?:/(csv|complete))?")


class ConflictError(Exception):
    """The requested operation conflicts with the current run."""


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    return number


def validate_run(payload):
    """Validate the HTTP boundary and return simulator settings and parameters."""
    if not isinstance(payload, dict):
        raise ValueError("Request body must be a JSON object")
    unknown = payload.keys() - (RUN_DEFAULTS.keys() | {"vehicle_parameters"})
    if unknown:
        raise ValueError(f"Unknown run fields: {', '.join(sorted(unknown))}")

    settings = {**RUN_DEFAULTS, **{key: payload[key] for key in RUN_DEFAULTS.keys() & payload.keys()}}
    if settings["scenario"] is not None and settings["scenario"] not in SCENARIOS:
        raise ValueError("Unknown scenario")
    if type(settings["fast"]) is not bool:
        raise ValueError("fast must be a boolean")
    for name in ("duration_s", "frequency_hz", "initial_depth", "latitude", "longitude"):
        settings[name] = _number(settings[name], name)
    if settings["duration_s"] <= 0:
        raise ValueError("duration_s must be positive")
    if settings["initial_depth"] < 0:
        raise ValueError("initial_depth must be nonnegative")
    clock = SimulationClock(settings["frequency_hz"])
    if not math.isfinite(settings["duration_s"] / clock.dt):
        raise ValueError("duration and frequency produce too many steps")
    ned_to_geodetic(0, 0, settings["latitude"], settings["longitude"])

    overrides = payload.get("vehicle_parameters", {})
    if not isinstance(overrides, dict):
        raise ValueError("vehicle_parameters must be an object")
    values = asdict(VehicleParameters())
    unknown = overrides.keys() - values.keys()
    if unknown:
        raise ValueError(f"Unknown vehicle parameters: {', '.join(sorted(unknown))}")
    for name, value in overrides.items():
        if isinstance(values[name], tuple):
            if not isinstance(value, list) or len(value) != len(values[name]):
                raise ValueError(f"{name} must be a list of {len(values[name])} numbers")
            values[name] = tuple(_number(item, name) for item in value)
        else:
            values[name] = _number(value, name)
    return settings, VehicleParameters(**values)


class RunManager:
    """Own one active run and its most recent CSV without touching controller IPC."""

    def __init__(self, socket_path="/tmp/sub-simulator.sock"):
        self.socket_path = socket_path
        self._directory = tempfile.TemporaryDirectory(prefix="vehicle-simulator-")
        self._lock = threading.Lock()
        self._run = None
        self._worker = None

    def start(self, payload):
        settings, parameters = validate_run(payload)
        with self._lock:
            if self._run is not None and self._run["status"] == "running":
                raise ConflictError("A simulation is already running")
            if self._run is not None:
                self._run["csv_path"].unlink(missing_ok=True)
            run_id = uuid4().hex
            run = {
                "run_id": run_id,
                "status": "running",
                "completion_reason": None,
                "stats": None,
                "error": None,
                "settings": settings,
                "vehicle_parameters": asdict(parameters),
                "csv_path": Path(self._directory.name) / f"{run_id}.csv",
                "stop_event": threading.Event(),
                "stop_reason": None,
            }
            self._run = run
            self._worker = threading.Thread(target=self._execute, args=(run, parameters))
            self._worker.start()
            return self._public(run)

    def _execute(self, run, parameters):
        try:
            stats = run_simulator(
                **run["settings"], parameters=parameters,
                socket_path=self.socket_path, output=run["csv_path"],
                stop_event=run["stop_event"],
            )
        except Exception as exc:
            traceback.print_exc()
            with self._lock:
                run["status"] = "failed"
                run["error"] = str(exc)
            return
        with self._lock:
            run["status"] = "completed"
            run["stats"] = stats
            run["completion_reason"] = (
                "horizon" if stats["simulation_time_s"] + 1e-12 >= run["settings"]["duration_s"]
                else run["stop_reason"] or "stopped"
            )

    @staticmethod
    def _public(run):
        return {key: run[key] for key in (
            "run_id", "status", "completion_reason", "stats", "error",
            "settings", "vehicle_parameters",
        )}

    def _get(self, run_id):
        if self._run is None or self._run["run_id"] != run_id:
            raise KeyError("Run not found")
        return self._run

    def status(self, run_id):
        with self._lock:
            return self._public(self._get(run_id))

    def complete(self, run_id):
        with self._lock:
            run = self._get(run_id)
            if run["status"] != "running":
                raise ConflictError("Run has already ended")
            run["stop_reason"] = "goal"
            run["stop_event"].set()
            return self._public(run)

    def result_path(self, run_id):
        with self._lock:
            run = self._get(run_id)
            if run["status"] != "completed":
                raise ConflictError("Results are available after a successful run")
            return run["csv_path"]

    def close(self):
        with self._lock:
            if self._run is not None and self._run["status"] == "running":
                self._run["stop_reason"] = "stopped"
                self._run["stop_event"].set()
            worker = self._worker
        if worker is not None:
            worker.join()
        self._directory.cleanup()


class Handler(BaseHTTPRequestHandler):
    def _json(self, status, value):
        body = json.dumps(value, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _read_json(self):
        try:
            length = int(self.headers.get("Content-Length", ""))
        except ValueError as exc:
            raise ValueError("Content-Length is required") from exc
        if not 0 < length <= 65536:
            raise ValueError("JSON body must be 1–65536 bytes")
        return json.loads(self.rfile.read(length))

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/config":
            self._json(200, {
                "scenarios": list(SCENARIOS),
                "run_defaults": RUN_DEFAULTS,
                "vehicle_parameters": asdict(VehicleParameters()),
            })
            return
        match = RUN_ROUTE.fullmatch(path)
        if match is None:
            self._json(404, {"error": "Endpoint not found"})
            return
        run_id, action = match.groups()
        try:
            if action is None:
                self._json(200, self.server.manager.status(run_id))
            elif action == "csv":
                path = self.server.manager.result_path(run_id)
                with path.open("rb") as result:
                    self.send_response(200)
                    self.send_header("Content-Type", "text/csv; charset=utf-8")
                    self.send_header("Content-Disposition", f'attachment; filename="vehicle-{run_id}.csv"')
                    self.send_header("Content-Length", str(os.fstat(result.fileno()).st_size))
                    self.send_header("Cache-Control", "no-store")
                    self.end_headers()
                    shutil.copyfileobj(result, self.wfile)
            else:
                self._json(404, {"error": "Endpoint not found"})
        except KeyError:
            self._json(404, {"error": "Run not found"})
        except ConflictError as exc:
            self._json(409, {"error": str(exc)})
        except FileNotFoundError:
            self._json(404, {"error": "Result file not found"})

    def do_POST(self):
        path = urlsplit(self.path).path
        try:
            if path == "/api/runs":
                self._json(202, self.server.manager.start(self._read_json()))
                return
            match = RUN_ROUTE.fullmatch(path)
            if match is not None and match.group(2) == "complete":
                self._json(202, self.server.manager.complete(match.group(1)))
                return
            self._json(404, {"error": "Endpoint not found"})
        except (ValueError, TypeError) as exc:
            self._json(400, {"error": str(exc)})
        except KeyError:
            self._json(404, {"error": "Run not found"})
        except ConflictError as exc:
            self._json(409, {"error": str(exc)})


def create_server(port=8765, socket_path="/tmp/sub-simulator.sock"):
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.manager = RunManager(socket_path)
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--socket", default="/tmp/sub-simulator.sock")
    args = parser.parse_args()
    server = create_server(args.port, args.socket)
    previous = signal.signal(
        signal.SIGTERM,
        lambda *_: threading.Thread(target=server.shutdown, daemon=True).start(),
    )
    try:
        print(f"Vehicle Simulator API listening at http://127.0.0.1:{server.server_port}", flush=True)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.manager.close()
        server.server_close()
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    main()
