"""Local HTTP control plane for the independent controller service."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
import re
import shutil
import signal
import threading
from urllib.parse import urlsplit

from .config import API_PORT, CONTROLLERS, CONTROL_DT, SIMULATOR_FREQUENCIES_HZ, SIMULATOR_HZ, run_defaults
from .controller import ConflictError, RunManager


RUN_ROUTE = re.compile(r"/api/runs/([0-9a-f]{32})(?:/(csv|state-csv|stop))?")


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

    def _csv(self, path, filename):
        with path.open("rb") as result:
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
            self.send_header("Content-Length", str(os.fstat(result.fileno()).st_size))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            shutil.copyfileobj(result, self.wfile)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/config":
            self._json(200, {
                "run_defaults": run_defaults(), "controllers": CONTROLLERS,
                "control_period_s": CONTROL_DT, "simulator_frequency_hz": SIMULATOR_HZ,
                "simulator_frequencies_hz": SIMULATOR_FREQUENCIES_HZ,
                "controller_pid": os.getpid(),
            })
            return
        match = RUN_ROUTE.fullmatch(path)
        if match is None or match.group(2) == "stop":
            self._json(404, {"error": "Endpoint not found"})
            return
        run_id, action = match.groups()
        try:
            if action is None:
                self._json(200, self.server.manager.status(run_id))
            elif action == "csv":
                self._csv(self.server.manager.result_path(run_id), f"controller-{run_id}.csv")
            else:
                self._csv(self.server.manager.result_path(run_id, "simulator"), f"vehicle-{run_id}.csv")
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
            if match is not None and match.group(2) == "stop":
                self._json(202, self.server.manager.stop(match.group(1)))
                return
            self._json(404, {"error": "Endpoint not found"})
        except (ValueError, TypeError) as exc:
            self._json(400, {"error": str(exc)})
        except KeyError:
            self._json(404, {"error": "Run not found"})
        except ConflictError as exc:
            self._json(409, {"error": str(exc)})


def create_server(port=API_PORT):
    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.manager = RunManager()
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=API_PORT)
    args = parser.parse_args()
    server = create_server(args.port)
    previous = signal.signal(
        signal.SIGTERM,
        lambda *_: threading.Thread(target=server.shutdown, daemon=True).start(),
    )
    try:
        print(f"Controller API listening at http://127.0.0.1:{server.server_port}", flush=True)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.manager.close()
        server.server_close()
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    main()
