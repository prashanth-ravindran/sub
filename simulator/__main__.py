"""Run with python -m simulator; physics owns its clock, never the UI."""

import argparse
import json
import signal
import threading

from simulator.scenarios import SCENARIOS
from simulator.simulation.simulator import run_simulator


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frequency", type=float, default=100.0, dest="frequency_hz")
    parser.add_argument("--duration", type=float, dest="duration_s", help="Simulation seconds; default unlimited, or 30 for scenarios")
    pacing = parser.add_mutually_exclusive_group()
    pacing.add_argument("--fast", action="store_true", default=True, help="Run without wall-clock pacing (default)")
    pacing.add_argument("--real-time", action="store_false", dest="fast", help="Pace simulation steps to wall-clock time")
    parser.add_argument("--scenario", choices=SCENARIOS)
    parser.add_argument("--socket", default="/tmp/sub-simulator.sock", dest="socket_path")
    parser.add_argument("--sync-controller", action="store_true", dest="synchronize_controller",
                        help="Accelerated lockstep with a controller over IPC")
    parser.add_argument("--control-period", type=float, default=0.05, dest="control_period_s")
    parser.add_argument("--controller-timeout", type=float, default=10.0, dest="controller_timeout_s")
    parser.add_argument("--output", help="Write state CSV; must be a new file")
    parser.add_argument("--initial-depth", type=float, default=50.0, help="Metres below surface")
    parser.add_argument("--latitude", type=float, default=13.0, help="NED origin latitude in degrees")
    parser.add_argument("--longitude", type=float, default=80.0, help="NED origin longitude in degrees")
    args = parser.parse_args()
    stopped = threading.Event()
    previous = {}
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, lambda *_: stopped.set())
        stats = run_simulator(**vars(args), stop_event=stopped)
        print(json.dumps(stats, sort_keys=True))
    except (ValueError, OSError, FloatingPointError, TimeoutError, ConnectionError) as exc:
        parser.exit(1, f"Simulator stopped: {exc}\n")
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    main()
