"""Fixed-step vehicle service, independent of controller and UI processes."""

from contextlib import ExitStack
import csv
import math
import threading

import numpy as np

from simulator.ipc.command_receiver import CommandReceiver, SimulatorServer
from simulator.ipc.messages import csv_row, state_message
from simulator.ipc.state_publisher import StatePublisher
from simulator.navigation.geodetic import ned_to_geodetic
from simulator.scenarios import SCENARIOS
from simulator.vehicle.parameters import VehicleParameters
from .clock import SimulationClock
from .integrator import step_vehicle


def run_simulator(
    *, frequency_hz=100.0, duration_s=None, fast=True, scenario=None,
    socket_path="/tmp/sub-simulator.sock", output=None, initial_depth=50.0,
    latitude=13.0, longitude=80.0, stop_event=None, parameters=None,
):
    """Run until duration (rounded up to a step) or a stop signal; return statistics."""
    stop_event = stop_event if stop_event is not None else threading.Event()
    clock = SimulationClock(frequency_hz, fast, sleep=stop_event.wait)
    if scenario is not None and scenario not in SCENARIOS:
        raise ValueError(f"Unknown scenario: {scenario}")
    if duration_s is None and scenario is not None:
        duration_s = 30.0
    if duration_s is not None and (not math.isfinite(duration_s) or duration_s <= 0):
        raise ValueError("duration must be finite and positive")
    if not math.isfinite(initial_depth) or initial_depth < 0:
        raise ValueError("initial-depth must be finite and nonnegative")
    ned_to_geodetic(0, 0, latitude, longitude)
    if duration_s is not None and not math.isfinite(duration_s / clock.dt):
        raise ValueError("duration and frequency produce too many steps")
    max_steps = None if duration_s is None else max(1, math.ceil(duration_s / clock.dt - 1e-12))
    parameters = parameters if parameters is not None else VehicleParameters()
    if not isinstance(parameters, VehicleParameters):
        raise TypeError("parameters must be VehicleParameters")
    state = np.zeros(13)
    state[2], state[3] = initial_depth, 1.0
    receiver, publisher = CommandReceiver(), StatePublisher()
    server = SimulatorServer(socket_path)
    steps = 0
    with ExitStack() as stack:
        stack.callback(server.close)
        server.start()
        writer = None
        if output is not None:
            file = stack.enter_context(open(output, "x", newline="", encoding="utf-8"))
            row = csv_row(state_message(state, 0, 0.0, latitude, longitude))
            writer = csv.DictWriter(file, fieldnames=row.keys())
            writer.writeheader()
        mode = "accelerated" if fast else "real-time"
        print(f"Simulator listening at {socket_path} ({frequency_hz:g} Hz simulation, {mode})", flush=True)
        clock.started = clock.now()
        while not stop_event.is_set() and (max_steps is None or steps < max_steps):
            time_s = steps * clock.dt
            server.accept_pending()
            actuators = receiver.poll(server, enabled=scenario is None)
            if scenario is not None:
                actuators = SCENARIOS[scenario](time_s)
            state = step_vehicle(state, actuators, time_s, clock.dt, parameters)
            steps += 1
            message = state_message(state, steps, steps * clock.dt, latitude, longitude)
            publisher.publish(server, message)
            if writer is not None:
                writer.writerow(csv_row(message))
            clock.wait(steps)
        elapsed = clock.elapsed_s
    return {
        "steps": steps, "simulation_time_s": steps * clock.dt,
        "wall_time_s": elapsed, "achieved_hz": steps / elapsed if elapsed else 0.0,
        "overruns": clock.overruns, "max_lateness_s": clock.max_lateness_s,
        "published": publisher.published, "dropped": publisher.dropped,
        "invalid_commands": receiver.invalid_packets,
    }
