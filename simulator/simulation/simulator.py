"""Fixed-step vehicle service, independent of controller and UI processes."""

from contextlib import ExitStack
import csv
import math
import threading

import numpy as np

from simulator.ipc.command_receiver import CommandReceiver, SimulatorServer
from simulator.ipc.messages import csv_row, state_message
from simulator.ipc.state_publisher import StatePublisher
from common.messages import command_is_expired
from simulator.navigation.geodetic import ned_to_geodetic
from simulator.scenarios import SCENARIOS
from simulator.vehicle.parameters import VehicleParameters
from .clock import SimulationClock
from .integrator import step_vehicle


def run_simulator(
    *, frequency_hz=100.0, duration_s=None, fast=True, scenario=None,
    socket_path="/tmp/sub-simulator.sock", output=None, initial_depth=50.0,
    latitude=13.0, longitude=80.0, stop_event=None, parameters=None,
    synchronize_controller=False, control_period_s=0.05, controller_timeout_s=10.0,
):
    """Run until duration (rounded up to a step) or a stop signal; return statistics."""
    stop_event = stop_event if stop_event is not None else threading.Event()
    clock = SimulationClock(frequency_hz, fast, sleep=stop_event.wait)
    if synchronize_controller:
        if not fast or scenario is not None:
            raise ValueError("Synchronized control requires accelerated external-controller mode")
        if not math.isfinite(control_period_s) or not 0 < control_period_s <= 0.1:
            raise ValueError("control_period_s must be in (0, 0.1] seconds")
        if not math.isfinite(controller_timeout_s) or controller_timeout_s <= 0:
            raise ValueError("controller_timeout_s must be finite and positive")
        control_steps = round(control_period_s / clock.dt)
        if control_steps < 1 or not math.isclose(control_steps * clock.dt, control_period_s, abs_tol=1e-10):
            raise ValueError("control_period_s must be an integer number of simulator steps")
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

        if synchronize_controller:
            deadline = clock.now() + controller_timeout_s
            while server.connection is None and not stop_event.is_set():
                server.accept_pending()
                if clock.now() >= deadline:
                    raise TimeoutError("Controller did not connect")
                stop_event.wait(0.001)

            def publish_boundary(message):
                deadline = clock.now() + controller_timeout_s
                while not stop_event.is_set():
                    if publisher.publish(server, message):
                        return
                    if server.connection is None:
                        raise ConnectionError("Controller disconnected")
                    if clock.now() >= deadline:
                        raise TimeoutError("Controller did not drain the boundary state")
                    stop_event.wait(0.001)

            def await_command(boundary_sequence):
                deadline = clock.now() + controller_timeout_s
                while not stop_event.is_set():
                    if server.connection is None:
                        raise ConnectionError("Controller disconnected")
                    command = receiver.poll(server, expected_state_sequence=boundary_sequence)
                    if receiver.stop_requested:
                        return None
                    if (receiver.command is not None
                            and receiver.command.get("state_sequence") == boundary_sequence
                            and not command_is_expired(receiver.command)):
                        return command
                    if clock.now() >= deadline:
                        raise TimeoutError(f"No command for state {boundary_sequence}")
                    stop_event.wait(0.001)
                return None

            if not stop_event.is_set():
                publish_boundary(state_message(state, 0, 0.0, latitude, longitude))
            actuators = await_command(0) if not stop_event.is_set() else None
        else:
            actuators = None
        while not stop_event.is_set() and (max_steps is None or steps < max_steps):
            if synchronize_controller and actuators is None:
                break
            time_s = steps * clock.dt
            if not synchronize_controller:
                server.accept_pending()
                actuators = receiver.poll(server, enabled=scenario is None)
                if scenario is not None:
                    actuators = SCENARIOS[scenario](time_s)
            state = step_vehicle(state, actuators, time_s, clock.dt, parameters)
            steps += 1
            message = state_message(state, steps, steps * clock.dt, latitude, longitude)
            boundary = synchronize_controller and (
                steps % control_steps == 0 or (max_steps is not None and steps == max_steps)
            )
            if boundary:
                publish_boundary(message)
            else:
                publisher.publish(server, message)
            if writer is not None:
                writer.writerow(csv_row(message))
            clock.wait(steps)
            if boundary:
                actuators = await_command(steps)
        elapsed = clock.elapsed_s
    return {
        "steps": steps, "simulation_time_s": steps * clock.dt,
        "wall_time_s": elapsed, "achieved_hz": steps / elapsed if elapsed else 0.0,
        "overruns": clock.overruns, "max_lateness_s": clock.max_lateness_s,
        "published": publisher.published, "dropped": publisher.dropped,
        "invalid_commands": receiver.invalid_packets,
    }
