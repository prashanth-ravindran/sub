# Note: Used AI to generate reasonable unit tests to cover commonsense issues and routine cases 



import json
import socket
import threading
import time

import pytest

from common.ipc import (
    IPCDisconnected,
    IPCError,
    MAX_MESSAGE_SIZE,
    UnixSeqPacketClient,
    UnixSeqPacketServer,
)


pytestmark = pytest.mark.skipif(
    not hasattr(socket, "SOCK_SEQPACKET"),
    reason="Unix SOCK_SEQPACKET is not supported on this platform",
)


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def create_connected_pair(tmp_path):
    """
    Creates a connected server/client IPC pair.

    Server.start() blocks waiting for a connection, so it runs briefly
    in a background thread during test setup.
    """

    socket_path = str(tmp_path / "auv_test.sock")

    server = UnixSeqPacketServer(socket_path)
    client = UnixSeqPacketClient(
        socket_path,
        retry_interval=0.01,
    )

    server_thread = threading.Thread(
        target=server.start,
        daemon=True,
    )

    server_thread.start()

    client.connect()

    server_thread.join(timeout=2.0)

    assert not server_thread.is_alive()
    assert server.connection is not None

    return server, client


def wait_for_message(receiver, timeout_s=1.0):
    """
    Poll receive_latest() until a message arrives or timeout expires.

    Keeps timing assumptions out of the tests.
    """

    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        message = receiver.receive_latest()

        if message is not None:
            return message

        time.sleep(0.001)

    raise AssertionError("Timed out waiting for IPC message")


# ---------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------

def test_server_and_client_connect(tmp_path):

    server, client = create_connected_pair(tmp_path)

    try:
        assert server.connection is not None
        assert client.socket is not None

    finally:
        client.close()
        server.close()


# ---------------------------------------------------------------------
# Simulator -> Controller
# ---------------------------------------------------------------------

def test_server_can_send_vehicle_state_to_client(tmp_path):

    server, client = create_connected_pair(tmp_path)

    message = {
        "type": "vehicle_state",
        "version": 1,
        "sequence": 42,
        "timestamp_ns": 123456789,
        "depth_m": 10.2,
        "velocity_body": {
            "u_mps": 1.5,
            "v_mps": 0.0,
            "w_mps": 0.0,
        },
    }

    try:
        server.send(message)

        received = wait_for_message(client)

        assert received == message
        assert received["type"] == "vehicle_state"
        assert received["sequence"] == 42
        assert received["depth_m"] == pytest.approx(10.2)

    finally:
        client.close()
        server.close()


# ---------------------------------------------------------------------
# Controller -> Simulator
# ---------------------------------------------------------------------

def test_client_can_send_actuator_command_to_server(tmp_path):

    server, client = create_connected_pair(tmp_path)

    command = {
        "type": "actuator_command",
        "version": 1,
        "sequence": 7,
        "timestamp_ns": 123456789,
        "valid_until_ns": 123656789,
        "actuators": {
            "propeller_rpm": 1750.0,
            "elevator_deg": -4.5,
            "rudder_deg": 2.0,
        },
    }

    try:
        client.send(command)

        received = wait_for_message(server)

        assert received == command

        actuators = received["actuators"]

        assert actuators["propeller_rpm"] == pytest.approx(1750.0)
        assert actuators["elevator_deg"] == pytest.approx(-4.5)
        assert actuators["rudder_deg"] == pytest.approx(2.0)

    finally:
        client.close()
        server.close()


# ---------------------------------------------------------------------
# Full duplex
# ---------------------------------------------------------------------

def test_bidirectional_communication(tmp_path):

    server, client = create_connected_pair(tmp_path)

    state = {
        "type": "vehicle_state",
        "version": 1,
        "sequence": 100,
        "timestamp_ns": 1000,
        "depth_m": 9.8,
    }

    command = {
        "type": "actuator_command",
        "version": 1,
        "sequence": 200,
        "timestamp_ns": 2000,
        "actuators": {
            "propeller_rpm": 1500.0,
            "elevator_deg": 3.0,
            "rudder_deg": -2.0,
        },
    }

    try:
        server.send(state)
        client.send(command)

        received_state = wait_for_message(client)
        received_command = wait_for_message(server)

        assert received_state["sequence"] == 100
        assert received_state["type"] == "vehicle_state"

        assert received_command["sequence"] == 200
        assert received_command["type"] == "actuator_command"

    finally:
        client.close()
        server.close()


# ---------------------------------------------------------------------
# Latest-message semantics
# ---------------------------------------------------------------------

def test_receive_latest_discards_older_messages(tmp_path):

    server, client = create_connected_pair(tmp_path)

    try:
        for sequence in range(1, 6):

            server.send({
                "type": "vehicle_state",
                "version": 1,
                "sequence": sequence,
                "timestamp_ns": sequence * 1000,
                "depth_m": float(sequence),
            })

        # All packets have been queued locally.
        latest = wait_for_message(client)

        assert latest["sequence"] == 5
        assert latest["depth_m"] == pytest.approx(5.0)

        # Queue should now be empty.
        assert client.receive_latest() is None

    finally:
        client.close()
        server.close()


# ---------------------------------------------------------------------
# Message boundaries
# ---------------------------------------------------------------------

def test_seqpacket_preserves_individual_messages(tmp_path):

    server, client = create_connected_pair(tmp_path)

    messages = [
        {
            "type": "test",
            "sequence": 1,
        },
        {
            "type": "test",
            "sequence": 2,
        },
        {
            "type": "test",
            "sequence": 3,
        },
    ]

    try:
        # Send raw packets so we can verify the OS-level message boundaries.
        for message in messages:

            payload = json.dumps(message).encode("utf-8")

            server.connection.send(payload)

        received_packets = []

        for _ in range(3):

            payload = client.socket.recv(MAX_MESSAGE_SIZE)

            received_packets.append(
                json.loads(payload.decode("utf-8"))
            )

        assert received_packets == messages

    finally:
        client.close()
        server.close()


# ---------------------------------------------------------------------
# Invalid JSON
# ---------------------------------------------------------------------

def test_invalid_json_raises_ipc_error(tmp_path):

    server, client = create_connected_pair(tmp_path)

    try:
        # Bypass normal send() deliberately so malformed data reaches
        # the receiver.
        server.connection.send(
            b"{this-is-not-valid-json"
        )

        with pytest.raises(
            IPCError,
            match="Invalid JSON",
        ):
            wait_for_message(client)

    finally:
        client.close()
        server.close()


# ---------------------------------------------------------------------
# NaN / Infinity protection
# ---------------------------------------------------------------------

def test_nan_cannot_be_serialized(tmp_path):

    server, client = create_connected_pair(tmp_path)

    message = {
        "type": "vehicle_state",
        "depth_m": float("nan"),
    }

    try:
        # json.dumps(..., allow_nan=False) should reject this.
        with pytest.raises(ValueError):
            server.send(message)

    finally:
        client.close()
        server.close()


def test_infinity_cannot_be_serialized(tmp_path):

    server, client = create_connected_pair(tmp_path)

    message = {
        "type": "vehicle_state",
        "u_mps": float("inf"),
    }

    try:
        with pytest.raises(ValueError):
            server.send(message)

    finally:
        client.close()
        server.close()


# ---------------------------------------------------------------------
# Maximum size
# ---------------------------------------------------------------------

def test_oversized_message_is_rejected(tmp_path):

    server, client = create_connected_pair(tmp_path)

    huge_message = {
        "type": "test",
        "payload": "x" * (MAX_MESSAGE_SIZE + 1000),
    }

    try:
        with pytest.raises(
            IPCError,
            match="IPC message too large",
        ):
            server.send(huge_message)

    finally:
        client.close()
        server.close()


# ---------------------------------------------------------------------
# Disconnect handling
# ---------------------------------------------------------------------

def test_server_detects_client_disconnect(tmp_path):

    server, client = create_connected_pair(tmp_path)

    try:
        client.close()

        deadline = time.monotonic() + 1.0

        disconnected = False

        while time.monotonic() < deadline:

            try:
                server.receive_latest()

            except IPCDisconnected:
                disconnected = True
                break

            time.sleep(0.001)

        assert disconnected, (
            "Server did not detect client disconnect"
        )

    finally:
        server.close()


def test_client_detects_server_disconnect(tmp_path):

    server, client = create_connected_pair(tmp_path)

    try:
        # Close only the connected server socket first.
        server.connection.close()
        server.connection = None

        deadline = time.monotonic() + 1.0

        disconnected = False

        while time.monotonic() < deadline:

            try:
                client.receive_latest()

            except IPCDisconnected:
                disconnected = True
                break

            time.sleep(0.001)

        assert disconnected, (
            "Client did not detect server disconnect"
        )

    finally:
        client.close()
        server.close()


# ---------------------------------------------------------------------
# Filesystem cleanup
# ---------------------------------------------------------------------

def test_server_close_removes_socket_file(tmp_path):

    socket_path = str(tmp_path / "auv_cleanup.sock")

    server = UnixSeqPacketServer(socket_path)
    client = UnixSeqPacketClient(
        socket_path,
        retry_interval=0.01,
    )

    thread = threading.Thread(
        target=server.start,
        daemon=True,
    )

    thread.start()

    client.connect()

    thread.join(timeout=2.0)

    assert socket_path

    try:
        import os

        assert os.path.exists(socket_path)

    finally:
        client.close()
        server.close()

    assert not os.path.exists(socket_path)