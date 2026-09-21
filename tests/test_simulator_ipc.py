import json
import socket

import numpy as np
import pytest

from common.ipc import MAX_MESSAGE_SIZE
from common.messages import make_actuator_command, validate_vehicle_state_or_raise
from simulator.ipc.command_receiver import CommandReceiver, SimulatorServer
from simulator.ipc.messages import state_message
from simulator.ipc.state_publisher import StatePublisher


@pytest.fixture
def server(tmp_path):
    endpoint = SimulatorServer(str(tmp_path / "s"))
    try:
        endpoint.start()
        yield endpoint
    finally:
        endpoint.close()


def connect(server):
    client = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    client.settimeout(1)
    client.connect(server.socket_path)
    server.accept_pending()
    assert server.connection is not None
    return client


def command(sequence=1, rpm=1800, now_ns=1000):
    return make_actuator_command(
        sequence=sequence, propeller_rpm=rpm, elevator_deg=1, rudder_deg=2,
        timestamp_ns=now_ns, validity_ns=100,
    )


def test_latest_valid_command_hold_expiry_disconnect_and_reconnect(server):
    receiver = CommandReceiver()
    assert receiver.poll(server, now_ns=1000) == (0, 0, 0)
    with connect(server) as client:
        for sequence in (1, 2, 3):
            client.send(json.dumps(command(sequence, rpm=sequence * 500)).encode())
        assert receiver.poll(server, now_ns=1000) == (1500, 1, 2)
        assert receiver.poll(server, now_ns=1100) == (1500, 1, 2)
        assert receiver.poll(server, now_ns=1101) == (0, 0, 0)
    assert receiver.poll(server, now_ns=1101) == (0, 0, 0)
    assert server.connection is None
    with connect(server) as client:
        client.send(json.dumps(command(0, now_ns=2000)).encode())
        assert receiver.poll(server, now_ns=2000) == (1800, 1, 2)


def test_bad_packets_do_not_erase_valid_command_or_crash(server):
    receiver = CommandReceiver()
    with connect(server) as client:
        client.send(json.dumps(command()).encode())
        bad_commands = [
            b"{", b"\xff", b"[]", b"null", b'{"type":"other"}',
            json.dumps(command(0)).encode(),  # Out of order.
            json.dumps(command(2, now_ns=0)).encode(),  # Expired.
            json.dumps(command()).replace('1800', 'NaN').encode(),
            json.dumps(command()).replace('1800', 'true').encode(),
            json.dumps(command()).replace('1800', '9' * 400).encode(),
            b"[" * 2000 + b"]" * 2000,
            b" " * (MAX_MESSAGE_SIZE + 1),
        ]
        # Poll between oversized packets to avoid filling the sender's buffer.
        assert receiver.poll(server, now_ns=1000) == (1800, 1, 2)
        for payload in bad_commands:
            client.send(payload)
            assert receiver.poll(server, now_ns=1000) == (1800, 1, 2)
        assert receiver.invalid_packets == len(bad_commands)
        assert receiver.poll(server, now_ns=1101) == (0, 0, 0)


def test_poll_is_bounded_and_scenario_mode_ignores_commands(server):
    receiver = CommandReceiver()
    with connect(server) as client:
        for sequence in range(65):
            client.send(json.dumps(command(sequence)).encode())
        receiver.poll(server, now_ns=1000)
        assert receiver.command["sequence"] == 63
        receiver.poll(server, now_ns=1000)
        assert receiver.command["sequence"] == 64
        assert receiver.poll(server, now_ns=1000, enabled=False) == (0, 0, 0)


def test_publisher_validates_state_and_counts_backpressure_without_blocking(server):
    state = np.zeros(13)
    state[2], state[3] = 50, 1
    message = state_message(state, 1, 0.01)
    validate_vehicle_state_or_raise(message)
    assert message["depth_m"] == message["position"]["down_m"] == 50
    publisher = StatePublisher()
    with connect(server) as client:
        publisher.publish(server, message)
        assert json.loads(client.recv(MAX_MESSAGE_SIZE)) == message
        server.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4096)
        for _ in range(100):
            publisher.publish(server, message)
        assert publisher.dropped > 0
        assert publisher.published + publisher.dropped == 101
    publisher.publish(server, message)
    assert server.connection is None


def test_existing_socket_is_not_unlinked_and_cleanup_preserves_replacement(server, tmp_path):
    second = SimulatorServer(server.socket_path)
    try:
        with pytest.raises(FileExistsError):
            second.start()
    finally:
        second.close()
    with connect(server):
        pass
    path = tmp_path / "s"
    path.unlink()
    path.write_text("replacement")
    server.close()
    assert path.read_text() == "replacement"


def test_existing_file_and_dangling_symlink_are_preserved(tmp_path):
    for path in (tmp_path / "file", tmp_path / "link"):
        if path.name == "file":
            path.write_text("keep")
        else:
            path.symlink_to(tmp_path / "missing")
        server = SimulatorServer(str(path))
        try:
            with pytest.raises(FileExistsError):
                server.start()
        finally:
            server.close()
        assert path.is_symlink() or path.read_text() == "keep"
