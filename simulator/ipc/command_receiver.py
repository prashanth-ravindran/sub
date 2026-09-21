"""Bounded command polling and a reconnectable, nonblocking server."""

import json
import os
import socket
import time

from common.ipc import MAX_MESSAGE_SIZE, UnixSeqPacketServer
from common.messages import (
    command_is_expired, get_actuator_values, validate_actuator_command_or_raise,
)


class SimulatorServer(UnixSeqPacketServer):
    """Reuse packet sending without the shared server's blocking startup/unlink."""

    def __init__(self, socket_path):
        super().__init__(socket_path)
        self._owned_identity = None

    def start(self):
        if os.path.lexists(self.socket_path):
            raise FileExistsError(f"Socket path already exists: {self.socket_path}")
        self.server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        self.server_socket.setblocking(False)
        self.server_socket.bind(self.socket_path)
        stat = os.lstat(self.socket_path)
        self._owned_identity = (stat.st_dev, stat.st_ino)
        self.server_socket.listen(1)

    def accept_pending(self):
        if self.connection is None:
            try:
                self.connection, _ = self.server_socket.accept()
                self.connection.setblocking(False)
            except BlockingIOError:
                pass

    def disconnect(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def close(self):
        self.disconnect()
        if self.server_socket is not None:
            self.server_socket.close()
            self.server_socket = None
        if self._owned_identity is not None:
            try:
                stat = os.lstat(self.socket_path)
                if (stat.st_dev, stat.st_ino) == self._owned_identity:
                    os.unlink(self.socket_path)
            except FileNotFoundError:
                pass
            self._owned_identity = None


class CommandReceiver:
    def __init__(self):
        self.command = None
        self.invalid_packets = 0
        self._connection = None
        self._sequence = -1

    def poll(self, server: SimulatorServer, *, now_ns=None, enabled=True):
        """Hold the newest valid command until its monotonic wall-clock expiry.

        Poll at most 64 packets per tick so a busy sender cannot starve physics.
        Scenario mode drains commands but does not use them.
        """
        if now_ns is None:
            now_ns = time.monotonic_ns()
        if self._connection is not server.connection:
            self.command = None
            self._sequence = -1
            self._connection = server.connection
        for _ in range(64):
            if server.connection is None:
                break
            try:
                payload, _, flags, _ = server.connection.recvmsg(MAX_MESSAGE_SIZE)
            except BlockingIOError:
                break
            except ConnectionResetError:
                server.disconnect()
                self.command = None
                break
            if not payload:
                server.disconnect()
                self.command = None
                break
            if not enabled:
                continue
            try:
                if flags & socket.MSG_TRUNC:
                    raise ValueError("Truncated command")
                command = json.loads(payload.decode("utf-8"))
                validate_actuator_command_or_raise(command, check_expiry=True, now_ns=now_ns)
                if command["sequence"] <= self._sequence:
                    raise ValueError("Out-of-order command")
            except (ValueError, OverflowError, RecursionError):
                self.invalid_packets += 1
                continue
            self.command = command
            self._sequence = command["sequence"]
        if not enabled or self.command is None or command_is_expired(self.command, now_ns):
            return (0.0, 0.0, 0.0)
        return get_actuator_values(self.command)
