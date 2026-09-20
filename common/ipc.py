import json
import os
import socket
import time
from typing import Optional, Dict, Any


MAX_MESSAGE_SIZE = 64 * 1024


class IPCError(Exception):
    pass


class IPCDisconnected(IPCError):
    pass


class UnixSeqPacketServer:
    """
    Simulator-side IPC endpoint.

    Creates a Unix-domain SOCK_SEQPACKET socket and waits for one controller
    process to connect.
    """

    def __init__(self, socket_path: str):
        self.socket_path = socket_path
        self.server_socket: Optional[socket.socket] = None
        self.connection: Optional[socket.socket] = None

    def start(self):
        # Remove stale socket file from previous run.
        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)

        self.server_socket = socket.socket(
            socket.AF_UNIX,
            socket.SOCK_SEQPACKET,
        )

        self.server_socket.bind(self.socket_path)
        self.server_socket.listen(1)

        print(f"[IPC] Waiting for controller at {self.socket_path}")

        self.connection, _ = self.server_socket.accept()

        # Non-blocking receive:
        # simulator should never wait for controller messages.
        self.connection.setblocking(False)

        print("[IPC] Controller connected")

    def send(self, message: Dict[str, Any]):
        if self.connection is None:
            raise IPCDisconnected("Controller is not connected")

        payload = json.dumps(
            message,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")

        if len(payload) > MAX_MESSAGE_SIZE:
            raise IPCError("IPC message too large")

        try:
            self.connection.send(payload)

        except (BrokenPipeError, ConnectionResetError):
            raise IPCDisconnected("Controller disconnected")

    def receive_latest(self) -> Optional[Dict[str, Any]]:
        """
        Drain all currently queued packets and return only the newest one.

        This is useful for control systems where the newest command/state
        matters more than processing old queued messages.
        """

        if self.connection is None:
            return None

        latest = None

        while True:
            try:
                payload = self.connection.recv(MAX_MESSAGE_SIZE)

                if not payload:
                    raise IPCDisconnected("Controller disconnected")

                latest = json.loads(payload.decode("utf-8"))

            except BlockingIOError:
                # No more packets currently available.
                break

            except json.JSONDecodeError as exc:
                raise IPCError(f"Invalid JSON received: {exc}")

            except ConnectionResetError:
                raise IPCDisconnected("Controller disconnected")

        return latest

    def close(self):
        if self.connection:
            self.connection.close()

        if self.server_socket:
            self.server_socket.close()

        if os.path.exists(self.socket_path):
            os.unlink(self.socket_path)


class UnixSeqPacketClient:
    """
    Controller-side IPC endpoint.
    """

    def __init__(
        self,
        socket_path: str,
        retry_interval: float = 0.5,
    ):
        self.socket_path = socket_path
        self.retry_interval = retry_interval
        self.socket: Optional[socket.socket] = None

    def connect(self):
        self.socket = socket.socket(
            socket.AF_UNIX,
            socket.SOCK_SEQPACKET,
        )

        print(f"[IPC] Connecting to simulator at {self.socket_path}")

        while True:
            try:
                self.socket.connect(self.socket_path)
                break

            except (FileNotFoundError, ConnectionRefusedError):
                time.sleep(self.retry_interval)

        self.socket.setblocking(False)

        print("[IPC] Connected to simulator")

    def send(self, message: Dict[str, Any]):
        if self.socket is None:
            raise IPCDisconnected("Not connected")

        payload = json.dumps(
            message,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")

        if len(payload) > MAX_MESSAGE_SIZE:
            raise IPCError("IPC message too large")

        try:
            self.socket.send(payload)

        except (BrokenPipeError, ConnectionResetError):
            raise IPCDisconnected("Simulator disconnected")

    def receive_latest(self) -> Optional[Dict[str, Any]]:
        if self.socket is None:
            return None

        latest = None

        while True:
            try:
                payload = self.socket.recv(MAX_MESSAGE_SIZE)

                if not payload:
                    raise IPCDisconnected("Simulator disconnected")

                latest = json.loads(payload.decode("utf-8"))

            except BlockingIOError:
                break

            except json.JSONDecodeError as exc:
                raise IPCError(f"Invalid JSON received: {exc}")

            except ConnectionResetError:
                raise IPCDisconnected("Simulator disconnected")

        return latest

    def close(self):
        if self.socket:
            self.socket.close()