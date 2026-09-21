"""Best-effort telemetry: a slow controller must not block integration."""

from common.ipc import IPCDisconnected


class StatePublisher:
    def __init__(self):
        self.published = 0
        self.dropped = 0

    def publish(self, server, message):
        if server.connection is None:
            return
        try:
            server.send(message)
            self.published += 1
        except BlockingIOError:
            self.dropped += 1
        except IPCDisconnected:
            server.disconnect()
