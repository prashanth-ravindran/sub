# sub

Why SOCK_SEQPACKET?

- Preserves message boundaries. Each send() corresponds to one packet at the receiver. With SOCK_STREAM, you only get a byte stream and must implement your own framing.
- Reliable and ordered. Messages arrive in order or the connection fails; you do not have to build retransmission logic yourself.
- Local IPC only. Using AF_UNIX avoids unnecessary TCP/IP networking overhead when simulator and controller run on the same machine.
- Simple dependency footprint. Python’s standard socket module is enough. No ROS 2, DDS, ZeroMQ, gRPC, broker, or external runtime is required.
- Works well with control semantics. You can make the socket nonblocking, drain queued packets, and keep only the newest state or command rather than processing stale backlog.
- Full duplex. The same connection supports simulator → controller state messages and controller → simulator actuator commands.
- Easy failure detection. Disconnects, broken pipes, and missing data are explicit and easy to test.

| Option        | Why not preferred here                                                                |
| ------------- | ------------------------------------------------------------------------------------- |
| `SOCK_STREAM` | Reliable, but requires manual message framing                                         |
| TCP sockets   | Useful across machines, unnecessary complexity locally                                |
| UDP           | Message-oriented, but unreliable                                                      |
| Shared memory | Very fast, but synchronization and consistency become your responsibility             |
| Named pipes   | Simple, but less convenient for structured bidirectional messaging                    |
| ZeroMQ        | Good API, but adds another dependency and abstraction layer                           |
| gRPC          | Far too heavy for 100 Hz local control messages                                       |
| ROS 2/DDS     | Excellent robotics ecosystem, but much more infrastructure than this assignment needs |


# References

https://www.fossen.biz/html/marineCraftModel.html - Used it to understand fossen model better
https://man7.org/linux/man-pages/man7/unix.7.html - UNIX manpages for SOCK_SEQPACKET
https://docs.python.org/3/library/socket.html - SOCK_SEQPACKET in python
