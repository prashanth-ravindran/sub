# sub

## Getting started

On a fresh Linux machine, install Python and the virtual-environment support
first:

```bash
sudo apt update
sudo apt install -y python3 python3-venv
```

The startup scripts create `.venv` automatically and install the pinned
packages from `requirements.txt` before launching the services. The first run
can take a few minutes while the virtual environment and scientific Python
dependencies are created. Later runs reuse the environment and are much
faster.

To run the vehicle simulator and its Streamlit UI:

```bash
scripts/start_vehicle_simulator.sh
```

Open the simulator UI at `http://127.0.0.1:8501`. Its local API listens on
port `8765`.

To run the controller and its Streamlit UI:

```bash
scripts/start_controller.sh
```

Open the controller UI at `http://127.0.0.1:8502`. Its local API listens on
port `8766`. Starting the controller does not require the standalone simulator
to be running. When a mission is submitted, the controller API automatically
starts a dedicated simulator process with its own Unix IPC socket, connects to
it over the repository IPC library, and stops that process when the mission
reaches its goal or time horizon.

The standalone simulator script is only needed when you want to run the
simulator independently, inspect its UI, or drive it from another client.

The scripts support `PYTHON_BIN`, `CONTROLLER_API_PORT`,
`CONTROLLER_UI_PORT`, `SIMULATOR_API_PORT`, and `SIMULATOR_UI_PORT` environment
overrides. Stop the script with Ctrl+C to stop both processes it started.

Why SOCK_SEQPACKET?

- Preserves message boundaries. Each send() corresponds to one packet at the receiver. With SOCK_STREAM, you only get a byte stream and must implement your own framing.
- Reliable and ordered. Messages arrive in order or the connection fails; you do not have to build retransmission logic yourself.
- Local IPC only. Using AF_UNIX avoids unnecessary TCP/IP networking overhead when simulator and controller run on the same machine.
- Simple dependency footprint. Python’s standard socket module is enough. No ROS 2, DDS, ZeroMQ, gRPC, broker, or external runtime is required.
- Works well with control semantics. You can make the socket nonblocking, drain queued packets, and keep only the newest state or command rather than processing stale backlog.
- Full duplex. The same connection supports simulator → controller state messages and controller → simulator actuator commands.
- Easy failure detection. Disconnects, broken pipes, and missing data are explicit and easy to test.

Other options considered are below

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

The controller is a waypoint-following autopilot for a torpedo-shaped autonomous underwater vehicle (AUV). It drives the vehicle toward a
  target latitude/longitude while maintaining a requested depth and forward speed.

It controls three actuators:

| Objective | Feedback | Controlled actuator |
|---|---|---|
| Forward speed | Surge velocity and acceleration | Propeller RPM |
| Depth and pitch | Depth, depth rate, pitch, pitch rate | Elevator |
| Heading toward waypoint | Position, yaw, yaw rate | Rudder |

  ### Controller - How it works

  1. Waypoint guidance

     The target latitude/longitude is converted into local north/east coordinates. Each cycle, the controller calculates:
      - Remaining north and east distance
      - Horizontal distance to the waypoint
      - Desired heading using atan2(east_error, north_error)

  2. Depth control

     This is a cascaded controller:

     depth error → desired pitch → elevator command

     The depth PID converts the difference between requested and actual depth into a pitch setpoint. The pitch PID then uses pitch error and
     pitch rate to command the elevator.

  3. Heading control

     The desired heading is compared with the current yaw. The error is wrapped into [-π, π], ensuring the vehicle takes the shortest turn. A
     PID controller then converts this error and yaw rate into a rudder command.

  4. Speed control

     The speed controller combines two terms:
      - Feed-forward RPM calculated from the shared Fossen model’s expected drag at the requested speed
      - PID correction based on surge-speed error and acceleration

     The feed-forward term provides approximately the thrust required to maintain speed, while the PID handles disturbances and transient
     errors.

# References

https://www.fossen.biz/html/marineCraftModel.html - Used it to understand fossen model better
https://man7.org/linux/man-pages/man7/unix.7.html - UNIX manpages for SOCK_SEQPACKET
https://docs.python.org/3/library/socket.html - SOCK_SEQPACKET in python
https://github.com/cybergalactic/FossenHandbook - The full fossen textbook slides
https://github.com/cybergalactic/MSS - Used to scout for reasonable parameters
