# AUV waypoint controller

The controller runs separately from the vehicle simulator. It owns the sole
Unix `SOCK_SEQPACKET` connection to its simulator process and uses the shared
`common.ipc` transport and `common.messages` version-1 state/actuator schema.
The Streamlit UI is another process and talks only to the controller's local
HTTP API.

From the repository root, install `requirements.txt`, then choose one mode:

```bash
scripts/start_vehicle_simulator.sh  # Vehicle Simulator API + UI, ports 8765/8501
scripts/start_controller.sh         # Controller API + UI, ports 8766/8502
```

The controller API starts a fresh `python -m simulator` process when a mission
is submitted. Its simulator uses a unique Unix socket, the mission start
latitude/longitude as NED origin, initial depth 0.1 m, default vehicle
parameters, a 100 Hz physics step, and synchronized accelerated time. The
controller updates at 20 Hz. The scratch PID, LQR, and LQI control laws and
gains are preserved; `linearization.py` uses the exact scratch `AUV.derivative`.
The flat-earth waypoint conversion, units, and actuator limits are unchanged.

In synchronized mode, the simulator publishes its initial state at sequence 0.
The controller replies with a command referencing that sequence, then the
simulator integrates five 0.01 s steps, publishing each state. At sequence 5
it waits for the next matching command. A goal or manual stop uses an IPC stop
message. The time horizon also ends at a control boundary. Missing commands,
lost connections, and startup failures time out instead of advancing open
loop. Uncontrolled simulator runs keep their prior accelerated behavior.

The controller API accepts one active run:

| Method and path | Purpose |
| --- | --- |
| `GET /api/config` | Mission defaults and controller choices |
| `POST /api/runs` | Start a PID, LQR, or LQI mission |
| `GET /api/runs/{id}` | Status, latest simulator state and controller output |
| `POST /api/runs/{id}/stop` | Stop at the next control boundary |
| `GET /api/runs/{id}/csv` | Download 20 Hz state/control samples |
| `GET /api/runs/{id}/state-csv` | Download all 100 Hz simulator states |

Example request:

```json
{"controller_type":"lqi","duration_s":450,"mission":{"target_lat":12.9752,"target_lon":80.2246}}
```

Omitted mission fields use the scratch `Mission` defaults. The controller UI
shows the latest state and actuator command once per second while running,
then renders the trajectory and control charts from the completed CSV. It
never processes UI events at the 100 Hz physics rate. Download results before
starting another run; the service retains only the latest run.
