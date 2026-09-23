# AUV waypoint controller

## Purpose and data flow

The controller is a separate process that guides the six-degree-of-freedom
simulator toward a latitude/longitude waypoint while tracking depth and
forward (surge) speed. The simulator owns the vehicle dynamics, clock, and
state; the controller owns waypoint guidance and the selectable PID, LQR, or
LQI law. The controller UI talks to its local HTTP API, not to the simulator.
When a mission starts, the API launches one dedicated simulator child process
with a unique Unix socket. It accepts only one active mission at a time.

| Component | Responsibility |
| --- | --- |
| `mission.py`, `guidance.py` | Validate mission input; convert latitude/longitude to local north/east; update target bearing and range. |
| `pid.py`, `lqr.py`, `lqi.py` | Map telemetry and setpoints to RPM, elevator, and rudder commands. |
| `linearization.py` | Numerically linearize the simulator-owned Fossen derivative at cruise for LQR/LQI. |
| `controller.py`, `ipc.py` | Own the simulator process, 20 Hz synchronized loop, IPC connection, stop conditions, and failure handling. |
| `http_api.py`, `../ui/controller_app.py` | Run-control API and separate Streamlit UI. |
| `logger.py`, `visualizer.py` | Control-boundary CSV and post-run plots. |

The simulator owns the Fossen model in `simulator/scratch/fossen.py`, including
its ellipsoidal submergence calculation. LQR/LQI call that model's derivative
for linearization; the controller does not maintain a second vehicle model.
The root [architecture](../README.md#architecture) and
[IPC section](../README.md#inter-process-communication-ipc) describe the
process boundaries, packet fields, sequence numbers, and timeouts.

## Mission and timing

The default mission starts at 12.9716° N, 80.2209° E, level and motionless,
with the vehicle's centre of gravity at the waterline (zero depth). The target
is 12.9752° N, 80.2246° E, about 567 m northeast. Desired depth is 10 m,
desired surge speed is 1.5 m/s, and arrival radius is 15 m. Duration defaults
to 450 s of simulated time.

The start coordinate defines the local North-East-Down origin. For these
short missions, guidance uses a flat-earth latitude/longitude conversion.
Each control cycle it computes the remaining north/east displacement,
horizontal range, and target heading with `atan2(east_error, north_error)`.
Heading error is wrapped to $[-\pi,\pi)$ so the requested turn is shortest.
Arrival is based on horizontal range alone; depth and speed are tracked but
are **not** additional arrival conditions.

The controller runs at a fixed 20 Hz simulated-time rate. The simulator's
physics and nominal state-publication rate can be selected as 60, 80, or
100 Hz, so each control interval contains three, four, or five RK4 steps.
100 Hz is the default. At sequence 0 and every control boundary, the
simulator waits for a command referencing that state sequence; it does not
advance open-loop if the command is missing. A matching stop packet ends the
run on arrival, manual stop, or the time horizon. Startup, communication,
and child-process failures are reported instead of silently continuing.
Simulated time is accelerated by default, not paced to wall-clock time.

## Control laws

All three modes drive the same actuator vector: propeller RPM, paired
elevator deflection, and paired rudder deflection. RPM is clipped to 0–3000;
each fin command is clipped to ±20°. Depth is positive down in NED. A dive
therefore needs a negative pitch setpoint (nose down) in the PID cascade.
Euler angles are shown in telemetry and plots, while the simulator integrates
attitude internally as a quaternion.

### PID

Four PID loops control depth, pitch, heading, and surge speed. Depth error
produces a pitch setpoint; the pitch loop produces elevator deflection.
Heading error and yaw rate produce rudder deflection. Speed control adds a
PID correction to the RPM that balances the model's estimated steady surge
drag at the requested speed:

```math
\begin{aligned}
I_{k+1} &= \mathrm{clip}(I_k+\Delta t\,e_k, -I_{\max}, I_{\max}), \\
c_k &= \mathrm{clip}(K_p e_k+K_i I_{k+1}-K_d\dot y_k, -c_{\max}, c_{\max}), \\
n_{\mathrm{trim}} &= 60\sqrt{\frac{d_u u^*+d_{u2}(u^*)^2}{k_{\mathrm{prop}}}}.
\end{aligned}
```

The derivative term acts on the measured rate rather than the setpoint,
avoiding a derivative kick when guidance changes the requested heading.
Each integral is clamped; the resulting command is also saturated. The
drag coefficients $d_u$ and $d_{u2}$ and propeller coefficient
$k_{\mathrm{prop}}$ come from the shared vehicle parameters. Angles and
angular rates enter the loops in radians and radians per second; fin
commands are in degrees. Integral limits are in each loop's error-times-time
units. The gains and limits are the implemented demonstration values, not
parameters identified from a physical vehicle:

| Loop | $K_p$ | $K_i$ | $K_d$ | Output limit | Integral limit |
| --- | ---: | ---: | ---: | ---: | ---: |
| Depth → pitch magnitude | 0.040 | 0.00035 | 0.12 | 22° | 10 |
| Pitch → elevator | 45 | 1.0 | 16 | 20° | 3 |
| Heading → rudder | 28 | 0.55 | 18 | 20° | 3 |
| Speed → RPM correction | 320 | 35 | 45 | 1200 RPM | 20 |

### LQR

LQR handles coupling among depth, roll, pitch, heading, BODY velocities,
and angular rates with one state-feedback law. It linearizes the
simulator-owned `AUV.derivative` by central finite differences at level
cruise: the requested depth and surge speed, zero lateral/vertical velocity
and angular rates, zero fin deflection, and steady-drag trim RPM. The default
10 m trim is fully submerged. The continuous local model is discretized
with a zero-order hold at 0.05 s; a discrete Riccati equation gives $K$:

```math
\begin{aligned}
\mathbf e_k &= [d-d^*,\;\phi,\;\theta,\;\mathrm{wrap}(\psi-\psi^*),\;u-u^*,\;v,\;w,\;p,\;q,\;r]^{\mathsf T}, \\
\mathbf e_{k+1} &\simeq \mathbf A_d\mathbf e_k+\mathbf B_d(\mathbf a_k-\mathbf a_{\mathrm{trim}}), \\
\mathbf a_k &= \mathrm{clip}(\mathbf a_{\mathrm{trim}}-\mathbf K\mathbf e_k).
\end{aligned}
```

The diagonal $Q$ weights are the inverse squares of state-error scales:
5 m depth; 10°, 15°, and 30° for roll, pitch, and heading; 0.3, 0.5, and
0.5 m/s for $u$, $v$, and $w$; and 10°/s, 15°/s, and 15°/s for $p$, $q$,
and $r$. The diagonal $R$ weights use 800 RPM and 20° for each fin. These
scales express the relative tracking and actuator-use trade-off. The
closed-loop eigenvalues of the *local, unsaturated, fixed-reference* default
linear model lie inside the unit circle; this is not a global stability
guarantee for a turning vehicle or a surface departure. LQR has no integral
states, so persistent model error or actuator bias can leave a steady
tracking offset.

### LQI and why it was added

The initial LQR implementation was adequate for the nominal mission but did
not force zero steady error under changed drag or biased fins. LQI was added
later, before the surface-buoyancy change, specifically to address those
offsets. It retains the LQR plant model and augments it with accumulated
depth, surge-speed, and wrapped-heading errors:

```math
\mathbf z_{k+1}=\mathbf z_k+\Delta t\,[e_d,\;e_u,\;e_\psi]^{\mathsf T},\qquad
\mathbf a_k=\mathrm{clip}\!\left(\mathbf a_{\mathrm{trim}}-
\mathbf K\begin{bmatrix}\mathbf e_k\\\mathbf z_k\end{bmatrix}\right).
```

The implementation limits the three integrals to ±6 m·s, ±20 m, and ±2π
rad·s and conditionally freezes an increment that would push a saturated
actuator farther into saturation. Relative to LQR, the LQI tuning raises
the depth-error weight by 25× and the pitch-error weight by 100×; its
integral-error scales are 30 m·s, 5 m, and 360°·s. These choices were tuned
for bounded commands and a controlled dive transient, not for a general
optimality or robustness guarantee.

The historical scratch robustness check doubles quadratic surge drag and
adds +2° elevator and +1° rudder bias at the actuator. From a submerged
cruise trim, with a 0.1 s control step for 180 s, the comparative final
tracking errors were:

| Law | Depth error | Surge-speed error | Heading error |
| --- | ---: | ---: | ---: |
| LQR | −0.523 m | −0.127 m/s | +1.553° |
| LQI | −0.0002 m | −0.00001 m/s | approximately 0° |

This is a specific model-mismatch experiment, not the default IPC mission
or a guarantee that LQI is always preferable. Integral action adds state
and tuning complexity and can worsen transients if poorly tuned. The
scratch self-test exercises the biased LQI case and its anti-windup rules.

## Run and inspect a mission

From the repository root on Linux, start the controller API and Streamlit UI:

```bash
scripts/start_controller.sh
```

The script creates `.venv` and installs pinned requirements if needed.
Open `http://127.0.0.1:8502`; the API listens on `127.0.0.1:8766`.
The controller UI lets you set start/target coordinates, depth, speed,
arrival radius, controller mode, horizon, and simulator rate. The separate
standalone simulator service does **not** need to be started. The ports can
be changed with `CONTROLLER_UI_PORT` and `CONTROLLER_API_PORT`.

The same run can be submitted over HTTP:

```bash
curl -sS -X POST http://127.0.0.1:8766/api/runs \
  -H 'Content-Type: application/json' \
  -d '{"controller_type":"lqi","simulator_frequency_hz":100,"duration_s":450,"mission":{"start_lat":12.9716,"start_lon":80.2209,"target_lat":12.9752,"target_lon":80.2246,"depth_m":10,"speed_mps":1.5,"arrival_radius_m":15}}'
```

The response contains a `run_id`. Omitted fields use the defaults above.
Useful endpoints are:

| Method and path | Purpose |
| --- | --- |
| `GET /api/config` | Defaults and supported controller/simulator rates. |
| `POST /api/runs` | Start one mission; returns HTTP 202 and a `run_id`. |
| `GET /api/runs/{id}` | Poll status and latest state/command; completion reasons are `goal`, `horizon`, or `stopped`. |
| `POST /api/runs/{id}/stop` | Request a stop at the next control boundary. |
| `GET /api/runs/{id}/csv` | Download controller samples at 20 Hz boundaries. |
| `GET /api/runs/{id}/state-csv` | Download every completed simulator physics step at the selected rate. |

The controller CSV includes time, NED and geographic position, actual and
requested depth/heading/speed/pitch, roll, target range, commands, and BODY
velocities/rates. Once a run completes, the UI renders a labelled nine-panel
figure: requested versus actual depth, heading, speed, and pitch; roll; RPM,
elevator, and rudder; and the north/east trajectory with start and target.
The simulator CSV resolves the intervening physics steps. Download both
before starting another run; only the latest run is retained by the service.

For reference, a default surface-start mission at 100 Hz produced this
sample output in simulated time. These are observed results, not fixed
arrival-time promises:

| Mode | Arrival time | Final depth | Final surge speed | Final target range |
| --- | ---: | ---: | ---: | ---: |
| PID | 369.80 s | 9.9865 m | 1.5000 m/s | 14.9883 m |
| LQR | 370.25 s | 10.0000 m | 1.5000 m/s | 14.9676 m |
| LQI | 369.70 s | 10.0000 m | 1.5000 m/s | 14.9905 m |

In each case the vehicle departed zero depth, commands stayed within their
limits, and the goal was reached. The repo's `tests/` folder contains the
controller model, service, and UI tests, including surface-start arrival
checks for all three laws at 60, 80, and 100 Hz. Run them with:

```bash
.venv/bin/python -m pytest -q tests/test_controller_models.py tests/test_controller_service.py tests/test_controller_ui.py
```

## Interpretation and limits

The nominal depth/speed traces and trajectory demonstrate waypoint tracking
for this illustrative plant. PID is easy to inspect and tune loop by loop;
LQR couples the vehicle states and actuator penalties but assumes its local
linearization and trim remain representative; LQI can remove persistent
offsets but needs anti-windup and more tuning. All three use the same
saturated actuator model and stop at the same horizontal arrival radius.

The plant coefficients are illustrative rather than measured. The flat-earth
guidance approximation is suitable for the short example, not long routes.
The LQR/LQI gain is recomputed for the requested depth and speed at startup,
but not continuously relinearized as attitude, depth, or speed change. The
default gain is designed around fully submerged level cruise even though the
mission begins at the surface. There is no environmental current, sensor
noise, obstacle avoidance, or real-hardware validation in this demonstration.
