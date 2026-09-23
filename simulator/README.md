# Underwater vehicle simulator


## Structure

| Module in `simulator.vehicle` | Responsibility |
| --- | --- |
| `parameters` | Immutable, validated physical parameters |
| `kinematics` | Quaternion/Euler conversions and BODY/NED motion |
| `rigid_body` | Rigid-body mass matrix about CG |
| `added_mass`, `coriolis` | Added inertia and rigid-body/added-mass coupling |
| `damping` | Dissipative linear and quadratic drag |
| `hydrostatics` | Weight, buoyancy, and restoring moments |
| `actuators` | Propeller RPM and paired fin deflections to forces/moments |
| `dynamics` | Assemble the thirteen state derivatives |

`simulation/` owns RK4, absolute-deadline timing, and the service loop.
`http_api.py` exposes local run control and completed CSV results to a
separate Streamlit process in `ui/`.
`navigation/geodetic.py` converts NED displacements to latitude/longitude.
`ipc/` adapts the existing `common.ipc` transport and `common.messages` schema.
`scenarios/` contains pure actuator schedules driven by simulation time.
All tests live in the repository's global `tests/` directory.

## Frames, state, and units

The right-handed BODY axes are forward, starboard, and down. The Earth frame
is North-East-Down (NED). The BODY origin is the centre of gravity (CG), and
its axes align with the principal inertia axes. Positive roll lowers the
starboard side, positive pitch raises the nose, and positive yaw turns north
toward east when level.

Use SI units: metres, seconds, kilograms, radians, newtons, and newton-metres.

```text
state = [north, east, down, qw, qx, qy, qz, u, v, w, p, q, r]  # shape (13,)
nu    = [u, v, w, p, q, r]                                  # shape (6,)
tau   = [X, Y, Z, K, M, N]                                  # shape (6,)
```

`u, v, w` and `p, q, r` are BODY linear velocities and angular rates.
`tau` contains applied BODY forces and moments, not RPM or fin deflections.
Depth equals the NED down coordinate if the NED origin is at the water surface.
The origin defaults to latitude 13°, longitude 80°, at the water surface.
The standalone service starts level and stationary at 50 m depth by default;
controller-launched runs start at zero depth. Initial depth and geographic
origin are configurable. At zero depth the CG is at the waterline and all
velocities are zero. The simulator allows the CG to cross the waterline
without clamping its depth; buoyancy changes with the submerged hull volume.

In Fossen's Euler-angle notation, the kinematic Jacobian maps BODY velocities
to NED position rates and Euler-angle rates:

```math
\begin{aligned}
\dot{\boldsymbol\eta} &= \mathbf J(\boldsymbol\eta)\boldsymbol\nu, \\
\mathbf J(\boldsymbol\eta) &=
\begin{bmatrix}
\mathbf R_{zyx}(\phi,\theta,\psi) & \mathbf 0 \\
\mathbf 0 & \mathbf T_{zyx}(\phi,\theta)
\end{bmatrix}.
\end{aligned}
```

Here $\boldsymbol\eta$ combines NED position and Euler attitude.
$\mathbf R_{zyx}$ maps BODY linear velocity to NED position rate, while
$\mathbf T_{zyx}$ maps BODY angular velocity to Euler-angle rate. The Python
`quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)` helper accepts Euler angles
to construct a quaternion state; the CLI and API currently start level and do
not expose an initial-attitude setting. The simulator then integrates
quaternion rates instead of Euler-angle rates because $\mathbf T_{zyx}$ is
singular at $\pm90^\circ$ pitch. State packets and CSV rows convert the
quaternion back to Euler angles for display and control.

Attitude uses Hamilton quaternions in scalar-first order `[qw, qx, qy, qz]`,
rotating BODY vectors into NED. The identity is `[1, 0, 0, 0]`; `q` and `-q`
represent the same orientation. The rotation matrix and its inverse satisfy:

```math
\begin{aligned}
\mathbf v_{\mathrm{NED}} &= \mathbf R\mathbf v_{\mathrm{BODY}}, \\
\mathbf v_{\mathrm{BODY}} &= \mathbf R^{\mathsf T}\mathbf v_{\mathrm{NED}}, \\
\mathbf R &= \mathbf R_z(\psi)\mathbf R_y(\theta)\mathbf R_x(\phi).
\end{aligned}
```

Here $\phi$, $\theta$, and $\psi$ denote roll, pitch, and yaw.
Euler conversions use SciPy's extrinsic `"xyz"` sequence with roll, pitch, yaw
in radians. At pitch ±90°, Euler angles are not unique: conversion retains
SciPy's gimbal-lock warning and returns an equivalent orientation. Internal
quaternion calculations have no Euler singularity.

For angular velocity $\boldsymbol\omega=[p,q,r]^{\mathsf T}$ and quaternion
vector part $\mathbf q_v=[q_x,q_y,q_z]^{\mathsf T}$:

```math
\begin{aligned}
\dot{\mathbf p} &= \mathbf R[u,v,w]^{\mathsf T}, \\
\dot{\mathbf q} &= \tfrac12
\begin{bmatrix}
-\mathbf q_v\cdot\boldsymbol\omega \\
q_w\boldsymbol\omega+\mathbf q_v\times\boldsymbol\omega
\end{bmatrix}.
\end{aligned}
```

BODY angular rates are not generally Euler-angle derivatives. Functions
normalize nonzero quaternions for attitude calculations and reject zero,
malformed, or nonfinite inputs. The integrator must normalize the
quaternion after completed integration steps; these functions do not mutate
the supplied state.

## Parameters and assumptions

`VehicleParameters` exposes these adjustable defaults:

| Field | Default | Meaning |
| --- | --- | --- |
| `mass_kg` | 84.0 | Rigid-body mass |
| `inertia_kg_m2` | `(0.7, 25.0, 25.0)` | Principal roll, pitch, yaw inertias at CG |
| `cb_height_m` | 0.02 | Centre of buoyancy (CB) above CG |
| `gravity_mps2` | 9.8 | Gravitational acceleration |
| `added_mass` | `(5, 80, 80, 0.1, 10, 10)` | Diagonal added mass/inertia, kg and kg·m² |
| `linear_damping` | `(2, 200, 200, 0.5, 400, 400)` | Linear drag coefficients |
| `quadratic_damping` | `(10, 200, 200, 0.1, 50, 50)` | Quadratic drag coefficients |
| `water_density_kg_m3` | 1025 | Seawater density |
| `propeller_thrust_coefficient` | 0.1 | N / (rev/s)² |
| `elevator_area_m2`, `rudder_area_m2` | 0.02 each | Total area of each symmetric pair |
| `fin_lift_slope_per_rad` | 4.0 | Assumed lift slope |
| `fin_x_m` | -0.8 | Fin position aft of CG, metres |
| `hull_length_m` | 2.0 | Length of the ellipsoid used for submergence |

These are illustrative engineering assumptions, not measured coefficients
or values copied from an MSS vehicle. At the default
water density of 1025 kg/m³, 84 kg corresponds to about 0.08195 m³ of full
displacement. A 2 m ellipsoid with that volume has an effective transverse
diameter of about 0.28 m. The assumed inertias are not derived from this
geometry.

When fully submerged, the model is neutrally buoyant: $B=W=mg$. Partial
submergence reduces buoyancy below weight.

CG is `[0, 0, 0]` and the nominal fully submerged CB is
`[0, 0, -cb_height_m]` in BODY coordinates.
Mass, principal inertias, and gravity must be finite and positive. CB height
must be finite and nonnegative; zero removes the restoring couple. Parameter
instances are immutable, including inertia values supplied as a list.
Hydrodynamic coefficient vectors use `[u, v, w, p, q, r]` order and must be
finite and nonnegative. Translational linear/quadratic drag coefficients
have units N/(m/s) and N/(m/s)²; rotational ones have units N·m/(rad/s)
and N·m/(rad/s)². Fin areas, lift slope, propeller coefficient, and density
must be positive, and the fin lever arm must be negative.

The hydrodynamic coefficients are illustrative, independently adjustable
assumptions, **not identified or validated vehicle data**. In particular,
pitch/yaw damping was chosen to give bounded, reproducible demonstration
responses. Change `VehicleParameters` in Python or supply per-run overrides
through the HTTP API when tuning a specific vehicle.

Vertical CB–CG separation gives both roll and pitch restoration. For a fully
submerged vehicle, this separation expresses stability directly; a
surface-vessel waterplane/metacentric model is not used here. At the surface,
only the hydrostatic buoyancy force and moment change with submergence;
added mass, damping, and actuator forces retain their submerged-vehicle
approximations.

## Equations and signs

Let $\mathbf S(\mathbf a)\mathbf b=\mathbf a\times\mathbf b$ and
$\mathbf I=\mathrm{diag}(I_x,I_y,I_z)$. The rigid-body and added-mass
terms are:

```math
\begin{aligned}
\mathbf M_{\mathrm{RB}} &= \mathrm{diag}(m,m,m,I_x,I_y,I_z), \\
\mathbf C_{\mathrm{RB}} &=
\begin{bmatrix}
m\mathbf S(\boldsymbol\omega) & \mathbf 0 \\
\mathbf 0 & -\mathbf S(\mathbf I\boldsymbol\omega)
\end{bmatrix}, \\
\mathbf M_{\mathrm A} &= \mathrm{diag}(\mathbf m_{\mathrm A}), \\
\mathbf a &= \mathbf M_{\mathrm A}\boldsymbol\nu, \\
\mathbf C_{\mathrm A} &=
\begin{bmatrix}
\mathbf 0 & -\mathbf S(\mathbf a_{1:3}) \\
-\mathbf S(\mathbf a_{1:3}) & -\mathbf S(\mathbf a_{4:6})
\end{bmatrix}.
\end{aligned}
```

Here $\mathbf m_{\mathrm A}$ is the six-element `added_mass` parameter.
With elementwise multiplication $\odot$, linear and quadratic damping give:

```math
\mathbf d(\boldsymbol\nu)
= \left(\mathbf d_{\mathrm{lin}}+
\mathbf d_{\mathrm{quad}}\odot|\boldsymbol\nu|\right)
\odot\boldsymbol\nu.
```

The resulting acceleration is:

```math
\begin{aligned}
\mathbf M &= \mathbf M_{\mathrm{RB}}+\mathbf M_{\mathrm A}, \\
\mathbf C &= \mathbf C_{\mathrm{RB}}+\mathbf C_{\mathrm A}, \\
\mathbf M\dot{\boldsymbol\nu}+\mathbf C\boldsymbol\nu
+\mathbf d+\mathbf g &= \boldsymbol\tau, \\
\dot{\boldsymbol\nu} &= \mathbf M^{-1}
\left(\boldsymbol\tau-\mathbf C\boldsymbol\nu-\mathbf d-\mathbf g\right).
\end{aligned}
```

This is Fossen's angular-rate form of rigid-body Coriolis coupling at CG.
It produces $m\boldsymbol\omega\times\mathbf v_{\mathrm{BODY}}$ and
$\boldsymbol\omega\times(\mathbf I\boldsymbol\omega)$. The mass matrix is
symmetric positive definite;
the Coriolis matrix is skew symmetric and contributes zero instantaneous
power, $\boldsymbol\nu^{\mathsf T}\mathbf C_{\mathrm{RB}}\boldsymbol\nu=0$.
These properties also hold for the total mass and Coriolis matrices.
$\boldsymbol\nu^{\mathsf T}\mathbf d\geq0$ ensures damping removes energy,
including when velocities reverse. Water is stationary: there is no ocean
current or relative-current acceleration term in this implementation.

Weight acts downward in NED, buoyancy upward. The nominal full-displacement
volume is $V=m/\rho$. Hull length $L$ sets the longitudinal semiaxis
$a=L/2$; the transverse semiaxes are $b$, with $V=4\pi ab^2/3$.
Let $\mathbf e_D=(e_x,e_y,e_z)$ be the NED-down unit vector expressed in BODY
axes, $\mathbf r_{\mathrm{CB}}$ the nominal CB position relative to CG, and
$z_{\mathrm{CG}}$ the CG depth below the waterline. The normalized immersion
$s$, submerged fraction, and buoyancy are:

```math
\begin{aligned}
b &= \sqrt{\frac{3V}{4\pi a}}, \\
s &= \frac{z_{\mathrm{CG}}+\mathbf e_D\cdot\mathbf r_{\mathrm{CB}}}
{\sqrt{a^2e_x^2+b^2e_y^2+b^2e_z^2}}, \\
f_{\mathrm{submerged}} &=
\begin{cases}
0, & s\leq-1, \\
\dfrac{(2-s)(1+s)^2}{4}, & -1<s<1, \\
1, & s\geq1,
\end{cases} \\
B &= mgf_{\mathrm{submerged}}.
\end{aligned}
```

For a partly submerged hull, the ellipsoidal-cap calculation also moves the
buoyancy centre to the submerged volume's centroid. Rotate weight and
buoyancy into BODY coordinates and calculate the buoyancy moment with
$\mathbf r_{\mathrm{CB,sub}}\times\mathbf B_{\mathrm{BODY}}$.
Weight has no moment about CG.
The simulator does not stop or clamp motion at the waterline; the fraction
varies continuously from zero to one. `restoring_vector` returns the
**negative** of the physical hydrostatic force/moment vector because $\mathbf g$
belongs on the equation's left-hand side. The same ellipsoid helper is in
`simulator/scratch/fossen.py` and is used by the operational hydrostatics.

When fully submerged, $f_{\mathrm{submerged}}=1$, the buoyancy centre is the
nominal CB, and the translational force cancels at every attitude. The remaining
components are:

```math
\begin{aligned}
g_{\mathrm{roll}} &= Wh\cos\theta\sin\phi, \\
g_{\mathrm{pitch}} &= Wh\sin\theta, \\
g_{\mathrm{yaw}} &= 0,
\end{aligned}
```

where $h$ is `cb_height_m`. The physical moments are $-g_{\mathrm{roll}}$ and
$-g_{\mathrm{pitch}}$, opposing small angular disturbances. With damping a
fully submerged vehicle settles toward level.

## Actuators and integration

Commands are `(propeller_rpm, elevator_deg, rudder_deg)`. Accepted ranges
match the shared protocol: 0–3000 RPM and ±20° for each fin pair.
The service rejects invalid commands rather than silently clipping them.

```math
\begin{aligned}
n &= \frac{\mathrm{RPM}}{60}, & X &= k_Tn^2, \\
\ell_f &= \tfrac12\rho\max(u,0)^2C_f, \\
Z &= \ell_f A_e\delta_e, & Y &= -\ell_f A_r\delta_r, \\
M &= -x_fZ, & N &= x_fY, \\
\boldsymbol\tau &= [X,Y,Z,0,M,N]^{\mathsf T}.
\end{aligned}
```

Here $k_T$ is the propeller thrust coefficient, $C_f$ the fin lift slope,
$A_e$ and $A_r$ the combined elevator and rudder areas, and $x_f$ their
aft position. Fin angles $\delta_e$ and $\delta_r$ are converted from degrees
to radians before computing forces.
Positive elevator produces a downward tail force and a nose-up pitch
moment. Positive rudder produces a portward tail force and starboard yaw.
Each pair is represented by its combined area; there is no extra factor of
two. Fins have no authority at rest or in reverse flow in this simplified
model. Stall, incidence, propeller wash, propeller reaction torque, actuator
lag, saturation dynamics, and independent fin actuation are not modeled.
The small-angle model is most credible at modest deflections.

`rk4_step(rhs, time_s, state, dt)` implements classical RK4 with four derivative
evaluations. `step_vehicle` holds commands fixed during a step, recalculates
speed-dependent fin forces at every RK stage, and normalizes the quaternion
after the completed step. Nonfinite states and invalid timesteps raise errors.

Geography uses a local **flat-Earth** NED frame. North/east displacements
convert linearly to latitude/longitude with scales fixed at the origin.
$R_E=6{,}371{,}000\,\mathrm m$ only sets the metres-to-degrees scale; it does
not introduce Earth curvature into the simulation:

```math
\begin{aligned}
\mathrm{lat} &= \mathrm{lat}_0+\frac{180d_N}{\pi R_E}, \\
\mathrm{lon} &= \mathrm{lon}_0+
\frac{180d_E}{\pi R_E\cos(\mathrm{lat}_{0,\mathrm{rad}})}.
\end{aligned}
```

Here $d_N$ and $d_E$ are north and east displacements. The cosine takes the
origin latitude in radians. Longitude wraps to
`[-180, 180)`. Polar origins and out-of-range latitude results are rejected.
The NED axes and level water surface stay fixed. Depth remains the NED down
coordinate. This flat-Earth approximation is intended for local, short paths.

## Use and verify

From the repository root, activate the existing environment with
`source .venv/bin/activate`. For a fresh environment:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

SciPy installs the NumPy dependency; no additional package is required for
these components. This example runs from the repository root:

```python
import numpy as np

from simulator.vehicle.dynamics import state_derivative
from simulator.vehicle.kinematics import quaternion_from_euler
from simulator.vehicle.parameters import VehicleParameters

parameters = VehicleParameters()
state = np.zeros(13)
state[2] = 10.0  # Start at 10 m depth.
state[3:7] = quaternion_from_euler(0.0, 0.0, 0.0)
tau = np.array([84.0, 0.0, 0.0, 0.0, 0.0, 0.0])

derivative = state_derivative(state, tau, parameters)
print(derivative[7:])  # [0.94382... 0. 0. 0. 0. 0.]: 84 N / (84 + 5) kg.
```

Run all tests, including existing IPC tests:

```bash
.venv/bin/python -m pytest -q
```

Use `python -m pytest` from the repository root so Python includes the local
packages on its import path. The tests cover matrix symmetry/positive
definiteness, Coriolis cross-products/skew symmetry/zero power, damping and
decay, neutral and partial buoyancy, submerged-centroid moments, surface
crossing, restoring signs, RK4 convergence, quaternion norm,
fin signs and speed scaling, geographic conversion, scenario transients,
and 50 Hz versus 100 Hz timestep convergence. Service tests exercise command
expiry, malformed/truncated packets, slow readers, reconnects, CSV output,
signal shutdown, and socket ownership. IPC tests require local Unix sockets.

## Run the service

Run from the repository root on a platform supporting Unix `SOCK_SEQPACKET`
(the project is developed/tested on Linux):

```bash
# Accelerated service with live commands over IPC; starts without a controller.
.venv/bin/python -m simulator

# Thirty simulated seconds, accelerated, with every state saved as CSV.
.venv/bin/python -m simulator --scenario surge_step --output surge.csv
.venv/bin/python -m simulator --scenario elevator_step --output elevator.csv
.venv/bin/python -m simulator --scenario rudder_step --output rudder.csv
.venv/bin/python -m simulator --scenario surge_step --initial-depth 0 --output surface-surge.csv

# Configurable real-time rate, duration, origin and initial depth.
.venv/bin/python -m simulator --real-time --frequency 50 --duration 10 --initial-depth 20 \
    --latitude 13 --longitude 80 --socket /tmp/sub-example.sock
```

Execution defaults to accelerated: no wall-clock pacing. The default
simulation frequency is 100 Hz, $\Delta t=0.01\,\mathrm s$. Each completed physics step
is written to CSV when output is requested, and publication is attempted for
a connected client. There is no separate publication-rate setting. In
accelerated mode, wall-clock publication rate depends on how fast the
simulation advances.
Physics time at step $k$ is $t_k=k\Delta t$. Use `--real-time` for wall-clock pacing
with absolute monotonic deadlines.
For a separate accelerated controller, `--sync-controller --control-period 0.05`
requires an integer number of physics steps in each 0.05 s control interval.
The controller UI offers 60, 80, and 100 Hz, giving three, four, or five
steps per control update, respectively. The simulator pauses at each control
boundary until a matching actuator command arrives over the shared Unix IPC
socket. It publishes the initial state at time zero, attempts every
intermediate state, and retries boundary states until delivered or timed out.
Only external-controller mode supports this option; ordinary accelerated
and real-time runs keep their existing behavior. See `controller/README.md`
for the controller service and UI.
An overrun increments a counter and subsequent steps catch up without
skipping physics steps. `--fast` explicitly selects the default accelerated
mode; it cannot be combined with `--real-time`. In Python, pass `fast=False`
to `run_simulator` for real-time pacing. An
explicit duration is rounded up to a whole step; live mode runs until stopped
if no duration is supplied. Scenario mode defaults to 30 simulated seconds.
SIGINT/Ctrl+C and SIGTERM stop cleanly, close the output, and remove the owned
socket. Existing socket paths and existing CSV files are refused. After an
unclean termination, check that no process is using a leftover socket before
removing it manually; the service never unlinks somebody else's endpoint.

## Vehicle Simulator UI and HTTP API

Start these in **separate terminals** from the repository root, after installing
`requirements.txt`:

```bash
.venv/bin/python -m simulator.http_api
.venv/bin/streamlit run ui/simulator_app.py
```

Open the Streamlit address printed by the second command. The API listens on
`http://127.0.0.1:8765` by default; `--port` changes it, and the UI can use
`VEHICLE_SIMULATOR_API_URL=http://127.0.0.1:<port>` to match. The API also
accepts `--socket` to choose the controller IPC path. Do not run the standalone
`python -m simulator` service with the same socket path at the same time.

The UI is named **Vehicle Simulator**. It selects one of the built-in step
scenarios or waits for an external controller, sets the time horizon, rate,
pacing, initial depth and geographic origin, and offers every validated
`VehicleParameters` field for override. The default is a 30-second accelerated
surge step at 100 Hz. The API accepts one active run, returning HTTP 409 for a
second start. The simulator keeps its own clock and controller connection;
Streamlit checks only run status about once per second. After completion it
downloads the CSV and plots depth, attitude, body velocities, and the NED
track. It never renders a chart once per physics sample.

The local JSON API is:

| Method and path | Purpose |
| --- | --- |
| `GET /api/config` | Scenario choices, run defaults, and vehicle parameter defaults |
| `POST /api/runs` | Start a run; returns HTTP 202 and a `run_id` |
| `GET /api/runs/{run_id}` | Get `running`, `completed`, or `failed` status, statistics, and completion reason |
| `GET /api/runs/{run_id}/csv` | Download the completed state CSV |
| `POST /api/runs/{run_id}/complete` | Signal that an external controller reached its goal |

An example start request is:

```json
{
  "scenario": "surge_step",
  "duration_s": 30,
  "frequency_hz": 100,
  "fast": true,
  "initial_depth": 50,
  "latitude": 13,
  "longitude": 80,
  "vehicle_parameters": {"mass_kg": 90, "added_mass": [5, 80, 80, 0.1, 10, 10]}
}
```

Omitted settings use the defaults from `/api/config`; omitted vehicle fields
use `VehicleParameters` defaults. Set `scenario` to `null` for controller IPC
commands instead of a built-in schedule. Every HTTP run has a finite positive
time horizon. A future controller can send `POST .../complete` to stop early;
the status then reports `completion_reason: "goal"`. Otherwise it reports
`"horizon"`. Physics failures appear as `failed` with an error message.
The API keeps only the latest run and its CSV until the next run or API exit;
download a result before starting another run. There is no controller UI or
goal-detection logic in this version.

CSV contains one row per completed step, with sequence, monotonic timestamp,
simulation time, NED position, geographic coordinates, Euler attitude, body
velocities/rates and depth. The optional buffered CSV write is synchronous;
slow storage can affect timing. At exit the service prints a JSON summary:
steps, simulated/wall duration, achieved Hz, overruns, maximum lateness,
published states, congestion drops, and invalid commands. Timing is best
effort, not a hard real-time guarantee.

A local Linux/Python 3.12 measurement on 2026-09-21 ran the surge scenario
in real-time mode at 100 Hz for 10 simulated seconds with a connected reader and CSV output:
1,000 steps took 10.0002 wall seconds (99.998 Hz), with zero loop overruns
and zero congestion drops. The reader received 999 states after connecting
just after startup, with no sequence gaps; CSV contained all 1,000 rows.
Mean publication interval was 10.003 ms and the maximum was 12.967 ms.
These are measurements on this machine, not latency guarantees.

| Scenario | Input | Expected 30 s response with defaults |
| --- | --- | --- |
| `surge_step` | 1800 RPM (90 N) at 2 s | Speed tends to 2.902 m/s; depth stays 50 m |
| `elevator_step` | Same thrust, +1° elevator at 10 s | Nose pitches up; depth decreases |
| `rudder_step` | Same thrust, +3° rudder at 10 s | Positive yaw and eastward displacement |

The table uses the default 50 m initial depth. In the zero-depth example
above, buoyancy is initially less than weight, so the vehicle descends from
the waterline before the 2 s propeller step. Compare `depth_m` and `u_mps`
in that run's CSV to see the surface departure and surge response.

Scenario schedules use simulation time and are the exclusive actuator
source while selected; inbound actuator packets are drained and ignored.
IPC state publication remains active, so a separate viewer can observe them.

## IPC contract

The service listens at `/tmp/sub-simulator.sock` by default and accepts one
active controller connection. In ordinary mode it continues integration with
zero commands until a controller connects, after disconnection, or after
command expiry. Controllers can reconnect without restarting an ordinary run.
Synchronized accelerated mode instead waits for a matching command at each
control boundary and fails on timeout or disconnection. Commands are
held between updates; malformed or expired packets do not erase the last
still-valid command. Sequence numbers must increase within a connection and
may restart on reconnection. Each tick processes at most 64 incoming packets
to bound receive work. If a reader falls behind, full-buffer state sends are
dropped and counted instead of blocking physics. Readers should drain queued
states and use the newest one; sequence gaps identify missed states.

Both directions use the existing version-1 JSON schema in `common/messages.py`.
`timestamp_ns` and command `valid_until_ns` use the host's **monotonic wall
clock**, including in `--fast` mode; `simulation_time_s` is a separate clock.
The shared command helper defaults to a 200 ms lifetime. Angular output is
**radians**, linear output m/s, angular rates rad/s; command fin angles are
**degrees**. Telemetry includes all six velocities, NED position, depth,
latitude/longitude, and roll/pitch/yaw. Internal quaternions remain internal.

A controller can use the unchanged shared client and message helper:

```python
from common.ipc import UnixSeqPacketClient
from common.messages import make_actuator_command

client = UnixSeqPacketClient("/tmp/sub-simulator.sock")
client.connect()  # Run the service first.
try:
    client.send(make_actuator_command(
        sequence=1, propeller_rpm=1800, elevator_deg=0, rudder_deg=0,
    ))
    # Refresh commands before expiry, increasing sequence each time.
    # In your loop: state = client.receive_latest()
finally:
    client.close()
```

The Streamlit UI uses the HTTP control plane for completed runs. It does not
connect to the single-controller Unix socket or drive physics timing.

## References

- [Fossen's Marine Craft Model](https://www.fossen.biz/html/marineCraftModel.html):
  the CG-origin special case of the rigid-body equations, angular-rate
  Coriolis parametrization, and submerged-vehicle restoring forces.
- [SciPy Euler conventions](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_euler.html),
  [quaternion conventions](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.from_quat.html),
  and [Euler output at gimbal lock](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Rotation.as_euler.html).
