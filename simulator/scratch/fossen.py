"""
fossen.py

A compact 6-DOF torpedo-shaped AUV simulator for experimentation.

Included:
  1) BODY/NED transforms + quaternion attitude
  2) Rigid-body mass/inertia + Newton-Euler Coriolis coupling
  3) Neutral buoyancy + CG/CB restoring moments
  4) Added mass + added-mass Coriolis + diagonal linear/quadratic damping
  5) Propeller, elevator and rudder actuator models
  6) RK4 integration
  7) NED -> latitude/longitude
  8) Thrust, elevator and rudder step scenarios

  Fossen model returns the derivative of the state vector, that an integrator needs to integrate to estimate the next
  state
        current state X
              │
              ▼
           controller (another component)
              │
              ▼
         forces τ
              │
              ▼
         Fossen model
              │
              ▼
          Ẋ = f(X,τ)
              │
              ▼
         ODE integrator
              │
              ▼
        next state X

Install:
    pip install numpy matplotlib

Run:
    python simulator/scratch/fossen.py --scenario all
    python simulator/scratch/fossen.py --scenario elevator
    python simulator/scratch/fossen.py --scenario rudder --no-plot

Frames:
    NED:  +x North, +y East,      +z Down
    BODY: +x Forward, +y Starboard, +z Down

State:
    [north, east, down,
     qw, qx, qy, qz,
     u, v, w, p, q, r]

Defaults match simulator.vehicle.parameters.VehicleParameters and the
simulator's built-in step scenarios. Commands use RPM and fin degrees;
logged attitude is in radians, body rates in rad/s, and latitude/longitude
in degrees. Samples follow completed RK4 steps, starting at 0.01 s.

"""

import argparse
import math
from dataclasses import dataclass

import numpy as np


# ---------------------------------------------------------------------------
# Basic math / coordinate transforms
# ---------------------------------------------------------------------------

def skew(a):
    """S(a) such that S(a) @ b == a x b."""
    x, y, z = a
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def q_normalize(q):
    return q / np.linalg.norm(q)


def q_to_R(q):
    """BODY -> NED rotation matrix."""
    w, x, y, z = q_normalize(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)],
    ])


def q_dot(q, omega):
    """Quaternion derivative for BODY angular velocity omega=[p,q,r]."""
    w, x, y, z = q
    p, qrate, r = omega
    return 0.5 * np.array([
        -x*p - y*qrate - z*r,
         w*p + y*r - z*qrate,
         w*qrate - x*r + z*p,
         w*r + x*qrate - y*p,
    ])

# use quaternions internally, euler angles at the control interface
def q_to_euler(q):
    """Return ZYX roll, pitch, yaw [rad]."""
    w, x, y, z = q_normalize(q)
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    s = np.clip(2*(w*y - z*x), -1.0, 1.0)
    pitch = math.asin(s)
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return np.array([roll, pitch, yaw])


def ned_to_latlon(north, east, lat0_deg, lon0_deg):
    """Flat-Earth conversion using fixed metres-to-degrees scales at the origin."""
    if not all(math.isfinite(v) for v in (north, east, lat0_deg, lon0_deg)):
        raise ValueError("Geographic coordinates must be finite")
    if not -90 < lat0_deg < 90 or not -180 <= lon0_deg <= 180:
        raise ValueError("Origin must have latitude strictly between ±90 and longitude within ±180")
    R_earth = 6_371_000.0
    lat = lat0_deg + math.degrees(north / R_earth)
    lon = lon0_deg + math.degrees(east / R_earth / math.cos(math.radians(lat0_deg)))
    if not -90 <= lat <= 90 or not math.isfinite(lon):
        raise ValueError("Displacement exceeds the local geographic model's domain")
    return lat, (lon + 180) % 360 - 180


# ---------------------------------------------------------------------------
# Vehicle parameters
# ---------------------------------------------------------------------------

@dataclass
class P:
    rho: float = 1025.0
    g: float = 9.8

    # ~2 m torpedo-shaped vehicle
    mass: float = 84.0
    Ixx: float = 0.70
    Iyy: float = 25.0
    Izz: float = 25.0

    # CB is 2 cm ABOVE CG. BODY +z is downward, hence negative.
    cb_z: float = -0.020

    # Added mass: much larger in sway/heave than surge for a slender body.
    added_u: float = 5.0
    added_v: float = 80.0
    added_w: float = 80.0
    added_p: float = 0.1
    added_q: float = 10.0
    added_r: float = 10.0

    # Diagonal linear damping
    linear_u: float = 2.0
    linear_v: float = 200.0
    linear_w: float = 200.0
    linear_p: float = 0.5
    linear_q: float = 400.0
    linear_r: float = 400.0

    # Diagonal quadratic damping
    quad_u: float = 10.0
    quad_v: float = 200.0
    quad_w: float = 200.0
    quad_p: float = 0.1
    quad_q: float = 50.0
    quad_r: float = 50.0

    # Propeller: T = k * n^2, n in rev/s; only nonnegative RPM is accepted.
    prop_k: float = 0.1

    # Combined area of each symmetric fin pair
    elevator_area: float = 0.020
    rudder_area: float = 0.020
    elevator_Cdelta: float = 4.0   # per rad
    rudder_Cdelta: float = 4.0     # per rad

    # Both fin pairs are aft of CG
    elevator_x: float = -0.80
    rudder_x: float = -0.80


# ---------------------------------------------------------------------------
# 6-DOF vehicle
# ---------------------------------------------------------------------------

class AUV:
    def __init__(self, p=P()):
        self.p = p

        # Stage 2: rigid-body matrix, BODY origin at CG.
        self.I = np.diag([p.Ixx, p.Iyy, p.Izz])
        self.M_RB = np.block([
            [p.mass*np.eye(3), np.zeros((3, 3))],
            [np.zeros((3, 3)), self.I],
        ])

        # Stage 4: positive physical added-mass matrix.
        self.M_A = np.diag([
            p.added_u, p.added_v, p.added_w,
            p.added_p, p.added_q, p.added_r,
        ])
        self.M = self.M_RB + self.M_A

        self.dlin = np.array([
            p.linear_u, p.linear_v, p.linear_w,
            p.linear_p, p.linear_q, p.linear_r,
        ])
        self.dquad = np.array([
            p.quad_u, p.quad_v, p.quad_w,
            p.quad_p, p.quad_q, p.quad_r,
        ])

        self.r_cb = np.array([0.0, 0.0, p.cb_z])

        # Stage 3: exactly neutral buoyancy.
        self.W = p.mass * p.g
        self.B = self.W

    def C_RB(self, nu):
        """
        Rigid-body Coriolis/centripetal matrix.

        C_RB @ nu produces the familiar Newton-Euler terms:
            m (omega x v)
            omega x (I omega)
        """
        v, omega = nu[:3], nu[3:]
        mv = self.p.mass * v
        Iomega = self.I @ omega
        return np.block([
            [np.zeros((3, 3)), -skew(mv)],
            [-skew(mv),        -skew(Iomega)],
        ])

    def C_A(self, nu):
        """Added-mass Coriolis matrix for block-diagonal M_A."""
        v, omega = nu[:3], nu[3:]
        pA = self.M_A[:3, :3] @ v
        hA = self.M_A[3:, 3:] @ omega
        return np.block([
            [np.zeros((3, 3)), -skew(pA)],
            [-skew(pA),        -skew(hA)],
        ])

    def D(self, nu):
        """
        Stage 4: diagonal linear + quadratic damping.

            D(nu) = diag(d_linear + d_quad*|nu|)
        """
        return np.diag(self.dlin + self.dquad*np.abs(nu))

    def hydrostatic_wrench(self, q):
        """
        Stage 3: gravity + buoyancy in BODY coordinates.

        W == B, so net hydrostatic force is zero.
        Because CB is above CG, attitude displacement creates a restoring
        roll/pitch moment.
        """
        R_nb = q_to_R(q).T  # NED -> BODY

        weight_b = R_nb @ np.array([0.0, 0.0, self.W])
        buoyancy_b = R_nb @ np.array([0.0, 0.0, -self.B])

        force = weight_b + buoyancy_b
        moment = np.cross(self.r_cb, buoyancy_b)  # weight acts at CG

        return np.r_[force, moment]

    def actuator_wrench(self, nu, rpm, elevator_deg, rudder_deg):
        """
        Stage 5: RPM/fin degrees -> [X,Y,Z,K,M,N].

        Propeller:
            +X thrust.

        Elevator:
            fin force ~ 0.5*rho*u^2*S*C_delta*delta.
            Positive delta creates +Z at the tail -> +pitch moment.
            Therefore NEGATIVE elevator gives nose-down/dive.

        Rudder:
            Positive delta is defined to create +yaw moment.

        At u<=0 the fins have no authority in this simplified model.
        Propeller wash is ignored.
        """
        if not (0 <= rpm <= 3000 and -20 <= elevator_deg <= 20 and -20 <= rudder_deg <= 20):
            raise ValueError("Actuators must be 0–3000 RPM and within ±20 degrees")
        p = self.p
        u = max(nu[0], 0.0)
        elevator_rad = math.radians(elevator_deg)
        rudder_rad = math.radians(rudder_deg)

        # Propeller
        n = rpm / 60.0
        thrust = p.prop_k * n**2
        F_prop = np.array([thrust, 0.0, 0.0])

        # Fin dynamic pressure
        qbar = 0.5 * p.rho * u*u

        # Elevators -> Z and pitch M
        Z_e = qbar * p.elevator_area * p.elevator_Cdelta * elevator_rad
        F_e = np.array([0.0, 0.0, Z_e])
        r_e = np.array([p.elevator_x, 0.0, 0.0])

        # Rudders -> Y and yaw N.
        # Minus sign chosen so positive rudder produces positive yaw moment
        # when the rudder is aft of CG.
        Y_r = -qbar * p.rudder_area * p.rudder_Cdelta * rudder_rad
        F_r = np.array([0.0, Y_r, 0.0])
        r_r = np.array([p.rudder_x, 0.0, 0.0])

        force = F_prop + F_e + F_r
        moment = np.cross(r_e, F_e) + np.cross(r_r, F_r)
        return np.r_[force, moment]

    def derivative(self, state, cmd):
        """
        Complete nonlinear state derivative.

        M nu_dot + (C_RB + C_A)nu + D(nu)nu
            = tau_actuator + tau_hydrostatic
        """
        q = q_normalize(state[3:7])
        nu = state[7:13]

        # Stage 1: BODY velocity -> NED position rate.
        pos_dot = q_to_R(q) @ nu[:3]
        quat_dot = q_dot(q, nu[3:])

        # Stages 2-5: forces/moments -> acceleration.
        C = self.C_RB(nu) + self.C_A(nu)
        tau = (
            self.actuator_wrench(
                nu, cmd["rpm"], cmd["elevator_deg"], cmd["rudder_deg"]
            )
            + self.hydrostatic_wrench(q)
        )

        rhs = tau - C @ nu - self.D(nu) @ nu
        nu_dot = np.linalg.solve(self.M, rhs)

        return np.r_[pos_dot, quat_dot, nu_dot]

    def step_rk4(self, state, cmd, dt):
        """Stage 6: one classical RK4 step."""
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        if state[2] < 0:
            raise ValueError("Vehicle is above the water surface")
        f = self.derivative
        k1 = f(state, cmd)
        k2 = f(state + 0.5*dt*k1, cmd)
        k3 = f(state + 0.5*dt*k2, cmd)
        k4 = f(state + dt*k3, cmd)

        new_state = state + dt*(k1 + 2*k2 + 2*k3 + k4)/6.0
        new_state[3:7] = q_normalize(new_state[3:7])
        if not np.all(np.isfinite(new_state)):
            raise ValueError("Integrated state must be finite")
        if new_state[2] < 0:
            raise ValueError("Vehicle crossed the water surface; submerged model no longer applies")
        return new_state


# ---------------------------------------------------------------------------
# Stage 9: simple open-loop scenarios
# ---------------------------------------------------------------------------

def zero_state():
    """Stationary, level vehicle at 50 m depth, matching the service default."""
    state = np.zeros(13)
    state[2] = 50.0
    state[3] = 1.0  # identity quaternion
    return state


def scenario_command(name, t):
    """
    All scenarios apply 1800 RPM from 2 s, matching simulator.scenarios.

    thrust:
        only the RPM step.

    elevator:
        +1 deg elevator from 10 s onward -> nose-up response.

    rudder:
        +3 deg rudder from 10 s onward -> starboard yaw response.
    """
    rpm = 1800.0 if t >= 2.0 else 0.0
    elevator_deg = 1.0 if name == "elevator" and t >= 10.0 else 0.0
    rudder_deg = 3.0 if name == "rudder" and t >= 10.0 else 0.0

    if name not in ("thrust", "elevator", "rudder"):
        raise ValueError(name)

    return {
        "rpm": rpm,
        "elevator_deg": elevator_deg,
        "rudder_deg": rudder_deg,
    }


def run(name, duration=30.0, dt=0.01, lat0=13.0, lon0=80.0):
    """
    dt=0.01 means a 100 Hz SIMULATION timestep.
    No wall-clock sleeping is used; this play-around script runs as fast
    as the CPU allows.
    Each row logs the completed step and the command held during that step.
    Duration rounds up to a whole step, as in the simulator service.
    """
    if not all(math.isfinite(v) and v > 0 for v in (duration, dt)):
        raise ValueError("duration and dt must be finite and positive")
    if not math.isfinite(duration / dt):
        raise ValueError("duration and dt produce too many steps")
    auv = AUV()
    x = zero_state()
    log = []

    for k in range(max(1, math.ceil(duration / dt - 1e-12))):
        t = k * dt
        cmd = scenario_command(name, t)
        x = auv.step_rk4(x, cmd, dt)

        roll, pitch, yaw = q_to_euler(x[3:7])
        lat, lon = ned_to_latlon(x[0], x[1], lat0, lon0)

        log.append([
            (k + 1) * dt,
            *x[:3],                        # N, E, D
            lat, lon,
            roll, pitch, yaw,              # rad, matching telemetry/CSV
            *x[7:13],                      # u,v,w,p,q,r
            cmd["rpm"],
            cmd["elevator_deg"],
            cmd["rudder_deg"],
        ])

    return np.asarray(log)


# Log units: seconds, metres, geographic degrees, attitude radians,
# body m/s and rad/s, then RPM and fin degrees.
T, N, E, DEPTH = 0, 1, 2, 3
LAT, LON = 4, 5
ROLL, PITCH, YAW = 6, 7, 8
U, V, W, P_RATE, Q_RATE, R_RATE = 9, 10, 11, 12, 13, 14
RPM, ELEVATOR, RUDDER = 15, 16, 17


def print_final(name, a):
    r = a[-1]
    print(f"\n--- {name.upper()} ---")
    print(f"NED      : N={r[N]:.3f}, E={r[E]:.3f}, D={r[DEPTH]:.3f} m")
    print(f"Lat/Lon  : {r[LAT]:.8f}, {r[LON]:.8f}")
    print(
        f"Attitude : roll={r[ROLL]:.5f}, pitch={r[PITCH]:.5f}, "
        f"yaw={r[YAW]:.5f} rad"
    )
    print(f"Velocity : u={r[U]:.3f}, v={r[V]:.3f}, w={r[W]:.3f} m/s")


def plot(name, a):
    """Small visualization helper; not part of the vehicle physics."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plots")
        return

    fig, ax = plt.subplots(4, 1, figsize=(9, 10), constrained_layout=True)

    ax[0].plot(a[:, T], a[:, DEPTH])
    ax[0].set_ylabel("Depth [m]")
    ax[0].grid()

    ax[1].plot(a[:, T], a[:, ROLL], label="roll")
    ax[1].plot(a[:, T], a[:, PITCH], label="pitch")
    ax[1].plot(a[:, T], a[:, YAW], label="yaw")
    ax[1].set_ylabel("Angle [rad]")
    ax[1].legend()
    ax[1].grid()

    ax[2].plot(a[:, T], a[:, U], label="u")
    ax[2].plot(a[:, T], a[:, V], label="v")
    ax[2].plot(a[:, T], a[:, W], label="w")
    ax[2].set_ylabel("Velocity [m/s]")
    ax[2].legend()
    ax[2].grid()

    ax[3].plot(a[:, T], a[:, RPM], label="RPM")
    ax[3].plot(a[:, T], a[:, ELEVATOR], label="elevator deg")
    ax[3].plot(a[:, T], a[:, RUDDER], label="rudder deg")
    ax[3].set_xlabel("Simulation time [s]")
    ax[3].set_ylabel("Actuators")
    ax[3].legend()
    ax[3].grid()

    fig.suptitle(f"6-DOF AUV - {name}")
    plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--scenario",
        choices=["thrust", "elevator", "rudder", "all"],
        default="all",
    )
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--dt", type=float, default=0.01)
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    if args.dt <= 0 or args.duration <= 0:
        raise ValueError("duration and dt must be positive")

    names = (
        ["thrust", "elevator", "rudder"]
        if args.scenario == "all"
        else [args.scenario]
    )

    for name in names:
        data = run(name, args.duration, args.dt)
        print_final(name, data)
        if not args.no_plot:
            plot(name, data)


if __name__ == "__main__":
    main()
