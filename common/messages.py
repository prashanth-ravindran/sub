"""
common/messages.py

Message definitions, creation helpers, and validation for communication
between the AUV simulator and controller.

The IPC transport itself lives in common/ipc.py.

Protocol version: 1

Message types:
    vehicle_state
        Simulator -> Controller

    actuator_command
        Controller -> Simulator

    simulation_stop
        Controller -> synchronized Simulator, at a control boundary

In synchronized mode, actuator_command includes state_sequence identifying
the published boundary state to which the command responds.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict, Optional


# ---------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------

PROTOCOL_VERSION = 1

MESSAGE_TYPE_VEHICLE_STATE = "vehicle_state"
MESSAGE_TYPE_ACTUATOR_COMMAND = "actuator_command"
MESSAGE_TYPE_SIMULATION_STOP = "simulation_stop"


# ---------------------------------------------------------------------
# Actuator limits
#
# These are simulation assumptions rather than universal physical limits.
# Keep them consistent with vehicle.yaml if you later move these values
# into configuration.
# ---------------------------------------------------------------------

MIN_PROPELLER_RPM = 0.0
MAX_PROPELLER_RPM = 3000.0

MIN_ELEVATOR_DEG = -20.0
MAX_ELEVATOR_DEG = 20.0

MIN_RUDDER_DEG = -20.0
MAX_RUDDER_DEG = 20.0


# Default lifetime of an actuator command.
#
# At a 100 Hz controller rate, 200 ms corresponds to 20 missed cycles.
# This value is configurable and is simply a reasonable assignment
# assumption.
DEFAULT_COMMAND_VALIDITY_NS = 200_000_000


# ---------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------

class MessageValidationError(ValueError):
    """
    Raised when an IPC message does not conform to the protocol.
    """

    pass


# ---------------------------------------------------------------------
# Generic helpers
# ---------------------------------------------------------------------

def monotonic_time_ns() -> int:
    """
    Return monotonic time in nanoseconds.

    Monotonic time should be used for control/IPC timing because it cannot
    jump backwards due to system clock adjustments.
    """

    return time.monotonic_ns()


def _is_finite_number(value: Any) -> bool:
    """
    True only for finite int/float values.

    bool is deliberately rejected because bool is a subclass of int
    in Python.
    """

    if isinstance(value, bool):
        return False

    if not isinstance(value, (int, float)):
        return False

    return math.isfinite(float(value))


def _is_integer(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _require_dict(
    parent: Dict[str, Any],
    field: str,
) -> Dict[str, Any]:

    value = parent.get(field)

    if not isinstance(value, dict):
        raise MessageValidationError(
            f"'{field}' must be an object"
        )

    return value


def _require_finite_number(
    parent: Dict[str, Any],
    field: str,
) -> float:

    if field not in parent:
        raise MessageValidationError(
            f"Missing required field '{field}'"
        )

    value = parent[field]

    if not _is_finite_number(value):
        raise MessageValidationError(
            f"'{field}' must be a finite number"
        )

    return float(value)


def _require_integer(
    parent: Dict[str, Any],
    field: str,
) -> int:

    if field not in parent:
        raise MessageValidationError(
            f"Missing required field '{field}'"
        )

    value = parent[field]

    if not _is_integer(value):
        raise MessageValidationError(
            f"'{field}' must be an integer"
        )

    return value


def _require_range(
    value: float,
    minimum: float,
    maximum: float,
    name: str,
) -> None:

    if not minimum <= value <= maximum:
        raise MessageValidationError(
            f"'{name}' out of range: "
            f"{value} not in [{minimum}, {maximum}]"
        )


# =====================================================================
# VEHICLE STATE MESSAGE
# =====================================================================

def make_vehicle_state(
    *,
    sequence: int,
    simulation_time_s: float,

    north_m: float,
    east_m: float,
    down_m: float,

    latitude_deg: float,
    longitude_deg: float,

    roll_rad: float,
    pitch_rad: float,
    yaw_rad: float,

    u_mps: float,
    v_mps: float,
    w_mps: float,

    p_radps: float,
    q_radps: float,
    r_radps: float,

    depth_m: float,

    timestamp_ns: Optional[int] = None,
) -> Dict[str, Any]:
    """
    Construct a vehicle-state message.

    This message travels:

        Simulator -> Controller
    """

    if timestamp_ns is None:
        timestamp_ns = monotonic_time_ns()

    message = {
        "type": MESSAGE_TYPE_VEHICLE_STATE,
        "version": PROTOCOL_VERSION,

        "sequence": sequence,
        "timestamp_ns": timestamp_ns,

        "simulation_time_s": simulation_time_s,

        "position": {
            "north_m": north_m,
            "east_m": east_m,
            "down_m": down_m,
        },

        "geodetic": {
            "latitude_deg": latitude_deg,
            "longitude_deg": longitude_deg,
        },

        "orientation": {
            "roll_rad": roll_rad,
            "pitch_rad": pitch_rad,
            "yaw_rad": yaw_rad,
        },

        "velocity_body": {
            "u_mps": u_mps,
            "v_mps": v_mps,
            "w_mps": w_mps,
        },

        "angular_velocity_body": {
            "p_radps": p_radps,
            "q_radps": q_radps,
            "r_radps": r_radps,
        },

        "depth_m": depth_m,
    }

    # Validate our own generated message.
    validate_vehicle_state_or_raise(message)

    return message


def validate_vehicle_state(
    message: Any,
) -> bool:
    """
    Return True if message is a valid VehicleState message.

    Does not raise.
    """

    try:
        validate_vehicle_state_or_raise(message)
        return True

    except MessageValidationError:
        return False


def validate_vehicle_state_or_raise(
    message: Any,
) -> None:
    """
    Fully validate a VehicleState message.

    Raises MessageValidationError if invalid.
    """

    if not isinstance(message, dict):
        raise MessageValidationError(
            "Vehicle state message must be an object"
        )

    if message.get("type") != MESSAGE_TYPE_VEHICLE_STATE:
        raise MessageValidationError(
            "Incorrect message type for VehicleState"
        )

    if message.get("version") != PROTOCOL_VERSION:
        raise MessageValidationError(
            f"Unsupported protocol version: "
            f"{message.get('version')}"
        )

    sequence = _require_integer(
        message,
        "sequence",
    )

    if sequence < 0:
        raise MessageValidationError(
            "'sequence' cannot be negative"
        )

    timestamp = _require_integer(
        message,
        "timestamp_ns",
    )

    if timestamp < 0:
        raise MessageValidationError(
            "'timestamp_ns' cannot be negative"
        )

    simulation_time = _require_finite_number(
        message,
        "simulation_time_s",
    )

    if simulation_time < 0:
        raise MessageValidationError(
            "'simulation_time_s' cannot be negative"
        )

    # ------------------------------------------------------------
    # Position
    # ------------------------------------------------------------

    position = _require_dict(
        message,
        "position",
    )

    _require_finite_number(
        position,
        "north_m",
    )

    _require_finite_number(
        position,
        "east_m",
    )

    _require_finite_number(
        position,
        "down_m",
    )

    # ------------------------------------------------------------
    # Latitude / longitude
    # ------------------------------------------------------------

    geodetic = _require_dict(
        message,
        "geodetic",
    )

    latitude = _require_finite_number(
        geodetic,
        "latitude_deg",
    )

    longitude = _require_finite_number(
        geodetic,
        "longitude_deg",
    )

    _require_range(
        latitude,
        -90.0,
        90.0,
        "latitude_deg",
    )

    _require_range(
        longitude,
        -180.0,
        180.0,
        "longitude_deg",
    )

    # ------------------------------------------------------------
    # Orientation
    # ------------------------------------------------------------

    orientation = _require_dict(
        message,
        "orientation",
    )

    _require_finite_number(
        orientation,
        "roll_rad",
    )

    _require_finite_number(
        orientation,
        "pitch_rad",
    )

    _require_finite_number(
        orientation,
        "yaw_rad",
    )

    # ------------------------------------------------------------
    # Linear body velocity
    # ------------------------------------------------------------

    velocity = _require_dict(
        message,
        "velocity_body",
    )

    _require_finite_number(
        velocity,
        "u_mps",
    )

    _require_finite_number(
        velocity,
        "v_mps",
    )

    _require_finite_number(
        velocity,
        "w_mps",
    )

    # ------------------------------------------------------------
    # Angular body velocity
    # ------------------------------------------------------------

    angular_velocity = _require_dict(
        message,
        "angular_velocity_body",
    )

    _require_finite_number(
        angular_velocity,
        "p_radps",
    )

    _require_finite_number(
        angular_velocity,
        "q_radps",
    )

    _require_finite_number(
        angular_velocity,
        "r_radps",
    )

    # ------------------------------------------------------------
    # Depth
    # ------------------------------------------------------------

    depth = _require_finite_number(
        message,
        "depth_m",
    )

    # Negative depth should normally not occur in this simulation.
    if depth < 0:
        raise MessageValidationError(
            "'depth_m' cannot be negative"
        )


# =====================================================================
# ACTUATOR COMMAND MESSAGE
# =====================================================================

def make_actuator_command(
    *,
    sequence: int,
    propeller_rpm: float,
    elevator_deg: float,
    rudder_deg: float,
    timestamp_ns: Optional[int] = None,
    validity_ns: int = DEFAULT_COMMAND_VALIDITY_NS,
    state_sequence: Optional[int] = None,
) -> Dict[str, Any]:
    """
    Construct an actuator-command message.

    This message travels:

        Controller -> Simulator
    """

    if timestamp_ns is None:
        timestamp_ns = monotonic_time_ns()

    if validity_ns <= 0:
        raise ValueError(
            "validity_ns must be greater than zero"
        )

    message = {
        "type": MESSAGE_TYPE_ACTUATOR_COMMAND,
        "version": PROTOCOL_VERSION,

        "sequence": sequence,
        "timestamp_ns": timestamp_ns,

        "valid_until_ns": (
            timestamp_ns + validity_ns
        ),

        "actuators": {
            "propeller_rpm": propeller_rpm,
            "elevator_deg": elevator_deg,
            "rudder_deg": rudder_deg,
        },
    }

    if state_sequence is not None:
        message["state_sequence"] = state_sequence

    validate_actuator_command_or_raise(
        message,
        check_expiry=False,
    )

    return message


def validate_actuator_command(
    message: Any,
    *,
    check_expiry: bool = False,
    now_ns: Optional[int] = None,
) -> bool:
    """
    Return True if message is a valid actuator command.

    If check_expiry=True, an expired command is considered invalid.
    """

    try:
        validate_actuator_command_or_raise(
            message,
            check_expiry=check_expiry,
            now_ns=now_ns,
        )

        return True

    except MessageValidationError:
        return False


def validate_actuator_command_or_raise(
    message: Any,
    *,
    check_expiry: bool = False,
    now_ns: Optional[int] = None,
) -> None:
    """
    Fully validate an actuator command.

    Raises MessageValidationError if invalid.
    """

    if not isinstance(message, dict):
        raise MessageValidationError(
            "Actuator command must be an object"
        )

    if message.get("type") != MESSAGE_TYPE_ACTUATOR_COMMAND:
        raise MessageValidationError(
            "Incorrect message type for ActuatorCommand"
        )

    if message.get("version") != PROTOCOL_VERSION:
        raise MessageValidationError(
            f"Unsupported protocol version: "
            f"{message.get('version')}"
        )

    sequence = _require_integer(
        message,
        "sequence",
    )

    if sequence < 0:
        raise MessageValidationError(
            "'sequence' cannot be negative"
        )

    if "state_sequence" in message and _require_integer(message, "state_sequence") < 0:
        raise MessageValidationError("'state_sequence' cannot be negative")

    timestamp = _require_integer(
        message,
        "timestamp_ns",
    )

    valid_until = _require_integer(
        message,
        "valid_until_ns",
    )

    if timestamp < 0:
        raise MessageValidationError(
            "'timestamp_ns' cannot be negative"
        )

    if valid_until <= timestamp:
        raise MessageValidationError(
            "'valid_until_ns' must be later than timestamp_ns"
        )

    # ------------------------------------------------------------
    # Actuator values
    # ------------------------------------------------------------

    actuators = _require_dict(
        message,
        "actuators",
    )

    rpm = _require_finite_number(
        actuators,
        "propeller_rpm",
    )

    elevator = _require_finite_number(
        actuators,
        "elevator_deg",
    )

    rudder = _require_finite_number(
        actuators,
        "rudder_deg",
    )

    _require_range(
        rpm,
        MIN_PROPELLER_RPM,
        MAX_PROPELLER_RPM,
        "propeller_rpm",
    )

    _require_range(
        elevator,
        MIN_ELEVATOR_DEG,
        MAX_ELEVATOR_DEG,
        "elevator_deg",
    )

    _require_range(
        rudder,
        MIN_RUDDER_DEG,
        MAX_RUDDER_DEG,
        "rudder_deg",
    )

    # ------------------------------------------------------------
    # Optional expiry check
    # ------------------------------------------------------------

    if check_expiry:

        if now_ns is None:
            now_ns = monotonic_time_ns()

        if now_ns > valid_until:
            raise MessageValidationError(
                "Actuator command has expired"
            )


def make_simulation_stop(*, state_sequence: int) -> Dict[str, Any]:
    """Ask a synchronized simulator to finish at a control boundary."""
    message = {
        "type": MESSAGE_TYPE_SIMULATION_STOP,
        "version": PROTOCOL_VERSION,
        "state_sequence": state_sequence,
    }
    validate_simulation_stop_or_raise(message)
    return message


def validate_simulation_stop_or_raise(message: Any) -> None:
    if not isinstance(message, dict) or message.get("type") != MESSAGE_TYPE_SIMULATION_STOP:
        raise MessageValidationError("Incorrect simulation stop message")
    if message.get("version") != PROTOCOL_VERSION:
        raise MessageValidationError("Unsupported protocol version")
    if _require_integer(message, "state_sequence") < 0:
        raise MessageValidationError("'state_sequence' cannot be negative")


# =====================================================================
# COMMAND EXPIRY
# =====================================================================

def command_is_valid(
    command: Any,
    now_ns: Optional[int] = None,
) -> bool:
    """
    Check whether a command is structurally valid AND has not expired.

    Useful inside the simulator before applying an actuator command.
    """

    return validate_actuator_command(
        command,
        check_expiry=True,
        now_ns=now_ns,
    )


def command_is_expired(
    command: Dict[str, Any],
    now_ns: Optional[int] = None,
) -> bool:
    """
    Check only command expiration.

    Does not perform complete message validation.
    """

    if now_ns is None:
        now_ns = monotonic_time_ns()

    valid_until = command.get(
        "valid_until_ns"
    )

    if not _is_integer(valid_until):
        return True

    return now_ns > valid_until


# =====================================================================
# SAFE COMMAND
# =====================================================================

def make_safe_actuator_command(
    sequence: int,
) -> Dict[str, Any]:
    """
    Generate the simulator's fallback command.

    For this assignment we define the fallback as:

        propeller RPM = 0
        elevator      = 0 deg
        rudder        = 0 deg

    In a real AUV, the safe-state behavior would be determined through
    system safety analysis.
    """

    return make_actuator_command(
        sequence=sequence,
        propeller_rpm=0.0,
        elevator_deg=0.0,
        rudder_deg=0.0,
    )


# =====================================================================
# CONVENIENCE EXTRACTION
# =====================================================================

def get_actuator_values(
    command: Dict[str, Any],
) -> tuple[float, float, float]:
    """
    Validate an actuator command and return:

        propeller_rpm,
        elevator_deg,
        rudder_deg
    """

    validate_actuator_command_or_raise(
        command,
        check_expiry=False,
    )

    actuators = command["actuators"]

    return (
        float(actuators["propeller_rpm"]),
        float(actuators["elevator_deg"]),
        float(actuators["rudder_deg"]),
    )


def get_vehicle_control_state(
    state: Dict[str, Any],
) -> Dict[str, float]:
    """
    Extract the quantities most commonly required by the controller.

    This keeps controller.py from having to know the complete JSON
    hierarchy.
    """

    validate_vehicle_state_or_raise(state)

    return {
        "north_m": float(
            state["position"]["north_m"]
        ),

        "east_m": float(
            state["position"]["east_m"]
        ),

        "depth_m": float(
            state["depth_m"]
        ),

        "latitude_deg": float(
            state["geodetic"]["latitude_deg"]
        ),

        "longitude_deg": float(
            state["geodetic"]["longitude_deg"]
        ),

        "roll_rad": float(
            state["orientation"]["roll_rad"]
        ),

        "pitch_rad": float(
            state["orientation"]["pitch_rad"]
        ),

        "yaw_rad": float(
            state["orientation"]["yaw_rad"]
        ),

        "u_mps": float(
            state["velocity_body"]["u_mps"]
        ),

        "v_mps": float(
            state["velocity_body"]["v_mps"]
        ),

        "w_mps": float(
            state["velocity_body"]["w_mps"]
        ),

        "p_radps": float(
            state["angular_velocity_body"]["p_radps"]
        ),

        "q_radps": float(
            state["angular_velocity_body"]["q_radps"]
        ),

        "r_radps": float(
            state["angular_velocity_body"]["r_radps"]
        ),
    }
