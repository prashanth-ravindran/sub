"""Built-in actuator sequences driven by simulation time."""

from .elevator_step import elevator_step
from .rudder_step import rudder_step
from .surge_step import surge_step

SCENARIOS = {
    "surge_step": surge_step,
    "elevator_step": elevator_step,
    "rudder_step": rudder_step,
}
