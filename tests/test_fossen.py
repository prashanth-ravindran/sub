"""Keep the standalone reference aligned with the service's physical defaults."""

import csv

import numpy as np
import pytest

from simulator.scratch import fossen
from simulator.navigation.geodetic import ned_to_geodetic
from simulator.scenarios import SCENARIOS
from simulator.simulation.simulator import run_simulator
from simulator.vehicle.actuators import actuator_forces
from simulator.vehicle.parameters import VehicleParameters
from simulator.simulation.integrator import step_vehicle
from simulator.vehicle.kinematics import quaternion_from_euler


SCENARIO_NAMES = {
    "thrust": "surge_step",
    "elevator": "elevator_step",
    "rudder": "rudder_step",
}
STATE_COLUMNS = (
    "simulation_time_s", "north_m", "east_m", "down_m",
    "latitude_deg", "longitude_deg", "roll_rad", "pitch_rad", "yaw_rad",
    "u_mps", "v_mps", "w_mps", "p_radps", "q_radps", "r_radps",
)


def test_all_reference_coefficients_match_vehicle_parameters():
    reference, parameters = fossen.P(), VehicleParameters()
    assert reference.hull_length_m == parameters.hull_length_m
    assert (
        reference.rho, reference.g, reference.mass,
        reference.Ixx, reference.Iyy, reference.Izz, reference.cb_z,
        reference.prop_k, reference.elevator_area, reference.rudder_area,
        reference.elevator_Cdelta, reference.rudder_Cdelta,
        reference.elevator_x, reference.rudder_x,
    ) == (
        parameters.water_density_kg_m3, parameters.gravity_mps2, parameters.mass_kg,
        *parameters.inertia_kg_m2, -parameters.cb_height_m,
        parameters.propeller_thrust_coefficient, parameters.elevator_area_m2,
        parameters.rudder_area_m2, parameters.fin_lift_slope_per_rad,
        parameters.fin_lift_slope_per_rad, parameters.fin_x_m, parameters.fin_x_m,
    )
    for prefix, expected in (
        ("added", parameters.added_mass),
        ("linear", parameters.linear_damping),
        ("quad", parameters.quadratic_damping),
    ):
        assert tuple(getattr(reference, f"{prefix}_{axis}") for axis in "uvwpqr") == expected


def test_schedules_match_in_degrees_at_step_boundaries():
    for name, canonical in SCENARIO_NAMES.items():
        for time_s in (0, 1.99, 2, 9.99, 10, 30):
            command = fossen.scenario_command(name, time_s)
            assert (command["rpm"], command["elevator_deg"], command["rudder_deg"]) == SCENARIOS[canonical](time_s)


@pytest.mark.parametrize("name, canonical", SCENARIO_NAMES.items())
def test_entire_default_scenario_matches_service_csv(name, canonical, tmp_path):
    reference = fossen.run(name)
    output = tmp_path / "states.csv"
    stats = run_simulator(
        scenario=canonical, fast=True, socket_path=str(tmp_path / "s"), output=output,
    )
    with output.open() as file:
        actual = np.array([[float(row[key]) for key in STATE_COLUMNS] for row in csv.DictReader(file)])
    assert reference.shape == (3000, 18)
    assert stats["steps"] == len(reference)
    np.testing.assert_array_equal(reference[:, fossen.T], actual[:, 0])
    np.testing.assert_allclose(reference[:, :15], actual, rtol=1e-10, atol=1e-10)
    # Inputs are held from the preceding step's start: the first response to
    # the t=2 RPM step is logged at t=2.01, and the fin step at t=10.01.
    assert reference[199, fossen.RPM] == 0
    assert reference[200, fossen.RPM] == 1800
    np.testing.assert_array_equal(reference[999, fossen.ELEVATOR:], [0, 0])
    np.testing.assert_array_equal(reference[1000, fossen.ELEVATOR:], SCENARIOS[canonical](10)[1:])
    print(f"{name}: maximum state difference {np.max(np.abs(reference[:, :15] - actual)):.3g}")


def test_partial_duration_rounds_up_and_geography_matches():
    data = fossen.run("thrust", duration=0.025)
    np.testing.assert_allclose(data[:, fossen.T], [0.01, 0.02, 0.03])
    np.testing.assert_array_equal(data[:, fossen.DEPTH], [50, 50, 50])
    for north, east, latitude, longitude in ((0, 0, 13, 80), (1000, 500, 13, 80), (0, 1000, 60, 179.999)):
        assert fossen.ned_to_latlon(north, east, latitude, longitude) == pytest.approx(
            ned_to_geodetic(north, east, latitude, longitude), abs=1e-12,
        )


def test_actuator_units_limits_reverse_flow_and_surface_boundary_match():
    vehicle, parameters = fossen.AUV(), VehicleParameters()
    for surge in (-2, 0, 3):
        velocity = np.array([surge, 0, 0, 0, 0, 0])
        command = (1800, 1, 3)
        np.testing.assert_allclose(
            vehicle.actuator_wrench(velocity, *command),
            actuator_forces(velocity, command, parameters),
        )
    for command in ((-1, 0, 0), (3001, 0, 0), (0, 21, 0), (0, 0, -21), (0, np.nan, 0)):
        with pytest.raises(ValueError, match="Actuators"):
            vehicle.actuator_wrench(np.zeros(6), *command)
    state = fossen.zero_state()
    state[2], state[9] = 0, -1
    command = fossen.scenario_command("thrust", 0)
    for depth in (-0.1, 0.0, 0.1, 0.5, 10.0):
        state[2] = depth
        for pitch in (0.0, 0.3, -0.3):
            state[3:7] = quaternion_from_euler(0.2, pitch, 0.7)
            np.testing.assert_allclose(
                vehicle.step_rk4(state, command, 0.01),
                step_vehicle(state, [0, 0, 0], 0, 0.01, parameters), atol=1e-12,
            )


@pytest.mark.parametrize("kwargs", [{"duration": 0}, {"dt": -1}, {"duration": np.nan}, {"dt": np.inf}])
def test_invalid_run_timing_is_rejected(kwargs):
    with pytest.raises(ValueError, match="finite and positive"):
        fossen.run("thrust", **kwargs)
