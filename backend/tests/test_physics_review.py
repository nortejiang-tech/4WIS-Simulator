"""Independent mechanics regressions from the ASTRA review (2026-09-08).

Manufactured forces isolate Newton-Euler balance from tyre calibration;
equilibrium, virtual work and reset reproducibility are independent oracles.
"""
from dataclasses import replace

import numpy as np
import pytest

from sim4wis.controller.registry import make_strategy
from sim4wis.controller.rws_common import axle_cornering_stiffness
from sim4wis.core.state import ControlCommand, DriverInput, EnvironmentState, VehicleParams
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.geometry import steering_linkage_metrics, wheel_rack_force_from_linkage
from sim4wis.vehicle.kinematic import KinematicModel
from sim4wis.vehicle.kingpin import kingpin_torque_terms
from sim4wis.vehicle.model_core import semi_implicit_wheel_spin
from sim4wis.vehicle.multibody import MultiBodyModel
from sim4wis.vehicle.tire import LinearTireModel, PacejkaTireModel


class LoadProportionalLateralForce(LinearTireModel):
    """A known distributed external force through the static load resultant."""
    def forces(self, alpha, kappa, fz, mu):
        return 0.0, 0.1 * fz, 0.0


def clean_params(**kw):
    return replace(VehicleParams(), drag_coeff_cd=0.0,
                   rolling_resistance_coeff=0.0, aero_lift_coeff_front=0.0,
                   aero_lift_coeff_rear=0.0, static_toe_front=0.0,
                   static_toe_rear=0.0, camber_thrust_coeff=0.0, **kw)


def test_body_rhs_has_no_side_effects_and_is_repeatable():
    model = SimplifiedDynamicModel(VehicleParams())
    fz = model.state.fz.copy()
    args = (np.array([10., 1., .2]), np.full(4, 10 / model.params.tire_radius),
            np.zeros(4), np.full(4, .9), np.zeros(4), 0., EnvironmentState())
    d1 = model._derivatives(*args)
    d2 = model._derivatives(*args)
    np.testing.assert_array_equal(model.state.fz, fz)
    np.testing.assert_array_equal(d1, d2)


@pytest.mark.parametrize("model_cls", [SimplifiedDynamicModel, MultiBodyModel])
def test_force_through_cg_does_not_create_yaw(model_cls):
    p = clean_params(cg_to_front=1.0)
    model = model_cls(p, LoadProportionalLateralForce())
    cmd = ControlCommand.zero()
    model.step(1e-5, cmd, EnvironmentState())
    assert abs(model.state.yaw_rate / 1e-5) < 2e-5
    assert model.state.ay == pytest.approx(.981, abs=2e-4)


@pytest.mark.parametrize("model_cls", [SimplifiedDynamicModel, MultiBodyModel])
def test_reset_replays_like_a_fresh_model(model_cls):
    p = clean_params(tire_relax_length=.4, longitudinal_mode="torque")
    used = model_cls(p)
    used.state.vx = 8.0
    used.state.wheel_omega[:] = 8.0 / p.tire_radius
    turn = ControlCommand.zero()
    turn.delta_cmd[:2] = .12
    turn.brake_cmd[:] = .3
    for _ in range(50):
        used.step(.005, turn, EnvironmentState())
    used.reset()
    fresh = model_cls(p)
    launch = ControlCommand.zero()
    launch.drive_torque_cmd[:] = 200.0
    for _ in range(5):
        a = used.step(.005, launch, EnvironmentState())
        b = fresh.step(.005, launch, EnvironmentState())
        np.testing.assert_allclose([a.vx, a.vy, a.yaw_rate], [b.vx, b.vy, b.yaw_rate], atol=1e-12)
        np.testing.assert_allclose(a.wheel_omega, b.wheel_omega, atol=1e-12)
        np.testing.assert_allclose(a.fz, b.fz, atol=1e-10)


@pytest.mark.parametrize("tire", [LinearTireModel(), PacejkaTireModel()])
@pytest.mark.parametrize("fz,mu", [(0., 1.), (7000., 0.)])
def test_no_contact_or_no_friction_means_exactly_zero_force(tire, fz, mu):
    assert tire.forces(.1, .1, fz, mu) == (0., 0., 0.)


@pytest.mark.parametrize("kappa", [-.05, .05, .2])
def test_wheel_integrator_preserves_actual_nonlinear_torque_equilibrium(kappa):
    tire = PacejkaTireModel()
    speed, r, fz, mu = 5., .395, 7000., .9
    omega = speed * (1 + kappa) / r
    fx = tire.forces(.03, kappa, fz, mu)[0]
    new, _, _ = semi_implicit_wheel_spin(
        dt=.005, wheel_omega=np.full(4, omega), vx_wheel=np.full(4, speed),
        alpha=np.full(4, .03), fz=np.full(4, fz), mu=np.full(4, mu),
        torque=np.full(4, r * fx), tire=tire, tire_radius=r, wheel_inertia=2.5)
    np.testing.assert_allclose(new, omega, atol=1e-10, rtol=1e-10)


def test_kingpin_planar_moment_matches_contact_patch_cross_product():
    p = VehicleParams()
    susp = replace(p.suspension, kingpin_inclination=0.)
    fx, fy = np.full(4, 2000.), np.full(4, 1000.)
    terms = kingpin_torque_terms(fx, fy, np.zeros(4), np.zeros(4), susp, tire_radius=p.tire_radius)
    # r_contact = (-mechanical trail, side*scrub); actuator effort = -(r x F)_z.
    trail = p.tire_radius * np.tan(susp.caster_angle)
    np.testing.assert_allclose(terms["m_fy"], trail * fy)
    np.testing.assert_allclose(terms["m_fx"], [1., -1., 1., -1.] * fx * susp.scrub_radius)


@pytest.mark.parametrize("angle", [-.2, 0., .2])
def test_linkage_force_obeys_virtual_work(angle):
    p = VehicleParams()
    d = np.full(4, angle)
    eps = 1e-6
    plus = steering_linkage_metrics(d + eps, p.steering_geometry)["rack_travel"]
    minus = steering_linkage_metrics(d - eps, p.steering_geometry)["rack_travel"]
    ds_ddelta = (plus - minus) / (2 * eps)
    torque = np.full(4, 100.)
    rack, motor, _ = wheel_rack_force_from_linkage(
        torque, d, p.steering_geometry, p.steering_arm_length,
        p.pinion_radius, 1., p.motor_gear_ratio)
    # Reported force uses a steering-positive convention; its magnitude must
    # still satisfy |F_rack ds| = |tau ddelta| for a lossless linkage.
    np.testing.assert_allclose(abs(rack * ds_ddelta), abs(torque), rtol=2e-6)
    np.testing.assert_allclose(motor, rack * p.pinion_radius / p.motor_gear_ratio)


def test_rear_steer_reference_uses_actual_axle_stiffness_scales():
    p = VehicleParams()
    cf, cr = axle_cornering_stiffness(p)
    assert cf == pytest.approx(2 * p.tire_c_alpha * p.tire_c_alpha_front_scale)
    assert cr == pytest.approx(2 * p.tire_c_alpha * p.tire_c_alpha_rear_scale)


@pytest.mark.parametrize("strategy", ["ackermann", "ideal_ackermann", "rear_wheel_steer"])
def test_driver_cornering_target_respects_available_grip(strategy):
    p = VehicleParams()
    model = KinematicModel(p)
    model.state.mu_avg = .85
    controller = make_strategy(strategy, p)
    cmd = controller.compute(DriverInput(throttle=.5, steering=1.), model.state, .005)
    s = model.step(.005, cmd, EnvironmentState(mu=.85))
    demand = abs(s.yaw_rate) * np.hypot(s.vx, s.vy)
    assert demand <= .85 * 9.81 * (1 + 1e-8)


def test_load_transfer_does_not_create_weight_at_wheel_lift():
    from sim4wis.vehicle.load_transfer import vertical_loads
    p = VehicleParams()
    for ax, ay in [(0., 25.), (25., 0.), (-25., -25.)]:
        fz = vertical_loads(p, ax, ay)
        assert np.min(fz) >= 0.
        assert sum(fz) == pytest.approx(p.mass * 9.81)


def test_quasi_static_yaw_balance_is_about_cg():
    from sim4wis.vehicle.model_core import solve_steady_state_body
    p = clean_params(cg_to_front=1.0)
    x = p.wheel_positions_body()[:, 0]
    ca = np.array([90000., 90000., 130000., 130000.])
    delta = np.array([.03, .03, 0., 0.])
    v = 15.
    beta, r = solve_steady_state_body(speed=v, delta=delta, c_alpha=ca,
                                    wheel_positions_body=p.wheel_positions_body(),
                                    mass=p.mass, cg_x=p.wheelbase / 2 - p.cg_to_front)
    fy = -ca * (beta + r * x / v - delta)
    assert sum(fy) == pytest.approx(p.mass * v * r)
    assert sum((x - (p.wheelbase / 2 - p.cg_to_front)) * fy) == pytest.approx(0., abs=1e-9)


def test_quasi_static_kc_ignores_absolute_road_altitude():
    from sim4wis.environment.disturbance import WheelEnvLocal
    class ElevatedRoad:
        base_mu = .9
        def wheel_env(self, pos):
            return WheelEnvLocal(ground_z=10.)
    p = clean_params(bump_steer_coeff=1.)
    high, low = SimplifiedDynamicModel(p), SimplifiedDynamicModel(p)
    a = high.step(.005, ControlCommand.zero(), EnvironmentState(scene=ElevatedRoad()))
    b = low.step(.005, ControlCommand.zero(), EnvironmentState(mu=.9))
    np.testing.assert_allclose(a.delta, b.delta, atol=1e-12)


@pytest.mark.parametrize("strategy", ["ackermann", "ideal_ackermann", "rear_wheel_steer", "crab"])
def test_parking_steering_does_not_require_throttle(strategy):
    p = VehicleParams()
    s = KinematicModel(p).state
    controller = make_strategy(strategy, p)
    parked = controller.compute(DriverInput(throttle=0., steering=.5), s, .005)
    rolling = make_strategy(strategy, p).compute(DriverInput(throttle=.1, steering=.5), s, .005)
    np.testing.assert_allclose(parked.delta_cmd, rolling.delta_cmd, atol=1e-12)
    np.testing.assert_array_equal(parked.wheel_speed_cmd, np.zeros(4))


@pytest.mark.parametrize("speed", [-5., 0., 5.])
@pytest.mark.parametrize("torque", [-500., 500., 5000.])
def test_implicit_wheel_balance_uses_returned_tire_force(speed, torque):
    tire = PacejkaTireModel()
    omega = np.full(4, speed / .395)
    new, force, _ = semi_implicit_wheel_spin(
        dt=.005, wheel_omega=omega, vx_wheel=np.full(4, speed), alpha=np.full(4, .04),
        fz=np.full(4, 7000.), mu=np.full(4, .9), torque=np.full(4, torque),
        tire=tire, tire_radius=.395, wheel_inertia=2.5)
    np.testing.assert_allclose(2.5 * (new - omega), .005 * (torque - .395 * force.fx), atol=2e-8)


def test_wider_rear_axle_preserves_common_icr_at_full_steer():
    p = clean_params(track_front=1.5, track_rear=2.4)
    s = KinematicModel(p).state
    cmd = make_strategy('ideal_ackermann', p).compute(DriverInput(throttle=.1, steering=1.), s)
    wheels = p.wheel_positions_body()
    vel = np.column_stack((-wheels[:, 1] + cmd.icr_target_body[1], wheels[:, 0]))
    lateral = -np.sin(cmd.delta_cmd) * vel[:, 0] + np.cos(cmd.delta_cmd) * vel[:, 1]
    np.testing.assert_allclose(lateral, 0., atol=1e-12)


def test_spin_command_has_a_manoeuvre_rate_independent_of_highway_top_speed():
    for vmax in [20., 55.6]:
        p = clean_params(v_max=vmax, steer_limit=np.deg2rad(80.))
        model = KinematicModel(p)
        cmd = make_strategy('zero_radius', p).compute(DriverInput(throttle=1.), model.state)
        s = model.step(.005, cmd, EnvironmentState())
        assert s.yaw_rate == pytest.approx(np.deg2rad(30.))


def test_kinematic_reset_preserves_static_axle_loads():
    model = KinematicModel(clean_params(cg_to_front=1.))
    fz = model.state.fz.copy()
    model.reset()
    np.testing.assert_array_equal(model.state.fz, fz)


@pytest.mark.parametrize('outer_x,inner_x', [(0., -.35), (-.15, -.35)])
def test_degenerate_or_dead_centre_linkage_is_not_marked_reachable(outer_x, inner_x):
    p = VehicleParams()
    g = replace(p.steering_geometry, front_outer_x=outer_x, front_outer_y=0.,
                front_inner_x=inner_x, front_inner_y=0., front_rack_axis_deg=0.)
    metrics = steering_linkage_metrics(np.zeros(4), g)
    assert not any(metrics['valid'][:2])
