from __future__ import annotations

import math

import numpy as np

from sim4wis.core.state import N_WHEELS, VehicleParams
from sim4wis.vehicle.model_core import (
    camber_per_wheel,
    drive_force_per_wheel,
    fz_with_aero_lift,
    load_sensitive_cornering_stiffness,
    quasi_static_wheel_loads,
    rotate_wheel_forces_to_body,
    static_toe_offsets,
    steady_state_slip_angles,
    wheel_alignment,
    wheel_slip_kinematics,
)


def test_wheel_slip_kinematics_preserves_single_wheel_sign_convention() -> None:
    p = VehicleParams()
    delta = np.zeros(N_WHEELS)
    delta[0] = math.radians(10.0)
    wheel_omega = np.full(N_WHEELS, 12.0 / p.tire_radius)
    wheel_omega[0] = 12.0 * math.cos(delta[0]) / p.tire_radius
    kin = wheel_slip_kinematics(
        vx=12.0,
        vy=0.0,
        yaw_rate=0.0,
        wheel_omega=wheel_omega,
        delta=delta,
        wheel_positions_body=p.wheel_positions_body(),
        tire_radius=p.tire_radius,
    )

    assert np.isclose(kin.alpha[0], -math.radians(10.0))
    assert np.allclose(kin.alpha[1:], 0.0)
    assert np.allclose(kin.kappa, 0.0, atol=1e-12)


def test_wheel_force_rotation_maps_wheel_longitudinal_to_body_axes() -> None:
    fx_body, fy_body = rotate_wheel_forces_to_body(
        fx_wheel=np.array([100.0, 0.0, 0.0, 0.0]),
        fy_wheel=np.zeros(N_WHEELS),
        delta=np.array([math.pi / 2.0, 0.0, 0.0, 0.0]),
    )

    assert abs(fx_body[0]) < 1e-9
    assert np.isclose(fy_body[0], 100.0)


def test_static_toe_and_camber_projection_are_mirror_symmetric() -> None:
    p = VehicleParams(
        static_toe_front=math.radians(0.2),
        static_toe_rear=math.radians(0.1),
    )

    assert np.allclose(
        static_toe_offsets(p),
        [-p.static_toe_front, p.static_toe_front, -p.static_toe_rear, p.static_toe_rear],
    )
    assert np.allclose(
        camber_per_wheel(p),
        [p.suspension.camber, -p.suspension.camber, p.suspension.camber, -p.suspension.camber],
    )


def test_wheel_alignment_bundle_matches_projection_helpers() -> None:
    p = VehicleParams(
        static_toe_front=math.radians(0.2),
        static_toe_rear=math.radians(0.1),
    )
    alignment = wheel_alignment(p)

    assert np.allclose(alignment.toe_offsets, static_toe_offsets(p))
    assert np.allclose(alignment.camber, camber_per_wheel(p))


def test_drive_force_and_aero_lift_scale_with_speed_squared() -> None:
    p = VehicleParams(
        rolling_resistance_coeff=0.0,
        drag_coeff_cd=0.30,
        frontal_area=2.5,
        aero_lift_coeff_front=0.30,
        aero_lift_coeff_rear=0.15,
    )
    fz_static = np.full(N_WHEELS, 7000.0)

    fx_10 = drive_force_per_wheel(p, 10.0)[0]
    fx_20 = drive_force_per_wheel(p, 20.0)[0]
    fz_10 = fz_with_aero_lift(p, fz_static, 10.0)
    fz_20 = fz_with_aero_lift(p, fz_static, 20.0)

    assert np.isclose(fx_20 / fx_10, 4.0)
    assert np.isclose((fz_static[0] - fz_20[0]) / (fz_static[0] - fz_10[0]), 4.0)


def test_load_sensitive_cornering_stiffness_tracks_vertical_load() -> None:
    p = VehicleParams(tire_c_alpha=120_000.0, tire_load_sensitivity_exp=0.8)
    fz_nom = p.mass * 9.80665 / N_WHEELS
    c_alpha = load_sensitive_cornering_stiffness(
        p,
        np.array([fz_nom, fz_nom * 2.0, fz_nom * 0.5, fz_nom]),
    )

    assert c_alpha[1] > c_alpha[0] > c_alpha[2]
    assert np.isclose(c_alpha[0], 120_000.0)


def test_quasi_static_wheel_loads_bundle_matches_load_helpers() -> None:
    p = VehicleParams(aero_lift_coeff_front=0.30, aero_lift_coeff_rear=0.15)
    fz_static = np.full(N_WHEELS, 7000.0)
    loads = quasi_static_wheel_loads(p, fz_static, speed=30.0)

    expected_fz = fz_with_aero_lift(p, fz_static, 30.0)
    assert np.allclose(loads.fz_static, fz_static)
    assert np.allclose(loads.fz, expected_fz)
    assert np.allclose(loads.c_alpha, load_sensitive_cornering_stiffness(p, expected_fz))


def test_steady_state_slip_angles_make_high_speed_single_corner_more_sensitive() -> None:
    p = VehicleParams(
        mass=2900.0,
        wheelbase=3.16,
        track_front=1.7,
        track_rear=1.7,
        tire_c_alpha=120_000.0,
    )
    deltas = np.zeros(N_WHEELS)
    deltas[0] = math.radians(1.0)
    c_alpha = np.full(N_WHEELS, p.tire_c_alpha)

    alpha_low, body_low = steady_state_slip_angles(
        speed=5.0,
        delta=deltas,
        c_alpha=c_alpha,
        wheel_positions_body=p.wheel_positions_body(),
        mass=p.mass,
    )
    alpha_high, body_high = steady_state_slip_angles(
        speed=50.0,
        delta=deltas,
        c_alpha=c_alpha,
        wheel_positions_body=p.wheel_positions_body(),
        mass=p.mass,
    )

    assert body_low.used_bicycle
    assert body_high.used_bicycle
    assert abs(alpha_high[0]) > abs(alpha_low[0]) * 2.0
