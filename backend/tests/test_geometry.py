"""Unit tests for the geometry helpers."""

from __future__ import annotations

import numpy as np

from sim4wis.vehicle.geometry import (
    line_intersection,
    steer_angle_for_icr,
    vehicle_icr_from_velocity,
    wheel_perpendicular_dir,
    wrap_to_pmhalfpi,
    wrap_to_pmpi,
)


def test_wrap_to_pmpi() -> None:
    assert abs(wrap_to_pmpi(0.0) - 0.0) < 1e-12
    assert abs(float(wrap_to_pmpi(2 * np.pi)) - 0.0) < 1e-12
    assert abs(float(wrap_to_pmpi(3 * np.pi)) - np.pi) < 1e-12 or abs(
        float(wrap_to_pmpi(3 * np.pi)) + np.pi
    ) < 1e-12


def test_wrap_to_pmhalfpi() -> None:
    # In-range angles stay
    assert abs(wrap_to_pmhalfpi(0.5) - 0.5) < 1e-12
    # 100° → -80°
    assert abs(wrap_to_pmhalfpi(np.deg2rad(100)) - np.deg2rad(-80)) < 1e-9
    # -120° → +60°
    assert abs(wrap_to_pmhalfpi(np.deg2rad(-120)) - np.deg2rad(60)) < 1e-9


def test_steer_angle_for_icr_straight_along_y() -> None:
    """If ICR is far away along Y axis (left), all wheels point ~straight."""
    wheels = np.array([[1.4, 0.78], [1.4, -0.78], [-1.4, 0.78], [-1.4, -0.78]])
    icr = np.array([0.0, 1000.0])  # very far → effectively straight
    delta = steer_angle_for_icr(wheels, icr)
    assert np.all(np.abs(delta) < 0.01)


def test_steer_angle_for_icr_zero_radius() -> None:
    """If ICR is at body origin, each wheel points tangentially."""
    wheels = np.array([[1.4, 0.78], [1.4, -0.78], [-1.4, 0.78], [-1.4, -0.78]])
    delta = steer_angle_for_icr(wheels, np.array([0.0, 0.0]))
    # All should be in [-π/2, π/2]
    assert np.all(np.abs(delta) <= np.pi / 2 + 1e-9)
    # Left/right mirror symmetry: FL vs FR and RL vs RR are opposite.
    assert abs(delta[0] + delta[1]) < 1e-9
    assert abs(delta[2] + delta[3]) < 1e-9
    # Antipodal wheels (FL/RR, FR/RL) lie on the same tangent line, so their
    # wrapped-to-±π/2 steer angles are equal.
    assert abs(delta[0] - delta[3]) < 1e-9
    assert abs(delta[1] - delta[2]) < 1e-9


def test_vehicle_icr_from_velocity() -> None:
    # Vehicle turning left at radius 10 m, forward 1 m/s
    # ω = v/R = 0.1 rad/s, ICR at (0, R) = (0, 10) in body frame
    icr = vehicle_icr_from_velocity(vx=1.0, vy=0.0, yaw_rate=0.1)
    assert abs(icr[0] - 0.0) < 1e-9
    assert abs(icr[1] - 10.0) < 1e-9

    # Straight line → NaN
    icr_inf = vehicle_icr_from_velocity(vx=10.0, vy=0.0, yaw_rate=0.0)
    assert np.all(np.isnan(icr_inf))


def test_line_intersection_basic() -> None:
    # Two lines through origin, perpendicular
    p = line_intersection(
        np.array([0.0, 0.0]), np.array([1.0, 0.0]),
        np.array([0.0, 0.0]), np.array([0.0, 1.0]),
    )
    assert np.allclose(p, [0.0, 0.0])

    # Parallel lines → NaN
    p = line_intersection(
        np.array([0.0, 0.0]), np.array([1.0, 0.0]),
        np.array([0.0, 1.0]), np.array([1.0, 0.0]),
    )
    assert np.all(np.isnan(p))


def test_wheel_perpendicular_dir() -> None:
    # δ = 0 → rolling along X, perpendicular along +Y
    d = wheel_perpendicular_dir(0.0)
    assert np.allclose(d, [0.0, 1.0])
    # δ = π/2 → rolling along Y, perpendicular along -X
    d = wheel_perpendicular_dir(np.pi / 2)
    assert np.allclose(d, [-1.0, 0.0])
