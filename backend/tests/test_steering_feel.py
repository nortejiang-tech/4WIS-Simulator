"""Tests — steering feel layer (work-package B).

Guards the three-layer mapping in controller/steering_feel.py:
  * full wheel travel stays usable and monotonic at every speed — the layer
    exists to remove a dead zone, so it must not introduce one,
  * full lock is still reachable at parking speed (turning circle unchanged),
  * full input at speed keeps a_y at the grip reference, on any μ,
  * the gear ratio tracks the hardware, so swapping wheels preserves feel,
  * the angle→curvature conversion matches ideal_ackermann's own geometry.
"""

from __future__ import annotations

import math
from dataclasses import replace

import pytest

from sim4wis.controller.ideal_ackermann import (
    _max_curvature,
    curvature_from_inner_front_angle,
    inner_front_angle_from_curvature,
)
from sim4wis.controller.steering_feel import (
    front_steer_angle,
    gear_ratio,
    low_speed_gear_ratio,
)
from sim4wis.core.state import VehicleParams

SPEEDS_KMH = (0.0, 5.0, 10.0, 30.0, 60.0, 100.0)


def _p(**over) -> VehicleParams:
    return VehicleParams(**over)


# ── 1. the whole travel does something, at every speed ───────────────────────

@pytest.mark.parametrize("v_kmh", SPEEDS_KMH)
def test_full_travel_is_monotonic_and_usable(v_kmh):
    """Every further degree of wheel must still change the front angle.

    Regression guard: the soft limit was a hard clamp, so past a threshold the
    input did nothing — usable travel was 54% at 10 km/h, 14% at 60 and 6.8% at
    100, i.e. a *worse* dead zone than the pre-layer car it replaced.
    """
    p = _p()
    v = v_kmh / 3.6
    prev = front_steer_angle(p, 0.0, v)
    for n in range(1, 201):
        d = front_steer_angle(p, n / 200.0, v)
        assert d > prev, f"response flat at |steering|={n / 200.0:.3f}, v={v_kmh} km/h"
        prev = d


def test_on_centre_gain_is_never_reduced():
    """Small corrections must keep working at motorway speed — the soft limit
    compresses the extremes, not the centre."""
    p = _p()
    eps = 1e-4
    for v_kmh in SPEEDS_KMH:
        v = v_kmh / 3.6
        gain = (front_steer_angle(p, eps, v) - front_steer_angle(p, -eps, v)) / (2 * eps)
        raw_gain = math.radians((p.steer_wheel_range / 2.0) / gear_ratio(p, v))
        assert gain >= raw_gain * 0.99, f"on-centre gain collapsed at {v_kmh} km/h"


# ── 2. parking speed still reaches full lock ─────────────────────────────────

def test_full_lock_reachable_at_parking_speed():
    """Turning-circle guard. A bare tanh saturation asymptotes and never gets
    there, which cost ~10% of lock and grew the minimum radius 3.04 → 3.4 m."""
    p = _p()
    delta = front_steer_angle(p, steering=1.0, v=0.0)
    assert delta == pytest.approx(p.steer_limit, rel=1e-6)


def test_minimum_turning_radius_preserved():
    p = _p()
    delta = front_steer_angle(p, steering=1.0, v=0.0)
    kappa = curvature_from_inner_front_angle(p, delta)
    assert 1.0 / kappa == pytest.approx(1.0 / _max_curvature(p), rel=1e-6)


# ── 3. grip reference is respected, and follows μ ────────────────────────────

@pytest.mark.parametrize("mu", [0.85, 0.50, 0.20])
def test_full_input_at_speed_lands_on_the_grip_reference(mu):
    """At speed a full input must command a_y ≈ steer_ay_ref_frac·μ·g — not 9g
    (the pre-layer failure), and not zero either."""
    p = _p()
    v = 60.0 / 3.6
    delta = front_steer_angle(p, steering=1.0, v=v, mu_avg=mu)
    ay = v * v * math.tan(delta) / p.wheelbase
    assert ay == pytest.approx(p.steer_ay_ref_frac * mu * 9.81, rel=0.02)


def test_soft_limit_tightens_on_ice():
    p = _p()
    v = 60.0 / 3.6
    assert front_steer_angle(p, 1.0, v, mu_avg=0.20) < front_steer_angle(p, 1.0, v, mu_avg=0.85)


def test_sign_and_odd_symmetry():
    p = _p()
    d_pos = front_steer_angle(p, steering=0.5, v=10.0)
    d_neg = front_steer_angle(p, steering=-0.5, v=10.0)
    assert d_pos > 0 > d_neg
    assert d_pos == pytest.approx(-d_neg)


# ── 4. ratios follow the hardware ────────────────────────────────────────────

@pytest.mark.parametrize("sw_range", [270.0, 540.0, 900.0])
def test_feel_survives_a_wheel_swap(sw_range):
    """Changing only `steer_wheel_range` must preserve the mapping.

    Regression guard: `steer_ratio_low` was hard-coded at 4.0, a value derived
    for a 270° wheel while the default range is 540°. Full lock then arrived at
    54% of travel and low-speed gain was ~1.85× the pre-layer car — the exact
    opposite of the "low-speed feel unchanged" claim.
    """
    p = _p(steer_wheel_range=sw_range)
    assert front_steer_angle(p, 1.0, 0.0) == pytest.approx(_p().steer_limit, rel=1e-6)
    assert front_steer_angle(p, 1.0, 60.0 / 3.6) == pytest.approx(
        front_steer_angle(_p(), 1.0, 60.0 / 3.6), rel=1e-6
    )


def test_auto_low_ratio_puts_full_travel_at_the_steer_limit():
    for sw_range in (270.0, 540.0, 900.0):
        p = _p(steer_wheel_range=sw_range)
        i_low = low_speed_gear_ratio(p)
        assert (sw_range / 2.0) / i_low == pytest.approx(math.degrees(p.steer_limit))


def test_explicit_ratio_overrides_the_auto_derivation():
    p = _p(steer_ratio_low=6.0, steer_ratio_high=18.0)
    assert low_speed_gear_ratio(p) == pytest.approx(6.0)
    assert gear_ratio(p, 0.0) == pytest.approx(6.0)
    assert gear_ratio(p, 1e6) == pytest.approx(18.0, rel=1e-3)


def test_ratio_interpolates_monotonically_with_speed():
    p = _p()
    prev = gear_ratio(p, 0.0)
    assert prev == pytest.approx(low_speed_gear_ratio(p))
    for v in range(0, 60, 5):
        r = gear_ratio(p, float(v))
        assert r >= prev - 1e-9
        prev = r
    assert gear_ratio(p, 100.0) > gear_ratio(p, 5.0)


# ── 5. angle ↔ curvature uses ideal_ackermann's own geometry ─────────────────

def test_curvature_conversion_round_trips():
    """κ → δ → κ must be exact. The bicycle relation κ = tanδ/L is NOT this
    vehicle's geometry: ideal_ackermann steers both axles symmetrically, so the
    ICR lever arm is L/2 and the track enters as well. Using the bicycle form
    dropped full-lock κ from 0.3291 to 0.2216 (radius 3.04 m → 4.51 m)."""
    p = _p()
    for frac in (0.01, 0.05, 0.25, 0.5, 1.0):
        kappa = _max_curvature(p) * frac
        delta = inner_front_angle_from_curvature(p, kappa)
        assert curvature_from_inner_front_angle(p, delta) == pytest.approx(kappa, rel=1e-12)


def test_full_lock_angle_is_the_steer_limit():
    """The conversion must agree with _max_curvature's own derivation."""
    p = _p()
    delta = inner_front_angle_from_curvature(p, _max_curvature(p))
    assert delta == pytest.approx(p.steer_limit, rel=1e-9)


def test_curvature_conversion_is_odd_symmetric():
    """δ is the *inner* wheel's angle and "inner" swaps sides with the turn, so
    the half-track term must always widen the radius. Regression guard: letting
    a negative tan δ into that term shrank it instead, making a right turn
    tighter than the mirror-image left turn (κ 0.568 vs 0.301 at full lock) and
    pushing the outer wheel past its steer limit — the four wheels then no
    longer shared one ICR."""
    p = _p()
    for deg in (0.5, 3.0, 12.0, 30.0, 35.0):
        d = math.radians(deg)
        assert curvature_from_inner_front_angle(p, -d) == pytest.approx(
            -curvature_from_inner_front_angle(p, d), rel=1e-15
        )
    for frac in (0.05, 0.5, 1.0):
        k = _max_curvature(p) * frac
        assert inner_front_angle_from_curvature(p, -k) == pytest.approx(
            -inner_front_angle_from_curvature(p, k), rel=1e-15
        )


def test_steering_produces_a_single_shared_icr():
    """The four wheel perpendiculars must meet at one point — the defining
    property of the ideal-Ackermann allocation."""
    import numpy as np
    from sim4wis.controller.registry import make_strategy
    from sim4wis.core.state import DriverInput, VehicleState

    p = _p()
    strat = make_strategy("ideal_ackermann", p)
    wheels = p.wheel_positions_body()
    for steering in (-0.9, -0.4, -0.1, 0.1, 0.4, 0.9):
        cmd = strat.compute(DriverInput(throttle=0.5, steering=steering), VehicleState())
        # each wheel's ICR lies on its own perpendicular at distance R
        r_icr = cmd.icr_target_body
        for i in range(4):
            arm = r_icr - wheels[i]
            heading = np.array([math.cos(cmd.delta_cmd[i]), math.sin(cmd.delta_cmd[i])])
            # the wheel must roll perpendicular to its arm to the ICR
            assert abs(float(np.dot(arm, heading))) < 1e-9, (
                f"wheel {i} not perpendicular to its ICR arm at steering={steering}"
            )


def test_left_and_right_steering_mirror_exactly():
    from sim4wis.controller.registry import make_strategy
    from sim4wis.core.state import DriverInput, VehicleState

    p = _p()
    strat = make_strategy("ideal_ackermann", p)
    for steering in (0.1, 0.4, 0.9, 1.0):
        left = strat.compute(DriverInput(throttle=0.5, steering=steering), VehicleState())
        right = strat.compute(DriverInput(throttle=0.5, steering=-steering), VehicleState())
        # FL mirrors FR, RL mirrors RR
        assert left.delta_cmd[0] == pytest.approx(-right.delta_cmd[1], rel=1e-12)
        assert left.delta_cmd[1] == pytest.approx(-right.delta_cmd[0], rel=1e-12)
        assert left.icr_target_body[1] == pytest.approx(-right.icr_target_body[1], rel=1e-12)


def test_bicycle_conversion_would_disagree():
    """Documents the size of the error the bicycle relation introduced, so a
    future refactor cannot quietly reintroduce it."""
    p = _p()
    kappa_bicycle = math.tan(p.steer_limit) / p.wheelbase
    assert kappa_bicycle < _max_curvature(p) * 0.7
