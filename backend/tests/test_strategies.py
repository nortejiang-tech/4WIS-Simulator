"""Tests for the five Phase-1 controller strategies.

Each test focuses on a property that's easy to assert in isolation:
    * Ackermann: rear wheels stay at 0; front wheels have correct
                 inner/outer differential.
    * Ideal Ackermann: ALL FOUR wheel perpendicular lines meet at one point —
                       this is the headline 4WIS property.
    * Rear-steer (in-phase 1:1): all four wheels parallel → pure translation.
    * Rear-steer (counter-phase -1): ICR on body-Y axis (centred).
    * Crab: all four wheels at the same angle; ω = 0.
    * Zero-radius: ω ≠ 0 but body-origin velocity = 0; wheels point through origin.
"""

from __future__ import annotations

import numpy as np
import pytest

from sim4wis.controller.registry import available_strategies, make_strategy
from sim4wis.core.state import DriverInput, VehicleParams, VehicleState
from sim4wis.vehicle.geometry import line_intersection, wheel_perpendicular_dir


@pytest.fixture
def params() -> VehicleParams:
    return VehicleParams()  # use defaults


def test_registry_lists_builtin_strategies() -> None:
    names = set(available_strategies())
    # Plugins may add more; the six built-ins must always be present.
    assert names >= {
        "ackermann", "ideal_ackermann", "rear_wheel_steer",
        "crab", "zero_radius", "follow_trajectory",
    }


def _intersect_all_pairs(positions: np.ndarray, deltas: np.ndarray) -> np.ndarray:
    """Compute pairwise intersection points of the four wheel perpendicular
    lines. Returns a (N, 2) array of intersection points (one per pair)."""
    pts = []
    for i in range(4):
        for j in range(i + 1, 4):
            inter = line_intersection(
                positions[i],
                wheel_perpendicular_dir(deltas[i]),
                positions[j],
                wheel_perpendicular_dir(deltas[j]),
            )
            pts.append(inter)
    return np.array(pts)


# ---------------------------------------------------------------------------
# Ideal Ackermann — THE headline property
# ---------------------------------------------------------------------------


def test_ideal_ackermann_shares_icr_across_four_wheels(params: VehicleParams) -> None:
    """Across all six wheel pairs, the perpendicular-line intersections agree —
    i.e. all four wheels have a *single common ICR*. This is what makes the
    strategy "ideal" relative to the front-only Ackermann."""
    strat = make_strategy("ideal_ackermann", params)
    state = VehicleState()
    # Pick a non-trivial steering input so the wheels actually deflect.
    cmd = strat.compute(DriverInput(throttle=0.5, steering=0.7), state)
    wheels = params.wheel_positions_body()
    intersections = _intersect_all_pairs(wheels, cmd.delta_cmd)

    # All 6 intersections should be the same point.
    centroid = np.nanmean(intersections, axis=0)
    distances = np.linalg.norm(intersections - centroid, axis=1)
    max_spread = float(np.nanmax(distances))
    assert max_spread < 1e-6, (
        f"ideal Ackermann should put all 4 wheel perpendiculars through one "
        f"point — got spread {max_spread:.3e} m across pairwise intersections."
    )

    # And that point should match the strategy's declared ICR target.
    assert np.allclose(centroid, cmd.icr_target_body, atol=1e-6)


def test_ideal_ackermann_straight_line(params: VehicleParams) -> None:
    """Zero steering → all four wheels at δ=0."""
    strat = make_strategy("ideal_ackermann", params)
    cmd = strat.compute(DriverInput(throttle=0.5, steering=0.0), VehicleState())
    assert np.allclose(cmd.delta_cmd, 0.0, atol=1e-9)
    assert np.all(np.isnan(cmd.icr_target_body))


def test_ideal_ackermann_inner_outer_relationship(params: VehicleParams) -> None:
    """In a left turn, the LEFT (inner) wheels should have a larger |δ|
    than the RIGHT (outer) wheels — this is the Ackermann condition."""
    strat = make_strategy("ideal_ackermann", params)
    cmd = strat.compute(DriverInput(throttle=0.3, steering=+0.5), VehicleState())
    # WheelIndex: 0=FL, 1=FR, 2=RL, 3=RR
    assert abs(cmd.delta_cmd[0]) > abs(cmd.delta_cmd[1]), \
        f"FL ({cmd.delta_cmd[0]:.4f}) should exceed FR ({cmd.delta_cmd[1]:.4f}) on a left turn"
    assert abs(cmd.delta_cmd[2]) > abs(cmd.delta_cmd[3]), \
        f"RL ({cmd.delta_cmd[2]:.4f}) should exceed RR ({cmd.delta_cmd[3]:.4f})"
    # Rear wheels should be in OPPOSITE phase to front (natural 4WIS at all speeds
    # because ICR is on Y axis through body center).
    assert np.sign(cmd.delta_cmd[0]) != np.sign(cmd.delta_cmd[2])


# ---------------------------------------------------------------------------
# Traditional Ackermann
# ---------------------------------------------------------------------------


def test_ackermann_rear_wheels_zero(params: VehicleParams) -> None:
    strat = make_strategy("ackermann", params)
    cmd = strat.compute(DriverInput(throttle=0.5, steering=0.6), VehicleState())
    assert abs(cmd.delta_cmd[2]) < 1e-9
    assert abs(cmd.delta_cmd[3]) < 1e-9


def test_ackermann_front_inner_outer(params: VehicleParams) -> None:
    """Front-axle inner wheel has larger |δ| than outer."""
    strat = make_strategy("ackermann", params)
    cmd = strat.compute(DriverInput(throttle=0.5, steering=+0.5), VehicleState())
    assert abs(cmd.delta_cmd[0]) > abs(cmd.delta_cmd[1])
    # ICR should sit on rear-axle extension (x = -L/2)
    L = params.wheelbase
    assert abs(cmd.delta_cmd[0]) > abs(cmd.delta_cmd[1])
    assert abs(cmd.icr_target_body[0] - (-L / 2.0)) < 1e-9


# ---------------------------------------------------------------------------
# Crab — all wheels parallel, no yaw
# ---------------------------------------------------------------------------


def test_crab_all_wheels_parallel(params: VehicleParams) -> None:
    strat = make_strategy("crab", params)
    cmd = strat.compute(DriverInput(throttle=0.5, steering=+0.5), VehicleState())
    # All four δ identical
    assert np.allclose(cmd.delta_cmd, cmd.delta_cmd[0])
    # ICR at infinity (NaN)
    assert np.all(np.isnan(cmd.icr_target_body))


# ---------------------------------------------------------------------------
# Zero radius — spin in place
# ---------------------------------------------------------------------------


def test_zero_radius_perpendiculars_through_origin() -> None:
    # The LS9 default steer_limit (±35°) cannot reach the tangential angles a
    # true zero-radius spin needs (≈±61° for its geometry) — commands get
    # clipped. Use a high-articulation vehicle (e.g. delivery robot ±90°) so
    # the pure geometry is testable.
    params = VehicleParams(steer_limit=1.5708)
    strat = make_strategy("zero_radius", params)
    cmd = strat.compute(DriverInput(throttle=0.5, steering=+1.0), VehicleState())
    wheels = params.wheel_positions_body()
    # Each wheel's perpendicular line should pass through origin (0, 0).
    for i in range(4):
        d_perp = wheel_perpendicular_dir(cmd.delta_cmd[i])
        # Distance from origin to line: |cross((origin - wheel), d_perp)|
        # since d_perp is unit-length, this is just |(-wheel) × d_perp|
        cross = (-wheels[i, 0]) * d_perp[1] - (-wheels[i, 1]) * d_perp[0]
        assert abs(cross) < 1e-9, f"wheel {i} perpendicular misses origin by {abs(cross):.3e}"
    assert np.allclose(cmd.icr_target_body, [0.0, 0.0])


# ---------------------------------------------------------------------------
# Rear-steer — special cases
# ---------------------------------------------------------------------------


def test_rear_steer_in_phase_one_to_one_is_pure_translation(params: VehicleParams) -> None:
    strat = make_strategy("rear_wheel_steer", params)
    cmd = strat.compute(
        DriverInput(throttle=0.5, steering=+0.4,
                    mode_params={"rws_mode": "fixed_ratio", "rear_ratio": +1.0}),
        VehicleState(),
    )
    # All four wheels parallel (effectively a crab) → equal δ
    assert np.allclose(cmd.delta_cmd, cmd.delta_cmd[0], atol=1e-9)
    assert np.all(np.isnan(cmd.icr_target_body))


def test_rear_steer_counter_phase_centred_icr(params: VehicleParams) -> None:
    strat = make_strategy("rear_wheel_steer", params)
    cmd = strat.compute(
        DriverInput(throttle=0.4, steering=+0.5,
                    mode_params={"rws_mode": "fixed_ratio", "rear_ratio": -1.0}),
        VehicleState(),
    )
    # Full counter-phase → ICR sits on body-Y axis (x = 0)
    assert abs(cmd.icr_target_body[0]) < 1e-6
