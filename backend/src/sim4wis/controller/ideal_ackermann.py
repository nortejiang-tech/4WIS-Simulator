"""Ideal Ackermann strategy for 4WIS — all four wheels share the same ICR.

This is the 4WIS hero strategy. Given a desired curvature κ (signed,
κ > 0 = left turn), the ICR is placed on the body-Y axis at (0, 1/κ), so
the body origin has pure forward motion (vy_origin = 0) and every wheel's
rolling axis is perpendicular to its own radius to the ICR. Result:

    * Zero side-slip at every wheel (in kinematic terms).
    * No tyre-scrub during steady-state cornering.
    * The geometric "Ackermann condition" is satisfied exactly, not
      approximately, across all four wheels.

That last property is the testable promise the test_ideal_ackermann.py
checks via line intersection of the four wheel perpendiculars.

Steering input mapping:
    driver.steering ∈ [-1, +1] → κ ∈ [-κ_max, +κ_max], where κ_max
    corresponds to the inner wheel reaching `steer_limit` at the smallest
    achievable radius. Computed once at strategy init based on geometry.
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import (
    BodyMotionTarget,
    ControllerStrategy,
    compute_commands,
)
from sim4wis.controller.longitudinal import speed_command
from sim4wis.controller.steering_feel import front_steer_angle
from sim4wis.core.state import ControlCommand, DriverInput, VehicleParams, VehicleState


def _max_curvature(params: VehicleParams) -> float:
    """Tightest κ before the inner wheel hits its steer limit.

    Inner front wheel position (left turn, so inner = FL at (+L/2, +tf/2))
    relative to ICR at (0, 1/κ):
        δ_inner = atan2(L/2, 1/κ - tf/2)
    Solve δ_inner = steer_limit for κ:
        tan(steer_limit) = (L/2) / (1/κ - tf/2)
        1/κ - tf/2 = (L/2) / tan(steer_limit)
        1/κ = (L/2) / tan(steer_limit) + tf/2
    """
    L = params.wheelbase
    tf = params.track_front
    inv_kappa_min = (L / 2.0) / np.tan(params.steer_limit) + tf / 2.0
    return 1.0 / inv_kappa_min


def curvature_from_inner_front_angle(params: VehicleParams, delta: float) -> float:
    """Inverse of `_max_curvature`, for any inner-front angle δ (not just the limit).

    Inverting the same relation, 1/κ = (L/2)/tan δ + tf/2:

        κ(δ) = tan δ / (L/2 + (tf/2)·tan δ)

    This strategy places the ICR on the lateral axis through the vehicle
    *centre* — a symmetric 4WIS turn where both axles steer — which is why the
    lever arm is L/2, not L. Converting the feel layer's angle with the plain
    bicycle κ = tan δ / L instead quietly turns the vehicle into a front-steer
    car: at full lock that drops κ from 0.3291 to 0.2216, i.e. the minimum
    turning radius grows from 3.04 m to 4.51 m (+48%).

    δ is the *inner* wheel's angle, and "inner" swaps sides with the turn, so
    the relation must be odd-symmetric: the half-track term always widens the
    radius. Letting a negative tan δ into that term instead shrinks it, which
    made a right turn tighter than the mirror-image left turn (κ 0.568 vs
    0.301 at full lock) and pushed the outer wheel past its steer limit, so
    the four wheels no longer shared one ICR.
    """
    t = float(np.tan(delta))
    denom = params.wheelbase / 2.0 + (params.track_front / 2.0) * abs(t)
    return t / denom


def inner_front_angle_from_curvature(params: VehicleParams, kappa: float) -> float:
    """Inverse of `curvature_from_inner_front_angle` — κ → δ [rad].

    Used to express a curvature target as the physical front angle an
    experiment should command (`unit: front_deg`)."""
    k = float(kappa)
    # Odd-symmetric, matching curvature_from_inner_front_angle: the half-track
    # term uses |κ| so a right turn mirrors a left one exactly.
    denom = 1.0 - abs(k) * params.track_front / 2.0
    if abs(denom) < 1e-12:
        return float(np.copysign(params.steer_limit, k))
    return float(np.arctan(k * (params.wheelbase / 2.0) / denom))


class IdealAckermannStrategy(ControllerStrategy):
    name = "ideal_ackermann"

    def __init__(self, params: VehicleParams) -> None:
        super().__init__(params)
        self._kappa_max = _max_curvature(params)

    def compute(self, driver: DriverInput, state: VehicleState, dt: float = 0.0) -> ControlCommand:
        p = self.params
        # B3 bypass: when an experiment sets mode_params["steer_raw_rad"], the
        # value IS the front-axle angle and must skip the feel layer — so the
        # validation measures the vehicle, not the driver mapping.
        mp = driver.mode_params or {}
        raw_rad = mp.get("steer_raw_rad")
        if raw_rad is not None:
            delta_eff = float(raw_rad)
            delta_eff = max(-p.steer_limit, min(p.steer_limit, delta_eff))
        elif mp.get("steer_bypass_feel"):
            # Open-loop excitation bypass: the normalised input maps straight to
            # curvature the way it did before the feel layer existed. An
            # open-loop steer test measures the *vehicle*, so it must not be
            # reshaped by the driver-input mapping — same reasoning as the
            # front_deg experiments, but keeping the caller's normalised axis.
            kappa_direct = self._kappa_max * float(driver.steering)
            delta_eff = inner_front_angle_from_curvature(p, kappa_direct)
        else:
            # Steering feel (work-package B): normalised input → effective front-
            # axle angle via variable gear ratio + μ-aware soft limit, then to the
            # curvature this strategy builds its ICR from.
            delta_eff = front_steer_angle(p, float(driver.steering), float(state.vx),
                                          float(state.mu_avg))
        # Convert with THIS strategy's geometry (symmetric 4WIS, ICR on the
        # lateral axis through the vehicle centre), not the bicycle formula —
        # see curvature_from_inner_front_angle.
        kappa = curvature_from_inner_front_angle(p, delta_eff)
        v_cmd = speed_command(p, driver, float(state.vx), dt, float(state.mu_avg))

        if abs(kappa) < 1e-9:
            target = BodyMotionTarget(vx=v_cmd, vy=0.0, omega=0.0,
                                      icr_target_body=np.array([np.nan, np.nan]))
        else:
            y_R = 1.0 / kappa
            omega = v_cmd * kappa   # vx = v_cmd, vy = 0, ω = vx/y_R = vx·κ
            target = BodyMotionTarget(
                vx=v_cmd,
                vy=0.0,
                omega=omega,
                icr_target_body=np.array([0.0, y_R]),
            )

        return compute_commands(
            wheels=p.wheel_positions_body(),
            target=target,
            tire_radius=p.tire_radius,
            steer_limit=p.steer_limit,
            delta_lock=None,  # all 4 wheels free → all 4 align with ICR
        )
