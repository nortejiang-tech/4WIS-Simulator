"""Shared helpers for rear-wheel-steering (RWS) strategy family.

All four RWS strategies (speed-scheduled ratio, steady+transient feed-forward,
yaw-rate feedback, model-following / zero-sideslip) reduce to:

    1. pick a front wheel-equivalent angle δ_front from the driver,
    2. pick a rear wheel-equivalent angle δ_rear from the control law,
    3. place the ICR at the intersection of the two axle perpendiculars and let
       the unified `compute_commands` helper derive the four wheel angles+speeds.

`command_from_axle_angles` does step 3 (extracted from the original
`rear_steer.py`). `zero_sideslip_ratio` is the analytic steady-state rear/front
ratio that yields zero body sideslip for the linear bicycle model — used both as
the default schedule for the speed-scheduled strategy and as the feed-forward
term for the model-following strategy.
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import BodyMotionTarget, compute_commands
from sim4wis.core.state import ControlCommand, VehicleParams


def axle_cornering_stiffness(p: VehicleParams) -> tuple[float, float]:
    """Per-axle cornering stiffness (Cf, Cr) in N/rad.

    The params carry a single per-tyre linear cornering stiffness
    (`tire_c_alpha`); an axle has two tyres.
    """
    c_axle = 2.0 * float(p.tire_c_alpha)
    return c_axle, c_axle


def zero_sideslip_ratio(p: VehicleParams, u: float) -> float:
    """Rear/front steer ratio k = δ_rear/δ_front giving steady-state β = 0.

    Derived from the linear bicycle model steady-state equations:

        k(u) = (−b + a·m·u²/(Cr·L)) / (a + b·m·u²/(Cf·L))

    where a = CG→front, b = CG→rear, L = wheelbase, m = mass.

    Low speed → k < 0 (counter-phase, tighter turns); high speed → k > 0
    (in-phase, stability). Crossover u² = b·Cr·L/(a·m).
    """
    L = float(p.wheelbase)
    a = float(p.cg_to_front)
    b = L - a
    m = float(p.mass)
    cf, cr = axle_cornering_stiffness(p)
    num = -b + (a * m * u * u) / (cr * L)
    den = a + (b * m * u * u) / (cf * L)
    if abs(den) < 1e-9:
        return 0.0
    return float(num / den)


def reference_yaw_rate(p: VehicleParams, u: float, delta_f: float) -> float:
    """Steady-state yaw rate of the zero-sideslip bicycle reference for δ_front.

        r = δ_front / (a/u + m·b·u/(Cf·L))

    Returns 0 at standstill (u≈0).
    """
    if abs(u) < 1e-3:
        return 0.0
    L = float(p.wheelbase)
    a = float(p.cg_to_front)
    b = L - a
    m = float(p.mass)
    cf, _cr = axle_cornering_stiffness(p)
    denom = a / u + (m * b * u) / (cf * L)
    if abs(denom) < 1e-9:
        return 0.0
    return float(delta_f / denom)


def command_from_axle_angles(
    p: VehicleParams,
    delta_f: float,
    delta_r: float,
    v_cmd: float,
) -> ControlCommand:
    """Build a ControlCommand from front/rear axle-equivalent steer angles.

    Places the ICR at the intersection of the front and rear axle
    perpendiculars (front midpoint at +L/2, rear at -L/2) and derives the four
    wheel angles + speeds via `compute_commands`.
    """
    L = float(p.wheelbase)
    delta_f = float(np.clip(delta_f, -p.steer_limit, p.steer_limit))
    delta_r = float(np.clip(delta_r, -p.steer_limit, p.steer_limit))

    if abs(delta_f) < 1e-9 and abs(delta_r) < 1e-9:
        target = BodyMotionTarget(
            vx=v_cmd, vy=0.0, omega=0.0,
            icr_target_body=np.array([np.nan, np.nan]),
        )
    elif abs(delta_f - delta_r) < 1e-9:
        # Parallel front/rear (in-phase 1:1) → pure translation, ICR at ∞.
        target = BodyMotionTarget(
            vx=v_cmd * np.cos(delta_f),
            vy=v_cmd * np.sin(delta_f),
            omega=0.0,
            icr_target_body=np.array([np.nan, np.nan]),
        )
    else:
        sf, cf = np.sin(delta_f), np.cos(delta_f)
        sr, cr = np.sin(delta_r), np.cos(delta_r)
        # front: (+L/2, 0) + t·(-sf, cf);  rear: (-L/2, 0) + s·(-sr, cr)
        A = np.array([[-sf, +sr], [cf, -cr]])
        rhs = np.array([-L, 0.0])
        t, _s = np.linalg.solve(A, rhs)
        icr = np.array([+L / 2.0 + t * (-sf), 0.0 + t * cf])
        omega = v_cmd / icr[1] if abs(icr[1]) > 1e-9 else 0.0
        target = BodyMotionTarget(
            vx=v_cmd,
            vy=-omega * icr[0],
            omega=omega,
            icr_target_body=icr,
        )

    return compute_commands(
        wheels=p.wheel_positions_body(),
        target=target,
        tire_radius=p.tire_radius,
        steer_limit=p.steer_limit,
        delta_lock=None,
    )
