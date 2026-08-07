"""Driver-input steering feel — the three-layer mapping real cars have.

Before work-package B, every strategy mapped the normalised steering input
linearly to curvature (or to a wheel angle), with no notion of:

  * how far the driver's wheel is actually turned (θ_sw),
  * the steering gear ratio — and that real gears are *variable* with speed,
  * a lateral-acceleration soft-limit so a low-μ surface can't be commanded
    into an impossible yaw.

The result (see docs/driving_experience_plan.md §1.2) was that at 60 km/h a
mere ±12° of a 270° wheel saturated the tyres — the usable travel collapsed
to the first 5% and everything after was dead-zone.

This module turns the normalised input into an effective **front-axle steer
angle δ_eff [rad]** through three layers, shared by every strategy that
works in curvature / front-angle space:

    Layer 1  steering ∈ [-1,1] → θ_sw (wheel angle, deg)
    Layer 2  θ_sw → δ_raw via a speed-dependent gear ratio i(v)
    Layer 3  δ_raw → δ_eff via a μ-aware lateral-acc soft limit

Strategies that consume a front-wheel angle (ideal_ackermann, rear_wheel_steer)
call `front_steer_angle()` and convert the result to curvature themselves.
Strategies with a different input semantics (crab = crab angle, zero_radius =
sign only) deliberately bypass this layer — see the call sites.
"""

from __future__ import annotations

import math

from sim4wis.core.state import VehicleParams

# Below this speed the soft-limit (Layer 3) is inactive: at parking speed the
# geometric κ_max already bounds the turn and we want full lock available.
_V_SOFT_MIN = 2.0   # m/s


# Default high/low ratio spread. i_high = _RATIO_SPREAD · i_low keeps the
# motorway feel proportional to the parking feel on any hardware.
_RATIO_SPREAD = 3.5


def low_speed_gear_ratio(params: VehicleParams) -> float:
    """i_low — the parking ratio: full wheel travel = full steering lock.

    This MUST be derived from `steer_wheel_range`, not fixed. A ratio picked
    for one wheel is wrong on another: i_low=4 gives a 270° wheel ±33.75° at
    the road wheel (≈ the 35° geometric limit — correct), but the *default*
    range is 540°, where the same ratio asks for ±67.5° — nearly twice the
    mechanical limit. Full lock then arrived at 54% of travel and the last
    46% was dead, so low-speed gain was ~1.85× the pre-feel-layer car rather
    than matching it.

    Setting `steer_ratio_low` > 0 overrides this with an explicit ratio.
    """
    explicit = float(getattr(params, "steer_ratio_low", 0.0))
    if explicit > 0.0:
        return explicit
    sw_half = float(getattr(params, "steer_wheel_range", 540.0)) / 2.0
    limit_deg = math.degrees(float(params.steer_limit))
    return sw_half / max(limit_deg, 1e-6)


def gear_ratio(params: VehicleParams, v: float) -> float:
    """Speed-dependent steering gear ratio i(v) = θ_sw / δ_front.

    Smooth interpolation between the low-speed ratio (tight parking, full lock
    reachable at full travel) and a high-speed ratio (stable, less twitchy).
    Both scale with the hardware's lock-to-lock range, so switching between a
    270° and a 540° wheel preserves the feel instead of halving the gain.
    """
    i_low = low_speed_gear_ratio(params)
    i_high = float(getattr(params, "steer_ratio_high", 0.0))
    if i_high <= 0.0:
        i_high = _RATIO_SPREAD * i_low
    v_ref = float(getattr(params, "steer_ratio_v_ref", 22.0))
    v2 = float(v) * float(v)
    return i_low + (i_high - i_low) * v2 / (v2 + v_ref * v_ref)


def front_steer_angle(params: VehicleParams, steering: float, v: float,
                      mu_avg: float = 0.85) -> float:
    """Normalised steering → effective front-axle steer angle [rad].

    Parameters
    ----------
    params    vehicle params (steer_wheel_range, gear-ratio knobs, wheelbase)
    steering  normalised driver input ∈ [-1, +1] (+ = left)
    v         current longitudinal speed [m/s] (state.vx)
    mu_avg    average surface μ across wheels (default dry asphalt)

    Returns δ_eff in radians, clamped to ±steer_limit. The mapping is odd-symmetric
    so sign is preserved.
    """
    sw_range = float(getattr(params, "steer_wheel_range", 540.0))
    i = gear_ratio(params, v)
    # Layer 1+2: steering → wheel angle (deg) → front angle (deg) → rad.
    theta_sw_deg = float(steering) * (sw_range / 2.0)
    delta_raw_deg = theta_sw_deg / max(i, 1e-6)
    delta_raw = math.radians(delta_raw_deg)
    # What full travel asks for at this speed — the reference the soft limit
    # normalises against so full lock stays reachable.
    delta_raw_max = math.radians((sw_range / 2.0) / max(i, 1e-6))

    # Layer 3: lateral-acceleration soft limit. Shape δ so the commanded
    # steady-state a_y ≈ v²·tan(δ)/L stays under steer_ay_ref_frac·μ·g. At low
    # speed the reference is huge (inactive); at high speed or on ice it pulls
    # δ_eff back so a full-lock input can't demand more grip than the road has.
    #
    # This is a *soft* limit — a smooth saturation, not a clamp. A hard clamp
    # solves the a_y problem while recreating the exact defect this layer was
    # built to remove: everything past the clamp is dead travel. Measured with
    # the clamp, usable travel was 54% at 10 km/h, 14% at 60 and 6.8% at 100 —
    # a worse dead-zone than the pre-layer car it replaced.
    #
    #   δ_eff = δ_allow · tanh(δ_raw / δ_allow) · g(s),   s = δ_raw_max/δ_allow
    #   g(s)  = min(s, 1) / tanh(s)
    #
    # tanh gives the monotonic compression (every further degree of wheel still
    # does something, all the way to the stop). g(s) normalises it so full
    # travel lands exactly on min(δ_raw_max, δ_allow):
    #
    #   * grip abundant (s < 1, parking): g = s/tanh(s), full travel reaches
    #     the geometric lock — a bare tanh would asymptote and cost ~10% of
    #     lock, pushing the minimum turning circle from 3.0 m to 3.4 m.
    #   * grip binding (s > 1, speed or ice): g = 1/tanh(s) → 1, full travel
    #     lands on the grip-limited angle.
    #
    # g is continuous through s = 1 and stays in [1, 1.31], so on-centre gain
    # is never reduced — small corrections keep working at motorway speed.
    frac = float(getattr(params, "steer_ay_ref_frac", 0.9))
    ay_ref = frac * float(mu_avg) * 9.81
    L = max(float(params.wheelbase), 1e-6)
    v_eff = max(abs(float(v)), _V_SOFT_MIN)
    # a_y = v²·tan(δ)/L  ⇒  tan(δ_allow) = ay_ref·L / v²
    tan_allow = ay_ref * L / (v_eff * v_eff)
    delta_allow = math.atan(tan_allow) if tan_allow > 0.0 else float(params.steer_limit)
    if delta_allow > 1e-9 and delta_raw_max > 1e-9:
        s = delta_raw_max / delta_allow
        g = min(s, 1.0) / math.tanh(s)
        delta_eff = delta_allow * math.tanh(delta_raw / delta_allow) * g
    else:
        delta_eff = delta_raw
    # never exceed the hard geometric limit either
    return max(-float(params.steer_limit), min(float(params.steer_limit), delta_eff))


def front_angle_to_curvature(delta_eff: float, wheelbase: float) -> float:
    """Bicycle-model conversion: front-axle angle → path curvature [1/m].

    κ = tan(δ)/L — valid only for a front-steer vehicle whose ICR sits on the
    rear axle. Strategies with a different ICR convention must NOT use this:
    `ideal_ackermann` steers both axles symmetrically (ICR on the lateral axis
    through the vehicle centre, lever arm L/2), so it converts with its own
    `curvature_from_inner_front_angle` instead.
    """
    return math.tan(delta_eff) / max(float(wheelbase), 1e-6)
