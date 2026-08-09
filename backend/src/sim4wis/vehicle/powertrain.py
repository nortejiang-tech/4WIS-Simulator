"""Drivetrain — pedal demand → per-wheel drive torque.

Four layers, applied in this order because each is a different piece of
hardware and they bind in different regimes:

    1. Demand      throttle → a total torque request
    2. Distribution front/rear split by `drive_front_ratio`
    3. Differential how an axle's share reaches its two wheels
    4. Limits      per-wheel motor torque, then the shared power bus

Only the *torque* longitudinal mode routes through here; `speed_servo` (the
default) keeps its wheel-speed servos and is deliberately left alone so every
existing baseline stays put. That means the powertrain envelope below does not
apply in speed-servo mode — that path is a validation/convenience control law,
not a drivetrain model.

Why the ordering matters
------------------------
The torque cap is per-motor hardware; the power cap is a shared bus. Applying
the power cap first would let a single wheel exceed its motor's rating as long
as the others were idle. Applying the torque cap first, then scaling the whole
set to fit the bus, is what the real hardware does.

This ordering also makes `drive_front_ratio` behave correctly in both regimes
without any special-casing:

  * **At launch** (ω → 0) the *torque* cap binds. A 100%-front split can only
    use the two front motors, so it genuinely has half the total torque of a
    50/50 split — which is the honest difference between a front-drive and an
    all-wheel-drive car, not an artefact.
  * **At speed** the *power* cap binds. Total torque is then P_max/ω whatever
    the split is, so sweeping `drive_front_ratio` isolates the effect of
    distribution alone.

What must emerge from this (not be coded in)
--------------------------------------------
Nothing here mentions understeer, oversteer or torque steer. Those follow from
putting Fx through particular contact patches and letting the rest of the model
respond:

  * Longitudinal load transfer (`load_transfer.vertical_loads`) unloads the
    front axle under acceleration, so a front-drive car's own traction limit
    fights it — hence the closed-form asymmetry between the FWD and RWD launch
    limits (see `docs`/tests).
  * The friction circle is shared between Fx and Fy, so whichever axle is
    driven loses lateral capability first: front → understeer on power, rear →
    oversteer.
  * `kingpin.kingpin_torque_terms` already turns Fx into a kingpin moment
    through the scrub radius (`m_fx = Fx · scrub`), so asymmetric front drive
    produces torque steer with no extra wiring.
"""

from __future__ import annotations

import numpy as np

from sim4wis.core.state import DriverInput, N_WHEELS, VehicleParams, VehicleState

FRONT = (0, 1)
REAR = (2, 3)
_OMEGA_EPS = 0.5   # [rad/s] floor for the P = T·ω division


def axle_split(params: VehicleParams) -> tuple[float, float]:
    """(front, rear) fractions of the total drive demand.

    `drive_front_ratio`: 1.0 = pure front drive, 0.0 = pure rear, 0.5 = an even
    all-wheel split. Continuous in between — the FWD/RWD/AWD presets are just
    three points on this axis.
    """
    lam = float(np.clip(getattr(params, "drive_front_ratio", 0.5), 0.0, 1.0))
    return lam, 1.0 - lam


def drive_torques(
    params: VehicleParams,
    driver: DriverInput,
    state: VehicleState,
) -> np.ndarray:
    """Pedal + gear → signed per-wheel drive torque [N·m]."""
    gear = int(driver.gear)
    # The brake owns the wheel while it is applied — same authority order the
    # speed path uses.
    if gear == 0 or float(driver.brake) > 1e-3 or driver.handbrake:
        return np.zeros(N_WHEELS)

    t_motor_max = float(getattr(params, "motor_torque_max", 3000.0))

    # A commanded speed overrides the pedal. Experiments (and cruise control)
    # state a speed, not a pedal position — without this the drivetrain
    # happily ignored an experiment's whole speed profile and ran at whatever
    # the pedal echo happened to be, which silently invalidates any run made
    # in torque mode.
    target = driver.mode_params.get("speed_target_ms") if driver.mode_params else None
    if target is not None:
        demand = _speed_tracking_demand(params, float(target), float(state.vx),
                                        t_motor_max)
        if demand <= 0.0:
            return np.zeros(N_WHEELS)
    else:
        demand = float(np.clip(driver.throttle, 0.0, 1.0))
        if demand <= 0.0:
            return np.zeros(N_WHEELS)

    # ── 1-2. demand → axles ──────────────────────────────────────────────
    # The reference is all four motors at full torque, so a 50/50 split at full
    # pedal asks exactly `motor_torque_max` of each wheel.
    t_total = demand * N_WHEELS * t_motor_max
    f_frac, r_frac = axle_split(params)
    per_wheel = np.array([
        f_frac * t_total / 2.0, f_frac * t_total / 2.0,
        r_frac * t_total / 2.0, r_frac * t_total / 2.0,
    ])

    # ── 3. differential ──────────────────────────────────────────────────
    if str(getattr(params, "diff_type", "independent")) == "open":
        per_wheel = _open_differential(params, state, per_wheel)

    # ── 4a. per-wheel motor torque cap (hardware, per motor) ─────────────
    per_wheel = np.minimum(per_wheel, t_motor_max)

    # ── 4b. shared power bus (P = Σ|T·ω|) ────────────────────────────────
    p_max = float(getattr(params, "motor_power_max", 0.0))
    if p_max > 0.0:
        omega = np.abs(np.asarray(state.wheel_omega, dtype=np.float64).reshape(N_WHEELS))
        p_demand = float(np.sum(per_wheel * np.maximum(omega, _OMEGA_EPS)))
        if p_demand > p_max:
            per_wheel = per_wheel * (p_max / p_demand)

    # ── 5. speed ceilings ────────────────────────────────────────────────
    # Torque mode has no speed target to rescale, so a ceiling becomes a taper:
    # drive torque fades out over the last stretch below it rather than being
    # cut at it, which would make the car surge and hunt at the threshold.
    #
    # Two ceilings apply, whichever is lower:
    #   * the driver speed limit (a driving aid, forward or reverse)
    #   * `v_max_reverse` in R — a real reverse gear is geared down, and the
    #     speed path has always enforced it. Leaving it out here let the car
    #     reverse at 62 m/s in torque mode while the same car was capped at 5
    #     in speed-servo mode.
    ceilings = []
    limit = float(getattr(params, "driver_speed_limit", 0.0))
    if limit > 0.0:
        ceilings.append(limit)
    if gear < 0:
        ceilings.append(float(getattr(params, "v_max_reverse", 5.0)))
    if ceilings:
        ceiling = min(ceilings)
        band = max(0.1 * ceiling, 1.0)               # taper width [m/s]
        over = (abs(float(state.vx)) - (ceiling - band)) / band
        per_wheel = per_wheel * float(np.clip(1.0 - over, 0.0, 1.0))

    gear_sign = -1.0 if gear < 0 else 1.0
    return gear_sign * per_wheel


def _speed_tracking_demand(params: VehicleParams, v_target: float,
                           v_actual: float, t_motor_max: float) -> float:
    """Speed error → normalised drive demand ∈ [0, 1]. Stateless.

    Torque mode has no speed loop of its own, so a commanded speed needs one.
    Rather than a PI (which would need an integrator living somewhere awkward
    and would wind up against the traction limit), this is a proportional law
    on the speed error plus a feed-forward of the resistance the vehicle is
    already fighting:

        F_needed = m·Kp·(v_target − v)  +  |F_resist(v)|

    The feed-forward is what removes the steady-state error a bare P law would
    have, so nothing has to integrate. `body_resistance_force` is the same drag
    + rolling term the model itself applies, so the two agree by construction.

    Returns 0 when the vehicle is already at or above target — coasting down is
    the brake's job, not the drivetrain's.
    """
    from sim4wis.vehicle.model_core import body_resistance_force

    err = abs(v_target) - abs(v_actual)
    if err <= 0.0:
        return 0.0
    kp = 1.5                                        # [1/s] closes ~1 m/s per 0.7 s
    f_needed = float(params.mass) * kp * err + abs(
        body_resistance_force(params, float(v_actual)))
    # Convert to the same normalised demand the pedal produces: full demand is
    # all four motors at their torque ceiling.
    f_full = N_WHEELS * t_motor_max / max(float(params.tire_radius), 1e-6)
    return float(np.clip(f_needed / max(f_full, 1e-6), 0.0, 1.0))


def _open_differential(
    params: VehicleParams,
    state: VehicleState,
    per_wheel: np.ndarray,
) -> np.ndarray:
    """Equal torque both sides of an axle, limited by the weaker contact patch.

    An open differential cannot hold a torque difference across it, so both
    wheels always receive the same torque — and the torque the axle can react
    is set by whichever side has less grip. Put one wheel on ice and the whole
    axle is limited to what the ice can take, which is the failure mode this
    option exists to demonstrate.

    Implemented quasi-statically from last step's friction capacity
    (`state.grip_capacity` = μ·Fz), turned into a torque through the tyre
    radius. That is a one-step lag on a quantity that changes slowly compared
    with the drivetrain, and it avoids needing a full driveline-inertia model
    to express what is really a kinematic constraint of the gearset.

    `independent` (the default, and what the platform did before this existed)
    is the 4WID case: four motors, no mechanical coupling, each wheel free to
    take its own torque.
    """
    out = np.array(per_wheel, dtype=np.float64)
    r = float(params.tire_radius)
    cap = np.asarray(state.grip_capacity, dtype=np.float64).reshape(N_WHEELS)
    if not bool(getattr(state, "grip_valid", False)) or not np.any(cap > 0.0):
        # No tyre data yet (first step, or a model without tyres): fall back to
        # equal split with no traction limit rather than inventing one.
        return out
    for axle in (FRONT, REAR):
        i, j = axle
        weakest = r * float(min(cap[i], cap[j]))
        shared = min(float(out[i]), float(out[j]), weakest)
        out[i] = out[j] = shared
    return out
