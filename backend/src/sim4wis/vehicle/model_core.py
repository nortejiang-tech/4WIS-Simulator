"""Shared low-level 4WIS vehicle math.

This module is the narrow physics/kinematics foundation used by the time-domain
models and the quasi-static load-analysis page. It intentionally has no
dependency on the simulator loop, controllers, API routers, or frontend-facing
schemas, so higher-level modules can converge on one math source without
becoming tightly coupled to each other.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from sim4wis.core.state import N_WHEELS, VehicleParams

G_ACCEL = 9.80665
VMIN_SLIP = 0.5  # [m/s] floor for slip-angle/slip-ratio denominators
# Below this wheel spin rate sign(ω) is meaningless for the friction brake.
_BRAKE_SPIN_EPS = 1e-3   # [rad/s]
# Below this wheel ground speed the vehicle counts as parked: a held wheel is
# "parked", not "locked and sliding".
_BRAKE_HOLD_EPS = 0.05   # [m/s]


@dataclass(frozen=True)
class WheelKinematics:
    """Per-wheel kinematics at one instant.

    All arrays are length-4 and use the project wheel order FL, FR, RL, RR.
    ``vx_body`` / ``vy_body`` are wheel-center velocities in body axes.
    ``vx_wheel`` / ``vy_wheel`` are the same velocities in each wheel's rolling
    frame after rotating by steer angle ``delta``.
    """

    vx_body: np.ndarray
    vy_body: np.ndarray
    vx_wheel: np.ndarray
    vy_wheel: np.ndarray
    alpha: np.ndarray
    kappa: np.ndarray


@dataclass(frozen=True)
class WheelAlignment:
    """Static per-wheel alignment terms applied before force calculation."""

    toe_offsets: np.ndarray
    camber: np.ndarray


@dataclass(frozen=True)
class WheelLoads:
    """Per-wheel vertical load and load-derived tyre stiffness."""

    fz_static: np.ndarray
    fz: np.ndarray
    c_alpha: np.ndarray


@dataclass(frozen=True)
class WheelForceSet:
    """Per-wheel forces/moments in a declared frame.

    ``frame`` is informational and should be either ``"wheel"`` or ``"body"``.
    Arrays are length-4 in project wheel order.
    """

    fx: np.ndarray
    fy: np.ndarray
    mz: np.ndarray
    frame: str = "wheel"


@dataclass(frozen=True)
class SteadyStateBody:
    """Linear steady-state body response used by quasi-static load analysis."""

    beta: float
    yaw_rate: float
    used_bicycle: bool


# Single-wheel analysis can be performed under two physically distinct framings:
#
#   "isolated"  — the wheel is treated as a stand-alone unit on a test bench.
#                 The vehicle body is *locked* in straight-line motion at speed
#                 V, so α_i = −δ_i exactly. All wheel-self physics (toe, camber
#                 thrust, drive force, aero lift on Fz, load-sensitive c_α,
#                 friction ellipse, kingpin chain, parking) are still applied;
#                 only the body's response (β, r) is suppressed.
#                 Engineering use: actuator/motor *worst-case* sizing.
#
#   "vehicle"   — the wheel is mounted on a real chassis whose body responds
#                 to the analysed wheel's force. The remaining three wheels
#                 hold δ=0 and provide reaction force; the body develops
#                 sideslip β and yaw rate r in steady-state equilibrium.
#                 α_i = β + r·x_i/V − δ_i then exhibits the well-known
#                 high-speed gain growth.
#                 Engineering use: driving-feel and δ_eq under cornering.
BodyCoupling = str  # "isolated" | "vehicle"


def wheel_center_velocities_body(
    vx: float,
    vy: float,
    yaw_rate: float,
    wheel_positions_body: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Velocity of each wheel center in body axes.

    For wheel position r_i = (x_i, y_i), v_i = v_origin + omega x r_i:
    ``vx_i = vx - yaw_rate*y_i`` and ``vy_i = vy + yaw_rate*x_i``.
    """

    wp = np.asarray(wheel_positions_body, dtype=np.float64).reshape(N_WHEELS, 2)
    vx_body = float(vx) - float(yaw_rate) * wp[:, 1]
    vy_body = float(vy) + float(yaw_rate) * wp[:, 0]
    return vx_body, vy_body


def body_to_wheel_frame(
    vx_body: np.ndarray,
    vy_body: np.ndarray,
    delta: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Rotate body-axis wheel-center velocities into each wheel frame."""

    vx_b = np.asarray(vx_body, dtype=np.float64).reshape(N_WHEELS)
    vy_b = np.asarray(vy_body, dtype=np.float64).reshape(N_WHEELS)
    d = np.asarray(delta, dtype=np.float64).reshape(N_WHEELS)
    c = np.cos(d)
    s = np.sin(d)
    vx_wheel = c * vx_b + s * vy_b
    vy_wheel = -s * vx_b + c * vy_b
    return vx_wheel, vy_wheel


def wheel_slip_kinematics(
    *,
    vx: float,
    vy: float,
    yaw_rate: float,
    wheel_omega: np.ndarray,
    delta: np.ndarray,
    wheel_positions_body: np.ndarray,
    tire_radius: float,
    min_longitudinal_speed: float = VMIN_SLIP,
) -> WheelKinematics:
    """Compute per-wheel slip angle and slip ratio from a planar body state."""

    vx_body, vy_body = wheel_center_velocities_body(
        vx, vy, yaw_rate, wheel_positions_body
    )
    vx_wheel, vy_wheel = body_to_wheel_frame(vx_body, vy_body, delta)
    denom = np.maximum(np.abs(vx_wheel), float(min_longitudinal_speed))
    alpha = np.arctan2(vy_wheel, denom)
    kappa = (
        float(tire_radius) * np.asarray(wheel_omega, dtype=np.float64).reshape(N_WHEELS)
        - vx_wheel
    ) / denom
    return WheelKinematics(
        vx_body=vx_body,
        vy_body=vy_body,
        vx_wheel=vx_wheel,
        vy_wheel=vy_wheel,
        alpha=alpha,
        kappa=kappa,
    )


def rotate_wheel_forces_to_body(
    fx_wheel: np.ndarray,
    fy_wheel: np.ndarray,
    delta: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Rotate wheel-frame tyre forces into body axes."""

    fx_w = np.asarray(fx_wheel, dtype=np.float64).reshape(N_WHEELS)
    fy_w = np.asarray(fy_wheel, dtype=np.float64).reshape(N_WHEELS)
    d = np.asarray(delta, dtype=np.float64).reshape(N_WHEELS)
    c = np.cos(d)
    s = np.sin(d)
    fx_body = c * fx_w - s * fy_w
    fy_body = s * fx_w + c * fy_w
    return fx_body, fy_body


def static_toe_offsets(params: VehicleParams) -> np.ndarray:
    """Per-wheel toe offset applied to delta_cmd -> delta_actual.

    Toe-in is positive at the axle level. With the project's steer convention,
    left wheels receive ``-toe`` and right wheels receive ``+toe``.
    """

    toe_f = float(params.static_toe_front)
    toe_r = float(params.static_toe_rear)
    return np.array([-toe_f, +toe_f, -toe_r, +toe_r], dtype=np.float64)


def camber_per_wheel(params: VehicleParams) -> np.ndarray:
    """Mirror-symmetric per-wheel camber angles."""

    camber_left = float(params.suspension.camber)
    return np.array(
        [camber_left, -camber_left, camber_left, -camber_left],
        dtype=np.float64,
    )


def wheel_alignment(params: VehicleParams) -> WheelAlignment:
    """Return static toe/camber terms as one typed bundle."""

    return WheelAlignment(
        toe_offsets=static_toe_offsets(params),
        camber=camber_per_wheel(params),
    )


def drive_force_per_wheel(params: VehicleParams, speed: float) -> np.ndarray:
    """Longitudinal force each wheel must transmit to hold speed."""

    crr = float(params.rolling_resistance_coeff)
    rho = float(params.air_density)
    cd = float(params.drag_coeff_cd)
    area = float(params.frontal_area)
    fx_total = (
        crr * float(params.mass) * G_ACCEL
        + 0.5 * rho * cd * area * float(speed) * float(speed)
    )
    return np.full(N_WHEELS, fx_total / N_WHEELS, dtype=np.float64)


def body_resistance_force(params: VehicleParams, vx: float) -> float:
    """Aero drag + rolling resistance along body X (signed, opposes motion).

    Same Crr / Cd·A coefficients the quasi-static load page uses in
    ``drive_force_per_wheel`` — this is the time-domain twin of that term so
    the workbench and the load page share one longitudinal-balance story.
    The rolling term uses tanh(v/0.5) as a smooth sign() so the force vanishes
    at standstill instead of chattering.
    """

    crr = float(params.rolling_resistance_coeff)
    rho = float(params.air_density)
    cd = float(params.drag_coeff_cd)
    area = float(params.frontal_area)
    v = float(vx)
    f_roll = crr * float(params.mass) * G_ACCEL * math.tanh(v / 0.5)
    f_aero = 0.5 * rho * cd * area * v * abs(v)
    return -(f_roll + f_aero)


def camber_thrust_alpha_offset(params: VehicleParams, fz: np.ndarray) -> np.ndarray:
    """Equivalent slip-angle offset that absorbs camber thrust into the tyre model.

    Same absorption the load-analysis page uses (H1): camber thrust
    ``Fy_camber = Cγ·γ·Fz`` at α = 0 maps to ``Δα = −Fy_camber / c_α`` so that
    feeding ``α + Δα`` into the tyre model reproduces the combined slip+camber
    force at small angles and saturates smoothly with it. The divisor must be
    the stiffness the tyre model actually runs with — load-sensitive c_α(Fz)
    when ``tire_load_sensitivity_time_domain`` is on, constant otherwise.
    """

    c_gamma = float(params.camber_thrust_coeff)
    camber = camber_per_wheel(params)
    fz_arr = np.asarray(fz, dtype=np.float64).reshape(N_WHEELS)
    if getattr(params, "tire_load_sensitivity_time_domain", False):
        c_alpha = np.maximum(load_sensitive_cornering_stiffness(params, fz_arr), 1.0)
    else:
        c_alpha = max(float(params.tire_c_alpha), 1.0)
    return -(c_gamma * camber * fz_arr) / c_alpha


def fz_with_aero_lift(
    params: VehicleParams,
    fz_static: np.ndarray,
    speed: float,
) -> np.ndarray:
    """Apply per-axle aero lift to static wheel loads."""

    rho = float(params.air_density)
    area = float(params.frontal_area)
    q = 0.5 * rho * area * float(speed) * float(speed)
    lift_f = float(params.aero_lift_coeff_front) * q
    lift_r = float(params.aero_lift_coeff_rear) * q
    fz = np.asarray(fz_static, dtype=np.float64).reshape(N_WHEELS).copy()
    fz[0] = max(float(fz_static[0]) - 0.5 * lift_f, 1e-3)
    fz[1] = max(float(fz_static[1]) - 0.5 * lift_f, 1e-3)
    fz[2] = max(float(fz_static[2]) - 0.5 * lift_r, 1e-3)
    fz[3] = max(float(fz_static[3]) - 0.5 * lift_r, 1e-3)
    return fz


def load_sensitive_cornering_stiffness(
    params: VehicleParams,
    fz: np.ndarray,
) -> np.ndarray:
    """c_alpha(Fz) = c_alpha0 * (Fz/Fz_nom)^p."""

    c_alpha0 = float(params.tire_c_alpha)
    exponent = float(getattr(params, "tire_load_sensitivity_exp", 0.8))
    fz_nom = max(float(params.mass) * G_ACCEL / N_WHEELS, 1.0)
    ratio = np.maximum(np.asarray(fz, dtype=np.float64).reshape(N_WHEELS) / fz_nom, 1e-3)
    return c_alpha0 * np.power(ratio, exponent)


def quasi_static_wheel_loads(
    params: VehicleParams,
    fz_static: np.ndarray,
    speed: float,
) -> WheelLoads:
    """Return static Fz, aero-adjusted Fz, and c_alpha(Fz) as one bundle."""

    fz0 = np.asarray(fz_static, dtype=np.float64).reshape(N_WHEELS)
    fz = fz_with_aero_lift(params, fz0, speed)
    return WheelLoads(
        fz_static=fz0,
        fz=fz,
        c_alpha=load_sensitive_cornering_stiffness(params, fz),
    )


def solve_steady_state_body(
    *,
    speed: float,
    delta: np.ndarray,
    c_alpha: np.ndarray,
    wheel_positions_body: np.ndarray,
    mass: float,
) -> tuple[float, float]:
    """Linear-bicycle (beta, yaw_rate) for a general 4-wheel steer pattern.

    The equations are:

        alpha_i ~= beta + yaw_rate*x_i/V - delta_i
        Fy_i = -c_alpha_i * alpha_i
        sum(Fy_i) = m*V*yaw_rate
        sum(x_i*Fy_i) = 0

    The function returns (0, 0) for singular or non-finite systems so callers
    can gracefully fall back to direct kinematic slip.
    """

    V = max(float(speed), 1e-3)
    d = np.asarray(delta, dtype=np.float64).reshape(N_WHEELS)
    ca = np.asarray(c_alpha, dtype=np.float64).reshape(N_WHEELS)
    wp = np.asarray(wheel_positions_body, dtype=np.float64).reshape(N_WHEELS, 2)
    x = wp[:, 0]

    A = float(np.sum(ca))
    B = float(np.sum(ca * x))
    C = float(np.sum(ca * x * x))
    D = float(np.sum(ca * d))
    E = float(np.sum(ca * x * d))
    mat = np.array([[A, B / V + float(mass) * V], [B, C / V]], dtype=np.float64)
    rhs = np.array([D, E], dtype=np.float64)
    try:
        sol = np.linalg.solve(mat, rhs)
    except np.linalg.LinAlgError:
        return 0.0, 0.0
    if not np.all(np.isfinite(sol)):
        return 0.0, 0.0
    return float(sol[0]), float(sol[1])


def steady_state_slip_angles(
    *,
    speed: float,
    delta: np.ndarray,
    c_alpha: np.ndarray,
    wheel_positions_body: np.ndarray,
    mass: float,
    body_coupling: BodyCoupling = "vehicle",
    bicycle_min_speed: float = 1.0,
    min_longitudinal_speed: float = VMIN_SLIP,
) -> tuple[np.ndarray, SteadyStateBody]:
    """Slip angles for quasi-static sweeps.

    ``body_coupling`` selects the analysis framing — see the BodyCoupling
    docstring above. ``"vehicle"`` (default) solves the steady-state bicycle
    above ``bicycle_min_speed`` and falls back to straight-body kinematics
    below that (numerical safety, not a model choice). ``"isolated"`` *always*
    uses the bench framing α = −δ regardless of speed; the wheel is treated
    as a stand-alone test unit.
    """

    d = np.asarray(delta, dtype=np.float64).reshape(N_WHEELS)
    wp = np.asarray(wheel_positions_body, dtype=np.float64).reshape(N_WHEELS, 2)

    use_vehicle = body_coupling == "vehicle" and float(speed) > float(bicycle_min_speed)

    if use_vehicle:
        beta, yaw_rate = solve_steady_state_body(
            speed=speed,
            delta=d,
            c_alpha=c_alpha,
            wheel_positions_body=wp,
            mass=mass,
        )
        alpha = beta + yaw_rate * wp[:, 0] / max(float(speed), 1e-3) - d
        return alpha, SteadyStateBody(beta=beta, yaw_rate=yaw_rate, used_bicycle=True)

    vx_body = np.full(N_WHEELS, float(speed), dtype=np.float64)
    vy_body = np.zeros(N_WHEELS, dtype=np.float64)
    vx_wheel, vy_wheel = body_to_wheel_frame(vx_body, vy_body, d)
    denom = np.maximum(np.abs(vx_wheel), float(min_longitudinal_speed))
    alpha = np.arctan2(vy_wheel, denom)
    return alpha, SteadyStateBody(beta=0.0, yaw_rate=0.0, used_bicycle=False)


def friction_brake_torques(
    *,
    brake_cmd: np.ndarray,
    brake_filtered: np.ndarray,
    wheel_omega: np.ndarray,
    vx_wheel: np.ndarray,
    fz: np.ndarray,
    mu: np.ndarray,
    params: VehicleParams,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Friction-brake torque per wheel [N·m], opposing wheel spin.

    Shared by the dynamic and multibody models (both drive the same wheel-spin
    ODE, so the brake belongs here next to `semi_implicit_wheel_spin` rather
    than duplicated in each model).

    `brake_cmd[i]` is the wheel's share of the pedal — the pedal fraction times
    its axle's `brake_bias_front` share. `brake_torque_max` is the
    **whole-vehicle** figure (sized against m·g·r), so each wheel gets its axle
    share divided by the two wheels on that axle:

        T_i = brake_filtered_i · brake_torque_max / 2

    Without that /2 the vehicle total came out at 2× the documented budget and
    every wheel locked at ~30% pedal.

    Lockup is deliberate (there is no ABS) and must be *reachable*: the caliper
    applies what the pedal asks, and the contact patch can only react r·μ·Fz.
    When the demand exceeds that, the surplus decelerates the wheel itself —
    ω runs down to zero and the tyre goes to full slip, where the friction
    ellipse eats the lateral force and the car stops steering.

    Clamping the applied torque *to* the capacity instead (the obvious-looking
    guard) quietly makes real lockup impossible: the wheel then always finds an
    equilibrium slip that balances the torque, so it sat at κ ≈ −0.08, rolling,
    while still being reported as locked.

    At a standstill `sign(ω)` carries no information, so the opposing direction
    is taken from the wheel's ground speed instead — otherwise the brake torque
    vanishes exactly when the wheel is fully locked and the car creeps away
    from under a full pedal (or rolls off under the parking brake).

    Returns (torque, locked_flags, brake_filtered_new).
    """
    p = params
    tau = max(float(getattr(p, "brake_tau", 0.08)), 1e-3)
    a = dt / (tau + dt)
    bf = np.asarray(brake_filtered, dtype=np.float64).reshape(N_WHEELS)
    bf = bf + a * (np.asarray(brake_cmd, dtype=np.float64).reshape(N_WHEELS) - bf)

    # Half the axle share: brake_torque_max is a whole-vehicle budget.
    t_demand = bf * float(getattr(p, "brake_torque_max", 12_000.0)) / 2.0
    r = float(p.tire_radius)
    omega = np.asarray(wheel_omega, dtype=np.float64).reshape(N_WHEELS)
    vxw = np.asarray(vx_wheel, dtype=np.float64).reshape(N_WHEELS)
    fz_arr = np.asarray(fz, dtype=np.float64).reshape(N_WHEELS)
    mu_arr = np.asarray(mu, dtype=np.float64).reshape(N_WHEELS)

    torque = np.zeros(N_WHEELS)
    locked = np.zeros(N_WHEELS, dtype=bool)
    for i in range(N_WHEELS):
        cap = r * float(mu_arr[i]) * float(fz_arr[i])   # contact-patch capacity
        # Direction the brake must oppose. Below the spin threshold the wheel
        # carries no sign, so fall back to the direction it would be dragged.
        if abs(float(omega[i])) > _BRAKE_SPIN_EPS:
            sign = math.copysign(1.0, float(omega[i]))
        elif abs(float(vxw[i])) > _BRAKE_HOLD_EPS:
            sign = math.copysign(1.0, float(vxw[i]))
        else:
            sign = 0.0
        demand = float(t_demand[i])
        torque[i] = -sign * demand
        # Locked = the demand outruns the contact patch, so the wheel cannot
        # keep rolling. Reported only while the car is actually moving: a wheel
        # held at rest under the parking brake is parked, not sliding.
        locked[i] = demand >= cap > 0.0 and abs(float(vxw[i])) > _BRAKE_HOLD_EPS
    return torque, locked, bf


def semi_implicit_wheel_spin(
    *,
    dt: float,
    wheel_omega: np.ndarray,
    vx_wheel: np.ndarray,
    alpha: np.ndarray,
    fz: np.ndarray,
    mu: np.ndarray,
    torque: np.ndarray,
    tire,
    tire_radius: float,
    wheel_inertia: float,
    brake_torque: np.ndarray | None = None,
    min_longitudinal_speed: float = VMIN_SLIP,
) -> tuple[np.ndarray, WheelForceSet, np.ndarray]:
    """Advance the wheel-spin ODE one step with a stiff-stable scheme.

    Why not leave ω inside the models' RK4 vector: the linearised wheel-spin
    mode has λ = c_κ·r²/(I_w·v_x,wheel). With the default parameters
    (c_κ = 1e5, r = 0.395, I_w = 2.5) that gives λ·h ≈ 3.1 at 10 m/s and ≈ 62
    at the 0.5 m/s slip floor — beyond RK4's real-axis stability limit (≈2.78).
    The instability is invisible when the equilibrium force is exactly zero
    (pre-v0.9 flat-road cruise) but erupts as sustained κ oscillation the
    moment any steady longitudinal force exists (drag, slope, accel).

    Scheme: backward-Euler on the *linear* slip force (unconditionally stable
    in the stiff regime), switching to forward-Euler when the tyre is
    friction-saturated (∂Fx/∂ω ≈ 0 there, so the ODE is non-stiff and the
    implicit-linear denominator would wrongly suppress wheelspin).

    Returns (omega_new, WheelForceSet at the new slip, kappa_new).
    """

    r = float(tire_radius)
    iw = max(float(wheel_inertia), 1e-6)
    c_kappa = float(getattr(tire, "c_kappa", 100_000.0))
    omega = np.asarray(wheel_omega, dtype=np.float64).reshape(N_WHEELS)
    vxw = np.asarray(vx_wheel, dtype=np.float64).reshape(N_WHEELS)
    al = np.asarray(alpha, dtype=np.float64).reshape(N_WHEELS)
    fz_arr = np.asarray(fz, dtype=np.float64).reshape(N_WHEELS)
    mu_arr = np.asarray(mu, dtype=np.float64).reshape(N_WHEELS)
    tq = np.asarray(torque, dtype=np.float64).reshape(N_WHEELS)
    tb = (np.zeros(N_WHEELS) if brake_torque is None
          else np.asarray(brake_torque, dtype=np.float64).reshape(N_WHEELS))

    omega_new = np.empty(N_WHEELS)
    fx_new = np.empty(N_WHEELS)
    fy_new = np.empty(N_WHEELS)
    mz_new = np.empty(N_WHEELS)
    kappa_new = np.empty(N_WHEELS)

    for i in range(N_WHEELS):
        d = max(abs(float(vxw[i])), float(min_longitudinal_speed))
        kappa_n = (r * float(omega[i]) - float(vxw[i])) / d
        fx_n, fy_n, _mz_n = tire.forces(float(al[i]), kappa_n, float(fz_arr[i]), float(mu_arr[i]))
        cap = float(mu_arr[i]) * float(fz_arr[i])
        saturated = cap > 1e-6 and math.hypot(fx_n, fy_n) >= 0.98 * cap
        if saturated:
            w = float(omega[i]) + dt / iw * (float(tq[i]) - r * fx_n)
        else:
            w = (iw * float(omega[i]) + dt * (float(tq[i]) + r * c_kappa * float(vxw[i]) / d)) / (
                iw + dt * r * r * c_kappa / d
            )
        # Brake zero-crossing clamp. A friction brake can bring a wheel to rest
        # but cannot spin it up backwards — it only ever opposes motion. Without
        # this the brake makes κ chatter sign→sign across zero every step, and
        # once the wheel is held at rest the torque would push it into reverse.
        # Scoped to the brake torque specifically: a *motor* is allowed to drive
        # a wheel through zero (that is how reversing works).
        if float(tb[i]) != 0.0:
            if float(omega[i]) * w < 0.0:
                w = 0.0                       # overshot through zero this step
            elif abs(float(omega[i])) <= _BRAKE_SPIN_EPS and w * float(tb[i]) > 0.0:
                # Already at rest and the brake itself is what would move it.
                w = 0.0
        k_new = (r * w - float(vxw[i])) / d
        fx_i, fy_i, mz_i = tire.forces(float(al[i]), k_new, float(fz_arr[i]), float(mu_arr[i]))
        omega_new[i] = w
        kappa_new[i] = k_new
        fx_new[i] = fx_i
        fy_new[i] = fy_i
        mz_new[i] = mz_i

    return omega_new, WheelForceSet(fx=fx_new, fy=fy_new, mz=mz_new, frame="wheel"), kappa_new


def low_speed_blend(params: VehicleParams, speed: float) -> float:
    """Parking/rolling blend weight; 1 at standstill, decays with speed."""

    blend_v = max(float(params.low_speed_blend_ms), 0.05)
    return math.exp(-((abs(float(speed)) / blend_v) ** 2))


def parking_turn_scale(params: VehicleParams, delta: np.ndarray) -> np.ndarray:
    """Smooth steering-angle saturation for parking contact-patch effects."""

    sat = math.radians(max(float(params.static_tire_deflection_deg), 0.5))
    return np.tanh(np.asarray(delta, dtype=np.float64).reshape(N_WHEELS) / sat)
