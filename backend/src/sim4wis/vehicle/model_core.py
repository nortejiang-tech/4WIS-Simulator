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
    bicycle_min_speed: float = 1.0,
    min_longitudinal_speed: float = VMIN_SLIP,
) -> tuple[np.ndarray, SteadyStateBody]:
    """Slip angles for quasi-static sweeps.

    Above ``bicycle_min_speed`` this uses the steady-state body response. At
    parking/creep speed it falls back to straight-body kinematics, matching the
    time-domain slip convention without dividing by a tiny speed.
    """

    d = np.asarray(delta, dtype=np.float64).reshape(N_WHEELS)
    wp = np.asarray(wheel_positions_body, dtype=np.float64).reshape(N_WHEELS, 2)
    if float(speed) > float(bicycle_min_speed):
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


def low_speed_blend(params: VehicleParams, speed: float) -> float:
    """Parking/rolling blend weight; 1 at standstill, decays with speed."""

    blend_v = max(float(params.low_speed_blend_ms), 0.05)
    return math.exp(-((abs(float(speed)) / blend_v) ** 2))


def parking_turn_scale(params: VehicleParams, delta: np.ndarray) -> np.ndarray:
    """Smooth steering-angle saturation for parking contact-patch effects."""

    sat = math.radians(max(float(params.static_tire_deflection_deg), 0.5))
    return np.tanh(np.asarray(delta, dtype=np.float64).reshape(N_WHEELS) / sat)
