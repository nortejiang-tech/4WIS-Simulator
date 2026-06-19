"""Quasi-static steering load analysis for the independent load page.

Models per (speed, requested_angle, wheel) the steady-state forces and torques
needed to *hold* a commanded wheel angle. Designed to expose the equilibrium
wheel angle δ_eq(v) — the angle where the rack force crosses zero — which
shifts with speed once the model includes the speed-dependent bias sources:

    * A1 longitudinal force balance: Fx_each = (Crr·m·g + ½ρ·Cd·A·v²) / 4
      → kingpin torque via scrub_radius (+ caster trail at δ≠0).
    * A2 camber thrust: Fy_camber = Cγ · γ_per_wheel · Fz at α=0.
    * A3 static toe: δ_actual = δ_cmd ± toe_axle (per-wheel mirror).

Without these, the previous model produced τ(δ=0) ≡ 0 by construction so
δ_eq always landed at 0° regardless of physical realism.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from sim4wis.controller.registry import available_strategies, make_strategy
from sim4wis.core.state import DriverInput, VehicleParams, VehicleState
from sim4wis.vehicle.geometry import wheel_rack_force_from_linkage
from sim4wis.vehicle.kingpin import kingpin_torque
from sim4wis.vehicle.load_transfer import vertical_loads
from sim4wis.vehicle.model_core import (
    drive_force_per_wheel,
    low_speed_blend,
    parking_turn_scale,
    quasi_static_wheel_loads,
    steady_state_slip_angles,
    wheel_alignment,
    WheelAlignment,
    WheelForceSet,
    WheelLoads,
)
from sim4wis.vehicle.tire import pacejka_combined_forces

WHEEL_LABELS = ("FL", "FR", "RL", "RR")


def sweep_load_analysis(
    params: VehicleParams,
    *,
    speeds: list[float],
    angles: list[float],
    wheel_index: int = 0,
    mode: str = "single_wheel",
    mu: float = 0.85,
) -> dict[str, Any]:
    """Return flattened load rows for speed/steer-angle sweeps."""
    wheel_index = int(np.clip(wheel_index, 0, 3))
    safe_speeds = [float(np.clip(v, 0.0, max(params.v_max, 0.1))) for v in speeds[:80]]
    safe_angles = [
        float(np.clip(a, -params.steer_limit, params.steer_limit))
        for a in angles[:161]
    ]
    if not safe_speeds:
        safe_speeds = [0.0, 5.0, 15.0]
    if not safe_angles:
        safe_angles = [0.0]

    # Cache the per-sweep invariants once. The actual force kernel is the
    # inline Pacejka path defined below (U3 fix — the old code switched to
    # Pacejka here but then threw the result away and used a linear-with-
    # hard-clip pipeline, so the promised smooth shoulder never reached the UI).
    fz_static = vertical_loads(params, ax=0.0, ay=0.0)
    alignment = wheel_alignment(params)
    tire = None  # legacy positional slot in the per-state cache call below

    rows: list[dict[str, Any]] = []
    for speed in safe_speeds:
        loads = quasi_static_wheel_loads(params, fz_static, speed)
        for angle in safe_angles:
            deltas_cmd = _deltas_for_mode(params, speed, angle, wheel_index, mode)
            rows.extend(_analyze_state(
                params, speed, angle, deltas_cmd, mu,
                tire=tire, loads=loads, alignment=alignment,
            ))

    summary = _summary(rows, wheel_index)
    return {
        "mode": mode,
        "wheel_index": wheel_index,
        "wheel_label": WHEEL_LABELS[wheel_index],
        "rows": rows,
        "summary": summary,
    }


def _deltas_for_mode(
    params: VehicleParams,
    speed: float,
    angle: float,
    wheel_index: int,
    mode: str,
) -> np.ndarray:
    if mode == "single_wheel" or mode not in available_strategies():
        out = np.zeros(4)
        out[wheel_index] = angle
        return out

    steering = 0.0 if params.steer_limit <= 1e-9 else angle / params.steer_limit
    steering = float(np.clip(steering, -1.0, 1.0))
    throttle = 0.0 if params.v_max <= 1e-9 else speed / params.v_max
    throttle = float(np.clip(throttle, -1.0, 1.0))
    state = VehicleState(vx=speed)
    try:
        strategy = make_strategy(mode, params)
        cmd = strategy.compute(DriverInput(throttle=throttle, steering=steering), state)
        return np.asarray(cmd.delta_cmd, dtype=np.float64)
    except Exception:
        out = np.zeros(4)
        out[wheel_index] = angle
        return out


# ---------------------------------------------------------------------------
# W1: steady-state bicycle coupling for per-wheel slip angle.
#
# v0.7.4 and earlier locked the body in pure straight motion, so the slip
# angle of any wheel reduced to α_i = −δ_i and the τ-δ saturation point was
# the same at every speed (only the plateau height varied via aero lift /
# load sensitivity). That's correct台架口径, but physically misleading for
# the 4WIS engineer — a real vehicle develops body sideslip β and yaw rate r
# in response to the corner, and high speed shrinks the per-wheel linear
# region just as the user pointed out.
#
# The solver below treats the 4 wheels as a generalised linear bicycle:
#
#     α_i ≈ β + r·x_i/V − δ_i    (small angle, wheel y-offset ignored as 2nd order)
#     F_y_i = −c_α,eff(F_z_i) · α_i
#     Σ F_y = m·V·r              (centripetal force balance)
#     Σ x_i · F_y_i = 0          (steady yaw — no angular accel)
#
# yielding a 2×2 linear system for (β, r). This makes the wheel slip angle
# grow with V² → linear region shrinks at speed → red "ideal" line steepens
# with speed → δ_eq drifts more with speed. All three are what the user
# called out as missing.
#
# The implementation lives in vehicle.model_core.steady_state_slip_angles so
# time-domain models and quasi-static sweeps can converge on the same low-level
# math without coupling their higher-level responsibilities together.
# ---------------------------------------------------------------------------


def _analyze_state(
    params: VehicleParams,
    speed: float,
    requested_angle: float,
    deltas_cmd: np.ndarray,
    mu: float,
    *,
    tire=None,  # kept for signature stability; the inline Pacejka kernel is used now
    loads: WheelLoads | None = None,
    alignment: WheelAlignment | None = None,
    fz: np.ndarray | None = None,
    toe_offsets: np.ndarray | None = None,
    camber: np.ndarray | None = None,
) -> list[dict[str, Any]]:
    if alignment is None:
        if toe_offsets is not None and camber is not None:
            alignment = WheelAlignment(
                toe_offsets=np.asarray(toe_offsets, dtype=np.float64),
                camber=np.asarray(camber, dtype=np.float64),
            )
        else:
            alignment = wheel_alignment(params)
    if loads is None:
        fz_static = vertical_loads(params, ax=0.0, ay=0.0) if fz is None else fz
        loads = quasi_static_wheel_loads(params, fz_static, speed)

    toe_offsets = alignment.toe_offsets
    camber = alignment.camber
    fz = loads.fz
    c_alpha_eff = loads.c_alpha

    deltas = deltas_cmd + toe_offsets
    fx_drive = drive_force_per_wheel(params, speed)
    c_kappa = float(params.tire_c_kappa)
    # Pacejka shape factors (cached in params; defaults are SUV-grade).
    cx = float(getattr(params, "tire_cx", 1.65))
    cy = float(getattr(params, "tire_cy", 1.30))
    ex = float(getattr(params, "tire_ex", -0.5))
    ey = float(getattr(params, "tire_ey", -1.0))
    alpha_sl = 0.20  # pneumatic-trail decay slip angle [rad]
    t_pn = float(params.tire_t_pneumatic)
    c_gamma = float(params.camber_thrust_coeff)

    fx = np.zeros(4)
    fy = np.zeros(4)
    mz = np.zeros(4)
    alpha = np.zeros(4)

    low_speed_factor = low_speed_blend(params, speed)
    moving_blend = 1.0 - low_speed_factor

    # W1: solve steady-state bicycle coupling for body (β, r) so the slip
    # angles capture the speed dependence the台架 kinematic α = −δ formula
    # misses. Below the blend speed the body is effectively static (parking
    # regime dominates anyway) → fall back to the simple kinematic α so we
    # don't divide by tiny V.
    wheel_positions = params.wheel_positions_body()
    alpha, _body_response = steady_state_slip_angles(
        speed=speed,
        delta=deltas,
        c_alpha=c_alpha_eff,
        wheel_positions_body=wheel_positions,
        mass=float(params.mass),
    )

    for i in range(4):
        # H1 / U3 equivalent-slip absorption ----------------------------------
        # camber thrust Fy_camber = C_γ·γ·Fz at α=0 → equivalent α offset:
        #   alpha_offset = -Fy_camber / c_alpha_eff  (so Pacejka(α_eq) reproduces
        #   the combined slip+camber Fy at small angles and saturates smoothly)
        fy_camber_demand_i = c_gamma * float(camber[i]) * float(fz[i])
        alpha_offset = -fy_camber_demand_i / max(float(c_alpha_eff[i]), 1.0)
        alpha_eq = float(alpha[i]) + alpha_offset
        # driving Fx → equivalent slip ratio
        kappa_eq = float(fx_drive[i]) / max(c_kappa, 1.0)

        # Pacejka pure-slip + friction ellipse (one shot, no two-stage clip).
        fx_i, fy_i = pacejka_combined_forces(
            alpha=alpha_eq,
            kappa=kappa_eq,
            fz=float(fz[i]),
            mu=mu,
            c_alpha=float(c_alpha_eff[i]),
            c_kappa=c_kappa,
            cx=cx,
            cy=cy,
            ex=ex,
            ey=ey,
        )

        # mz uses the actual rolling slip α (not the camber-shifted alpha_eq)
        # with Pacejka's slip-dependent pneumatic trail.
        trail = t_pn * max(0.0, 1.0 - abs(float(alpha[i])) / alpha_sl)
        mz_i = -fy_i * trail

        # Low-speed blend (parking mode handled separately below).
        fx[i] = fx_i * moving_blend
        fy[i] = fy_i * moving_blend
        mz[i] = mz_i * moving_blend

    cap = np.maximum(mu * fz, 1e-9)

    # Parking compensation (after the moving-state friction ellipse, since it
    # represents a quasi-static contact-patch deformation, not a sliding-
    # friction limit).
    fy = fy + _parking_side_force(params, deltas, fz, speed, mu)

    utilization = np.minimum(1.0, np.hypot(fx, fy) / cap)

    # Diagnostic camber Fy / drive Fx for row record (post-blend, pre-parking).
    fy_camber = c_gamma * camber * fz * moving_blend
    fx_drive_diag = fx_drive * moving_blend
    actual_forces = WheelForceSet(fx=fx, fy=fy, mz=mz, frame="wheel")

    tau = kingpin_torque(
        fx=actual_forces.fx,
        fy=actual_forces.fy,
        mz=actual_forces.mz,
        fz=fz,
        suspension=params.suspension,
        delta=deltas,
        tire_radius=params.tire_radius,
        t_pneumatic_extra=t_pn,
    )
    tau = tau + _parking_torque(params, deltas, fz, speed, mu)
    rack, motor, linkage = wheel_rack_force_from_linkage(
        tau,
        deltas,
        params.steering_geometry,
        params.steering_arm_length,
        params.pinion_radius,
        params.rack_mech_efficiency,
        params.motor_gear_ratio,
    )

    # Q1 ideal curves: what τ / F_rack / Fy_body would be if the tyre could
    # deliver the full demanded force without friction-ellipse clipping. This
    # exposes the underlying suspension/chassis transfer function — without
    # this, in the saturated region the user only sees the constant μ·Fz wall
    # and can't read the kingpin geometry's intrinsic trend.
    fy_slip_demand_lin = -c_alpha_eff * alpha
    fy_total_demand = (fy_slip_demand_lin + c_gamma * camber * fz) * moving_blend
    fx_total_demand = fx_drive * moving_blend
    fy_ideal = fy_total_demand + _parking_side_force(params, deltas, fz, speed, mu)
    fx_ideal = fx_total_demand
    mz_ideal = -fy_ideal * t_pn
    ideal_forces = WheelForceSet(fx=fx_ideal, fy=fy_ideal, mz=mz_ideal, frame="wheel")
    tau_ideal = kingpin_torque(
        fx=ideal_forces.fx,
        fy=ideal_forces.fy,
        mz=ideal_forces.mz,
        fz=fz,
        suspension=params.suspension,
        delta=deltas,
        tire_radius=params.tire_radius,
        t_pneumatic_extra=t_pn,
    )
    tau_ideal = tau_ideal + _parking_torque(params, deltas, fz, speed, mu)
    rack_ideal, motor_ideal, _ = wheel_rack_force_from_linkage(
        tau_ideal,
        deltas,
        params.steering_geometry,
        params.steering_arm_length,
        params.pinion_radius,
        params.rack_mech_efficiency,
        params.motor_gear_ratio,
    )

    rows: list[dict[str, Any]] = []
    for i in range(4):
        side_y = math.sin(float(deltas[i])) * float(actual_forces.fx[i]) + math.cos(float(deltas[i])) * float(actual_forces.fy[i])
        side_y_ideal = math.sin(float(deltas[i])) * float(ideal_forces.fx[i]) + math.cos(float(deltas[i])) * float(ideal_forces.fy[i])
        rows.append({
            "speed": speed,
            "requested_angle": requested_angle,
            "wheel_index": i,
            "wheel_label": WHEEL_LABELS[i],
            "delta": float(deltas[i]),
            "delta_cmd": float(deltas_cmd[i]),
            "slip_alpha": float(alpha[i]),
            "fz": float(fz[i]),
            "tire_fx": float(actual_forces.fx[i]),
            "tire_fy": float(actual_forces.fy[i]),
            "tire_mz": float(actual_forces.mz[i]),
            "fx_drive": float(fx_drive_diag[i]),
            "fy_camber": float(fy_camber[i]),
            "torque_steer": float(tau[i]),
            "torque_steer_ideal": float(tau_ideal[i]),
            "rack_force": float(rack[i]),
            "rack_force_ideal": float(rack_ideal[i]),
            "motor_torque": float(motor[i]),
            "motor_torque_ideal": float(motor_ideal[i]),
            "side_force_body_y": float(side_y),
            "side_force_body_y_ideal": float(side_y_ideal),
            "arm_tie_angle": float(linkage["arm_tie_angle"][i]),
            "tie_rack_angle": float(linkage["tie_rack_angle"][i]),
            "geometry_efficiency": float(linkage["efficiency"][i]),
            "rack_travel": float(linkage["rack_travel"][i]),
            "friction_utilization": float(utilization[i]),
        })
    return rows


def _parking_torque(
    params: VehicleParams,
    deltas: np.ndarray,
    fz: np.ndarray,
    speed: float,
    mu: float,
) -> np.ndarray:
    return (
        _parking_torque_coeff(params)
        * mu
        * fz
        * float(params.contact_patch_radius)
        * parking_turn_scale(params, deltas)
        * low_speed_blend(params, speed)
    )


def _parking_side_force(
    params: VehicleParams,
    deltas: np.ndarray,
    fz: np.ndarray,
    speed: float,
    mu: float,
) -> np.ndarray:
    return (
        _parking_lateral_coeff(params)
        * mu
        * fz
        * parking_turn_scale(params, deltas)
        * low_speed_blend(params, speed)
    )


def _parking_lateral_coeff(params: VehicleParams) -> float:
    v = float(params.parking_lateral_coeff)
    return v if v > 0.0 else float(params.parking_scrub_coeff)


def _parking_torque_coeff(params: VehicleParams) -> float:
    v = float(params.parking_torque_coeff)
    return v if v > 0.0 else float(params.parking_scrub_coeff)


def _summary(rows: list[dict[str, Any]], wheel_index: int) -> dict[str, Any]:
    if not rows:
        return {}
    peak_rack = max(rows, key=lambda r: abs(float(r["rack_force"])))
    peak_motor = max(rows, key=lambda r: abs(float(r["motor_torque"])))
    min_eff = min(rows, key=lambda r: float(r["geometry_efficiency"]))
    max_util = max(rows, key=lambda r: float(r["friction_utilization"]))
    warnings: list[str] = []
    if float(min_eff["geometry_efficiency"]) < 0.20:
        warnings.append("低几何效率：存在接近奇异或力臂不足的转角区间")
    if float(max_util["friction_utilization"]) > 0.95:
        warnings.append("轮胎接近附着极限：侧向力/转向阻力可能进入饱和")
    return {
        "peak_abs_rack_force": abs(float(peak_rack["rack_force"])),
        "peak_abs_rack_force_at": _where(peak_rack),
        "peak_abs_motor_torque": abs(float(peak_motor["motor_torque"])),
        "peak_abs_motor_torque_at": _where(peak_motor),
        "min_geometry_efficiency": float(min_eff["geometry_efficiency"]),
        "min_geometry_efficiency_at": _where(min_eff),
        "max_friction_utilization": float(max_util["friction_utilization"]),
        "warnings": warnings,
        "per_speed_equilibrium": _per_speed_equilibrium(rows, wheel_index),
    }


def _per_speed_equilibrium(
    rows: list[dict[str, Any]],
    wheel_index: int,
) -> list[dict[str, Any]]:
    """For the selected wheel: at each swept speed, find δ_cmd where rack force
    crosses zero. Falls back to torque_steer zero crossing if rack has none.

    Returns a list sorted by speed; entries carry both the zero-crossing wheel
    angle (`delta_eq` rad / `delta_eq_deg`) and the residual `rack_at_zero` /
    `torque_at_zero` at δ_cmd=0 so the UI can show "how much motor effort is
    needed to hold straight ahead".
    """
    selected = [r for r in rows if int(r["wheel_index"]) == int(wheel_index)]
    if not selected:
        return []
    by_speed: dict[float, list[dict[str, Any]]] = {}
    for r in selected:
        by_speed.setdefault(float(r["speed"]), []).append(r)
    out: list[dict[str, Any]] = []
    for speed in sorted(by_speed):
        bucket = sorted(by_speed[speed], key=lambda r: float(r["requested_angle"]))
        xs = [float(r["requested_angle"]) for r in bucket]
        rack = [float(r["rack_force"]) for r in bucket]
        torque = [float(r["torque_steer"]) for r in bucket]
        delta_eq = _zero_crossing(xs, rack)
        source = "rack"
        if delta_eq is None:
            delta_eq = _zero_crossing(xs, torque)
            source = "torque" if delta_eq is not None else "none"
        rack_at_zero = _interp_at(xs, rack, 0.0)
        torque_at_zero = _interp_at(xs, torque, 0.0)
        out.append({
            "speed": speed,
            "delta_eq": delta_eq,
            "delta_eq_deg": math.degrees(delta_eq) if delta_eq is not None else None,
            "rack_at_zero": rack_at_zero,
            "torque_at_zero": torque_at_zero,
            "source": source,
            "found": delta_eq is not None,
        })
    return out


def _zero_crossing(xs: list[float], ys: list[float]) -> float | None:
    """Linear-interpolate the |y|-minimising zero crossing of (xs, ys)."""
    if len(xs) < 2:
        return None
    roots: list[float] = []
    for i in range(1, len(xs)):
        y0, y1 = ys[i - 1], ys[i]
        if not (math.isfinite(y0) and math.isfinite(y1)):
            continue
        if abs(y0) < 1e-9:
            roots.append(xs[i - 1])
        if y0 == y1:
            continue
        if (y0 < 0.0 < y1) or (y1 < 0.0 < y0):
            x0, x1 = xs[i - 1], xs[i]
            roots.append(x0 + (0.0 - y0) * (x1 - x0) / (y1 - y0))
    if abs(ys[-1]) < 1e-9:
        roots.append(xs[-1])
    if not roots:
        return None
    return float(min(roots, key=abs))


def _interp_at(xs: list[float], ys: list[float], target: float) -> float | None:
    """Linear-interpolate y at x=target; returns None if out of range."""
    if not xs:
        return None
    if target <= xs[0]:
        return float(ys[0])
    if target >= xs[-1]:
        return float(ys[-1])
    for i in range(1, len(xs)):
        if xs[i - 1] <= target <= xs[i]:
            x0, x1 = xs[i - 1], xs[i]
            if abs(x1 - x0) < 1e-12:
                return float(ys[i])
            t = (target - x0) / (x1 - x0)
            return float(ys[i - 1] + (ys[i] - ys[i - 1]) * t)
    return float(ys[-1])


def _where(row: dict[str, Any]) -> dict[str, Any]:
    return {
        "speed": row["speed"],
        "requested_angle": row["requested_angle"],
        "wheel_label": row["wheel_label"],
        "delta": row["delta"],
    }


# ---------------------------------------------------------------------------
# δ_eq sensitivity sweep (v0.7.3) — for a fixed speed, vary one physics knob
# and report δ_eq(knob) so the user can see how strongly each parameter shifts
# the equilibrium wheel angle.
# ---------------------------------------------------------------------------

import dataclasses as _dc


def _override_param(params: VehicleParams, path: str, value: float) -> VehicleParams:
    parts = path.split(".")
    if len(parts) == 1:
        return _dc.replace(params, **{parts[0]: float(value)})
    if parts[0] == "suspension":
        susp = _dc.replace(params.suspension, **{parts[1]: float(value)})
        return _dc.replace(params, suspension=susp)
    if parts[0] == "steering_geometry":
        geom = _dc.replace(params.steering_geometry, **{parts[1]: float(value)})
        return _dc.replace(params, steering_geometry=geom)
    raise ValueError(f"unsupported sensitivity path: {path}")


def sweep_sensitivity(
    base_params: VehicleParams,
    *,
    vary_param: str,
    values: list[float],
    speed: float,
    angles: list[float],
    wheel_index: int = 0,
    mu: float = 0.85,
) -> dict[str, Any]:
    """For each value of `vary_param`, run a single-speed sweep and return the
    equilibrium δ_eq / rack_at_zero / torque_at_zero. Used by the UI sensitivity
    panel — fast because each per-value sweep is `1 × angles × 4 wheels`."""
    points: list[dict[str, Any]] = []
    safe_speed = float(np.clip(speed, 0.0, max(base_params.v_max, 0.1)))
    safe_angles = [
        float(np.clip(a, -base_params.steer_limit, base_params.steer_limit))
        for a in angles[:161]
    ] or [0.0]
    for raw in values[:64]:
        try:
            params = _override_param(base_params, vary_param, float(raw))
        except (TypeError, ValueError):
            continue
        # Re-run a minimal single-speed sweep.
        sweep = sweep_load_analysis(
            params,
            speeds=[safe_speed],
            angles=safe_angles,
            wheel_index=wheel_index,
            mode="single_wheel",
            mu=mu,
        )
        eq_list = sweep.get("summary", {}).get("per_speed_equilibrium") or []
        eq = eq_list[0] if eq_list else {}
        points.append({
            "value": float(raw),
            "delta_eq": eq.get("delta_eq"),
            "delta_eq_deg": eq.get("delta_eq_deg"),
            "rack_at_zero": eq.get("rack_at_zero"),
            "torque_at_zero": eq.get("torque_at_zero"),
            "found": bool(eq.get("found")),
        })
    return {
        "vary_param": vary_param,
        "speed": safe_speed,
        "wheel_index": int(wheel_index),
        "points": points,
    }
