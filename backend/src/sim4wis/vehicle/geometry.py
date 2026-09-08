"""Steering geometry helpers — pure functions, no state.

These helpers underpin both the controllers (which compute commanded δ) and
the model (which computes the actual vehicle ICR from velocities), so they
live in `vehicle/` rather than under a single strategy.

Conventions match docs/design.md §2:
    * Body frame: X forward, Y left, origin at midpoint of axles.
    * δ ∈ [-π/2, +π/2]: angle of wheel rolling axis vs. body X, CCW positive.
    * ICR position in body frame: (x_R, y_R). For pure heading-aligned forward
      motion with curvature κ (signed, κ>0 = left turn), ICR is at (0, 1/κ).
"""

from __future__ import annotations

import numpy as np

from sim4wis.core.state import SteeringGeometryParams


def wrap_to_pmpi(angle: float | np.ndarray) -> float | np.ndarray:
    """Wrap angle(s) to [-π, +π]."""
    return (np.asarray(angle) + np.pi) % (2.0 * np.pi) - np.pi


def wrap_to_pmhalfpi(angle: float | np.ndarray) -> float | np.ndarray:
    """Wrap angle(s) to [-π/2, +π/2].

    Wheel orientation is rotationally symmetric by 180° (the wheel rolls
    either forward or backward along its X axis). The "forward" convention
    keeps δ in this half-range so that small steering commands stay small.
    """
    a = wrap_to_pmpi(angle)
    a = np.where(a > np.pi / 2.0, a - np.pi, a)
    a = np.where(a < -np.pi / 2.0, a + np.pi, a)
    # If input was scalar, return scalar.
    if np.isscalar(angle):
        return float(a)
    return a


def steer_angle_for_icr(wheel_pos_body: np.ndarray, icr_body: np.ndarray) -> np.ndarray:
    """For each wheel position, compute the steer angle whose perpendicular
    passes through the given ICR point.

    Args:
        wheel_pos_body: (N, 2) array of wheel positions in body frame.
        icr_body: (2,) ICR position in body frame.

    Returns:
        (N,) array of steer angles δ in [-π/2, +π/2].

    Math:
        Wheel rolling direction must be perpendicular to the radius vector
        r = wheel - ICR. With body frame X-forward / Y-left, choose the
        perpendicular pointing in the +CCW-rotation tangent direction:

            δ = atan2(x_w - x_R, y_R - y_w)         (raw, in (-π, π])

        then wrap to [-π/2, π/2] (since the wheel can roll either way along
        its rolling axis; the sign of wheel ω handles forward vs backward).
    """
    p = np.asarray(wheel_pos_body, dtype=np.float64).reshape(-1, 2)
    xR, yR = float(icr_body[0]), float(icr_body[1])
    dx = p[:, 0] - xR
    dy_neg = yR - p[:, 1]
    raw = np.arctan2(dx, dy_neg)
    return wrap_to_pmhalfpi(raw)


def vehicle_icr_from_velocity(
    vx: float, vy: float, yaw_rate: float, eps: float = 1e-6
) -> np.ndarray:
    """Compute the instantaneous centre of rotation in body frame from the
    body-origin velocity (vx, vy) and yaw rate ω.

    Derivation: a point (x_R, y_R) in body frame has body-frame velocity
        v_point = v_origin + ω × r_point
        with ω = ω·ẑ, r = (x_R, y_R, 0):
        ω × r = (-ω·y_R, ω·x_R)
    Setting v_point = 0 gives:
        vx - ω·y_R = 0  →  y_R = +vx / ω
        vy + ω·x_R = 0  →  x_R = -vy / ω
    (left turn ω>0 with vx>0 puts the ICR on the left, +Y — matches the
    strategies' icr_target_body convention).

    For |ω| < eps the ICR is at infinity → return (NaN, NaN).
    """
    if abs(yaw_rate) < eps:
        return np.array([np.nan, np.nan])
    return np.array([-vy / yaw_rate, vx / yaw_rate])


def line_intersection(
    p1: np.ndarray, dir1: np.ndarray, p2: np.ndarray, dir2: np.ndarray
) -> np.ndarray:
    """Intersect two lines in 2D, each given by a point and a direction vector.

    Returns the intersection point, or (NaN, NaN) if the lines are parallel.
    Used by tests to verify that wheel perpendicular lines meet at one ICR.
    """
    # Solve p1 + t·dir1 = p2 + s·dir2 → t·dir1 - s·dir2 = p2 - p1
    A = np.column_stack([dir1, -dir2])
    b = p2 - p1
    det = A[0, 0] * A[1, 1] - A[0, 1] * A[1, 0]
    if abs(det) < 1e-12:
        return np.array([np.nan, np.nan])
    t = (b[0] * A[1, 1] - b[1] * A[0, 1]) / det
    return p1 + t * dir1


def steer_actuator(
    delta_act: np.ndarray, delta_cmd: np.ndarray, dt: float,
    tau: float, rate_max: float,
) -> np.ndarray:
    """Advance an actual steer angle toward its command: first-order lag (time
    constant `tau`) limited to `rate_max` [rad/s]. Models finite steering
    bandwidth so commands aren't applied instantaneously.
    """
    err = np.asarray(delta_cmd) - np.asarray(delta_act)
    rate = np.clip(err / max(tau, 1e-3), -rate_max, rate_max)
    return np.asarray(delta_act) + rate * dt


def wheel_icr_projection(
    wheel_pos_body: np.ndarray, delta: np.ndarray, icr_body: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Per-wheel steering centre + signed deviation from the vehicle ICR.

    A rigid body has exactly one true ICR; each wheel's steer angle only
    defines a *line* (the perpendicular through the wheel centre) on which
    its ICR must lie. We define the i-th wheel's steering centre as the
    orthogonal projection of the vehicle ICR R onto that line, and the
    deviation as the signed distance from R to the line (positive along the
    wheel's rolling direction). Under ideal Ackermann all four lines pass
    through R, so dev ≡ 0 and every point coincides with R — for any other
    strategy / slip condition, dev quantifies how much that wheel's steering
    geometry disagrees with the actual vehicle motion (its kinematic
    side-slip footprint).

    Args:
        wheel_pos_body: (N, 2) wheel positions in body frame.
        delta: (N,) actual steer angles [rad].
        icr_body: (2,) vehicle ICR in body frame; may contain NaN (straight
            line motion), in which case all outputs are NaN.

    Returns:
        (points, dev): (N, 2) projection points and (N,) signed distances.
    """
    p = np.asarray(wheel_pos_body, dtype=np.float64).reshape(-1, 2)
    d = np.asarray(delta, dtype=np.float64).reshape(-1)
    n = p.shape[0]
    if not np.all(np.isfinite(icr_body)):
        return np.full((n, 2), np.nan), np.full(n, np.nan)
    t = np.column_stack([np.cos(d), np.sin(d)])      # rolling directions (N,2)
    r = np.asarray(icr_body, dtype=np.float64) - p   # wheel → ICR (N,2)
    dev = np.einsum("ij,ij->i", r, t)                # signed distance along t
    points = np.asarray(icr_body, dtype=np.float64) - dev[:, None] * t
    return points, dev


def wheel_rack_force(
    kingpin_torques: np.ndarray,
    steering_arm_length: float,
    tie_rod_angle_deg: float,
    pinion_radius: float,
    rack_mech_efficiency: float,
    motor_gear_ratio: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Convert per-wheel kingpin torques to rack forces and motor torque demands.

    Force chain for split-rack 4WIS (分体齿条力链):
        τ_kingpin → F_tie = τ / L_arm → F_rack = F_tie · cos β → τ_motor = F_rack · r_p / (i · η)

    where β is the angle between the tie rod and the direction perpendicular to
    the rack axis (typically < 15°; cos β ≈ 1 for small β).

    Args:
        kingpin_torques: (4,) kingpin resistance torques [N·m]
        steering_arm_length: L_arm — moment arm from kingpin to tie-rod attachment [m]
        tie_rod_angle_deg: β — tie-rod deviation from rack-normal [deg]
        pinion_radius: r_p — pinion pitch-circle radius [m]
        rack_mech_efficiency: η — rack-and-pinion mechanical efficiency (0–1)
        motor_gear_ratio: i — reduction ratio from motor shaft to pinion

    Returns:
        rack_forces: (4,) rack axial forces [N]; sign matches kingpin torque sign
        motor_torques: (4,) motor shaft torque demands [N·m]
    """
    tau = np.asarray(kingpin_torques, dtype=np.float64)
    L = max(float(steering_arm_length), 1e-6)
    cos_beta = np.cos(np.deg2rad(float(tie_rod_angle_deg)))
    r_p = max(float(pinion_radius), 1e-6)
    eta = max(float(rack_mech_efficiency), 1e-6)
    i = max(float(motor_gear_ratio), 1e-6)

    rack_forces = tau * cos_beta / L
    motor_torques = rack_forces * r_p / (i * eta)
    return rack_forces, motor_torques


def steering_linkage_metrics(
    delta: np.ndarray,
    geometry: SteeringGeometryParams,
    *,
    rack_mech_efficiency: float = 1.0,
    steering_arm_length_fallback: float = 0.150,
) -> dict[str, np.ndarray]:
    """Compute split-rack linkage geometry for all four wheels.

    The hardpoints in :class:`SteeringGeometryParams` describe the left wheel
    at straight-ahead. Right wheels are mirrored across the local X axis. The
    outer ball joint rotates with the wheel; the inner joint slides on the
    rack axis while preserving the zero-position tie-rod length where possible.

    Returns shape-(4,) arrays:
        arm_tie_angle: angle between steering arm and tie rod [rad]
        tie_rack_angle: acute angle between tie rod and rack axis [rad]
        efficiency: ``sin(arm_tie) * cos(tie_rack) * rack_mech_efficiency``
        arm_length: instantaneous kingpin-to-outer-ball length [m]
        rack_travel: inner joint travel along rack axis from zero [m]
        rack_travel_derivative: signed ds/d(delta) [m/rad], from the constraint
        valid: whether this wheel angle is reachable with the given rack stroke
    """
    d = np.asarray(delta, dtype=np.float64).reshape(4)
    arm_tie = np.zeros(4)
    tie_rack = np.zeros(4)
    eff = np.zeros(4)
    arm_len = np.zeros(4)
    travel = np.zeros(4)
    derivative = np.zeros(4)
    valid = np.ones(4, dtype=bool)
    eta = float(np.clip(rack_mech_efficiency, 1e-6, 1.0))

    for i in range(4):
        is_front = i < 2
        is_left = i in (0, 2)
        prefix = "front" if is_front else "rear"
        mirror = 1.0 if is_left else -1.0

        outer0 = np.array([
            getattr(geometry, f"{prefix}_outer_x"),
            mirror * getattr(geometry, f"{prefix}_outer_y"),
        ], dtype=np.float64)
        inner0 = np.array([
            getattr(geometry, f"{prefix}_inner_x"),
            mirror * getattr(geometry, f"{prefix}_inner_y"),
        ], dtype=np.float64)
        axis_ang = np.deg2rad(getattr(geometry, f"{prefix}_rack_axis_deg"))
        axis = np.array([np.cos(axis_ang), mirror * np.sin(axis_ang)], dtype=np.float64)
        axis_norm = max(float(np.linalg.norm(axis)), 1e-12)
        axis = axis / axis_norm
        t_limit = max(float(getattr(geometry, f"{prefix}_rack_travel_limit")), 0.0)

        c, s = float(np.cos(d[i])), float(np.sin(d[i]))
        rot = np.array([[c, -s], [s, c]], dtype=np.float64)
        outer = rot @ outer0
        tie_len0 = max(float(np.linalg.norm(inner0 - outer0)), 1e-6)

        rel = inner0 - outer
        b = 2.0 * float(np.dot(axis, rel))
        cquad = float(np.dot(rel, rel) - tie_len0 * tie_len0)
        disc = b * b - 4.0 * cquad
        if disc >= 0.0:
            root = float(np.sqrt(disc))
            cand = [(-b + root) * 0.5, (-b - root) * 0.5]
            t = min(cand, key=abs)
        else:
            valid[i] = False
            # Geometry is over-constrained at this angle; use closest point on
            # the rack axis so efficiency still degrades smoothly.
            t = -float(np.dot(axis, rel))
        if t_limit > 0.0:
            valid[i] = valid[i] and abs(t) <= t_limit + 1e-10
            t = float(np.clip(t, -t_limit, t_limit))
        inner = inner0 + t * axis

        arm = outer
        tie = inner - outer
        la = max(float(np.linalg.norm(arm)), 1e-9)
        lt = max(float(np.linalg.norm(tie)), 1e-9)
        ua = arm / la
        ut = tie / lt
        dot_at = float(np.clip(np.dot(ua, ut), -1.0, 1.0))
        cross_at = float(ua[0] * ut[1] - ua[1] * ut[0])
        rack_projection = abs(float(np.dot(ut, axis)))
        valid[i] = valid[i] and la > 1e-8 and lt > 1e-8 and abs(cross_at) > 1e-10

        arm_tie[i] = float(np.arccos(dot_at))
        tie_rack[i] = float(np.arccos(np.clip(rack_projection, 0.0, 1.0)))
        eff[i] = max(abs(cross_at) * rack_projection * eta, 1e-6)
        arm_len[i] = la if la > 1e-8 else steering_arm_length_fallback
        travel[i] = t
        projection_signed = float(np.dot(ut, axis))
        if abs(projection_signed) > 1e-12:
            derivative[i] = la * cross_at / projection_signed
        else:
            derivative[i] = np.copysign(np.inf, cross_at)
            valid[i] = False

    return {
        "arm_tie_angle": arm_tie,
        "tie_rack_angle": tie_rack,
        "efficiency": eff,
        "arm_length": arm_len,
        "rack_travel": travel,
        "rack_travel_derivative": derivative,
        "valid": valid,
    }


def wheel_rack_force_from_linkage(
    kingpin_torques: np.ndarray,
    delta: np.ndarray,
    geometry: SteeringGeometryParams,
    steering_arm_length_fallback: float,
    pinion_radius: float,
    rack_mech_efficiency: float,
    motor_gear_ratio: float,
) -> tuple[np.ndarray, np.ndarray, dict[str, np.ndarray]]:
    """Convert kingpin torques through the hardpoint-derived rack linkage."""
    metrics = steering_linkage_metrics(
        delta,
        geometry,
        rack_mech_efficiency=rack_mech_efficiency,
        steering_arm_length_fallback=steering_arm_length_fallback,
    )
    tau = np.asarray(kingpin_torques, dtype=np.float64).reshape(4)
    # Virtual work: F*ds = tau*d(delta). Geometry is a motion ratio,
    # not an efficiency loss; the tie/rack cosine belongs in the numerator
    # of F = tau*cos(beta)/(arm*sin(theta)), not the denominator.
    lever = np.maximum(np.abs(metrics["rack_travel_derivative"]), 1e-6)
    eta = max(float(rack_mech_efficiency), 1e-6)
    r_p = max(float(pinion_radius), 1e-6)
    ratio = max(float(motor_gear_ratio), 1e-6)
    # Public rack-force signs remain steering-positive per corner. The signed
    # local-axis motion Jacobian is supplied separately in the metrics.
    rack_forces = tau / lever
    motor_torques = rack_forces * r_p / (ratio * eta)
    return rack_forces, motor_torques, metrics


def wheel_angle_from_rack_travel(
    rack_travel: float,
    geometry: SteeringGeometryParams,
    wheel_index: int,
    hint_delta: float = 0.0,
) -> dict[str, float]:
    """Solve the inverse: given rack inner-joint axial displacement, find δ.

    Mirrors the frontend `solveRackToWheel` so the two stay in sync. The
    linkage is a 4-bar (kingpin–outer ball–tie rod–inner ball on rack axis).
    For a given rack travel, the inner-joint position is fixed; the outer
    ball must lie on (a) a circle of radius `arm_length` around kingpin and
    (b) a circle of radius `tie_length` around the inner joint. Their two
    intersections give two candidate δ — pick the branch closest to `hint`
    so iteration stays on a continuous solution.

    Args:
        rack_travel: axial displacement [m] of the inner joint from zero
        geometry: hardpoint set
        wheel_index: 0=FL, 1=FR, 2=RL, 3=RR
        hint_delta: previous δ on the branch, used to disambiguate

    Returns:
        dict with keys:
            valid (bool): whether a real intersection exists
            delta (float): solved wheel angle [rad]; falls back to hint if invalid
            rack_travel (float): clamped to ±limit
            arm_tie_angle (float): [rad]
            tie_rack_angle (float): [rad]
            efficiency (float): same product as steering_linkage_metrics
    """
    is_front = wheel_index < 2
    is_left = wheel_index in (0, 2)
    prefix = "front" if is_front else "rear"
    mirror = 1.0 if is_left else -1.0

    outer0 = np.array([
        getattr(geometry, f"{prefix}_outer_x"),
        mirror * getattr(geometry, f"{prefix}_outer_y"),
    ], dtype=np.float64)
    inner0 = np.array([
        getattr(geometry, f"{prefix}_inner_x"),
        mirror * getattr(geometry, f"{prefix}_inner_y"),
    ], dtype=np.float64)
    axis_ang = np.deg2rad(getattr(geometry, f"{prefix}_rack_axis_deg"))
    axis = np.array([np.cos(axis_ang), mirror * np.sin(axis_ang)], dtype=np.float64)
    axis = axis / max(float(np.linalg.norm(axis)), 1e-12)
    t_limit = max(float(getattr(geometry, f"{prefix}_rack_travel_limit")), 0.0)

    t = float(np.clip(rack_travel, -t_limit, t_limit)) if t_limit > 0.0 else float(rack_travel)
    inner = inner0 + t * axis
    arm_length = max(float(np.linalg.norm(outer0)), 1e-6)
    tie_length = max(float(np.linalg.norm(inner0 - outer0)), 1e-6)
    theta0 = float(np.arctan2(outer0[1], outer0[0]))

    d = float(np.linalg.norm(inner))
    if d < 1e-9:
        return _inverse_result(False, hint_delta, t, 0.0, 0.0, 1e-6)

    a = (arm_length * arm_length - tie_length * tie_length + d * d) / (2.0 * d)
    h2 = arm_length * arm_length - a * a
    if h2 < -1e-9:
        return _inverse_result(False, hint_delta, t, 0.0, 0.0, 1e-6)
    h = float(np.sqrt(max(h2, 0.0)))
    along = inner * (a / d)
    perp = np.array([-inner[1] / d, inner[0] / d], dtype=np.float64)
    candidates = [along + h * perp, along - h * perp]

    best_delta = None
    best_score = float("inf")
    best_outer = candidates[0]
    for cand in candidates:
        delta = _wrap_pi(float(np.arctan2(cand[1], cand[0])) - theta0)
        score = abs(_wrap_pi(delta - hint_delta))
        if score < best_score:
            best_score = score
            best_delta = delta
            best_outer = cand
    assert best_delta is not None

    arm = best_outer
    tie = inner - best_outer
    la = max(float(np.linalg.norm(arm)), 1e-9)
    lt = max(float(np.linalg.norm(tie)), 1e-9)
    ua = arm / la
    ut = tie / lt
    dot_at = float(np.clip(np.dot(ua, ut), -1.0, 1.0))
    cross_at = float(ua[0] * ut[1] - ua[1] * ut[0])
    rack_projection = abs(float(np.dot(ut, axis)))
    arm_tie = float(np.arccos(dot_at))
    tie_rack = float(np.arccos(np.clip(rack_projection, 0.0, 1.0)))
    efficiency = max(abs(cross_at) * rack_projection, 1e-6)

    return _inverse_result(True, best_delta, t, arm_tie, tie_rack, efficiency)


def _inverse_result(
    valid: bool,
    delta: float,
    rack_travel: float,
    arm_tie: float,
    tie_rack: float,
    efficiency: float,
) -> dict[str, float]:
    return {
        "valid": valid,
        "delta": float(delta),
        "rack_travel": float(rack_travel),
        "arm_tie_angle": float(arm_tie),
        "tie_rack_angle": float(tie_rack),
        "efficiency": float(efficiency),
    }


def _wrap_pi(angle: float) -> float:
    a = (angle + np.pi) % (2.0 * np.pi)
    if a < 0:
        a += 2.0 * np.pi
    return float(a - np.pi)


def wheel_perpendicular_dir(delta: float) -> np.ndarray:
    """Unit vector perpendicular to the wheel's rolling direction (body frame).

    Wheel rolling direction = (cos δ, sin δ).
    Perpendicular (pointing toward "left" of wheel, i.e. +90° rotation) =
    (-sin δ, cos δ). Any scalar multiple of this lies on the line that must
    contain the ICR.
    """
    return np.array([-np.sin(delta), np.cos(delta)])
