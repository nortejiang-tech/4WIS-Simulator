"""Follow-trajectory strategy — pure-pursuit path tracking for 4WIS.

Reads the process-wide active `PathPlan` (set via the REST layer) and steers
the vehicle along it using pure pursuit. Because this is a 4WIS vehicle we
realise the pursuit curvature as an *ideal-Ackermann* body motion (all wheels
share one ICR), so steady tracking has zero tyre scrub.

Speed control:
    v_cmd = v_max · throttle. If throttle is ~0 and a `cruise_speed`
    mode-param is set, cruise at that speed so the path auto-runs for demos.
    Near the end of an open path the command tapers to 0. The command is also
    capped by the lateral-acceleration limit `ay_max` against the maximum
    upcoming path curvature inside a braking-distance preview window:
    v ≤ sqrt(ay_max / |κ|_max).

Tunables (`driver.mode_params` overrides, all optional):
    cruise_speed, lookahead_kv, lookahead_base, lookahead_min, lookahead_max,
    ay_max, ax_brake.

If no path is loaded the strategy degenerates to straight-line driving so the
selector never produces a broken state.
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import (
    BodyMotionTarget,
    ControllerStrategy,
    compute_commands,
)
from sim4wis.controller.longitudinal import speed_command
from sim4wis.controller.path import get_active_plan
from sim4wis.core.state import ControlCommand, DriverInput, VehicleParams, VehicleState


class FollowTrajectoryStrategy(ControllerStrategy):
    name = "follow_trajectory"

    # Lookahead: Ld = clamp(k_v·|v| + Ld0, Ld_min, Ld_max) — defaults,
    # overridable per-run via driver.mode_params (see module docstring).
    K_V = 0.6
    LD0 = 3.0
    LD_MIN = 2.5
    LD_MAX = 14.0
    AY_MAX = 3.0       # lateral acceleration limit for curvature speed cap [m/s²]
    AX_BRAKE = 2.0     # assumed comfortable braking decel for the preview window [m/s²]

    def __init__(self, params: VehicleParams) -> None:
        super().__init__(params)
        # Headless sessions (experiment.batch) inject their own plan here so
        # they never touch the process-wide active plan used by the RT loop.
        self.plan_override = None

    @staticmethod
    def _mp(driver: DriverInput, key: str, default: float) -> float:
        try:
            return float(driver.mode_params.get(key, default)) if driver.mode_params else default
        except (TypeError, ValueError):
            return default

    def compute(self, driver: DriverInput, state: VehicleState, dt: float = 0.0) -> ControlCommand:
        p = self.params
        plan = self.plan_override if self.plan_override is not None else get_active_plan()

        cruise = self._mp(driver, "cruise_speed", 0.0)
        k_v = self._mp(driver, "lookahead_kv", self.K_V)
        ld0 = self._mp(driver, "lookahead_base", self.LD0)
        ld_min = self._mp(driver, "lookahead_min", self.LD_MIN)
        ld_max = self._mp(driver, "lookahead_max", self.LD_MAX)
        ay_max = max(0.1, self._mp(driver, "ay_max", self.AY_MAX))
        ax_brake = max(0.1, self._mp(driver, "ax_brake", self.AX_BRAKE))

        v_cmd = speed_command(p, driver, float(state.vx), dt, float(state.mu_avg))
        if abs(v_cmd) < 0.1 and cruise != 0.0:
            v_cmd = cruise

        if plan.is_empty():
            target = BodyMotionTarget(vx=v_cmd, vy=0.0, omega=0.0,
                                      icr_target_body=np.array([np.nan, np.nan]))
            return compute_commands(wheels=p.wheel_positions_body(), target=target,
                                    tire_radius=p.tire_radius, steer_limit=p.steer_limit)

        pts = plan.points
        pos = np.array([state.x, state.y])

        # Nearest path index to the vehicle.
        d2 = np.sum((pts - pos) ** 2, axis=1)
        i_near = int(np.argmin(d2))

        speed = max(abs(state.vx), abs(v_cmd))
        ld = min(ld_max, max(ld_min, k_v * speed + ld0))

        # Walk forward along the path until we've covered the lookahead distance.
        n = pts.shape[0]
        acc = 0.0
        i = i_near
        target_pt = pts[-1]
        reached_end = True
        while True:
            j = i + 1
            if j >= n:
                if plan.closed:
                    j = 0
                else:
                    break
            acc += float(np.linalg.norm(pts[j] - pts[i]))
            i = j
            if acc >= ld:
                target_pt = pts[i]
                reached_end = False
                break
            if plan.closed and i == i_near:
                break

        # Transform lookahead point into body frame.
        c, s = np.cos(state.psi), np.sin(state.psi)
        dxw, dyw = target_pt[0] - state.x, target_pt[1] - state.y
        ex = c * dxw + s * dyw       # forward
        ey = -s * dxw + c * dyw      # left

        chord2 = ex * ex + ey * ey
        kappa = (2.0 * ey / chord2) if chord2 > 1e-6 else 0.0

        # Curvature speed planning: scan a braking-distance preview window
        # ahead of the vehicle for the maximum discrete path curvature, and cap
        # v_cmd so lateral acceleration stays within ay_max. The current
        # pursuit curvature is included so tight corrections also slow down.
        preview = max(ld, speed * speed / (2.0 * ax_brake))
        kappa_max = abs(kappa)
        acc2 = 0.0
        k = i_near
        for _ in range(n):  # at most one lap
            if acc2 >= preview:
                break
            if plan.closed:
                k1, k2 = (k + 1) % n, (k + 2) % n
            else:
                k1, k2 = k + 1, k + 2
                if k2 >= n:
                    break
            seg = pts[k1] - pts[k]
            seg2 = pts[k2] - pts[k1]
            l1 = float(np.linalg.norm(seg))
            l2 = float(np.linalg.norm(seg2))
            if l1 > 1e-6 and l2 > 1e-6:
                # Discrete curvature ≈ heading change / arc length over 3 points.
                h1 = np.arctan2(seg[1], seg[0])
                h2 = np.arctan2(seg2[1], seg2[0])
                dh = float(np.arctan2(np.sin(h2 - h1), np.cos(h2 - h1)))
                kappa_max = max(kappa_max, abs(dh) / (0.5 * (l1 + l2)))
            acc2 += l1
            k = k1
        if kappa_max > 1e-6:
            v_curve = float(np.sqrt(ay_max / kappa_max))
            v_cmd = float(np.clip(v_cmd, -v_curve, v_curve))

        # Taper speed approaching the end of an open path.
        if reached_end and not plan.closed:
            dist_end = float(np.linalg.norm(pts[-1] - pos))
            if dist_end < 3.0:
                v_cmd *= max(0.0, dist_end / 3.0)

        if abs(kappa) < 1e-9:
            target = BodyMotionTarget(vx=v_cmd, vy=0.0, omega=0.0,
                                      icr_target_body=np.array([np.nan, np.nan]))
        else:
            y_r = 1.0 / kappa
            target = BodyMotionTarget(vx=v_cmd, vy=0.0, omega=v_cmd * kappa,
                                      icr_target_body=np.array([0.0, y_r]))

        return compute_commands(wheels=p.wheel_positions_body(), target=target,
                                tire_radius=p.tire_radius, steer_limit=p.steer_limit)
