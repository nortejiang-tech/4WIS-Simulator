"""Fault-tolerant reconfiguration strategy — single stuck steering actuator.

Safety mechanism implemented here (the 4WIS redundancy argument):

**Same-axle mirror force cancellation (feedforward).** When wheel *i* jams at
δ_s, it deviates from its allocated angle by e = δ_s − δ_i^alloc and injects a
parasitic lateral force ≈ −c_α·e plus the corresponding yaw moment. Its
same-axle partner sits at the SAME longitudinal arm x, so commanding the
partner to deviate by −e cancels **both** the net lateral force and the net
yaw moment exactly in the linear tyre range:

        δ_partner = δ_partner^alloc − (δ_s − δ_i^alloc)

The vehicle holds its course; the axle pays with an internal tyre-force fight
(scrub → heat/wear, and reduced lateral reserve on that axle), which is why a
degraded-mode speed cap accompanies the reallocation. A front-steer-only
vehicle cannot do this for a rear-axle fault at all — and for a front fault it
must involve the driver; 4WIS does it wheel-level and driver-transparent.

**Yaw-rate feedback (stabilisation).** The mirror term is exact only in the
linear range with equal c_α. A rear-wheel jam at speed is a *spin excitation*:
during the detection window the body develops sideslip, and the mirrored rear
pair then runs at ±δ_s offsets around that sideslip — one of them saturates
first and the static cancellation no longer holds. The yaw loop therefore has
to act at ESP-grade authority, not as a trim:

        δ_fb = clip(k_r·(r_des − r), ±8°),  r_des = v·κ_driver,  k_r = 0.5
        applied to every healthy wheel, front +δ_fb / rear −δ_fb
        (front and rear arms are equal, so this is a pure yaw-moment command)

**Ramped speed reduction.** The degraded-mode cap is approached with a decel
limit (3 m/s²) instead of a step: the wheel-speed servos would otherwise
brake at friction-limit torque and the longitudinal force steals exactly the
tyre-ellipse margin the saturated axle needs for lateral stabilisation.

An alternative purely kinematic reallocation (project the desired ICR onto
the stuck wheel's steering line — zero scrub, but the "straight" request
degenerates to a crab drift and the transition itself excites yaw) was
evaluated and rejected for the in-emergency phase; see the safety report
(docs/reports/) for the comparison data. It remains attractive as a *parked /
creep-home* mode where scrub-free rolling matters more than course holding.

mode_params (ground truth from the experiment; from the monitor in a real car):
**Free-castering failure (`fault_kind: "free"`).** A de-energised wheel on a
non-self-locking mechanism passively aligns with its local velocity (caster
equilibrium ≈ the ideal-Ackermann direction), so it injects almost no
parasitic force — but the axle loses that wheel's cornering-stiffness share.
Mirroring is wrong here (there is nothing to cancel); the mitigation is
**authority compensation**: the healthy same-axle wheel takes `front_gain`
(≈2) times its nominal allocation to restore the axle force in the linear
range, with the same yaw PI and ramped speed cap on top.

mode_params (ground truth from the experiment; from the monitor in a real car):
    fault_wheel    0..3 (FL FR RL RR)
    fault_kind     "stuck" (self-locking jam) | "free" (castering), default "stuck"
    fault_angle    stuck angle [rad] (angle-sensor estimate; unused for free)
    fault_time     sim time of the fault [s]
    detect_delay   detection + arbitration latency [s] (default 0.15)
    v_limit_kmh    degraded-mode speed cap (default 60)
    k_yaw          yaw-rate feedback gain [rad per rad/s] (default 0.5)
    decel_max      speed-cap approach decel [m/s²] (default 3.0)
    front_gain     healthy-partner authority boost in free mode (default 2.0)
"""

from __future__ import annotations

import numpy as np

from sim4wis.controller.base import (
    BodyMotionTarget,
    ControllerStrategy,
    compute_commands,
)
from sim4wis.controller.ideal_ackermann import _max_curvature
from sim4wis.core.state import ControlCommand, DriverInput, VehicleParams, VehicleState

# Same-axle partner: FL<->FR, RL<->RR.
PARTNER = {0: 1, 1: 0, 2: 3, 3: 2}


class FaultReconfigStrategy(ControllerStrategy):
    name = "fault_reconfig"

    def __init__(self, params: VehicleParams) -> None:
        super().__init__(params)
        self._kappa_max = _max_curvature(params)
        self._v_detect: float | None = None   # speed latched at detection
        self._yaw_int = 0.0                   # yaw-error integral [rad]
        self._t_prev: float | None = None

    def compute(self, driver: DriverInput, state: VehicleState) -> ControlCommand:
        p = self.params
        mp = driver.mode_params or {}
        wheel = int(mp.get("fault_wheel", 0))
        delta_s = float(mp.get("fault_angle", 0.0))
        t_fault = float(mp.get("fault_time", 0.0))
        t_detect = t_fault + float(mp.get("detect_delay", 0.15))
        v_limit = float(mp.get("v_limit_kmh", 60.0)) / 3.6
        k_yaw = float(mp.get("k_yaw", 0.5))
        decel_max = float(mp.get("decel_max", 3.0))

        kappa = self._kappa_max * float(driver.steering)
        v_cmd = p.v_max * float(driver.throttle)

        detected = float(state.t) >= t_detect
        if detected:
            # Ramped speed cap: step-commanding the low target makes the wheel
            # servos brake at friction-limit torque, stealing the tyre-ellipse
            # margin the (possibly saturated) rear axle needs laterally.
            if self._v_detect is None:
                self._v_detect = abs(float(state.vx))
            v_cap = max(v_limit,
                        self._v_detect - decel_max * (float(state.t) - t_detect))
            v_cmd = float(np.clip(v_cmd, -v_cap, v_cap))

        # Nominal allocation (ideal Ackermann — all four wheels share one ICR).
        cmd = self._ideal(kappa, v_cmd)
        if not detected:
            return cmd

        # ---- degraded mode ---------------------------------------------------
        partner = PARTNER[wheel]
        fault_kind = str(mp.get("fault_kind", "stuck"))
        delta = np.array(cmd.delta_cmd, dtype=np.float64)

        if fault_kind == "free":
            # Castering wheel: no parasitic force to cancel — restore the lost
            # axle authority by boosting the healthy partner's allocation.
            front_gain = float(mp.get("front_gain", 2.0))
            delta[partner] = front_gain * delta[partner]
        else:
            # Self-locking jam: mirror cancellation of force AND moment.
            e = delta_s - float(cmd.delta_cmd[wheel])   # stuck wheel's angle error
            delta[wheel] = delta_s                      # reflect physical truth
            delta[partner] = delta[partner] - e

        # ESP-grade yaw-rate PI on every healthy wheel (front +, rear −: equal
        # arms → pure yaw moment). The integral term holds a standing counter-
        # moment when a saturated tyre leaves a residual imbalance the mirror
        # can't cancel (typical for rear-wheel jams at speed) — a P-only loop
        # would carry that as a permanent slow heading drift.
        r_des = v_cmd * kappa
        e_r = r_des - float(state.yaw_rate)
        t_now = float(state.t)
        dt = min(0.05, t_now - self._t_prev) if self._t_prev is not None else 0.0
        self._t_prev = t_now
        self._yaw_int = float(np.clip(self._yaw_int + e_r * dt,
                                      -np.deg2rad(6.0) / max(k_yaw, 1e-6),
                                      +np.deg2rad(6.0) / max(k_yaw, 1e-6)))
        fb = float(np.clip(k_yaw * (e_r + 0.8 * self._yaw_int),
                           -np.deg2rad(8.0), np.deg2rad(8.0)))
        for j in range(4):
            if j == wheel:
                continue
            delta[j] += fb if j < 2 else -fb

        delta = np.clip(delta, -p.steer_limit, p.steer_limit)
        return ControlCommand(
            delta_cmd=delta,
            wheel_speed_cmd=cmd.wheel_speed_cmd,
            icr_target_body=cmd.icr_target_body,
        )

    # Normal-mode allocation (same law as IdealAckermannStrategy).
    def _ideal(self, kappa: float, v_cmd: float) -> ControlCommand:
        p = self.params
        if abs(kappa) < 1e-9:
            target = BodyMotionTarget(vx=v_cmd, vy=0.0, omega=0.0,
                                      icr_target_body=np.array([np.nan, np.nan]))
        else:
            target = BodyMotionTarget(
                vx=v_cmd, vy=0.0, omega=v_cmd * kappa,
                icr_target_body=np.array([0.0, 1.0 / kappa]),
            )
        return compute_commands(
            wheels=p.wheel_positions_body(),
            target=target,
            tire_radius=p.tire_radius,
            steer_limit=p.steer_limit,
        )
