"""Simplified planar 4WIS vehicle dynamics (3-DOF body + 4 wheel-spin DOFs).

Assumptions:
    * Body is a rigid block; no roll or pitch DOFs.
    * Suspension is rigid; vertical loads come from quasi-static load transfer
      (see `load_transfer.vertical_loads`).
    * Steering is instantaneous — δ tracks δ_cmd with no actuator lag (Phase 3
      will add a first-order steer servo).
    * Each wheel has its own moment of inertia and motor torque servo.
    * Linear tire (with friction-circle saturation); see `tire.LinearTireModel`.
    * Per-wheel μ comes from `env.scene.wheel_env(wheel_world_pos)` if a scene
      is attached, else `env.mu`.
    * Slope: when the scene's `wheel_env` returns a nonzero `ground_z` profile
      across wheels, we approximate it as a longitudinal gravity component
      proportional to the (front - rear) z gradient. Set up by SpeedBump /
      Slope disturbances in step 12.

Integration:
    Fixed-step RK4 at `dt_sim` (default 5 ms). State vector
        y = [vx, vy, ω, ω_FL, ω_FR, ω_RL, ω_RR]
    Pose (x, y, ψ) is integrated separately by forward Euler over the same dt
    using the RK4-final body velocity (good enough at 200 Hz).
"""

from __future__ import annotations

import math

import numpy as np

from sim4wis.core.state import (
    ControlCommand,
    EnvironmentState,
    N_WHEELS,
    VehicleParams,
    VehicleState,
)
from sim4wis.vehicle.base import VehicleModel
from sim4wis.vehicle.geometry import steer_actuator, vehicle_icr_from_velocity
from sim4wis.vehicle.kingpin import kingpin_torque
from sim4wis.vehicle.load_transfer import vertical_loads
from sim4wis.vehicle.model_core import rotate_wheel_forces_to_body, wheel_slip_kinematics
from sim4wis.vehicle.tire import TireModel, make_tire
from sim4wis.vehicle.wheel_servo import WheelSpeedServo


class SimplifiedDynamicModel(VehicleModel):
    """Phase 2 dynamic model — pluggable tire, fixed-step RK4 integrator."""

    def __init__(self, params: VehicleParams, tire: TireModel | None = None) -> None:
        super().__init__(params)
        self.tire = tire or make_tire(params)
        t_max = getattr(params, "motor_torque_max", 2000.0)
        self.servos = [
            WheelSpeedServo(
                kp=getattr(params, "servo_kp", 200.0),
                ki=getattr(params, "servo_ki", 50.0),
                torque_limit=t_max,
            )
            for _ in range(N_WHEELS)
        ]
        self.iw = getattr(params, "wheel_inertia", 1.2)
        # Per-wheel slip diagnostics (populated each step)
        self.slip_alpha = np.zeros(N_WHEELS)
        self.slip_kappa = np.zeros(N_WHEELS)
        # Per-wheel tyre forces in wheel-aligned frame (diagnostics)
        self.tire_fx = np.zeros(N_WHEELS)
        self.tire_fy = np.zeros(N_WHEELS)
        self.tire_mz = np.zeros(N_WHEELS)
        # Actual steer angle (tracks command through the steering actuator).
        self._delta_act = np.zeros(N_WHEELS)
        # Initialize static vertical loads
        self.state.fz = vertical_loads(params, ax=0.0, ay=0.0)

    # ---- API ---------------------------------------------------------------

    def reset(self, init: VehicleState | None = None) -> None:
        super().reset(init)
        for s in self.servos:
            s.reset()
        self.slip_alpha[:] = 0.0
        self.slip_kappa[:] = 0.0
        self._delta_act = np.zeros(N_WHEELS)
        self.state.fz = vertical_loads(self.params, 0.0, 0.0)

    def step(
        self,
        dt: float,
        cmd: ControlCommand,
        env: EnvironmentState,
    ) -> VehicleState:
        s = self.state
        p = self.params

        # 1) Steering actuator: δ tracks δ_cmd with first-order lag + rate limit.
        self._delta_act = steer_actuator(
            self._delta_act, cmd.delta_cmd, dt,
            getattr(p, "steer_tau", 0.06), getattr(p, "steer_rate_max", 8.0),
        )
        s.delta[:] = self._delta_act

        # Per-wheel surface mu cache (re-evaluated using world wheel positions at start)
        wheel_mus, fz_offset, ground_z = self._compute_wheel_env(env)

        # 1b) Bump-steer approximation: a wheel riding over vertical travel toes
        #     by coeff·z (left wheels +, right wheels −). Engineering stand-in
        #     for the missing suspension DOF — gives a visible twitch over bumps.
        bsc = getattr(p, "bump_steer_coeff", 0.0)
        if bsc and np.any(ground_z):
            toe_sign = np.sign(p.wheel_positions_body()[:, 1])  # +1 left, −1 right
            s.delta[:] = np.clip(
                s.delta + bsc * ground_z * toe_sign, -p.steer_limit, p.steer_limit
            )

        # 2) RK4 integration of body + wheel dynamics
        y0 = np.concatenate([
            [s.vx, s.vy, s.yaw_rate],
            s.wheel_omega,
        ])
        # Capture the per-wheel motor torques computed at start of step (held over dt).
        t_motor = self._compute_motor_torques(cmd, s.wheel_omega, dt)
        # Estimate longitudinal grade from front-rear z difference (body frame).
        # Front wheels are at +L/2, rear at -L/2; positive Δz/L = uphill.
        z_front = 0.5 * (ground_z[0] + ground_z[1])
        z_rear  = 0.5 * (ground_z[2] + ground_z[3])
        grade_long = math.atan2(z_front - z_rear, p.wheelbase)

        def f(yv: np.ndarray) -> np.ndarray:
            return self._derivatives(yv, s.delta, t_motor, wheel_mus, fz_offset, grade_long, env)

        h = dt
        k1 = f(y0)
        k2 = f(y0 + h * 0.5 * k1)
        k3 = f(y0 + h * 0.5 * k2)
        k4 = f(y0 + h * k3)
        y1 = y0 + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        s.vx = float(y1[0])
        s.vy = float(y1[1])
        s.yaw_rate = float(y1[2])
        s.wheel_omega[:] = y1[3:3 + N_WHEELS]

        # Reflect the disturbance vertical load (SpeedBump) in the reported Fz
        # so the transient is visible in telemetry / CSV / 3D — clamped to the
        # same physical band used inside the force computation.
        if np.any(fz_offset):
            s.fz = np.clip(s.fz + fz_offset, 0.0, p.mass * 9.81)

        # 3) Pose integration (forward Euler at dt — good enough at 200 Hz)
        cp = math.cos(s.psi)
        sp = math.sin(s.psi)
        s.x += dt * (s.vx * cp - s.vy * sp)
        s.y += dt * (s.vx * sp + s.vy * cp)
        s.psi += dt * s.yaw_rate

        # 4) Update derived geometry from the final state
        s.vehicle_icr_body = vehicle_icr_from_velocity(s.vx, s.vy, s.yaw_rate)

        # 5) Steering resistance torque from real suspension geometry
        #    (Reimpell/Pacejka kingpin moment) — uses Fy, Fx, Mz, Fz and the
        #    full SuspensionParams (caster / KPI / scrub).
        s.torque_steer = kingpin_torque(
            fx=self.tire_fx, fy=self.tire_fy, mz=self.tire_mz, fz=s.fz,
            suspension=p.suspension, delta=s.delta, tire_radius=p.tire_radius,
        )

        # 6) Wheel positions (constant) — set once for the streamer
        s.wheel_pos_body = p.wheel_positions_body()

        s.t += dt
        return s

    # ---- internals ---------------------------------------------------------

    def _compute_motor_torques(
        self, cmd: ControlCommand, omega_actual: np.ndarray, dt: float,
    ) -> np.ndarray:
        """Run all 4 wheel-speed servos for one step; return torque per wheel.

        Note: a r·Fx feedforward was evaluated to remove the small front/rear κ
        asymmetry (plan §11 item 1) but rejected — feeding back the *actual*
        slip-dependent Fx is a positive feedback loop that causes wheelspin.
        The bare PI's residual asymmetry during hard accel is small and stable.
        """
        torques = np.zeros(N_WHEELS)
        for i in range(N_WHEELS):
            torques[i] = self.servos[i].update(
                omega_actual=float(omega_actual[i]),
                omega_cmd=float(cmd.wheel_speed_cmd[i]),
                dt=dt,
            )
        return torques

    def _compute_wheel_env(self, env: EnvironmentState) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Per-wheel (mu, fz_offset, ground_z) at current pose. Length-4 arrays."""
        s = self.state
        wheels = self.params.wheel_positions_body()
        cp = math.cos(s.psi)
        sp = math.sin(s.psi)
        scene = getattr(env, "scene", None)
        base = env.mu if scene is None else getattr(scene, "base_mu", env.mu)
        mus = np.full(N_WHEELS, base)
        fz_off = np.zeros(N_WHEELS)
        ground_z = np.zeros(N_WHEELS)
        if scene is not None:
            for i in range(N_WHEELS):
                xw = s.x + cp * wheels[i, 0] - sp * wheels[i, 1]
                yw = s.y + sp * wheels[i, 0] + cp * wheels[i, 1]
                local = scene.wheel_env(np.array([xw, yw]))
                mus[i] = local.mu_override if local.mu_override is not None else base
                fz_off[i] = local.fz_offset
                ground_z[i] = local.ground_z
        return mus, fz_off, ground_z

    def _compute_wheel_mus(self, env: EnvironmentState) -> np.ndarray:
        """Back-compat wrapper used in older paths."""
        mus, _fz, _gz = self._compute_wheel_env(env)
        return mus

    def _derivatives(
        self,
        y: np.ndarray,
        delta: np.ndarray,
        t_motor: np.ndarray,
        wheel_mus: np.ndarray,
        fz_offset: np.ndarray,
        grade_long: float,
        env: EnvironmentState,  # noqa: ARG002
    ) -> np.ndarray:
        """Return dy/dt for state y = [vx, vy, ω, ω_FL, ω_FR, ω_RL, ω_RR]."""
        vx, vy, omega = y[0], y[1], y[2]
        wheel_omega = y[3:3 + N_WHEELS]
        p = self.params
        wheels = p.wheel_positions_body()

        # Tire force per wheel (in wheel-aligned frame), then rotate into body frame.
        fx_body = np.zeros(N_WHEELS)
        fy_body = np.zeros(N_WHEELS)
        fx_wheel_per = np.zeros(N_WHEELS)
        fy_wheel_per = np.zeros(N_WHEELS)
        mz_wheel_per = np.zeros(N_WHEELS)
        alpha_per = np.zeros(N_WHEELS)
        kappa_per = np.zeros(N_WHEELS)

        # Use current Fz (held over the RK4 sub-steps — we update it after the step)
        # Add disturbance Fz pulse (SpeedBump), then clamp to a physical band.
        # The upper clamp (≈ 4× static per-wheel load = m·g) is a safety net so a
        # mis-configured / very stiff bump can't blow up the tyre + wheel-spin ODE.
        fz_max = self.params.mass * 9.81
        fz = np.clip(self.state.fz + fz_offset, 0.0, fz_max)

        kin = wheel_slip_kinematics(
            vx=float(vx),
            vy=float(vy),
            yaw_rate=float(omega),
            wheel_omega=wheel_omega,
            delta=delta,
            wheel_positions_body=wheels,
            tire_radius=p.tire_radius,
        )

        for i in range(N_WHEELS):
            alpha = float(kin.alpha[i])
            kappa = float(kin.kappa[i])
            fx, fy, mz = self.tire.forces(alpha, kappa, float(fz[i]), float(wheel_mus[i]))
            fx_wheel_per[i] = fx
            fy_wheel_per[i] = fy
            mz_wheel_per[i] = mz
            alpha_per[i] = alpha
            kappa_per[i] = kappa

        fx_body, fy_body = rotate_wheel_forces_to_body(
            fx_wheel_per, fy_wheel_per, delta
        )

        # Total body force + moment about body origin
        fx_total = float(np.sum(fx_body))
        fy_total = float(np.sum(fy_body))
        m_z_total = float(np.sum(
            wheels[:, 0] * fy_body - wheels[:, 1] * fx_body + mz_wheel_per
        ))

        # Gravity component along body X (slope effect — positive grade => uphill
        # in vehicle's heading, which decelerates).
        g = 9.81
        m = p.mass
        f_grade = -m * g * math.sin(grade_long)
        fx_total += f_grade

        # Newton-Euler in body frame (with Coriolis terms)
        iz = p.inertia_z
        vx_dot = fx_total / m + omega * vy
        vy_dot = fy_total / m - omega * vx
        omega_dot = m_z_total / iz

        # Wheel rotational dynamics
        omega_w_dot = np.zeros(N_WHEELS)
        for i in range(N_WHEELS):
            omega_w_dot[i] = (t_motor[i] - p.tire_radius * fx_wheel_per[i]) / self.iw

        # Side-effect: stash latest slip + force for diagnostics
        self.slip_alpha[:] = alpha_per
        self.slip_kappa[:] = kappa_per
        self.tire_fx[:] = fx_wheel_per
        self.tire_fy[:] = fy_wheel_per
        self.tire_mz[:] = mz_wheel_per

        # Update Fz from latest accel estimate (so the NEXT step uses fresh loads)
        ax_body = vx_dot - omega * vy   # body-frame longitudinal accel
        ay_body = vy_dot + omega * vx   # body-frame lateral accel
        self.state.fz = vertical_loads(self.params, ax_body, ay_body)

        return np.concatenate([[vx_dot, vy_dot, omega_dot], omega_w_dot])
