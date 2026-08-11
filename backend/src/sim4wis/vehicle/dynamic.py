"""Simplified planar 4WIS vehicle dynamics (3-DOF body + 4 wheel-spin DOFs).

Assumptions:
    * Body is a rigid block; no roll or pitch DOFs.
    * Suspension is rigid; vertical loads come from quasi-static load transfer
      (see `load_transfer.vertical_loads`) plus per-axle aero lift (∝ v²).
    * Steering: δ tracks δ_cmd through a first-order lag + rate limit
      (`steer_actuator`), then static toe offsets are added per wheel.
    * Each wheel has its own moment of inertia and motor torque servo.
    * Pluggable tire (linear / Pacejka, friction-circle/ellipse saturation);
      camber thrust enters as an equivalent slip-angle offset (same absorption
      as the load-analysis page).
    * Body-X resistance: aero drag (½ρ·Cd·A·v²) + rolling resistance (Crr·m·g)
      via `model_core.body_resistance_force` — shared coefficients with the
      quasi-static load page.
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
from sim4wis.vehicle.steering_link import make_steering_plant, plant_front_angle
from sim4wis.vehicle.kingpin import kingpin_torque
from sim4wis.vehicle.load_transfer import vertical_loads
from sim4wis.vehicle.model_core import (
    body_resistance_force,
    axle_cornering_scale,
    camber_thrust_alpha_offset,
    friction_brake_torques,
    kc_wheel_offsets,
    load_sensitive_mu,
    relax_slip,
    resolve_kc,
    store_grip_state as _store_grip,
    wheel_grip_state,
    fz_with_aero_lift,
    rotate_wheel_forces_to_body,
    semi_implicit_wheel_spin,
    static_toe_offsets,
    wheel_slip_kinematics,
)
from sim4wis.vehicle.tire import TireModel, make_tire
from sim4wis.vehicle.wheel_servo import WheelSpeedServo


class SimplifiedDynamicModel(VehicleModel):
    """Phase 2 dynamic model — pluggable tire, fixed-step RK4 integrator."""

    def __init__(self, params: VehicleParams, tire: TireModel | None = None) -> None:
        super().__init__(params)
        self.tire = tire or make_tire(params)
        t_max = getattr(params, "motor_torque_max", 2000.0)
        self.iw = getattr(params, "wheel_inertia", 1.2)
        # Reflected vehicle inertia per wheel for the servo's cmd-rate
        # feedforward: accelerating the body through the tyre looks like an
        # extra m·r²/4 on each wheel shaft.
        ff_inertia = self.iw + params.mass * params.tire_radius**2 / N_WHEELS
        self.servos = [
            WheelSpeedServo(
                kp=getattr(params, "servo_kp", 200.0),
                ki=getattr(params, "servo_ki", 50.0),
                torque_limit=t_max,
                ff_inertia=ff_inertia,
            )
            for _ in range(N_WHEELS)
        ]
        # Per-wheel slip diagnostics (populated each step)
        self.slip_alpha = np.zeros(N_WHEELS)
        self.slip_kappa = np.zeros(N_WHEELS)
        # Per-wheel tyre forces in wheel-aligned frame (diagnostics)
        self.tire_fx = np.zeros(N_WHEELS)
        self.tire_fy = np.zeros(N_WHEELS)
        self.tire_mz = np.zeros(N_WHEELS)
        # Actual steer angle (tracks command through the steering actuator).
        self._delta_act = np.zeros(N_WHEELS)
        # Steering system as a plant. None unless enabled AND the architecture
        # has a mechanical front axle — by-wire front axles do not steer through
        # a column and must not take this path.
        self._steering, self._mech_ratio = make_steering_plant(params)
        self._prev_hand = 0.0
        # Static toe (A3, same convention as the load page): the physical wheel
        # angle is the actuator angle plus the per-wheel alignment offset.
        self._toe = static_toe_offsets(params)
        # Axle cornering-stiffness split, absorbed as a slip-angle scale.
        self._alpha_scale = axle_cornering_scale(params)
        # Measured K&C, or a synthesised curve from the legacy bump_steer_coeff.
        self._kc = resolve_kc(params)
        self._kc_fx = np.zeros(N_WHEELS)
        self._kc_fy = np.zeros(N_WHEELS)
        self._kc_fz = vertical_loads(params, 0.0, 0.0)
        self._kc_mz = np.zeros(N_WHEELS)
        # Static corner load, the reference the inferred travel is measured from.
        self._fz_static = vertical_loads(params, 0.0, 0.0)
        self._kc_camber = np.zeros(N_WHEELS)
        self._kc_jounce = np.zeros(N_WHEELS)
        # K&C increments actually applied this step (diagnostics).
        self.kc_toe = np.zeros(N_WHEELS)
        self.kc_camber = np.zeros(N_WHEELS)
        # Relaxation-lagged slip states (see model_core.relax_slip).
        self._alpha_lag = np.zeros(N_WHEELS)
        self._kappa_lag = np.zeros(N_WHEELS)
        # Filtered friction-brake command per wheel (first-order, brake_tau).
        self._brake_f = np.zeros(N_WHEELS)
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
        if self._steering is None:
            self._delta_act = steer_actuator(
                self._delta_act, cmd.delta_cmd, dt,
                getattr(p, "steer_tau", 0.06), getattr(p, "steer_rate_max", 8.0),
            )
        else:
            # Front axle goes through the real steering system: the strategy
            # still decides where the wheels should point, the plant decides how
            # they get there — with assist, friction, inertia and motor limits.
            # The rear axle keeps the actuator model.
            self._delta_act[2:] = steer_actuator(
                self._delta_act[2:], cmd.delta_cmd[2:], dt,
                getattr(p, "steer_tau", 0.06), getattr(p, "steer_rate_max", 8.0),
            )
            delta_f, self._prev_hand = plant_front_angle(
                self._steering, self._mech_ratio, cmd.delta_cmd, dt,
                prev_hand=self._prev_hand, rack_force=float(np.sum(s.rack_force[:2])),
                speed_ms=float(s.vx),
            )
            self._delta_act[0] = self._delta_act[1] = delta_f
        s.delta[:] = np.clip(self._delta_act + self._toe, -p.steer_limit, p.steer_limit)

        # Per-wheel surface mu cache (re-evaluated using world wheel positions at start)
        wheel_mus, fz_offset, ground_z = self._compute_wheel_env(env)
        s.mu_avg = float(np.mean(wheel_mus))

        # 1b) K&C. This model carries no suspension DOF, so travel is inferred
        #     from the load it already computes: z = (Fz − Fz_static)/k_spring,
        #     plus whatever vertical displacement the road itself imposes. That
        #     is exactly as quasi-static as the rest of this model's vertical
        #     behaviour, costs nothing extra, and lets one K&C table drive both
        #     models rather than each having its own approximation.
        self._kc_jounce = ((self.state.fz - self._fz_static)
                           / max(float(p.suspension.spring_rate), 1.0)) + ground_z
        kc_toe, self._kc_camber = kc_wheel_offsets(
            self._kc, jounce=self._kc_jounce, fx=self._kc_fx, fy=self._kc_fy,
            fz=self._kc_fz, mz=self._kc_mz)
        self.kc_toe, self.kc_camber = kc_toe, self._kc_camber
        if np.any(kc_toe):
            s.delta[:] = np.clip(s.delta + kc_toe, -p.steer_limit, p.steer_limit)

        # 2) RK4 integration of the body DOFs (vx, vy, ω). Wheel spin is
        #    deliberately NOT in the RK4 vector: its linearised mode has
        #    λ·h > RK4's stability limit at low-mid speed (see
        #    model_core.semi_implicit_wheel_spin) — ω is held over the body
        #    step and advanced stiff-stably afterwards.
        if float(getattr(p, "tire_relax_length", 0.0)) > 0.0:
            kin0 = wheel_slip_kinematics(
                vx=s.vx, vy=s.vy, yaw_rate=s.yaw_rate,
                wheel_omega=s.wheel_omega, delta=s.delta,
                wheel_positions_body=p.wheel_positions_body(),
                tire_radius=p.tire_radius)
            self._alpha_lag = relax_slip(
                self._alpha_lag, self._alpha_scale * kin0.alpha, kin0.vx_wheel,
                float(p.tire_relax_length), dt)

        y0 = np.array([s.vx, s.vy, s.yaw_rate])
        # Capture the per-wheel motor torques computed at start of step (held over dt).
        t_motor = self._compute_motor_torques(cmd, s.wheel_omega, dt)
        wheel_omega_held = s.wheel_omega.copy()
        # Estimate longitudinal grade from front-rear z difference (body frame).
        # Front wheels are at +L/2, rear at -L/2; positive Δz/L = uphill.
        z_front = 0.5 * (ground_z[0] + ground_z[1])
        z_rear  = 0.5 * (ground_z[2] + ground_z[3])
        grade_long = math.atan2(z_front - z_rear, p.wheelbase)

        def f(yv: np.ndarray) -> np.ndarray:
            return self._derivatives(
                yv, wheel_omega_held, s.delta, wheel_mus, fz_offset, grade_long, env
            )

        h = dt
        k1 = f(y0)
        k2 = f(y0 + h * 0.5 * k1)
        k3 = f(y0 + h * 0.5 * k2)
        k4 = f(y0 + h * k3)
        y1 = y0 + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        # Body-frame specific force (what an accelerometer at the CG reads).
        # Taken from the RK4 slope at the accepted state rather than from a
        # finite difference of vx/vy, so it stays clean at 200 Hz.
        s.ax = float(y1[0] - y0[0]) / dt - float(y1[2]) * float(y1[1])
        s.ay = float(y1[1] - y0[1]) / dt + float(y1[2]) * float(y1[0])

        s.vx = float(y1[0])
        s.vy = float(y1[1])
        s.yaw_rate = float(y1[2])

        # 2b) Wheel-spin update at the final body state (semi-implicit, stiff-
        #     stable) + final consistent tyre forces for the diagnostics.
        kin = wheel_slip_kinematics(
            vx=s.vx, vy=s.vy, yaw_rate=s.yaw_rate,
            wheel_omega=wheel_omega_held, delta=s.delta,
            wheel_positions_body=p.wheel_positions_body(),
            tire_radius=p.tire_radius,
        )
        fz_now = np.clip(self.state.fz + fz_offset, 0.0, p.mass * 9.81)
        alpha_camber = camber_thrust_alpha_offset(p, fz_now, extra_camber=self._kc_camber)
        # Friction-brake torque (work-package A): added to the motor torque so
        # the wheel-spin ODE sees the net axle torque. Computed here rather than
        # at the top of the step because the lockup test needs the *transferred*
        # loads (fz_now) and the per-wheel ground speed — under hard braking the
        # front axle carries far more than its static share, which is exactly
        # what decides whether it locks.
        # Lockup is a friction-limited event, so it must see the same effective
        # mu the tyre forces do: the heavily loaded front wheel has less grip
        # than nominal, which is exactly what decides whether it locks first.
        mu_eff = load_sensitive_mu(p, wheel_mus, fz_now)
        t_brake, locked, self._brake_f = friction_brake_torques(
            brake_cmd=cmd.brake_cmd,
            brake_filtered=self._brake_f,
            wheel_omega=wheel_omega_held,
            vx_wheel=kin.vx_wheel,
            fz=fz_now,
            mu=mu_eff,
            params=p,
            dt=dt,
        )
        s.wheel_locked = locked
        t_net = t_motor + t_brake
        # Relaxation: the tyre winds up over a rolling distance, so the slip it
        # actually works at lags the geometric slip. Applied to the geometric
        # part only — the camber offset is a force equivalence, not a slip that
        # has to build.
        alpha_fed = (self._alpha_lag if float(getattr(p, "tire_relax_length", 0.0)) > 0.0
                     else self._alpha_scale * kin.alpha)
        omega_new, forces, kappa_new = semi_implicit_wheel_spin(
            dt=dt,
            wheel_omega=wheel_omega_held,
            vx_wheel=kin.vx_wheel,
            alpha=alpha_fed + alpha_camber,
            fz=fz_now,
            mu=mu_eff,
            torque=t_net,
            brake_torque=t_brake,
            tire=self.tire,
            tire_radius=p.tire_radius,
            wheel_inertia=self.iw,
        )
        s.wheel_omega[:] = omega_new
        self.slip_alpha[:] = kin.alpha
        self.slip_kappa[:] = kappa_new
        self.tire_fx[:] = forces.fx
        self.tire_fy[:] = forces.fy
        self.tire_mz[:] = forces.mz
        # Hand this step's tyre loads to next step's compliance terms.
        self._kc_fx, self._kc_fy = forces.fx.copy(), forces.fy.copy()
        self._kc_fz, self._kc_mz = fz_now.copy(), forces.mz.copy()
        # Friction budget from the forces that actually moved the vehicle this
        # step, so the readout can never disagree with the dynamics.
        _store_grip(s, wheel_grip_state(
            fx=forces.fx, fy=forces.fy, fz=fz_now, mu=mu_eff,
            alpha=kin.alpha, kappa=kappa_new, vx_wheel=kin.vx_wheel, tire=self.tire,
        ))

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
        """Drive torque per wheel for one step.

        In `longitudinal_mode == "torque"` the command IS the torque and the
        servos sit out (their state is reset so switching modes mid-run doesn't
        dump a stale integrator into the drivetrain). Otherwise the wheel-speed
        servos turn `wheel_speed_cmd` into torque, as below.

        The friction-brake command is read from `cmd.brake_cmd` and turned into
        an opposing torque in `friction_brake_torques` (added to the drive
        torque before the wheel-spin ODE). Here we only pass the per-wheel
        brake-active flag to the servo so it does not fight the brake.

        Note: a r·Fx feedforward was evaluated to remove the small front/rear κ
        asymmetry (plan §11 item 1) but rejected — feeding back the *actual*
        slip-dependent Fx is a positive feedback loop that causes wheelspin.
        The bare PI's residual asymmetry during hard accel is small and stable.
        """
        if str(getattr(self.params, "longitudinal_mode", "speed_servo")) == "torque":
            for servo in self.servos:
                servo.reset()
            drive = np.asarray(cmd.drive_torque_cmd, dtype=np.float64).reshape(N_WHEELS)
            # The brake still owns the wheel it is applied to.
            return np.where(np.asarray(cmd.brake_cmd) > 0.02, 0.0, drive)

        torques = np.zeros(N_WHEELS)
        for i in range(N_WHEELS):
            torques[i] = self.servos[i].update(
                omega_actual=float(omega_actual[i]),
                omega_cmd=float(cmd.wheel_speed_cmd[i]),
                dt=dt,
                brake_active=bool(cmd.brake_cmd[i] > 0.02),
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
        wheel_omega: np.ndarray,
        delta: np.ndarray,
        wheel_mus: np.ndarray,
        fz_offset: np.ndarray,
        grade_long: float,
        env: EnvironmentState,  # noqa: ARG002
    ) -> np.ndarray:
        """Return dy/dt for body state y = [vx, vy, ω] (wheel ω held over dt)."""
        vx, vy, omega = y[0], y[1], y[2]
        p = self.params
        wheels = p.wheel_positions_body()

        # Tire force per wheel (in wheel-aligned frame), then rotate into body frame.
        fx_body = np.zeros(N_WHEELS)
        fy_body = np.zeros(N_WHEELS)
        fx_wheel_per = np.zeros(N_WHEELS)
        fy_wheel_per = np.zeros(N_WHEELS)
        mz_wheel_per = np.zeros(N_WHEELS)

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

        # Camber thrust (A2): absorbed as an equivalent slip-angle offset so the
        # tyre model's friction limit applies to the combined force — the same
        # absorption the load-analysis page uses. Diagnostics keep the true α.
        alpha_camber = camber_thrust_alpha_offset(p, fz, extra_camber=self._kc_camber)

        # Relaxation-lagged slip. When enabled, the slip the tyre works at is
        # held over the RK4 sub-steps and advanced once per step (the same
        # treatment wheel spin already gets) rather than being added to the
        # integrated state vector. At 200 Hz against a ~30 ms lag the
        # difference is not resolvable, and it keeps the state vector — and the
        # stiff-mode analysis behind it — unchanged.
        #
        # With sigma = 0 the fresh per-stage slip is used exactly as before, so
        # the feature off is bit-identical to not having it.
        use_lag = float(getattr(p, "tire_relax_length", 0.0)) > 0.0
        # μ(Fz) must be applied HERE, on the loads this stage actually sees.
        # Applying it only in the wheel-spin pass leaves the body force — the
        # thing that moves the car — running on nominal μ, and then the friction
        # circle drawn from it reports utilisation above 1.0. k = 0 returns the
        # input array unchanged, so the feature off stays bit-identical.
        mu_stage = load_sensitive_mu(p, wheel_mus, fz)
        for i in range(N_WHEELS):
            alpha = float(self._alpha_lag[i] / max(float(self._alpha_scale[i]), 1e-9)
                          if use_lag else kin.alpha[i])
            kappa = float(kin.kappa[i])
            fx, fy, mz = self.tire.forces(
                float(self._alpha_scale[i]) * alpha + float(alpha_camber[i]),
                kappa, float(fz[i]), float(mu_stage[i])
            )
            fx_wheel_per[i] = fx
            fy_wheel_per[i] = fy
            mz_wheel_per[i] = mz

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

        # Aero drag + rolling resistance (time-domain twin of the load page's
        # drive-force balance) — opposes body-X motion, vanishes at standstill.
        fx_total += body_resistance_force(p, vx)

        # Newton-Euler in body frame (with Coriolis terms)
        iz = p.inertia_z
        vx_dot = fx_total / m + omega * vy
        vy_dot = fy_total / m - omega * vx
        omega_dot = m_z_total / iz

        # Update Fz from latest accel estimate (so the NEXT step uses fresh loads).
        # Aero lift (per axle, ∝ v²) is applied on top of the quasi-static
        # transfer so high speed unloads the tyres — same as the load page.
        ax_body = vx_dot - omega * vy   # body-frame longitudinal accel
        ay_body = vy_dot + omega * vx   # body-frame lateral accel
        self.state.fz = fz_with_aero_lift(
            self.params, vertical_loads(self.params, ax_body, ay_body), abs(vx)
        )

        return np.array([vx_dot, vy_dot, omega_dot])
