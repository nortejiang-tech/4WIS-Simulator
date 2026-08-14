"""Multibody 4WIS vehicle model (step 19) — 14 DOF, fixed-step RK4.

Degrees of freedom (14):
    * Sprung body 6:  x, y (planar) · ψ yaw · z heave · φ roll · θ pitch
    * Suspension  4:  per-corner unsprung vertical position z_u[i]
    * Wheel spin  4:  ω_w[i]

This is the high-fidelity model. Over `SimplifiedDynamicModel` it adds:
    * Real vertical dynamics — heave/bounce over bumps (quarter-car per corner
      with tyre vertical stiffness + suspension spring/damper).
    * Roll & pitch as actual DOF → load transfer is *dynamic* (lags lateral /
      longitudinal acceleration through the roll/pitch modes) instead of the
      quasi-static `load_transfer.vertical_loads()` used by the simplified model.
    * Bump-steer driven by **actual suspension deflection** (not the road-height
      hack the simplified model uses).

Design choice: implemented directly in NumPy (no Pinocchio / PyDy dependency)
to stay consistent with the rest of the codebase and the real-time budget.
At dt_sim = 5 ms the stiffest mode (wheel hop ≈ 11 Hz) is well within RK4
stability.

Frames & signs (see docs/design.md §2):
    z up, roll φ + = right side down, pitch θ + = nose up. Body origin = axle
    midpoint; CG is at body-x = L/2 − cg_to_front.
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
from sim4wis.vehicle.steering_link import (
    front_axle_step,
    idle_channels,
    make_steering_plant,
)
from sim4wis.steering.tracking.coupling import (
    corner_tracking_step,
    make_corner_trackers,
)
from sim4wis.vehicle.kingpin import kingpin_torque
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
    rotate_wheel_forces_to_body,
    semi_implicit_wheel_spin,
    static_toe_offsets,
    wheel_slip_kinematics,
)
from sim4wis.vehicle.tire import TireModel, make_tire
from sim4wis.vehicle.wheel_servo import WheelSpeedServo

G = 9.81

# RK4 state vector layout (length 24)
#   0:x 1:y 2:ψ 3:z 4:roll 5:pitch 6..9:z_u  10:vx 11:vy 12:ω 13:vz
#   14:roll_rate 15:pitch_rate 16..19:zu_dot 20..23:ω_w
IX, IY, IPSI, IZ, IROLL, IPITCH = 0, 1, 2, 3, 4, 5
IZU = slice(6, 10)
IVX, IVY, IOMEGA, IVZ, IROLLR, IPITCHR = 10, 11, 12, 13, 14, 15
IZUD = slice(16, 20)
IWW = slice(20, 24)


class MultiBodyModel(VehicleModel):
    """High-fidelity 3D body + 4 quarter-car suspensions + 4 wheel spins."""

    def __init__(self, params: VehicleParams, tire: TireModel | None = None) -> None:
        super().__init__(params)
        self.tire = tire or make_tire(params)
        t_max = getattr(params, "motor_torque_max", 2000.0)
        self.iw = getattr(params, "wheel_inertia", 1.2)
        # Reflected vehicle inertia per wheel (same feedforward as the
        # simplified model — see WheelSpeedServo docstring).
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

        self._precompute()

        # Internal velocity / unsprung states not held in VehicleState.
        self._vz = 0.0
        self._roll_rate = 0.0
        self._pitch_rate = 0.0
        self._zu = np.zeros(N_WHEELS)
        self._zu_dot = np.zeros(N_WHEELS)
        self._delta_act = np.zeros(N_WHEELS)  # steering actuator state
        # See vehicle/steering_link.py — None unless enabled and the
        # architecture has a mechanical front axle.
        self._steering, self._mech_ratio = make_steering_plant(params)
        #: Front-axle telemetry for the recorder. NaN until a plant runs,
        #: because a run without one has no hand torque — not a hand torque
        #: of zero.
        self.steering_channels = idle_channels()
        self._prev_hand = 0.0
        # Per-corner angle-tracking layer (see dynamic.py for the contract).
        self._tracking = make_corner_trackers(params)
        self._prev_delta_cmd = np.zeros(N_WHEELS)
        # Filtered friction-brake command per wheel (first-order, brake_tau).
        self._brake_f = np.zeros(N_WHEELS)

        # Diagnostics
        self.slip_alpha = np.zeros(N_WHEELS)
        self.slip_kappa = np.zeros(N_WHEELS)
        self.tire_fx = np.zeros(N_WHEELS)
        self.tire_fy = np.zeros(N_WHEELS)
        self.tire_mz = np.zeros(N_WHEELS)

        self.state.fz = self._corner_static_full.copy()

    def _precompute(self) -> None:
        p = self.params
        self.m = p.mass
        self.m_u = getattr(p, "unsprung_mass", 55.0)
        self.m_s = self.m - N_WHEELS * self.m_u
        L = p.wheelbase
        a = p.cg_to_front
        b = L - a
        # Sprung static corner loads (front pair / rear pair)
        wf = self.m_s * G * b / (2.0 * L)
        wr = self.m_s * G * a / (2.0 * L)
        self._W = np.array([wf, wf, wr, wr])              # FL,FR,RL,RR
        self._corner_static_full = self._W + self.m_u * G  # tyre vertical static
        self.x_cg = L / 2.0 - a                            # CG x in body frame
        self.h_cg = getattr(p, "cg_height", 0.55)

        self.k_s = p.suspension.spring_rate
        self.c_s = p.suspension.damper_rate
        self.arb = p.suspension.anti_roll_rate
        # Roll-couple distribution. With the fraction unset the bar stays a
        # pure body moment (legacy: it damps roll but transfers no load, so it
        # cannot affect handling balance); with it set, the same total stiffness
        # is delivered per corner and split between the axles.
        eps_f = float(getattr(p.suspension, "roll_stiffness_front_frac", 0.0))
        if eps_f > 0.0:
            eps_f = min(max(eps_f, 0.0), 1.0)
            self._arb_f = self.arb * eps_f
            self._arb_r = self.arb * (1.0 - eps_f)
            self._arb_body = 0.0          # the corner forces carry it now
        else:
            self._arb_f = self._arb_r = 0.0
            self._arb_body = self.arb
        self.k_t = getattr(p, "tire_vertical_stiffness", 280_000.0)
        self.bump_steer = getattr(p, "bump_steer_coeff", 0.0)

        # Roll / pitch inertia — box approximation from sprung mass + dimensions.
        L_body = 1.5 * L
        W_body = max(p.track_front, p.track_rear)
        H_body = 2.0 * self.h_cg
        self.I_roll = self.m_s * (W_body**2 + H_body**2) / 12.0
        self.I_pitch = self.m_s * (L_body**2 + H_body**2) / 12.0

        self._wheels = p.wheel_positions_body()           # (4,2)
        self._toe_sign = np.sign(self._wheels[:, 1])      # +1 left, −1 right
        self._toe = static_toe_offsets(p)                 # static alignment toe
        # Axle cornering-stiffness split, absorbed as a slip-angle scale.
        self._alpha_scale = axle_cornering_scale(p)
        # Measured K&C, or a synthesised curve from the legacy bump_steer_coeff.
        self._kc = resolve_kc(p)
        # Last step's tyre forces, for the compliance terms (see kc_wheel_offsets
        # on why a one-step lag is the right call here).
        self._kc_fx = np.zeros(N_WHEELS)
        self._kc_fy = np.zeros(N_WHEELS)
        self._kc_fz = self._corner_static_full.copy()
        self._kc_mz = np.zeros(N_WHEELS)
        # K&C increments actually applied this step (diagnostics).
        self.kc_toe = np.zeros(N_WHEELS)
        self.kc_camber = np.zeros(N_WHEELS)
        # Relaxation-lagged slip states (see model_core.relax_slip).
        self._alpha_lag = np.zeros(N_WHEELS)
        self._kappa_lag = np.zeros(N_WHEELS)
        # Aero (drag handled via body_resistance_force; lift split per axle).
        self._q_aero = 0.5 * float(p.air_density) * float(p.frontal_area)
        self._cl_f = float(p.aero_lift_coeff_front)
        self._cl_r = float(p.aero_lift_coeff_rear)

    # ---- API ---------------------------------------------------------------

    def reset(self, init: VehicleState | None = None) -> None:
        super().reset(init)
        for s in self.servos:
            s.reset()
        self._vz = 0.0
        self._roll_rate = 0.0
        self._pitch_rate = 0.0
        self._zu = np.zeros(N_WHEELS)
        self._zu_dot = np.zeros(N_WHEELS)
        self._delta_act = np.zeros(N_WHEELS)
        if self._steering is not None:
            self._steering.reset()
        if self._tracking is not None:
            for tracker in self._tracking[0].values():
                tracker.reset()
        self._prev_delta_cmd = np.zeros(N_WHEELS)
        self._prev_hand = 0.0
        self.slip_alpha[:] = 0.0
        self.slip_kappa[:] = 0.0
        self.state.fz = self._corner_static_full.copy()
        self.state.susp_defl = np.zeros(N_WHEELS)

    def step(self, dt: float, cmd: ControlCommand, env: EnvironmentState) -> VehicleState:
        s = self.state
        p = self.params

        # Per-wheel road height + μ at the current planar pose (held over the step).
        road_z, wheel_mus = self._road_and_mu(env)
        s.mu_avg = float(np.mean(wheel_mus))
        t_motor = self._motor_torques(cmd, s.wheel_omega, dt)
        # Steering actuator: actual δ tracks δ_cmd with lag + rate limit.
        # With a mechanical front axle and the plant enabled the front pair goes
        # through the real steering system instead — see vehicle/steering_link.py.
        if self._steering is None:
            self._delta_act = steer_actuator(
                self._delta_act, cmd.delta_cmd, dt,
                getattr(p, "steer_tau", 0.06), getattr(p, "steer_rate_max", 8.0),
            )
        else:
            self._delta_act[2:] = steer_actuator(
                self._delta_act[2:], cmd.delta_cmd[2:], dt,
                getattr(p, "steer_tau", 0.06), getattr(p, "steer_rate_max", 8.0),
            )
            delta_f, self._prev_hand, self.steering_channels = front_axle_step(
                self._steering, self._mech_ratio, cmd.delta_cmd, dt,
                prev_hand=self._prev_hand, rack_force=float(np.sum(s.rack_force[:2])),
                speed_ms=float(s.vx),
            )
            self._delta_act[0] = self._delta_act[1] = delta_f
            if self._tracking is not None:
                trackers, per_wheel = self._tracking
                tracked, track_channels = corner_tracking_step(
                    trackers, per_wheel, cmd.delta_cmd, dt,
                    rack_forces=s.rack_force, speed_ms=float(s.vx),
                    pinion_radius=float(p.pinion_radius),
                )
                for corner in trackers:
                    self._delta_act[corner] = tracked[corner]
                self.steering_channels.update(track_channels)
                self._prev_delta_cmd = cmd.delta_cmd.copy()
        delta_cmd = self._delta_act.astype(float)

        if float(getattr(p, "tire_relax_length", 0.0)) > 0.0:
            kin0 = wheel_slip_kinematics(
                vx=s.vx, vy=s.vy, yaw_rate=s.yaw_rate,
                wheel_omega=s.wheel_omega, delta=s.delta,
                wheel_positions_body=self._wheels, tire_radius=p.tire_radius)
            self._alpha_lag = relax_slip(
                self._alpha_lag, self._alpha_scale * kin0.alpha, kin0.vx_wheel,
                float(p.tire_relax_length), dt)

        y0 = np.empty(24)
        y0[IX], y0[IY], y0[IPSI] = s.x, s.y, s.psi
        y0[IZ], y0[IROLL], y0[IPITCH] = s.z, s.roll, s.pitch
        y0[IZU] = self._zu
        y0[IVX], y0[IVY], y0[IOMEGA] = s.vx, s.vy, s.yaw_rate
        y0[IVZ], y0[IROLLR], y0[IPITCHR] = self._vz, self._roll_rate, self._pitch_rate
        y0[IZUD] = self._zu_dot
        y0[IWW] = s.wheel_omega

        def f(y):
            return self._derivatives(y, delta_cmd, wheel_mus, road_z)

        k1 = f(y0)
        k2 = f(y0 + 0.5 * dt * k1)
        k3 = f(y0 + 0.5 * dt * k2)
        k4 = f(y0 + dt * k3)
        y1 = y0 + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        s.x, s.y, s.psi = float(y1[IX]), float(y1[IY]), float(y1[IPSI])
        s.z, s.roll, s.pitch = float(y1[IZ]), float(y1[IROLL]), float(y1[IPITCH])
        self._zu = y1[IZU]
        # Body-frame specific force (accelerometer reading at the CG).
        s.ax = float(y1[IVX] - y0[IVX]) / dt - float(y1[IOMEGA]) * float(y1[IVY])
        s.ay = float(y1[IVY] - y0[IVY]) / dt + float(y1[IOMEGA]) * float(y1[IVX])
        s.vx, s.vy, s.yaw_rate = float(y1[IVX]), float(y1[IVY]), float(y1[IOMEGA])
        self._vz = float(y1[IVZ])
        self._roll_rate = float(y1[IROLLR])
        self._pitch_rate = float(y1[IPITCHR])
        self._zu_dot = y1[IZUD]
        # y1[IWW] == y0[IWW]: wheel spin is held during the body RK4 and
        # advanced semi-implicitly inside _finalize (stiff-stable).

        # Derived / reported quantities (recompute forces at the final state).
        # The friction brake is applied inside _finalize, where the transferred
        # loads and per-wheel ground speeds it needs are available.
        self._finalize(s, delta_cmd, road_z, wheel_mus, t_motor, dt, cmd)
        s.t += dt
        return s

    # ---- internals ---------------------------------------------------------

    def _motor_torques(self, cmd: ControlCommand, omega_actual: np.ndarray, dt: float) -> np.ndarray:
        """Drive torque per wheel (mirrors the dynamic model — see there)."""
        if str(getattr(self.params, "longitudinal_mode", "speed_servo")) == "torque":
            for servo in self.servos:
                servo.reset()
            drive = np.asarray(cmd.drive_torque_cmd, dtype=np.float64).reshape(N_WHEELS)
            return np.where(np.asarray(cmd.brake_cmd) > 0.02, 0.0, drive)

        out = np.zeros(N_WHEELS)
        for i in range(N_WHEELS):
            out[i] = self.servos[i].update(
                omega_actual=float(omega_actual[i]),
                omega_cmd=float(cmd.wheel_speed_cmd[i]),
                dt=dt,
                brake_active=bool(cmd.brake_cmd[i] > 0.02),
            )
        return out

    def _road_and_mu(self, env: EnvironmentState) -> tuple[np.ndarray, np.ndarray]:
        s = self.state
        cp, sp = math.cos(s.psi), math.sin(s.psi)
        scene = getattr(env, "scene", None)
        base = env.mu if scene is None else getattr(scene, "base_mu", env.mu)
        road_z = np.zeros(N_WHEELS)
        mus = np.full(N_WHEELS, base)
        if scene is not None:
            for i in range(N_WHEELS):
                xw = s.x + cp * self._wheels[i, 0] - sp * self._wheels[i, 1]
                yw = s.y + sp * self._wheels[i, 0] + cp * self._wheels[i, 1]
                local = scene.wheel_env(np.array([xw, yw]))
                road_z[i] = local.ground_z
                if local.mu_override is not None:
                    mus[i] = local.mu_override
        return road_z, mus

    def _suspension_forces(self, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Return (F_spring up on body, F_tire_z up on unsprung, comp) per corner."""
        xw = self._wheels[:, 0]
        yw = self._wheels[:, 1]
        z, roll, pitch = y[IZ], y[IROLL], y[IPITCH]
        vz, rr, pr = y[IVZ], y[IROLLR], y[IPITCHR]
        z_u = y[IZU]
        zu_dot = y[IZUD]

        z_corner = z + xw * pitch + yw * roll
        z_corner_rate = vz + xw * pr + yw * rr
        comp = z_u - z_corner                      # suspension compression vs static
        comp_rate = zu_dot - z_corner_rate
        f_spring = self._W + self.k_s * comp + self.c_s * comp_rate

        # Anti-roll bars, as per-corner forces rather than a bare body moment.
        #
        # A bar of roll stiffness K_φ at an axle delivers its couple as ±K_φ·φ/t
        # at the two wheels. Summing the moment back gives Σ y·f_arb = −K_φ·φ,
        # i.e. exactly the body-level term this replaces — but now the force
        # travels down through the unsprung mass into the tyre load, which is
        # the whole point: a bar that only ever appears as a body moment damps
        # roll without transferring any load, so it cannot influence handling
        # balance at all.
        if self._arb_f > 0.0 or self._arb_r > 0.0:
            f_spring = f_spring + self._arb_corner(roll)

        f_spring = np.maximum(f_spring, 0.0)

        f_tire_z = np.maximum(self._corner_static_full + self.k_t * (self._road_z - z_u), 0.0)
        return f_spring, f_tire_z, comp

    def _arb_corner(self, roll: float) -> np.ndarray:
        """Per-corner anti-roll-bar force [N], same sign convention as f_spring.

        f_i = −K_φ,axle · φ · sign(y_i) / t_axle, so that
        Σ y_i·f_i = −K_φ·φ — the same restoring couple the body-level term
        produced, now routed through the corners.
        """
        tf = max(float(self.params.track_front), 1e-6)
        tr = max(float(self.params.track_rear), 1e-6)
        k = np.array([self._arb_f / tf, self._arb_f / tf,
                      self._arb_r / tr, self._arb_r / tr])
        return -k * float(roll) * self._toe_sign

    def _derivatives(self, y, delta_cmd, wheel_mus, road_z) -> np.ndarray:
        self._road_z = road_z  # used inside _suspension_forces
        p = self.params
        xw = self._wheels[:, 0]
        yw = self._wheels[:, 1]

        f_spring, f_tire_z, comp = self._suspension_forces(y)

        # Static toe + bump-steer from actual suspension compression.
        # K&C: kinematic toe/camber from actual suspension travel, plus the
        # compliance deflection under last step's tyre loads.
        kc_toe, kc_camber = kc_wheel_offsets(
            self._kc, jounce=comp, fx=self._kc_fx, fy=self._kc_fy,
            fz=self._kc_fz, mz=self._kc_mz)
        delta = np.clip(delta_cmd + self._toe + kc_toe, -p.steer_limit, p.steer_limit)

        vx, vy, omega = y[IVX], y[IVY], y[IOMEGA]
        wheel_omega = y[IWW]

        fx_body = np.zeros(N_WHEELS)
        fy_body = np.zeros(N_WHEELS)
        fx_wheel = np.zeros(N_WHEELS)
        kin = wheel_slip_kinematics(
            vx=float(vx),
            vy=float(vy),
            yaw_rate=float(omega),
            wheel_omega=wheel_omega,
            delta=delta,
            wheel_positions_body=self._wheels,
            tire_radius=p.tire_radius,
        )

        fy_wheel = np.zeros(N_WHEELS)
        # Camber thrust as an equivalent slip-angle offset (same absorption as
        # the load page / simplified dynamic model); diagnostics keep true α.
        alpha_camber = camber_thrust_alpha_offset(p, f_tire_z, extra_camber=kc_camber)
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
        mu_stage = load_sensitive_mu(p, wheel_mus, f_tire_z)
        for i in range(N_WHEELS):
            alpha = float(self._alpha_lag[i] / max(float(self._alpha_scale[i]), 1e-9)
                          if use_lag else kin.alpha[i])
            kappa = float(kin.kappa[i])
            fx, fy, mz = self.tire.forces(
                float(self._alpha_scale[i]) * alpha + float(alpha_camber[i]),
                kappa, float(f_tire_z[i]), float(mu_stage[i])
            )
            fx_wheel[i] = fx
            fy_wheel[i] = fy
            self.slip_alpha[i] = alpha
            self.slip_kappa[i] = kappa
            self.tire_fx[i] = fx
            self.tire_fy[i] = fy
            self.tire_mz[i] = mz

        fx_body, fy_body = rotate_wheel_forces_to_body(fx_wheel, fy_wheel, delta)

        fx_total = float(np.sum(fx_body))
        fy_total = float(np.sum(fy_body))
        mz_total = float(np.sum(xw * fy_body - yw * fx_body + self.tire_mz))

        # Longitudinal grade gravity — same engineering approximation as the
        # simplified model: the road-height difference front↔rear gives a body
        # slope, and climbing it costs −m·g·sin(grade) along body X. Without
        # this the Slope disturbance raised the wheels but never slowed the car.
        z_front = 0.5 * (road_z[0] + road_z[1])
        z_rear = 0.5 * (road_z[2] + road_z[3])
        grade_long = math.atan2(z_front - z_rear, p.wheelbase)
        fx_total += -self.m * G * math.sin(grade_long)

        # Aero drag + rolling resistance (shared coefficients with load page).
        fx_total += body_resistance_force(p, vx)

        # Planar (whole-vehicle mass) with Coriolis terms.
        m = self.m
        vx_dot = fx_total / m + omega * vy
        vy_dot = fy_total / m - omega * vx
        omega_dot = mz_total / p.inertia_z
        ax_body = vx_dot - omega * vy
        ay_body = vy_dot + omega * vx

        # Per-axle aero lift (∝ v², positive = unloads the axle) acts on the
        # sprung body: heave force + pitch moment; the suspension then passes
        # the change down to the tyre loads.
        q_dyn = self._q_aero * vx * vx
        lift_f = self._cl_f * q_dyn
        lift_r = self._cl_r * q_dyn

        # Vertical body (sprung mass) — gravity balanced by spring statics.
        vz_dot = (float(np.sum(f_spring)) + lift_f + lift_r - self.m_s * G) / self.m_s

        # Roll: spring moment + lateral-accel roll moment − ARB.
        q_roll = float(np.sum(yw * f_spring))
        m_roll_ext = self.m_s * ay_body * self.h_cg
        roll_dd = (q_roll + m_roll_ext - self._arb_body * y[IROLL]) / self.I_roll

        # Pitch: spring moment − gravity pitch moment + long-accel pitch moment
        # + per-axle aero lift moment (front lift at +L/2 pitches nose up).
        q_pitch = float(np.sum(xw * f_spring)) - self.m_s * G * self.x_cg
        m_pitch_ext = self.m_s * ax_body * self.h_cg
        m_pitch_aero = 0.5 * p.wheelbase * (lift_f - lift_r)
        pitch_dd = (q_pitch + m_pitch_ext + m_pitch_aero) / self.I_pitch

        # Unsprung vertical (quarter-car).
        zu_dd = (f_tire_z - f_spring - self.m_u * G) / self.m_u

        # Wheel spin is NOT integrated here — its linearised mode is too stiff
        # for RK4 at this dt at low-mid speed (see model_core.
        # semi_implicit_wheel_spin). ω is held over the body step and advanced
        # semi-implicitly in _finalize.

        # Pose kinematics.
        cp, sp = math.cos(y[IPSI]), math.sin(y[IPSI])
        dy = np.zeros(24)
        dy[IX] = vx * cp - vy * sp
        dy[IY] = vx * sp + vy * cp
        dy[IPSI] = omega
        dy[IZ] = y[IVZ]
        dy[IROLL] = y[IROLLR]
        dy[IPITCH] = y[IPITCHR]
        dy[IZU] = y[IZUD]
        dy[IVX] = vx_dot
        dy[IVY] = vy_dot
        dy[IOMEGA] = omega_dot
        dy[IVZ] = vz_dot
        dy[IROLLR] = roll_dd
        dy[IPITCHR] = pitch_dd
        dy[IZUD] = zu_dd
        dy[IWW] = 0.0
        return dy

    def _finalize(self, s: VehicleState, delta_cmd, road_z, wheel_mus, t_motor, dt, cmd) -> None:
        # Recompute suspension state + reported Fz / steering torque at final y.
        p = self.params
        y = np.empty(24)
        y[IZ], y[IROLL], y[IPITCH] = s.z, s.roll, s.pitch
        y[IZU] = self._zu
        y[IVZ], y[IROLLR], y[IPITCHR] = self._vz, self._roll_rate, self._pitch_rate
        y[IZUD] = self._zu_dot
        self._road_z = road_z
        f_spring, f_tire_z, comp = self._suspension_forces(y)
        s.fz = f_tire_z
        s.susp_defl = comp
        # Apply static toe + bump-steer to the reported actual steer angle too.
        kc_toe, kc_camber = kc_wheel_offsets(
            self._kc, jounce=comp, fx=self._kc_fx, fy=self._kc_fy,
            fz=self._kc_fz, mz=self._kc_mz)
        self.kc_toe[:], self.kc_camber[:] = kc_toe, kc_camber
        delta = np.clip(delta_cmd + self._toe + kc_toe, -p.steer_limit, p.steer_limit)
        s.delta[:] = delta

        # Wheel-spin update at the final body state (semi-implicit, stiff-
        # stable) + final consistent tyre forces for the diagnostics.
        kin = wheel_slip_kinematics(
            vx=s.vx, vy=s.vy, yaw_rate=s.yaw_rate,
            wheel_omega=s.wheel_omega, delta=delta,
            wheel_positions_body=self._wheels,
            tire_radius=p.tire_radius,
        )
        alpha_camber = camber_thrust_alpha_offset(p, f_tire_z, extra_camber=kc_camber)
        # Lockup is a friction-limited event, so it must see the same effective
        # mu the tyre forces do: the heavily loaded front wheel has less grip
        # than nominal, which is exactly what decides whether it locks first.
        mu_eff = load_sensitive_mu(p, wheel_mus, f_tire_z)
        t_brake, locked, self._brake_f = friction_brake_torques(
            brake_cmd=cmd.brake_cmd,
            brake_filtered=self._brake_f,
            wheel_omega=s.wheel_omega,
            vx_wheel=kin.vx_wheel,
            fz=f_tire_z,
            mu=mu_eff,
            params=p,
            dt=dt,
        )
        s.wheel_locked = locked
        alpha_fed = (self._alpha_lag if float(getattr(p, "tire_relax_length", 0.0)) > 0.0
                     else self._alpha_scale * kin.alpha)
        omega_new, forces, kappa_new = semi_implicit_wheel_spin(
            dt=dt,
            wheel_omega=s.wheel_omega,
            vx_wheel=kin.vx_wheel,
            alpha=alpha_fed + alpha_camber,
            fz=f_tire_z,
            mu=mu_eff,
            torque=t_motor + t_brake,
            brake_torque=t_brake,
            tire=self.tire,
            tire_radius=p.tire_radius,
            wheel_inertia=self.iw,
        )
        s.wheel_omega[:] = omega_new
        self.slip_alpha[:] = kin.alpha
        self.slip_kappa[:] = kappa_new
        # Hand this step's tyre loads to next step's compliance terms.
        self._kc_fx, self._kc_fy = forces.fx.copy(), forces.fy.copy()
        self._kc_fz, self._kc_mz = f_tire_z.copy(), forces.mz.copy()
        # Friction budget from the forces that actually moved the vehicle.
        _store_grip(s, wheel_grip_state(
            fx=forces.fx, fy=forces.fy, fz=f_tire_z, mu=mu_eff,
            alpha=kin.alpha, kappa=kappa_new, vx_wheel=kin.vx_wheel, tire=self.tire,
        ))
        self.tire_fx[:] = forces.fx
        self.tire_fy[:] = forces.fy
        self.tire_mz[:] = forces.mz

        s.vehicle_icr_body = vehicle_icr_from_velocity(s.vx, s.vy, s.yaw_rate)
        s.torque_steer = kingpin_torque(
            fx=self.tire_fx, fy=self.tire_fy, mz=self.tire_mz, fz=s.fz,
            suspension=p.suspension, delta=s.delta,
            tire_radius=p.tire_radius,
        )
        s.wheel_pos_body = self._wheels
