"""Per-wheel motor torque servo.

The Phase-1 strategies emit `wheel_speed_cmd[i]` per wheel; the dynamic
model can't directly assign wheel angular velocity (it's a state of the
ODE). So we wrap an internal PI controller around each wheel that converts
ω_cmd → motor torque, saturated by `motor_torque_max`. Phase 2 strategies
can extend `ControlCommand` to specify torque directly (bypassing this
servo) — for now all strategies route through here.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class WheelSpeedServo:
    """PI + command-rate feedforward on ω_wheel; output is motor torque [N·m].

    Feedforward: a pure PI tracking a speed *ramp* needs a sustained error to
    generate the acceleration torque (the wheel drags the whole vehicle — the
    reflected inertia m·r²/4 ≈ 45× the wheel's own). That lag charges the
    integrator, which after the ramp parks the vehicle a few % above target
    until drag bleeds it off. `ff_inertia · dω_cmd/dt` (low-passed) supplies
    the ramp torque openly so the PI only handles disturbances. This is a
    command-derivative feedforward — NOT the rejected r·Fx state feedback
    (see SimplifiedDynamicModel._compute_motor_torques), so it introduces no
    feedback path and cannot cause wheelspin.
    """
    kp: float = 200.0          # P gain [N·m / (rad/s)]
    ki: float = 50.0           # I gain
    integral_limit: float = 50.0
    torque_limit: float = 2000.0  # absolute output cap
    ff_inertia: float = 0.0    # feedforward inertia [kg·m²] (0 = off)
    ff_tau: float = 0.12       # low-pass on the cmd-rate estimate [s]

    integral: float = 0.0
    _cmd_prev: float | None = None
    _rate_f: float = 0.0

    def update(self, omega_actual: float, omega_cmd: float, dt: float,
               brake_active: bool = False) -> float:
        """Compute one step of motor torque.

        When `brake_active` is true the friction brake owns this wheel and the
        servo disengages completely (output 0, integrator cleared).

        Clamping the output to ≤ 0 is NOT enough: this is a *speed* servo, so
        while the brake pedal drags `ω_cmd` down the servo sees a large
        negative error and pours its full `torque_limit` of retarding torque in
        on top of the friction brake. That retarding branch has no pedal
        dependence, so any non-zero pedal produced the same maximum-effort stop
        (measured: 0.1 and 1.0 pedal both gave a 22.5 m / 12 s stop from
        60 km/h with all four wheels locked). Disengaging makes the friction
        torque the sole longitudinal authority during braking, which is what
        makes pedal travel mean something.

        The integrator is zeroed rather than frozen so releasing the brake
        doesn't dump a stale accumulated error back into the drivetrain.
        """
        if brake_active:
            self.integral = 0.0
            # Keep tracking the command so the feedforward derivative doesn't
            # spike on the first step after release.
            self._cmd_prev = omega_cmd
            self._rate_f = 0.0
            return 0.0

        err = omega_cmd - omega_actual
        # Command-rate feedforward (filtered numerical derivative of ω_cmd).
        if self._cmd_prev is not None and dt > 1e-9:
            rate_raw = (omega_cmd - self._cmd_prev) / dt
        else:
            rate_raw = 0.0
        self._cmd_prev = omega_cmd
        a = dt / (self.ff_tau + dt)
        self._rate_f += a * (rate_raw - self._rate_f)
        t_ff = self.ff_inertia * self._rate_f

        t_unsat = self.kp * err + self.ki * self.integral + t_ff
        # Output saturation
        t = t_unsat
        if t > self.torque_limit:
            t = self.torque_limit
        elif t < -self.torque_limit:
            t = -self.torque_limit
        # Anti-windup by conditional integration (integrator clamping): freeze
        # the integrator while the output is saturated AND the error would
        # push it further into saturation.
        if t == t_unsat or err * t_unsat < 0.0:
            self.integral += err * dt
            if self.integral > self.integral_limit:
                self.integral = self.integral_limit
            elif self.integral < -self.integral_limit:
                self.integral = -self.integral_limit
        return t

    def reset(self) -> None:
        self.integral = 0.0
        self._cmd_prev = None
        self._rate_f = 0.0
