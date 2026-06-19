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
    """First-order PI on ω_wheel; output is motor torque [N·m]."""
    kp: float = 200.0          # P gain [N·m / (rad/s)]
    ki: float = 50.0           # I gain
    integral_limit: float = 50.0
    torque_limit: float = 2000.0  # absolute output cap

    integral: float = 0.0

    def update(self, omega_actual: float, omega_cmd: float, dt: float) -> float:
        err = omega_cmd - omega_actual
        # Anti-windup: only integrate when not saturated
        self.integral += err * dt
        if self.integral > self.integral_limit:
            self.integral = self.integral_limit
        elif self.integral < -self.integral_limit:
            self.integral = -self.integral_limit
        t = self.kp * err + self.ki * self.integral
        # Output saturation
        if t > self.torque_limit:
            return self.torque_limit
        if t < -self.torque_limit:
            return -self.torque_limit
        return t

    def reset(self) -> None:
        self.integral = 0.0
