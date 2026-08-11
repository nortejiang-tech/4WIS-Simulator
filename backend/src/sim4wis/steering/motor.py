"""Assist motor — a torque source with the limits that actually bite.

Modelled at torque level, not as a field-oriented drive. At vehicle-dynamics
timescales the current loop is an order of magnitude faster than anything it
feeds, so a first-order torque response reproduces the dynamics that matter;
what does not come for free, and what this module is really about, is the
**limits**.

Three of them, and each shows up as a different complaint in a real car:

    peak torque        assist saturates — heavy at the end of a parking turn
    torque-speed       back-EMF leaves nothing at speed, so a fast evasive
                       input outruns the assist and the wheel goes heavy
                       exactly when the driver needs it lightest
    thermal derate     the third parking manoeuvre in a row is heavier than
                       the first

A motor chosen only on continuous torque passes a steady-state check and fails
all three. Reporting the operating point (torque, speed, thermal state) is what
lets the sizing phase build a duty cycle rather than a single number.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from sim4wis.steering.params import MotorParams

_RPM_TO_RAD_S = 2.0 * math.pi / 60.0


@dataclass
class MotorState:
    """Everything the sizing phase needs, per step."""

    torque: float = 0.0            # 实际输出转矩 [N·m]（电机轴）
    command: float = 0.0           # 请求转矩 [N·m]
    speed: float = 0.0             # 电机角速度 [rad/s]
    available: float = 0.0         # 当前工况下可用峰值 [N·m]
    heat: float = 0.0              # 归一化热状态（1.0 = 连续转矩长期工作点）
    #: True while the request exceeds what the motor can deliver — the flag a
    #: sizing review looks for, and the reason a manoeuvre felt wrong.
    saturated: bool = False

    @property
    def current(self) -> float:
        """Placeholder for the current channel; filled by `Motor` with K_t."""
        return self._current

    _current: float = 0.0


class Motor:
    """First-order torque response inside a torque-speed-thermal envelope."""

    def __init__(self, params: MotorParams) -> None:
        self.p = params
        self.state = MotorState()

    def reset(self) -> None:
        self.state = MotorState()

    # ---- envelope ----------------------------------------------------------

    def available_torque(self, speed: float, heat: float) -> float:
        """Peak torque available right now [N·m].

        Falls linearly to zero at no-load speed (the back-EMF envelope), then
        is scaled by thermal derating. Both are floors, not suggestions: a
        command above this is simply not delivered.
        """
        p = self.p
        w_max = max(p.no_load_speed_rpm * _RPM_TO_RAD_S, 1e-6)
        envelope = p.peak_torque * max(0.0, 1.0 - abs(speed) / w_max)
        # Derating starts once the thermal state passes the continuous point
        # and is complete one unit later.
        excess = min(max(heat - 1.0, 0.0), 1.0)
        derate = 1.0 - (1.0 - p.thermal_derate) * excess
        return envelope * derate

    # ---- step --------------------------------------------------------------

    def step(self, dt: float, command: float, speed: float) -> MotorState:
        """Advance one step. `command` and the result are motor-shaft torque."""
        p = self.p
        s = self.state
        dt = max(float(dt), 1e-9)

        available = self.available_torque(speed, s.heat)
        target = max(-available, min(available, float(command)))

        # First-order torque response. tau = 1/(2*pi*f_bw) is the usual
        # small-signal equivalent of a closed current loop.
        tau_e = 1.0 / max(2.0 * math.pi * p.bandwidth_hz, 1e-6)
        alpha = 1.0 - math.exp(-dt / tau_e)
        torque = s.torque + (target - s.torque) * alpha

        # Thermal state chases (tau/tau_continuous)^2 — heating goes as I^2R
        # and torque is proportional to current.
        load = (torque / max(p.continuous_torque, 1e-6)) ** 2
        beta = 1.0 - math.exp(-dt / max(p.thermal_tau_s, 1e-6))
        heat = s.heat + (load - s.heat) * beta

        self.state = MotorState(
            torque=torque,
            command=float(command),
            speed=float(speed),
            available=available,
            heat=heat,
            saturated=abs(float(command)) > available + 1e-9,
            _current=torque / max(p.torque_constant, 1e-9),
        )
        return self.state
