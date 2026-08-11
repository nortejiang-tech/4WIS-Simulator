"""Actuator sizing — turning manoeuvres into a specification.

The question a system engineer actually has to answer is not "what torque does
this car need" but "does *this* motor cover *this* car across everything it
will be asked to do, and by how much". Those are different questions, and only
the second one has an answer you can put in a supplier requirement.

So this walks a small library of worst cases, drives the steering plant through
each one, and reduces the result to the four numbers a motor is bought on:

    peak torque     what the end of a full-lock parking turn demands
    peak speed      what an evasive input demands — and where the back-EMF
                    envelope bites, which peak torque alone never shows
    RMS torque      the duty-cycle number thermal sizing is done against
    thermal state   after a repeated manoeuvre, because the third parking
                    turn in a row is not the first one

A motor chosen on continuous torque alone passes a steady-state check and fails
three of these. That is the entire reason this module drives a plant instead of
evaluating a formula.

**The load comes from the existing quasi-static model.** `sweep_load_analysis`
already computes rack force against steer angle, speed and mu, including the
parking terms; this interpolates that curve rather than inventing a second one,
so a sizing result and the load page cannot disagree about what the tyres are
doing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from sim4wis.core.state import VehicleParams
from sim4wis.steering import architecture as arch
from sim4wis.steering.assist import get as get_assist_map
from sim4wis.steering.plant import SteeringPlant
from sim4wis.vehicle.load_analysis import sweep_load_analysis

#: Above this saturated (or driver-limited) fraction a scenario's numbers stop
#: meaning anything.
#:
#: KNOWN LIMITATION, and it lands squarely on this module.
#:
#: The plant behaves in two of the three regimes. Where the actuator
#: comfortably covers the load it is clean (7.95 N.m at the hand wheel, 1639
#: rpm for full-lock parking on the default vehicle). Where it clearly cannot,
#: the driver-torque limit bounds it and the answer "this motor cannot do it"
#: comes out cleanly. In the **marginal band between** — a motor that almost
#: holds the load — it still rings, and that is exactly the band a sizing pass
#: operates in, so this is not a corner case here.
#:
#: Two contributors have been removed already: the imposed-angle convention
#: asserting an angle nothing could hold (fixed by the driver-torque limit),
#: and a damping schedule that read the torque available at this instant and so
#: depended on the motor speed it was controlling (fixed by capping on static
#: capability). What remains is the plant entering and leaving assist
#: saturation within a manoeuvre.
#:
#: The likely fix is to solve the torsion/load equilibrium when assist is
#: saturated rather than integrating through it. Until then this flag is the
#: guard: report that the motor does not cover the case, and refuse to say by
#: how much.
SATURATION_TRUST_LIMIT = 0.05


@dataclass(frozen=True)
class SizingScenario:
    """One worst case, and what it is there to expose."""

    id: str
    label: str
    exposes: str
    speed_kmh: float
    mu: float
    amplitude_deg: float          # 路面轮转角幅值
    rate_deg_s: float             # 方向盘转速（折算前的路面轮等效）
    cycles: int = 1
    #: Share of a representative duty cycle, used for the RMS torque only.
    duty_weight: float = 1.0


#: Deliberately small. Each entry earns its place by exposing something the
#: others cannot, and a library nobody reads is worse than four cases everyone
#: does.
DEFAULT_SCENARIOS: tuple[SizingScenario, ...] = (
    SizingScenario(
        "parking_full_lock", "原地全锁", "峰值转矩",
        speed_kmh=0.0, mu=0.9, amplitude_deg=35.0, rate_deg_s=25.0,
        duty_weight=0.05,
    ),
    SizingScenario(
        "parking_repeat", "连续挪车 ×3", "热降额",
        speed_kmh=0.0, mu=0.9, amplitude_deg=35.0, rate_deg_s=25.0, cycles=3,
        duty_weight=0.05,
    ),
    SizingScenario(
        "low_speed_manoeuvre", "低速大转角", "综合负载",
        speed_kmh=20.0, mu=0.9, amplitude_deg=25.0, rate_deg_s=20.0,
        duty_weight=0.20,
    ),
    SizingScenario(
        "evasive_at_speed", "高速紧急避让", "峰值转速 / 反电动势包络",
        speed_kmh=100.0, mu=0.9, amplitude_deg=4.0, rate_deg_s=90.0,
        duty_weight=0.70,
    ),
)


@dataclass
class ScenarioResult:
    scenario: SizingScenario
    peak_motor_torque: float = 0.0
    peak_motor_speed: float = 0.0
    rms_motor_torque: float = 0.0
    peak_power_w: float = 0.0
    peak_heat: float = 0.0
    peak_rack_force: float = 0.0
    peak_hand_torque: float = 0.0
    #: Fraction of the run where the motor could not deliver what was asked.
    saturated_fraction: float = 0.0
    #: Fraction of the run where holding the commanded angle needed more than
    #: the driver-torque limit — i.e. the manoeuvre was not performed as asked.
    hand_limited_fraction: float = 0.0
    #: True when the run spent long enough past capability that the trace is
    #: not physical. See `SATURATION_TRUST_LIMIT`.
    beyond_capability: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.scenario.id,
            "label": self.scenario.label,
            "exposes": self.scenario.exposes,
            "peak_motor_torque_nm": round(self.peak_motor_torque, 3),
            "peak_motor_speed_rpm": round(self.peak_motor_speed * 60 / (2 * math.pi), 1),
            "rms_motor_torque_nm": round(self.rms_motor_torque, 3),
            "peak_power_w": round(self.peak_power_w, 1),
            "peak_thermal_state": round(self.peak_heat, 3),
            "peak_rack_force_n": round(self.peak_rack_force, 1),
            "peak_hand_torque_nm": round(self.peak_hand_torque, 2),
            "saturated_fraction": round(self.saturated_fraction, 3),
            "hand_limited_fraction": round(self.hand_limited_fraction, 3),
            "beyond_capability": self.beyond_capability,
            **({"note": "电机能力不足，饱和后的数值不可信 —— 只能读出"
                        "「不够」，读不出「差多少」"} if self.beyond_capability else {}),
        }


@dataclass
class ActuatorRequirement:
    """The specification, plus whether the fitted motor meets it."""

    peak_torque: float = 0.0
    peak_speed: float = 0.0
    rms_torque: float = 0.0
    peak_power_w: float = 0.0
    peak_heat: float = 0.0
    driven_by: dict[str, str] = field(default_factory=dict)
    scenarios: list[ScenarioResult] = field(default_factory=list)
    verdict: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "requirement": {
                "peak_torque_nm": round(self.peak_torque, 3),
                "peak_speed_rpm": round(self.peak_speed * 60 / (2 * math.pi), 1),
                "rms_torque_nm": round(self.rms_torque, 3),
                "peak_power_w": round(self.peak_power_w, 1),
                "peak_thermal_state": round(self.peak_heat, 3),
            },
            "driven_by": self.driven_by,
            "scenarios": [s.to_dict() for s in self.scenarios],
            "verdict": self.verdict,
        }


def _rack_force_curve(params: VehicleParams, speed_kmh: float, mu: float):
    """Interpolator: |road wheel angle| [rad] → rack force [N] at this speed."""
    limit = float(params.steer_limit)
    angles = list(np.linspace(0.0, limit, 25))
    data = sweep_load_analysis(
        params, speeds=[speed_kmh / 3.6], angles=angles, wheel_index=0, mu=mu,
    )
    xs, ys = [], []
    for row in data["rows"]:
        xs.append(abs(float(row["delta"])))
        ys.append(abs(float(row["rack_force"])))
    order = np.argsort(xs)
    xs = np.asarray(xs)[order]
    ys = np.asarray(ys)[order]
    # Both front tie rods react on the same rack.
    return lambda a: 2.0 * float(np.interp(abs(a), xs, ys))


def run_scenario(
    params: VehicleParams,
    scenario: SizingScenario,
    *,
    dt: float = 0.002,
) -> ScenarioResult:
    """Drive the plant through one worst case and reduce it to numbers."""
    sys_params = params.steering_system
    architecture = arch.get(sys_params.architecture)
    plant = SteeringPlant(
        params=sys_params,
        assist_map=get_assist_map(sys_params.assist_map),
        pinion_radius=float(params.pinion_radius),
        motor_gear_ratio=float(architecture.motor_gear_ratio),
    )
    plant.reset()

    from sim4wis.controller.steering_feel import low_speed_gear_ratio

    ratio = max(float(low_speed_gear_ratio(params)), 1e-6)
    rack_of = _rack_force_curve(params, scenario.speed_kmh, scenario.mu)

    amp = math.radians(scenario.amplitude_deg)
    rate = math.radians(scenario.rate_deg_s)
    quarter = amp / max(rate, 1e-6)          # time to go lock-to-centre
    period = 4.0 * quarter
    total = period * scenario.cycles
    n = max(int(total / dt), 10)

    res = ScenarioResult(scenario=scenario)
    sq_sum = 0.0
    saturated = 0
    limited = 0
    speed_ms = scenario.speed_kmh / 3.6

    for i in range(n):
        t = i * dt
        # Triangular sweep at the stated rate — a constant-rate input is what
        # a steering robot does and what a rate-limited actuator is sized
        # against, unlike a sine whose peak rate is momentary.
        #
        # It starts at centre. Starting at an extreme puts a full-lock step on
        # the column at t = 0, which is not a manoeuvre any car performs and
        # which this model duly answered with 64 000 rpm and 1189 N.m at the
        # hand wheel — a reminder that a driver profile is part of the physics.
        phase = (t % period) / period
        if phase < 0.25:
            tri = phase / 0.25                     # 0 → +1
            direction = 1.0
        elif phase < 0.75:
            tri = 1.0 - (phase - 0.25) / 0.25      # +1 → -1
            direction = -1.0
        else:
            tri = -1.0 + (phase - 0.75) / 0.25     # -1 → 0
            direction = 1.0
        delta = amp * tri
        hand = delta * ratio
        hand_rate = direction * rate * ratio

        s = plant.step(
            dt,
            hand_angle=hand,
            hand_rate=hand_rate,
            rack_force=rack_of(plant.state.pinion_angle / ratio),
            speed_ms=speed_ms,
        )
        res.peak_motor_torque = max(res.peak_motor_torque, abs(s.motor_torque))
        res.peak_motor_speed = max(res.peak_motor_speed, abs(s.motor_speed))
        res.peak_heat = max(res.peak_heat, s.motor_heat)
        res.peak_rack_force = max(res.peak_rack_force, abs(s.load_torque)
                                  / max(float(params.pinion_radius), 1e-6))
        res.peak_hand_torque = max(res.peak_hand_torque, abs(s.hand_torque))
        res.peak_power_w = max(res.peak_power_w, abs(s.motor_torque * s.motor_speed))
        sq_sum += s.motor_torque ** 2
        saturated += 1 if s.motor_saturated else 0
        limited += 1 if s.hand_limited else 0

    res.rms_motor_torque = math.sqrt(sq_sum / n)
    res.saturated_fraction = saturated / n
    res.hand_limited_fraction = limited / n
    # Either signal means the run did not do what was asked: the motor could
    # not deliver, or nothing could hold the wheel there.
    res.beyond_capability = (
        res.saturated_fraction > SATURATION_TRUST_LIMIT
        or res.hand_limited_fraction > SATURATION_TRUST_LIMIT
    )
    return res


def size_actuator(
    params: VehicleParams,
    scenarios: tuple[SizingScenario, ...] = DEFAULT_SCENARIOS,
) -> ActuatorRequirement:
    """Walk the worst cases and produce the specification with a verdict."""
    results = [run_scenario(params, s) for s in scenarios]
    req = ActuatorRequirement(scenarios=results)

    def _worst(attr: str) -> tuple[float, str]:
        best = max(results, key=lambda r: getattr(r, attr))
        return getattr(best, attr), best.scenario.id

    req.peak_torque, torque_by = _worst("peak_motor_torque")
    req.peak_speed, speed_by = _worst("peak_motor_speed")
    req.peak_power_w, power_by = _worst("peak_power_w")
    req.peak_heat, heat_by = _worst("peak_heat")
    # RMS over the duty cycle, not over the concatenation of the runs: a spec
    # built by averaging test cases weights parking like motorway driving.
    total_w = sum(s.scenario.duty_weight for s in results) or 1.0
    req.rms_torque = math.sqrt(
        sum(s.scenario.duty_weight * s.rms_motor_torque ** 2 for s in results) / total_w
    )
    req.driven_by = {
        "peak_torque": torque_by, "peak_speed": speed_by,
        "peak_power": power_by, "thermal": heat_by,
    }

    m = params.steering_system.motor
    w_max = m.no_load_speed_rpm * 2 * math.pi / 60
    checks = {
        "peak_torque": (req.peak_torque, m.peak_torque),
        "continuous_torque": (req.rms_torque, m.continuous_torque),
        "speed": (req.peak_speed, w_max),
    }
    verdict: dict[str, Any] = {"pass": True, "checks": {}}
    for name, (need, have) in checks.items():
        margin = (have - need) / have if have else float("-inf")
        ok = need <= have
        verdict["checks"][name] = {
            "required": round(need, 3), "available": round(have, 3),
            "margin_pct": round(margin * 100, 1), "pass": ok,
        }
        verdict["pass"] = verdict["pass"] and ok
    # Saturation anywhere is a finding even when every headline check passes:
    # it means a manoeuvre was not delivered as asked.
    sat = [s.scenario.id for s in results if s.beyond_capability]
    if sat:
        verdict["beyond_capability_in"] = sat
        verdict["note"] = ("这些工况已超出作动器能力，其峰值数字不可信；"
                           "先换更大的电机再读余量。")
        verdict["pass"] = False
    req.verdict = verdict
    return req
