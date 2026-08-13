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
from sim4wis.steering.plant import SteeringPlant, STEERING_INNER_DT
from sim4wis.vehicle.load_analysis import sweep_load_analysis

#: Above this driver-limited fraction a scenario's numbers stop meaning
#: anything, because the run stopped performing the manoeuvre it was given.
#:
#: This rule used to be about **assist saturation**, and it was written when
#: saturation meant divergence: the marginal band — a motor that almost holds
#: the load — rang, so any run that spent time past capability produced a trace
#: nobody could read. Three defects fed that band and all three are gone: the
#: driver was infinitely stiff (fixed by giving the grip a real impedance), the
#: ECU damping opposed the manoeuvre rather than the mode it was there to damp,
#: and the input profile reversed direction in zero time. A motor-size sweep at
#: full-lock parking is now monotone from 2 to 12 N.m with no oscillation
#: anywhere.
#:
#: So saturation on its own is no longer a reason to distrust anything — it is
#: an ordinary finding, and it reads exactly as it feels: the motor delivered
#: less than the map asked for and the steering got heavier. What still voids a
#: run is the **driver** running out: past their torque limit the wheel never
#: reaches the commanded angle, so the peaks describe a manoeuvre that did not
#: happen. Those runs still answer "this motor is not enough"; they cannot
#: answer "by how much", because what bounded them was the driver's arms.
DELIVERY_TRUST_LIMIT = 0.05


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
    #: Angular acceleration of the input, at the road wheel [deg/s²].
    #:
    #: Not a refinement — without it the profile is physically impossible and
    #: the plant answers accordingly. A pure triangle reverses direction in
    #: zero time, so the commanded rate steps by 2R at every apex and by R at
    #: t = 0; through the driver's grip damper that is an unbounded torque
    #: step, and the driver duly pinned on their 25 N.m limit for 83% of the
    #: evasive run — a "manoeuvre" nothing could perform, measured to three
    #: decimal places. Both a human and a steering robot have a finite one.
    accel_deg_s2: float = 200.0


#: Deliberately small. Each entry earns its place by exposing something the
#: others cannot, and a library nobody reads is worse than four cases everyone
#: does.
DEFAULT_SCENARIOS: tuple[SizingScenario, ...] = (
    SizingScenario(
        "parking_full_lock", "原地全锁", "峰值转矩",
        speed_kmh=0.0, mu=0.9, amplitude_deg=35.0, rate_deg_s=25.0,
        accel_deg_s2=150.0, duty_weight=0.05,
    ),
    SizingScenario(
        "parking_repeat", "连续挪车 ×3", "热降额",
        speed_kmh=0.0, mu=0.9, amplitude_deg=35.0, rate_deg_s=25.0, cycles=3,
        accel_deg_s2=150.0, duty_weight=0.05,
    ),
    SizingScenario(
        "low_speed_manoeuvre", "低速大转角", "综合负载",
        speed_kmh=20.0, mu=0.9, amplitude_deg=25.0, rate_deg_s=20.0,
        accel_deg_s2=200.0, duty_weight=0.20,
    ),
    SizingScenario(
        # Deliberately the sharpest input in the library: through the default
        # 7.71 ratio, 700 deg/s² at the road wheel is ~5 400 deg/s² at the hand
        # wheel, which is about what a startled driver manages. The parking
        # cases are an order of magnitude gentler because a deliberate input is.
        "evasive_at_speed", "高速紧急避让", "峰值转速 / 反电动势包络",
        speed_kmh=100.0, mu=0.9, amplitude_deg=4.0, rate_deg_s=90.0,
        accel_deg_s2=700.0, duty_weight=0.70,
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
    #: True when the run spent long enough failing to deliver the commanded
    #: manoeuvre that its peaks describe something that did not happen.
    #: See `DELIVERY_TRUST_LIMIT`.
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
            **({"note": "手力矩已达上限，车轮没有到达指令角度 —— 只能读出"
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


#: Speed at which the tyre is considered to be rolling rather than scrubbing
#: [m/s]. Below it a steered tyre pivots in its own contact patch and the load
#: is dissipative; above it the tyre rolls, develops a slip angle, and the load
#: becomes the restoring aligning torque. The transition is over in the first
#: few km/h, which is what 1.5 m/s (~5 km/h) encodes.
#:
#: This split exists because `sweep_load_analysis` is a **quasi-static** model:
#: it answers "what force holds this angle", which does not say what the rack
#: does when the wheel gets away from the driver. Signing that force by angle
#: at standstill builds a 400 N.m centring spring — and a sizing run duly used
#: it to fling the pinion backwards at 345 000 rpm. Taking its magnitude
#: instead, as this module first did, is worse: the load then pushes the same
#: way whichever way the wheels point.
_ROLLING_SPEED_MS = 1.5


def _rack_force_curve(params: VehicleParams, speed_kmh: float, mu: float):
    """Interpolator: road wheel angle [rad] → (aligning [N], scrub [N]).

    The first is angle-directed and restores toward centre; the second is a
    magnitude that opposes motion. See `_ROLLING_SPEED_MS`.
    """
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
    v = abs(speed_kmh) / 3.6
    rolling = v / (v + _ROLLING_SPEED_MS)

    def curve(a: float) -> tuple[float, float]:
        # Both front tie rods react on the same rack.
        mag = 2.0 * float(np.interp(abs(a), xs, ys))
        return math.copysign(rolling * mag, a), (1.0 - rolling) * mag

    return curve


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
    accel = math.radians(max(scenario.accel_deg_s2, 1e-6))

    # Waypoints, not a clock. A steering robot is told "go to +35 deg, then to
    # -35, then back to centre, at this rate and this acceleration"; it is not
    # told to be at a particular angle at a particular millisecond. Driving it
    # by waypoints is both what the hardware does and what removes the apex
    # discontinuity, because the reversal is now a deceleration through zero
    # rather than an instantaneous sign change.
    targets = [amp, -amp, 0.0] * max(scenario.cycles, 1)

    # Upper bound on the run: every leg at worst covers its distance at the
    # slowest of the two limits, plus the ramp at each end, plus a settle.
    leg = 2.0 * amp
    span = sum([amp] + [leg] * (len(targets) - 2) + [amp])
    total = span / max(rate, 1e-6) + 2.0 * len(targets) * rate / accel + 0.5
    n = max(int(total / dt), 10)

    res = ScenarioResult(scenario=scenario)
    sq_sum = 0.0
    saturated = 0
    limited = 0
    steps = 0
    speed_ms = scenario.speed_kmh / 3.6

    delta = 0.0            # commanded road wheel angle [rad]
    delta_rate = 0.0       # and its rate — a state now, not a square wave
    prev_hand = 0.0        # previous outer-step commanded hand angle, for the ramp
    leg_index = 0
    settle = 0.0

    for _ in range(n):
        target = targets[min(leg_index, len(targets) - 1)]
        gap = target - delta
        if abs(gap) < 1e-4 and abs(delta_rate) < 1e-3:
            if leg_index < len(targets) - 1:
                leg_index += 1
            else:
                settle += dt
                if settle > 0.1:
                    break
        # Fastest rate from which the target can still be reached without
        # overshooting: v² = 2·a·d. Below the rate limit this is what shapes
        # the approach; above it the rate limit governs the middle of the leg.
        v_brake = math.sqrt(2.0 * accel * abs(gap))
        want = math.copysign(min(rate, v_brake), gap) if gap else 0.0
        dv = max(-accel * dt, min(accel * dt, want - delta_rate))
        delta_rate += dv
        delta += delta_rate * dt

        hand = delta * ratio
        hand_rate = delta_rate * ratio

        aligning, scrub = rack_of(plant.state.pinion_angle / ratio)
        # Multi-rate, as in vehicle/steering_link.plant_front_angle: the peak
        # quantities (hand torque, motor torque) are under-resolved when the
        # plant sees the command as a 2 ms staircase, so advance it at the
        # layer's inner rate with the commanded angle interpolated across the
        # span. The waypoint trajectory is piecewise-linear, so the ramp is
        # the exact command, not an approximation.
        n_inner = max(1, int(round(dt / max(STEERING_INNER_DT, 1e-9))))
        h = dt / n_inner
        s = plant.state
        for k in range(n_inner):
            frac = (k + 1) / n_inner
            hand_k = prev_hand + (hand - prev_hand) * frac
            s = plant.step(
                h,
                hand_angle=hand_k,
                hand_rate=hand_rate,
                rack_force=aligning,
                rack_resistance=scrub,
                speed_ms=speed_ms,
            )
        prev_hand = hand
        steps += 1
        res.peak_motor_torque = max(res.peak_motor_torque, abs(s.motor_torque))
        res.peak_motor_speed = max(res.peak_motor_speed, abs(s.motor_speed))
        res.peak_heat = max(res.peak_heat, s.motor_heat)
        # Total tyre load, both shares — `load_torque` carries only the
        # angle-directed one, and a rack force that vanished at standstill
        # would be a lie in the one place it matters most.
        res.peak_rack_force = max(res.peak_rack_force, abs(aligning) + abs(scrub))
        res.peak_hand_torque = max(res.peak_hand_torque, abs(s.hand_torque))
        res.peak_power_w = max(res.peak_power_w, abs(s.motor_torque * s.motor_speed))
        sq_sum += s.motor_torque ** 2
        saturated += 1 if s.motor_saturated else 0
        limited += 1 if s.hand_limited else 0

    n = max(steps, 1)
    res.rms_motor_torque = math.sqrt(sq_sum / n)
    res.saturated_fraction = saturated / n
    res.hand_limited_fraction = limited / n
    # Either signal means the run did not do what was asked: the motor could
    # not deliver, or nothing could hold the wheel there.
    # Only the driver running out voids the run. Assist saturation is reported
    # and is a finding in its own right, but the trace stays readable.
    res.beyond_capability = res.hand_limited_fraction > DELIVERY_TRUST_LIMIT
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
    # A manoeuvre the driver could not perform voids its own numbers, and no
    # headline margin computed from the rest is worth reading past it.
    undelivered = [s.scenario.id for s in results if s.beyond_capability]
    if undelivered:
        verdict["beyond_capability_in"] = undelivered
        verdict["note"] = ("这些工况下手力矩已达上限、车轮未到达指令角度，"
                           "其峰值数字不可信；先换更大的电机再读余量。")
        verdict["pass"] = False
    # Saturation is reported separately: the run is readable, but the assist
    # was not delivering what the map asked for and the steering was heavier
    # than the calibration intends.
    saturated = [s.scenario.id for s in results
                 if s.saturated_fraction > DELIVERY_TRUST_LIMIT
                 and not s.beyond_capability]
    if saturated:
        verdict["assist_saturated_in"] = saturated
        verdict["saturation_note"] = ("助力在这些工况下饱和：转向比标定意图更重，"
                                      "但数值有效 —— 这是一条发现，不是一次失效。")
    req.verdict = verdict
    return req
