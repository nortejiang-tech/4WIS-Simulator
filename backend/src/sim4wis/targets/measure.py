"""Measurements — what compliance is checked against.

A target set does not know or care where a number came from; it needs a metric
name, a value, and the operating point the value belongs to. That is all a
`Measurement` is, and it is the whole reason one compliance evaluator can judge
a sweep of vehicle runs and an actuator sizing pass without special-casing
either.

One field is not decoration. **`trusted`** carries forward whatever the source
already knows about its own numbers, and an untrusted measurement never
produces a verdict — it produces NOT EVALUATED with the reason attached. The
sizing module states plainly that past its saturation limit its peaks mean
nothing, and a compliance table that turned such a run green would be worse
than no table at all: it would launder an unusable result into a signed-off
requirement. Once a module has admitted its own limit, everything downstream
has to honour it.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:                                    # pragma: no cover
    from sim4wis.steering.sizing import ActuatorRequirement
    from sim4wis.study.result import StudyResult

#: What `sizing` reports per scenario, and the unit each is expressed in.
SCENARIO_METRICS: dict[str, str] = {
    "peak_motor_torque_nm": "N·m",
    "peak_motor_speed_rpm": "rpm",
    "rms_motor_torque_nm": "N·m",
    "peak_power_w": "W",
    "peak_thermal_state": "—",
    "peak_rack_force_n": "N",
    "peak_hand_torque_nm": "N·m",
}

#: The rolled-up specification. Named apart from the per-scenario metrics
#: rather than distinguished by a coordinate, because they answer different
#: questions and a reader should not have to check a selector to tell which one
#: a row is about: `rms_motor_torque_nm` is one manoeuvre, `required_rms_torque_nm`
#: is the duty cycle.
REQUIREMENT_METRICS: dict[str, str] = {
    "required_peak_torque_nm": "N·m",
    "required_peak_speed_rpm": "rpm",
    "required_rms_torque_nm": "N·m",
    "required_peak_power_w": "W",
    "required_thermal_state": "—",
}


@dataclass(frozen=True)
class Measurement:
    """One number, at one operating point, from one source."""

    metric: str
    value: float
    #: Operating-point coordinates the `at` selector filters on. Empty means
    #: the measurement is unconditional and only `at: all` reaches it.
    at: dict[str, Any] = field(default_factory=dict)
    label: str = ""
    source: str = ""
    trusted: bool = True
    untrusted_reason: str = ""

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {
            "metric": self.metric, "value": self.value,
            "at": self.at, "label": self.label, "source": self.source,
        }
        if not self.trusted:
            d["untrusted_reason"] = self.untrusted_reason
        return d


def from_study(result: StudyResult) -> list[Measurement]:
    """Every metric of every grid cell, keyed by the study's own coordinates.

    A cell's coordinates are its *sweep* coordinates, which is the honest
    limitation to know about: a target written "at 100 km/h" can only be
    checked when speed is an axis of the study. It is not silently ignored —
    the evaluator reports it as unevaluated and names the axis it wanted.
    """
    out: list[Measurement] = []
    src = f"study:{result.study}"
    for row in result.rows:
        for metric, value in row.metrics.items():
            v = float(value)
            bad = math.isnan(v) or math.isinf(v)
            out.append(Measurement(
                metric=metric, value=v, at=dict(row.coords), label=row.label,
                source=src, trusted=not bad,
                untrusted_reason="指标为 NaN/Inf" if bad else "",
            ))
    return out


def from_sizing(req: ActuatorRequirement, *, source: str = "sizing") -> list[Measurement]:
    """Scenario results and the rolled-up specification.

    Trust is propagated per metric rather than in bulk. A maximum is only as
    trustworthy as the scenario that produced it, which `driven_by` names; the
    duty-cycle RMS mixes every scenario, so any untrustworthy run taints it.
    """
    out: list[Measurement] = []
    tainted = {s.scenario.id for s in req.scenarios if s.beyond_capability}
    reason = "工况超出作动器能力，饱和后的数值不可信"

    for s in req.scenarios:
        at = {"scenario": s.scenario.id}
        values = {
            "peak_motor_torque_nm": s.peak_motor_torque,
            "peak_motor_speed_rpm": s.peak_motor_speed * 60.0 / (2.0 * math.pi),
            "rms_motor_torque_nm": s.rms_motor_torque,
            "peak_power_w": s.peak_power_w,
            "peak_thermal_state": s.peak_heat,
            "peak_rack_force_n": s.peak_rack_force,
            "peak_hand_torque_nm": s.peak_hand_torque,
        }
        for metric, value in values.items():
            out.append(Measurement(
                metric=metric, value=float(value), at=at, label=s.scenario.label,
                source=f"{source}:{s.scenario.id}",
                trusted=not s.beyond_capability,
                untrusted_reason=reason if s.beyond_capability else "",
            ))

    driven = req.driven_by
    rollup = {
        "required_peak_torque_nm": (req.peak_torque, driven.get("peak_torque")),
        "required_peak_speed_rpm": (req.peak_speed * 60.0 / (2.0 * math.pi),
                                    driven.get("peak_speed")),
        "required_peak_power_w": (req.peak_power_w, driven.get("peak_power")),
        "required_thermal_state": (req.peak_heat, driven.get("thermal")),
    }
    for metric, (value, by) in rollup.items():
        bad = by in tainted
        out.append(Measurement(
            metric=metric, value=float(value), at={}, label=f"包络（{by or '—'}）",
            source=f"{source}:requirement", trusted=not bad,
            untrusted_reason=f"{reason}（{by}）" if bad else "",
        ))
    out.append(Measurement(
        metric="required_rms_torque_nm", value=float(req.rms_torque), at={},
        label="占空比 RMS", source=f"{source}:requirement",
        trusted=not tainted,
        untrusted_reason=(f"{reason}（{', '.join(sorted(tainted))}）" if tainted else ""),
    ))
    return out


def known_metrics() -> dict[str, str]:
    """Metric → unit, for every measurement source this module can produce."""
    return {**SCENARIO_METRICS, **REQUIREMENT_METRICS}
