"""Tracking-layer step-response analysis — the evaluation protocol's first tool.

FR-10's step arm: given a run whose maneuver is a held step on the front
axle, extract per-corner rise time, overshoot, settling time, steady-state
error and peak deviation from the recorded ``delta_cmd_*`` / ``delta_*``
channels. These are **process metrics**: they are defined only for this
manoeuvre class and need the whole trace, not one scalar — same reasoning
as the ISO 13674 on-centre analysis.

The analysis refuses when the trace does not look like a step (no
commanded step detected, or the step is smaller than the quantisation
noise) rather than returning numbers that happen to parse.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

#: Fraction of the settled command rise used as the rise window (10–90 %).
_RISE_LO, _RISE_HI = 0.1, 0.9
#: Settling band, fraction of the step amplitude.
_SETTLE_BAND = 0.02
#: Minimum commanded step to analyse [rad].
_MIN_STEP = 0.002
#: Last fraction of the post-step window used for the steady-state error.
_SS_FRACTION = 0.2


class ProcedureError(ValueError):
    """The run is not a tracking step — analysis refuses."""


@dataclass(frozen=True)
class StepMetrics:
    rise_s: float
    overshoot_pct: float
    settle_s: float
    ss_err_rad: float
    peak_dev_rad: float
    amplitude_rad: float


def _analyse_one(cmd: np.ndarray, resp: np.ndarray, dt: float) -> StepMetrics:
    # The step is found on the command: last index before the command leaves
    # the initial plateau, and the settled command value.
    c0 = float(cmd[0])
    c_end = float(cmd[-1])
    moved = np.where(np.abs(cmd - c0) > max(0.05 * abs(c_end - c0), 1e-12))[0]
    if moved.size == 0:
        raise ProcedureError("commanded channel never steps")
    i_step = int(moved[0])
    amp = c_end - c0
    if i_step < 10 or cmd.size - i_step < int(0.5 / max(dt, 1e-9)):
        raise ProcedureError("not enough samples around the step")
    # A step-response run holds the command constant after the step; a weave
    # (or any other manoeuvre) keeps moving it. Analysing a weave with step
    # metrics would produce numbers that look like tracking quality and are
    # not — so the analysis refuses instead.
    post_drift = float(np.max(np.abs(cmd[i_step:] - c_end)))
    if post_drift > 0.01 * abs(amp) + 1e-12:
        raise ProcedureError(
            f"command drifts {post_drift:.2e} rad after the step — not a "
            "step-response run; the tracking metrics refuse")

    resp_b = resp - float(resp[i_step - 1])
    cmd_b = cmd - c0
    n_ss = max(int(resp.size * _SS_FRACTION), 8)

    # A wheel that was NOT stepped (a cross-talk witness, e.g. the rear on a
    # front step) has no meaningful rise/settle; what matters is how well it
    # held zero — its whole deviation is the story.
    if abs(amp) < _MIN_STEP:
        ss_err = float((resp_b[-n_ss:] - cmd_b[-n_ss:]).mean())
        peak_dev = float(np.max(np.abs(resp_b[i_step:] - cmd_b[i_step:])))
        return StepMetrics(rise_s=0.0, overshoot_pct=0.0, settle_s=0.0,
                           ss_err_rad=ss_err, peak_dev_rad=peak_dev,
                           amplitude_rad=amp)

    y = resp_b / amp  # normalised response, 0 → 1

    # Rise: 10 % → 90 % of the final value.
    i10 = int(np.where(y >= _RISE_LO)[0][0]) if np.any(y >= _RISE_LO) else i_step
    i90 = int(np.where(y >= _RISE_HI)[0][0]) if np.any(y >= _RISE_HI) else i_step
    rise = max((i90 - i10) * dt, 0.0)

    # Overshoot: peak above the final value, in percent of the step.
    peak = float(np.max(y[i_step:]))
    overshoot = max(peak - 1.0, 0.0) * 100.0

    # Settling: last exit of the ±2 % band.
    outside = np.where(np.abs(y - 1.0) > _SETTLE_BAND)[0]
    i_settle = int(outside[-1]) + 1 if outside.size else i_step
    settle = (i_settle - i_step) * dt

    # Steady state, baseline-corrected (`delta_*` carries static toe).
    resp_ss = float(resp[-n_ss:].mean()) - float(resp[i_step - 1])
    ss_err = resp_ss - amp

    # Peak deviation after the response first reaches 90 % of the step. A
    # clean first-order approach reads exactly 10 % of the step there; the
    # metric is what the loop failed to remove on top of that — overshoot,
    # oscillation, sag.
    i90d = int(np.where(y >= 0.9)[0][0]) if np.any(y >= 0.9) else i_step
    peak_dev = float(np.max(np.abs(resp_b[i90d:] - cmd_b[i90d:])))
    return StepMetrics(rise_s=rise, overshoot_pct=overshoot, settle_s=settle,
                       ss_err_rad=ss_err, peak_dev_rad=peak_dev, amplitude_rad=amp)


@dataclass(frozen=True)
class TrackingStepResult:
    per_wheel: dict[str, StepMetrics]

    def get(self, wheel: str) -> StepMetrics:
        return self.per_wheel[wheel]


def analyse_tracking_step(
    t: np.ndarray,
    channels: dict[str, np.ndarray],
    wheels: tuple[str, ...] = ("fl", "rl"),
) -> TrackingStepResult:
    """Step-response metrics per wheel, from recorded delta_cmd/delta pairs.

    `fl` is the stepped wheel (the manoeuvre drives the front axle), `rl` is
    the cross-talk witness — it should hold zero, and how well it does is
    part of the control-quality story.
    """
    t = np.asarray(t, dtype=float)
    if t.size < 20:
        raise ProcedureError("trace too short for a step analysis")
    dt = float(np.median(np.diff(t)))
    out: dict[str, StepMetrics] = {}
    for w in wheels:
        cmd = channels.get(f"delta_cmd_{w}")
        resp = channels.get(f"delta_{w}")
        if cmd is None or resp is None:
            raise ProcedureError(f"channels delta_cmd_{w} / delta_{w} missing")
        out[w] = _analyse_one(np.asarray(cmd, dtype=float),
                              np.asarray(resp, dtype=float), dt)
    return TrackingStepResult(out)
