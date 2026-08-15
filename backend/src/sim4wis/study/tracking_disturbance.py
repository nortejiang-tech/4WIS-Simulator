"""Tracking-layer load-disturbance analysis — FR-10's disturbance arm.

A genuine vehicle-level disturbance: the run drives with a settled small
front angle and crosses a low-μ patch (a scene disturbance), so the tyre
aligning force — and with it the corner trackers' load — steps abruptly.
The analysis detects the onset from the rack-force channel itself (the
largest short-window step), then measures how far each corner's tracking
deviation is thrown and how long it takes to recover.

This is the arm where the disturbance-rejecting designs are supposed to
show their worth: a DOB/ADRC should recover with less deviation than the
same loop without the observer, and the comparison table is the point.

Onset detection: the rack force is differentiated over a sliding window;
the onset is the window whose change is the largest fraction of the trace's
range. Refuses when no step stands out above the trace's own noise (the
patch was missed, or the load never moved) — an unnoticed non-event must
not read as excellent tracking.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class ProcedureError(ValueError):
    """Not a load-disturbance run, or no detectable onset — analysis refuses."""


@dataclass(frozen=True)
class DisturbanceMetrics:
    onset_s: float
    #: Largest |delta − delta_cmd| after the onset [rad], front axle.
    peak_dev_rad: float
    #: Time from onset until the front deviation re-enters ±0.01 rad [s].
    #: None when it never re-enters inside the window — a real finding.
    recover_s: float | None
    #: The load step size that caused it [N], for context.
    rack_step_n: float


def _detect_onset(t: np.ndarray, rack: np.ndarray, window_s: float = 0.1,
                  min_step_n: float = 100.0) -> tuple[int, int, float]:
    """Indices (step_begin, step_mid) of the largest rack-force step and its
    magnitude [N].

    The deviation must be measured from the step's BEGINNING (the transient
    throw is the point); the reported onset time is the window MIDPOINT —
    a physical rack step takes about one window, and the midpoint is the
    honest "when it happened"."""
    dt = float(np.median(np.diff(t)))
    w = max(int(round(window_s / dt)), 2)
    steps = np.abs(rack[w:] - rack[:-w])
    rng = float(np.max(rack) - np.min(rack))
    if rng < 1.0:
        raise ProcedureError(
            f"齿条力全程幅值仅 {rng:.2f} N —— 没有可检测的负载扰动")
    i = int(np.argmax(steps))
    if steps[i] < 0.25 * rng or steps[i] < min_step_n:
        # The relative test alone cannot refuse: the step itself defines the
        # range. The absolute floor carries the "worth the name" part —
        # vehicle-measured disturbance steps are 1-2 kN; noise wiggles, tens
        # of newtons.
        raise ProcedureError(
            f"最大齿条力阶跃 {steps[i]:.1f} N 不足全程幅值的 25% 或绝对值 "
            f"{min_step_n:.0f} N —— 扰动不显著，不能当作负载阶跃工况分析")
    return i, i + w // 2, float(steps[i])


def analyse_tracking_disturbance(
    t: np.ndarray,
    channels: dict[str, np.ndarray],
    wheel: str = "fl",
    recover_band_rad: float = 0.01,
) -> DisturbanceMetrics:
    t = np.asarray(t, dtype=float)
    rack = np.asarray(channels[f"rack_force_{wheel}"], dtype=float)
    cmd = np.asarray(channels[f"delta_cmd_{wheel}"], dtype=float)
    resp = np.asarray(channels[f"delta_{wheel}"], dtype=float)
    if t.size < 200:
        raise ProcedureError("trace too short for a disturbance analysis")

    i_begin, i_mid, step_n = _detect_onset(t, rack)
    dev = np.abs(resp - cmd)
    post = dev[i_begin:]
    peak = float(np.max(post)) if post.size else 0.0

    dt = float(np.median(np.diff(t)))
    inside = post <= recover_band_rad
    outside = np.where(~inside)[0]
    if not inside[-1]:
        # The trace still ends outside the band: never recovered inside
        # this window — reported as None, not as the window length.
        recover = None
    elif outside.size == 0:
        recover = 0.0  # never left the band — nothing to recover from
    else:
        # Recovery = the last instant outside the band; everything after
        # stays in. Measuring from the step's beginning means pre-throw
        # samples cannot shortcut this — only the throw itself counts.
        recover = float((outside[-1] + 1) * dt)
    return DisturbanceMetrics(
        onset_s=float(t[i_mid]), peak_dev_rad=peak, recover_s=recover,
        rack_step_n=step_n,
    )
