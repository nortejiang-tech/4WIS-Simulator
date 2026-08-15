"""Tests — load-disturbance tracking analysis (FR-10's disturbance arm)."""

from __future__ import annotations

import numpy as np
import pytest

from sim4wis.study.tracking_disturbance import (
    ProcedureError,
    analyse_tracking_disturbance,
)


def _run_with_step(onset_s: float, step_n: float, dev_rad: float,
                   recover_s: float = 0.2, t_end: float = 4.0, dt: float = 0.005):
    """Synthetic trace: flat rack force, a step at onset, a deviation bump
    that recovers — the shape the analyser is for."""
    t = np.arange(0.0, t_end, dt)
    rack = np.where(t >= onset_s, 500.0 + step_n, 500.0)
    cmd = np.full_like(t, 0.03)
    # deviation bump: rises at onset, decays with the recovery time constant
    tau = recover_s / 3.0
    dev = dev_rad * np.exp(-np.clip(t - onset_s, 0.0, None) / tau)
    dev[t < onset_s] = 0.0
    resp = cmd + dev
    return t, {"rack_force_fl": rack, "delta_cmd_fl": cmd, "delta_fl": resp}


def test_onset_peak_and_recovery_are_recovered():
    t, ch = _run_with_step(onset_s=2.0, step_n=1500.0, dev_rad=0.03,
                           recover_s=0.15)
    m = analyse_tracking_disturbance(t, ch)
    # Reported at the window midpoint: an instantaneous synthetic step at
    # 2.0 s reads 1.95 s (the window is 0.1 s wide).
    assert m.onset_s == pytest.approx(2.0, abs=0.06)
    assert m.peak_dev_rad == pytest.approx(0.03, rel=0.05)
    # Band re-entry at τ·ln 3 ≈ 0.055 s after the step; measured from the
    # step's beginning ≈ 0.05 s earlier → ≈ 0.105 s.
    assert m.recover_s == pytest.approx(0.105, abs=0.06)
    assert m.rack_step_n == pytest.approx(1500.0, rel=0.05)


def test_a_flat_trace_is_refused_not_scored_perfect():
    t = np.arange(0.0, 4.0, 0.005)
    ch = {"rack_force_fl": np.full_like(t, 500.0),
          "delta_cmd_fl": np.full_like(t, 0.03),
          "delta_fl": np.full_like(t, 0.03)}
    with pytest.raises(ProcedureError, match="可检测的负载扰动"):
        analyse_tracking_disturbance(t, ch)


def test_an_insignificant_step_is_refused():
    # A 5 N wiggle on a 500 N swing: not a disturbance event.
    t, ch = _run_with_step(onset_s=2.0, step_n=5.0, dev_rad=0.001)
    with pytest.raises(ProcedureError, match="扰动不显著"):
        analyse_tracking_disturbance(t, ch)


def test_never_recovering_reports_none_not_a_number():
    t, ch = _run_with_step(onset_s=1.0, step_n=1500.0, dev_rad=0.03,
                           recover_s=1e6)  # decays far slower than the window
    m = analyse_tracking_disturbance(t, ch)
    assert m.recover_s is None  # honest: never re-entered the band
