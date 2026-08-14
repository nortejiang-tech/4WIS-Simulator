"""Tests — tracking step-response analysis (FR-10's step arm)."""

from __future__ import annotations

import numpy as np
import pytest

from sim4wis.study.tracking_step import (
    ProcedureError,
    _analyse_one,
    analyse_tracking_step,
)


def _first_order_step(amp: float, tau: float, t_end: float, dt: float,
                      toe: float = 0.0):
    """An exact first-order response to a step — analytic rise/settle."""
    t = np.arange(0.0, t_end + dt / 2, dt)
    cmd = np.where(t >= t_end / 3, amp, 0.0)
    resp = toe + (amp - toe) * 0.0
    resp = toe + np.where(t >= t_end / 3, amp, 0.0) * 0.0
    # piecewise: before the step the baseline is toe; after, exponential.
    i0 = int((t_end / 3) / dt)
    resp = np.full_like(t, toe)
    resp[i0:] = toe + amp * (1.0 - np.exp(-(t[i0:] - t[i0]) / tau))
    return t, cmd, resp


def test_first_order_metrics_match_the_analytic_values():
    amp, tau = 0.1, 0.05
    t, cmd, resp = _first_order_step(amp, tau, 3.0, 0.005)
    m = _analyse_one(cmd, resp, 0.005)
    # rise 10→90 %: ln(1−0.1) − ln(1−0.9) = ln 9 ≈ 2.197.
    assert m.rise_s == pytest.approx(tau * np.log(9.0), rel=0.05)
    assert m.overshoot_pct == 0.0
    # settle into ±2 %: τ·ln(50) ≈ 3.91·τ.
    assert m.settle_s == pytest.approx(tau * np.log(50.0), rel=0.08)
    assert abs(m.ss_err_rad) < 1e-10


def test_static_toe_does_not_read_as_tracking_error():
    amp, tau = 0.1, 0.05
    t, cmd, resp = _first_order_step(amp, tau, 3.0, 0.005, toe=-0.001745)
    m = _analyse_one(cmd, resp, 0.005)
    assert abs(m.ss_err_rad) < 1e-10
    # Post-90 % deviation of a clean first-order approach is the ~10 % still
    # un-arrived at the first sample past the crossing (sample-quantised).
    # The crossing sample can land anywhere in [90 %, ~95 %], so the
# deviation band is [5 %, 10 %] of the step.
    assert 0.05 * amp <= m.peak_dev_rad <= 0.10 * amp + 1e-12


def test_analysis_refuses_a_non_step():
    t = np.linspace(0, 1, 200)
    cmd = np.full_like(t, 0.1)
    with pytest.raises(ProcedureError, match="never steps"):
        _analyse_one(cmd, cmd.copy(), 0.005)


def test_a_tiny_step_is_treated_as_a_cross_talk_witness():
    # A wheel commanded with (near) zero is the witness case: its metrics
    # measure how well it held zero, not a rise time.
    t = np.linspace(0, 1, 200)
    cmd = np.where(t > 0.5, 0.0001, 0.0)
    resp = cmd.copy() * 0.5
    m = _analyse_one(cmd, resp, 0.005)
    assert m.rise_s == 0.0 and m.settle_s == 0.0
    assert m.peak_dev_rad == pytest.approx(0.00005, abs=1e-6)


def test_full_analysis_covers_fl_and_rl():
    t, cmd, resp = _first_order_step(0.1, 0.05, 3.0, 0.005)
    rl_cmd = np.where(t >= t[-1] / 3, 1e-6, 0.0)  # witness: (near) zero step
    rl_resp = 0.5 * rl_cmd
    result = analyse_tracking_step(t, {
        "delta_cmd_fl": cmd, "delta_fl": resp,
        "delta_cmd_rl": rl_cmd, "delta_rl": rl_resp,
    })
    assert result.get("fl").rise_s > 0.0
    assert result.get("rl").ss_err_rad == pytest.approx(0.0, abs=1e-6)
