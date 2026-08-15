"""Tests — swept-sine tracking analysis (FR-10's frequency arm)."""

from __future__ import annotations

import math

import numpy as np
import pytest

from sim4wis.study.tracking_sweep import (
    ProcedureError,
    analyse_tracking_sweep,
)


def _stepped_sine(amp: float, fc: float, freqs: tuple[float, ...],
                  cycles: int = 4, dt: float = 0.005):
    """Command = stepped sine; response = its exact first-order filtering.

    First-order closed loop with cutoff fc: amplitude ratio 1/sqrt(1+(f/fc)²),
    phase lag atan(f/fc). The analysis must recover both, and the −3 dB
    bandwidth must land on fc.
    """
    t_all, cmd_all, resp_all = [], [], []
    t0 = 0.0
    for f in freqs:
        n = int(cycles / f / dt)
        t = t0 + np.arange(n) * dt
        c = amp * np.sin(2 * math.pi * f * t)
        ratio = 1.0 / math.sqrt(1.0 + (f / fc) ** 2)
        lag = math.atan(f / fc)
        r = amp * ratio * np.sin(2 * math.pi * f * t - lag)
        t_all.append(t)
        cmd_all.append(c)
        resp_all.append(r)
        t0 = t[-1] + dt
    return (np.concatenate(t_all), np.concatenate(cmd_all),
            np.concatenate(resp_all))


def test_first_order_ratios_phase_and_bandwidth_match_the_analytic_values():
    fc = 3.0
    t, cmd, resp = _stepped_sine(0.0035, fc, (0.5, 1.0, 2.0, 5.0, 10.0))
    m = analyse_tracking_sweep(t, {"delta_cmd_fl": cmd, "delta_fl": resp})
    for f in (0.5, 1.0, 2.0, 5.0, 10.0):
        ratio, lag = m.get(f)
        want_ratio = 1.0 / math.sqrt(1.0 + (f / fc) ** 2)
        want_lag = math.degrees(math.atan(f / fc))
        assert ratio == pytest.approx(want_ratio, rel=0.05), f
        assert lag == pytest.approx(want_lag, rel=0.05), f
    # The −3 dB crossing is exactly fc for a first-order system; the
    # interpolated value carries the log-log two-point interpolation error
    # across the knee (measured 9 % with 2→5 Hz spacing — accuracy is
    # bounded by the segment spacing, documented in the guide).
    assert m.bandwidth_hz == pytest.approx(fc, rel=0.12)


def test_all_pass_bandwidth_is_refused_with_only_a_bound():
    # fc far above the sweep: every ratio ≥ 0.707 → bandwidth is a lower
    # bound only, and the analysis says so instead of inventing a number.
    t, cmd, resp = _stepped_sine(0.0035, 500.0, (0.5, 1.0, 2.0))
    m = analyse_tracking_sweep(t, {"delta_cmd_fl": cmd, "delta_fl": resp})
    assert m.bandwidth_hz is None
    assert m.max_frequency_hz == pytest.approx(2.0, rel=0.05)


def test_analysis_refuses_a_non_sweep_trace():
    t = np.linspace(0, 10, 2000)
    cmd = np.where(t > 5, 0.1, 0.0)  # a step, not a sweep
    with pytest.raises(ProcedureError, match="频率没有分段|过零点太少"):
        analyse_tracking_sweep(t, {"delta_cmd_fl": cmd, "delta_fl": cmd})
