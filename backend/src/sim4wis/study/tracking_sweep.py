"""Tracking-layer swept-sine analysis — the frequency-domain evaluation arm.

FR-10's second arm: a stepped-sine manoeuvre on the front axle (segments of
held frequency, e.g. 0.5/1/2/5 Hz), from which the analysis extracts per-
segment amplitude ratio and phase lag between ``delta_cmd`` and ``delta``,
and interpolates the −3 dB tracking bandwidth.

Segmentation is derived from the command itself (zero-crossing half-period
estimates; a boundary is where the local period changes materially), so the
analysis never needs the spec's segment table — it measures whatever the
manoeuvre actually did, same philosophy as the on-centre weave analyser
(measured frequency, not spec frequency).

Per segment the response is fitted with A·sin + B·cos at the segment's
median measured frequency (correlation least squares — robust to the
segment edges), for both command and response; amplitude ratio and phase
lag follow. Refuses when a segment carries too few cycles to fit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


class ProcedureError(ValueError):
    """The run is not a stepped-sine tracking sweep — analysis refuses."""


@dataclass(frozen=True)
class SweepMetrics:
    #: Frequency → (amplitude ratio, phase lag [deg]); the measured keys.
    per_frequency: dict[float, tuple[float, float]]
    #: −3 dB bandwidth [Hz]: log-interpolated between the straddling
    #: segments. None when every segment is still above 0.707 — the
    #: bandwidth then exceeds the swept range, and only the bound is known.
    bandwidth_hz: float | None
    #: Highest measured frequency [Hz] — the bound when bandwidth is None.
    max_frequency_hz: float

    def get(self, freq: float, tol: float = 0.05) -> tuple[float, float]:
        for f, v in self.per_frequency.items():
            if abs(f - freq) <= tol * max(freq, 1e-9):
                return v
        raise ProcedureError(f"频率 {freq} Hz 不在本次扫掠的测量点里："
                             f"{sorted(self.per_frequency)}")


def _crossing_times(t: np.ndarray, x: np.ndarray) -> np.ndarray:
    """Zero-crossing times [s], linearly interpolated between samples.

    Sample-index crossings quantise the period by dt; at 10 Hz recorded at
    200 Hz that is a 10 % frequency bias (measured: 9.09 Hz). Interpolating
    the crossing inside its sample interval removes it.
    """
    xc = x - float(np.mean(x))
    idx = np.where(np.diff(np.signbit(xc)))[0]
    if idx.size < 6:
        raise ProcedureError("命令信号过零点太少 —— 不是扫频工况")
    x0, x1 = xc[idx], xc[idx + 1]
    frac = np.abs(x0) / np.maximum(np.abs(x0) + np.abs(x1), 1e-12)
    return t[idx] + frac * (t[idx + 1] - t[idx])


def _half_periods(t: np.ndarray, x: np.ndarray) -> np.ndarray:
    return 2.0 * np.diff(_crossing_times(t, x))


def _fit_sine(t: np.ndarray, x: np.ndarray, freq: float) -> tuple[float, float]:
    """Least-squares A·sin(ωt)+B·cos(ωt)+C → (amplitude, phase [rad])."""
    w = 2.0 * math.pi * freq
    basis = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(basis, x, rcond=None)
    return math.hypot(coef[0], coef[1]), math.atan2(coef[1], coef[0])


def analyse_tracking_sweep(
    t: np.ndarray,
    channels: dict[str, np.ndarray],
    wheel: str = "fl",
) -> SweepMetrics:
    t = np.asarray(t, dtype=float)
    cmd = np.asarray(channels[f"delta_cmd_{wheel}"], dtype=float)
    resp = np.asarray(channels[f"delta_{wheel}"], dtype=float)
    if t.size < 100:
        raise ProcedureError("trace too short for a sweep analysis")
    dt = float(np.median(np.diff(t)))

    halves = _half_periods(t, cmd)
    freq_est = 1.0 / halves
    # Segment wherever the frequency moves by more than 25 %.
    bounds = [0]
    for i in range(1, freq_est.size):
        if abs(freq_est[i] - freq_est[i - 1]) > 0.25 * max(
                abs(freq_est[i - 1]), 1e-9):
            bounds.append(i)
    bounds.append(freq_est.size)
    if len(bounds) < 3:
        raise ProcedureError(
            "命令信号的频率没有分段 —— 不是 stepped-sine 扫频工况")

    ct = _crossing_times(t, cmd)
    per_frequency: dict[float, tuple[float, float]] = {}
    for a, b in zip(bounds[:-1], bounds[1:], strict=True):
        if b - a < 3:
            continue  # transition edge, not a segment
        i0 = int(np.searchsorted(t, ct[a]))
        i1 = int(np.searchsorted(t, ct[min(b, ct.size - 1)]))
        # Drop the boundary half-periods: they span the frequency change and
        # bias the median estimate (measured: a 5 Hz segment read 5.26 Hz
        # before the trim).
        inner = freq_est[a + 1:b - 1] if b - a > 3 else freq_est[a:b]
        if i1 - i0 < int(2.0 / max(np.median(inner), 1e-6) / dt):
            raise ProcedureError("某个频率段不足两个整周期，无法拟合")
        f = float(np.median(inner))
        amp_c, ph_c = _fit_sine(t[i0:i1], cmd[i0:i1], f)
        amp_r, ph_r = _fit_sine(t[i0:i1], resp[i0:i1], f)
        if amp_c < 1e-6:
            raise ProcedureError("某段命令幅值近零，无法定义幅值比")
        ratio = amp_r / amp_c
        lag = math.degrees((ph_c - ph_r) % (2.0 * math.pi))
        if lag > 180.0:
            lag -= 360.0
        per_frequency[round(f, 4)] = (float(ratio), float(lag))

    if len(per_frequency) < 2:
        raise ProcedureError("有效频率点不足两个，无法估计带宽")

    freqs = sorted(per_frequency)
    ratios = [per_frequency[f][0] for f in freqs]
    bandwidth = None
    for f_lo, f_hi, r_lo, r_hi in zip(freqs[:-1], freqs[1:],
                                      ratios[:-1], ratios[1:], strict=True):
        if r_lo >= 0.707 > r_hi:
            # Log-frequency interpolation of the 0.707 crossing.
            frac = ((math.log(r_lo) - math.log(0.707))
                    / max(math.log(r_lo) - math.log(r_hi), 1e-12))
            bandwidth = math.exp(math.log(f_lo)
                                 + frac * (math.log(f_hi) - math.log(f_lo)))
            break
    return SweepMetrics(per_frequency=per_frequency, bandwidth_hz=bandwidth,
                        max_frequency_hz=float(freqs[-1]))
