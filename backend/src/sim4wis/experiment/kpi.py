"""Server-side KPI computation over a RunResult's channel arrays.

This is the backend home of the metrics the frontend ScorePanel used to
compute on its 30-second rolling buffer — plus step-response metrics that only
make sense on a complete, reproducible run. KPIs are stored in the run's
meta.json so the analysis UI never has to re-derive them.

Base metrics (any maneuver):
    icr_dev_peak_m / icr_dev_rms_m   per-wheel steering-centre deviation
    yaw_rate_peak_dps                peak |yaw rate| [°/s]
    vy_peak_kmh                      peak |lateral velocity| [km/h]
    steer_energy_nms                 Σ_wheels Σ_t |motor torque|·dt [N·m·s]
    rack_force_peak_n                peak |rack force| over wheels [N]
    slip_alpha_peak_deg              peak |tyre side-slip| over wheels [°]
    speed_error_rms_kmh              RMS (vx − target) once a target exists

Step-response metrics (first step-kind steer segment, if any):
    yaw_gain_dps                     steady-state yaw rate per unit steering
    yaw_rise_time_s                  10 → 90 % rise time of |yaw rate|
    yaw_overshoot_pct                (peak − steady) / steady × 100
    yaw_settling_time_s              last time |yaw − steady| > 5 %·steady
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from sim4wis.experiment.schema import Experiment
from sim4wis.experiment.session import RunResult, WHEELS

R2D = 180.0 / math.pi


def compute_recorded_kpis(result: RunResult) -> dict[str, float]:
    """Descriptive motion metrics for arbitrary human input/channel selection.

    No invented speed target or step response is inferred from a recording.
    Unrecorded channels do not turn into zero-valued measurements.
    """
    out = {}
    for channel, metric, scale in (("yaw_rate", "yaw_rate_peak_dps", R2D), ("vy", "vy_peak_kmh", 3.6)):
        values = np.asarray(result.channels.get(channel, []), dtype=float)
        values = values[np.isfinite(values)]
        if values.size:
            out[metric] = float(np.max(np.abs(values))) * scale
    return out


def compute_kpis(result: RunResult, exp: Experiment) -> dict[str, Any]:
    t = np.asarray(result.t, dtype=np.float64)
    ch = {k: np.asarray(v, dtype=np.float64) for k, v in result.channels.items()}
    out: dict[str, Any] = {}
    if t.size == 0:
        return out

    dt = np.diff(t, prepend=t[0])

    # --- base metrics ---------------------------------------------------------
    # ICR deviation is only meaningful while genuinely turning: near-straight
    # driving puts the vehicle ICR kilometres away and the signed distances
    # blow up without any physical significance (the old frontend ScorePanel
    # had the same flaw). Gate on |yaw rate| > ~2.9 °/s.
    icr = np.vstack([ch[f"icr_dev_{w}"] for w in WHEELS])
    turning = np.abs(ch["yaw_rate"]) > 0.05
    finite = np.isfinite(icr) & turning[None, :]
    if np.any(finite):
        vals = np.abs(icr[finite])
        out["icr_dev_peak_m"] = float(np.max(vals))
        out["icr_dev_rms_m"] = float(np.sqrt(np.mean(vals**2)))
    else:
        out["icr_dev_peak_m"] = 0.0
        out["icr_dev_rms_m"] = 0.0

    out["yaw_rate_peak_dps"] = float(np.max(np.abs(ch["yaw_rate"])) * R2D)
    out["vy_peak_kmh"] = float(np.max(np.abs(ch["vy"])) * 3.6)

    energy = 0.0
    rack_peak = 0.0
    slip_peak = 0.0
    for w in WHEELS:
        energy += float(np.sum(np.abs(ch[f"motor_torque_{w}"]) * dt))
        rack_peak = max(rack_peak, float(np.max(np.abs(ch[f"rack_force_{w}"]))))
        slip_peak = max(slip_peak, float(np.max(np.abs(ch[f"slip_alpha_{w}"]))))
    out["steer_energy_nms"] = energy
    out["rack_force_peak_n"] = rack_peak
    out["slip_alpha_peak_deg"] = slip_peak * R2D

    # Speed tracking error vs the experiment's speed targets (per step).
    err = _speed_error(t, ch["vx"], exp)
    if err is not None:
        out["speed_error_rms_kmh"] = err

    # --- step-response metrics -------------------------------------------------
    step = _first_step_segment(exp)
    if step is not None:
        t0, t1, amp, t_step = step
        metrics = _step_metrics(t, ch["yaw_rate"], t0 + t_step, t1, amp)
        out.update(metrics)

    return out


def _speed_error(t: np.ndarray, vx: np.ndarray, exp: Experiment) -> float | None:
    """RMS speed error [km/h] over the portions with an active target.

    The first 2 s after each target change are excluded (acceleration
    transient, not a tracking property)."""
    target = np.full(t.size, np.nan)
    t_cursor = 0.0
    current: float | None = None
    for s in exp.maneuver.steps:
        if s.speed_kmh is not None:
            current = float(s.speed_kmh) / 3.6
            settle = 2.0 + float(s.speed_ramp_s)
            mask = (t >= t_cursor + settle) & (t < t_cursor + s.duration)
        else:
            mask = (t >= t_cursor) & (t < t_cursor + s.duration)
        if current is not None:
            target[mask] = current
        t_cursor += s.duration
    valid = np.isfinite(target)
    if not np.any(valid):
        return None
    return float(np.sqrt(np.mean((vx[valid] - target[valid]) ** 2)) * 3.6)


def _first_step_segment(exp: Experiment) -> tuple[float, float, float, float] | None:
    """Return (seg_start_t, seg_end_t, amplitude, lead_in) of the first
    step-kind steer segment, in run time coordinates."""
    t_cursor = 0.0
    for s in exp.maneuver.steps:
        if s.steer.kind == "step" and abs(s.steer.amplitude) > 1e-6:
            return t_cursor, t_cursor + s.duration, s.steer.amplitude, s.steer.t_step
        t_cursor += s.duration
    return None


def _step_metrics(
    t: np.ndarray,
    yaw: np.ndarray,
    t_on: float,
    t_end: float,
    amplitude: float,
) -> dict[str, float]:
    """Classic step-response numbers on the yaw-rate channel."""
    sel = (t >= t_on) & (t <= t_end)
    if np.count_nonzero(sel) < 5:
        return {}
    ts = t[sel]
    ys = yaw[sel] * R2D * float(np.sign(amplitude))

    # Steady value: mean of the last 20 % of the window.
    n_tail = max(3, int(0.2 * ys.size))
    steady = float(np.mean(ys[-n_tail:]))
    out: dict[str, float] = {}
    if abs(steady) < 1e-6:
        return out
    out["yaw_gain_dps"] = steady / abs(amplitude)

    peak = float(np.max(ys))
    out["yaw_overshoot_pct"] = max(0.0, (peak - steady) / abs(steady) * 100.0)

    # 10–90 % rise time.
    y10, y90 = 0.1 * steady, 0.9 * steady
    t10 = _first_crossing(ts, ys, y10)
    t90 = _first_crossing(ts, ys, y90)
    if t10 is not None and t90 is not None and t90 >= t10:
        out["yaw_rise_time_s"] = t90 - t10

    # 5 % settling time (relative to step onset).
    band = 0.05 * abs(steady)
    outside = np.abs(ys - steady) > band
    if np.any(outside):
        last_out = ts[np.max(np.nonzero(outside))]
        out["yaw_settling_time_s"] = float(last_out - ts[0])
    else:
        out["yaw_settling_time_s"] = 0.0
    return out


def _first_crossing(t: np.ndarray, y: np.ndarray, level: float) -> float | None:
    above = y >= level
    idx = np.nonzero(above)[0]
    if idx.size == 0:
        return None
    i = int(idx[0])
    if i == 0:
        return float(t[0])
    # Linear interpolation between the bracketing samples.
    y0, y1 = y[i - 1], y[i]
    if abs(y1 - y0) < 1e-12:
        return float(t[i])
    frac = (level - y0) / (y1 - y0)
    return float(t[i - 1] + frac * (t[i] - t[i - 1]))
