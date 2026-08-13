"""Residual panel (C3 · 3.1a) — measured bench data vs the same-condition sim.

The first stage of the calibration workbench. It answers one question: given a
bench CSV measured under a known condition, how far is the model's prediction
from it, channel by channel?

Contract
--------
* reference CSV: a header row of channel names plus a ``t`` column in seconds.
  Channels are any subset of the sim's scalar channels
  (steer_hand_angle / steer_hand_torque / ay / yaw_rate / vx / ...).
* procedure: a study-spec file (procedures/*.yaml) that pins the SAME condition
  the bench ran — speed, weave frequency and amplitude, plant enabled. One sim
  run of its baseline is executed at its record_hz.
* time alignment: the two records do not share a clock. The shift is found by
  cross-correlation on the first periodic channel available in both
  (preferring steer_hand_angle) over a coarse grid.
* residuals: per channel, over the aligned overlap — RMS, peak, mean-abs, and
  the correlation between the two traces.
* report: one self-contained HTML file with the residual table plus overlay
  plots (inline SVG, no dependencies).

This stage deliberately outputs residuals ONLY. It never rewrites the
parameter library — fitting is stage 3.1b, and a residual panel that silently
changed parameters would mistake a fit for truth.
"""

from __future__ import annotations

import csv
import html
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from sim4wis.experiment.session import SCALAR_CHANNELS, SimSession
from sim4wis.study.expand import baseline_experiment
from sim4wis.study.spec import StudySpec

#: Channels tried, in order, for the time alignment — a periodic driver-side
#: signal first, vehicle responses later.
ALIGN_PREFERENCE = (
    "steer_hand_angle",
    "steer_hand_torque",
    "driver_steering",
    "yaw_rate",
    "ay",
    "vx",
)

#: Coarse grid step for the alignment search [s].
ALIGN_GRID_S = 0.01
#: Alignment search bound [s].
ALIGN_MAX_SHIFT_S = 3.0
#: Alignment comparison grid rate [Hz].
ALIGN_GRID_HZ = 50.0

#: Plot downsampling cap per trace.
MAX_PLOT_POINTS = 800


def load_reference(path: str | Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Parse a bench CSV into (t, channels) with standard names.

    Refuses unknown column names rather than guessing: a misspelled header
    must fail loudly, not silently produce a pretty empty report.
    """
    p = Path(path)
    with p.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        header = list(reader.fieldnames or [])
        rows = list(reader)
    if not rows:
        raise ValueError(f"{p}: empty reference CSV")
    if "t" not in header:
        raise ValueError(f"{p}: reference CSV needs a 't' column (seconds)")
    unknown = [c for c in header if c != "t" and c not in SCALAR_CHANNELS]
    if unknown:
        raise ValueError(
            f"{p}: unknown channel column(s) {unknown}; known: {sorted(SCALAR_CHANNELS)}"
        )
    t = np.array([float(r["t"]) for r in rows], dtype=float)
    channels: dict[str, np.ndarray] = {}
    for c in header:
        if c == "t":
            continue
        vals = [np.nan if r[c] == "" else float(r[c]) for r in rows]
        channels[c] = np.array(vals, dtype=float)
    order = np.argsort(t)
    return t[order], {k: v[order] for k, v in channels.items()}


def run_sim_channels(procedure_path: str | Path) -> tuple[np.ndarray, dict[str, np.ndarray], dict[str, Any]]:
    """Run the procedure's baseline once; return (t, channels, meta)."""
    raw = yaml.safe_load(Path(procedure_path).read_text(encoding="utf-8")) or {}
    spec = StudySpec.model_validate(raw)
    exp = baseline_experiment(spec)
    result = SimSession(exp).run()
    t = np.asarray(result.t, dtype=float)
    channels = {k: np.asarray(v, dtype=float) for k, v in result.channels.items()}
    meta = {"study": spec.study, "record_hz": float(exp.record_hz)}
    return t, channels, meta


def align_time(
    meas_t: np.ndarray,
    meas_ch: dict[str, np.ndarray],
    sim_t: np.ndarray,
    sim_ch: dict[str, np.ndarray],
) -> tuple[float, str | None]:
    """Best time shift so that meas_t - shift lines up with sim_t.

    Cross-correlation on a common 50 Hz grid over a coarse shift lattice. The
    reference channel is the first ALIGN_PREFERENCE entry present in both.
    """
    ref = next((c for c in ALIGN_PREFERENCE if c in meas_ch and c in sim_ch), None)
    if ref is None:
        return 0.0, None
    lo = max(float(meas_t[0]), float(sim_t[0])) + 1.0
    hi = min(float(meas_t[-1]), float(sim_t[-1])) - 1.0
    if hi - lo < 1.0:
        return 0.0, ref
    grid = np.arange(lo, hi, 1.0 / ALIGN_GRID_HZ)
    m = np.interp(grid, meas_t, meas_ch[ref])
    s = np.interp(grid, sim_t, sim_ch[ref])
    m = m - m.mean()
    s = s - s.mean()
    m_norm = float(np.sqrt((m * m).sum()))
    if m_norm < 1e-12 or float(np.sqrt((s * s).sum())) < 1e-12:
        return 0.0, ref
    best_shift, best_corr = 0.0, -2.0
    shift = -ALIGN_MAX_SHIFT_S
    while shift <= ALIGN_MAX_SHIFT_S + 1e-9:
        s_shifted = np.interp(grid + shift, sim_t, sim_ch[ref])
        s_shifted = s_shifted - s_shifted.mean()
        s_norm = float(np.sqrt((s_shifted * s_shifted).sum()))
        # Normalise by the *shifted* trace's own norm — edge clamping changes
        # it, and reusing the unshifted norm misranks near-perfect matches.
        corr = float((m * s_shifted).sum() / (m_norm * s_norm)) if s_norm > 1e-12 else -2.0
        if corr > best_corr:
            best_corr, best_shift = corr, float(shift)
        shift += ALIGN_GRID_S
    return best_shift, ref


def channel_residuals(
    meas_t: np.ndarray,
    meas_ch: dict[str, np.ndarray],
    sim_t: np.ndarray,
    sim_ch: dict[str, np.ndarray],
    shift: float,
    channels: list[str] | None = None,
) -> dict[str, dict[str, float]]:
    """Per-channel error over the aligned overlap (meas interpolated onto sim grid)."""
    common = [c for c in meas_ch if c in sim_ch]
    if channels:
        common = [c for c in channels if c in common]
    # align_time returns shift with sim(t + shift) ≈ meas(t); a sim-time
    # sample therefore corresponds to measurement time (sim_t - shift), i.e.
    # meas interpolated at (sim_t - shift) == meas_t + shift.
    lo = float(meas_t[0]) + shift
    hi = float(meas_t[-1]) + shift
    mask = (sim_t >= lo - 1e-9) & (sim_t <= hi + 1e-9)
    if int(np.count_nonzero(mask)) < 8:
        raise ValueError("aligned overlap is too short to compare")
    out: dict[str, dict[str, float]] = {}
    for c in common:
        m = np.interp(sim_t[mask], meas_t + shift, meas_ch[c])
        s = sim_ch[c][mask]
        d = m - s
        n = int(d.size)
        rms = float(np.sqrt((d * d).sum() / n))
        peak = float(np.max(np.abs(d)))
        mean_abs = float(np.abs(d).mean())
        sd = float(np.std(s))
        corr = 0.0
        if sd > 1e-12 and float(np.std(m)) > 1e-12:
            corr = float(np.corrcoef(m, s)[0, 1])
        out[c] = {
            "n": n, "rms": rms, "peak": peak, "mean_abs": mean_abs,
            "sim_std": sd, "rms_over_std": rms / sd if sd > 1e-12 else float("nan"),
            "corr": corr,
        }
    return out


def _downsample(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if x.size <= MAX_PLOT_POINTS:
        return x, y
    idx = np.linspace(0, x.size - 1, MAX_PLOT_POINTS).astype(int)
    return x[idx], y[idx]


def _svg_overlay(x: np.ndarray, a: np.ndarray, b: np.ndarray,
                 label_a: str, label_b: str) -> str:
    x = x - x[0]
    xmax = max(float(x[-1]), 1e-9)
    lo = float(min(np.nanmin(a), np.nanmin(b)))
    hi = float(max(np.nanmax(a), np.nanmax(b)))
    span = max(hi - lo, 1e-9)
    w, h = 640.0, 180.0

    def pts(y: np.ndarray) -> str:
        coords = [
            f"{float(xx) / xmax * (w - 40) + 30:.1f},"
            f"{h - 20 - (float(yy) - lo) / span * (h - 40):.1f}"
            for xx, yy in zip(x, y, strict=False)
        ]
        return " ".join(coords)

    return (
        f'<svg viewBox="0 0 {w} {h}" xmlns="http://www.w3.org/2000/svg" '
        f'style="width:100%;max-width:640px;background:#0f172a;border-radius:6px">'
        f'<polyline points="{pts(a)}" fill="none" stroke="#60a5fa" stroke-width="1.4"/>'
        f'<polyline points="{pts(b)}" fill="none" stroke="#fbbf24" stroke-width="1.4"/>'
        f'<text x="6" y="14" fill="#e2e8f0" font-size="10">{html.escape(label_a)} (meas)</text>'
        f'<text x="6" y="26" fill="#e2e8f0" font-size="10">{html.escape(label_b)} (sim)</text>'
        "</svg>"
    )


def render_report(
    out_html: str | Path,
    *,
    title: str,
    meta: dict[str, Any],
    shift_s: float,
    align_channel: str | None,
    meas_t: np.ndarray,
    meas_ch: dict[str, np.ndarray],
    sim_t: np.ndarray,
    sim_ch: dict[str, np.ndarray],
    residuals: dict[str, dict[str, float]],
) -> Path:
    rows = []
    for c, r in residuals.items():
        rows.append(
            "<tr>"
            f"<td>{html.escape(c)}</td>"
            f"<td>{r['rms']:.4g}</td><td>{r['peak']:.4g}</td>"
            f"<td>{r['mean_abs']:.4g}</td>"
            f"<td>{r['sim_std']:.4g}</td>"
            f"<td>{r['rms_over_std']:.3f}</td>"
            f"<td>{r['corr']:.4f}</td><td>{r['n']}</td>"
            "</tr>"
        )
    plots = []
    for c in residuals:
        m_on_sim = np.interp(sim_t, meas_t + shift_s, meas_ch[c])
        x, a = _downsample(sim_t, m_on_sim)
        _, b = _downsample(sim_t, sim_ch[c])
        plots.append(
            f'<h3>{html.escape(c)}</h3>'
            + _svg_overlay(x, a, b, c + " (meas)", c + " (sim)")
        )
    body = (
        "<html><head><meta charset='utf-8'><title>residual report</title>"
        "<style>body{font-family:system-ui;background:#0b1220;color:#e2e8f0;"
        "max-width:760px;margin:2rem auto;padding:0 1rem}"
        "table{border-collapse:collapse;width:100%;font-size:12px}"
        "th,td{border:1px solid #334155;padding:4px 8px;text-align:right}"
        "th{background:#1e293b}td:first-child{text-align:left}"
        "h2,h3{margin-bottom:.4rem}</style></head><body>"
        f"<h2>{html.escape(title)}</h2>"
        f"<p>align shift: {shift_s:+.3f} s on {html.escape(align_channel or '-')} · "
        f"sim record {meta.get('record_hz', float('nan')):.0f} Hz · "
        f"study {html.escape(str(meta.get('study', '')))}</p>"
        "<table><tr><th>channel</th><th>RMS</th><th>peak</th><th>mean|·|</th>"
        "<th>sim σ</th><th>RMS/σ</th><th>corr</th><th>n</th></tr>"
        + "".join(rows)
        + "</table><h2>overlays</h2>"
        + "".join(plots)
        + "<p><em>Residuals only — nothing here rewrites the parameter library "
        "(fitting is stage 3.1b).</em></p></body></html>"
    )
    out = Path(out_html)
    out.write_text(body, encoding="utf-8")
    return out


def residual_panel(
    reference_csv: str | Path,
    procedure_path: str | Path,
    out_html: str | Path | None = None,
    channels: list[str] | None = None,
) -> dict[str, Any]:
    """One command: bench CSV + same-condition procedure -> residual report."""
    meas_t, meas_ch = load_reference(reference_csv)
    sim_t, sim_ch, meta = run_sim_channels(procedure_path)
    shift, ref = align_time(meas_t, meas_ch, sim_t, sim_ch)
    residuals = channel_residuals(meas_t, meas_ch, sim_t, sim_ch, shift, channels)
    title = f"{Path(reference_csv).name} vs {Path(procedure_path).name}"
    if out_html is None:
        out_html = Path(reference_csv).with_suffix(".residual.html")
    path = render_report(
        out_html, title=title, meta=meta, shift_s=shift, align_channel=ref,
        meas_t=meas_t, meas_ch=meas_ch, sim_t=sim_t, sim_ch=sim_ch,
        residuals=residuals,
    )
    return {
        "reference": str(Path(reference_csv)),
        "procedure": str(Path(procedure_path)),
        "shift_s": shift,
        "align_channel": ref,
        "residuals": residuals,
        "report": str(path),
    }
