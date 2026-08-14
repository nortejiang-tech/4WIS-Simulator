"""Parameter identification (C3 · 3.1b) — fit steering parameters to bench data.

Stage 3.1a (residual.py) answers "how far is the model from the bench".
This stage answers "which parameters would make it closer": a deterministic
least-squares fit of a small set of steering scalars against the same
bench-CSV-vs-sim comparison, built on the residual panel's alignment and
channel machinery.

Method
------
Nelder-Mead in bounds-normalised coordinates, pure numpy (no scipy in the
venv, and a simplex method needs no gradients through a simulation). The
initial point is the parameter-library default, the initial simplex is a
fixed offset — nothing random anywhere, so the same inputs reproduce the
same fit bit for bit. The objective is the per-channel RMS of the
measurement-normalised residual ( ``(meas - sim) / std(meas)`` ) summed over
the comparison channels; normalising by the *measurement's* σ keeps the
metric fixed while the parameters move.

``steer_hand_angle`` is excluded from the objective by default: in the sim it
echoes the driver command to ~99.97 % and does not usefully depend on the
steering parameters, so it would only add a constant to the loss. The
alignment shift is held fixed *within* each optimiser pass (the weave is
driven by a periodic command, so phase differences caused by the parameters
belong in the residual, not in the clock offset) — but after the first pass
the alignment is redone against the fitted sim and the fit repeated, because
the first alignment runs against the default-parameter sim and a plant
mismatch biases its peak by a few ms.

Guardrail (same as 3.1a, and it holds until 3.1c says otherwise): the fit
outputs a report and a JSON record. It never rewrites the parameter library
— an identified value is a *finding*, and promoting it to a default is a
separate, reviewed decision.
"""

from __future__ import annotations

import copy
import html
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

import sim4wis
from sim4wis.calibration.residual import (
    align_time,
    channel_residuals,
    load_reference,
)
from sim4wis.experiment.session import SimSession
from sim4wis.study.expand import baseline_experiment
from sim4wis.study.spec import StudySpec


@dataclass(frozen=True)
class FitParam:
    """One fittable scalar: where it lives, where it starts, where it may go."""

    #: Sub-dataclass of SteeringSystemParams ("column" / "motor" / "rack").
    section: str
    #: Field name on that sub-dataclass.
    field: str
    #: Library default — also the fit's starting point.
    default: float
    #: Inclusive bounds. Wide enough to bracket plausible hardware, narrow
    #: enough that the optimiser cannot wander into fantasy.
    bounds: tuple[float, float]


#: The fittable set: the on-centre-relevant steering scalars. Deliberately
#: NOT the assist map (a table, not a scalar — 3.1c's business) and not
#: vehicle-side numbers like the axle cornering split (same reason).
FIT_PARAMS: dict[str, FitParam] = {
    "column.torsion_stiffness_nm_per_deg": FitParam(
        "column", "torsion_stiffness_nm_per_deg", 2.0, (0.5, 6.0)),
    "column.damping": FitParam(
        "column", "damping", 0.30, (0.05, 3.0)),
    "column.coulomb_friction": FitParam(
        "column", "coulomb_friction", 0.15, (0.0, 1.5)),
    "rack.coulomb_friction_n": FitParam(
        "rack", "coulomb_friction_n", 260.0, (0.0, 1200.0)),
    "rack.viscous_n_per_mps": FitParam(
        "rack", "viscous_n_per_mps", 900.0, (100.0, 3000.0)),
}

#: Channel removed from the objective (parameter-independent command echo).
EXCLUDED_FROM_LOSS = ("steer_hand_angle",)

#: Optimiser budget. A 5-parameter simplex typically converges in 150-250
#: evaluations; the cap is a guard for a flat objective, not a target.
DEFAULT_MAX_EVALS = 300


def _load_procedure_raw(procedure_path: str | Path) -> dict[str, Any]:
    raw = yaml.safe_load(Path(procedure_path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{procedure_path}: not a study-spec mapping")
    return raw


def _inject_overrides(
    raw: dict[str, Any],
    values: dict[str, float],
) -> dict[str, Any]:
    """Copy the spec with the fit parameters written into the overrides."""
    raw = copy.deepcopy(raw)
    vehicle = raw.setdefault("baseline", {}).setdefault("vehicle", {})
    overrides = vehicle.setdefault("overrides", {})
    steer = overrides.setdefault("steering_system", {})
    if isinstance(steer.get("enabled"), bool) and not steer["enabled"]:
        raise ValueError(
            "the procedure has steering_system.enabled: false — the fittable "
            "parameters have no effect on such a run, so nothing can be "
            "identified from it"
        )
    steer.setdefault("enabled", True)
    for name, value in values.items():
        p = FIT_PARAMS[name]
        block = steer.setdefault(p.section, {})
        block[p.field] = float(value)
    return raw


def run_sim_with_values(
    raw: dict[str, Any],
    values: dict[str, float],
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """One sim run of the procedure with the given parameter values."""
    spec = StudySpec.model_validate(_inject_overrides(raw, values))
    result = SimSession(baseline_experiment(spec)).run()
    t = np.asarray(result.t, dtype=float)
    channels = {k: np.asarray(v, dtype=float) for k, v in result.channels.items()}
    return t, channels


def nelder_mead(
    f: Callable[[np.ndarray], float],
    x0: np.ndarray,
    *,
    lo: np.ndarray,
    hi: np.ndarray,
    max_evals: int = 200,
    xatol: float = 1e-3,
    fatol: float = 1e-6,
    memo: dict[tuple[float, ...], float] | None = None,
    progress: Callable[[int, float, np.ndarray], None] | None = None,
) -> tuple[np.ndarray, float, int]:
    """Bounded Nelder-Mead in normalised coordinates, deterministic.

    Works on ``u ∈ [0,1]^n`` with ``x = lo + u·(hi-lo)`` clamped, initial
    simplex = x0 plus a fixed 10 %-of-range offset per axis. Evaluations are
    memoised on the rounded x-tuple so repeated vertices cost nothing.
    """
    n = x0.size
    span = hi - lo
    evals = [0]

    def to_x(u: np.ndarray) -> np.ndarray:
        return lo + np.clip(u, 0.0, 1.0) * span

    def eval_f(u: np.ndarray) -> float:
        x = to_x(u)
        evals[0] += 1
        if memo is not None:
            key = tuple(np.round(x, 12))
            if key in memo:
                evals[0] -= 1
                return memo[key]
        val = f(x)
        if memo is not None:
            memo[tuple(np.round(x, 12))] = val
        return val

    u0 = np.clip((x0 - lo) / span, 0.0, 1.0)
    # Fixed initial simplex: the start point plus one vertex per axis with a
    # 0.1 offset (mirrored if that leaves [0,1]). No randomness anywhere.
    simplex = [u0]
    for i in range(n):
        v = u0.copy()
        v[i] = v[i] + 0.1 if v[i] + 0.1 <= 1.0 else v[i] - 0.1
        simplex.append(v)
    simplex = np.array(simplex, dtype=float)
    vals = np.array([eval_f(u) for u in simplex])
    n_evals = evals[0]

    alpha, gamma, rho, sigma = 1.0, 2.0, 0.5, 0.5
    while evals[0] < max_evals:
        order = np.argsort(vals, kind="stable")
        simplex, vals = simplex[order], vals[order]
        if (np.max(np.abs(simplex[1:] - simplex[0])) <= xatol
                and vals[-1] - vals[0] <= fatol):
            break
        centroid = simplex[:-1].mean(axis=0)
        ur = centroid + alpha * (centroid - simplex[-1])
        fr = eval_f(ur)
        if fr < vals[0]:
            ue = centroid + gamma * (centroid - simplex[-1])
            fe = eval_f(ue)
            if fe < fr:
                simplex[-1], vals[-1] = ue, fe
            else:
                simplex[-1], vals[-1] = ur, fr
        elif fr < vals[-2]:
            simplex[-1], vals[-1] = ur, fr
        else:
            uc = centroid + rho * (simplex[-1] - centroid)
            fc = eval_f(uc)
            if fc < vals[-1]:
                simplex[-1], vals[-1] = uc, fc
            else:
                for i in range(1, n + 1):
                    simplex[i] = simplex[0] + sigma * (simplex[i] - simplex[0])
                    vals[i] = eval_f(simplex[i])
        n_evals = evals[0]
        if progress is not None:
            best = int(np.argmin(vals))
            progress(n_evals, float(vals[best]), to_x(simplex[best]))

    best = int(np.argmin(vals))
    return to_x(simplex[best]), float(vals[best]), n_evals


def fit_reference(
    reference_csv: str | Path,
    procedure_path: str | Path,
    *,
    params: list[str] | None = None,
    channels: list[str] | None = None,
    max_evals: int = DEFAULT_MAX_EVALS,
    passes: int = 2,
    out_html: str | Path | None = None,
    progress: Callable[[int, float, np.ndarray], None] | None = None,
) -> dict[str, Any]:
    """Fit the named parameters to the bench CSV; report, never rewrite.

    Returns the full record (also the source of the JSON provenance file and
    the HTML report): defaults vs fitted values, residual table before and
    after, evaluation count and wall time.

    Two passes by default: the first alignment runs against the
    *default-parameter* sim, and the recorded hand-wheel angle the alignment
    keys on follows the command through the grip impedance — a slightly
    different plant therefore shows up as a few ms of spurious clock offset,
    which a single pass pays for as a residual floor. After the first fit the
    alignment is redone against the *fitted* sim (the parameter mismatch now
    absorbed) and the fit repeated, which removes that floor. Deterministic
    throughout: same inputs, same passes, same result.
    """
    names = list(params) if params else list(FIT_PARAMS)
    unknown = [p for p in names if p not in FIT_PARAMS]
    if unknown:
        raise ValueError(
            f"unknown fit parameter(s) {unknown}; fittable: {list(FIT_PARAMS)}"
        )
    fps = [FIT_PARAMS[p] for p in names]
    x0 = np.array([p.default for p in fps], dtype=float)
    lo = np.array([p.bounds[0] for p in fps], dtype=float)
    hi = np.array([p.bounds[1] for p in fps], dtype=float)

    meas_t, meas_ch = load_reference(reference_csv)
    raw = _load_procedure_raw(procedure_path)

    cache: dict[tuple[float, ...], tuple[np.ndarray, dict[str, np.ndarray]]] = {}

    def run_cached(values: dict[str, float]):
        # Missing keys mean the library default; the cache key is the full
        # effective vector so a partial dict and its explicit form collide.
        key = tuple(np.round(
            [values.get(n, FIT_PARAMS[n].default) for n in names], 12))
        if key not in cache:
            cache[key] = run_sim_with_values(raw, values)
        return cache[key]

    sim_t0, sim_ch0 = run_cached({})

    loss_channels = [
        c for c in meas_ch
        if c in sim_ch0 and c not in EXCLUDED_FROM_LOSS
    ] if channels is None else [
        c for c in channels
        if c in meas_ch and c in sim_ch0 and c not in EXCLUDED_FROM_LOSS
    ]
    if not loss_channels:
        raise ValueError(
            "no comparison channels carry parameter information "
            f"(common channels minus {list(EXCLUDED_FROM_LOSS)} is empty)"
        )
    # A channel that is constant in the measurement cannot constrain the fit
    # (its σ-normalised residual is undefined); drop it, and only refuse when
    # nothing informative remains.
    dropped = [c for c in loss_channels if float(np.std(meas_ch[c])) < 1e-9]
    loss_channels = [c for c in loss_channels if c not in dropped]
    if not loss_channels:
        raise ValueError(
            f"every comparison channel is constant in the reference ({dropped})"
        )
    meas_sigma = {c: float(np.std(meas_ch[c])) for c in loss_channels}

    def make_objective(shift: float) -> Callable[[np.ndarray], float]:
        def objective(x: np.ndarray) -> float:
            sim_t, sim_ch = run_cached({n: float(v) for n, v in zip(names, x, strict=True)})
            mask = (sim_t >= float(meas_t[0]) + shift - 1e-9) & (
                sim_t <= float(meas_t[-1]) + shift + 1e-9)
            if int(np.count_nonzero(mask)) < 8:
                return 1e6  # outside the overlap — no objective, no gradient either
            total = 0.0
            for c in loss_channels:
                m = np.interp(sim_t[mask], meas_t + shift, meas_ch[c])
                d = (m - sim_ch[c][mask]) / meas_sigma[c]
                total += float(np.sqrt((d * d).mean()))
            return total
        return objective

    t_start = time.perf_counter()
    pass_records = []
    shift, align_channel = align_time(meas_t, meas_ch, sim_t0, sim_ch0)
    x_best, fitted = x0.copy(), {
            n: float(v) for n, v in zip(names, x0, strict=True)}
    total_evals = 0
    for pass_i in range(max(1, int(passes))):
        if pass_i > 0:
            # Re-align on the fitted sim: the parameter mismatch that biased
            # the first alignment is now absorbed, so the peak sits at the
            # true clock offset.
            sim_tb, sim_cb = run_cached(fitted)
            shift, align_channel = align_time(meas_t, meas_ch, sim_tb, sim_cb)
        memo: dict[tuple[float, ...], float] = {}
        x_best, f_best, n_evals = nelder_mead(
            make_objective(shift), x0 if pass_i == 0 else x_best,
            lo=lo, hi=hi, max_evals=max_evals,
            memo=memo, progress=progress,
        )
        total_evals += n_evals
        fitted = {n: float(v) for n, v in zip(names, x_best, strict=True)}
        pass_records.append({
            "pass": pass_i + 1, "shift_s": shift,
            "loss_after": f_best, "n_evals": n_evals,
        })
    wall_s = time.perf_counter() - t_start

    residuals_before = channel_residuals(
        meas_t, meas_ch, sim_t0, sim_ch0, shift)
    sim_t1, sim_ch1 = run_cached(fitted)
    residuals_after = channel_residuals(
        meas_t, meas_ch, sim_t1, sim_ch1, shift)
    loss_final = make_objective(shift)

    record = {
        "sim4wis_version": sim4wis.__version__,
        "reference": str(Path(reference_csv)),
        "procedure": str(Path(procedure_path)),
        "align": {"shift_s": shift, "channel": align_channel},
        "loss_channels": loss_channels,
        "dropped_constant_channels": dropped,
        "params": {
            n: {
                "default": float(FIT_PARAMS[n].default),
                "fitted": fitted[n],
                "bounds": list(FIT_PARAMS[n].bounds),
            }
            for n in names
        },
        "loss": {"before": float(loss_final(x0)), "after": float(loss_final(x_best)),
                 "n_evals": total_evals, "wall_s": wall_s, "passes": pass_records},
        "residuals_before": residuals_before,
        "residuals_after": residuals_after,
        "guardrail": "identification only — the parameter library is not "
                     "rewritten (promotion is a separate, reviewed step)",
    }

    if out_html is not None:
        record["report"] = str(
            render_fit_report(out_html, record,
                              meas_t=meas_t, meas_ch=meas_ch,
                              sim_t0=sim_t0, sim_ch0=sim_ch0,
                              sim_t1=sim_t1, sim_ch1=sim_ch1,
                              shift=shift)
        )
    return record


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


def _svg_overlay(x: np.ndarray, series: list[tuple[np.ndarray, str, str]],
                 w: float = 640.0, h: float = 200.0) -> str:
    """One inline SVG with N polylines (meas vs sim@default vs sim@fitted)."""
    x = x - x[0]
    xmax = max(float(x[-1]), 1e-9)
    lo = min(float(np.nanmin(y)) for y, _, _ in series)
    hi = max(float(np.nanmax(y)) for y, _, _ in series)
    span = max(hi - lo, 1e-9)

    def pts(y: np.ndarray) -> str:
        return " ".join(
            f"{float(xx) / xmax * (w - 40) + 30:.1f},"
            f"{h - 20 - (float(yy) - lo) / span * (h - 40):.1f}"
            for xx, yy in zip(x, y, strict=False)
        )

    lines = "".join(
        f'<polyline points="{pts(y)}" fill="none" stroke="{color}" '
        f'stroke-width="1.3"/>' for y, _, color in series)
    legend = "".join(
        f'<text x="6" y="{14 + 12 * i}" fill="{color}" font-size="10">'
        f"{html.escape(label)}</text>"
        for i, (_, label, color) in enumerate(series))
    return (
        f'<svg viewBox="0 0 {w} {h}" xmlns="http://www.w3.org/2000/svg" '
        f'style="width:100%;max-width:640px;background:#0f172a;'
        f'border-radius:6px">{lines}{legend}</svg>'
    )


def _downsample(x: np.ndarray, y: np.ndarray, cap: int = 800
                ) -> tuple[np.ndarray, np.ndarray]:
    if x.size <= cap:
        return x, y
    idx = np.linspace(0, x.size - 1, cap).astype(int)
    return x[idx], y[idx]


def render_fit_report(
    out_html: str | Path,
    record: dict[str, Any],
    *,
    meas_t: np.ndarray,
    meas_ch: dict[str, np.ndarray],
    sim_t0: np.ndarray,
    sim_ch0: dict[str, np.ndarray],
    sim_t1: np.ndarray,
    sim_ch1: dict[str, np.ndarray],
    shift: float,
) -> Path:
    """Self-contained HTML: parameter table, before/after residuals, overlays."""
    rows = []
    for n, p in record["params"].items():
        rel = (p["fitted"] - p["default"]) / max(abs(p["default"]), 1e-12)
        rows.append(
            "<tr>"
            f"<td>{html.escape(n)}</td><td>{p['default']:.4g}</td>"
            f"<td>{p['fitted']:.4g}</td>"
            f"<td>{p['bounds'][0]:.4g} – {p['bounds'][1]:.4g}</td>"
            f"<td>{rel:+.1%}</td>"
            "</tr>"
        )
    res_rows = []
    for c in record["residuals_after"]:
        b = record["residuals_before"].get(c, {})
        a = record["residuals_after"][c]
        res_rows.append(
            "<tr>"
            f"<td>{html.escape(c)}</td>"
            f"<td>{b.get('rms', float('nan')):.4g}</td>"
            f"<td>{a['rms']:.4g}</td>"
            f"<td>{b.get('rms_over_std', float('nan')):.3f}</td>"
            f"<td>{a['rms_over_std']:.3f}</td>"
            f"<td>{a['corr']:.4f}</td>"
            "</tr>"
        )
    plots = []
    for c in record["loss_channels"]:
        m_on_sim = np.interp(sim_t1, meas_t + shift, meas_ch[c])
        x, m = _downsample(sim_t1, m_on_sim)
        _, s0 = _downsample(sim_t1, sim_ch0[c])
        _, s1 = _downsample(sim_t1, sim_ch1[c])
        plots.append(
            f"<h3>{html.escape(c)}</h3>"
            + _svg_overlay(x, [
                (m, c + " (meas)", "#60a5fa"),
                (s0, c + " (sim, defaults)", "#94a3b8"),
                (s1, c + " (sim, fitted)", "#fbbf24"),
            ])
        )
    loss = record["loss"]
    body = (
        "<html><head><meta charset='utf-8'><title>fit report</title>"
        "<style>body{font-family:system-ui;background:#0b1220;color:#e2e8f0;"
        "max-width:760px;margin:2rem auto;padding:0 1rem}"
        "table{border-collapse:collapse;width:100%;font-size:12px;"
        "margin-bottom:1rem}"
        "th,td{border:1px solid #334155;padding:4px 8px;text-align:right}"
        "th{background:#1e293b}td:first-child{text-align:left}"
        "h2,h3{margin-bottom:.4rem}.muted{color:#94a3b8}</style></head><body>"
        f"<h2>Parameter identification — {html.escape(record['reference'])}"
        f" vs {html.escape(record['procedure'])}</h2>"
        f"<p class='muted'>align {shift:+.3f} s on "
        f"{html.escape(str(record['align']['channel']))} · "
        f"{loss['n_evals']} evals in {loss['wall_s']:.1f} s · "
        f"loss {loss['before']:.3f} → {loss['after']:.3f} · "
        f"sim4wis {html.escape(record['sim4wis_version'])}</p>"
        "<h2>Parameters</h2><table><tr><th>parameter</th><th>default</th>"
        "<th>fitted</th><th>bounds</th><th>Δ vs default</th></tr>"
        + "".join(rows)
        + "</table><h2>Residuals (before → after)</h2>"
        "<table><tr><th>channel</th><th>RMS def</th><th>RMS fit</th>"
        "<th>RMS/σ def</th><th>RMS/σ fit</th><th>corr fit</th></tr>"
        + "".join(res_rows)
        + "</table><h2>Overlays</h2>" + "".join(plots)
        + "<p><em>Identification only — nothing here rewrites the parameter "
        "library; promoting a fitted value to a default is a separate, "
        "reviewed decision (3.1c).</em></p></body></html>"
    )
    out = Path(out_html)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(body, encoding="utf-8")
    return out
