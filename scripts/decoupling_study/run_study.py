"""Run the whole steering-decoupling matrix and write one JSON.

    python scripts/decoupling_study/run_study.py [--out PATH] [--jobs N]

Every number the report quotes comes from this file, so it is written to be
re-runnable: fixed seeds are not needed (the model is deterministic), and the
output carries the git commit and the parameter set it was produced under.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "backend" / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

from architectures import ALGORITHMS, ALGO_BY_KEY, BY_KEY, DEFAULT_LAW, LADDER  # noqa: E402
from harness import (  # noqa: E402
    G, calibrate_steer_for_ay, frequency_point, step_steer, steady_state,
    turning_circle,
)
from sim4wis.core.state import VehicleParams  # noqa: E402

AY_TARGET = 4.0            # m/s^2 — ISO 7401 reference step level
STEP_SPEED = 100.0         # km/h
SS_SPEEDS = (30, 45, 60, 75, 90, 105, 120)
FREQS = (0.2, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0)
LOW_MU = 0.40


def _mp(arch_key, algo_key):
    """Mode params for a rung/law pair, including the variable-ratio flag."""
    algo = ALGO_BY_KEY[algo_key]
    mp = dict(algo.mode_params)
    arch = BY_KEY[arch_key]
    if not arch.variable_ratio:
        # Without steer-by-wire the front angle is whatever the column gives;
        # the experiment commands the road-wheel angle directly either way, so
        # the flag changes nothing about how the input is applied here. It is
        # recorded so the report can say which rungs could also have reshaped
        # the front angle and chose not to.
        mp["fixed_column"] = True
    return mp


# ── individual jobs (module-level so they pickle) ──────────────────────────

def job_calibrate(args):
    arch_key, algo_key, v_kmh, mu = args
    arch = BY_KEY[arch_key]
    delta, ay = calibrate_steer_for_ay(arch, _mp(arch_key, algo_key), v_kmh,
                                       AY_TARGET, mu)
    return dict(arch=arch_key, algo=algo_key, v=v_kmh, mu=mu,
                delta_deg=delta, ay=ay)


def job_step(args):
    arch_key, algo_key, v_kmh, delta_deg, mu, trace = args
    r = step_steer(BY_KEY[arch_key], _mp(arch_key, algo_key), v_kmh, delta_deg,
                   mu, trace=trace)
    r.update(arch=arch_key, algo=algo_key, v=v_kmh, mu=mu)
    return r


def job_steady(args):
    arch_key, algo_key, v_kmh, delta_deg, mu = args
    r = steady_state(BY_KEY[arch_key], _mp(arch_key, algo_key), v_kmh, delta_deg, mu)
    r.update(arch=arch_key, algo=algo_key, v=v_kmh, mu=mu)
    return r


def job_freq(args):
    arch_key, algo_key, v_kmh, f, delta_deg, mu = args
    r = frequency_point(BY_KEY[arch_key], _mp(arch_key, algo_key), v_kmh, f,
                        delta_deg, mu)
    r.update(arch=arch_key, algo=algo_key, v=v_kmh, mu=mu)
    return r


def job_circle(args):
    arch_key, algo_key = args
    r = turning_circle(BY_KEY[arch_key], _mp(arch_key, algo_key))
    r.update(arch=arch_key, algo=algo_key)
    return r


def understeer_gradient(rows, L):
    """K = d(delta - L/R)/d(a_y), the ISO 4138 definition, by least squares."""
    rows = [r for r in rows if abs(r["ay"]) > 0.3 and r["util_f"] < 0.95]
    if len(rows) < 3:
        return float("nan"), float("nan")
    ay = np.array([abs(r["ay"]) for r in rows])
    dl = np.array([abs(r["delta_f"]) for r in rows])
    R = np.array([abs(r["vx"]) ** 2 / abs(r["ay"]) for r in rows])
    k, _ = np.polyfit(ay, dl - L / R, 1)
    return float(k) * (180.0 / math.pi) * G, float(np.max(ay))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(ROOT / "docs" / "reports" /
                                         "decoupling_study_data.json"))
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 4) - 1))
    args = ap.parse_args()

    t0 = time.time()
    stage_dir = Path(args.out).with_suffix("")
    stage_dir.mkdir(parents=True, exist_ok=True)

    def stage(name, value):
        """Checkpoint a stage. The matrix takes minutes; losing all of it to a
        typo in the last stage (which is exactly what happened the first time)
        is not an acceptable failure mode."""
        (stage_dir / f"{name}.json").write_text(json.dumps(value))
        return value

    p = VehicleParams()
    L = p.wheelbase
    pool = ProcessPoolExecutor(max_workers=args.jobs)
    log = lambda *a: print(*a, flush=True)  # noqa: E731

    ladder_keys = [a.key for a in LADDER]
    ladder_pairs = [(k, DEFAULT_LAW[k]) for k in ladder_keys]
    # Algorithm sweep runs on FIXED hardware (L2) so the law is the only
    # variable; L2 is the rung where every law is executable.
    algo_pairs = [("L2", a.key) for a in ALGORITHMS]
    all_pairs = ladder_pairs + [q for q in algo_pairs if q not in ladder_pairs]

    # 1) Calibrate the step input to equal lateral acceleration.
    log(f"[1/6] calibrating step input to a_y = {AY_TARGET} m/s^2 "
        f"({len(all_pairs)} cases)")
    cal_args = [(a, g, STEP_SPEED, 0.90) for a, g in all_pairs]
    cal = stage('cal', list(pool.map(job_calibrate, cal_args)))
    cal_by = {(c["arch"], c["algo"]): c for c in cal}
    for c in cal:
        log(f"      {c['arch']:>3} {c['algo']:<13} delta = {c['delta_deg']:5.2f} deg "
            f"-> a_y = {c['ay']:.3f} m/s^2")

    # Low-mu calibration is separate: 4 m/s^2 is 100% of a mu=0.4 surface, so
    # the low-mu step is taken at 2.5 m/s^2 instead.
    log(f"[2/6] calibrating low-mu step (mu = {LOW_MU}) to 2.5 m/s^2")
    cal_lo_args = [(a, g, STEP_SPEED, LOW_MU) for a, g in ladder_pairs]
    cal_lo = []
    for a, g in ladder_pairs:
        arch = BY_KEY[a]
        d, ay = calibrate_steer_for_ay(arch, _mp(a, g), STEP_SPEED, 2.5, LOW_MU)
        cal_lo.append(dict(arch=a, algo=g, delta_deg=d, ay=ay))
        log(f"      {a:>3} delta = {d:5.2f} deg -> a_y = {ay:.3f} m/s^2")
    stage("cal_lo", cal_lo)

    # 3) Step steer at equal a_y, with traces for the ladder rungs.
    log("[3/6] step steer (ISO 7401) at equal lateral acceleration")
    step_args = [(a, g, STEP_SPEED, cal_by[(a, g)]["delta_deg"], 0.90,
                  (a, g) in ladder_pairs) for a, g in all_pairs]
    step_args += [(c["arch"], c["algo"], STEP_SPEED, c["delta_deg"], LOW_MU, False)
                  for c in cal_lo]
    steps = stage('steps', list(pool.map(job_step, step_args)))

    # 4) Steady-state cornering sweep -> understeer gradient, limit a_y.
    log("[4/6] steady-state cornering sweep (ISO 4138)")
    ss_args = []
    for a, g in all_pairs:
        for v in SS_SPEEDS:
            ss_args.append((a, g, v, 3.0, 0.90))
    # Limit sweep: hold 90 km/h and wind the angle on until a_y stops growing.
    lim_args = [(a, g, 90.0, d, 0.90) for a, g in all_pairs
                for d in (2.0, 4.0, 6.0, 8.0, 10.0, 13.0, 16.0)]
    ss = stage('ss', list(pool.map(job_steady, ss_args)))
    lim = stage('lim', list(pool.map(job_steady, lim_args)))

    grads = {}
    for a, g in all_pairs:
        rows = [r for r in ss if r["arch"] == a and r["algo"] == g]
        k, _ = understeer_gradient(rows, L)
        lrows = [r for r in lim if r["arch"] == a and r["algo"] == g]
        ay_max = max((abs(r["ay"]) for r in lrows), default=float("nan"))
        beta_at_limit = max((abs(math.degrees(r["beta"])) for r in lrows),
                            default=float("nan"))
        grads[f"{a}/{g}"] = dict(arch=a, algo=g, K=k, ay_max=ay_max,
                                 beta_at_limit=beta_at_limit)
        log(f"      {a:>3} {g:<13} K = {k:6.3f} deg/g   a_y,max = {ay_max:5.2f}"
            f"   beta_limit = {beta_at_limit:5.2f} deg")

    # 5) Frequency response.
    log("[5/6] frequency response sweep")
    fr_args = [(a, g, STEP_SPEED, f, 1.5, 0.90) for a, g in all_pairs for f in FREQS]
    freq = stage('freq', list(pool.map(job_freq, fr_args)))

    # 6) Low-speed manoeuvrability.
    log("[6/6] low-speed turning circle")
    circles = stage('circles', list(pool.map(job_circle, ladder_pairs)))
    for c in circles:
        log(f"      {c['arch']:>3} R_cg = {c['R_cg']:5.2f} m   "
            f"turning circle = {c['turning_circle']:5.2f} m   "
            f"delta_r = {c['delta_r']:+.2f} deg")

    pool.shutdown()

    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "--short", "HEAD"], cwd=ROOT, text=True).strip()
    except Exception:
        commit = "unknown"

    out = dict(
        meta=dict(
            commit=commit,
            generated=time.strftime("%Y-%m-%d %H:%M:%S %Z"),
            wall_seconds=round(time.time() - t0, 1),
            dt=0.002, model="multibody", longitudinal_mode="torque",
            ay_target=AY_TARGET, step_speed_kmh=STEP_SPEED,
            low_mu=LOW_MU, mu=0.90,
            vehicle=dict(mass=p.mass, wheelbase=p.wheelbase,
                         track_front=p.track_front, track_rear=p.track_rear,
                         cg_to_front=p.cg_to_front, cg_height=p.cg_height,
                         steer_limit_deg=math.degrees(p.steer_limit)),
        ),
        ladder=[dict(key=a.key, label=a.label, label_cn=a.label_cn, dof=a.dof,
                     rear_limit_deg=a.rear_limit_deg, steer_tau=a.steer_tau,
                     steer_rate_max=a.steer_rate_max,
                     variable_ratio=a.variable_ratio, per_wheel=a.per_wheel,
                     blurb=a.blurb, law=DEFAULT_LAW[a.key]) for a in LADDER],
        algorithms=[dict(key=a.key, label=a.label, label_cn=a.label_cn,
                         blurb=a.blurb) for a in ALGORITHMS],
        calibration=cal, calibration_low_mu=cal_lo,
        step=steps, steady=ss, limit=lim, gradients=grads,
        frequency=freq, circles=circles,
    )
    dest = Path(args.out)
    dest.parent.mkdir(parents=True, exist_ok=True)
    dest.write_text(json.dumps(out, indent=1))
    log(f"\nwrote {dest}  ({dest.stat().st_size / 1024:.0f} kB, "
        f"{time.time() - t0:.0f} s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
