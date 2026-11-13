#!/usr/bin/env python3
"""Repeat whole experiments, alternating engines; verify parity before speed claims."""

from __future__ import annotations
import argparse
import hashlib
import json
import platform
from pathlib import Path
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend/src"))
import numpy as np  # noqa: E402
from sim4wis import __version__  # noqa: E402
from sim4wis.native import engine_info, use_engine  # noqa: E402
from sim4wis.experiment.schema import Experiment  # noqa: E402
from sim4wis.experiment.session import run_experiment  # noqa: E402


def config(model, tire, case, duration):
    faults = [
        dict(
            id="fl",
            fault_type="stuck_hold",
            wheel=0,
            t_start=1.713,
            t_end=duration - 0.321,
        )
    ]
    if case == "simultaneous":
        faults.append(
            dict(
                id="fr",
                fault_type="stuck_hold",
                wheel=1,
                t_start=1.713,
                t_end=duration - 0.321,
            )
        )
    else:
        faults.extend(
            [
                dict(
                    id="rr",
                    fault_type="free_caster",
                    wheel=3,
                    t_start=2.123,
                    t_end=duration - 0.321,
                ),
                dict(
                    id="bias",
                    fault_type="sensor_bias",
                    wheel=1,
                    value=0.02,
                    t_start=2.321,
                ),
                dict(
                    id="noise",
                    fault_type="sensor_noise",
                    wheel=1,
                    value=0.003,
                    t_start=2.321,
                ),
                dict(
                    id="drop",
                    fault_type="sensor_dropout",
                    wheel=0,
                    t_start=2.543,
                    t_end=duration - 0.123,
                ),
            ]
        )
    return Experiment.model_validate(
        dict(
            name="native_benchmark",
            model_type=model,
            strategy="ideal_ackermann",
            dt=0.005,
            record_hz=200,
            fault_seed=20261114,
            vehicle={"overrides": {"tire_model": tire}},
            faults=faults,
            maneuver={
                "steps": [
                    {
                        "duration": duration,
                        "speed_kmh": 40,
                        "speed_ramp_s": 1,
                        "steer": {
                            "kind": "sine",
                            "amplitude": 2,
                            "unit": "front_deg",
                            "freq_hz": 0.2,
                        },
                    }
                ]
            },
        )
    )


def compare(ref, actual):
    assert actual.meta["engine"]["body_core"] == "cpp17"
    assert ref.t == actual.t and ref.n_steps == actual.n_steps
    assert ref.meta["fault_timeline"] == actual.meta["fault_timeline"]
    differences = {}
    for name, values in ref.channels.items():
        x, y = np.array(values), np.array(actual.channels[name])
        atol = (
            1e-5
            if name.startswith(
                ("fz_", "rack_force_", "motor_torque_", "torque_steer_", "grip_margin_")
            )
            else 1e-9
        )
        np.testing.assert_allclose(
            x, y, atol=atol, rtol=1e-9, equal_nan=True, err_msg=name
        )
        mask = np.isfinite(x) & np.isfinite(y)
        differences[name] = (
            float(np.max(np.abs(x[mask] - y[mask]))) if mask.any() else None
        )
    return differences


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeat", type=int, default=5)
    ap.add_argument("--duration", type=float, default=14.0)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    if args.repeat < 3 or args.duration < 3:
        ap.error("at least 3 repeats and 3 simulation seconds required")
    with use_engine("native"):
        info = engine_info()
    try:
        commit = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=ROOT, text=True
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        commit = None
    result = {
        "version": __version__,
        "date": "2026-11-14",
        "source_commit": commit,
        "python": sys.version,
        "platform": platform.platform(),
        "machine": platform.machine(),
        "cpu": platform.processor(),
        "numpy": np.__version__,
        "engine": info,
        "method": "one warmup per engine; alternating order; whole run_experiment including recording; no concurrent benchmark workers",
        "repeat": args.repeat,
        "duration_s": args.duration,
        "speedup_target": 1.5,
        "parity_tolerance": {
            "rtol": 1e-9,
            "atol_state_SI": 1e-9,
            "atol_force_margin_N": 1e-5,
        },
        "cases": [],
    }
    for model in ("simplified_dynamic", "multibody"):
        for tire in ("linear", "pacejka"):
            for case in ("simultaneous", "staggered"):
                exp = config(model, tire, case, args.duration)
                runs = {}
                times = {"python": [], "native": []}
                for engine in times:
                    with use_engine(engine):
                        runs[engine] = run_experiment(exp)
                differences = compare(runs["python"], runs["native"])
                for repeat in range(args.repeat):
                    for engine in (
                        ("python", "native")
                        if repeat % 2 == 0
                        else ("native", "python")
                    ):
                        with use_engine(engine):
                            start = time.perf_counter()
                            run_experiment(exp)
                            elapsed = time.perf_counter() - start
                        times[engine].append(elapsed)
                medians = {e: statistics.median(v) for e, v in times.items()}
                speedup = medians["python"] / medians["native"]
                row = {
                    "model": model,
                    "tire": tire,
                    "case": case,
                    "experiment": exp.model_dump(mode="json"),
                    "experiment_sha256": hashlib.sha256(
                        exp.model_dump_json().encode()
                    ).hexdigest(),
                    "wall_seconds": times,
                    "median_seconds": medians,
                    "speedup": speedup,
                    "parity": "PASS",
                    "max_abs_channel_differences": differences,
                    "speed_target": "PASS" if speedup >= 1.5 else "FAIL",
                    "event_timeline": runs["native"].meta["fault_timeline"],
                }
                result["cases"].append(row)
                print(
                    model,
                    tire,
                    case,
                    "speedup",
                    round(speedup, 3),
                    "parity PASS",
                    flush=True,
                )
    result["status"] = (
        "PASS" if all(r["speed_target"] == "PASS" for r in result["cases"]) else "FAIL"
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
