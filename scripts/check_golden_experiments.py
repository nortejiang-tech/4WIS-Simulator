#!/usr/bin/env python3
"""Check reproducible experiments against KPI golden baselines.

This is a lightweight numerical regression gate. It runs selected experiment
YAMLs in memory, computes backend KPIs, and compares a small set of stable
metrics against docs/golden_experiments.json. It also runs a small set of
single-wheel-failure samples from the research script so safety-critical
failure behavior has a fast regression signal without regenerating the full
report, plus the v2 ISO 13674 on-centre weave procedure so the steering-plant
process metrics (torque gradient, hysteresis, deadband, phase lag, …) have the
same protection before any multi-rate integration work touches the plant.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import tempfile
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend" / "src"))

from sim4wis import __version__ as SIM_VERSION  # noqa: E402
from sim4wis.experiment import store as run_store  # noqa: E402
from sim4wis.experiment.kpi import compute_kpis  # noqa: E402
from sim4wis.experiment.schema import Experiment  # noqa: E402
from sim4wis.experiment.session import run_experiment  # noqa: E402
from sim4wis.study.runner import run_sync  # noqa: E402
from sim4wis.study.spec import StudySpec  # noqa: E402

import study_single_wheel_failure as swf  # noqa: E402


BASELINE = ROOT / "docs" / "golden_experiments.json"

EXPERIMENT_GOLDENS: dict[str, dict[str, Any]] = {
    "step_steer_60kmh": {
        "path": "experiments/step_steer_60kmh.yaml",
        "metrics": {
            "yaw_rate_peak_dps": {"abs_tol": 0.05, "rel_tol": 0.01},
            "speed_error_rms_kmh": {"abs_tol": 0.05, "rel_tol": 0.02},
            "yaw_gain_dps": {"abs_tol": 0.05, "rel_tol": 0.02},
            "yaw_rise_time_s": {"abs_tol": 0.02, "rel_tol": 0.05},
            "yaw_overshoot_pct": {"abs_tol": 0.2, "rel_tol": 0.05},
            "slip_alpha_peak_deg": {"abs_tol": 0.05, "rel_tol": 0.02},
        },
    },
    "iso3888_dlc_60kmh": {
        "path": "experiments/iso3888_dlc_60kmh.yaml",
        "metrics": {
            "yaw_rate_peak_dps": {"abs_tol": 0.05, "rel_tol": 0.01},
            "vy_peak_kmh": {"abs_tol": 0.05, "rel_tol": 0.02},
            "speed_error_rms_kmh": {"abs_tol": 0.05, "rel_tol": 0.02},
            "steer_energy_nms": {"abs_tol": 10.0, "rel_tol": 0.02},
            "rack_force_peak_n": {"abs_tol": 5.0, "rel_tol": 0.02},
            "slip_alpha_peak_deg": {"abs_tol": 0.05, "rel_tol": 0.02},
        },
    },
}

SINGLE_WHEEL_GOLDENS: dict[str, dict[str, Any]] = {
    "sw_straight100_rl_stuck_value_baseline": {
        "scenario": "straight100",
        "wheel": 2,
        "fault": "stuck_value",
        "mitigation": "baseline",
        "expected_class": "C3",
        "metrics": {
            "xtrack_react": {"abs_tol": 0.03, "rel_tol": 0.02},
            "xtrack_2_5s": {"abs_tol": 0.03, "rel_tol": 0.02},
            "ttld_s": {"abs_tol": 0.03, "rel_tol": 0.02},
            "dyaw_peak_dps": {"abs_tol": 0.2, "rel_tol": 0.02},
            "dpsi_2s_deg": {"abs_tol": 0.2, "rel_tol": 0.02},
        },
    },
    "sw_straight100_rl_stuck_value_mitigated": {
        "scenario": "straight100",
        "wheel": 2,
        "fault": "stuck_value",
        "mitigation": "mitigated",
        "expected_class": "C2",
        "metrics": {
            "xtrack_react": {"abs_tol": 0.03, "rel_tol": 0.02},
            "xtrack_2_5s": {"abs_tol": 0.03, "rel_tol": 0.02},
            "ttld_s": {"abs_tol": 0.03, "rel_tol": 0.02},
            "dyaw_peak_dps": {"abs_tol": 0.2, "rel_tol": 0.02},
            "dpsi_2s_deg": {"abs_tol": 0.2, "rel_tol": 0.02},
        },
    },
    "sw_curve60_fl_free_caster_baseline": {
        "scenario": "curve60",
        "wheel": 0,
        "fault": "free_caster",
        "mitigation": "baseline",
        "expected_class": "C2",
        "metrics": {
            "xtrack_react": {"abs_tol": 0.03, "rel_tol": 0.02},
            "xtrack_2_5s": {"abs_tol": 0.03, "rel_tol": 0.02},
            "dyaw_peak_dps": {"abs_tol": 0.2, "rel_tol": 0.02},
            "dyaw_resid_dps": {"abs_tol": 0.05, "rel_tol": 0.05},
            "beta_peak_deg": {"abs_tol": 0.05, "rel_tol": 0.02},
        },
    },
}


# The v2 steering plant's process metrics, pinned through the objective-test
# procedure itself: the study runner executes `procedures/iso13674_oncentre.yaml`
# (100 km/h, 0.2 Hz, front 0.4°, plant enabled, 50 Hz recording) and the
# on-centre analyser reduces the run to these ten. This is the anchor that must
# pass before — and after — any multi-rate integration of the plant: tamper
# with plant.py or oncentre.py and the check goes red.
ONCENTRE_GOLDENS: dict[str, dict[str, Any]] = {
    "oncentre_weave_100kmh": {
        "path": "procedures/iso13674_oncentre.yaml",
        "metrics": {
            "onc_torque_gradient_nm_per_g": {"abs_tol": 0.02, "rel_tol": 0.005},
            "onc_torque_at_0_1g_nm": {"abs_tol": 0.005, "rel_tol": 0.005},
            "onc_torque_hysteresis_nm": {"abs_tol": 0.005, "rel_tol": 0.005},
            "onc_torque_deadband_deg": {"abs_tol": 0.005, "rel_tol": 0.005},
            "onc_angle_gradient_deg_per_g": {"abs_tol": 0.02, "rel_tol": 0.005},
            "onc_yaw_phase_lag_deg": {"abs_tol": 0.02, "rel_tol": 0.005},
            "onc_torque_gradient_nm_per_deg": {"abs_tol": 0.002, "rel_tol": 0.002},
            "onc_ay_amplitude_g": {"abs_tol": 0.001, "rel_tol": 0.005},
            "onc_sw_amplitude_deg": {"abs_tol": 0.005, "rel_tol": 0.005},
            "onc_frequency_hz": {"abs_tol": 0.0005, "rel_tol": 0.005},
        },
    },
}


def load_experiment(path: Path) -> Experiment:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return Experiment.model_validate(raw)


def run_experiment_goldens() -> dict[str, Any]:
    out: dict[str, Any] = {}
    for name, spec in EXPERIMENT_GOLDENS.items():
        exp = load_experiment(ROOT / spec["path"])
        result = run_experiment(exp)
        kpis = compute_kpis(result, exp)
        out[name] = {
            "experiment": spec["path"],
            "n_samples": len(result.t),
            "duration_s": result.duration_s,
            "kpis": {metric: float(kpis[metric]) for metric in spec["metrics"]},
        }
    return out


def _run_single_wheel_case(scenario: str, wheel: int, fault_type: str, mitigation: str) -> dict[str, Any]:
    _steps, t_fault, _v = swf.scenario_steps(scenario)
    ref = run_experiment(swf.build_exp(scenario, None))
    fault = swf.make_fault(wheel, fault_type, t_fault)
    baseline = run_experiment(swf.build_exp(scenario, fault))

    if mitigation == "baseline":
        run = baseline
    elif mitigation == "mitigated":
        kind = "free" if fault_type == "free_caster" else "stuck"
        if fault_type == "stuck_hold":
            angle = swf.stuck_angle_from(baseline, wheel, t_fault)
        elif fault_type == "stuck_value":
            angle = math.radians(swf.STUCK_DEG)
        else:
            angle = 0.0
        run = run_experiment(
            swf.build_exp(
                scenario,
                fault,
                "fault_reconfig",
                swf.mitigation_params(wheel, kind, angle, t_fault),
            )
        )
    else:
        raise ValueError(f"unknown mitigation: {mitigation}")

    metrics = swf.compute_metrics(run, ref, t_fault)
    return {
        "source": "scripts/study_single_wheel_failure.py",
        "scenario": scenario,
        "wheel": wheel,
        "fault": fault_type,
        "mitigation": mitigation,
        "t_fault": t_fault,
        "n_samples": len(run.t),
        "duration_s": run.duration_s,
        "c_class": swf.c_class(metrics),
        "kpis": {
            metric: float(metrics[metric])
            for metric in SINGLE_WHEEL_GOLDENS[
                f"sw_{scenario}_{['fl', 'fr', 'rl', 'rr'][wheel]}_{fault_type}_{mitigation}"
            ]["metrics"]
        },
    }


def run_single_wheel_goldens() -> dict[str, Any]:
    out: dict[str, Any] = {}
    for name, spec in SINGLE_WHEEL_GOLDENS.items():
        out[name] = _run_single_wheel_case(
            spec["scenario"],
            int(spec["wheel"]),
            spec["fault"],
            spec["mitigation"],
        )
    return out


def _run_oncentre_case(spec_cfg: dict[str, Any]) -> dict[str, Any]:
    """One weave run through the study runner, reduced to the ten process metrics.

    Goes through the study runner rather than reimplementing anything: the
    procedure file pins the condition (plant enabled, 100 km/h, 0.2 Hz, 50 Hz
    recording) and the on-centre analyser produces the numbers. Run artifacts
    are diverted to a throwaway directory so the gate itself leaves no state.
    """
    raw = yaml.safe_load((ROOT / spec_cfg["path"]).read_text(encoding="utf-8")) or {}
    spec = StudySpec.model_validate(raw)

    old_runs = os.environ.get("SIM4WIS_RUNS_DIR")
    old_studies = os.environ.get("SIM4WIS_STUDIES_DIR")
    os.environ["SIM4WIS_RUNS_DIR"] = tempfile.mkdtemp(prefix="golden_onc_runs_")

    os.environ["SIM4WIS_STUDIES_DIR"] = tempfile.mkdtemp(prefix="golden_onc_studies_")
    try:
        _, result = run_sync(spec, write_report=False)
        row = result.rows[0]
        # Sample count is read while the diverted runs dir is still in effect.
        n_samples = 0
        if row.run_id:
            try:
                raw_ch = run_store.load_run_channels(row.run_id, ["t"])
                n_samples = len(raw_ch.get("t", []))
            except Exception:  # noqa: BLE001 - sample count is informational
                n_samples = 0
    finally:
        if old_runs is None:
            os.environ.pop("SIM4WIS_RUNS_DIR", None)
        else:
            os.environ["SIM4WIS_RUNS_DIR"] = old_runs
        if old_studies is None:
            os.environ.pop("SIM4WIS_STUDIES_DIR", None)
        else:
            os.environ["SIM4WIS_STUDIES_DIR"] = old_studies
    return {
        "experiment": spec_cfg["path"],
        "n_samples": n_samples,
        "duration_s": round(result.elapsed_s, 3),
        "kpis": {
            metric: float(row.metrics[metric])
            for metric in spec_cfg["metrics"]
        },
    }


def run_oncentre_goldens() -> dict[str, Any]:
    out: dict[str, Any] = {}
    for name, spec in ONCENTRE_GOLDENS.items():
        out[name] = _run_oncentre_case(spec)
    return out


def run_goldens() -> dict[str, Any]:
    return {
        **run_experiment_goldens(),
        **run_single_wheel_goldens(),
        **run_oncentre_goldens(),
    }


def write_baseline(actual: dict[str, Any]) -> None:
    data = {
        "schema": 1,
        "sim4wis_version": SIM_VERSION,
        "note": "Generated by scripts/check_golden_experiments.py --update.",
        "experiments": actual,
        "tolerances": {
            name: {metric: tol for metric, tol in spec["metrics"].items()}
            for name, spec in {
                **EXPERIMENT_GOLDENS, **SINGLE_WHEEL_GOLDENS, **ONCENTRE_GOLDENS,
            }.items()
        },
        "classifications": {
            name: spec["expected_class"]
            for name, spec in SINGLE_WHEEL_GOLDENS.items()
        },
    }
    BASELINE.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def load_baseline() -> dict[str, Any]:
    if not BASELINE.is_file():
        raise FileNotFoundError(f"{BASELINE} missing; run with --update first")
    return json.loads(BASELINE.read_text(encoding="utf-8"))


def compare(actual: dict[str, Any], baseline: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    expected = baseline.get("experiments", {})
    tolerances = baseline.get("tolerances", {})
    classifications = baseline.get("classifications", {})
    for name, spec in {
        **EXPERIMENT_GOLDENS, **SINGLE_WHEEL_GOLDENS, **ONCENTRE_GOLDENS,
    }.items():
        if name not in expected:
            failures.append(f"{name}: missing baseline experiment")
            continue
        if actual[name]["n_samples"] != expected[name]["n_samples"]:
            failures.append(
                f"{name}: n_samples {actual[name]['n_samples']} != {expected[name]['n_samples']}"
            )
        for metric in spec["metrics"]:
            got = actual[name]["kpis"].get(metric)
            want = expected[name]["kpis"].get(metric)
            tol = (tolerances.get(name, {}) or {}).get(metric, spec["metrics"][metric])
            if got is None or want is None:
                failures.append(f"{name}.{metric}: missing value")
                continue
            allowed = max(float(tol["abs_tol"]), abs(float(want)) * float(tol["rel_tol"]))
            delta = abs(float(got) - float(want))
            if not math.isfinite(delta) or delta > allowed:
                failures.append(
                    f"{name}.{metric}: got {got:.6g}, expected {want:.6g}, "
                    f"delta {delta:.3g} > tol {allowed:.3g}"
                )
        if name in SINGLE_WHEEL_GOLDENS:
            got_class = actual[name].get("c_class")
            want_class = classifications.get(name, spec["expected_class"])
            if got_class != want_class:
                failures.append(f"{name}.c_class: got {got_class}, expected {want_class}")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--update", action="store_true", help="rewrite the golden baseline")
    args = parser.parse_args()

    actual = run_goldens()
    if args.update:
        write_baseline(actual)
        print(f"updated {BASELINE}")
        return 0

    baseline = load_baseline()
    failures = compare(actual, baseline)
    if failures:
        print("Golden experiment regression failures:", file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        print("\nIf these changes are intentional, rerun with --update and review the diff.", file=sys.stderr)
        return 1

    for name, data in actual.items():
        print(f"{name}: ok ({data['n_samples']} samples)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
