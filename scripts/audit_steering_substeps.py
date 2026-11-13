#!/usr/bin/env python3
"""On-centre sensitivity after outer-ramp-preserving vehicle substeps."""
import json
import argparse
import os
from pathlib import Path
import sys
import tempfile

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "backend/src"))
from sim4wis.study.spec import StudySpec
from sim4wis.study.runner import run_sync


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controls-only", action="store_true")
    args = parser.parse_args()
    out = {}
    with tempfile.TemporaryDirectory(prefix="steering_audit_") as directory:
        os.environ["SIM4WIS_RUNS_DIR"] = directory + "/runs"
        os.environ["SIM4WIS_STUDIES_DIR"] = directory + "/studies"
        names = (["step_steer_golden", "iso3888_dlc_golden"] if args.controls_only
                 else ["iso13674_oncentre"])
        for name in names:
            samples = {}
            for dt in [.005, .0025, .00125, .000625]:
                raw = yaml.safe_load((ROOT / f"procedures/{name}.yaml").read_text())
                raw["baseline"]["dt"] = dt
                _, result = run_sync(StudySpec.model_validate(raw), write_report=False)
                samples[str(dt)] = result.rows[0].metrics
                print(json.dumps({"case": name, "dt": dt, "metrics": samples[str(dt)]}), flush=True)
            out[name] = samples
    if not args.controls_only:
        out = out["iso13674_oncentre"]
    file = "control_convergence.json" if args.controls_only else "steering_convergence.json"
    path = ROOT / "docs/reports/multipoint_v0.105.0" / file
    path.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n")


if __name__ == "__main__":
    main()
