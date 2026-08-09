"""Resource / data directory resolution.

Works both in the dev repo and in the packaged portable build. The packaged
launcher sets `SIM4WIS_DATA_DIR` to the package root; everything else is
resolved relative to it so the editable folders (projects / scripts_lib /
plugins) and the built frontend live *outside* any frozen bundle and can be
edited without rebuilding.

Resolution order for the data root:
    1. $SIM4WIS_DATA_DIR              (set by the portable launcher)
    2. the dev repo root              (…/4WIS Simulator)

Individual folders also honour their own env overrides where they already
existed (e.g. $SIM4WIS_PROJECTS_DIR).
"""

from __future__ import annotations

import os
from pathlib import Path


def repo_root() -> Path:
    # backend/src/sim4wis/paths.py → parents[3] = repo root
    return Path(__file__).resolve().parents[3]


def data_root() -> Path:
    env = os.environ.get("SIM4WIS_DATA_DIR")
    return Path(env) if env else repo_root()


def projects_dir() -> Path:
    env = os.environ.get("SIM4WIS_PROJECTS_DIR")
    return Path(env) if env else data_root() / "projects"


def vehicle_profiles_dir() -> Path:
    env = os.environ.get("SIM4WIS_VEHICLE_PROFILES_DIR")
    return Path(env) if env else data_root() / "vehicle_profiles"


def kc_profiles_dir() -> Path:
    """Measured suspension K&C characteristics (see vehicle/kc.py).

    Kept beside `vehicle_profiles` rather than inside them: one vehicle can be
    run against several suspension measurements (a rig sweep before and after a
    change, a supplier's data vs. an estimate), and one measurement can serve
    several vehicle variants.
    """
    env = os.environ.get("SIM4WIS_KC_PROFILES_DIR")
    return Path(env) if env else data_root() / "kc_profiles"


def scripts_lib_dir() -> Path:
    return data_root() / "scripts_lib"


def experiments_dir() -> Path:
    """Experiment definitions (*.yaml) — the batch-run unit of work."""
    env = os.environ.get("SIM4WIS_EXPERIMENTS_DIR")
    return Path(env) if env else data_root() / "experiments"


def runs_dir() -> Path:
    """Run artifacts (runs/<id>/meta.json + data.csv)."""
    env = os.environ.get("SIM4WIS_RUNS_DIR")
    return Path(env) if env else data_root() / "runs"


def plugins_dir() -> Path:
    """Directory containing strategy plugins (*.fmu, *.slx, …)."""
    return data_root() / "plugins" / "strategies"


def dist_dir() -> Path | None:
    """Locate the built frontend (`dist/`), or None if not present.

    Packaged layout puts it at <data_root>/app/dist; the dev tree builds it at
    <repo>/frontend/dist.
    """
    for cand in (data_root() / "app" / "dist", repo_root() / "frontend" / "dist"):
        if (cand / "index.html").is_file():
            return cand
    return None
