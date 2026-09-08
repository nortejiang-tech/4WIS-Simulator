"""Persistence for experiments (YAML definitions) and runs (result artifacts).

Layout (all under the data root, portable-build friendly):

    experiments/<name>.yaml          Experiment definition
    runs/<run_id>/meta.json          experiment snapshot + KPIs + bookkeeping
    runs/<run_id>/data.csv           t + all recorded channels

CSV keeps the platform dependency-free (numpy/pandas both read it directly);
if artifact sizes ever become a problem, swap this module's read/write pair
for parquet without touching callers.
"""

from __future__ import annotations

import csv
import json
import math
import re
import shutil
import time
import uuid
from pathlib import Path
from typing import Any

import yaml

from sim4wis.experiment.schema import Experiment
from sim4wis.experiment.session import RunResult
from sim4wis.paths import experiments_dir, runs_dir

_NAME_RE = re.compile(r"^[\w\-一-鿿]{1,64}$")


def _safe_name(name: str) -> str:
    if not _NAME_RE.match(name):
        raise ValueError(f"invalid name: {name!r}")
    return name


# ---------------------------------------------------------------------------
# Experiment definitions (YAML)
# ---------------------------------------------------------------------------


def ensure_experiments_root() -> Path:
    root = experiments_dir()
    root.mkdir(parents=True, exist_ok=True)
    return root


def list_experiments() -> list[dict[str, Any]]:
    root = ensure_experiments_root()
    items: list[dict[str, Any]] = []
    for p in sorted(root.glob("*.yaml")):
        try:
            raw = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
            items.append({
                "name": p.stem,
                "description": str(raw.get("description", "")),
                "strategy": str(raw.get("strategy", "")),
                "model_type": str(raw.get("model_type", "")),
                "maneuver": str((raw.get("maneuver") or {}).get("name", "")),
            })
        except Exception:
            items.append({"name": p.stem, "description": "(unreadable)", "strategy": "",
                          "model_type": "", "maneuver": ""})
    return items


def load_experiment(name: str) -> Experiment:
    path = ensure_experiments_root() / f"{_safe_name(name)}.yaml"
    if not path.exists():
        raise FileNotFoundError(name)
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return Experiment.model_validate(raw)


def save_experiment(exp: Experiment) -> Path:
    path = ensure_experiments_root() / f"{_safe_name(exp.name)}.yaml"
    path.write_text(
        yaml.safe_dump(exp.model_dump(mode="json"), allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return path


def delete_experiment(name: str) -> bool:
    path = ensure_experiments_root() / f"{_safe_name(name)}.yaml"
    if path.exists():
        path.unlink()
        return True
    return False


# ---------------------------------------------------------------------------
# Run artifacts
# ---------------------------------------------------------------------------


def ensure_runs_root() -> Path:
    root = runs_dir()
    root.mkdir(parents=True, exist_ok=True)
    return root


def new_run_id() -> str:
    return f"{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:6]}"


def save_run(
    result: RunResult,
    *,
    label: str = "",
    kpis: dict[str, Any] | None = None,
    job_id: str | None = None,
) -> str:
    from sim4wis import __version__
    run_id = new_run_id()
    d = ensure_runs_root() / run_id
    d.mkdir(parents=True, exist_ok=False)

    meta = {
        "run_id": run_id,
        "label": label,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "sim4wis_version": __version__,
        "duration_s": result.duration_s,
        "n_steps": result.n_steps,
        "n_samples": len(result.t),
        "channels": result.channel_names(),
        "kpis": kpis or {},
        "job_id": job_id,
        **result.meta,
    }
    (d / "meta.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=1), encoding="utf-8"
    )

    names = result.channel_names()
    with (d / "data.csv").open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", *names])
        cols = [result.channels[c] for c in names]
        for i, t in enumerate(result.t):
            w.writerow([_fmt(t), *(_fmt(col[i]) for col in cols)])
    return run_id


def _fmt(v: float) -> str:
    f = float(v)
    if not math.isfinite(f):
        return ""
    return f"{f:.17g}"


def list_runs() -> list[dict[str, Any]]:
    """Meta-lite for every stored run, newest first."""
    root = ensure_runs_root()
    out: list[dict[str, Any]] = []
    for d in sorted((p for p in root.iterdir() if p.is_dir()), reverse=True):
        meta_path = d / "meta.json"
        if not meta_path.is_file():
            continue
        try:
            m = json.loads(meta_path.read_text(encoding="utf-8"))
        except Exception:
            continue
        exp = m.get("experiment") or {}
        out.append({
            "run_id": m.get("run_id", d.name),
            "label": m.get("label", ""),
            "created_at": m.get("created_at", ""),
            "duration_s": m.get("duration_s"),
            "n_samples": m.get("n_samples"),
            "strategy": exp.get("strategy", ""),
            "model_type": exp.get("model_type", ""),
            "experiment_name": exp.get("name", ""),
            "kpis": m.get("kpis", {}),
            "job_id": m.get("job_id"),
        })
    return out


def load_run_meta(run_id: str) -> dict[str, Any]:
    path = ensure_runs_root() / _safe_name(run_id) / "meta.json"
    if not path.is_file():
        raise FileNotFoundError(run_id)
    return json.loads(path.read_text(encoding="utf-8"))


def load_run_channels(
    run_id: str,
    names: list[str] | None = None,
    decimate: int = 1,
) -> dict[str, list[float | None]]:
    """Read selected channels (plus t) from a run's CSV.

    Returns {"t": [...], "<name>": [...]}; empty CSV fields become None
    (JSON-friendly NaN representation)."""
    path = ensure_runs_root() / _safe_name(run_id) / "data.csv"
    if not path.is_file():
        raise FileNotFoundError(run_id)
    decimate = max(1, int(decimate))
    with path.open("r", encoding="utf-8", newline="") as f:
        r = csv.reader(f)
        header = next(r)
        if names is None:
            wanted = header[1:]
        else:
            wanted = [n for n in names if n in header[1:]]
        idx = {n: header.index(n) for n in wanted}
        out: dict[str, list[float | None]] = {"t": []}
        for n in wanted:
            out[n] = []
        for i, row in enumerate(r):
            if i % decimate:
                continue
            out["t"].append(float(row[0]))
            for n in wanted:
                cell = row[idx[n]]
                out[n].append(float(cell) if cell else None)
    return out


def delete_run(run_id: str) -> bool:
    d = ensure_runs_root() / _safe_name(run_id)
    if d.is_dir():
        shutil.rmtree(d)
        return True
    return False
