"""Study artifacts — what survives the session.

`studies/<id>/` holds the spec that was run, the result table, and (when asked
for) the report. The point is that a conclusion can be traced back to the exact
declaration that produced it, months later, by someone who was not there.

Provenance is collected here rather than at the call site because it must be
non-optional. A study result without a git sha and a parameter hash is a claim
nobody can re-check, and "we forgot to record it" is not recoverable after the
fact.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
import time
import uuid
from pathlib import Path
from typing import Any

from sim4wis.paths import studies_dir
from sim4wis.study.result import StudyResult
from sim4wis.study.spec import StudySpec


def ensure_studies_root() -> Path:
    root = studies_dir()
    root.mkdir(parents=True, exist_ok=True)
    return root


def new_study_id(name: str) -> str:
    safe = "".join(c for c in name if c.isalnum() or c in "-_")[:40] or "study"
    return f"{time.strftime('%Y%m%d_%H%M%S')}_{safe}_{uuid.uuid4().hex[:4]}"


def _git_sha() -> str:
    """Short sha, suffixed `-dirty` when the tree has uncommitted changes.

    A result produced from a dirty tree is not reproducible from the sha alone,
    and saying so is the whole value of recording it.
    """
    try:
        root = Path(__file__).resolve().parents[4]
        sha = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=root, text=True, capture_output=True, timeout=5,
        )
        if sha.returncode != 0:
            return "unknown"
        out = sha.stdout.strip()
        dirty = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=root, text=True, capture_output=True, timeout=5,
        )
        if dirty.returncode == 0 and dirty.stdout.strip():
            out += "-dirty"
        return out
    except Exception:                                # noqa: BLE001 - provenance is best-effort
        return "unknown"


def params_hash(payload: dict[str, Any]) -> str:
    """Stable hash of a fully resolved parameter set.

    Keyed on the *resolved* params — profile plus overrides, already merged —
    because that is what the physics saw. Hashing the spec's `vehicle` block
    instead would make two studies look identical while one of them silently
    loaded a different profile from disk.
    """
    blob = json.dumps(payload, sort_keys=True, separators=(",", ":"), default=str)
    return hashlib.sha256(blob.encode("utf-8")).hexdigest()[:16]


def provenance(spec: StudySpec, resolved_params: dict[str, Any] | None = None) -> dict[str, Any]:
    from sim4wis import __version__

    return {
        "sim4wis_version": __version__,
        "git_sha": _git_sha(),
        "spec_digest": spec.digest(),
        "model": spec.model,
        "strategy": spec.baseline.strategy,
        "dt": spec.baseline.dt,
        "record_hz": spec.baseline.record_hz,
        "params_hash": params_hash(resolved_params) if resolved_params else None,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
    }


def save(study_id: str, spec: StudySpec, result: StudyResult) -> Path:
    d = ensure_studies_root() / study_id
    d.mkdir(parents=True, exist_ok=True)
    (d / "spec.json").write_text(
        json.dumps(spec.model_dump(mode="json"), ensure_ascii=False, indent=1), encoding="utf-8"
    )
    (d / "result.json").write_text(
        json.dumps(result.to_summary(), ensure_ascii=False, indent=1), encoding="utf-8"
    )
    return d


def load(study_id: str) -> dict[str, Any]:
    d = ensure_studies_root() / study_id
    result = d / "result.json"
    if not result.is_file():
        raise FileNotFoundError(study_id)
    return json.loads(result.read_text(encoding="utf-8"))


def load_spec(study_id: str) -> StudySpec:
    d = ensure_studies_root() / study_id
    path = d / "spec.json"
    if not path.is_file():
        raise FileNotFoundError(study_id)
    return StudySpec.model_validate(json.loads(path.read_text(encoding="utf-8")))


def list_studies() -> list[dict[str, Any]]:
    root = ensure_studies_root()
    out: list[dict[str, Any]] = []
    for d in sorted((p for p in root.iterdir() if p.is_dir()), reverse=True):
        path = d / "result.json"
        if not path.is_file():
            continue
        try:
            r = json.loads(path.read_text(encoding="utf-8"))
        except Exception:                            # noqa: BLE001
            continue
        out.append({
            "study_id": d.name,
            "study": r.get("study", ""),
            "question": r.get("question", ""),
            "model": r.get("model", ""),
            "spec_digest": r.get("spec_digest", ""),
            "rows": len(r.get("rows", [])),
            "all_passed": r.get("all_passed"),
            "created_at": (r.get("provenance") or {}).get("created_at", ""),
            "report_path": r.get("report_path"),
        })
    return out
