"""Project YAML I/O.

Layout on disk:
    <repo>/projects/<name>.yaml

Save writes atomically (via a temp file + rename) so a crash mid-write
doesn't corrupt an existing project.
"""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import yaml

from sim4wis.paths import projects_dir
from sim4wis.project.schema import ProjectFile


def projects_root() -> Path:
    """Return the directory where project YAML files live (env override allowed)."""
    return projects_dir()


def ensure_root() -> Path:
    root = projects_root()
    root.mkdir(parents=True, exist_ok=True)
    return root


def list_projects() -> list[str]:
    root = ensure_root()
    return sorted(p.stem for p in root.glob("*.yaml") if p.is_file())


def load_project(name: str) -> ProjectFile:
    path = ensure_root() / f"{name}.yaml"
    if not path.exists():
        raise FileNotFoundError(name)
    with path.open("r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    return ProjectFile.model_validate(raw)


def save_project(name: str, proj: ProjectFile) -> Path:
    root = ensure_root()
    path = root / f"{name}.yaml"
    data = proj.model_dump(mode="json")
    # Atomic write: dump to a temp in the same dir, then os.replace
    fd, tmp = tempfile.mkstemp(prefix=f".{name}.", suffix=".yaml.tmp", dir=str(root))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        os.replace(tmp, path)
    except Exception:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return path


def delete_project(name: str) -> bool:
    path = ensure_root() / f"{name}.yaml"
    if not path.exists():
        return False
    path.unlink()
    return True
