#!/usr/bin/env python3
"""Suggest manifest.source_artifacts entries for incoming/reference benchmarks.

This helper computes checksums for source files that live inside a benchmark
directory and prints a copyable JSON snippet for manifest.source_artifacts.

It intentionally does not modify any files or promote assumptions. Paths are
validated to avoid using absolute paths, escaping the benchmark root, or using
generated benchmark files as source evidence.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROLE = "TODO: describe raw source evidence role"


@dataclass(frozen=True)
class SuggestedArtifacts:
    source_artifacts: tuple[dict[str, str], ...]


def _load_reference_checker(root: Path = ROOT) -> Any:
    script = root / "scripts" / "check_reference_benchmarks.py"
    spec = importlib.util.spec_from_file_location("check_reference_benchmarks_for_artifact_suggestions", script)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {script}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _sanitize_root_path(benchmark_dir: Path) -> Path:
    if not benchmark_dir.is_dir():
        raise ValueError(f"benchmark_dir must be an existing directory: {benchmark_dir}")
    return benchmark_dir


def suggest_reference_artifacts(
    benchmark_dir: Path,
    source_paths: tuple[str, ...],
    role: str = DEFAULT_ROLE,
) -> SuggestedArtifacts:
    checker = _load_reference_checker()
    benchmark_dir = _sanitize_root_path(benchmark_dir)
    if not source_paths:
        raise ValueError("at least one source file path is required")
    if not isinstance(role, str) or not role.strip():
        raise ValueError("role must be a non-empty string")

    role = role.strip()
    artifacts: list[dict[str, str]] = []
    for raw_path in source_paths:
        rel_path = str(raw_path).strip()
        if not rel_path:
            raise ValueError("source file paths must be non-empty")

        try:
            source_path = checker._resolve_artifact_path(benchmark_dir, rel_path)
        except Exception as exc:
            raise ValueError(f"{raw_path!r} is not a valid in-directory relative file path: {exc}") from exc

        normalized_path = Path(rel_path).as_posix()
        if normalized_path in checker.GENERATED_BENCHMARK_FILES:
            raise ValueError(
                f"{raw_path!r} is not valid as source artifact; "
                f"generated benchmark files cannot be raw evidence: {normalized_path!r}"
            )
        if not source_path.is_file():
            raise ValueError(f"{raw_path!r} must point to an existing file inside {benchmark_dir.name}")

        sha256 = checker._sha256_file(source_path)
        artifacts.append({"path": normalized_path, "role": role, "sha256": sha256})

    return SuggestedArtifacts(source_artifacts=tuple(artifacts))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "benchmark_dir",
        type=Path,
        help="benchmark directory containing reference.csv and sim4wis_experiment.yaml",
    )
    parser.add_argument(
        "source_paths",
        nargs="+",
        metavar="source_path",
        help="relative paths to raw/source files used for provenance",
    )
    parser.add_argument("--role", default=DEFAULT_ROLE, help="role for each source artifact entry")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        result = suggest_reference_artifacts(
            args.benchmark_dir,
            tuple(args.source_paths),
            role=args.role,
        )
    except Exception as exc:
        print(f"reference artifact suggestion failed: {exc}", file=sys.stderr)
        return 1
    payload = {"source_artifacts": result.source_artifacts}
    print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
