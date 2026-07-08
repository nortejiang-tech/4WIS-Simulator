#!/usr/bin/env python3
"""Promote a checked incoming benchmark into active validation_data/.

This script is intentionally conservative: it refuses to move an incoming
benchmark unless the existing reference checker accepts that benchmark and its
source type is independent (external_tool, bench, scaled_vehicle, or
full_vehicle). It does not regenerate review reports; run the report commands
after promotion so the diff is explicit.
"""

from __future__ import annotations

import argparse
import importlib.util
import shutil
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INCOMING_ROOT = ROOT / "validation_data" / ".incoming"
DEFAULT_VALIDATION_ROOT = ROOT / "validation_data"


def _load_reference_checker(root: Path = ROOT) -> Any:
    script = root / "scripts" / "check_reference_benchmarks.py"
    spec = importlib.util.spec_from_file_location("check_reference_benchmarks_for_promote", script)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"could not load {script}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _display(path: Path, root: Path = ROOT) -> str:
    try:
        return str(path.relative_to(root))
    except ValueError:
        return str(path)


def validate_incoming_benchmark(path: Path, root: Path = ROOT) -> tuple[bool, list[str]]:
    checker = _load_reference_checker(root)
    return checker.assess_incoming_benchmark_for_promotion(path)


def promote_reference_benchmark(
    benchmark_id: str,
    incoming_root: Path = DEFAULT_INCOMING_ROOT,
    validation_root: Path = DEFAULT_VALIDATION_ROOT,
    dry_run: bool = False,
) -> Path:
    incoming = incoming_root / benchmark_id
    dest = validation_root / benchmark_id
    if not incoming.is_dir():
        raise FileNotFoundError(f"{incoming} does not exist")
    if dest.exists():
        raise FileExistsError(f"{dest} already exists; refusing to overwrite validation evidence")

    ok, errors = validate_incoming_benchmark(incoming)
    if not ok:
        formatted = "\n  - ".join(errors)
        raise RuntimeError(f"{incoming} is not promotable:\n  - {formatted}")

    if dry_run:
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(incoming), str(dest))
    return dest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("benchmark_id")
    parser.add_argument("--incoming-root", type=Path, default=DEFAULT_INCOMING_ROOT)
    parser.add_argument("--validation-root", type=Path, default=DEFAULT_VALIDATION_ROOT)
    parser.add_argument("--dry-run", action="store_true", help="validate without moving files")
    args = parser.parse_args()

    try:
        dest = promote_reference_benchmark(
            args.benchmark_id,
            incoming_root=args.incoming_root,
            validation_root=args.validation_root,
            dry_run=args.dry_run,
        )
    except Exception as exc:
        print(f"promotion failed: {exc}", file=sys.stderr)
        return 1
    verb = "would promote" if args.dry_run else "promoted"
    print(f"{verb}: {args.benchmark_id} -> {_display(dest)}")
    print("next: regenerate docs/reports/reference_benchmark_review.md and docs/reports/v1_readiness.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
