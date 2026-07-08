#!/usr/bin/env python3
"""Normalize exported reference data into the Sim4WIS reference.csv schema.

This script only converts raw CSV columns and units. It does not create or
modify manifest.json, does not choose metrics/tolerances, and does not make the
benchmark promotable by itself.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path


REQUIRED_CHANNELS = ("t", "vx", "vy", "yaw_rate", "pose_x", "pose_y", "driver_steering")
DEFAULT_UNITS = {
    "t": "s",
    "vx": "m/s",
    "vy": "m/s",
    "yaw_rate": "rad/s",
    "pose_x": "m",
    "pose_y": "m",
    "driver_steering": "raw",
}


@dataclass(frozen=True)
class NormalizeResult:
    output: Path
    rows: int


def parse_key_values(items: list[str], label: str) -> dict[str, str]:
    parsed: dict[str, str] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"{label} must use channel=source format: {item!r}")
        key, value = item.split("=", 1)
        key = key.strip()
        value = value.strip()
        if not key or not value:
            raise ValueError(f"{label} must not contain empty key or value: {item!r}")
        parsed[key] = value
    return parsed


def convert_value(value: float, channel: str, unit: str) -> float:
    unit = unit.strip()
    if channel == "t":
        if unit == "s":
            return value
        if unit == "ms":
            return value / 1000.0
    elif channel in {"vx", "vy"}:
        if unit == "m/s":
            return value
        if unit == "km/h":
            return value / 3.6
    elif channel == "yaw_rate":
        if unit == "rad/s":
            return value
        if unit == "deg/s":
            return value * math.pi / 180.0
    elif channel in {"pose_x", "pose_y"}:
        if unit == "m":
            return value
        if unit == "mm":
            return value / 1000.0
    elif channel == "driver_steering":
        if unit in {"raw", "normalized", "rad", "deg"}:
            return value if unit != "deg" else value * math.pi / 180.0
    raise ValueError(f"unsupported unit {unit!r} for channel {channel!r}")


def normalize_reference_csv(
    input_path: Path,
    output_path: Path,
    mappings: dict[str, str],
    units: dict[str, str] | None = None,
) -> NormalizeResult:
    units = {**DEFAULT_UNITS, **(units or {})}
    missing_mapping = [channel for channel in REQUIRED_CHANNELS if channel not in mappings]
    if missing_mapping:
        raise ValueError(f"missing required channel mapping(s): {', '.join(missing_mapping)}")
    unknown_mappings = sorted(set(mappings) - set(REQUIRED_CHANNELS))
    if unknown_mappings:
        raise ValueError(f"unknown target channel mapping(s): {', '.join(unknown_mappings)}")
    unknown_units = sorted(set(units) - set(REQUIRED_CHANNELS))
    if unknown_units:
        raise ValueError(f"unknown target unit channel(s): {', '.join(unknown_units)}")

    with input_path.open(newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        if not reader.fieldnames:
            raise ValueError(f"{input_path} has no header row")
        source_columns = set(reader.fieldnames)
        missing_sources = [source for source in mappings.values() if source not in source_columns]
        if missing_sources:
            raise ValueError(f"{input_path} missing source column(s): {', '.join(missing_sources)}")

        rows: list[dict[str, float]] = []
        last_t: float | None = None
        for row_idx, raw_row in enumerate(reader, start=2):
            out: dict[str, float] = {}
            for channel in REQUIRED_CHANNELS:
                source = mappings[channel]
                raw_value = (raw_row.get(source) or "").strip()
                if raw_value == "":
                    raise ValueError(f"{input_path}:{row_idx}: source column {source!r} is empty")
                try:
                    numeric = float(raw_value)
                except ValueError as exc:
                    raise ValueError(f"{input_path}:{row_idx}: source column {source!r} is not numeric: {raw_value!r}") from exc
                converted = convert_value(numeric, channel, units[channel])
                if not math.isfinite(converted):
                    raise ValueError(f"{input_path}:{row_idx}: channel {channel!r} converted to non-finite value")
                out[channel] = converted
            if last_t is not None and out["t"] <= last_t:
                raise ValueError(f"{input_path}:{row_idx}: t must be strictly increasing")
            last_t = out["t"]
            rows.append(out)

    if len(rows) < 2:
        raise ValueError(f"{input_path} must contain at least two samples")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=REQUIRED_CHANNELS)
        writer.writeheader()
        for row in rows:
            writer.writerow({channel: f"{row[channel]:.12g}" for channel in REQUIRED_CHANNELS})
    return NormalizeResult(output=output_path, rows=len(rows))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--map",
        action="append",
        default=[],
        help="target=source column mapping, e.g. --map yaw_rate=YawRate_deg_s",
    )
    parser.add_argument(
        "--unit",
        action="append",
        default=[],
        help="target=unit for source data before conversion, e.g. --unit yaw_rate=deg/s",
    )
    args = parser.parse_args()

    try:
        result = normalize_reference_csv(
            args.input,
            args.output,
            parse_key_values(args.map, "--map"),
            parse_key_values(args.unit, "--unit"),
        )
    except Exception as exc:
        print(f"normalize failed: {exc}", file=sys.stderr)
        return 1
    print(f"wrote {result.rows} rows: {result.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
