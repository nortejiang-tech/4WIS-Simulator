from __future__ import annotations

import csv
import importlib.util
import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "normalize_reference_csv.py"


def load_module():
    spec = importlib.util.spec_from_file_location("normalize_reference_csv", SCRIPT)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


def write_raw_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def read_reference_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def test_normalize_reference_csv_converts_units(tmp_path: Path) -> None:
    normalizer = load_module()
    raw = tmp_path / "raw.csv"
    out = tmp_path / "validation_data" / ".incoming" / "carmaker_case" / "reference.csv"
    write_raw_csv(
        raw,
        [
            {
                "Time_ms": "0",
                "Vx_kmh": "36",
                "Vy_kmh": "3.6",
                "YawRate_deg_s": "90",
                "X_mm": "1000",
                "Y_mm": "2000",
                "Steer_deg": "10",
            },
            {
                "Time_ms": "500",
                "Vx_kmh": "72",
                "Vy_kmh": "7.2",
                "YawRate_deg_s": "180",
                "X_mm": "3000",
                "Y_mm": "4000",
                "Steer_deg": "20",
            },
        ],
    )

    result = normalizer.normalize_reference_csv(
        raw,
        out,
        {
            "t": "Time_ms",
            "vx": "Vx_kmh",
            "vy": "Vy_kmh",
            "yaw_rate": "YawRate_deg_s",
            "pose_x": "X_mm",
            "pose_y": "Y_mm",
            "driver_steering": "Steer_deg",
        },
        {
            "t": "ms",
            "vx": "km/h",
            "vy": "km/h",
            "yaw_rate": "deg/s",
            "pose_x": "mm",
            "pose_y": "mm",
            "driver_steering": "deg",
        },
    )

    assert result.output == out
    assert result.rows == 2
    rows = read_reference_csv(out)
    assert list(rows[0]) == ["t", "vx", "vy", "yaw_rate", "pose_x", "pose_y", "driver_steering"]
    assert float(rows[0]["t"]) == 0.0
    assert float(rows[1]["t"]) == 0.5
    assert float(rows[0]["vx"]) == 10.0
    assert float(rows[0]["vy"]) == 1.0
    assert math.isclose(float(rows[0]["yaw_rate"]), math.pi / 2.0)
    assert float(rows[0]["pose_x"]) == 1.0
    assert float(rows[0]["pose_y"]) == 2.0
    assert math.isclose(float(rows[0]["driver_steering"]), math.pi / 18.0)


def test_normalize_reference_csv_crops_and_zeroes_time(tmp_path: Path) -> None:
    normalizer = load_module()
    raw = tmp_path / "raw.csv"
    out = tmp_path / "reference.csv"
    write_raw_csv(
        raw,
        [
            {"t": "0.0", "vx": "1", "vy": "0", "yaw_rate": "0", "x": "0", "y": "0", "steer": "0"},
            {"t": "0.5", "vx": "2", "vy": "0", "yaw_rate": "0", "x": "1", "y": "0", "steer": "0.1"},
            {"t": "1.0", "vx": "3", "vy": "0", "yaw_rate": "0", "x": "2", "y": "0", "steer": "0.2"},
            {"t": "1.5", "vx": "4", "vy": "0", "yaw_rate": "0", "x": "3", "y": "0", "steer": "0.3"},
        ],
    )

    result = normalizer.normalize_reference_csv(
        raw,
        out,
        {
            "t": "t",
            "vx": "vx",
            "vy": "vy",
            "yaw_rate": "yaw_rate",
            "pose_x": "x",
            "pose_y": "y",
            "driver_steering": "steer",
        },
        crop_start_s=0.5,
        crop_end_s=1.0,
        zero_time=True,
    )

    assert result.rows == 2
    rows = read_reference_csv(out)
    assert [float(row["t"]) for row in rows] == [0.0, 0.5]
    assert [float(row["vx"]) for row in rows] == [2.0, 3.0]
    assert [float(row["pose_x"]) for row in rows] == [1.0, 2.0]


def test_normalize_reference_csv_refuses_invalid_crop_window(tmp_path: Path) -> None:
    normalizer = load_module()
    raw = tmp_path / "raw.csv"
    write_raw_csv(
        raw,
        [
            {"t": "0", "vx": "1", "vy": "0", "yaw_rate": "0", "x": "0", "y": "0", "steer": "0"},
            {"t": "1", "vx": "1", "vy": "0", "yaw_rate": "0", "x": "1", "y": "0", "steer": "0"},
        ],
    )

    try:
        normalizer.normalize_reference_csv(
            raw,
            tmp_path / "reference.csv",
            {
                "t": "t",
                "vx": "vx",
                "vy": "vy",
                "yaw_rate": "yaw_rate",
                "pose_x": "x",
                "pose_y": "y",
                "driver_steering": "steer",
            },
            crop_start_s=1.0,
            crop_end_s=1.0,
        )
    except ValueError as exc:
        assert "crop_end_s must be greater than crop_start_s" in str(exc)
    else:
        raise AssertionError("expected invalid crop window to fail")


def test_normalize_reference_csv_refuses_too_narrow_crop_window(tmp_path: Path) -> None:
    normalizer = load_module()
    raw = tmp_path / "raw.csv"
    write_raw_csv(
        raw,
        [
            {"t": "0", "vx": "1", "vy": "0", "yaw_rate": "0", "x": "0", "y": "0", "steer": "0"},
            {"t": "1", "vx": "1", "vy": "0", "yaw_rate": "0", "x": "1", "y": "0", "steer": "0"},
            {"t": "2", "vx": "1", "vy": "0", "yaw_rate": "0", "x": "2", "y": "0", "steer": "0"},
        ],
    )

    try:
        normalizer.normalize_reference_csv(
            raw,
            tmp_path / "reference.csv",
            {
                "t": "t",
                "vx": "vx",
                "vy": "vy",
                "yaw_rate": "yaw_rate",
                "pose_x": "x",
                "pose_y": "y",
                "driver_steering": "steer",
            },
            crop_start_s=0.75,
            crop_end_s=1.25,
        )
    except ValueError as exc:
        assert "crop window must retain at least two samples" in str(exc)
    else:
        raise AssertionError("expected too-narrow crop window to fail")


def test_normalize_reference_csv_refuses_missing_mapping(tmp_path: Path) -> None:
    normalizer = load_module()
    raw = tmp_path / "raw.csv"
    write_raw_csv(raw, [{"Time_s": "0"}, {"Time_s": "1"}])

    try:
        normalizer.normalize_reference_csv(raw, tmp_path / "reference.csv", {"t": "Time_s"})
    except ValueError as exc:
        assert "missing required channel mapping" in str(exc)
    else:
        raise AssertionError("expected missing channel mapping to fail")


def test_normalize_reference_csv_refuses_missing_source_column(tmp_path: Path) -> None:
    normalizer = load_module()
    raw = tmp_path / "raw.csv"
    write_raw_csv(
        raw,
        [
            {"Time_s": "0", "Vx": "1"},
            {"Time_s": "1", "Vx": "2"},
        ],
    )

    try:
        normalizer.normalize_reference_csv(
            raw,
            tmp_path / "reference.csv",
            {
                "t": "Time_s",
                "vx": "Vx",
                "vy": "Vy",
                "yaw_rate": "YawRate",
                "pose_x": "X",
                "pose_y": "Y",
                "driver_steering": "Steer",
            },
        )
    except ValueError as exc:
        assert "missing source column" in str(exc)
        assert "Vy" in str(exc)
    else:
        raise AssertionError("expected missing source column to fail")


def test_normalize_reference_csv_refuses_unknown_unit_channel(tmp_path: Path) -> None:
    normalizer = load_module()
    raw = tmp_path / "raw.csv"
    write_raw_csv(
        raw,
        [
            {"t": "0", "vx": "1", "vy": "0", "yaw_rate": "0", "x": "0", "y": "0", "steer": "0"},
            {"t": "1", "vx": "1", "vy": "0", "yaw_rate": "0", "x": "1", "y": "0", "steer": "0"},
        ],
    )

    try:
        normalizer.normalize_reference_csv(
            raw,
            tmp_path / "reference.csv",
            {
                "t": "t",
                "vx": "vx",
                "vy": "vy",
                "yaw_rate": "yaw_rate",
                "pose_x": "x",
                "pose_y": "y",
                "driver_steering": "steer",
            },
            {"yaw": "deg/s"},
        )
    except ValueError as exc:
        assert "unknown target unit channel" in str(exc)
        assert "yaw" in str(exc)
    else:
        raise AssertionError("expected unknown unit channel to fail")


def test_normalize_reference_csv_refuses_non_monotonic_time(tmp_path: Path) -> None:
    normalizer = load_module()
    raw = tmp_path / "raw.csv"
    write_raw_csv(
        raw,
        [
            {"t": "0", "vx": "1", "vy": "0", "yaw_rate": "0", "x": "0", "y": "0", "steer": "0"},
            {"t": "0", "vx": "1", "vy": "0", "yaw_rate": "0", "x": "1", "y": "0", "steer": "0"},
        ],
    )

    try:
        normalizer.normalize_reference_csv(
            raw,
            tmp_path / "reference.csv",
            {
                "t": "t",
                "vx": "vx",
                "vy": "vy",
                "yaw_rate": "yaw_rate",
                "pose_x": "x",
                "pose_y": "y",
                "driver_steering": "steer",
            },
        )
    except ValueError as exc:
        assert "t must be strictly increasing" in str(exc)
    else:
        raise AssertionError("expected non-monotonic time to fail")


def test_normalize_reference_csv_refuses_unsupported_unit(tmp_path: Path) -> None:
    normalizer = load_module()

    try:
        normalizer.convert_value(1.0, "vx", "mph")
    except ValueError as exc:
        assert "unsupported unit" in str(exc)
    else:
        raise AssertionError("expected unsupported unit to fail")
