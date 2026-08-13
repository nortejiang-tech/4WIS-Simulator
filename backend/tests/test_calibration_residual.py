"""Tests — the calibration residual panel (C3 · 3.1a).

The honest end-to-end check is a self-comparison: export a sim run's own
channels as a "bench CSV", hand them to the panel, and the residuals must
come back ~zero — including after the bench clock has been shifted, which
exercises the alignment. A perturbed plant must then show up as nonzero
residuals, and a misspelled column must be refused, not guessed at.
"""

from __future__ import annotations

import csv
from pathlib import Path

import pytest
import yaml

from sim4wis.calibration.residual import (
    load_reference,
    residual_panel,
    run_sim_channels,
)

ONCENTRE = Path(__file__).resolve().parents[2] / "procedures" / "iso13674_oncentre.yaml"
CHANNELS = ("t", "steer_hand_angle", "steer_hand_torque", "ay", "yaw_rate", "vx")


@pytest.fixture(scope="module")
def bench_csv(tmp_path_factory) -> Path:
    """A synthetic "weave bench CSV": the sim's own channels, exported."""
    t, ch, _meta = run_sim_channels(ONCENTRE)
    out = tmp_path_factory.mktemp("cal") / "weave_bench.csv"
    with out.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(CHANNELS)
        for i in range(t.size):
            writer.writerow([repr(float(t[i]))] + [repr(float(ch[c][i])) for c in CHANNELS[1:]])
    return out


def test_self_comparison_has_zero_residuals(bench_csv, tmp_path):
    out = residual_panel(bench_csv, ONCENTRE,
                         out_html=tmp_path / "self.html")
    assert out["align_channel"] == "steer_hand_angle"
    for c in ("steer_hand_angle", "steer_hand_torque", "ay", "yaw_rate", "vx"):
        r = out["residuals"][c]
        assert r["rms_over_std"] < 0.02, (c, r)
        assert r["corr"] > 0.99, (c, r)


def test_alignment_recovers_a_shifted_clock(bench_csv, tmp_path):
    # Shift the bench clock by +0.35 s: the panel must find it and cancel it.
    rows = list(csv.reader(bench_csv.open(encoding="utf-8")))
    shifted = tmp_path / "shifted.csv"
    with shifted.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(rows[0])
        for row in rows[1:]:
            writer.writerow([f"{float(row[0]) + 0.35:.6f}"] + row[1:])
    out = residual_panel(shifted, ONCENTRE, out_html=tmp_path / "shifted.html")
    # The bench clock reads 0.35 s ahead of the sim's; the returned shift is
    # the offset to apply to the sim trace, hence negative.
    assert out["shift_s"] == pytest.approx(-0.35, abs=0.02), out["shift_s"]
    r = out["residuals"]["steer_hand_torque"]
    assert r["rms_over_std"] < 0.02, r


def test_a_perturbed_plant_shows_nonzero_residuals(bench_csv, tmp_path):
    # Rack friction 900 N instead of 260 N: the weave hand torque must differ.
    raw = yaml.safe_load(ONCENTRE.read_text(encoding="utf-8"))
    raw["study"] = "cal_perturbed"
    raw["baseline"]["vehicle"]["overrides"]["steering_system"]["rack"] = {
        "coulomb_friction_n": 900.0,
    }
    variant = tmp_path / "perturbed.yaml"
    variant.write_text(yaml.safe_dump(raw, allow_unicode=True, sort_keys=False),
                       encoding="utf-8")
    out = residual_panel(bench_csv, variant, out_html=tmp_path / "perturbed.html")
    r = out["residuals"]["steer_hand_torque"]
    assert r["rms_over_std"] > 0.02, r
    assert r["corr"] < 0.999, r


def test_unknown_columns_are_refused(tmp_path):
    bad = tmp_path / "bad.csv"
    bad.write_text(
        "t,steer_hand_angle,hand_toque\n" "0.0,0.1,0.2\n", encoding="utf-8"
    )
    with pytest.raises(ValueError, match="unknown channel"):
        load_reference(bad)


def test_the_report_is_self_contained(bench_csv, tmp_path):
    out = residual_panel(bench_csv, ONCENTRE, out_html=tmp_path / "rep.html")
    text = Path(out["report"]).read_text(encoding="utf-8")
    assert "<table" in text and "<svg" in text
    assert "3.1b" in text  # the guardrail note ships with every report
