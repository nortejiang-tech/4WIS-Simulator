"""Platform refactor Phase A — experiment schema / headless session / batch.

Covers:
    * Experiment YAML roundtrip + steer-profile shapes
    * SimSession determinism (same experiment twice → identical arrays)
    * KPI computation (base + step-response metrics)
    * variant expansion (dotted overrides) + sync batch acceptance
    * REST smoke: save/list experiment, start batch, poll, fetch run data
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest

from sim4wis.experiment import batch, store
from sim4wis.experiment.kpi import compute_kpis
from sim4wis.experiment.schema import Experiment, Maneuver, ManeuverStep, SteerProfile
from sim4wis.experiment.session import run_experiment


@pytest.fixture(autouse=True)
def _isolated_dirs(tmp_path, monkeypatch):
    monkeypatch.setenv("SIM4WIS_EXPERIMENTS_DIR", str(tmp_path / "experiments"))
    monkeypatch.setenv("SIM4WIS_RUNS_DIR", str(tmp_path / "runs"))


# Steering amplitudes are normalised curvature fractions for ideal_ackermann:
# at 60 km/h, amp 0.05 → R≈61 m → a_y≈4.6 m/s² (firm but well within grip).
# Larger values would demand >1 g and turn the run into a drift test.
def _step_experiment(speed_kmh: float = 60.0, amp: float = 0.05) -> Experiment:
    return Experiment(
        name="step_test",
        strategy="ideal_ackermann",
        model_type="simplified_dynamic",
        maneuver=Maneuver(name="step", steps=[
            ManeuverStep(name="accel", duration=5.0,
                         steer=SteerProfile(kind="constant", amplitude=0.0),
                         speed_kmh=speed_kmh, speed_ramp_s=3.0),
            ManeuverStep(name="step", duration=6.0,
                         steer=SteerProfile(kind="step", amplitude=amp, t_step=0.5)),
        ]),
        record_hz=50.0,
    )


def _dlc_experiment(strategy: str, speed_kmh: float) -> Experiment:
    return Experiment(
        name="dlc",
        strategy=strategy,
        maneuver=Maneuver(name="dlc", steps=[
            ManeuverStep(name="accel", duration=4.0, speed_kmh=speed_kmh, speed_ramp_s=2.5),
            ManeuverStep(name="dlc", duration=8.0,
                         steer=SteerProfile(kind="dlc", amplitude=0.06)),
        ]),
    )


# ---- schema -------------------------------------------------------------------


def test_schema_yaml_roundtrip() -> None:
    exp = _step_experiment()
    store.save_experiment(exp)
    names = [e["name"] for e in store.list_experiments()]
    assert "step_test" in names
    loaded = store.load_experiment("step_test")
    assert loaded == exp
    assert store.delete_experiment("step_test")


def test_steer_profiles_shapes() -> None:
    step = SteerProfile(kind="step", amplitude=0.5, t_step=0.5)
    assert step.value(0.0, 8.0) == 0.0
    assert step.value(1.0, 8.0) == 0.5

    ramp = SteerProfile(kind="ramp", start=-0.2, amplitude=0.6)
    assert ramp.value(0.0, 4.0) == pytest.approx(-0.2)
    assert ramp.value(4.0, 4.0) == pytest.approx(0.6)

    sine = SteerProfile(kind="sine", amplitude=0.3, freq_hz=0.5)
    assert sine.value(0.5, 8.0) == pytest.approx(0.3)  # quarter period

    dlc = SteerProfile(kind="dlc", amplitude=0.5)
    assert dlc.value(0.27 * 8.0, 8.0) > 0.4       # first bump peaks left
    assert dlc.value(0.65 * 8.0, 8.0) < -0.4      # second bump right
    assert dlc.value(0.0, 8.0) == 0.0

    sweep = SteerProfile(kind="sweep", amplitude=0.3, f0_hz=0.1, f1_hz=2.0)
    assert abs(sweep.value(0.0, 8.0)) < 1e-12


# ---- session -----------------------------------------------------------------


def test_session_reaches_speed_and_is_deterministic() -> None:
    exp = _step_experiment()
    r1 = run_experiment(exp)
    r2 = run_experiment(exp)

    assert r1.n_steps == int(round(11.0 / exp.dt))
    assert len(r1.t) == pytest.approx(11.0 * exp.record_hz, rel=0.02)
    # End of accel phase: vehicle holds the 60 km/h target.
    vx = np.asarray(r1.channels["vx"])
    t = np.asarray(r1.t)
    accel_end = np.searchsorted(t, 4.9)
    assert vx[accel_end] * 3.6 == pytest.approx(60.0, abs=2.0)
    # Bit-level determinism.
    assert r1.t == r2.t
    for name in r1.channels:
        assert r1.channels[name] == r2.channels[name], name


def test_follow_trajectory_path_override_runs() -> None:
    exp = Experiment(
        name="follow",
        strategy="follow_trajectory",
        path={"template": "double_lane_change"},
        mode_params={"cruise_speed": 8.0},
        maneuver=Maneuver(steps=[ManeuverStep(duration=6.0, speed_kmh=None)]),
    )
    r = run_experiment(exp)
    x = np.asarray(r.channels["pose_x"])
    assert x[-1] > 10.0            # actually drove along the path
    assert np.all(np.isfinite(x))


# ---- KPI ----------------------------------------------------------------------


def test_step_kpis() -> None:
    exp = _step_experiment()
    result = run_experiment(exp)
    k = compute_kpis(result, exp)

    assert k["yaw_rate_peak_dps"] > 1.0
    assert k["rack_force_peak_n"] > 100.0
    assert k["steer_energy_nms"] > 0.0
    assert "speed_error_rms_kmh" in k and k["speed_error_rms_kmh"] < 5.0
    # Step-response set present and sane.
    assert k["yaw_gain_dps"] > 0.0
    assert 0.0 < k.get("yaw_rise_time_s", 1e9) < 3.0
    assert k["yaw_settling_time_s"] < 6.0


# ---- batch ----------------------------------------------------------------------


def test_variant_expansion_dotted_paths() -> None:
    exp = _step_experiment()
    items = batch.expand_variants(exp, [
        {"label": "fast", "overrides": {"maneuver.steps.0.speed_kmh": 90}},
        {"label": "rws", "overrides": {"strategy": "rear_wheel_steer"}},
        {"label": "heavy", "overrides": {"vehicle.overrides.mass": 3300}},
    ])
    assert [lbl for lbl, _ in items] == ["fast", "rws", "heavy"]
    assert items[0][1].maneuver.steps[0].speed_kmh == 90
    assert items[1][1].strategy == "rear_wheel_steer"
    assert items[2][1].vehicle.overrides["mass"] == 3300
    # base untouched
    assert exp.maneuver.steps[0].speed_kmh == 60.0


def test_batch_acceptance_dlc_matrix() -> None:
    """The Phase-A acceptance case (shrunk): strategies × speeds DLC matrix."""
    exp = _dlc_experiment("ideal_ackermann", 40.0)
    variants = [
        {"label": f"{s}@{v}", "overrides": {"strategy": s, "maneuver.steps.0.speed_kmh": v}}
        for s in ("ideal_ackermann", "rear_wheel_steer")
        for v in (40.0, 60.0)
    ]
    t0 = time.time()
    job = batch.run_batch_sync(exp, variants)
    elapsed = time.time() - t0

    assert job.status == "done"
    assert job.done == job.total == 4
    assert len(job.run_ids) == 4
    assert elapsed < 60.0
    # KPIs exist per run and differ across speeds (physics actually varied).
    yaw40 = job.kpis[0]["yaw_rate_peak_dps"]
    yaw60 = job.kpis[1]["yaw_rate_peak_dps"]
    assert yaw40 != yaw60

    # Artifacts on disk and readable back.
    runs = store.list_runs()
    assert len(runs) == 4
    meta = store.load_run_meta(job.run_ids[0])
    assert meta["kpis"]["yaw_rate_peak_dps"] == pytest.approx(yaw40)
    data = store.load_run_channels(job.run_ids[0], ["vx", "yaw_rate"], decimate=2)
    assert len(data["t"]) > 100
    assert set(data.keys()) == {"t", "vx", "yaw_rate"}
    # Cleanup path works.
    assert store.delete_run(job.run_ids[0])
    assert len(store.list_runs()) == 3


def test_run_csv_nan_roundtrip() -> None:
    """icr_dev is NaN while driving straight — must come back as None."""
    exp = Experiment(
        name="straight",
        maneuver=Maneuver(steps=[ManeuverStep(duration=2.0, speed_kmh=30.0)]),
    )
    result = run_experiment(exp)
    kpis = compute_kpis(result, exp)
    run_id = store.save_run(result, label="straight", kpis=kpis)
    data = store.load_run_channels(run_id, ["icr_dev_fl"])
    assert any(v is None for v in data["icr_dev_fl"])


# ---- REST smoke -----------------------------------------------------------------


def test_rest_experiment_and_batch_flow() -> None:
    from fastapi.testclient import TestClient
    from sim4wis.core.simulator import reset_simulator
    from sim4wis.main import app

    reset_simulator()
    exp = _step_experiment()
    with TestClient(app) as client:
        # Save + list + get definition.
        r = client.post("/api/experiments/step_test", json=exp.model_dump(mode="json"))
        assert r.status_code == 200
        assert any(e["name"] == "step_test" for e in client.get("/api/experiments").json()["experiments"])
        got = client.get("/api/experiments/step_test").json()
        assert got["strategy"] == "ideal_ackermann"

        # Templates listing.
        tpl = client.get("/api/maneuver-templates").json()
        assert "dlc" in tpl["steer_kinds"]
        assert "double_lane_change" in tpl["path_templates"]

        # Start a short batch by name and poll to completion.
        short = exp.model_dump(mode="json")
        short["maneuver"]["steps"][0]["duration"] = 1.0
        short["maneuver"]["steps"][1]["duration"] = 2.0
        r = client.post("/api/batch", json={"experiment": short, "variants": []})
        assert r.status_code == 200
        job_id = r.json()["job_id"]
        deadline = time.time() + 30.0
        status = None
        while time.time() < deadline:
            status = client.get(f"/api/batch/{job_id}").json()
            if status["status"] in ("done", "error"):
                break
            time.sleep(0.1)
        assert status is not None and status["status"] == "done", status
        assert status["done"] == 1

        run_id = status["runs"][0]["run_id"]
        meta = client.get(f"/api/runs/{run_id}").json()
        assert meta["n_samples"] > 0
        data = client.get(f"/api/runs/{run_id}/data", params={"channels": "vx"}).json()
        assert len(data["vx"]) == len(data["t"]) > 0
        assert client.delete(f"/api/runs/{run_id}").status_code == 200

        # 404s
        assert client.get("/api/runs/nope_123").status_code in (400, 404)
        assert client.get("/api/batch/nope").status_code == 404

    assert math.isfinite(1.0)  # sentinel: file executed to the end
