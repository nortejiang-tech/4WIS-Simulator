"""Tests — /api/study/*, the contract the CLI and the MCP server share.

The trace endpoint gets the most attention here. It is the one place a caller
can ask for something big, and the guard rails on it (channels required, sample
count capped, decimation applied server-side) are the difference between an
agent reading a result and an agent filling its context with a CSV.
"""

from __future__ import annotations

import time

import pytest
from fastapi.testclient import TestClient

from sim4wis.main import create_app


@pytest.fixture(autouse=True)
def _isolated_dirs(tmp_path, monkeypatch):
    monkeypatch.setenv("SIM4WIS_RUNS_DIR", str(tmp_path / "runs"))
    monkeypatch.setenv("SIM4WIS_STUDIES_DIR", str(tmp_path / "studies"))


@pytest.fixture
def client():
    with TestClient(create_app()) as c:
        yield c


SPEC = {
    "study": "api_check",
    "question": "does the HTTP face carry a whole study?",
    "model": "simplified_dynamic",
    "baseline": {
        "name": "s",
        "strategy": "ideal_ackermann",
        "maneuver": {"steps": [
            {"duration": 2.0, "speed_kmh": 60, "steer": {"kind": "constant", "amplitude": 0.0}},
            {"duration": 8.0, "speed_kmh": 60,
             "steer": {"kind": "step", "amplitude": 1.0, "unit": "front_deg", "t_step": 0.5}},
        ]},
    },
    "sweep": {"speed_kmh": {"values": [40, 60], "bind": "maneuver.steps.1.speed_kmh"}},
    "metrics": [
        {"name": "r_dps", "expr": "degrees(steady(yaw_rate))"},
        {"name": "ay", "expr": "steady(ay)"},
    ],
    "criteria": [{"metric": "ay", "must": "abs < 4.0", "at": "all"}],
    "report": {"template": "sweep"},
}


def _run(client) -> dict:
    started = client.post("/api/study/run", json=SPEC)
    assert started.status_code == 200, started.text
    job_id = started.json()["job_id"]
    for _ in range(300):
        job = client.get(f"/api/study/jobs/{job_id}").json()
        if job["status"] != "running":
            return job
        time.sleep(0.1)
    pytest.fail("study job did not finish")


class TestCapabilities:
    def test_serves_everything_needed_to_write_a_spec(self, client):
        caps = client.get("/api/study/capabilities").json()
        assert {m["id"] for m in caps["models"]} >= {"kinematic", "simplified_dynamic", "multibody"}
        assert any(m["name"] == "yaw_gain_dps" for m in caps["metrics"])
        assert all("requires" in m for m in caps["metrics"])
        assert "ideal_ackermann" in caps["strategies"]
        assert "yaw_rate" in caps["channels"] and "delta_fl" in caps["channels"]

    def test_states_what_is_not_implemented_yet(self, client):
        notes = " ".join(client.get("/api/study/capabilities").json()["notes"])
        assert "solve_for" in notes and "envelope" in notes


class TestDryRun:
    def test_reports_the_grid_and_the_cost(self, client):
        d = client.post("/api/study/dry-run", json=SPEC).json()
        assert d["ok"] and d["grid"] == 2
        assert d["sim_seconds"] == pytest.approx(20.0)
        assert d["cells"] == ["speed_kmh=40", "speed_kmh=60"]

    def test_names_the_problem_without_running(self, client):
        bad = {**SPEC, "metrics": ["not_a_metric"], "criteria": []}
        d = client.post("/api/study/dry-run", json=bad).json()
        assert not d["ok"] and "unknown metric" in d["problems"][0]

    def test_an_invalid_spec_is_refused_by_run_too(self, client):
        bad = {**SPEC, "metrics": ["not_a_metric"], "criteria": []}
        r = client.post("/api/study/run", json=bad)
        assert r.status_code == 400 and "problems" in r.json()["detail"]


class TestRun:
    def test_carries_a_whole_study(self, client):
        job = _run(client)
        assert job["status"] == "done", job["error"]
        result = job["result"]
        assert [r["label"] for r in result["rows"]] == ["speed_kmh=40", "speed_kmh=60"]
        assert all("r_dps" in r["metrics"] for r in result["rows"])
        assert result["all_passed"] is True
        assert result["provenance"]["params_hash"]

    def test_the_study_is_retrievable_afterwards(self, client):
        job = _run(client)
        sid = job["study_id"]
        assert client.get(f"/api/study/{sid}").json()["spec_digest"] == job["result"]["spec_digest"]
        assert sid in [s["study_id"] for s in client.get("/api/study").json()["studies"]]
        report = client.get(f"/api/study/{sid}/report")
        assert report.status_code == 200 and b"<table" in report.content

    def test_unknown_ids_are_404(self, client):
        assert client.get("/api/study/nope").status_code == 404
        assert client.get("/api/study/jobs/nope").status_code == 404


class TestTrace:
    def test_returns_only_the_channels_asked_for_downsampled(self, client):
        run_id = _run(client)["result"]["rows"][0]["run_id"]
        tr = client.get(f"/api/study/trace/{run_id}",
                        params={"channels": "vx,ay", "max_points": 25}).json()
        assert set(tr["channels"]) == {"t", "vx", "ay"}
        assert tr["n_samples"] <= 25 < tr["of_total"]
        assert tr["decimate"] > 1

    def test_channels_are_required(self, client):
        run_id = _run(client)["result"]["rows"][0]["run_id"]
        # Returning everything by default is the failure this endpoint exists
        # to prevent, so the parameter has no default at all.
        assert client.get(f"/api/study/trace/{run_id}").status_code == 422

    def test_the_sample_cap_is_enforced(self, client):
        run_id = _run(client)["result"]["rows"][0]["run_id"]
        r = client.get(f"/api/study/trace/{run_id}",
                       params={"channels": "vx", "max_points": 10_000_000})
        assert r.status_code == 422

    def test_unknown_channels_are_reported_not_ignored(self, client):
        run_id = _run(client)["result"]["rows"][0]["run_id"]
        tr = client.get(f"/api/study/trace/{run_id}",
                        params={"channels": "vx,not_a_channel"}).json()
        assert tr["missing_channels"] == ["not_a_channel"]
