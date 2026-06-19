"""API tests — /api/params validation + partial merge semantics."""

from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from sim4wis.core.simulator import reset_simulator
from sim4wis.main import app


@pytest.fixture
def client():
    reset_simulator()
    with TestClient(app) as c:
        yield c
    reset_simulator()


def test_get_params_full_set(client: TestClient) -> None:
    d = client.get("/api/params").json()
    for key in ("wheelbase", "mass", "servo_kp", "tire_model", "suspension"):
        assert key in d
    assert "spring_rate" in d["suspension"]


def test_partial_update_only_touches_given_fields(client: TestClient) -> None:
    before = client.get("/api/params").json()
    r = client.post("/api/params", json={"mass": 2500.0})
    assert r.status_code == 200
    after = r.json()
    assert after["mass"] == 2500.0
    assert after["wheelbase"] == before["wheelbase"]
    assert after["suspension"] == before["suspension"]


def test_nested_suspension_merge(client: TestClient) -> None:
    r = client.post("/api/params", json={"suspension": {"camber": -0.02}})
    assert r.status_code == 200
    d = r.json()
    assert d["suspension"]["camber"] == -0.02
    # untouched sibling field keeps its default
    assert d["suspension"]["scrub_radius"] == 0.015


@pytest.mark.parametrize("body", [
    {"mass": -5},
    {"mass": 0},
    {"wheelbase": 0.1},
    {"steer_limit": 3.0},
    {"tire_radius": 2.0},
    {"v_max": -1},
    {"tire_model": "magic"},
    {"suspension": {"scrub_radius": -0.01}},
])
def test_invalid_values_rejected_422(client: TestClient, body: dict) -> None:
    r = client.post("/api/params", json=body)
    assert r.status_code == 422


def test_cg_must_be_inside_wheelbase(client: TestClient) -> None:
    r = client.post("/api/params", json={"cg_to_front": 5.0})
    assert r.status_code == 422
    assert "wheelbase" in r.json()["detail"]


def test_valid_boundary_accepted(client: TestClient) -> None:
    r = client.post("/api/params", json={"steer_limit": 1.5708})
    assert r.status_code == 200
