"""Smoke test — backend boots and /health returns ok."""

from fastapi.testclient import TestClient

from sim4wis.main import app
from sim4wis.vehicle.model_registry import model_ids, model_layer


def test_health() -> None:
    client = TestClient(app)
    r = client.get("/health")
    assert r.status_code == 200
    assert r.json() == {"status": "ok"}


def test_strategies_listed() -> None:
    client = TestClient(app)
    r = client.get("/api/strategies")
    assert r.status_code == 200
    names = r.json()["strategies"]
    for required in ("ackermann", "ideal_ackermann", "rear_wheel_steer", "crab", "zero_radius"):
        assert required in names


def test_model_registry_marks_primary_vs_research() -> None:
    assert model_ids(include_research=False) == ["kinematic", "simplified_dynamic"]
    assert "multibody" in model_ids()
    assert model_layer("kinematic") == "primary"
    assert model_layer("simplified_dynamic") == "primary"
    assert model_layer("multibody") == "research"


def test_model_endpoint_exposes_model_layers() -> None:
    client = TestClient(app)
    r = client.get("/api/model")
    assert r.status_code == 200
    body = r.json()
    assert body["primary"] == ["kinematic", "simplified_dynamic"]
    layers = {m["id"]: m["layer"] for m in body["models"]}
    assert layers["kinematic"] == "primary"
    assert layers["simplified_dynamic"] == "primary"
    assert layers["multibody"] == "research"
