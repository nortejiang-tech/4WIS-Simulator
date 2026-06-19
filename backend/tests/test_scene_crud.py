"""API tests — scene disturbance CRUD + project scene persistence."""

from __future__ import annotations

import numpy as np
import pytest
from fastapi.testclient import TestClient

from sim4wis.core.simulator import get_simulator, reset_simulator
from sim4wis.main import app


@pytest.fixture
def client():
    reset_simulator()
    with TestClient(app) as c:
        yield c
    reset_simulator()


def test_crud_roundtrip(client: TestClient) -> None:
    # Create
    r = client.post("/api/scene/disturbances",
                    json={"type": "ice_patch", "x": 20, "y": 0, "width": 6, "length": 10, "mu": 0.2})
    assert r.status_code == 200
    d = r.json()
    assert d["id"].startswith("ice_patch_")
    # Read
    scene = client.get("/api/scene").json()
    assert len(scene["disturbances"]) == 1
    # Update (partial)
    r = client.put(f"/api/scene/disturbances/{d['id']}", json={"mu": 0.5, "x": 30})
    assert r.status_code == 200
    assert r.json()["mu"] == 0.5
    assert r.json()["x"] == 30
    assert r.json()["width"] == 6      # untouched
    # Delete
    assert client.delete(f"/api/scene/disturbances/{d['id']}").status_code == 200
    assert client.delete(f"/api/scene/disturbances/{d['id']}").status_code == 404
    assert client.get("/api/scene").json()["disturbances"] == []


def test_created_disturbance_affects_effective_mu(client: TestClient) -> None:
    client.post("/api/scene/disturbances",
                json={"type": "ice_patch", "x": 50, "y": 0, "width": 10, "length": 10, "mu": 0.15})
    sim = get_simulator()
    assert sim.scene.effective_mu(np.array([50.0, 0.0])) == pytest.approx(0.15)
    assert sim.scene.effective_mu(np.array([0.0, 0.0])) == pytest.approx(sim.scene.base_mu)


@pytest.mark.parametrize("body", [
    {"type": "volcano", "x": 0, "y": 0},                                   # unknown type
    {"type": "ice_patch", "x": 0, "y": 0, "mu": 3.0},                      # μ out of range
    {"type": "speed_bump", "x": 0, "y": 0, "stiffness": 1e9},              # Fz-pulse blowup guard
    {"type": "slope", "x": 0, "y": 0, "angle": 1.0},                       # > 20° grade
    {"type": "ice_patch", "x": 0, "y": 0, "width": -1},
])
def test_invalid_disturbances_rejected(client: TestClient, body: dict) -> None:
    assert client.post("/api/scene/disturbances", json=body).status_code == 422


def test_unique_id_generation(client: TestClient) -> None:
    ids = set()
    for _ in range(3):
        r = client.post("/api/scene/disturbances",
                        json={"type": "slope", "x": 0, "y": 0, "angle": 0.1})
        ids.add(r.json()["id"])
    assert len(ids) == 3


def test_clear_all(client: TestClient) -> None:
    for _ in range(2):
        client.post("/api/scene/disturbances", json={"type": "ice_patch", "x": 0, "y": 0})
    r = client.post("/api/scene/clear")
    assert r.json()["removed"] == 2
    assert client.get("/api/scene").json()["disturbances"] == []


def test_project_save_persists_scene(client: TestClient, tmp_path) -> None:
    client.post("/api/scene/disturbances",
                json={"type": "speed_bump", "x": 30, "y": 0, "width": 6, "length": 0.5,
                      "height": 0.05, "stiffness": 1e5})
    name = "__pytest_scene_roundtrip"
    try:
        assert client.post(f"/api/projects/{name}", json={"name": name}).status_code == 200
        saved = client.get(f"/api/projects/{name}").json()
        assert len(saved["scene"]["disturbances"]) == 1
        assert saved["scene"]["disturbances"][0]["type"] == "speed_bump"
        # Round-trip: clear live scene, load project back.
        client.post("/api/scene/clear")
        r = client.post(f"/api/projects/{name}/load")
        assert r.json()["disturbances"] == 1
    finally:
        from sim4wis.paths import projects_dir
        f = projects_dir() / f"{name}.yaml"
        if f.exists():
            f.unlink()
