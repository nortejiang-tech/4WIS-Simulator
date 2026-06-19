"""Tests for the fault injection framework (Task #13) and plugin strategies
(Tasks #14, #15).

Covers:
  * FaultInjector actuator fault modes
  * FaultInjector sensor fault modes
  * FaultInjector management (add / remove / clear / list)
  * UserJsStrategy pass-through behaviour
  * HotReloadStrategy loads template + computes angles
  * REST API: GET/POST/DELETE /api/faults
  * REST API: GET /api/user_python/status
  * Smoke: user_python and user_js appear in /api/strategies
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from sim4wis.core.state import VehicleParams
from sim4wis.fault import FaultConfig, FaultInjector, FaultType


# ---- FaultInjector: actuator faults -----------------------------------------

def test_motor_stuck_zero() -> None:
    fi = FaultInjector()
    fi.add(FaultConfig(fault_type=FaultType.MOTOR_STUCK_ZERO, wheel="fl"))
    cmd = np.array([0.3, -0.3, 0.2, -0.2])
    result = fi.apply_to_cmd(cmd)
    assert result[0] == 0.0
    assert result[1] == -0.3
    assert np.allclose(result[2:], [0.2, -0.2])


def test_motor_stuck_angle() -> None:
    fi = FaultInjector()
    fi.add(FaultConfig(fault_type=FaultType.MOTOR_STUCK_ANGLE, wheel="fr", value=0.4))
    cmd = np.array([0.3, -0.3, 0.2, -0.2])
    result = fi.apply_to_cmd(cmd)
    assert result[0] == 0.3   # unaffected
    assert abs(result[1] - 0.4) < 1e-9  # stuck at 0.4 rad


def test_motor_limited_range() -> None:
    fi = FaultInjector()
    fi.add(FaultConfig(fault_type=FaultType.MOTOR_LIMITED_RANGE, wheel="rr", value=0.1))
    cmd = np.array([0.3, -0.3, 0.2, 0.5])  # rr = 0.5 exceeds limit 0.1
    result = fi.apply_to_cmd(cmd)
    assert abs(result[3] - 0.1) < 1e-9   # clamped to 0.1
    assert result[0] == 0.3               # unaffected


def test_inactive_fault_skipped() -> None:
    fi = FaultInjector()
    fi.add(FaultConfig(fault_type=FaultType.MOTOR_STUCK_ZERO, wheel="fl", active=False))
    cmd = np.array([0.3, -0.3, 0.2, -0.2])
    result = fi.apply_to_cmd(cmd)
    assert np.allclose(result, cmd)


# ---- FaultInjector: sensor faults -------------------------------------------

def test_sensor_bias() -> None:
    fi = FaultInjector()
    fi.add(FaultConfig(fault_type=FaultType.SENSOR_BIAS, wheel="rl", value=0.05))
    delta = np.array([0.1, -0.1, 0.2, -0.2])
    reported = fi.apply_to_reported(delta)
    assert abs(reported[2] - 0.25) < 1e-9   # 0.2 + 0.05
    assert np.allclose(reported[[0, 1, 3]], delta[[0, 1, 3]])


def test_sensor_dropout_returns_last_value() -> None:
    fi = FaultInjector()
    fi.add(FaultConfig(fault_type=FaultType.SENSOR_DROPOUT, wheel="fl"))
    # First call — seeds the last-known value
    delta1 = np.array([0.3, -0.3, 0.2, -0.2])
    r1 = fi.apply_to_reported(delta1)
    assert abs(r1[0] - 0.0) < 1e-9   # initial last_reported is 0.0

    # Second call — should hold the value from r1 (which was 0.0)
    delta2 = np.array([0.5, -0.5, 0.3, -0.3])
    r2 = fi.apply_to_reported(delta2)
    assert abs(r2[0] - 0.0) < 1e-9   # still 0.0 (last known before fault became active)


def test_sensor_noise_has_zero_mean() -> None:
    """Over 10 000 samples, noise bias should be < 0.01 rad."""
    fi = FaultInjector()
    fi.add(FaultConfig(fault_type=FaultType.SENSOR_NOISE, wheel="fr", value=0.05))
    offsets = []
    base = np.zeros(4)
    for _ in range(10_000):
        r = fi.apply_to_reported(base.copy())
        offsets.append(r[1])
    assert abs(float(np.mean(offsets))) < 0.01


# ---- FaultInjector: management ----------------------------------------------

def test_add_and_list() -> None:
    fi = FaultInjector()
    cfg = fi.add(FaultConfig(fault_type=FaultType.MOTOR_STUCK_ZERO, wheel="fl"))
    faults = fi.list_faults()
    assert len(faults) == 1
    assert faults[0].id == cfg.id


def test_remove_existing() -> None:
    fi = FaultInjector()
    cfg = fi.add(FaultConfig(fault_type=FaultType.MOTOR_STUCK_ZERO, wheel="fl"))
    assert fi.remove(cfg.id) is True
    assert len(fi.list_faults()) == 0


def test_remove_nonexistent() -> None:
    fi = FaultInjector()
    assert fi.remove("no-such-id") is False


def test_clear() -> None:
    fi = FaultInjector()
    fi.add(FaultConfig(fault_type=FaultType.MOTOR_STUCK_ZERO, wheel="fl"))
    fi.add(FaultConfig(fault_type=FaultType.SENSOR_BIAS, wheel="fr", value=0.1))
    fi.clear()
    assert len(fi.list_faults()) == 0


def test_set_active_toggle() -> None:
    fi = FaultInjector()
    cfg = fi.add(FaultConfig(fault_type=FaultType.MOTOR_STUCK_ZERO, wheel="fl", active=True))
    assert fi.has_active is True
    assert fi.set_active(cfg.id, False) is True
    assert fi.has_active is False


def test_has_active_false_when_empty() -> None:
    fi = FaultInjector()
    assert fi.has_active is False


# ---- plugin strategies ------------------------------------------------------

def test_user_js_strategy_zero_on_init() -> None:
    from sim4wis.controller.user_js import UserJsStrategy
    p = VehicleParams()
    s = UserJsStrategy(p)
    from sim4wis.core.state import DriverInput, VehicleState
    cmd = s.compute(DriverInput(), VehicleState())
    assert np.allclose(cmd.delta_cmd, 0.0)


def test_user_js_strategy_uses_set_cmd() -> None:
    from sim4wis.controller.user_js import UserJsStrategy
    p = VehicleParams()
    s = UserJsStrategy(p)
    s.set_steer_cmd(0.3, -0.3, 0.15, -0.15)
    from sim4wis.core.state import DriverInput, VehicleState
    cmd = s.compute(DriverInput(), VehicleState())
    assert abs(cmd.delta_cmd[0] - 0.3) < 1e-9
    assert abs(cmd.delta_cmd[1] - (-0.3)) < 1e-9


def test_user_js_strategy_clips_to_limit() -> None:
    from sim4wis.controller.user_js import UserJsStrategy
    p = VehicleParams()
    s = UserJsStrategy(p)
    s.set_steer_cmd(99.0, -99.0, 99.0, -99.0)  # wildly out of range
    from sim4wis.core.state import DriverInput, VehicleState
    cmd = s.compute(DriverInput(), VehicleState())
    assert np.all(np.abs(cmd.delta_cmd) <= p.steer_limit + 1e-9)


def test_user_python_strategy_loads_template() -> None:
    from sim4wis.controller.user_python import HotReloadStrategy
    p = VehicleParams()
    s = HotReloadStrategy(p)
    st = s.status_dict()
    assert st["status"] in ("ok", "no_file")   # file may or may not exist in CI


def test_user_python_strategy_computes_when_file_exists(tmp_path) -> None:
    """Write a valid user_strategy.py and verify compute() returns sane angles."""
    code = """
def compute(driver, state):
    angle = driver['steering'] * state['steer_limit']
    return {'delta_cmd': [angle, angle, -angle, -angle]}
"""
    plugin = tmp_path / "user_strategy.py"
    plugin.write_text(code)

    from sim4wis.controller.user_python import HotReloadStrategy
    p = VehicleParams()
    s = HotReloadStrategy(p)
    # Redirect to the temp file and reload
    s._path = plugin
    s._load()
    assert s._status == "ok", f"load failed: {s._error}"

    from sim4wis.core.state import DriverInput, VehicleState
    d = DriverInput(steering=0.5)
    st = VehicleState()
    cmd = s.compute(d, st)
    expected = 0.5 * p.steer_limit
    assert abs(cmd.delta_cmd[0] - expected) < 1e-6
    assert abs(cmd.delta_cmd[2] + expected) < 1e-6


# ---- REST API smoke ---------------------------------------------------------

def test_fault_rest_list_empty() -> None:
    from fastapi.testclient import TestClient
    from sim4wis.core.simulator import reset_simulator
    from sim4wis.main import app

    reset_simulator()
    client = TestClient(app)
    r = client.get("/api/faults")
    assert r.status_code == 200
    assert r.json() == {"faults": []}


def test_fault_rest_add_and_delete() -> None:
    from fastapi.testclient import TestClient
    from sim4wis.core.simulator import reset_simulator
    from sim4wis.main import app

    reset_simulator()
    client = TestClient(app)

    # Add a fault
    r = client.post("/api/faults", json={
        "fault_type": "motor_stuck_zero",
        "wheel": "fl",
        "value": 0.0,
        "active": True,
    })
    assert r.status_code == 201
    body = r.json()
    fault_id = body["id"]
    assert body["fault_type"] == "motor_stuck_zero"
    assert body["wheel"] == "fl"

    # List — should have 1
    r = client.get("/api/faults")
    assert len(r.json()["faults"]) == 1

    # Toggle inactive
    r = client.patch(f"/api/faults/{fault_id}", json={"active": False})
    assert r.status_code == 200
    assert r.json()["active"] is False

    # Delete
    r = client.delete(f"/api/faults/{fault_id}")
    assert r.status_code == 200

    # List — empty again
    r = client.get("/api/faults")
    assert r.json() == {"faults": []}


def test_fault_rest_clear_all() -> None:
    from fastapi.testclient import TestClient
    from sim4wis.core.simulator import reset_simulator
    from sim4wis.main import app

    reset_simulator()
    client = TestClient(app)

    for wheel in ("fl", "fr", "rl", "rr"):
        client.post("/api/faults", json={"fault_type": "sensor_bias", "wheel": wheel, "value": 0.1})

    r = client.get("/api/faults")
    assert len(r.json()["faults"]) == 4

    r = client.delete("/api/faults")
    assert r.status_code == 200
    assert r.json()["cleared"] is True

    r = client.get("/api/faults")
    assert r.json() == {"faults": []}


def test_strategies_include_user_plugins() -> None:
    """Smoke: user_python and user_js appear in /api/strategies."""
    from fastapi.testclient import TestClient
    from sim4wis.core.simulator import reset_simulator
    from sim4wis.main import app

    reset_simulator()
    client = TestClient(app)
    r = client.get("/api/strategies")
    assert r.status_code == 200
    names = r.json()["strategies"]
    assert "user_python" in names
    assert "user_js" in names


def test_user_python_status_endpoint() -> None:
    from fastapi.testclient import TestClient
    from sim4wis.core.simulator import reset_simulator
    from sim4wis.main import app

    reset_simulator()
    client = TestClient(app)
    r = client.get("/api/user_python/status")
    assert r.status_code == 200
    body = r.json()
    assert "status" in body
    assert body["status"] in ("ok", "no_file", "error")
    assert "file_path" in body
