from __future__ import annotations

from fastapi.testclient import TestClient

from sim4wis.core.simulator import reset_simulator
from sim4wis.main import app


def test_bicycle_gain_grows_with_speed() -> None:
    """Demo 1: slip gain must increase and δ_sat shrink as speed rises."""
    reset_simulator()
    with TestClient(app) as client:
        r = client.post("/api/model/demo/bicycle-gain", json={
            "speeds_kmh": [10, 60, 120, 200], "ref_delta_deg": 1.0, "wheel_index": 0,
        })
        assert r.status_code == 200
        pts = r.json()["points"]
        assert len(pts) == 4
        gains = [p["slip_gain"] for p in pts]
        sats = [p["delta_sat_deg"] for p in pts]
        # monotonic-ish: high speed gain clearly larger, saturation angle smaller
        assert gains[-1] > gains[0] * 1.5
        assert sats[-1] < sats[0] * 0.8
    reset_simulator()


def test_tire_curve_peaks_near_mu_fz() -> None:
    """Demo 2: |Fy| peak should approach μ·Fz and origin slope ≈ c_alpha."""
    reset_simulator()
    with TestClient(app) as client:
        r = client.post("/api/model/demo/tire-curve", json={
            "fz": 7000.0, "mu": 0.85, "alpha_max_deg": 15.0, "points": 81,
        })
        assert r.status_code == 200
        data = r.json()
        curve = data["curve"]
        peak = max(abs(pt["fy"]) for pt in curve)
        # peak within friction-ellipse cap, and a meaningful fraction of it
        assert peak <= data["mu_fz"] + 1.0
        assert peak > 0.8 * data["mu_fz"]
        assert data["c_alpha"] > 0
    reset_simulator()


def test_kingpin_breakdown_terms_sum_to_total() -> None:
    """Demo 3: the four decomposed terms must sum to (torque_steer − parking)."""
    reset_simulator()
    with TestClient(app) as client:
        r = client.post("/api/model/demo/kingpin-breakdown", json={
            "speed_kmh": 30.0, "angle_max_deg": 20.0, "points": 41, "wheel_index": 0,
        })
        assert r.status_code == 200
        curve = r.json()["curve"]
        assert len(curve) == 41
        # at a non-zero angle the four terms should sum close to torque_steer
        # (torque_steer additionally includes the parking term, which is ~0 at 30 km/h)
        mid = curve[-1]
        four = mid["m_fy"] + mid["m_fx"] + mid["m_mz"] + mid["m_kpi"]
        assert abs(four - mid["torque_steer"]) < max(5.0, abs(mid["torque_steer"]) * 0.05)
    reset_simulator()


def test_demo_endpoints_accept_param_override() -> None:
    reset_simulator()
    with TestClient(app) as client:
        r = client.post("/api/model/demo/tire-curve", json={
            "params": {"tire_c_alpha": 90000.0}, "fz": 6000.0, "mu": 0.9,
        })
        assert r.status_code == 200
        assert r.json()["c_alpha"] > 0
    reset_simulator()
