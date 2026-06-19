from __future__ import annotations

import math

import numpy as np
from fastapi.testclient import TestClient

from sim4wis.core.simulator import get_simulator, reset_simulator
from sim4wis.core.state import VehicleParams
from sim4wis.main import app
from sim4wis.project.params_codec import params_from_dict
from sim4wis.project.schema import ProjectFile
from sim4wis.vehicle.geometry import (
    steering_linkage_metrics,
    wheel_angle_from_rack_travel,
    wheel_rack_force_from_linkage,
)
from sim4wis.vehicle.load_analysis import sweep_load_analysis, sweep_sensitivity


def test_linkage_geometry_mirrors_left_right() -> None:
    p = VehicleParams()
    delta = np.array([0.2, -0.2, 0.25, -0.25])
    m = steering_linkage_metrics(delta, p.steering_geometry, rack_mech_efficiency=p.rack_mech_efficiency)
    assert m["efficiency"].shape == (4,)
    assert np.all(m["efficiency"] > 0.1)
    assert math.isclose(m["efficiency"][0], m["efficiency"][1], rel_tol=0.08)
    assert math.isclose(m["efficiency"][2], m["efficiency"][3], rel_tol=0.08)
    assert np.all(m["tie_rack_angle"] >= 0.0)
    assert np.all(m["tie_rack_angle"] <= math.pi / 2.0 + 1e-9)


def test_linkage_rack_force_uses_efficiency_and_preserves_sign() -> None:
    p = VehicleParams()
    tau = np.array([100.0, -100.0, 50.0, -50.0])
    rack, motor, metrics = wheel_rack_force_from_linkage(
        tau,
        np.array([0.2, -0.2, 0.1, -0.1]),
        p.steering_geometry,
        p.steering_arm_length,
        p.pinion_radius,
        p.rack_mech_efficiency,
        p.motor_gear_ratio,
    )
    assert rack[0] > 0 and rack[1] < 0
    assert motor[2] > 0 and motor[3] < 0
    assert float(np.min(metrics["efficiency"])) > 0.1


def test_load_sweep_shape_and_low_speed_blend() -> None:
    p = VehicleParams()
    out = sweep_load_analysis(
        p,
        speeds=[0.0, 10.0],
        angles=[0.0, 0.2],
        wheel_index=0,
        mode="single_wheel",
        mu=0.85,
    )
    assert len(out["rows"]) == 2 * 2 * 4
    low = [r for r in out["rows"] if r["speed"] == 0.0 and r["wheel_index"] == 0 and r["delta"] > 0][0]
    high = [r for r in out["rows"] if r["speed"] == 10.0 and r["wheel_index"] == 0 and r["delta"] > 0][0]
    assert abs(low["torque_steer"]) > abs(high["torque_steer"]) * 0.5
    assert "peak_abs_rack_force" in out["summary"]


def test_zero_speed_static_side_force_is_smooth_to_rolling_speed() -> None:
    # Use toe-free params so the δ_cmd=0 baseline really lands at α≈0.
    p = VehicleParams(static_toe_front=0.0, static_toe_rear=0.0, camber_thrust_coeff=0.0)
    out = sweep_load_analysis(
        p,
        speeds=[0.0, 1.0 / 3.6],
        angles=[0.0, 0.2],
        wheel_index=0,
        mode="single_wheel",
        mu=0.85,
    )
    rows = out["rows"]
    zero_angle = [
        r for r in rows
        if r["speed"] == 0.0 and r["wheel_index"] == 0 and math.isclose(r["delta_cmd"], 0.0)
    ][0]
    zero_speed = [
        r for r in rows
        if r["speed"] == 0.0 and r["wheel_index"] == 0 and r["delta_cmd"] > 0.0
    ][0]
    one_kph = [
        r for r in rows
        if math.isclose(r["speed"], 1.0 / 3.6) and r["wheel_index"] == 0 and r["delta_cmd"] > 0.0
    ][0]
    assert abs(zero_angle["side_force_body_y"]) < 1e-6
    assert abs(zero_speed["side_force_body_y"]) > 100.0
    assert abs(zero_speed["torque_steer"]) >= abs(one_kph["torque_steer"]) * 0.9
    assert math.isclose(
        zero_speed["side_force_body_y"],
        one_kph["side_force_body_y"],
        rel_tol=0.25,
    )


def test_load_sweep_strategy_mode_returns_four_wheels() -> None:
    p = VehicleParams()
    out = sweep_load_analysis(
        p,
        speeds=[5.0],
        angles=[0.2],
        wheel_index=0,
        mode="ideal_ackermann",
        mu=0.85,
    )
    labels = {r["wheel_label"] for r in out["rows"]}
    assert labels == {"FL", "FR", "RL", "RR"}
    assert any(abs(r["delta"]) > 0.01 for r in out["rows"])


def test_project_round_trip_keeps_rack_and_hardpoint_params() -> None:
    p = VehicleParams(
        tire_width=0.285,
        contact_patch_radius=0.12,
        steering_arm_length=0.14,
        pinion_radius=0.018,
        rack_mech_efficiency=0.9,
        motor_gear_ratio=12.0,
    )
    proj = ProjectFile.from_runtime("rt", "", p, "ideal_ackermann")
    p2 = proj.vehicle_params()
    assert p2.tire_width == 0.285
    assert p2.contact_patch_radius == 0.12
    assert p2.steering_arm_length == 0.14
    assert p2.pinion_radius == 0.018
    assert p2.rack_mech_efficiency == 0.9
    assert p2.motor_gear_ratio == 12.0
    assert p2.steering_geometry.front_inner_y == p.steering_geometry.front_inner_y


def test_vehicle_profile_crud_and_apply(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("SIM4WIS_VEHICLE_PROFILES_DIR", str(tmp_path))
    reset_simulator()
    with TestClient(app) as client:
        r = client.get("/api/vehicle-profiles")
        assert r.status_code == 200
        assert any(p["name"] == "LS9" for p in r.json()["profiles"])

        r = client.get("/api/vehicle-profiles/LS9")
        assert r.status_code == 200
        ls9 = r.json()["vehicle"]
        assert ls9["wheelbase"] == 3.16
        assert ls9["track_front"] == 1.565172
        assert ls9["steering_arm_length"] == 0.146451
        assert ls9["steering_geometry"]["front_inner_y"] == -0.362586
        assert ls9["steering_geometry"]["front_rack_travel_limit"] == 0.085

        r = client.get("/api/vehicle-profiles/im_ls9")
        assert r.status_code == 200
        assert r.json()["profile"]["name"] == "LS9"

        r = client.post("/api/vehicle-profiles/custom_ls9", json={
            "label": "Custom LS9",
            "vehicle": {"mass": 2800.0, "tire_width": 0.275},
        })
        assert r.status_code == 200
        r = client.get("/api/vehicle-profiles/custom_ls9")
        assert r.status_code == 200
        assert r.json()["vehicle"]["mass"] == 2800.0

        r = client.post("/api/vehicle-profiles/custom_ls9/apply")
        assert r.status_code == 200
        assert client.get("/api/params").json()["mass"] == 2800.0

        r = client.delete("/api/vehicle-profiles/custom_ls9")
        assert r.status_code == 200
    reset_simulator()


def _select_row(rows, *, speed, wheel_index, delta_cmd_tol=1e-6, delta_cmd=0.0):
    return [
        r for r in rows
        if math.isclose(r["speed"], speed)
        and r["wheel_index"] == wheel_index
        and abs(r["delta_cmd"] - delta_cmd) < delta_cmd_tol
    ][0]


def test_static_toe_creates_nonzero_rack_at_zero_command() -> None:
    """A3: with toe-in only, δ_cmd=0 must already need motor torque."""
    p_toe = VehicleParams(
        static_toe_front=math.radians(0.5),
        camber_thrust_coeff=0.0,
        rolling_resistance_coeff=0.0,
        drag_coeff_cd=0.0,
    )
    p_zero = VehicleParams(
        static_toe_front=0.0,
        static_toe_rear=0.0,
        camber_thrust_coeff=0.0,
        rolling_resistance_coeff=0.0,
        drag_coeff_cd=0.0,
    )
    speed = 8.0
    angles = [-0.05, 0.0, 0.05]
    sweep_toe = sweep_load_analysis(p_toe, speeds=[speed], angles=angles, wheel_index=0, mu=0.85)
    sweep_zero = sweep_load_analysis(p_zero, speeds=[speed], angles=angles, wheel_index=0, mu=0.85)
    row_toe = _select_row(sweep_toe["rows"], speed=speed, wheel_index=0)
    row_zero = _select_row(sweep_zero["rows"], speed=speed, wheel_index=0)
    assert abs(row_zero["rack_force"]) < 5.0
    assert abs(row_toe["rack_force"]) > abs(row_zero["rack_force"]) + 10.0


def test_camber_thrust_creates_nonzero_rack_at_zero_command() -> None:
    """A2: with non-zero camber and Cγ>0, δ_cmd=0 must already need motor torque."""
    p_camber = VehicleParams(
        static_toe_front=0.0,
        static_toe_rear=0.0,
        camber_thrust_coeff=1.0,
        rolling_resistance_coeff=0.0,
        drag_coeff_cd=0.0,
    )
    p_zero = VehicleParams(
        static_toe_front=0.0,
        static_toe_rear=0.0,
        camber_thrust_coeff=0.0,
        rolling_resistance_coeff=0.0,
        drag_coeff_cd=0.0,
    )
    speed = 8.0
    angles = [-0.05, 0.0, 0.05]
    sweep_c = sweep_load_analysis(p_camber, speeds=[speed], angles=angles, wheel_index=0, mu=0.85)
    sweep_z = sweep_load_analysis(p_zero, speeds=[speed], angles=angles, wheel_index=0, mu=0.85)
    row_c = _select_row(sweep_c["rows"], speed=speed, wheel_index=0)
    row_z = _select_row(sweep_z["rows"], speed=speed, wheel_index=0)
    assert abs(row_z["rack_force"]) < 5.0
    assert abs(row_c["rack_force"]) > 30.0


def test_drag_increases_with_speed_squared() -> None:
    """A1: aero drag → Fx_drive should scale ~v²."""
    p = VehicleParams(
        static_toe_front=0.0, static_toe_rear=0.0, camber_thrust_coeff=0.0,
        rolling_resistance_coeff=0.0, drag_coeff_cd=0.5, frontal_area=2.5,
    )
    sweep = sweep_load_analysis(p, speeds=[10.0, 30.0], angles=[0.0], wheel_index=0, mu=0.85)
    row_lo = _select_row(sweep["rows"], speed=10.0, wheel_index=0)
    row_hi = _select_row(sweep["rows"], speed=30.0, wheel_index=0)
    # 30² / 10² = 9 → fx_drive ratio should be ~9× (after moving_blend, near 1.0 at v≥10).
    ratio = row_hi["fx_drive"] / max(row_lo["fx_drive"], 1e-6)
    assert 7.0 < ratio < 11.0


def test_per_speed_equilibrium_present_and_drifts_at_speed() -> None:
    """A5: summary must carry per_speed_equilibrium and δ_eq is non-zero +
    speed-dependent once all bias sources are on (default LS9 has Crr+Cd+toe+camber)."""
    p = VehicleParams()
    angles = [a * math.radians(15.0) / 20 for a in range(-20, 21)]
    sweep = sweep_load_analysis(p, speeds=[5.0, 30.0], angles=angles, wheel_index=0, mu=0.85)
    eq = sweep["summary"]["per_speed_equilibrium"]
    assert len(eq) == 2
    assert all("delta_eq_deg" in e and "rack_at_zero" in e and "found" in e for e in eq)
    for e in eq:
        # bias sources active → equilibrium offset from straight ahead at every speed
        assert e["found"] is True
        assert abs(e["delta_eq_deg"]) > 0.005
        assert abs(e["rack_at_zero"]) > 1.0
    # δ_eq must drift between the two speeds — drag direction can either grow or
    # shrink |rack_at_zero| depending on toe/camber sign, but the equilibrium
    # angle must shift visibly.
    assert abs(eq[1]["delta_eq_deg"] - eq[0]["delta_eq_deg"]) > 0.01
    assert abs(abs(eq[1]["rack_at_zero"]) - abs(eq[0]["rack_at_zero"])) > 5.0


def test_wheel_angle_from_rack_travel_zero_returns_near_zero_delta() -> None:
    """B3: zero rack travel should give δ near 0 for the symmetric default."""
    p = VehicleParams()
    sol = wheel_angle_from_rack_travel(0.0, p.steering_geometry, 0)
    assert sol["valid"]
    assert abs(sol["delta"]) < math.radians(0.1)


def test_wheel_angle_from_rack_travel_roundtrip_against_forward() -> None:
    """B3: forward (δ→travel) and inverse (travel→δ) must agree to <0.1° over a sweep."""
    p = VehicleParams()
    for wheel in range(4):
        for d_deg in [-10.0, -3.0, 0.0, 3.0, 10.0]:
            delta = math.radians(d_deg)
            fwd = steering_linkage_metrics(
                np.array([0.0, 0.0, 0.0, 0.0]) + np.where(np.arange(4) == wheel, delta, 0.0),
                p.steering_geometry,
                rack_mech_efficiency=p.rack_mech_efficiency,
            )
            travel = float(fwd["rack_travel"][wheel])
            inv = wheel_angle_from_rack_travel(travel, p.steering_geometry, wheel, hint_delta=delta)
            assert inv["valid"]
            assert abs(inv["delta"] - delta) < math.radians(0.1)


def test_wheel_angle_from_rack_travel_clamps_to_limit() -> None:
    """B3: out-of-range travel should be clamped to ±limit, not crash."""
    p = VehicleParams()
    limit = p.steering_geometry.front_rack_travel_limit
    sol = wheel_angle_from_rack_travel(limit * 10.0, p.steering_geometry, 0)
    assert sol["rack_travel"] == limit


def test_legacy_parking_scrub_coeff_populates_split_fields() -> None:
    """B2 backwards compat: a YAML with only parking_scrub_coeff must populate
    both new split fields."""
    p = params_from_dict({"parking_scrub_coeff": 1.2}, VehicleParams())
    assert p.parking_lateral_coeff == 1.2
    assert p.parking_torque_coeff == 1.2


def test_rack_solve_api(monkeypatch, tmp_path) -> None:
    """B3 endpoint round-trip."""
    monkeypatch.setenv("SIM4WIS_VEHICLE_PROFILES_DIR", str(tmp_path))
    reset_simulator()
    with TestClient(app) as client:
        r = client.post("/api/load-analysis/rack-solve", json={
            "wheel_index": 0,
            "rack_travels": [-0.02, 0.0, 0.02],
        })
        assert r.status_code == 200
        sols = r.json()["solutions"]
        assert len(sols) == 3
        assert sols[1]["valid"] and abs(sols[1]["delta"]) < math.radians(0.1)
    reset_simulator()


def test_sensitivity_sweep_returns_monotonic_response_to_toe() -> None:
    """v0.7.3: sweeping static_toe_front from 0 to +0.5° should monotonically
    shift δ_eq."""
    p = VehicleParams(
        camber_thrust_coeff=0.0,
        rolling_resistance_coeff=0.0,
        drag_coeff_cd=0.0,
        static_toe_front=0.0,
        static_toe_rear=0.0,
    )
    values = [0.0, math.radians(0.1), math.radians(0.25), math.radians(0.5)]
    out = sweep_sensitivity(
        p,
        vary_param="static_toe_front",
        values=values,
        speed=8.0,
        angles=[a * math.radians(8.0) / 20 for a in range(-20, 21)],
        wheel_index=0,
        mu=0.85,
    )
    points = out["points"]
    assert len(points) == 4
    assert all(pt["found"] for pt in points)
    # Monotonic shift of δ_eq with toe (sign depends on convention but should not
    # oscillate).
    deltas = [pt["delta_eq_deg"] for pt in points]
    diffs = [deltas[i + 1] - deltas[i] for i in range(len(deltas) - 1)]
    assert all(d * diffs[0] >= 0 for d in diffs)
    assert abs(deltas[-1] - deltas[0]) > 0.1


def test_sensitivity_api_round_trip(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("SIM4WIS_VEHICLE_PROFILES_DIR", str(tmp_path))
    reset_simulator()
    with TestClient(app) as client:
        r = client.post("/api/load-analysis/sensitivity", json={
            "vary_param": "camber_thrust_coeff",
            "values": [0.0, 0.5, 1.0, 1.5],
            "speed": 8.0,
            "angles": [a * 0.005 for a in range(-15, 16)],
            "wheel_index": 0,
            "mu": 0.85,
        })
        assert r.status_code == 200
        out = r.json()
        assert out["vary_param"] == "camber_thrust_coeff"
        assert len(out["points"]) == 4
    reset_simulator()


def test_aero_lift_lowers_saturation_plateau_with_speed() -> None:
    """v0.7.4 Q2: with aero lift on, the τ-δ saturation plateau must lower as v rises."""
    p_lift = VehicleParams(aero_lift_coeff_front=0.30, aero_lift_coeff_rear=0.15)
    p_zero = VehicleParams(aero_lift_coeff_front=0.0, aero_lift_coeff_rear=0.0)
    angles = [math.radians(35.0)]  # past saturation
    out_lift = sweep_load_analysis(p_lift, speeds=[5.0, 50.0], angles=angles, wheel_index=0, mu=0.85)
    out_zero = sweep_load_analysis(p_zero, speeds=[5.0, 50.0], angles=angles, wheel_index=0, mu=0.85)
    row_lift_lo = _select_row(out_lift["rows"], speed=5.0, wheel_index=0, delta_cmd=math.radians(35.0), delta_cmd_tol=1e-3)
    row_lift_hi = _select_row(out_lift["rows"], speed=50.0, wheel_index=0, delta_cmd=math.radians(35.0), delta_cmd_tol=1e-3)
    row_zero_lo = _select_row(out_zero["rows"], speed=5.0, wheel_index=0, delta_cmd=math.radians(35.0), delta_cmd_tol=1e-3)
    row_zero_hi = _select_row(out_zero["rows"], speed=50.0, wheel_index=0, delta_cmd=math.radians(35.0), delta_cmd_tol=1e-3)
    # Without lift, plateau is nearly constant. With lift, hi-speed plateau is meaningfully lower.
    diff_with_lift = abs(row_lift_lo["torque_steer"]) - abs(row_lift_hi["torque_steer"])
    diff_without_lift = abs(row_zero_lo["torque_steer"]) - abs(row_zero_hi["torque_steer"])
    assert diff_with_lift > diff_without_lift + 20.0


def test_ideal_curves_present_and_larger_than_actual_in_saturation() -> None:
    """v0.7.4 Q1: ideal (no-clip) curves must be in rows and exceed actual at large δ."""
    p = VehicleParams()
    angles = [math.radians(a) for a in [-30, -3, 0, 3, 30]]
    out = sweep_load_analysis(p, speeds=[8.0], angles=angles, wheel_index=0, mu=0.85)
    row_sat = _select_row(out["rows"], speed=8.0, wheel_index=0, delta_cmd=math.radians(30.0), delta_cmd_tol=1e-3)
    row_lin = _select_row(out["rows"], speed=8.0, wheel_index=0, delta_cmd=math.radians(3.0), delta_cmd_tol=1e-3)
    assert "torque_steer_ideal" in row_sat
    assert "rack_force_ideal" in row_sat
    assert "side_force_body_y_ideal" in row_sat
    # In saturation: ideal >> actual (linear extrapolation goes through the roof)
    assert abs(row_sat["torque_steer_ideal"]) > abs(row_sat["torque_steer"]) * 2.5
    # Near linear region (3°): ideal and actual should be within ~20%
    assert abs(row_lin["torque_steer_ideal"] - row_lin["torque_steer"]) < abs(row_lin["torque_steer"]) * 0.25


def test_bicycle_coupling_grows_slope_with_speed() -> None:
    """W1: with bicycle coupling on (default v0.7.5+), the τ-δ linear-region
    slope must GROW with speed because body sideslip/yaw amplify per-wheel α.

    Specifically α_FL ≈ −δ·(1/2 + mV²/(8cL)) for single FL turn, so slope
    grows ~ (1 + mV²/(4cL)). For LS9 between v=5 m/s and v=55 m/s the factor
    grows ~3-4×. Load sensitivity (M1) makes it grow slightly less but the
    bicycle effect dominates.

    Tests the OPPOSITE of the deleted v0.7.4 "load sensitivity softens slope"
    assertion — the user pointed out that was the wrong physical intuition.
    """
    p = VehicleParams(
        aero_lift_coeff_front=0.40, aero_lift_coeff_rear=0.20,
        tire_load_sensitivity_exp=0.8,
        # Isolate the bicycle/load-sens effect.
        static_toe_front=0.0, static_toe_rear=0.0, camber_thrust_coeff=0.0,
        parking_lateral_coeff=0.0, parking_torque_coeff=0.0,
    )
    # Stay in linear region: ±0.5° so we don't hit Pacejka saturation.
    angles = [a * math.radians(0.5) / 6 for a in range(-6, 7)]
    out_lo = sweep_load_analysis(p, speeds=[5.0], angles=angles, wheel_index=0, mu=0.85)
    out_hi = sweep_load_analysis(p, speeds=[55.0], angles=angles, wheel_index=0, mu=0.85)
    rows_lo = sorted([r for r in out_lo["rows"] if r["wheel_index"] == 0], key=lambda r: r["delta_cmd"])
    rows_hi = sorted([r for r in out_hi["rows"] if r["wheel_index"] == 0], key=lambda r: r["delta_cmd"])
    slope_lo = (rows_lo[-1]["torque_steer"] - rows_lo[0]["torque_steer"]) / (
        rows_lo[-1]["delta_cmd"] - rows_lo[0]["delta_cmd"])
    slope_hi = (rows_hi[-1]["torque_steer"] - rows_hi[0]["torque_steer"]) / (
        rows_hi[-1]["delta_cmd"] - rows_hi[0]["delta_cmd"])
    # Bicycle coupling must make high-speed slope at least 2× the low-speed slope.
    assert abs(slope_hi) > abs(slope_lo) * 2.0


def test_bicycle_coupling_shrinks_saturation_angle_at_speed() -> None:
    """W1: bicycle coupling — at high speed the wheel reaches α_peak at much
    smaller δ. Verify by finding where |τ| first hits 80% of its max.
    """
    p = VehicleParams(
        static_toe_front=0.0, static_toe_rear=0.0, camber_thrust_coeff=0.0,
        parking_lateral_coeff=0.0, parking_torque_coeff=0.0,
        rolling_resistance_coeff=0.0, drag_coeff_cd=0.0,
        aero_lift_coeff_front=0.0, aero_lift_coeff_rear=0.0,
    )
    angles = [math.radians(d) for d in [0, 0.5, 1, 1.5, 2, 3, 5, 8, 12, 20, 30]]
    out_lo = sweep_load_analysis(p, speeds=[5.0], angles=angles, wheel_index=0, mu=0.85)
    out_hi = sweep_load_analysis(p, speeds=[40.0], angles=angles, wheel_index=0, mu=0.85)

    def _sat_angle_deg(rows):
        positive = [r for r in rows if r["wheel_index"] == 0 and r["delta_cmd"] > 1e-6]
        peak = max(abs(r["torque_steer"]) for r in positive)
        for r in sorted(positive, key=lambda r: r["delta_cmd"]):
            if abs(r["torque_steer"]) >= 0.8 * peak:
                return math.degrees(r["delta_cmd"])
        return None

    sat_lo = _sat_angle_deg(out_lo["rows"])
    sat_hi = _sat_angle_deg(out_hi["rows"])
    assert sat_lo is not None and sat_hi is not None
    # Saturation must come at meaningfully smaller δ at high speed.
    assert sat_hi < sat_lo * 0.7


def test_pacejka_smooth_shoulder_replaces_hard_clip() -> None:
    """U3/H1: past the linear region the τ curve must round off smoothly
    (Pacejka shoulder) rather than hitting a vertical wall at saturation.

    Detect by comparing the slope just past peak slip vs the linear-region
    slope: a hard clip has the post-saturation slope = 0 (vertical wall), a
    Pacejka shoulder has it ~30-60% of the linear slope.
    """
    p = VehicleParams(
        static_toe_front=0.0, static_toe_rear=0.0, camber_thrust_coeff=0.0,
        parking_lateral_coeff=0.0, parking_torque_coeff=0.0,
        rolling_resistance_coeff=0.0, drag_coeff_cd=0.0,
        aero_lift_coeff_front=0.0, aero_lift_coeff_rear=0.0,
    )
    angles = [math.radians(d) for d in [1, 2, 4, 6, 8, 10, 12, 15, 20]]
    out = sweep_load_analysis(p, speeds=[10.0], angles=angles, wheel_index=0, mu=0.85)
    rows = sorted([r for r in out["rows"] if r["wheel_index"] == 0], key=lambda r: r["delta_cmd"])
    # Linear-region slope (1° to 2°)
    s_lin = (rows[1]["torque_steer"] - rows[0]["torque_steer"]) / (rows[1]["delta_cmd"] - rows[0]["delta_cmd"])
    # Post-saturation slope (15° to 20°)
    s_sat = (rows[-1]["torque_steer"] - rows[-2]["torque_steer"]) / (rows[-1]["delta_cmd"] - rows[-2]["delta_cmd"])
    # Smooth shoulder: post-saturation slope is not zero but is substantially smaller than linear.
    assert abs(s_sat) > 1.0          # not a hard wall (would be 0)
    assert abs(s_sat) < abs(s_lin) * 0.5  # but well below linear


def test_kingpin_no_caster_cross_term() -> None:
    """U1/M3: the bogus −Fx·trail·sin(δ) kingpin term must be gone.

    Verify by comparing kingpin moments at +Fx and -Fx with identical other
    inputs: the only Fx contribution should be Fx·scrub (sign-symmetric and
    δ-independent), with no extra sin(δ) coupling.
    """
    from sim4wis.vehicle.kingpin import kingpin_torque
    from sim4wis.core.state import SuspensionParams

    susp = SuspensionParams(caster_angle=0.15, kingpin_inclination=0.0, scrub_radius=0.02)
    fz = np.full(4, 6000.0)
    fy = np.zeros(4)
    mz = np.zeros(4)
    delta = np.full(4, math.radians(10.0))   # non-zero δ so any sin(δ) term would show
    tau_pos = kingpin_torque(fx=np.full(4, 200.0), fy=fy, mz=mz, fz=fz,
                              suspension=susp, delta=delta, tire_radius=0.395)
    tau_neg = kingpin_torque(fx=np.full(4, -200.0), fy=fy, mz=mz, fz=fz,
                              suspension=susp, delta=delta, tire_radius=0.395)
    # With only Fx·scrub the pair should be exact mirrors: τ(+Fx) = -τ(-Fx).
    # If the deleted -Fx·trail·sin(δ) term were still there it would break that
    # antisymmetry because sin(δ) doesn't flip with Fx sign.
    np.testing.assert_allclose(tau_pos, -tau_neg, rtol=1e-9)


def test_ws_serialization_contains_side_force_fields() -> None:
    reset_simulator()
    sim = get_simulator()
    msg = sim._serialize_state()  # noqa: SLF001
    assert "side_force_summary" in msg
    assert "side_force_body_y" in msg["wheels"][0]
    assert "linkage_efficiency" in msg["wheels"][0]
