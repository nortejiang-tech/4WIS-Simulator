"""Tests — measured K&C suspension characteristic.

Each mechanism is isolated: a test that turns on the whole table at once can
only tell you "something changed". These turn on one curve at a time and check
the vehicle moved by the amount the table says it should.

Why this matters at all: before K&C the suspension was rigid with a constant
camber, so lateral load could not deflect a wheel and the outer wheel kept its
static camber right through a roll. That left no way to express the mechanism
chassis engineers actually tune, and the only route to a production-like
understeer gradient was to back out an axle cornering-stiffness ratio — fitting
a number instead of modelling a mechanism.
"""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from sim4wis.controller.longitudinal import apply_brake_command, apply_drive_command
from sim4wis.controller.registry import make_strategy
from sim4wis.core.derived import update_derived_outputs
from sim4wis.core.state import DriverInput, EnvironmentState, VehicleParams
from sim4wis.vehicle.dynamic import SimplifiedDynamicModel
from sim4wis.vehicle.kc import (
    AxleKC,
    ComplianceCoeffs,
    KinematicCurve,
    VehicleKC,
    axle_from_dict,
    kc_from_bump_steer_coeff,
    kc_from_dict,
    load_kc_profile,
)
from sim4wis.vehicle.multibody import MultiBodyModel

DT = 0.002
MU = 0.90
ROOT = Path(__file__).resolve().parents[2]
DEG = math.pi / 180.0


# ─── helpers ────────────────────────────────────────────────────────────────

def _toe_only(front_deg_per_kN: float = 0.0, rear_deg_per_kN: float = 0.0) -> VehicleKC:
    """Compliance-steer only — no kinematic curves, so the mechanism is alone."""
    return VehicleKC(
        front=AxleKC(compliance=ComplianceCoeffs(
            toe_per_fy=front_deg_per_kN * DEG / 1000.0)),
        rear=AxleKC(compliance=ComplianceCoeffs(
            toe_per_fy=rear_deg_per_kN * DEG / 1000.0)),
    )


def _camber_only(gain_deg_per_100mm: float) -> VehicleKC:
    """Camber gain only, as a straight line through the origin."""
    j = (-0.10, 0.0, 0.10)
    v = (gain_deg_per_100mm * DEG, 0.0, -gain_deg_per_100mm * DEG)
    curve = KinematicCurve(jounce_m=j, value=v)
    axle = AxleKC(camber=curve)
    return VehicleKC(front=axle, rear=axle)


def _run(kc, delta_deg, v_kmh, model="multibody", secs=14.0, **over):
    p = replace(VehicleParams(), kc=kc, longitudinal_mode="torque", **over)
    m = (MultiBodyModel if model == "multibody" else SimplifiedDynamicModel)(p)
    m.reset()
    strat = make_strategy("rear_wheel_steer", p)
    d = DriverInput(gear=1, mode_params={
        "rws_mode": "fixed_ratio", "rear_ratio": 0.0,
        "steer_raw_rad": math.radians(delta_deg),
        "speed_target_ms": v_kmh / 3.6})
    env = EnvironmentState(mu=MU)
    for _ in range(int(secs / DT)):
        c = strat.compute(d, m.state, DT)
        apply_brake_command(c, d, p)
        apply_drive_command(c, d, p, m.state)
        m.step(DT, c, env)
        update_derived_outputs(m.state, p)
    return m


def _understeer_gradient(kc, model="multibody") -> float:
    L = VehicleParams().wheelbase
    rows = []
    for v in (30, 45, 60, 75, 90):
        m = _run(kc, 2.0, v, model)
        s = m.state
        if abs(s.ay) < 0.05:
            continue
        rows.append((abs(s.ay), abs(float(np.mean(s.delta[:2]))), s.vx))
    ay = np.array([r[0] for r in rows])
    dl = np.array([r[1] for r in rows])
    R = np.array([r[2] ** 2 / r[0] for r in rows])
    return float(np.polyfit(ay, dl - L / R, 1)[0]) * (180 / math.pi) * 9.81


# ─── 1. data model + units ──────────────────────────────────────────────────

def test_empty_kc_is_the_rigid_default():
    kc = VehicleKC()
    assert kc.is_empty
    j = np.zeros(4)
    assert np.allclose(kc.per_wheel_toe(j), 0.0)
    assert np.allclose(kc.per_wheel_camber(j), 0.0)


def test_engineering_units_convert_on_load():
    """Files are written in deg/mm/kN because that is what rig reports use;
    nothing downstream may see a degree."""
    kc = kc_from_dict({
        "front": {
            "kinematics": {"jounce_mm": [-50, 0, 50], "toe_deg": [0.5, 0.0, -0.5]},
            "compliance": {"toe_per_fy_deg_per_kN": -0.1},
        }
    })
    # 0.5 deg at -50 mm
    assert kc.front.toe.at(np.array([-0.05]))[0] == pytest.approx(0.5 * DEG)
    # -0.1 deg/kN = -0.1 deg per 1000 N
    assert kc.front.compliance.toe_per_fy == pytest.approx(-0.1 * DEG / 1000.0)


def test_curve_interpolates_and_extrapolates_flat():
    """Linear between measured points; FLAT outside them. Extrapolating a curve
    that was about to turn over would invent behaviour the rig never saw."""
    c = KinematicCurve(jounce_m=(-0.05, 0.0, 0.05), value=(1.0, 0.0, -1.0))
    assert c.at(np.array([-0.025]))[0] == pytest.approx(0.5)
    assert c.at(np.array([0.0]))[0] == pytest.approx(0.0)
    assert c.at(np.array([-0.50]))[0] == pytest.approx(1.0), "flat, not extrapolated"
    assert c.at(np.array([+0.50]))[0] == pytest.approx(-1.0)


def test_mismatched_curve_lengths_are_rejected():
    with pytest.raises(ValueError, match="paired"):
        axle_from_dict({"kinematics": {"jounce_mm": [0, 50], "toe_deg": [0.0]}})


def test_kinematic_toe_mirrors_left_right_like_static_toe():
    """The curve is axle-level toe-IN; the per-wheel split must match the
    static-toe convention (left −, right +) so the two simply add."""
    kc = kc_from_dict({"front": {"kinematics": {
        "jounce_mm": [-50, 0, 50], "toe_deg": [1.0, 0.0, -1.0]}}})
    toe = kc.per_wheel_toe(np.array([-0.05, -0.05, 0.0, 0.0]))
    assert toe[0] == pytest.approx(-1.0 * DEG)
    assert toe[1] == pytest.approx(+1.0 * DEG)


def test_compliance_is_not_mirrored():
    """Fy is already signed per wheel in the wheel frame — a lateral force
    pointing left deflects the wheel the same way on either side."""
    kc = _toe_only(front_deg_per_kN=-0.1)
    z = np.zeros(4)
    fy = np.array([1000.0, 1000.0, 0.0, 0.0])
    toe = kc.per_wheel_compliance_toe(z, fy, z)
    assert toe[0] == pytest.approx(toe[1])
    assert toe[0] == pytest.approx(-0.1 * DEG)


def test_legacy_bump_steer_coeff_becomes_a_curve():
    """The old single number keeps working through the same code path."""
    kc = kc_from_bump_steer_coeff(2.5)
    assert not kc.is_empty
    # 2.5 rad per metre of travel → 0.05 rad at 20 mm
    assert kc.front.toe.at(np.array([0.02]))[0] == pytest.approx(0.05, rel=1e-9)
    assert kc_from_bump_steer_coeff(0.0).is_empty


def test_shipped_estimate_profile_loads():
    kc = load_kc_profile(ROOT / "kc_profiles" / "ls9_estimate.yaml")
    assert not kc.is_empty
    assert not kc.front.compliance.is_rigid
    assert not kc.rear.compliance.is_rigid


# ─── 2. mechanisms, one at a time, on a running vehicle ─────────────────────

@pytest.mark.parametrize("model", ["multibody", "dynamic"])
def test_compliance_steer_matches_the_coefficient(model):
    """Isolated: only front compliance-steer is on. The toe the model applies
    must equal C·Fy for the force the tyre is actually carrying."""
    c_deg_per_kn = -0.20
    kc = _toe_only(front_deg_per_kN=c_deg_per_kn)
    m = _run(kc, 4.0, 70, model)
    fy_front = float(np.mean(m.tire_fy[:2]))
    applied = float(np.mean(m.kc_toe[:2]))
    expected = c_deg_per_kn * DEG * fy_front / 1000.0
    assert applied == pytest.approx(expected, rel=0.02)


@pytest.mark.parametrize("model", ["multibody", "dynamic"])
def test_compliance_steer_moves_the_understeer_gradient(model):
    """The mechanism this whole module exists for: front toe-out under lateral
    force pushes the car toward understeer, rear toe-in likewise. Before K&C
    there was no way to express this at all."""
    base = _understeer_gradient(None, model)
    pushy = _understeer_gradient(_toe_only(front_deg_per_kN=-0.15,
                                           rear_deg_per_kN=+0.08), model)
    loose = _understeer_gradient(_toe_only(front_deg_per_kN=+0.15,
                                           rear_deg_per_kN=-0.08), model)
    assert pushy > base + 0.2, "front toe-out under Fy must add understeer"
    assert loose < base - 0.2, "the opposite compliance must remove it"


def test_camber_gain_changes_cornering_force_but_not_the_limit():
    """Camber was a CONSTANT before this — the outer wheel kept its static
    angle right through a roll.

    Note what the model can and cannot express. Camber enters as an equivalent
    slip-angle offset, so it shifts the force curve; the friction circle still
    caps the resultant at μ·Fz whatever the camber is. So camber gain moves the
    LINEAR-range force and leaves the limit essentially untouched. A real tyre
    also loses peak μ at large camber — that is not modelled, and this test
    pins the boundary so nobody reads more into the camber curve than is there.
    """
    mid_flat = _run(None, 2.0, 70, "multibody")
    mid_gain = _run(_camber_only(3.0), 2.0, 70, "multibody")
    fy_flat = float(np.sum(np.abs(mid_flat.tire_fy)))
    fy_gain = float(np.sum(np.abs(mid_gain.tire_fy)))
    assert not math.isclose(fy_flat, fy_gain, rel_tol=1e-4), (
        "camber gain must change the cornering force it develops"
    )

    def limit_ay(kc):
        return max(abs(_run(kc, d, 70, "multibody").state.ay) for d in (8, 10, 13))
    # …and the limit does NOT move, because the circle caps it either way.
    assert limit_ay(None) == pytest.approx(limit_ay(_camber_only(3.0)), rel=0.02)


@pytest.mark.parametrize("model", ["multibody", "dynamic"])
def test_antisymmetric_jounce_produces_roll_steer(model):
    """A toe-vs-jounce curve under roll gives ROLL STEER — the axle steers as a
    unit — not a toe change.

    One side extends and toes in, the other compresses and toes out by the same
    amount; under the left−/right+ mirroring those compose into a common steer
    angle. That is the real mechanism (and one of the main handling-balance
    levers on a live axle), so the two wheels must come out with the SAME sign.
    """
    kc = kc_from_dict({
        "front": {"kinematics": {"jounce_mm": [-100, 0, 100],
                                 "toe_deg": [2.0, 0.0, -2.0]}},
        "rear": {"kinematics": {"jounce_mm": [-100, 0, 100],
                                "toe_deg": [0.0, 0.0, 0.0]}},
    })
    m = _run(kc, 6.0, 70, model)
    assert m.kc_toe[0] * m.kc_toe[1] > 0.0, "roll steer: both front wheels one way"
    assert np.max(np.abs(m.kc_toe[:2])) > 1e-4
    assert np.allclose(m.kc_toe[2:], 0.0), "rear curve was flat"


def test_symmetric_jounce_produces_a_true_toe_change():
    """The other half of the same curve: when both wheels move together (dive,
    squat, a two-wheel bump) the mirroring gives opposite per-wheel angles —
    an actual toe change rather than a steer."""
    kc = kc_from_dict({"front": {"kinematics": {
        "jounce_mm": [-100, 0, 100], "toe_deg": [2.0, 0.0, -2.0]}}})
    toe = kc.per_wheel_toe(np.array([0.05, 0.05, 0.0, 0.0]))
    assert toe[0] * toe[1] < 0.0
    assert toe[0] == pytest.approx(-toe[1])


def test_both_models_agree_on_the_same_table():
    """One measurement must drive both models — otherwise the table is not a
    property of the suspension, it is a property of whichever model read it."""
    kc = load_kc_profile(ROOT / "kc_profiles" / "ls9_estimate.yaml")
    k_mb = _understeer_gradient(kc, "multibody")
    k_dy = _understeer_gradient(kc, "dynamic")
    assert k_mb == pytest.approx(k_dy, abs=0.15)


# ─── 3. off means off ───────────────────────────────────────────────────────

@pytest.mark.parametrize("model", ["multibody", "dynamic"])
def test_no_table_means_no_change(model):
    """The whole migration rests on this: with no K&C data the models must
    behave exactly as they did before the feature existed."""
    m = _run(None, 4.0, 70, model)
    assert np.allclose(m.kc_toe, 0.0)
    assert np.allclose(m.kc_camber, 0.0)


def test_kc_survives_extreme_travel_without_blowing_up():
    kc = load_kc_profile(ROOT / "kc_profiles" / "ls9_estimate.yaml")
    m = _run(kc, 10.0, 90, "multibody", secs=8.0)
    for arr in (m.state.delta, m.kc_toe, m.kc_camber, m.state.fz):
        assert np.all(np.isfinite(arr))
