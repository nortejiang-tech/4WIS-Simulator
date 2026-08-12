"""Tests — on-centre analysis and the procedure metric tier.

Two themes. The first is **refusal**: a weave metric extracted from a run that
was not a weave, or that had no steering plant, or that was really a handling
manoeuvre, must raise rather than return a number. An on-centre gradient
computed from a straight line comes back as an excellent result.

The second is **estimator honesty**. Several of these check the analysis
against a synthetic signal with a known answer, because that is the only way to
tell a model defect from an analysis defect — and this module's development
produced one of each. The phase estimator reported 26° of lag on a signal that
had 2°, which sent me looking for a plant defect that did not exist; the cause
was the excitation frequency, which the analysis now measures rather than
assumes.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from sim4wis.experiment.schema import Experiment
from sim4wis.experiment.session import SimSession
from sim4wis.study import metrics as metrics_mod
from sim4wis.study import oncentre as oc
from sim4wis.study.metrics import PROCEDURE, collect

# ---------------------------------------------------------------------------
# Synthetic — a known answer
# ---------------------------------------------------------------------------


def _sine(f=0.2, lead=3.0, span=25.0, dt=0.02, amp=1.0, lag_deg=0.0):
    t = np.arange(0.0, lead + span, dt)
    phase = 2 * math.pi * f * (t - lead) - math.radians(lag_deg)
    x = np.where(t < lead, 0.0, amp * np.sin(phase))
    return t, x


class TestEstimators:
    @pytest.mark.parametrize(("freq", "lead"), [(0.2, 3.0), (0.2, 0.0), (0.5, 5.0)])
    def test_frequency_is_measured_from_the_crossings_not_the_record(self, freq, lead):
        """A settling lead-in is dead time, not part of the period.

        Dividing by the whole span read 0.197 Hz for a 0.200 Hz weave with a
        3 s lead-in, and that error propagates straight into the phase.
        """
        t, x = _sine(f=freq, lead=lead)
        _, got = oc._cycles_and_frequency(t, x)
        assert got == pytest.approx(freq, rel=1e-3)

    @pytest.mark.parametrize("lag", [0.0, 5.0, 15.0, 30.0])
    def test_phase_lag_recovers_a_known_lag(self, lag):
        t, drive = _sine()
        _, resp = _sine(amp=0.7, lag_deg=lag)
        keep = t >= 3.0
        got = oc._phase_lag_deg(t[keep], drive[keep], resp[keep], 0.2)
        assert got == pytest.approx(lag, abs=0.05)

    def test_a_pure_ellipse_has_no_deadband_offset(self):
        # Torque in quadrature with angle is a lag, not friction: the loop is
        # open but symmetric, so both branches cross zero torque at the same
        # |angle| and the widths are what they are — the point is that the
        # measurement is taken from crossings and needs no window.
        t, angle = _sine()
        _, torque = _sine(amp=1.0, lag_deg=90.0)
        keep = t >= 3.0
        t, angle, torque = t[keep], angle[keep], torque[keep]
        rate = np.gradient(angle, t)
        width = oc._loop_width(angle, torque, rate, "test")
        assert width == pytest.approx(2.0, abs=0.05)   # ±1 at the crossings


# ---------------------------------------------------------------------------
# Refusals
# ---------------------------------------------------------------------------


def _weave_run(amp_deg=0.4, freq=0.2, kmh=100.0, plant=True, cycles=6, hz=50):
    exp = Experiment.model_validate({
        "name": "weave", "strategy": "ideal_ackermann",
        "model_type": "simplified_dynamic", "record_hz": hz,
        "vehicle": {"overrides": {"steering_system": {"enabled": plant}}},
        "maneuver": {"steps": [
            {"duration": 4.0, "speed_kmh": kmh,
             "steer": {"kind": "constant", "amplitude": 0.0}},
            {"duration": cycles / freq, "speed_kmh": kmh,
             "steer": {"kind": "sine", "amplitude": amp_deg,
                       "unit": "front_deg", "freq_hz": freq}},
        ]},
    })
    r = SimSession(exp).run()
    return np.asarray(r.t, float), {k: np.asarray(v, float) for k, v in r.channels.items()}


def _analyse(t, c):
    return oc.analyse_weave(
        t, hand_angle=c["steer_hand_angle"], hand_torque=c["steer_hand_torque"],
        plant_active=c["steer_plant_active"], ay=c["ay"], yaw_rate=c["yaw_rate"],
    )


class TestRefusals:
    def test_a_run_without_the_plant_is_refused_not_answered(self):
        """The channel exists and is NaN; that must not become a good result."""
        t, c = _weave_run(plant=False)
        with pytest.raises(oc.ProcedureError, match="未在本次运行中启用|没有转向系统通道"):
            _analyse(t, c)

    def test_a_straight_line_run_is_refused(self):
        exp = Experiment.model_validate({
            "name": "straight", "strategy": "ideal_ackermann",
            "model_type": "simplified_dynamic", "record_hz": 50,
            "vehicle": {"overrides": {"steering_system": {"enabled": True}}},
            "maneuver": {"steps": [{"duration": 20.0, "speed_kmh": 100,
                                    "steer": {"kind": "constant", "amplitude": 0.0}}]},
        })
        r = SimSession(exp).run()
        c = {k: np.asarray(v, float) for k, v in r.channels.items()}
        with pytest.raises(oc.ProcedureError):
            _analyse(np.asarray(r.t, float), c)

    def test_a_handling_manoeuvre_is_refused_as_an_on_centre_test(self):
        """Past ~0.35 g this stops being an on-centre test.

        Not a nicety: at 0.84 g the fitted torque gradient came out negative —
        a car that gets easier to turn the harder it corners — because the
        tyres had saturated and the analysis answered anyway.
        """
        t, c = _weave_run(amp_deg=4.0)
        with pytest.raises(oc.ProcedureError, match="超出中心区范围"):
            _analyse(t, c)

    def test_too_few_cycles_is_refused(self):
        t, c = _weave_run(cycles=1)
        with pytest.raises(oc.ProcedureError, match="周期"):
            _analyse(t, c)


# ---------------------------------------------------------------------------
# What the analysis actually says
# ---------------------------------------------------------------------------


class TestWeaveResult:
    def test_the_lateral_quantities_are_robust_to_amplitude(self):
        """The reason the shipped targets judge on N·m/g rather than N·m/deg.

        Over the usable amplitude range the ay-domain gradient moves by a few
        percent while the angle-domain one nearly halves.
        """
        small = _analyse(*_weave_run(amp_deg=0.25))
        large = _analyse(*_weave_run(amp_deg=0.45))
        assert small.ay_amplitude < large.ay_amplitude
        ratio = large.torque_gradient_nm_per_g / small.torque_gradient_nm_per_g
        assert 0.9 < ratio < 1.1, f"ay-domain gradient moved by {ratio:.2f}x"
        angle_ratio = large.torque_gradient_nm_per_rad / small.torque_gradient_nm_per_rad
        assert angle_ratio < 0.85, (
            "the angle-domain gradient used to be amplitude-independent? "
            f"ratio={angle_ratio:.2f} — if this now holds, the targets can move back"
        )

    def test_the_measured_frequency_matches_the_excitation(self):
        for f in (0.15, 0.2, 0.4):
            w = _analyse(*_weave_run(freq=f))
            assert w.frequency_hz == pytest.approx(f, rel=0.05)

    def test_yaw_lag_grows_with_excitation_frequency(self):
        slow = _analyse(*_weave_run(freq=0.2))
        fast = _analyse(*_weave_run(freq=0.8))
        assert 0.0 < slow.yaw_phase_lag_deg < fast.yaw_phase_lag_deg

    def test_the_loop_width_is_not_monotone_in_rack_friction(self):
        """A finding, pinned rather than hidden — delete this when it is fixed.

        The loop width reads naturally as "friction feel", and on this model it
        is not: raising rack Coulomb friction from 0 to 500 N makes the loop
        *narrower*, and it is already ~1 N·m with friction set to zero. At
        0.2 Hz the vehicle's own lateral-dynamics lag contributes a quadrature
        term of the same order and the opposite sign, and the two partly
        cancel.

        Both effects are real and a real weave contains both. What does not
        follow is writing a friction requirement against this number, which is
        why the shipped target set records it as `should`. If this test ever
        fails because the trend became monotone, the target can be promoted and
        this test deleted.
        """
        def width(coulomb):
            exp = Experiment.model_validate({
                "name": "w", "strategy": "ideal_ackermann",
                "model_type": "simplified_dynamic", "record_hz": 50,
                "vehicle": {"overrides": {"steering_system": {
                    "enabled": True, "rack": {"coulomb_friction_n": coulomb}}}},
                "maneuver": {"steps": [
                    {"duration": 4.0, "speed_kmh": 100,
                     "steer": {"kind": "constant", "amplitude": 0.0}},
                    {"duration": 30.0, "speed_kmh": 100,
                     "steer": {"kind": "sine", "amplitude": 0.4,
                               "unit": "front_deg", "freq_hz": 0.2}}]},
            })
            r = SimSession(exp).run()
            c = {k: np.asarray(v, float) for k, v in r.channels.items()}
            return _analyse(np.asarray(r.t, float), c).torque_hysteresis_nm

        frictionless, mid = width(0.0), width(500.0)
        assert frictionless > 0.5, (
            "a frictionless rack now shows a narrow loop — the vehicle-lag "
            "contribution may have been separated out; re-check the targets"
        )
        assert mid < frictionless, (
            f"the loop width became monotone in friction ({frictionless:.3f} -> "
            f"{mid:.3f}); promote onc_torque_hysteresis_nm to a judged target "
            "and delete this test"
        )

    def test_an_unassisted_heavy_vehicle_cannot_be_weaved_at_speed(self):
        """A refusal that is the right answer, not a failure.

        With the assist map set to `none` this 2.9 t SUV needs ~120 N·m at the
        pinion to hold 0.4 deg at 100 km/h; the driver's 25 N·m limit stops the
        wheel moving, so there are no cycles to analyse and the analysis says
        so rather than reporting a superb on-centre result from a flat line.
        """
        exp = Experiment.model_validate({
            "name": "w", "strategy": "ideal_ackermann",
            "model_type": "simplified_dynamic", "record_hz": 50,
            "vehicle": {"overrides": {"steering_system": {
                "enabled": True, "assist_map": "none"}}},
            "maneuver": {"steps": [
                {"duration": 4.0, "speed_kmh": 100,
                 "steer": {"kind": "constant", "amplitude": 0.0}},
                {"duration": 30.0, "speed_kmh": 100,
                 "steer": {"kind": "sine", "amplitude": 0.4,
                           "unit": "front_deg", "freq_hz": 0.2}}]},
        })
        run = SimSession(exp).run()
        c = {k: np.asarray(v, float) for k, v in run.channels.items()}
        with pytest.raises(oc.ProcedureError, match="周期|幅值"):
            _analyse(np.asarray(run.t, float), c)


# ---------------------------------------------------------------------------
# The metric tier
# ---------------------------------------------------------------------------


class TestProcedureTier:
    def test_one_analysis_serves_every_metric_of_that_run(self):
        # Ten metrics, one pass over the data — otherwise two metrics of the
        # same run could disagree about what the run did.
        calls = {"n": 0}

        real = metrics_mod.ANALYSES["weave"]

        def counting(t, ch):
            calls["n"] += 1
            return real(t, ch)

        metrics_mod.ANALYSES["weave"] = counting
        try:
            t, c = _weave_run()
            names = sorted(PROCEDURE)
            out = collect(names, [], {}, list(t), {k: list(v) for k, v in c.items()})
        finally:
            metrics_mod.ANALYSES["weave"] = real
        assert calls["n"] == 1
        assert not out.errors and len(out.values) == len(names)

    def test_a_refusal_is_recorded_per_metric_not_raised(self):
        t, c = _weave_run(plant=False)
        out = collect(sorted(PROCEDURE), [], {}, list(t), {k: list(v) for k, v in c.items()})
        assert not out.values
        assert all("ProcedureError" in e for e in out.errors.values())

    def test_procedure_metrics_are_advertised(self):
        from sim4wis.study.metrics import describe_metrics

        by_name = {m["name"]: m for m in describe_metrics()}
        assert by_name["onc_torque_gradient_nm_per_g"]["source"] == "procedure"
        assert by_name["onc_torque_gradient_nm_per_g"]["requires"] == "steering_plant"
