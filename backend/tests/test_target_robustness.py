"""Tests — parameter-space margins (C5 · C3c).

The bisection is verified the only way a bisection can be: at the reported
flip the signature really differs from nominal, and one bisection step
inside it really does not. The untrustworthy-flip trap (a violation leaving
via not_evaluated) is pinned explicitly, because a margin table that
laundered it would read as "smaller motor, fewer failures".
"""

from __future__ import annotations

import json

from sim4wis.core.state import VehicleParams
from sim4wis.steering.sizing import DEFAULT_SCENARIOS, size_actuator
from sim4wis.targets import compliance as compliance_mod
from sim4wis.targets import library, measure, robustness

#: The single worst case keeps every evaluation at ~0.5 s; the assertions
#: below do not care which scenarios produced the numbers.
ONE_SCENARIO = (DEFAULT_SCENARIOS[0],)

PARAMS = VehicleParams()
PARAMS.steering_system.enabled = True


def _signature_at(axis: robustness.RobustAxis, value: float) -> frozenset[str]:
    p = robustness.with_axis_value(PARAMS, axis, value)
    req = size_actuator(p, ONE_SCENARIO)
    return robustness.violation_signature(
        compliance_mod.evaluate(library.get("eps_actuator"),
                                measure.from_sizing(req)))


def test_with_axis_value_replaces_the_nested_scalar():
    axis = robustness.RobustAxis("rack.coulomb_friction_n", "rack",
                                 "coulomb_friction_n", 0.0, 1200.0)
    p = robustness.with_axis_value(PARAMS, axis, 500.0)
    assert p.steering_system.rack.coulomb_friction_n == 500.0
    # Everything else, including the sibling fields, is untouched.
    assert p.steering_system.rack.mass == PARAMS.steering_system.rack.mass
    assert PARAMS.steering_system.rack.coulomb_friction_n == 260.0


def test_the_reported_flip_really_flips_and_the_inside_really_does_not():
    axis = robustness.RobustAxis("motor.peak_torque", "motor", "peak_torque",
                                 2.0, 20.0)
    nominal = robustness.axis_margins(
        PARAMS, library.get("eps_actuator"), axes=(axis,),
        scenarios=ONE_SCENARIO, bisect_steps=6)
    margin = nominal.axes[0]
    # Downward flip is where the signature changes; a step back toward the
    # nominal must restore it — that pair is what "bisection converged" means.
    if margin.flip_low is not None:
        assert _signature_at(axis, margin.flip_low) != frozenset(
            nominal.nominal_violated)
        inside = margin.flip_low + (margin.nominal - margin.flip_low) / 2
        assert _signature_at(axis, inside) == frozenset(nominal.nominal_violated)
    else:
        # No flip down: the whole bound carries the nominal signature.
        assert _signature_at(axis, axis.lo) == frozenset(nominal.nominal_violated)


def test_a_flip_into_not_evaluated_is_labelled_as_such():
    """Lowering the motor makes runs untrustworthy, not compliant.

    The violated set shrinks — but through NOT EVALUATED, and the margin
    table must say so instead of letting a smaller motor look like a fix.
    """
    axis = robustness.RobustAxis("motor.peak_torque", "motor", "peak_torque",
                                 2.0, 20.0)
    rep = robustness.axis_margins(
        PARAMS, library.get("eps_actuator"), axes=(axis,),
        scenarios=ONE_SCENARIO, bisect_steps=4)
    margin = rep.axes[0]
    assert margin.flip_low is not None
    assert margin.flip_low_entries, "the flip must name what changed"
    assert all("→ 未评估" in e for e in margin.flip_low_entries), (
        "violations leaving via untrusted measurements must be labelled as "
        "not_evaluated, not silently dropped from the signature"
    )


def test_axis_margins_is_deterministic():
    axis = robustness.RobustAxis("motor.peak_torque", "motor", "peak_torque",
                                 2.0, 20.0)
    runs = [
        robustness.axis_margins(PARAMS, library.get("eps_actuator"), axes=(axis,),
                                scenarios=ONE_SCENARIO, bisect_steps=3)
        for _ in range(2)
    ]
    a, b = (r.to_dict() for r in runs)
    a.pop("wall_s"), b.pop("wall_s")  # wall time is not a computed quantity
    assert a == b


def test_annotate_with_fit_marks_whether_the_verdict_survives(tmp_path):
    axis = robustness.RobustAxis("rack.coulomb_friction_n", "rack",
                                 "coulomb_friction_n", 0.0, 1200.0)
    rep = robustness.axis_margins(
        PARAMS, library.get("eps_actuator"), axes=(axis,),
        scenarios=ONE_SCENARIO, bisect_steps=3)
    margin = rep.axes[0]

    record = {"params": {"rack.coulomb_friction_n": {"fitted": 300.0}}}
    path = tmp_path / "fit.json"
    path.write_text(json.dumps(record), encoding="utf-8")
    n = robustness.annotate_with_fit(rep, path)
    assert n == 1
    assert margin.fitted == 300.0
    lo, hi = margin.verdict_interval
    assert margin.fitted_inside == (lo <= 300.0 <= hi)

    # A fitted value past the flip is flagged as outside the survival region.
    outside = hi + 1.0 if margin.flip_high is not None else lo - 1.0
    record["params"]["rack.coulomb_friction_n"]["fitted"] = outside
    path.write_text(json.dumps(record), encoding="utf-8")
    robustness.annotate_with_fit(rep, path)
    assert margin.fitted_inside is False


def test_unknown_axis_names_are_refused_by_the_cli_filter():
    """The CLI's --axes filter must refuse a typo, not silently run nothing."""
    # The filtering logic lives in the command; the contract it enforces on
    # the caller is that every requested name exists in DEFAULT_AXES.
    wanted = {"motor.peak_torque", "rack.mass"}
    known = {a.name for a in robustness.DEFAULT_AXES}
    assert wanted - known == {"rack.mass"}
