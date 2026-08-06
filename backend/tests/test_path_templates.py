"""Unit tests — standard-maneuver course geometry.

These guard the two properties that make a generated course *drivable*: the
reference line has to stay inside the cone lane it is supposed to thread, and
the ISO courses have to keep their published dimensions as the vehicle width
changes.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from sim4wis.controller.path import (
    _SPECS,
    Cone,
    clear_active_plan,
    list_templates,
    plan_from_template,
    plan_from_waypoints,
    template_specs,
)


@pytest.fixture(autouse=True)
def _clean_path():
    yield
    clear_active_plan()


VEHICLE = {"vehicle_width": 1.8, "vehicle_length": 4.6}


def _boxes(plan) -> list[tuple[float, float, float, float]]:
    """Painted section rectangles → (x0, x1, y_lo, y_hi)."""
    out = []
    for m in plan.marks:
        pts = np.asarray(m.points, dtype=float)
        if pts.shape[0] == 5 and np.allclose(pts[0], pts[-1]):
            out.append((pts[:, 0].min(), pts[:, 0].max(),
                        pts[:, 1].min(), pts[:, 1].max()))
    return out


# ---------------------------------------------------------------------------
# every template
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", list_templates())
def test_every_template_builds_a_usable_course(name: str) -> None:
    plan = plan_from_template(name, dict(VEHICLE))
    assert plan.name == name
    assert plan.label, "every template needs a human label for the panel"
    assert plan.notes, "notes carry the laid-out geometry into the UI"
    assert not plan.is_empty()
    assert len(plan.cones) > 0, "a maneuver you can't see is not a maneuver"
    assert all(isinstance(c, Cone) for c in plan.cones)
    assert all(c.kind in ("cone", "pole") for c in plan.cones)
    assert all(c.height > 0 for c in plan.cones)
    # Dense enough for pure pursuit: no gap larger than ~1 m.
    seg = np.linalg.norm(np.diff(plan.points, axis=0), axis=1)
    assert seg.max() < 1.0


@pytest.mark.parametrize("name", list_templates())
def test_serialize_round_trip(name: str) -> None:
    d = plan_from_template(name, dict(VEHICLE)).serialize()
    assert set(d) >= {"name", "label", "notes", "closed", "points", "cones", "marks"}
    assert d["cones"] and set(d["cones"][0]) == {"x", "y", "kind", "color", "height"}
    for m in d["marks"]:
        assert set(m) == {"points", "color", "width", "dash"}
        assert len(m["points"]) >= 2


def test_template_specs_cover_every_template() -> None:
    specs = {s["name"]: s for s in template_specs()}
    assert set(specs) == set(list_templates())
    for name, spec in specs.items():
        assert spec["label"] and spec["desc"]
        for p in spec["params"]:
            assert set(p) >= {"key", "label", "default", "min", "max", "step"}
            assert p["min"] <= p["default"] <= p["max"]
            # A spec'd knob must actually reach the generator.
            plan = plan_from_template(name, {**VEHICLE, p["key"]: p["default"]})
            assert not plan.is_empty()
        assert _SPECS[name] is not None


def test_unknown_template_raises() -> None:
    with pytest.raises(KeyError):
        plan_from_template("no_such_maneuver", {})


def test_vehicle_params_are_dropped_for_templates_that_ignore_them() -> None:
    """The REST layer always injects vehicle size; templates that don't take it
    must not blow up (that would break every script with `params: {}`)."""
    for name in list_templates():
        plan_from_template(name, dict(VEHICLE))


def test_waypoint_plan_has_no_furniture() -> None:
    plan = plan_from_waypoints([[0, 0], [10, 0], [20, 5]])
    assert plan.cones == [] and plan.marks == []
    assert plan.points.shape[0] > 2


# ---------------------------------------------------------------------------
# lane-change courses
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", ["double_lane_change", "iso3888_1", "lane_change"])
@pytest.mark.parametrize("width", [1.6, 1.8, 2.1])
def test_reference_line_stays_inside_every_cone_section(name: str, width: float) -> None:
    plan = plan_from_template(name, {"vehicle_width": width})
    boxes = _boxes(plan)
    assert boxes, "lane-change courses paint one box per cone section"
    for x0, x1, y_lo, y_hi in boxes:
        inside = plan.points[(plan.points[:, 0] >= x0) & (plan.points[:, 0] <= x1)]
        assert inside.shape[0] > 0, "the path must actually cross the section"
        assert inside[:, 1].min() > y_lo, f"{name}: path leaves section on the right"
        assert inside[:, 1].max() < y_hi, f"{name}: path leaves section on the left"
        # And with room for the body, not just the centreline.
        clear = min(inside[:, 1].min() - y_lo, y_hi - inside[:, 1].max())
        assert clear > width / 2.0 * 0.5


@pytest.mark.parametrize("width", [1.6, 1.8, 2.1])
def test_iso3888_2_matches_the_standard(width: float) -> None:
    """Sections 12 / 11 / 12 m, widths 1.1b+0.25, b+1, 1.3b+0.25, gaps 13.5 /
    12.5 m, and the 1 m edge-to-edge offset of the side lane."""
    plan = plan_from_template("double_lane_change", {"vehicle_width": width})
    boxes = sorted(_boxes(plan))
    assert len(boxes) == 3
    lengths = [round(x1 - x0, 3) for x0, x1, _, _ in boxes]
    widths = [round(y_hi - y_lo, 3) for _, _, y_lo, y_hi in boxes]
    assert lengths == [12.0, 11.0, 12.0]
    assert widths == pytest.approx([1.1 * width + 0.25, width + 1.0, 1.3 * width + 0.25])

    gaps = [round(boxes[1][0] - boxes[0][1], 3), round(boxes[2][0] - boxes[1][1], 3)]
    assert gaps == [13.5, 12.5]

    # Side lane sits 1 m clear of the entry lane, edge to edge.
    assert boxes[1][2] - boxes[0][3] == pytest.approx(1.0)
    # Entry and exit sections share the original lane centre.
    assert (boxes[0][2] + boxes[0][3]) / 2 == pytest.approx(0.0)
    assert (boxes[2][2] + boxes[2][3]) / 2 == pytest.approx(0.0)


def test_iso3888_1_matches_the_standard() -> None:
    plan = plan_from_template("iso3888_1", {"vehicle_width": 1.8})
    boxes = sorted(_boxes(plan))
    assert [round(x1 - x0, 3) for x0, x1, _, _ in boxes] == [15.0, 25.0, 15.0]
    assert round(boxes[1][0] - boxes[0][1], 3) == 30.0
    assert round(boxes[2][0] - boxes[1][1], 3) == 25.0
    assert (boxes[1][2] + boxes[1][3]) / 2 == pytest.approx(3.5)


def test_lane_change_sections_widen_with_the_vehicle() -> None:
    narrow = _boxes(plan_from_template("double_lane_change", {"vehicle_width": 1.6}))
    wide = _boxes(plan_from_template("double_lane_change", {"vehicle_width": 2.1}))
    for (_, _, ny0, ny1), (_, _, wy0, wy1) in zip(sorted(narrow), sorted(wide)):
        assert (wy1 - wy0) > (ny1 - ny0)


# ---------------------------------------------------------------------------
# slalom
# ---------------------------------------------------------------------------


def test_slalom_path_passes_beside_each_pole_not_through_it() -> None:
    plan = plan_from_template("slalom", {"n_gates": 6, "spacing": 18.0,
                                         "vehicle_width": 1.8})
    poles = [c for c in plan.cones if c.kind == "pole"]
    assert len(poles) == 6
    half_body = 1.8 / 2.0
    for pole in poles:
        d = float(np.hypot(plan.points[:, 0] - pole.x,
                           plan.points[:, 1] - pole.y).min())
        assert d > half_body + 0.5, "the reference line drives through a pole"


def test_slalom_alternates_sides() -> None:
    plan = plan_from_template("slalom", {"n_gates": 5, "spacing": 15.0})
    poles = sorted((c for c in plan.cones if c.kind == "pole"), key=lambda c: c.x)
    sides = []
    for pole in poles:
        i = int(np.argmin(np.abs(plan.points[:, 0] - pole.x)))
        sides.append(math.copysign(1.0, plan.points[i, 1]))
    assert all(a != b for a, b in zip(sides, sides[1:]))


def test_slalom_offset_defaults_to_body_clearance_and_can_be_pinned() -> None:
    auto = plan_from_template("slalom", {"vehicle_width": 2.0})
    assert abs(auto.points[:, 1]).max() == pytest.approx(2.0 / 2 + 1.2, abs=0.05)
    pinned = plan_from_template("slalom", {"vehicle_width": 2.0, "offset": 3.0})
    assert abs(pinned.points[:, 1]).max() == pytest.approx(3.0, abs=0.05)


# ---------------------------------------------------------------------------
# circles
# ---------------------------------------------------------------------------


def test_skidpad_is_a_closed_circle_of_the_requested_radius() -> None:
    r = 25.0
    plan = plan_from_template("skidpad", {"radius": r})
    assert plan.closed
    d = np.hypot(plan.points[:, 0], plan.points[:, 1] - r)
    assert d.max() == pytest.approx(r, abs=1e-6)
    assert d.min() == pytest.approx(r, abs=1e-6)
    # Cones ring both edges of the lane.
    cd = np.array([math.hypot(c.x, c.y - r) for c in plan.cones])
    assert (cd > r).any() and (cd < r).any()


def test_figure_eight_cones_sit_inside_both_loops() -> None:
    r = 15.0
    plan = plan_from_template("figure_eight", {"radius": r})
    assert plan.closed
    for centre in (+r, -r):
        ring = [c for c in plan.cones
                if abs(math.hypot(c.x, c.y - centre) - (r - 2.0)) < 1e-6]
        assert len(ring) > 6


def test_parking_slot_scales_with_the_vehicle() -> None:
    plan = plan_from_template("parking", {"vehicle_length": 4.6, "vehicle_width": 1.8,
                                          "margin": 1.5})
    box = _boxes(plan)[0]
    assert (box[1] - box[0]) == pytest.approx(4.6 + 1.5)
    assert (box[3] - box[2]) == pytest.approx(1.8 + 0.8)
    # The line ends inside the slot, not out in the lane.
    assert box[2] < plan.points[-1][1] < box[3]


def test_straight_gates_bookend_the_course() -> None:
    plan = plan_from_template("straight", {"length": 100.0, "gate_spacing": 25.0})
    xs = sorted({round(c.x, 3) for c in plan.cones})
    assert xs == [0.0, 25.0, 50.0, 75.0, 100.0]
    assert all(c.color == "#22c55e" for c in plan.cones if c.x == 0.0)
    assert all(c.color == "#dc2626" for c in plan.cones if c.x == 100.0)
