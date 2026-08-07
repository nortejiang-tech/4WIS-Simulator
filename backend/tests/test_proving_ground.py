"""Tests — proving-ground scenario + course anchoring (work-package C).

Guards the properties the plan §C5 lists:
  * multi-lane roads with the nominal lane width and standard dash rhythm,
  * exact corner radii (the point of having *labelled* curvature),
  * a long-enough straight for 0→100→0 runs,
  * multiple on-pavement spawn points,
  * named anchors that path templates land on, and that anchored courses
    actually transform (not still sit at the origin).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from sim4wis.controller.path import clear_active_plan, plan_from_template
from sim4wis.environment import scenario as scn


@pytest.fixture(autouse=True)
def _clean():
    yield
    scn.clear_scenario()
    clear_active_plan()


def _load_pg() -> scn.Scenario:
    scn.load_scenario("proving_ground")
    active = scn.get_active()
    assert active is not None
    return active


# ── 1. lane widths & count (GB 5768.3 default 3.75 m) ────────────────────────

def test_lane_widths_and_count():
    """The straight road has 3 lanes at the nominal 3.75 m width.

    The asphalt is 3 × lane_width = 11.25 m wide; the two solid edge lines sit
    at ±half that. Lane count = (edge separation / lane_width)."""
    pg = _load_pg()
    straight_edge_lines = [
        ln for ln in pg.lines if ln.color == scn.WHITE and ln.width == 0.20
        and len(ln.points) == 2 and abs(ln.points[0][1] - ln.points[1][1]) < 1e-9
        and ln.points[0][0] == 0.0
    ]
    assert len(straight_edge_lines) >= 2
    ys = sorted({round(p[1], 3) for ln in straight_edge_lines for p in ln.points})
    edge_sep = abs(ys[-1] - ys[0])
    lane_width = 3.75
    n_lanes_expected = 3
    assert edge_sep == pytest.approx(n_lanes_expected * lane_width, abs=0.02)
    assert edge_sep / lane_width == pytest.approx(n_lanes_expected, abs=0.01)


def test_dash_pattern_serialized():
    """Lines with a dash_pattern serialise the (on, off) rhythm and the 2D/3D
    front-ends can read it — the whole point of carrying it as data."""
    pg = _load_pg()
    ser = pg.serialize()
    dp_lines = [ln for ln in ser["lines"] if ln.get("dash_pattern")]
    assert dp_lines, "expected dashed lane dividers with dash_pattern"
    # GB 5768.3 motorway rhythm: 6 m mark / 9 m gap
    for ln in dp_lines:
        on, off = ln["dash_pattern"]
        assert (on, off) == (6.0, 9.0) or (on, off) == (3.0, 3.0)  # skidpad ref


# ── 2. corner radii match their nominal value (< 0.5%) ───────────────────────

@pytest.mark.parametrize("label,radius", [
    ("corner_R150", 150.0),
    ("corner_R100", 100.0),
    ("corner_R60", 60.0),
    ("corner_R40", 40.0),
    ("corner_R25", 25.0),
])
def test_corner_radii_match_nominal(label, radius):
    """Each labelled corner's lane-edge circles are at the nominal radius.

    The annulus inner/outer radii are nominal ± half_road_width; the centre
    radius (mid-pavement) must equal the nominal radius to < 0.5%."""
    pg = _load_pg()
    markers = [m for m in pg.markers if m.meta.get("corner") == label]
    assert markers, f"no entry marker for {label}"
    m = markers[0]
    assert m.meta.get("radius") == pytest.approx(radius, rel=0.005)


# ── 3. straight length ≥ 800 m ───────────────────────────────────────────────

def test_straight_length():
    """The high-speed straight must reach ≥ 800 m (plan: 0→100 km/h + brake)."""
    pg = _load_pg()
    # distance posts are placed every 50 m from m=50 onward; the max is the
    # last post before straight_len.
    straight_posts = [m.meta["m"] for m in pg.markers
                      if m.type == "distance" and "corner" not in m.meta]
    assert straight_posts
    assert max(straight_posts) >= 750  # last 50 m post before 800


# ── 4. spawns: ≥ 4, all on pavement (sanity: within the ground base) ──────────

def test_all_spawns_are_on_pavement():
    """At least 4 named spawns, and each is within the scenario's ground box
    (a coarse sanity check that none lands off-world)."""
    pg = _load_pg()
    assert len(pg.spawns) >= 4
    for sp in pg.spawns:
        # ground base rect is _rect(400, -260, 1300, 500): x∈[-250,1050], y∈[-510,-10]
        assert -300 < sp.x < 1100
        assert -600 < sp.y < 200


def test_find_spawn_falls_back():
    pg = _load_pg()
    # named lookup
    pose = pg.find_spawn("操控区")
    assert pose[0] == pytest.approx(pg.anchors["handling"][0], abs=1.0)
    # unknown name → effective spawn (not a crash)
    assert pg.find_spawn("nope") == pg.effective_spawn
    # None → effective spawn
    assert pg.find_spawn(None) == pg.effective_spawn


# ── 5. anchors exist & anchored courses transform ────────────────────────────

def test_anchors_exist_and_courses_fit():
    pg = _load_pg()
    for name in ("straight_start", "skidpad", "handling"):
        assert name in pg.anchors, f"missing required anchor {name}"


def test_anchored_course_is_translated():
    """An anchored course leaves the world origin; an unanchored one does not."""
    _load_pg()
    # unanchored: at origin
    p0 = plan_from_template("slalom", {"n_gates": 3, "spacing": 6.0,
                                       "entry": 12.0, "offset": 1.5,
                                       "vehicle_width": 2.0})
    assert p0.points[0][0] == pytest.approx(0.0, abs=1.0)
    # anchored onto handling pad
    p1 = plan_from_template("slalom", {"n_gates": 3, "spacing": 6.0,
                                       "entry": 12.0, "offset": 1.5,
                                       "vehicle_width": 2.0}, anchor="handling")
    ax = scn.get_active().anchors["handling"][0]
    assert p1.points[0][0] > 100.0, "anchored course did not move off the origin"
    assert p1.points[0][0] == pytest.approx(ax, abs=1.0)
    assert "handling" in p1.notes


def test_bad_anchor_raises():
    _load_pg()
    with pytest.raises(KeyError):
        plan_from_template("slalom", anchor="does_not_exist")


def test_anchor_without_scenario_raises():
    scn.clear_scenario()
    with pytest.raises(ValueError):
        plan_from_template("slalom", anchor="handling")


# ── 6. backward-compat: legacy scenarios serialise with empty spawns/anchors ─

def test_legacy_scenarios_have_no_named_spawns():
    """plaza/town/track_* keep their single `spawn`; spawns/anchors stay empty
    so their serialised shape is a strict superset of the old contract."""
    for name in ("plaza", "town", "track_small", "track_shanghai"):
        scn.load_scenario(name)
        ser = scn.get_active().serialize()
        assert ser["spawns"] == []
        assert ser["anchors"] == {}
        # the legacy single `spawn` field is still present and non-empty
        assert len(ser["spawn"]) == 3
