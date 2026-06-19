"""Reference-path plan + standard-maneuver generators.

A `PathPlan` is a dense world-frame polyline that the `follow_trajectory`
strategy tracks with pure pursuit, plus an optional list of **cones** (ground
markers) so a human driver can also run the maneuver manually.

The active plan is a process-wide singleton (like the Simulator) set via the
REST layer. A monotonically increasing `version` lets the frontend know when
to re-fetch the (relatively static) path instead of streaming it at 60 Hz.

Frame: world X-forward, Y-left (see docs/design.md §2). Maneuvers are laid out
starting near the world origin along +X, matching the vehicle's default spawn.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass
class PathPlan:
    """A dense reference polyline plus optional ground cones."""
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))  # (N, 2) world
    cones: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))   # (M, 2) world
    closed: bool = False
    name: str = ""

    def is_empty(self) -> bool:
        return self.points.shape[0] < 2

    def serialize(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "closed": self.closed,
            "points": [[float(p[0]), float(p[1])] for p in self.points],
            "cones": [[float(c[0]), float(c[1])] for c in self.cones],
        }


# ---------------------------------------------------------------------------
# Process-wide active plan
# ---------------------------------------------------------------------------

_active: PathPlan = PathPlan()
_version: int = 0


def get_active_plan() -> PathPlan:
    return _active


def get_version() -> int:
    return _version


def set_active_plan(plan: PathPlan) -> int:
    global _active, _version
    _active = plan
    _version += 1
    return _version


def clear_active_plan() -> int:
    return set_active_plan(PathPlan())


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def resample(points: np.ndarray, step: float = 0.5) -> np.ndarray:
    """Resample a polyline to roughly-uniform arc-length spacing `step` [m]."""
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if pts.shape[0] < 2:
        return pts
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = float(s[-1])
    if total < 1e-9:
        return pts[:1]
    n = max(2, int(round(total / step)) + 1)
    su = np.linspace(0.0, total, n)
    x = np.interp(su, s, pts[:, 0])
    y = np.interp(su, s, pts[:, 1])
    return np.column_stack([x, y])


def catmull_rom(waypoints: np.ndarray, samples_per_seg: int = 16,
                closed: bool = False) -> np.ndarray:
    """Smooth a set of waypoints with a centripetal Catmull-Rom spline."""
    wp = np.asarray(waypoints, dtype=np.float64).reshape(-1, 2)
    if wp.shape[0] < 3:
        return wp
    pts = wp.copy()
    if closed:
        ext = np.vstack([pts[-1], pts, pts[0], pts[1]])
    else:
        ext = np.vstack([pts[0], pts, pts[-1]])
    out: list[np.ndarray] = []
    n_seg = ext.shape[0] - 3
    for i in range(n_seg):
        p0, p1, p2, p3 = ext[i], ext[i + 1], ext[i + 2], ext[i + 3]
        for j in range(samples_per_seg):
            t = j / samples_per_seg
            t2, t3 = t * t, t * t * t
            a = 2 * p1
            b = (-p0 + p2) * t
            c = (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
            d = (-p0 + 3 * p1 - 3 * p2 + p3) * t3
            out.append(0.5 * (a + b + c + d))
    out.append(ext[-2])
    return np.array(out)


def plan_from_waypoints(points: list[list[float]], *, closed: bool = False,
                        name: str = "custom") -> PathPlan:
    wp = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if wp.shape[0] < 2:
        return PathPlan(points=wp, closed=closed, name=name)
    smooth = catmull_rom(wp, closed=closed) if wp.shape[0] >= 3 else wp
    dense = resample(smooth, step=0.5)
    return PathPlan(points=dense, cones=np.zeros((0, 2)), closed=closed, name=name)


# ---------------------------------------------------------------------------
# Standard maneuver generators → PathPlan (path + cones)
# ---------------------------------------------------------------------------


def gen_straight(length: float = 60.0) -> PathPlan:
    xs = np.linspace(0.0, length, int(length / 0.5) + 1)
    pts = np.column_stack([xs, np.zeros_like(xs)])
    return PathPlan(points=pts, name="straight")


def gen_arc(radius: float = 20.0, sweep_deg: float = 180.0) -> PathPlan:
    sweep = math.radians(sweep_deg)
    n = max(8, int(abs(radius * sweep) / 0.5))
    th = np.linspace(0.0, sweep, n)
    # Start at origin heading +X, turning left (center at (0, radius)).
    x = radius * np.sin(th)
    y = radius * (1.0 - np.cos(th))
    return PathPlan(points=np.column_stack([x, y]), name="arc")


def gen_slalom(n_gates: int = 6, spacing: float = 18.0, offset: float = 2.5) -> PathPlan:
    """Cones in a centreline row spaced `spacing`; path weaves ±offset."""
    cones = np.array([[spacing * (i + 1), 0.0] for i in range(n_gates)])
    # Build weaving waypoints: peak amplitude between cones, zero at cones.
    wp: list[list[float]] = [[0.0, 0.0]]
    for i in range(n_gates):
        xc = spacing * (i + 1)
        side = 1.0 if i % 2 == 0 else -1.0
        wp.append([xc - spacing / 2.0, side * offset])
        wp.append([xc, 0.0])
    wp.append([spacing * (n_gates + 0.5), 0.0])
    dense = resample(catmull_rom(np.array(wp)), step=0.5)
    return PathPlan(points=dense, cones=cones, name="slalom")


def gen_double_lane_change(lane_width: float = 3.5, lane_offset: float = 3.5) -> PathPlan:
    """ISO-3888-style double lane change (simplified, symmetric)."""
    # Section lengths (m): entry, transition, side lane, transition, exit.
    seg = [15.0, 13.5, 11.0, 12.5, 15.0]
    x = 0.0
    wp = [[0.0, 0.0]]
    x += seg[0]; wp.append([x, 0.0])
    x += seg[1]; wp.append([x, lane_offset])
    x += seg[2]; wp.append([x, lane_offset])
    x += seg[3]; wp.append([x, 0.0])
    x += seg[4]; wp.append([x, 0.0])
    dense = resample(catmull_rom(np.array(wp)), step=0.5)
    # Cones marking the three lanes (left+right edges of each gate).
    hw = lane_width / 2.0
    cones = []
    gates = [(seg[0] / 2, 0.0), (seg[0] + seg[1] + seg[2] / 2, lane_offset),
             (sum(seg) - seg[4] / 2, 0.0)]
    for gx, gy in gates:
        cones.append([gx, gy + hw])
        cones.append([gx, gy - hw])
    return PathPlan(points=dense, cones=np.array(cones), name="double_lane_change")


def gen_figure_eight(radius: float = 12.0) -> PathPlan:
    """Closed figure-eight: two tangent circles."""
    n = max(24, int(2 * math.pi * radius / 0.5))
    th = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    # Left circle centred at (0, radius), right circle at (0, -radius).
    left = np.column_stack([radius * np.sin(th), radius - radius * np.cos(th)])
    right = np.column_stack([radius * np.sin(th), -radius + radius * np.cos(th)])
    pts = np.vstack([left, right[::-1]])
    return PathPlan(points=pts, closed=True, name="figure_eight")


def gen_parking(slot_length: float = 6.5, slot_width: float = 2.5,
                approach: float = 10.0) -> PathPlan:
    """Parallel-parking S-curve into a slot to the right of the lane."""
    wp = [
        [0.0, 0.0],
        [approach, 0.0],
        [approach + slot_length * 0.5, -slot_width * 0.55],
        [approach + slot_length, -slot_width],
        [approach + slot_length * 1.2, -slot_width],
    ]
    dense = resample(catmull_rom(np.array(wp)), step=0.3)
    # Cones outlining the slot box (right side of the lane).
    x0 = approach
    y_top = -slot_width + slot_width * 0.0   # lane-side edge
    cones = np.array([
        [x0, -slot_width - 0.3],
        [x0 + slot_length, -slot_width - 0.3],
        [x0, y_top - 0.3],
        [x0 + slot_length, y_top - 0.3],
    ])
    return PathPlan(points=dense, cones=cones, name="parking")


_TEMPLATES = {
    "straight": gen_straight,
    "arc": gen_arc,
    "slalom": gen_slalom,
    "double_lane_change": gen_double_lane_change,
    "figure_eight": gen_figure_eight,
    "parking": gen_parking,
}


def list_templates() -> list[str]:
    return list(_TEMPLATES.keys())


def plan_from_template(name: str, params: dict[str, Any] | None = None) -> PathPlan:
    if name not in _TEMPLATES:
        raise KeyError(name)
    fn = _TEMPLATES[name]
    return fn(**(params or {}))
