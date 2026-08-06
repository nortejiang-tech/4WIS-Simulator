"""Reference-path plan + standard-maneuver generators.

A `PathPlan` is a dense world-frame polyline that the `follow_trajectory`
strategy tracks with pure pursuit, plus the **course furniture** a human driver
needs to run the same maneuver by hand:

    cones   ground markers (75 cm 锥桶 / 1.5 m 标杆) — what you actually aim at
    marks   painted ground lines (lane edges, section boxes, start/finish)

The furniture is not decoration: the standard maneuvers below are laid out to
their published geometry (ISO 3888-1/-2 lane changes, ISO 4138 steady-state
circle), so the cone lane widths scale with the vehicle's own width and driving
between them means the same thing here as it does on a proving ground.

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

# ── palette ──────────────────────────────────────────────────────────────────
# Orange is the working cone; green/red bookend the course (entry / exit) and
# yellow flags the displaced lane of a lane-change so the offset reads at a
# glance from the roof camera.
CONE_ORANGE = "#f97316"
CONE_RED = "#dc2626"
CONE_GREEN = "#22c55e"
CONE_YELLOW = "#eab308"
CONE_BLUE = "#38bdf8"
PAINT = "#e8e8e8"
PAINT_DIM = "#94a3b8"

# Drawn body width = max(track) × 1.14 (see frontend vehicleShape.bodyDimensions).
# Keeping the same factor here means the ISO lane widths match the vehicle the
# user actually sees between the cones.
BODY_WIDTH_FACTOR = 1.14
DEFAULT_VEHICLE_WIDTH = 1.78    # LS9 default track 1.565 m × 1.14
DEFAULT_VEHICLE_LENGTH = 3.79   # LS9 default wheelbase 3.16 m × 1.20


@dataclass
class Cone:
    """A ground marker.

    `kind` picks the glyph both viewports draw, not just a colour:
        cone   75 cm reflective traffic cone — the default course marker
        pole   1.5 m red/white slalom pole — reads much better at speed, so
               it's what the slalom gates use
    """

    x: float
    y: float
    kind: str = "cone"
    color: str = CONE_ORANGE
    height: float = 0.75

    def serialize(self) -> dict[str, Any]:
        return {"x": float(self.x), "y": float(self.y), "kind": self.kind,
                "color": self.color, "height": float(self.height)}


@dataclass
class Mark:
    """A painted ground line (chalk / tape), drawn under the cones."""

    points: list[list[float]]
    color: str = PAINT
    width: float = 0.12
    dash: bool = False

    def serialize(self) -> dict[str, Any]:
        return {"points": [[float(p[0]), float(p[1])] for p in self.points],
                "color": self.color, "width": float(self.width), "dash": self.dash}


@dataclass
class PathPlan:
    """A dense reference polyline plus its course furniture."""

    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))  # (N, 2) world
    cones: list[Cone] = field(default_factory=list)
    marks: list[Mark] = field(default_factory=list)
    closed: bool = False
    name: str = ""
    label: str = ""       # human-readable name for the panel
    notes: str = ""       # the geometry that was actually laid out, in one line

    def is_empty(self) -> bool:
        return self.points.shape[0] < 2

    def serialize(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "label": self.label,
            "notes": self.notes,
            "closed": self.closed,
            "points": [[float(p[0]), float(p[1])] for p in self.points],
            "cones": [c.serialize() for c in self.cones],
            "marks": [m.serialize() for m in self.marks],
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


def _smooth_path(waypoints: list[list[float]], step: float = 0.5) -> np.ndarray:
    wp = np.asarray(waypoints, dtype=np.float64).reshape(-1, 2)
    if wp.shape[0] < 3:
        return resample(wp, step)
    return resample(catmull_rom(wp), step)


def plan_from_waypoints(points: list[list[float]], *, closed: bool = False,
                        name: str = "custom") -> PathPlan:
    wp = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if wp.shape[0] < 2:
        return PathPlan(points=wp, closed=closed, name=name, label="自定义航点")
    smooth = catmull_rom(wp, closed=closed) if wp.shape[0] >= 3 else wp
    dense = resample(smooth, step=0.5)
    return PathPlan(points=dense, closed=closed, name=name, label="自定义航点")


# ---------------------------------------------------------------------------
# Course-furniture helpers
# ---------------------------------------------------------------------------


def _cone_row(x0: float, y0: float, x1: float, y1: float, *, step: float = 3.0,
              kind: str = "cone", color: str = CONE_ORANGE,
              height: float = 0.75) -> list[Cone]:
    """Cones every ~`step` m along a segment, both endpoints included."""
    dist = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(round(dist / max(step, 0.5))))
    return [Cone(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n,
                 kind=kind, color=color, height=height) for i in range(n + 1)]


def _cone_arc(cx: float, cy: float, radius: float, th0: float, th1: float, *,
              step: float = 6.0, color: str = CONE_ORANGE,
              kind: str = "cone", height: float = 0.75) -> list[Cone]:
    """Cones every ~`step` m of arc length along a circle."""
    n = max(2, int(round(abs(radius * (th1 - th0)) / max(step, 0.5))))
    return [Cone(cx + radius * math.cos(th0 + (th1 - th0) * i / n),
                 cy + radius * math.sin(th0 + (th1 - th0) * i / n),
                 kind=kind, color=color, height=height) for i in range(n)]


def _gate(x: float, y_lo: float, y_hi: float, color: str) -> list[Cone]:
    """A cross-course gate: one cone on each side of the lane."""
    return [Cone(x, y_hi, color=color), Cone(x, y_lo, color=color)]


def _cross(x: float, y_lo: float, y_hi: float, color: str = PAINT,
           width: float = 0.2) -> Mark:
    return Mark([[x, y_lo], [x, y_hi]], color=color, width=width)


def _box(x0: float, x1: float, y_lo: float, y_hi: float, color: str = PAINT,
         width: float = 0.12) -> Mark:
    """Closed rectangle outline (a painted test section)."""
    return Mark([[x0, y_lo], [x1, y_lo], [x1, y_hi], [x0, y_hi], [x0, y_lo]],
                color=color, width=width)


def _arc_pts(cx: float, cy: float, r: float, th0: float, th1: float,
             n: int = 96) -> list[list[float]]:
    return [[cx + r * math.cos(th0 + (th1 - th0) * i / n),
             cy + r * math.sin(th0 + (th1 - th0) * i / n)] for i in range(n + 1)]


def _fmt(v: float) -> str:
    return f"{v:.2f}".rstrip("0").rstrip(".")


# ---------------------------------------------------------------------------
# Standard maneuver generators → PathPlan (path + cones + marks)
# ---------------------------------------------------------------------------


def gen_straight(length: float = 120.0, lane_width: float = 3.5,
                 gate_spacing: float = 20.0) -> PathPlan:
    """Straight lane with distance gates — acceleration / braking / stability."""
    length = max(20.0, float(length))
    hw = max(1.5, lane_width) / 2.0
    pts = resample(np.array([[0.0, 0.0], [length, 0.0]]), 0.5)

    marks = [
        Mark([[0.0, +hw], [length, +hw]], PAINT, 0.14),
        Mark([[0.0, -hw], [length, -hw]], PAINT, 0.14),
        Mark([[0.0, 0.0], [length, 0.0]], PAINT_DIM, 0.10, dash=True),
        _cross(0.0, -hw, hw, PAINT, 0.3),
        _cross(length, -hw, hw, PAINT, 0.3),
    ]
    cones: list[Cone] = []
    x = 0.0
    while x <= length + 1e-6:
        color = CONE_GREEN if x < 1e-6 else (CONE_RED if x >= length - 1e-6 else CONE_ORANGE)
        cones += _gate(x, -hw, hw, color)
        x += max(5.0, gate_spacing)
    if cones and abs(cones[-1].x - length) > 1e-6:
        cones += _gate(length, -hw, hw, CONE_RED)

    return PathPlan(points=pts, cones=cones, marks=marks, name="straight",
                    label="直线",
                    notes=f"直线 {_fmt(length)} m · 车道宽 {_fmt(lane_width)} m · "
                          f"每 {_fmt(gate_spacing)} m 一道门（绿=起点，红=终点）")


def gen_arc(radius: float = 25.0, sweep_deg: float = 180.0,
            lane_width: float = 4.0) -> PathPlan:
    """Constant-radius turn, cone-lined on both sides."""
    radius = max(4.0, float(radius))
    sweep = math.radians(float(sweep_deg))
    n = max(8, int(abs(radius * sweep) / 0.5))
    th = np.linspace(0.0, sweep, n)
    # Start at origin heading +X, turning left (centre at (0, radius)).
    pts = np.column_stack([radius * np.sin(th), radius * (1.0 - np.cos(th))])

    hw = max(1.5, lane_width) / 2.0
    # Circle centred at (0, radius); the path is the u = (sin θ, −cos θ) ray.
    a0, a1 = -math.pi / 2, -math.pi / 2 + sweep
    marks = [
        Mark(_arc_pts(0.0, radius, radius + hw, a0, a1), PAINT, 0.14),
        Mark(_arc_pts(0.0, radius, radius - hw, a0, a1), PAINT, 0.14),
        Mark(_arc_pts(0.0, radius, radius, a0, a1), PAINT_DIM, 0.10, dash=True),
    ]
    cones = (_cone_arc(0.0, radius, radius + hw, a0, a1, step=8.0)
             + _cone_arc(0.0, radius, radius - hw, a0, a1, step=8.0))
    cones += _gate(0.0, -hw, hw, CONE_GREEN)

    return PathPlan(points=pts, cones=cones, marks=marks, name="arc", label="圆弧",
                    notes=f"定半径 R={_fmt(radius)} m · 转角 {_fmt(sweep_deg)}° · "
                          f"车道宽 {_fmt(lane_width)} m")


def gen_slalom(n_gates: int = 6, spacing: float = 18.0, entry: float = 20.0,
               offset: float = 0.0,
               vehicle_width: float = DEFAULT_VEHICLE_WIDTH) -> PathPlan:
    """Slalom: a row of poles at fixed spacing, path weaving *past* each one.

    The reference line reaches full lateral offset **at** each pole and crosses
    the centreline halfway between them — i.e. the car drives beside the pole,
    alternating sides. (An earlier version put the zero crossing at the pole,
    which drove the vehicle straight through it.)
    """
    n_gates = max(2, int(n_gates))
    spacing = max(6.0, float(spacing))
    entry = max(5.0, float(entry))
    # Default amplitude: half the body plus ~1.2 m of clearance to the pole.
    off = float(offset) if offset > 0 else vehicle_width / 2.0 + 1.2

    poles: list[Cone] = []
    for i in range(n_gates):
        xc = entry + i * spacing
        color = CONE_GREEN if i == 0 else (CONE_RED if i == n_gates - 1 else "#ef4444")
        poles.append(Cone(xc, 0.0, kind="pole", color=color, height=1.5))

    last_x = entry + (n_gates - 1) * spacing
    run_out = last_x + spacing

    # Entry / exit gates, wide enough to aim at from a distance.
    gate_hw = off + 1.5
    cones = list(poles)
    cones += _gate(entry - spacing / 2.0, -gate_hw, gate_hw, CONE_GREEN)
    cones += _gate(run_out, -gate_hw, gate_hw, CONE_RED)

    marks = [
        Mark([[0.0, 0.0], [run_out + 10.0, 0.0]], PAINT_DIM, 0.10, dash=True),
        _cross(entry - spacing / 2.0, -gate_hw, gate_hw, PAINT, 0.25),
        _cross(run_out, -gate_hw, gate_hw, PAINT, 0.25),
    ]

    wp: list[list[float]] = [[0.0, 0.0], [entry - spacing / 2.0, 0.0]]
    for i in range(n_gates):
        xc = entry + i * spacing
        side = 1.0 if i % 2 == 0 else -1.0
        wp.append([xc, side * off])                    # beside the pole
        wp.append([xc + spacing / 2.0, 0.0])           # centreline crossing
    wp.append([run_out + 10.0, 0.0])

    return PathPlan(points=_smooth_path(wp), cones=cones, marks=marks,
                    name="slalom", label="绕桩 (Slalom)",
                    notes=f"{n_gates} 根标杆 · 桩距 {_fmt(spacing)} m · "
                          f"路线侧向幅值 ±{_fmt(off)} m（车宽 {_fmt(vehicle_width)} m）· "
                          f"引入段 {_fmt(entry)} m")


def _lane_change_course(sections: list[tuple[float, float, float]],
                        gaps: list[float], entry: float, run_out: float,
                        *, name: str, label: str, notes: str,
                        cone_step: float = 3.0) -> PathPlan:
    """Build a cone-lined lane-change course.

    `sections` is [(length, width, centre_y), ...] laid end-to-end along +X with
    `gaps[i]` metres of open ground between section i and i+1. Every section is
    lined with cones on both edges and painted as a box; the reference path runs
    down the centre of each section and transitions across the gaps.
    """
    cones: list[Cone] = []
    marks: list[Mark] = []
    wp: list[list[float]] = [[0.0, 0.0]]

    x = entry
    spans: list[tuple[float, float, float, float]] = []   # (x0, x1, cy, hw)
    for i, (length, width, cy) in enumerate(sections):
        hw = width / 2.0
        spans.append((x, x + length, cy, hw))
        x += length + (gaps[i] if i < len(gaps) else 0.0)

    for i, (x0, x1, cy, hw) in enumerate(spans):
        first, last = i == 0, i == len(spans) - 1
        # Displaced sections get yellow cones so the offset lane reads instantly.
        edge = CONE_YELLOW if abs(cy) > 1e-6 else CONE_ORANGE
        cones += _cone_row(x0, cy + hw, x1, cy + hw, step=cone_step, color=edge)
        cones += _cone_row(x0, cy - hw, x1, cy - hw, step=cone_step, color=edge)
        if first:
            cones += _gate(x0, cy - hw, cy + hw, CONE_GREEN)
        if last:
            cones += _gate(x1, cy - hw, cy + hw, CONE_RED)
        marks.append(_box(x0, x1, cy - hw, cy + hw))
        # Section centre + entry/exit waypoints keep the spline inside the box.
        wp += [[x0, cy], [(x0 + x1) / 2.0, cy], [x1, cy]]
        if not last:
            nx0, ncy = spans[i + 1][0], spans[i + 1][2]
            wp.append([(x1 + nx0) / 2.0, (cy + ncy) / 2.0])   # mid-transition

    marks.append(_cross(0.0, -1.5, 1.5, PAINT, 0.3))
    marks.append(Mark([[0.0, 0.0], [entry, 0.0]], PAINT_DIM, 0.10, dash=True))
    wp.insert(1, [entry * 0.6, 0.0])
    wp.append([spans[-1][1] + run_out, spans[-1][2]])

    return PathPlan(points=_smooth_path(wp), cones=cones, marks=marks,
                    name=name, label=label, notes=notes)


def gen_double_lane_change(vehicle_width: float = DEFAULT_VEHICLE_WIDTH,
                           entry: float = 15.0) -> PathPlan:
    """ISO 3888-2 severe lane change ("moose test"), laid out to the standard.

        section 1   12.0 m   width 1.1·b + 0.25
        gap         13.5 m
        section 2   11.0 m   width b + 1.00   offset 1 m from section 1
        gap         12.5 m
        section 3   12.0 m   width 1.3·b + 0.25   (back in the original lane)

    The 1 m offset is edge-to-edge, so the centre-to-centre displacement is
    w1/2 + 1 + w2/2 ≈ 3.5 m for a passenger car.
    """
    b = max(1.2, float(vehicle_width))
    w1 = 1.1 * b + 0.25
    w2 = b + 1.0
    w3 = 1.3 * b + 0.25
    offset = w1 / 2.0 + 1.0 + w2 / 2.0

    return _lane_change_course(
        sections=[(12.0, w1, 0.0), (11.0, w2, offset), (12.0, w3, 0.0)],
        gaps=[13.5, 12.5], entry=float(entry), run_out=20.0,
        name="double_lane_change", label="双移线 · ISO 3888-2 (麋鹿)",
        notes=f"ISO 3888-2 · 车宽 {_fmt(b)} m → 段1 {_fmt(w1)} m / "
              f"侧道 {_fmt(w2)} m / 段3 {_fmt(w3)} m · 横向偏移 {_fmt(offset)} m · "
              f"段长 12/11/12 m，间隔 13.5/12.5 m")


def gen_iso3888_1(vehicle_width: float = DEFAULT_VEHICLE_WIDTH,
                  entry: float = 20.0) -> PathPlan:
    """ISO 3888-1 double lane change — the longer, gentler obstacle-avoidance
    course (15/25/15 m sections, 3.5 m lateral offset)."""
    b = max(1.2, float(vehicle_width))
    w1 = 1.1 * b + 0.25
    w2 = 1.2 * b + 0.25
    w3 = 1.3 * b + 0.25
    offset = 3.5

    return _lane_change_course(
        sections=[(15.0, w1, 0.0), (25.0, w2, offset), (15.0, w3, 0.0)],
        gaps=[30.0, 25.0], entry=float(entry), run_out=25.0, cone_step=4.0,
        name="iso3888_1", label="双移线 · ISO 3888-1",
        notes=f"ISO 3888-1 · 车宽 {_fmt(b)} m → 段1 {_fmt(w1)} m / "
              f"侧道 {_fmt(w2)} m / 段3 {_fmt(w3)} m · 横向偏移 {_fmt(offset)} m · "
              f"段长 15/25/15 m，间隔 30/25 m")


def gen_lane_change(offset: float = 3.5, transition: float = 25.0,
                    entry: float = 25.0,
                    vehicle_width: float = DEFAULT_VEHICLE_WIDTH) -> PathPlan:
    """Single lane change — move over one lane and stay there."""
    b = max(1.2, float(vehicle_width))
    w = 1.1 * b + 0.25
    off = float(offset)

    return _lane_change_course(
        sections=[(20.0, w, 0.0), (25.0, w, off)],
        gaps=[max(8.0, float(transition))], entry=float(entry), run_out=20.0,
        name="lane_change", label="单移线",
        notes=f"单移线 · 车道宽 {_fmt(w)} m（车宽 {_fmt(b)} m）· "
              f"横向偏移 {_fmt(off)} m · 换道段 {_fmt(transition)} m")


def gen_skidpad(radius: float = 30.0, lane_width: float = 4.0) -> PathPlan:
    """ISO 4138 steady-state circular test — a closed cone circle."""
    radius = max(6.0, float(radius))
    hw = max(1.5, lane_width) / 2.0
    n = max(48, int(2 * math.pi * radius / 0.5))
    th = np.linspace(-math.pi / 2, 3 * math.pi / 2, n, endpoint=False)
    # Centre at (0, radius) so the circle starts at the origin heading +X.
    pts = np.column_stack([radius * np.cos(th), radius + radius * np.sin(th)])

    marks = [
        Mark(_arc_pts(0.0, radius, radius + hw, 0.0, 2 * math.pi), PAINT, 0.14),
        Mark(_arc_pts(0.0, radius, radius - hw, 0.0, 2 * math.pi), PAINT, 0.14),
        Mark(_arc_pts(0.0, radius, radius, 0.0, 2 * math.pi), PAINT_DIM, 0.10, dash=True),
        _cross(0.0, -hw, hw, PAINT, 0.3),
    ]
    cones = (_cone_arc(0.0, radius, radius + hw, 0.0, 2 * math.pi, step=8.0)
             + _cone_arc(0.0, radius, radius - hw, 0.0, 2 * math.pi, step=8.0,
                         color=CONE_BLUE))
    cones += _gate(0.0, -hw, hw, CONE_GREEN)

    return PathPlan(points=pts, cones=cones, marks=marks, closed=True,
                    name="skidpad", label="定圆 (ISO 4138)",
                    notes=f"稳态回转 R={_fmt(radius)} m（中线）· 车道宽 "
                          f"{_fmt(lane_width)} m · 外圈橙桩 / 内圈蓝桩")


def gen_figure_eight(radius: float = 15.0) -> PathPlan:
    """Closed figure-eight: two tangent circles, each ringed by cones."""
    radius = max(5.0, float(radius))
    n = max(24, int(2 * math.pi * radius / 0.5))
    th = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    # Left circle centred at (0, radius), right circle at (0, -radius).
    left = np.column_stack([radius * np.sin(th), radius - radius * np.cos(th)])
    right = np.column_stack([radius * np.sin(th), -radius + radius * np.cos(th)])
    pts = np.vstack([left, right[::-1]])

    # The cones sit *inside* each loop — that's the thing you drive around.
    r_in = max(2.0, radius - 2.0)
    cones = (_cone_arc(0.0, +radius, r_in, 0.0, 2 * math.pi, step=6.0)
             + _cone_arc(0.0, -radius, r_in, 0.0, 2 * math.pi, step=6.0,
                         color=CONE_BLUE))
    cones += [Cone(0.0, 0.0, kind="pole", color=CONE_GREEN, height=1.5)]
    marks = [
        Mark(_arc_pts(0.0, +radius, r_in, 0.0, 2 * math.pi), PAINT_DIM, 0.10, dash=True),
        Mark(_arc_pts(0.0, -radius, r_in, 0.0, 2 * math.pi), PAINT_DIM, 0.10, dash=True),
    ]
    return PathPlan(points=pts, cones=cones, marks=marks, closed=True,
                    name="figure_eight", label="八字",
                    notes=f"两个相切圆 R={_fmt(radius)} m · 绕内圈桩阵行驶（橙/蓝各一圈）")


def gen_parking(vehicle_length: float = DEFAULT_VEHICLE_LENGTH,
                vehicle_width: float = DEFAULT_VEHICLE_WIDTH,
                approach: float = 14.0, margin: float = 1.5) -> PathPlan:
    """Parallel-parking bay on the right, sized from the vehicle.

    Slot = vehicle length + `margin`, vehicle width + 0.8 m — the usual test
    dimensions. The reference line is a forward S-curve into the slot: pure
    pursuit cannot reverse, so auto-follow only approximates the manoeuvre;
    driving it by hand (in reverse) is the point of the markings.
    """
    lv = max(2.0, float(vehicle_length))
    bv = max(1.2, float(vehicle_width))
    slot_len = lv + max(0.5, float(margin))
    slot_w = bv + 0.8
    approach = max(5.0, float(approach))

    curb_y = -3.0                      # road edge
    y_far = curb_y                     # slot far (curb) side
    y_near = curb_y + slot_w           # slot lane side
    y_slot = (y_far + y_near) / 2.0
    x0, x1 = approach, approach + slot_len
    total = x1 + 12.0

    marks = [
        Mark([[0.0, curb_y], [total, curb_y]], PAINT, 0.16),          # kerb
        Mark([[0.0, 0.0], [total, 0.0]], PAINT_DIM, 0.10, dash=True),  # lane centre
        _box(x0, x1, y_far, y_near, PAINT, 0.14),
    ]
    # Kerb-side corners get cones; the lane-side corners get poles, which stay
    # visible over the bodywork once the car is alongside the slot.
    cones = [
        Cone(x0, y_far, color=CONE_GREEN), Cone(x1, y_far, color=CONE_RED),
        Cone(x0, y_near, kind="pole", color=CONE_GREEN, height=1.2),
        Cone(x1, y_near, kind="pole", color=CONE_RED, height=1.2),
    ]

    wp = [
        [0.0, 0.0],
        [approach * 0.7, 0.0],
        [x0 + slot_len * 0.35, y_slot * 0.45],
        [x0 + slot_len * 0.75, y_slot],
        [x1 - 0.4, y_slot],
    ]
    return PathPlan(points=_smooth_path(wp, 0.3), cones=cones, marks=marks,
                    name="parking", label="侧方停车",
                    notes=f"车位 {_fmt(slot_len)} × {_fmt(slot_w)} m"
                          f"（车长 {_fmt(lv)} + {_fmt(margin)} / 车宽 {_fmt(bv)} + 0.8）· "
                          f"跟踪策略只能前进入位，倒库请手动驾驶")


_TEMPLATES = {
    "straight": gen_straight,
    "arc": gen_arc,
    "slalom": gen_slalom,
    "lane_change": gen_lane_change,
    "double_lane_change": gen_double_lane_change,
    "iso3888_1": gen_iso3888_1,
    "skidpad": gen_skidpad,
    "figure_eight": gen_figure_eight,
    "parking": gen_parking,
}


# ---------------------------------------------------------------------------
# Template metadata — drives the panel's parameter inputs
# ---------------------------------------------------------------------------

def _p(key: str, label: str, default: float, lo: float, hi: float,
       step: float, unit: str = "m") -> dict[str, Any]:
    return {"key": key, "label": label, "default": default, "min": lo,
            "max": hi, "step": step, "unit": unit}


# Params the caller never sets by hand — the REST layer fills them from the
# active vehicle so the ISO lane widths track the car being driven.
VEHICLE_PARAMS = ("vehicle_width", "vehicle_length")

_SPECS: dict[str, dict[str, Any]] = {
    "straight": {"label": "直线", "desc": "加速 / 制动 / 直线稳定性，按门计距",
                 "params": [_p("length", "长度", 120, 20, 400, 10),
                            _p("lane_width", "车道宽", 3.5, 2.5, 8, 0.1),
                            _p("gate_spacing", "门间距", 20, 5, 50, 5)]},
    "arc": {"label": "圆弧", "desc": "定半径转弯，两侧锥桶夹道",
            "params": [_p("radius", "半径", 25, 5, 200, 1),
                       _p("sweep_deg", "转角", 180, 30, 360, 15, "°"),
                       _p("lane_width", "车道宽", 4, 2.5, 10, 0.1)]},
    "slalom": {"label": "绕桩 (Slalom)", "desc": "标杆阵列，左右交替绕行",
               "params": [_p("n_gates", "桩数", 6, 2, 20, 1, "根"),
                          _p("spacing", "桩距", 18, 8, 40, 1),
                          _p("entry", "引入段", 20, 5, 60, 5),
                          _p("offset", "侧向幅值", 0, 0, 6, 0.1)]},
    "lane_change": {"label": "单移线", "desc": "换到相邻车道并保持",
                    "params": [_p("offset", "横向偏移", 3.5, 1, 8, 0.1),
                               _p("transition", "换道段", 25, 8, 60, 1),
                               _p("entry", "引入段", 25, 5, 60, 5)]},
    "double_lane_change": {"label": "双移线 · ISO 3888-2 (麋鹿)",
                           "desc": "标准麋鹿测试，车道宽随车宽自适应",
                           "params": [_p("entry", "引入段", 15, 5, 60, 5)]},
    "iso3888_1": {"label": "双移线 · ISO 3888-1",
                  "desc": "较缓的标准双移线（15/25/15 m 段）",
                  "params": [_p("entry", "引入段", 20, 5, 60, 5)]},
    "skidpad": {"label": "定圆 (ISO 4138)", "desc": "稳态回转，测不足/过多转向",
                "params": [_p("radius", "半径", 30, 8, 200, 1),
                           _p("lane_width", "车道宽", 4, 2.5, 10, 0.1)]},
    "figure_eight": {"label": "八字", "desc": "左右交替满转，考察对称性",
                     "params": [_p("radius", "半径", 15, 5, 60, 1)]},
    "parking": {"label": "侧方停车", "desc": "车位尺寸随车长/车宽自适应",
                "params": [_p("approach", "接近段", 14, 5, 40, 1),
                           _p("margin", "车位余量", 1.5, 0.3, 4, 0.1)]},
}


def list_templates() -> list[str]:
    return list(_TEMPLATES.keys())


def template_specs() -> list[dict[str, Any]]:
    """Name + label + tunable parameters for every template (drives the UI)."""
    return [{"name": n, **_SPECS.get(n, {"label": n, "desc": "", "params": []})}
            for n in _TEMPLATES]


def template_accepts(name: str, key: str) -> bool:
    fn = _TEMPLATES.get(name)
    return bool(fn) and key in fn.__code__.co_varnames[:fn.__code__.co_argcount]


def plan_from_template(name: str, params: dict[str, Any] | None = None) -> PathPlan:
    if name not in _TEMPLATES:
        raise KeyError(name)
    fn = _TEMPLATES[name]
    kwargs = dict(params or {})
    # Drop vehicle params the generator doesn't take, so a caller (or the REST
    # layer) can always pass them without knowing each signature.
    for k in VEHICLE_PARAMS:
        if k in kwargs and not template_accepts(name, k):
            kwargs.pop(k)
    return fn(**kwargs)
