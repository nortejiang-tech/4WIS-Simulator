"""Static driving scenarios — visual environments for manual-driving context.

A scenario is purely decorative geometry (road surfaces, painted markings,
traffic lights, track edges) that gives the manual driver a sense of place. It
is *fetched once* over REST when loaded (kept in sync via a version counter,
exactly like the reference path) — never streamed per-frame — so rendering stays
cheap. The three primitives are intentionally minimal so both the 2D (Konva) and
3D (three.js) front-ends can draw any scenario generically:

    Surface  filled polygon (asphalt / concrete / grass / paint blocks)
    Line     polyline with width + dash (lane markings, track edges)
    Marker   point feature with a type (traffic_light / start_finish)

Coordinates are world-frame metres (x forward/up, y left), matching the sim.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# ── palette (hex, theme-independent — the scenes are "painted on the ground") ──
ASPHALT = "#3a3f47"
CONCRETE = "#54585f"
GRASS = "#2f5d3a"
KERB_A = "#d23b3b"
KERB_B = "#e8e8e8"
WHITE = "#e8e8e8"
YELLOW = "#e3b341"
RUNOFF = "#7a6a3a"
COBBLE = "#6d645b"      # paved square (cobblestone)
WATER = "#2f4a63"       # brook / canal
STONE = "#8a8278"       # church / landmark masonry
# warm medieval rooftop/facade tones for ordinary buildings
BLDG = ["#7a6f63", "#6e645a", "#837567", "#695f57", "#7d7468", "#746a5f"]


@dataclass
class Surface:
    points: list[list[float]]
    color: str
    kind: str = "road"

    def serialize(self) -> dict:
        return {"points": self.points, "color": self.color, "kind": self.kind}


@dataclass
class Line:
    points: list[list[float]]
    color: str = WHITE
    width: float = 0.15          # metres
    dash: bool = False
    # Optional (on, off) dash rhythm in metres; when set it overrides `dash`
    # and lets a builder paint standard lane-marking rhythms (e.g. GB 5768.3
    # "6 m mark / 9 m gap" for motorway lane dividers). None = legacy uniform
    # dash behaviour keyed off the boolean `dash`.
    dash_pattern: tuple[float, float] | None = None

    def serialize(self) -> dict:
        return {
            "points": self.points,
            "color": self.color,
            "width": self.width,
            "dash": self.dash,
            "dash_pattern": list(self.dash_pattern) if self.dash_pattern is not None else None,
        }


@dataclass
class Marker:
    type: str                    # traffic_light | start_finish | distance
    x: float
    y: float
    heading: float = 0.0         # rad, +x forward
    meta: dict = field(default_factory=dict)

    def serialize(self) -> dict:
        return {"type": self.type, "x": self.x, "y": self.y, "heading": self.heading, "meta": self.meta}


@dataclass
class Spawn:
    """A named spawn pose (x, y, heading) — lets a scenario offer more than one
    start point (e.g. a proving ground with a long-straight start, a skidpad
    entry, a handling area)."""
    name: str
    x: float
    y: float
    heading: float = 0.0         # rad, +x forward

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.heading)

    def serialize(self) -> dict:
        return {"name": self.name, "x": self.x, "y": self.y, "heading": self.heading}


@dataclass
class Scenario:
    name: str
    label: str
    surfaces: list[Surface] = field(default_factory=list)
    lines: list[Line] = field(default_factory=list)
    markers: list[Marker] = field(default_factory=list)
    spawn: tuple[float, float, float] = (0.0, 0.0, 0.0)   # x, y, heading (legacy single spawn)
    spawns: list[Spawn] = field(default_factory=list)      # named spawn points (optional)
    # Named anchor poses (x, y, heading) that path templates can be rigidly
    # transformed onto so a slalom/skidpad/straight course aligns with a real
    # road instead of always being laid down at the world origin along +X.
    anchors: dict[str, tuple[float, float, float]] = field(default_factory=dict)

    @property
    def effective_spawn(self) -> tuple[float, float, float]:
        """Resolve the spawn pose: prefer the first named spawn, fall back to `spawn`."""
        if self.spawns:
            return self.spawns[0].as_tuple()
        return self.spawn

    def find_spawn(self, name: str | None) -> tuple[float, float, float]:
        """Return the pose for a named spawn, or the effective spawn if name
        is None / not found."""
        if name:
            for sp in self.spawns:
                if sp.name == name:
                    return sp.as_tuple()
        return self.effective_spawn

    def serialize(self) -> dict:
        return {
            "name": self.name,
            "label": self.label,
            "surfaces": [s.serialize() for s in self.surfaces],
            "lines": [ln.serialize() for ln in self.lines],
            "markers": [m.serialize() for m in self.markers],
            "spawn": list(self.effective_spawn),
            "spawns": [sp.serialize() for sp in self.spawns],
            "anchors": {k: list(v) for k, v in self.anchors.items()},
        }


# ── geometry helpers ─────────────────────────────────────────────────────────

def _rect(cx: float, cy: float, w: float, h: float) -> list[list[float]]:
    """Axis-aligned rectangle centred at (cx,cy), width w (along x), height h (along y)."""
    hx, hy = w / 2, h / 2
    return [[cx - hx, cy - hy], [cx + hx, cy - hy], [cx + hx, cy + hy], [cx - hx, cy + hy]]


def _offset_loop(pts: list[list[float]], dist: float) -> list[list[float]]:
    """Offset a closed polyline by `dist` along per-vertex normals (left = +dist)."""
    n = len(pts)
    out: list[list[float]] = []
    for i in range(n):
        x0, y0 = pts[(i - 1) % n]
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % n]
        # incoming + outgoing segment normals (left normal of dir = (-dy, dx))
        nx, ny = 0.0, 0.0
        for (ax, ay, bx, by) in ((x0, y0, x1, y1), (x1, y1, x2, y2)):
            dx, dy = bx - ax, by - ay
            ln = math.hypot(dx, dy) or 1.0
            nx += -dy / ln
            ny += dx / ln
        ln = math.hypot(nx, ny) or 1.0
        out.append([x1 + dist * nx / ln, y1 + dist * ny / ln])
    return out


def _smooth_loop(pts: list[list[float]], iters: int = 2) -> list[list[float]]:
    """Chaikin corner-cutting for a closed loop — rounds hand-authored corners."""
    for _ in range(iters):
        n = len(pts)
        new: list[list[float]] = []
        for i in range(n):
            x1, y1 = pts[i]
            x2, y2 = pts[(i + 1) % n]
            new.append([0.75 * x1 + 0.25 * x2, 0.75 * y1 + 0.25 * y2])
            new.append([0.25 * x1 + 0.75 * x2, 0.25 * y1 + 0.75 * y2])
        pts = new
    return pts


def _track_from_centerline(name: str, label: str, center: list[list[float]],
                           half_w: float, spawn_idx: int = 0,
                           smooth: int = 2) -> Scenario:
    """Build a closed-loop track scenario from a centerline polyline."""
    cl = _smooth_loop(center, smooth)
    left = _offset_loop(cl, +half_w)
    right = _offset_loop(cl, -half_w)
    # Asphalt ribbon = left edge forward + right edge backward.
    ribbon = left + right[::-1]
    surfaces = [Surface(ribbon, ASPHALT, "road")]
    lines = [
        Line(left + [left[0]], WHITE, 0.20),
        Line(right + [right[0]], WHITE, 0.20),
        Line(cl + [cl[0]], YELLOW, 0.10, dash=True),
    ]
    # Start/finish across the track at spawn point.
    sx, sy = cl[spawn_idx]
    nxt = cl[(spawn_idx + 1) % len(cl)]
    head = math.atan2(nxt[0] - sx, 0) if False else math.atan2(nxt[1] - sy, nxt[0] - sx)
    # perpendicular line across width
    lx, ly = left[spawn_idx]
    rx, ry = right[spawn_idx]
    lines.append(Line([[lx, ly], [rx, ry]], KERB_B, 0.5))
    markers = [Marker("start_finish", sx, sy, head)]
    return Scenario(name, label, surfaces, lines, markers, spawn=(sx, sy, head))


def _smooth_open(pts: list[list[float]], iters: int = 2) -> list[list[float]]:
    """Chaikin corner-cutting for an OPEN polyline (endpoints preserved)."""
    for _ in range(iters):
        new = [pts[0]]
        for i in range(len(pts) - 1):
            x1, y1 = pts[i]
            x2, y2 = pts[i + 1]
            new.append([0.75 * x1 + 0.25 * x2, 0.75 * y1 + 0.25 * y2])
            new.append([0.25 * x1 + 0.75 * x2, 0.25 * y1 + 0.75 * y2])
        new.append(pts[-1])
        pts = new
    return pts


def _offset_polyline(pts: list[list[float]], dist: float) -> list[list[float]]:
    """Offset an OPEN polyline by `dist` along left normals (single-segment
    normal at the ends, averaged at interior vertices)."""
    n = len(pts)
    out: list[list[float]] = []
    for i in range(n):
        if i == 0 or i == n - 1:
            (ax, ay), (bx, by) = (pts[0], pts[1]) if i == 0 else (pts[-2], pts[-1])
            dx, dy = bx - ax, by - ay
            ln = math.hypot(dx, dy) or 1.0
            nx, ny = -dy / ln, dx / ln
        else:
            x0, y0 = pts[i - 1]
            x1, y1 = pts[i]
            x2, y2 = pts[i + 1]
            nx, ny = 0.0, 0.0
            for (ax, ay, bx, by) in ((x0, y0, x1, y1), (x1, y1, x2, y2)):
                dx, dy = bx - ax, by - ay
                ln = math.hypot(dx, dy) or 1.0
                nx += -dy / ln
                ny += dx / ln
            ln = math.hypot(nx, ny) or 1.0
            nx, ny = nx / ln, ny / ln
        x1, y1 = pts[i]
        out.append([x1 + dist * nx, y1 + dist * ny])
    return out


def _road(cl: list[list[float]], half_w: float, two_way: bool = True,
          closed: bool = False, smooth: int = 0) -> tuple[list[Surface], list[Line], list[list[float]]]:
    """Asphalt ribbon + edge lines (+ dashed centre if two_way) from a centerline.
    Returns (surfaces, lines, smoothed_centerline)."""
    if smooth:
        cl = _smooth_loop(cl, smooth) if closed else _smooth_open(cl, smooth)
    if closed:
        left = _offset_loop(cl, +half_w)
        right = _offset_loop(cl, -half_w)
        edges = [Line(left + [left[0]], WHITE, 0.16), Line(right + [right[0]], WHITE, 0.16)]
        centre = Line(cl + [cl[0]], WHITE, 0.10, dash=True)
    else:
        left = _offset_polyline(cl, +half_w)
        right = _offset_polyline(cl, -half_w)
        edges = [Line(left, WHITE, 0.14), Line(right, WHITE, 0.14)]
        centre = Line(cl, WHITE, 0.10, dash=True)
    surfs = [Surface(left + right[::-1], ASPHALT, "road")]
    return surfs, edges + ([centre] if two_way else []), cl


def _water(cl: list[list[float]], half_w: float, smooth: int = 2) -> Surface:
    cl = _smooth_open(cl, smooth)
    left = _offset_polyline(cl, +half_w)
    right = _offset_polyline(cl, -half_w)
    return Surface(left + right[::-1], WATER, "water")


def _walk(pts: list[list[float]], step: float) -> list[tuple[float, float, float]]:
    """Arc-length samples (x, y, heading) every `step` metres along a polyline."""
    seg = [math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
           for i in range(len(pts) - 1)]
    total = sum(seg)
    out: list[tuple[float, float, float]] = []
    d = step * 0.5
    while d < total:
        rem = d
        for i, ln in enumerate(seg):
            if rem <= ln:
                t = rem / (ln or 1.0)
                x0, y0 = pts[i]
                x1, y1 = pts[i + 1]
                out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t,
                            math.atan2(y1 - y0, x1 - x0)))
                break
            rem -= ln
        d += step
    return out


def _orect(cx: float, cy: float, along: float, depth: float, heading: float) -> list[list[float]]:
    """Oriented rectangle centred at (cx,cy), `along` parallel to heading, `depth` across."""
    ca, sa = math.cos(heading), math.sin(heading)
    base = [(-along / 2, -depth / 2), (along / 2, -depth / 2),
            (along / 2, depth / 2), (-along / 2, depth / 2)]
    return [[cx + px * ca - py * sa, cy + px * sa + py * ca] for px, py in base]


def _dist_pt_polyline(px: float, py: float, cl: list[list[float]]) -> float:
    best = 1e18
    for i in range(len(cl) - 1):
        ax, ay = cl[i]
        bx, by = cl[i + 1]
        dx, dy = bx - ax, by - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / l2))
        qx, qy = ax + t * dx, ay + t * dy
        best = min(best, math.hypot(px - qx, py - qy))
    return best


def _buildings_along(cl: list[list[float]], side: float, depth: float,
                     avoid: list[tuple[list[list[float]], float]],
                     keepout: list[tuple[float, float, float]], *,
                     step: float = 14.0, along: float = 11.0, seed: int = 0) -> list[Surface]:
    """Place building footprints in a row offset to one `side` of a street
    centerline, skipping any that would sit on another road (`avoid` = list of
    (centerline, clearance)) or inside a keep-out circle (cx, cy, r)."""
    out: list[Surface] = []
    for k, (x, y, h) in enumerate(_walk(cl, step)):
        nx, ny = -math.sin(h), math.cos(h)               # left normal of heading
        bx, by = x + side * nx, y + side * ny
        if any(math.hypot(bx - cx, by - cy) < r for cx, cy, r in keepout):
            continue
        if any(_dist_pt_polyline(bx, by, road) < clr for road, clr in avoid):
            continue
        d = depth * (0.78 + 0.30 * (((k * 53 + seed * 17) % 7) / 6.0))
        a = along * (0.82 + 0.30 * (((k * 31 + seed * 11) % 5) / 4.0))
        out.append(Surface(_orect(bx, by, a, d, h), BLDG[(k + seed) % len(BLDG)], "building"))
    return out


# ── scenario builders ────────────────────────────────────────────────────────

def _plaza() -> Scenario:
    half = 50.0
    surfaces = [
        Surface(_rect(0, 0, 2 * (half + 12), 2 * (half + 12)), GRASS, "grass"),
        Surface(_rect(0, 0, 2 * half, 2 * half), CONCRETE, "plaza"),
    ]
    lines = []
    # faint reference grid every 10 m (helps judge distance/scale)
    for d in range(-40, 41, 10):
        lines.append(Line([[-half, d], [half, d]], "#6b6f78", 0.06))
        lines.append(Line([[d, -half], [d, half]], "#6b6f78", 0.06))
    lines.append(Line(_rect(0, 0, 2 * half, 2 * half) + [[-half, -half]], WHITE, 0.2))
    return Scenario("plaza", "广场", surfaces, lines, [], spawn=(0, 0, 0))


def _town() -> Scenario:
    """Crowded medieval small-town network, inspired by the old-town street
    layout of Schwäbisch Gmünd: an oval ring road (former wall line) around a
    dense core of irregular lanes, a central market square, two church squares,
    and the Josefsbach brook. Recognisable in character, not survey-accurate."""
    surfaces: list[Surface] = []
    lines: list[Line] = []

    # ── ground base ──
    surfaces.append(Surface(_rect(0, 0, 480, 380), GRASS, "grass"))

    # ── street centerlines (x = E-W long axis, y = N-S) ──
    ring = [
        [168, 6], [150, 58], [112, 92], [60, 110], [6, 114],
        [-52, 110], [-108, 90], [-150, 46], [-162, -8], [-146, -60],
        [-98, -94], [-38, -108], [26, -106], [86, -92], [134, -60], [162, -26],
    ]
    axis = [[-152, 8], [-95, 4], [-40, 2], [18, 6], [72, 14], [120, 30], [152, 40]]
    north_st = [[-128, 52], [-70, 50], [-10, 52], [50, 56], [105, 64]]
    south_st = [[-126, -44], [-66, -46], [-6, -44], [54, -40], [104, -36]]
    cross1 = [[-110, 80], [-110, 30], [-112, -20], [-108, -72]]
    cross2 = [[-60, 96], [-58, 40], [-60, -10], [-58, -80]]
    cross3 = [[-6, 108], [-4, 40], [-6, -12], [-4, -92]]
    cross4 = [[52, 92], [54, 44], [52, -8], [54, -72]]
    cross5 = [[100, 76], [102, 36], [100, -18], [98, -58]]
    diag1 = [[-95, 4], [-78, 26], [-58, 40]]
    diag2 = [[18, 6], [38, -18], [54, -40]]
    bach = [[-160, -78], [-92, -72], [-22, -70], [40, -66], [104, -60], [160, -50]]

    # brook sits just above the ground, under the roads that bridge it
    surfaces.append(_water(bach, 2.5))

    # ── paved squares (drawn under the roads so streets read on top) ──
    markt = [[-36, -8], [40, -2], [44, 22], [-32, 16]]            # Marktplatz
    mplatz = [[-8, -42], [42, -38], [40, -15], [-10, -19]]        # Münsterplatz
    jplatz = [[-132, -8], [-100, -5], [-102, 20], [-134, 17]]     # Johannisplatz
    for sq in (markt, mplatz, jplatz):
        surfaces.append(Surface(sq, COBBLE, "plaza"))

    # ── roads (ring closed; the rest open). collect (centerline, half_w) ──
    main_streets = [(axis, 4.6), (north_st, 3.7), (south_st, 3.7)]
    lanes = [(cross1, 3.1), (cross2, 3.3), (cross3, 3.6), (cross4, 3.3),
             (cross5, 3.1), (diag1, 2.7), (diag2, 2.7)]

    rs, rl, ring_cl = _road(ring, 6.0, two_way=True, closed=True, smooth=2)
    surfaces += rs
    avoid: list[tuple[list[list[float]], float]] = [(ring_cl + [ring_cl[0]], 8.0)]
    smoothed: list[tuple[list[list[float]], float]] = []
    for cl, hw in main_streets:
        s, ln, c = _road(cl, hw, two_way=True, smooth=2)
        surfaces += s
        lines += ln
        smoothed.append((c, hw))
        avoid.append((c, hw + 5.5))
    for cl, hw in lanes:
        s, ln, c = _road(cl, hw, two_way=False, smooth=2)
        surfaces += s
        lines += ln
        smoothed.append((c, hw))
        avoid.append((c, hw + 5.0))
    avoid.append((_smooth_open(bach, 2), 6.0))     # don't build on the brook
    lines += rl                                     # ring edge/centre lines on top

    # ── landmark churches (taller masonry, drawn over the plazas) ──
    muenster = [[8, -37], [35, -34], [34, -19], [7, -22]]         # Heilig-Kreuz-Münster
    johannis = [[-126, -3], [-108, -1], [-109, 13], [-127, 11]]   # St. Johannis
    surfaces.append(Surface(muenster, STONE, "landmark"))
    surfaces.append(Surface(johannis, STONE, "landmark"))

    # ── buildings: dense rows along every street, auto-avoiding other roads ──
    keepout = [
        (4, 7, 30), (24, 12, 26),       # Marktplatz (elongated → two circles)
        (17, -28, 26),                  # Münsterplatz + Münster
        (-117, 6, 24),                  # Johannisplatz + church
    ]
    blds: list[Surface] = []
    # ring: build only on the inner side (toward the core / "inside the walls")
    blds += _buildings_along(ring_cl + [ring_cl[0]], +13.0, 11.0, avoid, keepout,
                             step=15.0, along=12.0, seed=2)
    # every inner street: a row on each side fills the blocks back-to-back
    for i, (cl, hw) in enumerate(smoothed):
        off = hw + 7.0
        blds += _buildings_along(cl, +off, 10.0, avoid, keepout, seed=i)
        blds += _buildings_along(cl, -off, 10.0, avoid, keepout, seed=i + 4)
    surfaces += blds

    # ── traffic lights at the main ring gates / junctions ──
    markers = [
        Marker("traffic_light", -150, 20, 0.0, {"state": "green"}),       # west gate
        Marker("traffic_light", 150, 26, math.pi, {"state": "red"}),      # east gate
        Marker("traffic_light", -18, 104, -math.pi / 2, {"state": "red"}),   # north (cross3)
        Marker("traffic_light", 8, -100, math.pi / 2, {"state": "green"}),   # south (cross3)
        Marker("traffic_light", -60, 96, -math.pi / 2, {"state": "green"}),  # north (cross2)
        Marker("traffic_light", 54, -72, math.pi / 2, {"state": "red"}),     # south (cross4)
    ]

    # spawn: west gate, on the market axis, facing into town (+x)
    return Scenario("town", "小镇 · 士瓦本格明德", surfaces, lines, markers,
                    spawn=(-144, 8, 0.0))


def _track_small() -> Scenario:
    """Compact handling circuit — a kidney loop, ~0.5 km."""
    center = [
        [60, 0], [70, 18], [60, 38], [30, 48], [0, 44],
        [-26, 50], [-54, 40], [-64, 16], [-58, -12], [-34, -26],
        [-4, -22], [22, -30], [50, -24],
    ]
    return _track_from_centerline("track_small", "小赛道", center, half_w=6.0, spawn_idx=0)


def _track_shanghai() -> Scenario:
    """Approximation of Shanghai International Circuit's signature layout
    (the 上-shaped infield + long back straight), scaled to a ~1.6 km lap.
    Recognisable, not survey-accurate."""
    s = 1.7  # scale
    center = [
        [120, 0], [150, 10], [165, 35],            # start straight → T1 entry
        [160, 60], [140, 78], [120, 86],           # T1-2 long right "snail"
        [104, 80], [96, 64], [104, 48],            # tightening into T3
        [124, 36], [134, 16], [120, -6],           # T3-4 back out
        [96, -16], [70, -10], [52, 4],             # T5 → T6 (left)
        [40, 26], [20, 36], [-6, 30],              # T7-8 esses
        [-26, 16], [-30, -8], [-48, -22],          # T9 hairpin-ish
        [-78, -20], [-104, -6], [-120, 16],        # T11-12 onto back straight
        [-110, 42], [-80, 52], [-44, 50],          # long back straight (T13 end)
        [-6, 46], [30, 40], [66, 28], [98, 14],    # sweep back to T14
    ]
    center = [[x * s, y * s] for x, y in center]
    return _track_from_centerline("track_shanghai", "上海国际赛车场(近似)",
                                  center, half_w=7.5, spawn_idx=0, smooth=3)


def _arc_points(cx: float, cy: float, r: float, th0: float, th1: float,
                n: int = 96) -> list[list[float]]:
    """Points along a circular arc of radius `r` centred at (cx,cy), from
    angle th0 to th1 (radians, math convention). Local copy so the environment
    module need not depend on controller.path."""
    return [[cx + r * math.cos(th0 + (th1 - th0) * i / n),
             cy + r * math.sin(th0 + (th1 - th0) * i / n)] for i in range(n + 1)]


def _arc_road(cx: float, cy: float, radius: float, th0: float, th1: float,
              n_lanes: int = 2, lane_width: float = 3.75,
              ) -> tuple[list[Surface], list[Line]]:
    """Exact circular-arc road: asphalt annulus + standard lane markings.

    Unlike `_road` (which offsets a polyline and smoothes it with Chaikin),
    this generator is built from true circles so the radius is exact and
    labelled — "different curvatures" is a measurable, reproducible property.

    Lane markings follow GB 5768.3:
        * solid white edge lines, 0.20 m wide, at the outer/inner pavement edges
        * dashed white lane dividers between lanes, 0.15 m wide, 6 m mark / 9 m gap
    Returns (surfaces, lines). The road is one-way (no yellow centre line).
    """
    half_w = n_lanes * lane_width / 2.0
    r_out = radius + half_w
    r_in = radius - half_w
    # Asphalt annulus: outer arc forward + inner arc reversed → filled ring.
    outer = _arc_points(cx, cy, r_out, th0, th1)
    inner = _arc_points(cx, cy, r_in, th0, th1)
    ribbon = outer + inner[::-1]
    surfaces = [Surface(ribbon, ASPHALT, "road")]
    lines: list[Line] = [
        Line(outer, WHITE, 0.20),
        Line(inner, WHITE, 0.20),
    ]
    # Interior lane dividers: one between each adjacent pair of lanes.
    for i in range(1, n_lanes):
        r_div = r_in + i * lane_width
        lines.append(Line(_arc_points(cx, cy, r_div, th0, th1),
                          WHITE, 0.15, dash_pattern=(6.0, 9.0)))
    return surfaces, lines


def _straight_road(x0: float, y0: float, length: float, heading: float,
                   n_lanes: int = 2, lane_width: float = 3.75,
                   ) -> tuple[list[Surface], list[Line]]:
    """Straight multi-lane road starting at (x0,y0) along `heading` (rad).

    Mirrors `_arc_road` for the straight section. Edge lines solid white 0.20 m;
    interior dividers dashed white 0.15 m, 6 m mark / 9 m gap (GB 5768.3)."""
    ca, sa = math.cos(heading), math.sin(heading)
    half_w = n_lanes * lane_width / 2.0
    # Forward (along heading) and left-normal vectors.
    fx, fy = ca, sa
    nx, ny = -sa, ca

    def at(s: float, off: float) -> list[float]:
        x = x0 + s * fx + off * nx
        y = y0 + s * fy + off * ny
        return [x, y]

    left = [at(0.0, +half_w), at(length, +half_w)]
    right = [at(0.0, -half_w), at(length, -half_w)]
    surfaces = [Surface(left + right[::-1], ASPHALT, "road")]
    lines: list[Line] = [
        Line(left, WHITE, 0.20),
        Line(right, WHITE, 0.20),
    ]
    for i in range(1, n_lanes):
        off = -half_w + i * lane_width
        lines.append(Line([at(0.0, off), at(length, off)],
                          WHITE, 0.15, dash_pattern=(6.0, 9.0)))
    return surfaces, lines


def _proving_ground() -> Scenario:
    """Proving ground — a flat, drivable road network for driving tests.

    Built to the requirements in docs/driving_experience_plan.md §C2:
    standard lane markings, multiple lanes, several exact-curvature corners,
    a long straight, a skidpad and an open handling pad. Flat ground only —
    no scenery meshes, no textures (rendering-load neutral). All corner radii
    are explicit parameters, so curvature is labelled and reproducible.

    Layout (≈ 1300 × 500 m, +x forward, +y left):
        ① 800 m × 3-lane high-speed straight along +X from the origin
        ② a return loop of 5 exact-curvature corners (R = 150/100/60/40/25)
           linking the far end of the straight back toward the start
        ③ a R = 30 m skidpad (concentric lane lines) east of the straight end
        ④ a 200 × 60 m open handling pad for slalom / lane-change
    """
    n_lanes = 3
    lane_width = 3.75
    straight_len = 800.0

    surfaces: list[Surface] = []
    lines: list[Line] = []
    markers: list[Marker] = []

    # ground base (flat grass under everything)
    surfaces.append(Surface(_rect(400, -260, 1300, 500), GRASS, "grass"))

    # ① High-speed straight: 3 lanes along +X from (0,0).
    s_straight, l_straight = _straight_road(0.0, 0.0, straight_len, 0.0,
                                            n_lanes=n_lanes, lane_width=lane_width)
    surfaces += s_straight
    lines += l_straight
    # Distance posts every 50 m (driving-relevant, not decoration).
    for m in range(50, int(straight_len), 50):
        # pair of posts just outside each edge line
        markers.append(Marker("distance", m, +(n_lanes * lane_width) / 2 + 1.5,
                              0.0, {"m": m}))
        markers.append(Marker("distance", m, -(n_lanes * lane_width) / 2 - 1.5,
                              0.0, {"m": m}))

    # ② Return loop: 5 exact-curvature corners joining the far end of the
    #    straight back to the start. Each corner is a true circular arc with
    #    an explicit radius; a labelled anchor records (centre, radius) so the
    #    radius is measurable and reproducible. The corners are laid out as a
    #    descending-radius sequence south of the straight.
    corners = [
        # (centre_x, centre_y, radius, th0, th1, label)
        # Angles in math convention; we route clockwise (decreasing angle) so
        # each arc leaves the straight end heading +x and curves down/right.
        (820.0,  -60.0, 150.0, math.pi / 2,  0.0,            "corner_R150"),
        (820.0, -210.0, 100.0, math.pi,      math.pi / 2,    "corner_R100"),
        (720.0, -210.0,  60.0, 0.0,         -math.pi / 2,    "corner_R60"),
        (660.0, -150.0,  40.0, -math.pi / 2,-math.pi,        "corner_R40"),
        (620.0,  -60.0,  25.0, 0.0,          math.pi / 2,    "corner_R25"),
    ]
    anchors: dict[str, tuple[float, float, float]] = {}
    for (ccx, ccy, crad, cth0, cth1, clabel) in corners:
        cs, cl = _arc_road(ccx, ccy, crad, cth0, cth1,
                           n_lanes=n_lanes, lane_width=lane_width)
        surfaces += cs
        lines += cl
        # entry marker + radius anchor (centre pose; nominal radius in meta)
        ex = ccx + crad * math.cos(cth0)
        ey = ccy + crad * math.sin(cth0)
        markers.append(Marker("distance", ex, ey, 0.0,
                              {"m": 0, "corner": clabel, "radius": crad}))
        anchors[clabel] = (ccx, ccy, 0.0)

    # ③ Skidpad: R = 30 m circle with concentric inner/outer lane lines,
    #    east of the straight end so it doesn't overlap.
    skid_cx, skid_cy, skid_r = 940.0, -60.0, 30.0
    # Pavement ring a bit wider than the circle for runoff; inner/outer edges.
    skid_outer = _arc_points(skid_cx, skid_cy, skid_r + 3.0, 0.0, 2 * math.pi)
    skid_inner = _arc_points(skid_cx, skid_cy, skid_r - 3.0, 0.0, 2 * math.pi)
    surfaces.append(Surface(skid_outer + skid_inner[::-1], ASPHALT, "road"))
    lines.append(Line(skid_outer, WHITE, 0.20))
    lines.append(Line(skid_inner, WHITE, 0.20))
    # the nominal circle itself as a dashed reference
    lines.append(Line(_arc_points(skid_cx, skid_cy, skid_r, 0.0, 2 * math.pi),
                      YELLOW, 0.12, dash_pattern=(3.0, 3.0)))
    anchors["skidpad"] = (skid_cx - skid_r - 4.0, skid_cy, 0.0)

    # ④ Open handling pad: 200 × 60 m of empty pavement for slalom / DLC.
    pad_cx, pad_cy = 940.0, -260.0
    pad_w, pad_h = 200.0, 60.0
    surfaces.append(Surface(_rect(pad_cx, pad_cy, pad_w, pad_h), CONCRETE, "plaza"))
    lines.append(Line(_rect(pad_cx, pad_cy, pad_w, pad_h) + [
        [pad_cx - pad_w / 2, pad_cy - pad_h / 2]], WHITE, 0.20))
    anchors["handling"] = (pad_cx - pad_w / 2 + 5.0, pad_cy, 0.0)

    # straight anchor at the start of the straight (just inside the origin)
    anchors["straight_start"] = (8.0, 0.0, 0.0)
    # a second straight anchor at the far end (for acceleration-then-brake runs)
    anchors["straight_mid"] = (400.0, 0.0, 0.0)

    spawns = [
        Spawn("直线起点", 8.0, 0.0, 0.0),
        Spawn("环路入口", 820.0, 90.0, 0.0),
        Spawn("定圆入口", float(anchors["skidpad"][0]), float(anchors["skidpad"][1]), 0.0),
        Spawn("操控区", float(anchors["handling"][0]), float(anchors["handling"][1]), 0.0),
    ]

    return Scenario(
        name="proving_ground",
        label="试验场",
        surfaces=surfaces,
        lines=lines,
        markers=markers,
        spawn=(8.0, 0.0, 0.0),
        spawns=spawns,
        anchors=anchors,
    )


_BUILDERS = {
    "plaza": _plaza,
    "town": _town,
    "track_small": _track_small,
    "track_shanghai": _track_shanghai,
    "proving_ground": _proving_ground,
}

# ── module-level active scenario (mirrors controller.path) ────────────────────
_active: Scenario | None = None
_version: int = 0


def list_scenarios() -> list[dict]:
    return [{"name": n, "label": _BUILDERS[n]().label} for n in _BUILDERS]


def get_active() -> Scenario | None:
    return _active


def get_version() -> int:
    return _version


def load_scenario(name: str) -> int:
    global _active, _version
    if name not in _BUILDERS:
        raise KeyError(name)
    _active = _BUILDERS[name]()
    _version += 1
    return _version


def clear_scenario() -> int:
    global _active, _version
    _active = None
    _version += 1
    return _version
