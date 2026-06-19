"""Road & disturbance system.

A `Disturbance` is a geometric region in world frame that modifies the
ground / friction conditions felt by a wheel passing through it.

Phase 2a (this module) only handles **μ-class** disturbances — they change
the friction coefficient or load offset without modifying the road's z
profile. This means they're compatible with the planar KinematicModel
(which ignores them) *and* the SimplifiedDynamicModel coming in step 10
(which reads the absolute mu_override and fz_offset when computing tyre forces).

Phase 2b (step 12) adds `SpeedBump` and `Slope` which need a dynamic
model to be meaningful.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, ClassVar

import numpy as np


@dataclass
class WheelEnvLocal:
    """Local ground conditions at one wheel contact patch.

    `mu_override` is an **absolute** friction coefficient that replaces the
    scene's base μ when a friction disturbance (ice patch / split-μ) covers the
    wheel. None means "no friction disturbance here → use base μ". When several
    friction regions overlap a wheel, the most slippery (minimum) wins.
    """
    mu_override: float | None = None
    fz_offset: float = 0.0     # additive [N]
    ground_z: float = 0.0      # [m] — Phase 2b


class Disturbance(ABC):
    """Abstract disturbance region."""
    type_id: ClassVar[str] = "base"
    id: str

    @abstractmethod
    def query(self, wheel_world_pos: np.ndarray) -> WheelEnvLocal | None:
        """Return per-wheel local env if the wheel is inside this region,
        else None. Caller aggregates across all disturbances.
        """

    def serialize(self) -> dict[str, Any]:
        """JSON-friendly snapshot for the frontend to draw."""
        raise NotImplementedError


def _inside_oriented_rect(
    px: float, py: float,
    cx: float, cy: float,
    width: float, length: float, heading: float,
) -> tuple[bool, float, float]:
    """Test if world point (px, py) is inside an oriented rectangle.

    Returns (inside, u_along, v_across) where u_along ∈ [-length/2, +length/2]
    and v_across ∈ [-width/2, +width/2] are the local coordinates inside the
    rectangle (relative to its centre, x-aligned with the rectangle's heading).
    """
    dx = px - cx
    dy = py - cy
    c = math.cos(-heading)
    s = math.sin(-heading)
    u = c * dx - s * dy
    v = s * dx + c * dy
    inside = (abs(u) <= length / 2.0) and (abs(v) <= width / 2.0)
    return inside, u, v


# ---------------------------------------------------------------------------
# Concrete disturbances — Phase 2a (mu only)
# ---------------------------------------------------------------------------


@dataclass
class IcePatch(Disturbance):
    """Rectangular low-mu patch with an **absolute** friction coefficient.

    Half-width / half-length use the rectangle's local axes; `heading` rotates
    the rectangle in world frame (0 = aligned with world X).
    """
    type_id: ClassVar[str] = "ice_patch"

    id: str
    x: float
    y: float
    width: float                  # cross-axis [m]
    length: float                 # along-axis [m]
    mu: float = 0.3               # absolute μ inside the patch (0.3 = wet/icy)
    heading: float = 0.0          # [rad]

    def query(self, wheel_world_pos: np.ndarray) -> WheelEnvLocal | None:
        inside, _u, _v = _inside_oriented_rect(
            float(wheel_world_pos[0]), float(wheel_world_pos[1]),
            self.x, self.y, self.width, self.length, self.heading,
        )
        if not inside:
            return None
        return WheelEnvLocal(mu_override=self.mu)

    def serialize(self) -> dict[str, Any]:
        return {
            "type": self.type_id, "id": self.id,
            "x": self.x, "y": self.y,
            "width": self.width, "length": self.length,
            "heading": self.heading, "mu": self.mu,
        }


@dataclass
class SplitMu(Disturbance):
    """Split-µ road — left half has mu_left, right half has mu_right, both
    **absolute** friction coefficients.

    "Left" = positive v_across in the rectangle's local frame, "right" =
    negative. So with heading=0 (aligned with world X) and a vehicle driving
    in +X direction, the left side of the rectangle is in +Y world.
    """
    type_id: ClassVar[str] = "split_mu"

    id: str
    x: float
    y: float
    width: float
    length: float
    mu_left: float = 0.9
    mu_right: float = 0.3
    heading: float = 0.0

    def query(self, wheel_world_pos: np.ndarray) -> WheelEnvLocal | None:
        inside, _u, v = _inside_oriented_rect(
            float(wheel_world_pos[0]), float(wheel_world_pos[1]),
            self.x, self.y, self.width, self.length, self.heading,
        )
        if not inside:
            return None
        mu = self.mu_left if v >= 0.0 else self.mu_right
        return WheelEnvLocal(mu_override=mu)

    def serialize(self) -> dict[str, Any]:
        return {
            "type": self.type_id, "id": self.id,
            "x": self.x, "y": self.y,
            "width": self.width, "length": self.length,
            "heading": self.heading,
            "mu_left": self.mu_left, "mu_right": self.mu_right,
        }


# ---------------------------------------------------------------------------
# Scene container (lives on EnvironmentState)
# ---------------------------------------------------------------------------


@dataclass
class SpeedBump(Disturbance):
    """Transient vertical-force pulse over a thin strip.

    Modelled as a raised-cosine `fz_offset` profile across the length of
    the bump. When the wheel is inside, fz_offset adds a positive load
    proportional to a "spring rate" times the local bump height.
    """
    type_id: ClassVar[str] = "speed_bump"

    id: str
    x: float
    y: float
    width: float                  # cross-track length [m]
    length: float                 # along-track length [m] (typical: 0.3-0.8)
    height: float = 0.05          # bump peak height [m]
    stiffness: float = 1.0e5      # Fz pulse = stiffness × profile [N/m].
    # NOTE: peak fz_offset = stiffness × height. Keep this in the few-kN range
    # (≈ static wheel load) — far larger values make the planar model's tyre
    # forces / wheel-spin dynamics go unstable. The dynamic model additionally
    # clamps total Fz as a safety net (see SimplifiedDynamicModel).
    heading: float = 0.0

    def query(self, wheel_world_pos: np.ndarray) -> WheelEnvLocal | None:
        inside, u, _v = _inside_oriented_rect(
            float(wheel_world_pos[0]), float(wheel_world_pos[1]),
            self.x, self.y, self.width, self.length, self.heading,
        )
        if not inside:
            return None
        # Raised cosine height profile: peak at centre, zero at edges.
        u_norm = 2.0 * u / max(self.length, 1e-6)         # in [-1, +1]
        z = self.height * 0.5 * (1.0 + math.cos(math.pi * u_norm))
        return WheelEnvLocal(fz_offset=self.stiffness * z, ground_z=z)

    def serialize(self) -> dict[str, Any]:
        return {
            "type": self.type_id, "id": self.id,
            "x": self.x, "y": self.y,
            "width": self.width, "length": self.length,
            "heading": self.heading,
            "height": self.height, "stiffness": self.stiffness,
        }


@dataclass
class Slope(Disturbance):
    """Constant-grade slope region.

    The dynamic model interprets this as a longitudinal gravity component
    along the slope's heading. Lateral and vertical effects are ignored
    (the bump/swap doesn't add roll/pitch — the planar model has none).
    Inside the region, `ground_z` rises linearly along the heading axis.
    """
    type_id: ClassVar[str] = "slope"

    id: str
    x: float
    y: float
    width: float
    length: float
    angle: float = 0.0        # grade angle [rad] — positive = uphill in +heading
    heading: float = 0.0      # direction of "uphill"

    def query(self, wheel_world_pos: np.ndarray) -> WheelEnvLocal | None:
        inside, u, _v = _inside_oriented_rect(
            float(wheel_world_pos[0]), float(wheel_world_pos[1]),
            self.x, self.y, self.width, self.length, self.heading,
        )
        if not inside:
            return None
        # Local z increases linearly along the slope axis
        z = (u + self.length / 2.0) * math.tan(self.angle)
        # Convey the slope angle via fz_offset as a small reduction (cos approx)
        # and via ground_z (model can read both). Sign: positive angle reduces fz
        # very slightly (cos(angle) → 1 for small).
        return WheelEnvLocal(
            fz_offset=0.0,        # body weight handled by dynamic model
            ground_z=z,
        )

    def serialize(self) -> dict[str, Any]:
        return {
            "type": self.type_id, "id": self.id,
            "x": self.x, "y": self.y,
            "width": self.width, "length": self.length,
            "heading": self.heading, "angle": self.angle,
        }


_REGISTRY: dict[str, type[Disturbance]] = {
    IcePatch.type_id: IcePatch,
    SplitMu.type_id: SplitMu,
    SpeedBump.type_id: SpeedBump,
    Slope.type_id: Slope,
}


def disturbance_from_dict(d: dict[str, Any]) -> Disturbance:
    """Build a Disturbance from a YAML-parsed dict."""
    t = d.get("type")
    if t not in _REGISTRY:
        raise ValueError(f"Unknown disturbance type: {t!r}")
    cls = _REGISTRY[t]
    # Pop 'type' and pass the rest as kwargs.
    payload = {k: v for k, v in d.items() if k != "type"}
    payload.setdefault("id", f"{t}_{id(d):x}")
    return cls(**payload)


@dataclass
class Scene:
    """Aggregate of all disturbances + the base surface conditions."""
    base_mu: float = 0.85   # dry concrete default
    surface: str = "flat"
    disturbances: list[Disturbance] = field(default_factory=list)

    def wheel_env(self, wheel_world_pos: np.ndarray) -> WheelEnvLocal:
        """Combine all disturbances at the given wheel position.

        Friction is an absolute override: if any friction region covers the
        wheel, the most slippery (minimum μ) wins; otherwise mu_override stays
        None (caller uses base μ). fz / ground_z effects sum.
        """
        local = WheelEnvLocal()
        for d in self.disturbances:
            r = d.query(wheel_world_pos)
            if r is None:
                continue
            if r.mu_override is not None:
                local.mu_override = (
                    r.mu_override if local.mu_override is None
                    else min(local.mu_override, r.mu_override)
                )
            local.fz_offset += r.fz_offset
            local.ground_z += r.ground_z
        return local

    def effective_mu(self, wheel_world_pos: np.ndarray) -> float:
        """Absolute μ at this wheel: a friction region's value if present, else base_mu."""
        ov = self.wheel_env(wheel_world_pos).mu_override
        return ov if ov is not None else self.base_mu

    def serialize(self) -> dict[str, Any]:
        return {
            "base_mu": self.base_mu,
            "surface": self.surface,
            "disturbances": [d.serialize() for d in self.disturbances],
        }

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> Scene:
        return cls(
            base_mu=float(d.get("base_mu", 1.0)),
            surface=str(d.get("surface", "flat")),
            disturbances=[disturbance_from_dict(x) for x in (d.get("disturbances") or [])],
        )
