"""Kinematics & Compliance — the suspension's measured characteristic.

This is the interface a chassis department actually works through. A K&C rig
sweeps each corner through its travel and records how the wheel moves (toe,
camber, caster, track), then pushes/pulls at the contact patch and records how
much the wheel deflects under load. Those two families of curves ARE the
suspension, as far as handling is concerned — which is why CarSim models
suspension by table lookup rather than by solving a linkage. Tables are both
faster and, crucially, directly measurable.

Before this module the platform had:

    * camber as a CONSTANT — no camber gain in roll at all
    * bump steer as a single linear coefficient — no real toe curve
    * compliance: nothing whatsoever

That last one is the significant gap. On a real car, compliance steer
contributes on the same order as geometry to handling balance, and it is one
of the main things chassis engineers tune. Without it, the only way to reach a
production-like understeer gradient was to back out an axle cornering-stiffness
ratio — fitting a number rather than modelling a mechanism.

For a four-wheel-steer vehicle it matters more, not less. The premise of
decoupled steering is "I can place each wheel where I want it"; compliance
means the angle you get differs from the angle you asked for, load-dependently
and independently at each corner.

Units
-----
Files are written in ENGINEERING units — degrees, millimetres, kN — because
that is what rig reports use and a human copying numbers out of one should not
have to convert. Everything is converted to SI on load; nothing downstream
sees a degree.

Sign conventions (matching the rest of the platform)
----------------------------------------------------
    jounce   + = suspension compressing (wheel moves up relative to body)
    toe      + = toe-IN at the axle level; the per-wheel split (left −,
                 right +) is applied by the caller, same as static toe
    camber   + = top of the wheel leaning out, per `SuspensionParams.camber`
    forces   Fy, Fx, Fz in the wheel frame, newtons
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

DEG = math.pi / 180.0
MM = 1e-3
KN = 1e3


@dataclass(frozen=True)
class KinematicCurve:
    """One measured quantity vs suspension travel, linearly interpolated.

    A rig produces a sweep of discrete points; linear interpolation between
    them is the honest reading of that data. Extrapolation is deliberately
    flat (numpy's default) rather than linear: past the measured travel the
    curve is unknown, and holding the last value is the conservative choice —
    a linear extrapolation of a curve that was about to turn over invents
    behaviour the rig never saw.
    """

    jounce_m: tuple[float, ...] = ()
    value: tuple[float, ...] = ()

    @property
    def is_empty(self) -> bool:
        return len(self.jounce_m) < 2

    def at(self, jounce: np.ndarray) -> np.ndarray:
        j = np.asarray(jounce, dtype=np.float64)
        if self.is_empty:
            return np.zeros_like(j)
        return np.interp(j, self.jounce_m, self.value)


@dataclass(frozen=True)
class ComplianceCoeffs:
    """Wheel deflection per unit applied force/moment [SI: rad/N, rad/(N·m), m/N].

    Defaults of zero mean a rigid suspension, i.e. the platform's behaviour
    before this existed.
    """

    toe_per_fy: float = 0.0        # rad/N   — lateral force steer
    toe_per_fx: float = 0.0        # rad/N   — longitudinal force steer
    toe_per_mz: float = 0.0        # rad/N·m — aligning-moment steer
    camber_per_fy: float = 0.0     # rad/N
    camber_per_fz: float = 0.0     # rad/N
    lat_per_fy: float = 0.0        # m/N     — lateral wheel-centre deflection
    long_per_fx: float = 0.0       # m/N     — longitudinal deflection

    @property
    def is_rigid(self) -> bool:
        return all(v == 0.0 for v in (
            self.toe_per_fy, self.toe_per_fx, self.toe_per_mz,
            self.camber_per_fy, self.camber_per_fz,
            self.lat_per_fy, self.long_per_fx,
        ))


@dataclass(frozen=True)
class AxleKC:
    """One axle's measured K&C characteristic."""

    name: str = ""
    source: str = ""
    toe: KinematicCurve = field(default_factory=KinematicCurve)
    camber: KinematicCurve = field(default_factory=KinematicCurve)
    caster: KinematicCurve = field(default_factory=KinematicCurve)
    half_track: KinematicCurve = field(default_factory=KinematicCurve)
    wheelbase: KinematicCurve = field(default_factory=KinematicCurve)
    compliance: ComplianceCoeffs = field(default_factory=ComplianceCoeffs)

    @property
    def is_empty(self) -> bool:
        """True when this axle carries no data — the caller then keeps the
        platform's pre-K&C behaviour exactly."""
        return (self.toe.is_empty and self.camber.is_empty
                and self.caster.is_empty and self.half_track.is_empty
                and self.wheelbase.is_empty and self.compliance.is_rigid)


@dataclass(frozen=True)
class VehicleKC:
    """Front + rear axle K&C. An all-empty instance is the 'no data' case."""

    front: AxleKC = field(default_factory=AxleKC)
    rear: AxleKC = field(default_factory=AxleKC)

    @property
    def is_empty(self) -> bool:
        return self.front.is_empty and self.rear.is_empty

    def per_wheel_toe(self, jounce: np.ndarray) -> np.ndarray:
        """Kinematic toe change per wheel [rad], FL/FR/RL/RR.

        The curve is the AXLE-level toe-in, matching how rig reports quote it
        and matching `static_toe_front/rear`; the left/right split (left −,
        right +) is the same convention `static_toe_offsets` uses, so the two
        simply add.
        """
        f = self.front.toe.at(jounce[:2])
        r = self.rear.toe.at(jounce[2:])
        return np.array([-f[0], +f[1], -r[0], +r[1]], dtype=np.float64)

    def per_wheel_camber(self, jounce: np.ndarray) -> np.ndarray:
        """Kinematic camber change per wheel [rad], mirrored left/right."""
        f = self.front.camber.at(jounce[:2])
        r = self.rear.camber.at(jounce[2:])
        return np.array([+f[0], -f[1], +r[0], -r[1]], dtype=np.float64)

    def per_wheel_compliance_toe(
        self, fx: np.ndarray, fy: np.ndarray, mz: np.ndarray,
    ) -> np.ndarray:
        """Compliance toe change per wheel [rad].

        Fy/Mz are already signed per wheel in the wheel frame, so unlike the
        kinematic curves these need no left/right mirroring — a lateral force
        pointing left deflects the wheel the same way whichever side it is on.
        """
        out = np.zeros(4, dtype=np.float64)
        for i, c in ((0, self.front.compliance), (1, self.front.compliance),
                     (2, self.rear.compliance), (3, self.rear.compliance)):
            out[i] = (c.toe_per_fy * float(fy[i])
                      + c.toe_per_fx * float(fx[i])
                      + c.toe_per_mz * float(mz[i]))
        return out

    def per_wheel_compliance_camber(
        self, fy: np.ndarray, fz: np.ndarray,
    ) -> np.ndarray:
        out = np.zeros(4, dtype=np.float64)
        for i, c in ((0, self.front.compliance), (1, self.front.compliance),
                     (2, self.rear.compliance), (3, self.rear.compliance)):
            out[i] = c.camber_per_fy * float(fy[i]) + c.camber_per_fz * float(fz[i])
        return out


# ─── loading ────────────────────────────────────────────────────────────────

def _curve(raw: dict[str, Any], jounce_key: str, value_key: str,
           value_scale: float) -> KinematicCurve:
    j = raw.get(jounce_key)
    v = raw.get(value_key)
    if not j or not v:
        return KinematicCurve()
    if len(j) != len(v):
        raise ValueError(
            f"K&C curve '{value_key}' has {len(v)} points but "
            f"'{jounce_key}' has {len(j)} — a rig sweep must be paired"
        )
    order = np.argsort(np.asarray(j, dtype=np.float64))
    jj = np.asarray(j, dtype=np.float64)[order] * MM
    vv = np.asarray(v, dtype=np.float64)[order] * value_scale
    return KinematicCurve(jounce_m=tuple(jj.tolist()), value=tuple(vv.tolist()))


def axle_from_dict(raw: dict[str, Any] | None, name: str = "") -> AxleKC:
    """Build one axle's K&C from the engineering-unit file form."""
    if not raw:
        return AxleKC(name=name)
    kin = raw.get("kinematics") or {}
    comp = raw.get("compliance") or {}
    jk = "jounce_mm"
    return AxleKC(
        name=str(raw.get("name", name)),
        source=str(raw.get("source", "")),
        toe=_curve(kin, jk, "toe_deg", DEG),
        camber=_curve(kin, jk, "camber_deg", DEG),
        caster=_curve(kin, jk, "caster_deg", DEG),
        half_track=_curve(kin, jk, "half_track_mm", MM),
        wheelbase=_curve(kin, jk, "wheelbase_mm", MM),
        compliance=ComplianceCoeffs(
            # deg/kN → rad/N, deg/kN·m → rad/N·m, mm/kN → m/N
            toe_per_fy=float(comp.get("toe_per_fy_deg_per_kN", 0.0)) * DEG / KN,
            toe_per_fx=float(comp.get("toe_per_fx_deg_per_kN", 0.0)) * DEG / KN,
            toe_per_mz=float(comp.get("toe_per_mz_deg_per_kNm", 0.0)) * DEG / KN,
            camber_per_fy=float(comp.get("camber_per_fy_deg_per_kN", 0.0)) * DEG / KN,
            camber_per_fz=float(comp.get("camber_per_fz_deg_per_kN", 0.0)) * DEG / KN,
            lat_per_fy=float(comp.get("lat_per_fy_mm_per_kN", 0.0)) * MM / KN,
            long_per_fx=float(comp.get("long_per_fx_mm_per_kN", 0.0)) * MM / KN,
        ),
    )


def kc_from_dict(raw: dict[str, Any] | None) -> VehicleKC:
    """Build a VehicleKC from a `kc:` section. None/empty → the rigid default."""
    if not raw:
        return VehicleKC()
    return VehicleKC(
        front=axle_from_dict(raw.get("front"), "front"),
        rear=axle_from_dict(raw.get("rear"), "rear"),
    )


def load_kc_profile(path: str | Path) -> VehicleKC:
    """Load a `kc_profiles/*.yaml` file."""
    import yaml
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    return kc_from_dict(raw.get("kc", raw))


def kc_from_bump_steer_coeff(coeff: float, travel_m: float = 0.10) -> VehicleKC:
    """Back-compat shim: synthesise a linear toe curve from `bump_steer_coeff`.

    The old model had one number — toe per metre of travel, same on both axles.
    Expressing it as a two-point curve means the K&C path can subsume it
    without changing any result, so the legacy parameter keeps working while
    the table interface becomes the single code path.
    """
    if not coeff:
        return VehicleKC()
    j = (-travel_m, travel_m)
    v = (-coeff * travel_m, coeff * travel_m)
    curve = KinematicCurve(jounce_m=j, value=v)
    axle = AxleKC(name="legacy_bump_steer", source="bump_steer_coeff", toe=curve)
    return VehicleKC(front=axle, rear=axle)
