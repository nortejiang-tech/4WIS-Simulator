"""Tire models.

Phase 2 ships with a **linear-with-friction-circle** model: longitudinal and
lateral forces are linear in slip ratio / slip angle respectively, then the
combined force magnitude is capped at μ·Fz (the friction circle).

`PacejkaTireModel` adds a simplified Magic Formula (B/C/D/E per axis +
friction-ellipse combined-slip clipping). Select via
`VehicleParams.tire_model = "linear" | "pacejka"` and `make_tire()`.

Slip convention (matches Pacejka / SAE):
    α (slip angle) [rad] — angle between wheel rolling axis and actual
                            wheel velocity. Positive when wheel moves to the
                            left of its rolling direction.
    κ (slip ratio) [-]   — (r·ω - vx_wheel) / vx_wheel, with sign such that
                            κ > 0 when wheel is driving (spinning faster than
                            ground), κ < 0 when braking.

Force convention (returned in wheel-aligned frame):
    Fx > 0  → wheel pushes vehicle forward (along rolling axis)
    Fy > 0  → wheel pushes vehicle toward its left side (positive lateral)
    Mz > 0  → CCW moment about wheel vertical axis (self-aligning torque)
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass


def pacejka_pure_slip_forces(
    *,
    alpha: float,
    kappa: float,
    fz: float,
    mu: float,
    c_alpha: float,
    c_kappa: float,
    cx: float,
    cy: float,
    ex: float,
    ey: float,
) -> tuple[float, float]:
    """Pacejka Magic Formula pure-slip Fx, Fy before combined-slip clipping.

    Small-slip slopes match ``c_kappa`` and ``c_alpha`` by deriving B from
    ``D = mu*fz``. This helper is stateless so load-analysis can use per-wheel
    load-sensitive stiffness without constructing temporary model objects.
    """

    d = max(float(mu) * float(fz), 1e-3)
    bx = float(c_kappa) / (float(cx) * d)
    by = float(c_alpha) / (float(cy) * d)
    xk = bx * float(kappa)
    xa = by * float(alpha)
    fx0 = d * math.sin(float(cx) * math.atan(xk - float(ex) * (xk - math.atan(xk))))
    fy0 = -d * math.sin(float(cy) * math.atan(xa - float(ey) * (xa - math.atan(xa))))
    return fx0, fy0


def friction_ellipse_clip(
    fx: float,
    fy: float,
    *,
    fz: float,
    mu: float,
) -> tuple[float, float]:
    """Clip Fx/Fy to the combined-slip friction ellipse."""

    d = max(float(mu) * float(fz), 1e-3)
    r = (float(fx) / d) ** 2 + (float(fy) / d) ** 2
    if r > 1.0:
        scale = 1.0 / math.sqrt(r)
        return float(fx) * scale, float(fy) * scale
    return float(fx), float(fy)


def pacejka_combined_forces(
    *,
    alpha: float,
    kappa: float,
    fz: float,
    mu: float,
    c_alpha: float,
    c_kappa: float,
    cx: float,
    cy: float,
    ex: float,
    ey: float,
) -> tuple[float, float]:
    """Pacejka pure-slip forces followed by friction-ellipse clipping."""

    fx0, fy0 = pacejka_pure_slip_forces(
        alpha=alpha,
        kappa=kappa,
        fz=fz,
        mu=mu,
        c_alpha=c_alpha,
        c_kappa=c_kappa,
        cx=cx,
        cy=cy,
        ex=ex,
        ey=ey,
    )
    return friction_ellipse_clip(fx0, fy0, fz=fz, mu=mu)


def _magic_formula_peak_x(c: float, e: float) -> float:
    """Slip argument x = B·slip at which D·sin(C·atan(y(x))) peaks.

    With y(x) = x − E·(x − atan x), the derivative vanishes where
    cos(C·atan y) = 0, i.e. y(x*) = tan(π/(2C)). C and E are fixed shape
    factors, so x* is a constant of the tyre — solved once at construction and
    reused, which is what makes the peak-slip query cheap enough to run every
    step on every wheel.
    """
    if c <= 1e-6:
        return float("inf")
    target = math.tan(math.pi / (2.0 * c))
    x = max(target, 0.1)
    for _ in range(64):                      # Newton, converges in a handful
        y = x - e * (x - math.atan(x))
        dy = 1.0 - e * (1.0 - 1.0 / (1.0 + x * x))
        if abs(dy) < 1e-12:
            break
        step = (y - target) / dy
        x -= step
        if abs(step) < 1e-14:
            break
    return abs(x)


class TireModel(ABC):
    @abstractmethod
    def forces(self, alpha: float, kappa: float, fz: float, mu: float) -> tuple[float, float, float]:
        """Return (Fx, Fy, Mz_self_aligning) in wheel-aligned frame."""

    @abstractmethod
    def peak_slips(self, fz: float, mu: float) -> tuple[float, float]:
        """Return (|α| , |κ|) at which pure-slip force stops rising [rad, -].

        This is what separates "using 95% of the grip on the way up" from
        "using 95% on the way down". Both report the same utilisation, but the
        first is controllable and the second is departing — utilisation alone
        cannot tell a driver or a log which one they are in, so every grip
        readout in the platform pairs it with a beyond-peak flag derived from
        these.
        """


@dataclass
class LinearTireModel(TireModel):
    """Linear-in-slip with friction-circle saturation.

    Defaults calibrated for the 智己 LS9 default vehicle (~2900 kg, ~7 kN/tyre
    static, large 21" tyres). Cornering / longitudinal stiffness scale with
    vertical load, so these are higher than a mid-size sedan's. At the LS9's
    static load this keeps steady-state cornering side-slip small (max |α| ≈
    3-4° at 8 m/s) while still saturating realistically at the friction limit.

    Load sensitivity (C3-1): with ``fz_nom > 0`` and ``load_exp > 0`` the
    cornering stiffness scales as c_α(Fz) = c_α0·(Fz/Fz_nom)^p — the same law
    the quasi-static load page uses. This is what makes lateral load transfer
    reduce an axle's total lateral capacity (the physical root of the
    understeer/oversteer budget); with it off, transfer only moves force
    between wheels without changing the axle sum in the linear region.
    Longitudinal stiffness stays constant — the wheel-speed servo loop is
    tuned against c_κ and must not wander with load.
    """
    c_alpha: float = 120_000.0     # cornering stiffness [N/rad] at Fz_nom
    c_kappa: float = 100_000.0     # longitudinal stiffness [N] (servo gains tuned for this)
    t_pneumatic: float = 0.03      # pneumatic trail [m]
    fz_nom: float = 0.0            # nominal per-wheel load [N]; 0 = load sensitivity off
    load_exp: float = 0.0          # exponent p in (Fz/Fz_nom)^p; 0 = off

    def c_alpha_eff(self, fz: float) -> float:
        if self.fz_nom <= 0.0 or self.load_exp <= 0.0:
            return self.c_alpha
        ratio = max(fz / self.fz_nom, 1e-3)
        return self.c_alpha * ratio ** self.load_exp

    def forces(self, alpha: float, kappa: float, fz: float, mu: float) -> tuple[float, float, float]:
        # Linear pure-slip forces
        fx0 = self.c_kappa * kappa
        fy0 = -self.c_alpha_eff(fz) * alpha    # negative: positive α generates force toward right (-Y)
        # Friction-circle saturation
        f_max = max(0.0, mu * fz)
        mag = math.hypot(fx0, fy0)
        if mag > f_max and mag > 1e-9:
            scale = f_max / mag
            fx = fx0 * scale
            fy = fy0 * scale
        else:
            fx, fy = fx0, fy0
        # Self-aligning torque (linear with lateral force, opposes turning)
        mz = -fy * self.t_pneumatic
        return fx, fy, mz

    def peak_slips(self, fz: float, mu: float) -> tuple[float, float]:
        """Saturation slips: where the linear rise meets the friction circle.

        This model has no falling side — past saturation the force is flat, not
        decreasing. So "beyond peak" here means "on the plateau", which is
        still the information that matters: more slip buys nothing. The
        distinction between plateau and true drop-off only appears with the
        Pacejka model.
        """
        cap = max(float(mu) * float(fz), 0.0)
        alpha_peak = cap / max(self.c_alpha_eff(fz), 1e-6)
        kappa_peak = cap / max(self.c_kappa, 1e-6)
        return alpha_peak, kappa_peak


@dataclass
class PacejkaTireModel(TireModel):
    """Simplified Magic Formula (4-parameter pure slip + friction ellipse).

    Pure-slip curves:
        Fx0 =  D·sin(Cx·arctan(Bx·κ − Ex·(Bx·κ − arctan(Bx·κ))))
        Fy0 = −D·sin(Cy·arctan(By·α − Ey·(By·α − arctan(By·α))))
    with D = μ·Fz (so per-wheel μ from scene disturbances directly scales the
    peak, same contract as LinearTireModel) and B derived from the small-slip
    stiffness so that the origin slope matches the linear model exactly:
        Bx = c_kappa / (Cx·D),   By = c_alpha / (Cy·D)
    Combined slip uses friction-ellipse clipping — the smooth analogue of the
    linear model's friction circle.

    Self-aligning torque uses a slip-dependent pneumatic trail
        t(α) = t0·max(0, 1 − |α|/α_sl)
    which decays to zero at large slip (more realistic than constant trail).
    """

    c_alpha: float = 120_000.0     # small-slip cornering stiffness [N/rad] at Fz_nom
    c_kappa: float = 100_000.0     # small-slip longitudinal stiffness [N]
    t_pneumatic: float = 0.03      # pneumatic trail at zero slip [m]
    cx: float = 1.65               # longitudinal shape factor
    cy: float = 1.30               # lateral shape factor
    ex: float = -0.5               # longitudinal curvature factor
    ey: float = -1.0               # lateral curvature factor
    alpha_sl: float = 0.2          # trail decay slip angle [rad]
    fz_nom: float = 0.0            # nominal per-wheel load [N]; 0 = load sensitivity off
    load_exp: float = 0.0          # exponent p in c_α(Fz) = c_α0·(Fz/Fz_nom)^p

    def c_alpha_eff(self, fz: float) -> float:
        if self.fz_nom <= 0.0 or self.load_exp <= 0.0:
            return self.c_alpha
        ratio = max(fz / self.fz_nom, 1e-3)
        return self.c_alpha * ratio ** self.load_exp

    def forces(self, alpha: float, kappa: float, fz: float, mu: float) -> tuple[float, float, float]:
        fx0, fy0 = pacejka_combined_forces(
            alpha=alpha,
            kappa=kappa,
            fz=fz,
            mu=mu,
            c_alpha=self.c_alpha_eff(fz),
            c_kappa=self.c_kappa,
            cx=self.cx,
            cy=self.cy,
            ex=self.ex,
            ey=self.ey,
        )
        trail = self.t_pneumatic * max(0.0, 1.0 - abs(alpha) / self.alpha_sl)
        mz = -fy0 * trail
        return fx0, fy0, mz

    def peak_slips(self, fz: float, mu: float) -> tuple[float, float]:
        """True Magic-Formula peaks, past which the force genuinely falls.

        The peak sits at a fixed slip *argument* x* = B·slip set only by the
        shape factors, and B = stiffness/(C·D) with D = μ·Fz — so the peak slip
        itself scales linearly with μ·Fz:

            slip_peak = x* · C · μ·Fz / stiffness

        which is why this needs no per-step root finding.
        """
        d = max(float(mu) * float(fz), 1e-9)
        alpha_peak = self._x_peak_y * self.cy * d / max(self.c_alpha_eff(fz), 1e-6)
        kappa_peak = self._x_peak_x * self.cx * d / max(self.c_kappa, 1e-6)
        return alpha_peak, kappa_peak

    def __post_init__(self) -> None:
        # Shape-factor constants — solved once, reused every step.
        self._x_peak_x = _magic_formula_peak_x(self.cx, self.ex)
        self._x_peak_y = _magic_formula_peak_x(self.cy, self.ey)


def make_tire(params: object) -> TireModel:
    """Build the tire model selected by ``params.tire_model``.

    Stiffness / trail fields are shared between the two models so switching
    keeps small-slip behaviour (and the wheel-speed servo tuning) identical.
    ``tire_load_sensitivity_time_domain`` (C3-1) turns on the same
    c_α(Fz) = c_α0·(Fz/Fz_nom)^p law the load page uses; Fz_nom = m·g/4.
    """
    kw = dict(
        c_alpha=getattr(params, "tire_c_alpha", 120_000.0),
        c_kappa=getattr(params, "tire_c_kappa", 100_000.0),
        t_pneumatic=getattr(params, "tire_t_pneumatic", 0.03),
    )
    if getattr(params, "tire_load_sensitivity_time_domain", False):
        kw["fz_nom"] = float(getattr(params, "mass", 2900.0)) * 9.80665 / 4.0
        kw["load_exp"] = float(getattr(params, "tire_load_sensitivity_exp", 0.8))
    if getattr(params, "tire_model", "linear") == "pacejka":
        return PacejkaTireModel(
            **kw,
            cx=getattr(params, "tire_cx", 1.65),
            cy=getattr(params, "tire_cy", 1.30),
            ex=getattr(params, "tire_ex", -0.5),
            ey=getattr(params, "tire_ey", -1.0),
        )
    return LinearTireModel(**kw)
