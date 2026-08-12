"""On-centre analysis — the weave test, reduced to numbers.

What "on-centre feel" is, and why it needs its own analysis
-----------------------------------------------------------
Almost all motorway driving happens within a few degrees of straight ahead, at
lateral accelerations below 0.2 g. That region is where a car feels precise or
vague, and it is invisible to every metric this repository had: a steady-state
yaw gain is measured well outside it, and a step response passes through it in
50 ms. What decides it is the **relationship between steering wheel torque and
angle near zero** — which is a hysteresis loop, not a curve, because friction
in the rack and the column makes the path out differ from the path back.

So the test is a slow sinusoidal weave (0.2 Hz is the usual choice) held at
constant speed, and the analysis is done on cross-plots rather than on time
histories:

    torque ── vs ── steering wheel angle     the loop; friction and precision
    torque ── vs ── lateral acceleration     what effort buys in cornering
    angle  ── vs ── lateral acceleration     steering sensitivity
    yaw rate  vs   steering wheel angle      how late the car answers

Each cross-plot gives its numbers at or near **zero**, which is the whole point:
these are on-centre metrics, and a gradient measured at 0.5 g describes a
different part of the car.

Relationship to ISO 13674-1
---------------------------
This follows the standard's *measurement convention* — the weave excitation,
the cross-plots, and the quantities read off them. It is not a conformance
implementation, and two things are worth naming rather than leaving implied:

* The lateral-acceleration quantities regress over ±0.1 g, which is the
  standard's own neighbourhood. The **torque gradient at zero angle** needs an
  angle neighbourhood, and that is the one free parameter here (`ANGLE_WINDOW`,
  a fraction of the achieved amplitude); it is stated so a reader can disagree
  with it.
* Hysteresis and deadband use **no window at all**. They are read off the
  actual zero crossings of each branch, interpolated between samples and
  averaged over cycles. The first version of this module fitted them from a
  windowed regression like the gradient, and the result moved by 5x with the
  weave amplitude — a fraction-of-amplitude window is not comparable across
  amplitudes for a quantity that is absolute. Measuring the crossing directly
  removes the parameter rather than documenting it.
* Nothing here is correlated against a real vehicle. These numbers are as good
  as the plant that produced them, and that plant's parameters are engineering
  estimates.

What the loop width is, and is not
----------------------------------
`torque_hysteresis_nm` and `angle_deadband_rad` are the width of the loop at
zero angle and at zero torque. It is tempting to read them as "friction feel",
and on this model that reading does not hold: **they are not monotone in rack
Coulomb friction.** Measured on the default vehicle at 100 km/h, 0.2 Hz, 0.19 g:

    rack Coulomb [N]      0     80    160    260    500    900
    loop width [N·m]  0.987  0.849  0.711  0.534  0.260  1.656

It is already 0.99 N·m with the friction term set to **zero**, so most of the
loop at this frequency is not friction at all. Two quadrature contributions are
present and they have opposite sign: Coulomb friction acts with the direction
of motion, while the tyre load lags the steer angle by the vehicle's own
lateral-dynamics time constant (~0.18 s here, ≈13° of phase at 0.2 Hz). Over
part of the range they cancel.

Both effects are real, and a real weave on a real car contains both — the
standard's own quantity is the raw loop width. What does not follow is using it
as a *requirement* on steering friction, which is why the shipped
`steering_feel` target set records these two as `should` rather than judging on
them, and why `test_the_loop_width_is_not_monotone_in_rack_friction` pins the
behaviour rather than hiding it. Separating the two contributions needs either
a frequency low enough that the vehicle lag vanishes, or a rig test with the
vehicle held still; both are V3 work that has not been done.

Refusals
--------
Every function here raises rather than returning a plausible number when the
run cannot support the measurement: no steering plant, too few cycles, an
amplitude too small to fit through, a torque signal that never changes sign.
A weave metric computed from a straight-line run would come back as a very
good on-centre result.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

#: Fraction of the achieved steering amplitude used as the regression
#: neighbourhood for the torque gradient. Wide enough to fit through sampling
#: noise, narrow enough to still be "on centre".
ANGLE_WINDOW = 0.25

#: Above this lateral-acceleration amplitude the run is not an on-centre test
#: and the analysis refuses it [m/s²]. 0.35 g, against a target of 0.1-0.2 g.
#:
#: Not a nicety. Swept over amplitude on the default vehicle, everything is
#: orderly up to ~0.25 g and then the tyres saturate: at 0.84 g the fitted
#: torque gradient came out **negative** — a car that gets easier to turn the
#: harder you corner. The analysis was being run on a vehicle that was sliding,
#: and it answered anyway.
AY_CEILING = 3.43

#: Lateral-acceleration neighbourhood for the ay-domain quantities [m/s²].
#: 0.1 g, which is the standard's own.
AY_WINDOW = 0.981

#: Schmitt-trigger threshold for crossing detection, as a fraction of the
#: signal amplitude. Guards against counting the settling lead-in as cycles.
_CROSSING_HYSTERESIS = 0.25

#: Cycles that must be present before anything is measured. The first is
#: discarded as transient, so two is the floor and three is the useful minimum.
MIN_CYCLES = 2.0


class ProcedureError(ValueError):
    """This run cannot support this measurement, and why."""


@dataclass(frozen=True)
class Branch:
    """One side of the hysteresis loop: slope, intercept, and the zero crossing."""

    slope: float
    intercept: float

    @property
    def zero_crossing(self) -> float:
        """Where the fitted line crosses y = 0."""
        if abs(self.slope) < 1e-12:
            return math.nan
        return -self.intercept / self.slope


@dataclass(frozen=True)
class WeaveResult:
    """Everything one weave run yields."""

    cycles: float
    frequency_hz: float
    angle_amplitude_rad: float
    ay_amplitude: float

    torque_gradient_nm_per_rad: float
    torque_hysteresis_nm: float
    angle_deadband_rad: float

    torque_gradient_nm_per_g: float | None
    torque_at_0_1g_nm: float | None
    angle_gradient_rad_per_g: float | None
    yaw_phase_lag_deg: float | None


def _require_plant(active: np.ndarray | None) -> None:
    if active is None:
        raise ProcedureError(
            "本次运行没有转向系统通道 —— 需要 vehicle.overrides.steering_system.enabled = true"
        )
    if not np.all(np.isfinite(active)) or not np.all(active > 0.5):
        raise ProcedureError(
            "转向系统被控对象未在本次运行中启用，手力矩没有被仿真 —— "
            "中心区指标无从谈起（置 vehicle.overrides.steering_system.enabled = true）"
        )


def _cycles_and_frequency(t: np.ndarray, x: np.ndarray) -> tuple[float, float]:
    """Count complete cycles and estimate the excitation frequency.

    From zero crossings rather than from the spec, so the analysis measures
    what the run actually did instead of what it was asked to do. That is not
    pedantry: a misspelled `freq_hz` used to run at the default and the whole
    analysis would have been done at the wrong frequency.

    The frequency is taken **between the first and last crossing**, not over
    the whole record. A weave spec normally opens with a few seconds of
    straight running to let the transient settle, and dividing by the total
    span counts that dead time as part of the period — 0.197 Hz for a 0.200 Hz
    weave with a 3 s lead-in, which then leaks into the phase estimate.

    Crossings are detected with a **Schmitt trigger** at `_CROSSING_HYSTERESIS`
    of the amplitude, not on the raw sign. The lead-in is not perfectly zero —
    static toe and a settling transient leave it jittering about the mean — so
    a bare sign test counted spurious crossings there, stretched the span back
    into the dead time, and read 6.5 cycles at 0.191 Hz for a clean 6-cycle
    0.200 Hz weave.
    """
    centred = np.asarray(x, dtype=float) - float(np.mean(x))
    amp = float(np.max(np.abs(centred))) if centred.size else 0.0
    if amp <= 0.0:
        return 0.0, math.nan
    h = _CROSSING_HYSTERESIS * amp

    times: list[float] = []
    state = 0                                  # -1 below -h, +1 above +h, 0 unarmed
    for i in range(centred.size):
        v = float(centred[i])
        if state <= 0 and v > h:
            if state == -1:
                times.append(_interp_zero(t, centred, i))
            state = 1
        elif state >= 0 and v < -h:
            if state == 1:
                times.append(_interp_zero(t, centred, i))
            state = -1

    crossings = len(times)
    cycles = crossings / 2.0
    if crossings < 2:
        return cycles, math.nan
    span = times[-1] - times[0]
    freq = (crossings - 1) / (2.0 * span) if span > 0 else math.nan
    return cycles, freq


def _interp_zero(t: np.ndarray, x: np.ndarray, i: int) -> float:
    """Time at which `x` last passed through zero, at or before index `i`."""
    j = i
    while j > 0 and (float(x[j]) < 0.0) == (float(x[j - 1]) < 0.0):
        j -= 1
    if j == 0:
        return float(t[0])
    a, b = float(x[j - 1]), float(x[j])
    if a == b:
        return float(t[j])
    f = -a / (b - a)
    return float(t[j - 1]) + f * (float(t[j]) - float(t[j - 1]))


def _branch_fit(x: np.ndarray, y: np.ndarray, mask: np.ndarray, window: float,
                what: str) -> Branch:
    """Least-squares line through the samples of one branch near x = 0."""
    near = mask & (np.abs(x) <= window)
    if int(np.count_nonzero(near)) < 4:
        raise ProcedureError(
            f"{what}：零点附近可用样本不足（{int(np.count_nonzero(near))} 个）—— "
            "提高记录频率、加长时长，或放大扫掠幅值"
        )
    slope, intercept = np.polyfit(x[near], y[near], 1)
    return Branch(float(slope), float(intercept))


def _branch_crossings(x: np.ndarray, y: np.ndarray, rate: np.ndarray,
                      rising: bool) -> list[float]:
    """Values of `y` where `x` crosses zero, on one branch of the loop.

    Linear interpolation between the bracketing samples: at 0.2 Hz and 50 Hz
    recording, snapping to the nearer sample would quantise the crossing by a
    whole sample of steering angle, which is the size of the quantity being
    measured.
    """
    out: list[float] = []
    for i in range(len(x) - 1):
        a, b = float(x[i]), float(x[i + 1])
        if a == b or (a < 0.0) == (b < 0.0):
            continue
        r = 0.5 * (float(rate[i]) + float(rate[i + 1]))
        if (r > 0.0) != rising:
            continue
        f = -a / (b - a)
        out.append(float(y[i]) + f * (float(y[i + 1]) - float(y[i])))
    return out


def _loop_width(x: np.ndarray, y: np.ndarray, rate: np.ndarray, what: str) -> float:
    """How far apart the two branches are where `x` is zero.

    This is the hysteresis, measured rather than fitted. Averaged over every
    crossing in the record so one noisy cycle cannot set it.
    """
    up = _branch_crossings(x, y, rate, rising=True)
    down = _branch_crossings(x, y, rate, rising=False)
    if not up or not down:
        raise ProcedureError(
            f"{what}：迟滞环没有形成完整的上下行穿越（上行 {len(up)} 次、"
            f"下行 {len(down)} 次）—— 扫掠可能没有过零，或周期数不足"
        )
    return abs(float(np.mean(up)) - float(np.mean(down)))


def _split_branches(rate: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Rising and falling halves of the loop.

    The two are not the same line, and the gap between them *is* the friction —
    averaging over both without splitting them measures nothing and reports it
    confidently.
    """
    return rate > 0, rate < 0


def _loop(x: np.ndarray, y: np.ndarray, rate: np.ndarray, window: float,
          what: str) -> tuple[Branch, Branch]:
    up, down = _split_branches(rate)
    return (_branch_fit(x, y, up, window, f"{what}（上行）"),
            _branch_fit(x, y, down, window, f"{what}（下行）"))


def _phase_lag_deg(t: np.ndarray, drive: np.ndarray, response: np.ndarray,
                   freq: float) -> float | None:
    """How far the response trails the input, in degrees at the weave frequency.

    A single-bin DFT rather than a peak-to-peak time difference: at 0.2 Hz with
    a 20 Hz record the peak lands within one sample of several neighbours, and
    the answer would quantise to whole samples — 18° of phase each.
    """
    if not (freq and math.isfinite(freq)) or len(t) < 8:
        return None
    w = 2.0 * math.pi * freq
    basis = np.exp(-1j * w * t)
    a = complex(np.sum((drive - drive.mean()) * basis))
    b = complex(np.sum((response - response.mean()) * basis))
    if abs(a) < 1e-12 or abs(b) < 1e-12:
        return None
    lag = math.degrees(math.atan2((b / a).imag, (b / a).real))
    # A response cannot lead its input by more than a quarter cycle here; wrap
    # into (-180, 180] and report the trailing sense as positive.
    return -lag


def analyse_weave(
    t: np.ndarray,
    *,
    hand_angle: np.ndarray,
    hand_torque: np.ndarray,
    plant_active: np.ndarray | None = None,
    ay: np.ndarray | None = None,
    yaw_rate: np.ndarray | None = None,
    drop_first_cycle: bool = True,
) -> WeaveResult:
    """Reduce one weave run to its on-centre numbers."""
    _require_plant(plant_active)

    t = np.asarray(t, dtype=float)
    angle = np.asarray(hand_angle, dtype=float)
    torque = np.asarray(hand_torque, dtype=float)
    if t.size < 16 or angle.size != t.size or torque.size != t.size:
        raise ProcedureError("样本太少或通道长度不一致，无法做中心区分析")
    if not np.all(np.isfinite(angle)) or not np.all(np.isfinite(torque)):
        raise ProcedureError("手力矩或方向盘转角通道含 NaN —— 该架构可能没有这个信号")

    cycles, freq = _cycles_and_frequency(t, angle)
    if cycles < MIN_CYCLES:
        raise ProcedureError(
            f"只有 {cycles:.1f} 个周期，至少需要 {MIN_CYCLES:.0f} 个"
            "（第一个周期作为瞬态丢弃）—— 这是一次 weave 试验吗？"
        )

    if drop_first_cycle and freq > 0:
        keep = t >= (t[0] + 1.0 / freq)
        if int(np.count_nonzero(keep)) > 16:
            t, angle, torque = t[keep], angle[keep], torque[keep]
            if ay is not None:
                ay = np.asarray(ay, dtype=float)[keep]
            if yaw_rate is not None:
                yaw_rate = np.asarray(yaw_rate, dtype=float)[keep]

    amp = float(np.max(np.abs(angle - np.mean(angle))))
    if amp < math.radians(1.0):
        raise ProcedureError(
            f"方向盘扫掠幅值只有 {math.degrees(amp):.2f}°，太小，"
            "拟合出的梯度由数值噪声主导"
        )

    rate = np.gradient(angle, t)
    win = ANGLE_WINDOW * amp
    up, down = _loop(angle, torque, rate, win, "力矩-转角环")

    # Gradient is fitted — it is a slope, and a slope needs a neighbourhood.
    # Hysteresis and deadband are *measured*: how far apart the two branches
    # are where the angle is zero, and where the torque is zero. The second is
    # the angle you can move through before the car answers.
    gradient = 0.5 * (up.slope + down.slope)
    hysteresis = _loop_width(angle, torque, rate, "力矩-转角环（零转角处）")
    deadband = _loop_width(torque, angle, rate, "力矩-转角环（零力矩处）")

    t_grad_g = t_at_01g = a_grad_g = None
    ay_amp = 0.0
    if ay is not None and np.all(np.isfinite(ay)):
        ay_amp = float(np.max(np.abs(ay - np.mean(ay))))
        if ay_amp > AY_CEILING:
            raise ProcedureError(
                f"侧向加速度幅值 {ay_amp / 9.81:.2f} g 已超出中心区范围"
                f"（上限 {AY_CEILING / 9.81:.2f} g）—— 这是一次操纵试验，"
                "不是中心区试验；请减小扫掠幅值"
            )
        if ay_amp > 0.2:
            ay_rate = np.gradient(np.asarray(ay, dtype=float), t)
            win_ay = min(AY_WINDOW, ANGLE_WINDOW * ay_amp * 4.0)
            try:
                u2, d2 = _loop(np.asarray(ay), torque, ay_rate, win_ay, "力矩-侧向加速度")
                t_grad_g = 0.5 * (u2.slope + d2.slope) * 9.81
                mid_slope = 0.5 * (u2.slope + d2.slope)
                mid_icept = 0.5 * (u2.intercept + d2.intercept)
                t_at_01g = mid_slope * 0.981 + mid_icept
                u3, d3 = _loop(np.asarray(ay), angle, ay_rate, win_ay, "转角-侧向加速度")
                a_grad_g = 0.5 * (u3.slope + d3.slope) * 9.81
            except ProcedureError:
                # The lateral channels are a bonus; losing them must not cost
                # the torque-angle loop, which is the core of the test.
                pass

    lag = None
    if yaw_rate is not None and np.all(np.isfinite(yaw_rate)):
        lag = _phase_lag_deg(t, angle, np.asarray(yaw_rate, dtype=float), freq)

    return WeaveResult(
        cycles=cycles, frequency_hz=freq, angle_amplitude_rad=amp, ay_amplitude=ay_amp,
        torque_gradient_nm_per_rad=gradient,
        torque_hysteresis_nm=hysteresis,
        angle_deadband_rad=deadband,
        torque_gradient_nm_per_g=t_grad_g,
        torque_at_0_1g_nm=t_at_01g,
        angle_gradient_rad_per_g=a_grad_g,
        yaw_phase_lag_deg=lag,
    )
