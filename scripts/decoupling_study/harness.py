"""Manoeuvre harness for the steering-decoupling study.

Everything runs on the multibody model in torque mode, so the longitudinal
channel is a real powertrain rather than a speed servo that would quietly hold
the car at its target through a manoeuvre it should have been slowed by.

The architecture's rear-angle authority is imposed by clipping the rear wheel
commands after the control law has run. At the authorities involved (0, 3, 5
deg) the Ackermann spread across a rear pair is under 0.1 deg, so clipping per
wheel and clipping the axle are the same thing to well inside the resolution of
anything reported here; at L3's 35 deg the clip never binds.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np

from sim4wis.controller.longitudinal import apply_brake_command, apply_drive_command
from sim4wis.controller.registry import make_strategy
from sim4wis.core.derived import update_derived_outputs
from sim4wis.core.state import DriverInput, EnvironmentState, VehicleParams
from sim4wis.vehicle.multibody import MultiBodyModel

DT = 0.002
G = 9.80665
SETTLE_S = 6.0          # straight-line settling before any steer input


class Run:
    """One manoeuvre. Owns the model, applies the architecture's rear clip."""

    def __init__(self, arch, algo_mode_params: dict, v_kmh: float,
                 mu: float = 0.90, base: VehicleParams | None = None):
        self.arch = arch
        self.p = arch.params(base)
        self.p = replace(self.p, longitudinal_mode="torque")
        self.m = MultiBodyModel(self.p)
        self.m.reset()
        self.strat = make_strategy("rear_wheel_steer", self.p)
        self.d = DriverInput(gear=1, mode_params={
            **algo_mode_params, "speed_target_ms": v_kmh / 3.6})
        self.env = EnvironmentState(mu=mu)
        self._rear_lim = arch.rear_limit_rad
        self.t = 0.0

    def steer(self, rad: float) -> None:
        self.d.mode_params["steer_raw_rad"] = float(rad)

    def step(self) -> None:
        c = self.strat.compute(self.d, self.m.state, DT)
        # Architecture rear-angle authority.
        np.clip(c.delta_cmd[2:], -self._rear_lim, self._rear_lim, out=c.delta_cmd[2:])
        apply_brake_command(c, self.d, self.p)
        apply_drive_command(c, self.d, self.p, self.m.state)
        self.m.step(DT, c, self.env)
        update_derived_outputs(self.m.state, self.p)
        self.t += DT

    def run_for(self, secs: float) -> None:
        for _ in range(int(round(secs / DT))):
            self.step()

    def settle(self, tol: float = 0.01, max_s: float = 40.0) -> None:
        """Run straight until the car is actually AT the target speed.

        A fixed settle time is not safe: on mu = 0.4 the launch is traction
        limited, so six seconds leaves the car still accelerating and the
        "step" gets applied mid-launch. The first version of this study did
        exactly that and reported a 2408 ms yaw response time for L0 which was
        really the time left to finish accelerating. Converge on speed instead,
        and require it to hold — a car still gaining 0.5 km/h a second is not
        a valid initial condition for a transient measurement.
        """
        self.steer(0.0)
        target = float(self.d.mode_params.get("speed_target_ms", 0.0))
        self.run_for(2.0)
        held = 0.0
        while self.t < max_s:
            self.run_for(0.25)
            if target <= 0 or abs(self.s.vx - target) / max(target, 1e-6) < tol:
                held += 0.25
                if held >= 1.0:
                    return
            else:
                held = 0.0
        raise RuntimeError(
            f"speed did not settle: {self.s.vx:.2f} vs target {target:.2f} m/s "
            f"after {max_s:.0f} s (mu = {self.env.mu})")

    @property
    def s(self):
        return self.m.state

    def sample(self) -> dict:
        s = self.s
        beta = math.atan2(s.vy, max(abs(s.vx), 0.1))
        util = np.asarray(s.grip_util, dtype=float)
        return dict(t=self.t, vx=s.vx, vy=s.vy, yaw=s.yaw_rate, ay=s.ay, ax=s.ax,
                    beta=beta, roll=s.roll,
                    delta_f=float(np.mean(s.delta[:2])),
                    delta_r=float(np.mean(s.delta[2:])),
                    util_f=float(np.mean(util[:2])), util_r=float(np.mean(util[2:])),
                    x=s.x, y=s.y, psi=s.psi)


# ── steady-state cornering ─────────────────────────────────────────────────

def steady_state(arch, mode_params, v_kmh, delta_deg, mu=0.90, hold=10.0) -> dict:
    r = Run(arch, mode_params, v_kmh, mu)
    r.settle()
    r.steer(math.radians(delta_deg))
    r.run_for(hold)
    # Average the last half second so a residual limit-cycle does not read as
    # the steady value.
    acc = []
    for _ in range(int(0.5 / DT)):
        r.step()
        acc.append(r.sample())
    keys = ("vx", "yaw", "ay", "beta", "delta_f", "delta_r", "util_f", "util_r")
    out = {k: float(np.mean([a[k] for a in acc])) for k in keys}
    out["delta_cmd_deg"] = delta_deg
    out["R"] = out["vx"] / out["yaw"] if abs(out["yaw"]) > 1e-6 else float("inf")
    return out


def calibrate_steer_for_ay(arch, mode_params, v_kmh, ay_target, mu=0.90,
                           lo=0.1, hi=20.0, tol=0.02, iters=12):
    """Steer angle giving a target steady lateral acceleration.

    ISO 7401 specifies the step input by the steady-state lateral acceleration
    it produces, not by the angle. That matters here more than usual: the whole
    point of the ladder is that the rungs have different steering gains, so
    comparing them at equal ANGLE would confound "responds faster" with "turns
    harder". Every transient metric in this study is taken at equal a_y.
    """
    best = None
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        ay = abs(steady_state(arch, mode_params, v_kmh, mid, mu, hold=7.0)["ay"])
        best = (mid, ay)
        if abs(ay - ay_target) < tol:
            break
        if ay < ay_target:
            lo = mid
        else:
            hi = mid
    return best  # (delta_deg, ay_achieved)


# ── ISO 7401 step steer ────────────────────────────────────────────────────

def step_steer(arch, mode_params, v_kmh, delta_deg, mu=0.90, secs=4.0,
               trace=False) -> dict:
    r = Run(arch, mode_params, v_kmh, mu)
    r.settle()
    r.steer(math.radians(delta_deg))
    rows = []
    for _ in range(int(secs / DT)):
        r.step()
        rows.append(r.sample())

    t = np.array([q["t"] for q in rows]) - rows[0]["t"]
    yaw = np.abs([q["yaw"] for q in rows])
    ay = np.abs([q["ay"] for q in rows])
    beta = np.array([q["beta"] for q in rows])

    tail = int(0.5 / DT)
    yaw_ss = float(np.mean(yaw[-tail:]))
    ay_ss = float(np.mean(ay[-tail:]))
    beta_ss = float(np.mean(beta[-tail:]))

    def response_time(sig, ss):
        if ss <= 1e-6:
            return float("nan")
        idx = np.argmax(sig >= 0.9 * ss)
        return float(t[idx]) if sig[idx] >= 0.9 * ss else float("nan")

    peak_i = int(np.argmax(yaw))
    out = dict(
        yaw_ss=yaw_ss, ay_ss=ay_ss, beta_ss=beta_ss,
        yaw_t90=response_time(yaw, yaw_ss),
        ay_t90=response_time(ay, ay_ss),
        yaw_overshoot=(float(np.max(yaw)) / max(yaw_ss, 1e-9) - 1.0) * 100.0,
        yaw_peak_time=float(t[peak_i]),
        beta_peak=float(np.max(np.abs(beta))),
        # TB — the lag between lateral acceleration and yaw rate reaching 90%,
        # ISO 7401's "response time gain". A car that builds yaw without
        # building the sideslip that normally precedes the lateral force feels
        # sharper to the driver at the same steady-state gain.
        tb=response_time(ay, ay_ss) - response_time(yaw, yaw_ss),
        delta_cmd_deg=delta_deg,
        yaw_gain=yaw_ss / max(math.radians(delta_deg), 1e-9),
        beta_per_g=math.degrees(beta_ss) / max(ay_ss / G, 1e-9),
    )
    if trace:
        out["trace"] = dict(
            t=t.tolist(),
            yaw=[math.degrees(q["yaw"]) for q in rows],
            ay=[q["ay"] for q in rows],
            beta=[math.degrees(q["beta"]) for q in rows],
            delta_f=[math.degrees(q["delta_f"]) for q in rows],
            delta_r=[math.degrees(q["delta_r"]) for q in rows],
            x=[q["x"] for q in rows], y=[q["y"] for q in rows],
            psi=[q["psi"] for q in rows],
        )
    return out


# ── frequency response ─────────────────────────────────────────────────────

def frequency_point(arch, mode_params, v_kmh, freq_hz, delta_deg, mu=0.90,
                    cycles=6) -> dict:
    """Single-frequency sine steer; gain and phase by one-cycle correlation.

    The first two cycles are discarded as transient and the fit is taken over
    a whole number of the remaining cycles, so the estimate is not biased by a
    partial period.
    """
    r = Run(arch, mode_params, v_kmh, mu)
    r.settle()
    amp = math.radians(delta_deg)
    w = 2.0 * math.pi * freq_hz
    period = 1.0 / freq_hz
    total = cycles * period
    warm = 2 * period

    ts, yaws, betas, deltas = [], [], [], []
    n = int(total / DT)
    for k in range(n):
        tt = k * DT
        r.steer(amp * math.sin(w * tt))
        r.step()
        if tt >= warm:
            q = r.sample()
            ts.append(tt); yaws.append(q["yaw"])
            betas.append(q["beta"]); deltas.append(amp * math.sin(w * tt))

    ts = np.array(ts); yaws = np.array(yaws); betas = np.array(betas)
    c, s = np.cos(w * ts), np.sin(w * ts)
    norm = 2.0 / len(ts)

    def phasor(sig):
        return complex(float(np.dot(sig, s)) * norm, float(np.dot(sig, c)) * norm)

    inp = phasor(np.array(deltas))
    ph_yaw = phasor(yaws) / inp if abs(inp) > 1e-12 else complex(0, 0)
    ph_beta = phasor(betas) / inp if abs(inp) > 1e-12 else complex(0, 0)
    return dict(freq=freq_hz,
                yaw_gain=abs(ph_yaw), yaw_phase=math.degrees(np.angle(ph_yaw)),
                beta_gain=abs(ph_beta), beta_phase=math.degrees(np.angle(ph_beta)))


# ── low-speed manoeuvrability ──────────────────────────────────────────────

def turning_circle(arch, mode_params, v_kmh=10.0, mu=0.90) -> dict:
    """Radius at full lock. The rear-steer benefit here is purely kinematic —
    counter-phase rear steer moves the ICR forward and inboard — so it shows up
    even at a walking pace where no tyre is working hard."""
    p = VehicleParams()
    full = math.degrees(p.steer_limit) if arch.per_wheel else 35.0
    # A non-per-wheel architecture still commands its front axle to the same
    # mechanical limit; what differs is whether the rear can follow.
    ss = steady_state(arch, mode_params, v_kmh, min(full, 35.0), mu, hold=12.0)
    R = abs(ss["R"])
    # Wall-to-wall: outer front corner sweeps a larger circle than the CG.
    half_track = p.track_front / 2.0
    overhang = 0.9
    R_wall = math.hypot(R + half_track, p.wheelbase / 2.0 + overhang)
    out = dict(ss)
    out.update(R_cg=R, R_wall=R_wall, turning_circle=2.0 * R_wall,
               delta_f=math.degrees(ss["delta_f"]),
               delta_r=math.degrees(ss["delta_r"]))
    return out
