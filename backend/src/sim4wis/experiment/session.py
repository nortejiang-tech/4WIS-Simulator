"""SimSession — headless, deterministic, faster-than-realtime execution.

The realtime `Simulator` and this class are two hosts of the same simulation
core: model + strategy + scene stepped at a fixed dt, with the shared
`update_derived_outputs` filling the rack-force chain after every step. The
differences are deliberate:

    * No wall-clock pacing — a run executes as fast as the CPU allows.
    * Driver input comes from the experiment's sim-time maneuver, not the WS.
    * Every push-interval sample is written into in-memory channel arrays that
      the caller persists as a run artifact.

Determinism: no clocks, no randomness — running the same Experiment twice
produces identical arrays (guarded by tests).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable

from sim4wis.core.step import advance_model
from sim4wis.controller.registry import make_strategy
from sim4wis.core.state import DriverInput, EnvironmentState, VehicleParams
from sim4wis.environment.disturbance import Scene
from sim4wis.experiment.schema import Experiment, PathSpec
from sim4wis.project.params_codec import params_from_dict
from sim4wis.vehicle.model_registry import make_vehicle_model
from sim4wis.vehicle.steering_link import STEERING_CHANNELS, idle_channels

WHEELS = ("fl", "fr", "rl", "rr")

# Strategies that honour `mode_params["steer_raw_rad"]` and can therefore run a
# `unit: front_deg` maneuver. Any other strategy silently ignores the key and
# would run the experiment with zero steering, so we refuse instead.
FRONT_DEG_STRATEGIES = frozenset({
    "ideal_ackermann", "rear_wheel_steer", "fault_reconfig",
})

# Channels recorded by every headless run (superset of the RT recorder's
# defaults — batch analysis wants slip + driver inputs too).
SCALAR_CHANNELS = (
    "pose_x", "pose_y", "pose_psi",
    "vx", "vy", "yaw_rate",
    "driver_steering", "driver_throttle",
    # Body specific force — the g-g trace for post-run analysis.
    "ax", "ay",
    # Front axle, when a steering plant is configured. NaN otherwise — see
    # steering_link.idle_channels for why not zero.
    *STEERING_CHANNELS,
)
_IDLE_STEERING = idle_channels()
WHEEL_CHANNELS = (
    "delta", "delta_cmd", "omega", "fz", "torque_steer",
    "rack_force", "motor_torque", "slip_alpha", "slip_kappa", "icr_dev",
    # Friction budget, so a run can be reviewed for where grip ran out and
    # whether the tyre was still on the rising side when it did.
    "grip_util", "grip_margin_lat", "grip_margin_long",
)


def resolve_vehicle_params(exp: Experiment) -> VehicleParams:
    """profile (optional) + overrides (optional) → VehicleParams."""
    base = VehicleParams()
    if exp.vehicle.profile:
        from sim4wis.project.vehicle_profiles import load_profile
        raw = load_profile(exp.vehicle.profile)
        base = params_from_dict(raw.get("vehicle"), base)
    if exp.vehicle.overrides:
        base = params_from_dict(exp.vehicle.overrides, base)
    return base


def build_plan(spec: PathSpec):
    """PathSpec → PathPlan (pure; never touches the process-wide active plan)."""
    from sim4wis.controller.path import plan_from_template, plan_from_waypoints
    if spec.template:
        return plan_from_template(spec.template, spec.params)
    if spec.waypoints:
        return plan_from_waypoints(spec.waypoints, closed=spec.closed)
    return None


@dataclass
class RunResult:
    t: list[float]
    channels: dict[str, list[float]]
    duration_s: float
    n_steps: int
    meta: dict[str, Any] = field(default_factory=dict)

    def channel_names(self) -> list[str]:
        return list(self.channels.keys())


class SimSession:
    """One headless execution context for one Experiment."""

    def __init__(self, exp: Experiment) -> None:
        self.exp = exp
        self.params = resolve_vehicle_params(exp)
        self.model = make_vehicle_model(self.params, exp.model_type)
        self.strategy = make_strategy(exp.strategy, self.params)
        self._front_deg_ok = exp.strategy in FRONT_DEG_STRATEGIES
        from sim4wis.project.schema import SceneSection
        scene = SceneSection(**exp.scene).to_scene() if exp.scene else Scene()
        self.env = EnvironmentState(scene=scene, mu=scene.base_mu)
        if exp.path is not None:
            plan = build_plan(exp.path)
            if plan is not None and hasattr(self.strategy, "plan_override"):
                self.strategy.plan_override = plan
        # Latched hold angles for stuck_hold faults (captured at activation).
        self._fault_hold: dict[int, float] = {}
        # Free-caster integrator states: key → [delta, delta_dot].
        self._free_state: dict[int, list[float]] = {}

    # ---- execution -----------------------------------------------------------

    def run(self, progress: Callable[[float], None] | None = None) -> RunResult:
        exp = self.exp
        dt = float(exp.dt)
        record_every = max(1, int(round(1.0 / (float(exp.record_hz) * dt))))

        t_out: list[float] = []
        chans: dict[str, list[float]] = {c: [] for c in SCALAR_CHANNELS}
        for base in WHEEL_CHANNELS:
            for w in WHEELS:
                chans[f"{base}_{w}"] = []

        driver = DriverInput(mode_params=dict(exp.mode_params))
        v_max = max(float(self.params.v_max), 0.1)
        total = max(exp.maneuver.total_duration, 1e-9)
        elapsed = 0.0
        n_steps = 0
        speed_target = 0.0  # current speed target [m/s]

        self.model.reset()

        for step_def in exp.maneuver.steps:
            if step_def.mode_params:
                driver.mode_params.update(step_def.mode_params)
            speed_prev = speed_target
            if step_def.speed_kmh is not None:
                speed_target = float(step_def.speed_kmh) / 3.6
            ramp = float(step_def.speed_ramp_s) if step_def.speed_kmh is not None else 0.0
            n = max(1, int(round(step_def.duration / dt)))
            for k in range(n):
                t_local = k * dt
                if ramp > 0.0 and t_local < ramp:
                    v_cmd = speed_prev + (speed_target - speed_prev) * (t_local / ramp)
                else:
                    v_cmd = speed_target
                # Command the speed DIRECTLY, not through the pedal.
                #
                # The old form (`throttle = v_cmd / v_max`) round-trips the
                # target through the driver-input model and only cancels while
                # that mapping stays exactly linear over exactly [0, v_max].
                # A pedal curve, a speed limiter or a powertrain envelope would
                # each silently move every golden baseline. This is the
                # longitudinal twin of the `steer_raw_rad` decoupling: a
                # validation run measures the vehicle, not the driver model.
                #
                # throttle/gear are still set so telemetry and any strategy that
                # reads them sees a coherent driver state.
                driver.mode_params["speed_target_ms"] = float(v_cmd)
                driver.gear = -1 if v_cmd < 0.0 else 1
                driver.throttle = min(1.0, abs(v_cmd) / v_max)
                steer_raw = step_def.steer.value(t_local, step_def.duration)
                if step_def.steer.unit == "front_deg" and not self._front_deg_ok:
                    raise ValueError(
                        f"strategy '{self.exp.strategy}' does not support "
                        f"steer unit 'front_deg': it ignores "
                        f"mode_params['steer_raw_rad']. The run would silently "
                        f"execute with zero steering and produce a straight-line "
                        f"'baseline'. Use unit 'normalized', or one of: "
                        f"{sorted(FRONT_DEG_STRATEGIES)}."
                    )
                if step_def.steer.unit == "front_deg":
                    # Bypass the driver feel layer: the amplitude IS the front-
                    # wheel angle in degrees. Pass it through mode_params as a
                    # raw radian value so the strategy skips its feel mapping
                    # and the validation measures the vehicle, not the driver.
                    #
                    # Mutate the live dict — do NOT rebuild it from
                    # `step_def.mode_params`. That drops the experiment-level
                    # mode_params merged in at construction, which is where
                    # fault_reconfig reads fault_wheel / fault_time /
                    # detect_delay / v_limit_kmh from. Losing them silently
                    # rearmed the mitigation controller with defaults (fault at
                    # t=0), which read as a huge cross-track error rather than
                    # as a configuration failure.
                    driver.mode_params["steer_raw_rad"] = math.radians(steer_raw)
                    driver.steering = 0.0
                else:
                    # Clear any raw angle a previous front_deg step left behind,
                    # or the strategy keeps bypassing the feel layer.
                    driver.mode_params.pop("steer_raw_rad", None)
                    driver.steering = max(-1.0, min(1.0, steer_raw))
                cmd = self.strategy.compute(driver, self.model.state, dt)
                self._apply_faults(cmd)
                # Fill the friction-brake actuator command (same helper the
                # live loop uses, so the two cannot drift apart).
                advance_model(self.model, self.params, driver, cmd, dt, self.env)
                if n_steps % record_every == 0:
                    self._sample(t_out, chans, cmd, driver)
                n_steps += 1
            elapsed += step_def.duration
            if progress is not None:
                progress(min(elapsed / total, 1.0))

        return RunResult(
            t=t_out,
            channels=chans,
            duration_s=exp.maneuver.total_duration,
            n_steps=n_steps,
            meta={
                "experiment": exp.model_dump(mode="json"),
                "record_hz": exp.record_hz,
                "n_samples": len(t_out),
            },
        )

    # ---- fault injection ---------------------------------------------------------

    def _apply_faults(self, cmd) -> None:
        """Overwrite steer commands for active faults (see schema.FaultSpec)."""
        t = float(self.model.state.t)
        dt = float(self.exp.dt)
        for i, f in enumerate(self.exp.faults):
            if t < f.t_start:
                continue
            w = int(f.wheel)
            key = i * 4 + w
            if f.fault_type == "stuck_zero":
                cmd.delta_cmd[w] = 0.0
            elif f.fault_type == "stuck_hold":
                if key not in self._fault_hold:
                    # Latch the *actual* wheel angle at activation — a jam
                    # freezes the mechanism where it physically is.
                    self._fault_hold[key] = float(self.model.state.delta[w])
                cmd.delta_cmd[w] = self._fault_hold[key]
            elif f.fault_type == "stuck_value":
                cmd.delta_cmd[w] = float(f.value)
            elif f.fault_type == "limited":
                lim = abs(float(f.value))
                cmd.delta_cmd[w] = max(-lim, min(lim, float(cmd.delta_cmd[w])))
            elif f.fault_type == "free_caster":
                # De-energised, non-self-locking mechanism: the wheel is a
                # castering DOF driven back by the tyre kingpin moment.
                # τ_steer > 0 is the holding torque the (absent) actuator
                # would have to supply, so the free wheel feels −τ_steer.
                if key not in self._free_state:
                    self._free_state[key] = [float(self.model.state.delta[w]), 0.0]
                delta, ddot = self._free_state[key]
                tau_kp = float(self.model.state.torque_steer[w])
                drive = -f.eta_rev * tau_kp
                fric = f.c_damp * ddot + f.tau_coulomb * math.tanh(ddot / 0.05)
                acc = (drive - fric) / f.j_steer
                ddot += acc * dt                      # semi-implicit Euler
                delta += ddot * dt
                lim = float(self.params.steer_limit)
                if delta > lim:
                    delta, ddot = lim, min(ddot, 0.0)
                elif delta < -lim:
                    delta, ddot = -lim, max(ddot, 0.0)
                self._free_state[key] = [delta, ddot]
                cmd.delta_cmd[w] = delta

    # ---- sampling --------------------------------------------------------------

    def _sample(self, t_out: list[float], chans: dict[str, list[float]], cmd, driver: DriverInput) -> None:
        s = self.model.state
        t_out.append(float(s.t))
        chans["pose_x"].append(float(s.x))
        chans["pose_y"].append(float(s.y))
        chans["pose_psi"].append(float(s.psi))
        chans["vx"].append(float(s.vx))
        chans["vy"].append(float(s.vy))
        chans["yaw_rate"].append(float(s.yaw_rate))
        chans["driver_steering"].append(float(driver.steering))
        chans["driver_throttle"].append(float(driver.throttle))
        chans["ax"].append(_finite(s.ax))
        chans["ay"].append(_finite(s.ay))
        steer = getattr(self.model, "steering_channels", None) or _IDLE_STEERING
        for name in STEERING_CHANNELS:
            chans[name].append(float(steer.get(name, math.nan)))
        slip_a = getattr(self.model, "slip_alpha", None)
        slip_k = getattr(self.model, "slip_kappa", None)
        for i, w in enumerate(WHEELS):
            chans[f"delta_{w}"].append(float(s.delta[i]))
            chans[f"delta_cmd_{w}"].append(float(cmd.delta_cmd[i]))
            chans[f"omega_{w}"].append(float(s.wheel_omega[i]))
            chans[f"fz_{w}"].append(float(s.fz[i]))
            chans[f"torque_steer_{w}"].append(float(s.torque_steer[i]))
            chans[f"rack_force_{w}"].append(float(s.rack_force[i]))
            chans[f"motor_torque_{w}"].append(float(s.motor_torque_demand[i]))
            chans[f"slip_alpha_{w}"].append(_finite(slip_a[i]) if slip_a is not None else 0.0)
            chans[f"slip_kappa_{w}"].append(_finite(slip_k[i]) if slip_k is not None else 0.0)
            chans[f"grip_util_{w}"].append(_finite(s.grip_util[i]))
            chans[f"grip_margin_lat_{w}"].append(_finite(s.grip_margin_lat[i]))
            chans[f"grip_margin_long_{w}"].append(_finite(s.grip_margin_long[i]))
            dev = float(s.wheel_icr_dev[i])
            chans[f"icr_dev_{w}"].append(dev if math.isfinite(dev) else math.nan)


def _finite(v: Any) -> float:
    f = float(v)
    return f if math.isfinite(f) else 0.0


def run_experiment(exp: Experiment, progress: Callable[[float], None] | None = None) -> RunResult:
    """Convenience: build a fresh session and execute it once."""
    return SimSession(exp).run(progress)


__all__ = [
    "SimSession", "RunResult", "run_experiment",
    "resolve_vehicle_params", "build_plan",
    "SCALAR_CHANNELS", "WHEEL_CHANNELS", "WHEELS",
]
