"""CornerTracker — one controller+plant+sensor per corner, and the step the
vehicle models call.

The strategy still commands ``delta_cmd``; when the tracking layer is
enabled, every corner the architecture gives an actuator to (SBW front,
RWS rear, all four on 4WIS) is advanced through its own tracker instead of
the legacy actuator model. Corners without an actuator keep the legacy path.

Target selection follows the architecture's capability: with
``per_wheel_steer`` (4WIS) each corner follows its own ``delta_cmd[i]``;
otherwise the front corners follow the averaged front command and the rear
corners the averaged rear command — the same quantity the legacy path used,
so the averaged case stays comparable to it.

Multi-rate: a controller that declares ``inner_rate_hz`` is stepped at that
rate with the target linearly interpolated across the outer frame (the D5
lesson — a held command under-resolves what the controller acts on). A
controller with ``inner_rate_hz = None`` (the open-loop reference) is
stepped once per outer frame with the final target, which is exactly how
the legacy update worked and why it stays bit-identical.

The load torque is the corner-referred rack force: ``rack_force[i] ×
pinion_radius``. That is the honest first-order equivalent — the corner
actuator is modelled in the wheel domain and the rack chain's gearing
lives in the pinion radius; a finer per-corner linkage model is the
actuator-spec work (M2), not the control layer's.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from sim4wis.core.state import N_WHEELS, VehicleParams
from sim4wis.steering import architecture as arch
from sim4wis.steering.tracking.controller import (
    AngleTrackingController,
    TrackingOutput,
    make_controller,
)
from sim4wis.steering.tracking.feedback import AngleSensor
from sim4wis.steering.tracking.plant import CornerActuatorPlant

#: Wheel labels in wheel-index order, for channel names.
WHEELS = ("fl", "fr", "rl", "rr")


def controlled_corners(architecture: arch.Architecture) -> tuple[int, ...]:
    """The wheel indices this architecture gives a corner actuator to."""
    corners: list[int] = []
    if architecture.front_path == "by_wire":
        corners += [0, 1]
    if architecture.rear_axle != "none":
        corners += [2, 3]
    return tuple(corners)


@dataclass
class CornerTracker:
    """The per-corner state bundle the vehicle model holds and advances."""

    corner: int
    controller: AngleTrackingController
    plant: CornerActuatorPlant
    sensor: AngleSensor
    #: Actuator truth — what the tyre actually sees [rad]. The controller's
    #: direct-angle branch or the plant's integration writes this.
    angle: float = 0.0
    #: Previous frame's target, for the inner-rate interpolation.
    prev_target: float = 0.0
    #: Post-step diagnostics (deviation, rate_limited, …) for recording.
    diagnostics: dict[str, float] = field(default_factory=dict)

    def reset(self, angle: float = 0.0) -> None:
        self.angle = float(angle)
        self.prev_target = float(angle)
        self.diagnostics = {}
        self.controller.reset()
        self.plant.reset(float(angle))

    def apply(self, out: TrackingOutput, *, dt: float, load_torque: float) -> None:
        """Apply one controller decision to the corner state."""
        if out.angle_set is not None:
            self.angle = float(out.angle_set)
            self.plant.reset(self.angle)  # keep the plant's view in sync
        elif out.torque_cmd is not None:
            self.plant.step(dt, out.torque_cmd, load_torque)
            self.angle = self.plant.angle
        else:
            raise ValueError("controller returned neither angle_set nor torque_cmd")
        self.diagnostics = dict(out.diagnostics)


def make_corner_trackers(
    params: VehicleParams,
) -> tuple[dict[int, CornerTracker], bool] | None:
    """Build the layer's trackers, or None when the layer is inactive.

    Returns (trackers by wheel index, per_wheel_targets). The layer is
    inactive when disabled in the params, when the architecture has no
    corner actuators, or when the architecture is unknown — in every such
    case the legacy paths run untouched (the disabled-layer contract).
    """
    sys_params = getattr(params, "steering_system", None)
    if sys_params is None or not sys_params.enabled:
        return None
    ac = sys_params.angle_control
    if not ac.enabled:
        return None
    try:
        architecture = arch.get(sys_params.architecture)
    except arch.ArchitectureError:
        return None
    corners = controlled_corners(architecture)
    if not corners:
        return None
    per_wheel = architecture.can(arch.CAP_PER_WHEEL)
    trackers: dict[int, CornerTracker] = {}
    for i in corners:
        # One controller instance per corner: their integrators and filters
        # are per-corner state, and a shared instance would silently couple
        # the corners through it.
        trackers[i] = CornerTracker(
            corner=i,
            controller=make_controller(ac.controller, **ac.controller_kwargs),
            plant=CornerActuatorPlant(
                inertia_kgm2=ac.plant_inertia_kgm2,
                damping_nms_per_rad=ac.plant_damping_nms_per_rad,
                coulomb_friction_nm=ac.plant_friction_nm,
                peak_torque_nm=ac.plant_peak_torque_nm,
                rate_limit_rad_s=ac.plant_rate_limit_rad_s,
                transmission_stiffness_nms_per_rad=(
                    ac.plant_transmission_stiffness_nms_per_rad),
                backlash_rad=ac.plant_backlash_rad,
                motor_inertia_fraction=ac.plant_motor_inertia_fraction,
            ),
            sensor=AngleSensor(
                quant_rad=ac.sensor_quant_rad,
                delay_steps=ac.sensor_delay_steps,
                noise_std_rad=ac.sensor_noise_std_rad,
                seed=ac.sensor_seed + i,
            ),
        )
    return trackers, per_wheel


def _one_step(tr: CornerTracker, load: float, speed_ms: float,
              h: float, t: float, t_rate: float) -> None:
    """One inner step: measure, decide, apply — for one tracker."""
    out = tr.controller.step(
        h,
        target_angle=t,
        target_rate=t_rate,
        feedback_angle=tr.sensor.measure(tr.angle),
        plant_angle=tr.angle,
        load_torque=load,
        speed_ms=speed_ms,
    )
    tr.apply(out, dt=h, load_torque=load)


def corner_tracking_step(
    trackers: dict[int, CornerTracker],
    per_wheel: bool,
    delta_cmd: np.ndarray,
    dt: float,
    *,
    rack_forces: np.ndarray,
    speed_ms: float,
    pinion_radius: float,
) -> tuple[np.ndarray, dict[str, float]]:
    """Advance every tracker one outer frame.

    Returns (delta_act for the tracked corners [rad], corner channels).
    """
    dt = max(float(dt), 1e-9)
    front_cmd = 0.5 * (float(delta_cmd[0]) + float(delta_cmd[1]))
    rear_cmd = 0.5 * (float(delta_cmd[2]) + float(delta_cmd[3]))
    delta_act = np.zeros(N_WHEELS)
    channels: dict[str, float] = {}

    for corner, tr in trackers.items():
        target = (float(delta_cmd[corner]) if per_wheel
                  else (front_cmd if corner < 2 else rear_cmd))
        load = float(rack_forces[corner]) * max(float(pinion_radius), 1e-9)
        rate = tr.controller.inner_rate_hz

        if rate is None:
            # Legacy semantics: one step per outer frame, command held at the
            # frame's final value. This is what keeps open_loop bit-exact.
            _one_step(tr, load, float(speed_ms), dt, target, 0.0)
        else:
            n = max(1, int(round(dt * float(rate))))
            h = dt / n
            t_rate = (target - tr.prev_target) / dt
            for k in range(n):
                frac = (k + 1) / n
                _one_step(tr, load, float(speed_ms), h,
                          tr.prev_target + (target - tr.prev_target) * frac,
                          t_rate)
        tr.prev_target = target
        delta_act[corner] = tr.angle
        channels[f"steer_corner_deviation_{WHEELS[corner]}"] = target - tr.angle
        for key, value in tr.diagnostics.items():
            channels[f"steer_corner_{key}_{WHEELS[corner]}"] = float(value)
    return delta_act, channels
