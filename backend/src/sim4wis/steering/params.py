"""Steering-system parameters — the hardware the assist control acts on.

What this adds, and what it deliberately does not
-------------------------------------------------
The repository already carries the *static* force path: tyre forces → kingpin
moment (`vehicle/kingpin.py`, Reimpell) → rack force through the hardpoint
linkage → motor torque demand (`wheel_rack_force_from_linkage`), with
`steering_arm_length`, `pinion_radius`, `rack_mech_efficiency` and
`motor_gear_ratio` already on `SteeringGeometryParams`. That path is good and
this module does not duplicate any of it.

What was missing is everything *between the driver's hands and the pinion*, and
everything about how the motor actually responds:

    hand wheel ──column J,c,friction── torsion bar K ──┬── pinion ── rack ── (existing path)
                                                       │
                                    torque sensor  τ_tb ┘  → assist map → motor

Without a torsion bar there is no torque-sensor signal, and without that there
is no input for any EPS control law and no hand-wheel torque to report. That is
the gap this package closes; see docs/v2_steering_platform_plan.md §2.

Defaults describe the LS9-class vehicle the rest of the repository is
calibrated for: 2.9 t SUV, 3.16 m wheelbase, 540° lock-to-lock, 20 mm pinion.
They are *plausible engineering estimates, not measured hardware* — the same
status the suspension geometry carries, and stated here for the same reason.

Units are SI throughout: N·m, rad, kg·m², N·m·s/rad. Anything in degrees or
km/h carries it in the field name.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class ColumnParams:
    """Hand wheel, column shaft and the torsion bar that senses driver effort.

    `torsion_stiffness` is the single most consequential number here: it sets
    the torque-sensor gain, and with the pinion-side inertia it sets the
    frequency of the column mode. Production torsion bars run 1.5–2.5 N·m/deg;
    softer gives a finer torque signal and a vaguer feel, stiffer the reverse.
    """

    inertia: float = 0.05                   # J_c 方向盘+柱转动惯量 [kg·m²]
    damping: float = 0.30                   # c_c 柱粘性阻尼 [N·m·s/rad]
    coulomb_friction: float = 0.15          # 柱库仑摩擦 [N·m]
    torsion_stiffness_nm_per_deg: float = 2.0   # 扭杆刚度 [N·m/deg]
    torsion_damping: float = 0.05           # 扭杆内阻 [N·m·s/rad]
    #: Sensor saturation — a real torque sensor clips, and a control law that
    #: assumes it does not will behave differently at the stops.
    sensor_range_nm: float = 10.0
    #: What the thing holding the wheel can actually apply [N·m].
    #:
    #: 25 N.m is above any sustained human effort (a strong driver two-handed
    #: is ~15) and below a test robot's capability, so it bounds the model
    #: without truncating anything a real test would measure.
    hand_torque_limit_nm: float = 25.0

    #: Grip impedance — the driver (or robot) as a spring-damper onto the
    #: commanded angle, rather than as an infinitely stiff constraint.
    #:
    #: This is what stops the marginal band from ringing, and it is physics
    #: rather than a numerical patch. With assist saturated the motor clips
    #: *everything* it was asked for, damping included, and the pinion is left
    #: as a mass on the torsion bar at zeta ~ 0.03. A real car does not ring
    #: there because a real pair of arms is compliant and damped and absorbs
    #: it. An imposed angle is infinitely stiff and absorbs nothing, so the
    #: model rang exactly where the hardware does not.
    #:
    #: The value is the *effective* stiffness of a driver actively holding a
    #: target angle, not passive arm impedance. Passive arms are 10-30
    #: N.m/rad, which leaves 22% of steady-state droop — the wheel sitting well
    #: short of where it was asked for — because a passive arm never corrects.
    #: A driver watching the road does. Measured against both requirements:
    #:
    #:     stiffness   droop   2 N.m motor, full-lock parking
    #:            20   22.1%   1429 rpm   (stable)
    #:            60    7.1%   1693 rpm   (stable)
    #:           200    2.1%   1725 rpm   (stable)
    #:           400    1.0%   1871 rpm   (stable)
    #:
    #: Stability holds throughout, so the choice is free on that axis and is
    #: made on droop. Damping is set for zeta ~= 0.7 against the column inertia.
    grip_stiffness: float = 200.0           # [N·m/rad]
    grip_damping: float = 4.4               # [N·m·s/rad]

    @property
    def torsion_stiffness(self) -> float:
        """K_tb [N·m/rad] — the form every equation here wants."""
        import math

        return self.torsion_stiffness_nm_per_deg * 180.0 / math.pi


@dataclass
class MotorParams:
    """Assist motor, as a torque source with the limits that actually bite.

    Modelled at torque level rather than as a full field-oriented drive: at
    vehicle-dynamics timescales the current loop is an order of magnitude
    faster than anything it feeds, so a first-order torque response with the
    right *limits* reproduces the behaviour that matters. The limits are the
    point — a motor sized by its continuous torque and then run into its
    speed limit during an evasive manoeuvre is the failure this exists to show.
    """

    torque_constant: float = 0.055          # K_t [N·m/A]
    inertia: float = 1.2e-4                 # J_m 转子惯量 [kg·m²]
    bandwidth_hz: float = 40.0              # 转矩响应带宽 [Hz]
    #: Sized against the platform's own load model, not chosen. Full-lock
    #: parking at mu = 0.9 needs 413 N.m at the pinion; through the R-EPS
    #: reduction of 63 that is 6.6 N.m at the motor, so 5.5 does not cover this
    #: vehicle — the first sizing run said so, and the default now reflects it.
    peak_torque: float = 8.0                # 峰值转矩 [N·m]
    continuous_torque: float = 4.5          # 连续转矩 [N·m]（热降额目标）
    #: No-load speed; torque falls **linearly to zero** there, which is the
    #: part that is easy to get wrong. A brisk parking turn of ~190°/s at the
    #: hand wheel is ~2000 rpm through the R-EPS reduction, and an earlier
    #: default of 2800 rpm looked like ample headroom — but at 2000 of 2800 the
    #: motor has only 28% of its torque left, so it saturated through every
    #: parking scenario while its *peak* rating was never the problem.
    #: Production EPS BLDCs are rated several times their working speed.
    no_load_speed_rpm: float = 8000.0
    #: Time constant of the thermal state that drives derating [s]. Minutes,
    #: not seconds — this is winding-to-housing, and it is why parking
    #: manoeuvres repeated back-to-back behave differently from the first one.
    thermal_tau_s: float = 120.0
    #: Derate factor applied once the thermal state is fully saturated.
    thermal_derate: float = 0.65


@dataclass
class RackParams:
    """Rack inertia and friction, referred to the rack (N, kg, m).

    Coulomb friction is the term that makes on-centre feel what it is: it sets
    the torque deadband and most of the hysteresis loop that ISO 13674
    measures. It is also the term most often left out of a simple model, which
    is why such models cannot reproduce on-centre behaviour at all.
    """

    mass: float = 3.2                       # 齿条+拉杆等效质量 [kg]
    coulomb_friction_n: float = 260.0       # 齿条库仑摩擦 [N]
    viscous_n_per_mps: float = 900.0        # 齿条粘性阻尼 [N/(m/s)]
    #: Reverse (back-drive) efficiency. Forward efficiency already lives on
    #: SteeringGeometryParams as `rack_mech_efficiency`; the reverse path is a
    #: separate number and is what decides how much road feel survives and how
    #: well the wheel returns. Worm gears are markedly worse backwards than
    #: forwards; ball screws are nearly symmetric.
    reverse_efficiency: float = 0.75


@dataclass
class AngleControlParams:
    """The per-corner angle-tracking control layer.

    Sits between ``delta_cmd`` and the wheel for every corner an
    architecture gives an actuator to (SBW front, RWS rear, all four on
    4WIS). Off by default and the disabled layer is bit-identical to the
    legacy actuator paths; the default controller (``open_loop``) reproduces
    the legacy by-wire update bit-exactly. Scope and contracts:
    docs/steering_control_layer_requirements.md.
    """

    enabled: bool = False
    #: Controller registry name (open_loop / pid_single / … later pid_cascade,
    #: lqr, mpc, smc, adrc, h_inf).
    controller: str = "open_loop"
    #: Constructor kwargs for the controller (gains, bandwidths, …).
    controller_kwargs: dict[str, object] = field(default_factory=dict)

    # ---- torque-mode corner actuator (wheel domain) ------------------------
    #: J of the corner actuator referred to the road wheel [kg·m²].
    plant_inertia_kgm2: float = 0.6
    #: Viscous damping [N·m·s/rad].
    plant_damping_nms_per_rad: float = 4.0
    #: Coulomb friction [N·m] — with stiction hold at rest.
    plant_friction_nm: float = 0.5
    #: Saturation of the actuator torque [N·m] at the wheel. Sized from the
    #: repo's own actuator-sizing module, not a placeholder: full-lock
    #: parking costs ~104 N·m per corner (5.2 kN rack × 0.02 m pinion) and
    #: a 0.5 g evasive at 60 km/h ~40 N·m of aligning load — the original
    #: default of 40 sat exactly ON the highway load with zero margin, and
    #: the validation sweep caught the wheel being blown off its command
    #: (measured: 3° step @60 km/h, wheel driven to −0.34 rad with the
    #: actuator pinned at its limit). 120 covers both with margin.
    plant_peak_torque_nm: float = 120.0
    #: Optional wheel-domain rate limit [rad/s]; None = off (the legacy
    #: rate limit lives in the open_loop controller itself).
    plant_rate_limit_rad_s: float | None = None
    #: Transmission refinement (direction 3): finite coupling stiffness
    #: [N·m/rad] turns the actuator into two masses with a resonance.
    #: None = rigid — the original single-mass path, bit-exact.
    plant_transmission_stiffness_nms_per_rad: float | None = None
    #: Backlash dead band across the transmission [rad]; 0 = none.
    plant_backlash_rad: float = 0.0
    #: Motor-side share of the inertia when compliant [fraction].
    plant_motor_inertia_fraction: float = 0.2

    # ---- angle sensor -------------------------------------------------------
    #: Quantisation step [rad] (~0.03 deg).
    sensor_quant_rad: float = 5.0e-4
    #: Sampling-pipeline delay in control periods.
    sensor_delay_steps: int = 1
    #: Measurement noise std [rad]. Off by default — the open_loop reference
    #: must not have a measurement in its loop, and bit-reproducibility stays
    #: the default everywhere.
    sensor_noise_std_rad: float = 0.0
    #: RNG seed; per-corner sensors offset by the wheel index.
    sensor_seed: int = 1234


@dataclass
class SteeringSystemParams:
    """The steering system as a plant. Off by default.

    `enabled = False` means every equation here is bypassed and the vehicle
    behaves exactly as it did before this package existed — the same
    convention `k = 0` and `sigma = 0` already use in the tyre model. That is
    a hard requirement, not a nicety: the golden baselines must not move
    because a new subsystem was added.
    """

    enabled: bool = False
    column: ColumnParams = field(default_factory=ColumnParams)
    motor: MotorParams = field(default_factory=MotorParams)
    rack: RackParams = field(default_factory=RackParams)
    #: Per-corner angle-tracking control layer. Disabled by default: the
    #: vehicle then behaves exactly as it did before the layer existed.
    angle_control: AngleControlParams = field(default_factory=AngleControlParams)
    #: Which architecture this system is. Decides whether there is a
    #: mechanical front axle to run the plant on at all, and the reduction
    #: ratio — see sim4wis.steering.architecture.
    architecture: str = "r_eps"
    #: Named assist calibration; resolved against the assist-map library.
    assist_map: str = "default"

    #: Target damping ratio of the assist loop.
    #:
    #: Assist is proportional feedback on torsion-bar twist, so it multiplies
    #: the effective stiffness the pinion inertia works against — by the boost
    #: ratio, which is ~59x at parking on the default map. The mechanical
    #: damping present (torsion-bar internal plus rack viscous, 0.41 N*m*s/rad)
    #: leaves that loop at zeta = 0.0036, i.e. undamped: the pinion overshoots
    #: the hand wheel, twist goes negative, assist reverses, and it diverges.
    #: Measured before this term existed: assist *raised* parking effort from
    #: 96.5 to 174.3 N*m.
    #:
    #: This is not a modelling artefact. It is why every production EPS carries
    #: a damping function in the ECU. The damping gain is scheduled to hold
    #: this ratio as boost varies; a real calibration uses a table, and this is
    #: the shape that table approximates.
    assist_damping_ratio: float = 0.6

    #: Motor-to-pinion reduction of the *physical* drive.
    #:
    #: Deliberately separate from `SteeringGeometryParams.motor_gear_ratio`,
    #: which defaults to 10 and feeds the existing `motor_torque_demand`
    #: diagnostic. The two disagree by an order of magnitude and both cannot be
    #: right: at ratio 10 a 10.4 kN parking rack force needs a 20.8 N·m motor,
    #: where production EPS motors are 3–6 N·m. A real R-EPS ball-screw plus
    #: belt drive works out near 63, derived rather than guessed:
    #:     F_rack/tau_m = 2*pi*i_belt/lead = 2*pi*2.5/0.005 = 3142 N per N*m
    #:     N = (F_rack/tau_m) * r_pinion = 3142 * 0.020 = 63
    #: which needs 3.3 N*m for a 10.4 kN parking force — inside a 5.5 N*m peak.
    #:
    #: Changing the legacy field would move an existing output, so the plant
    #: carries its own and the two get reconciled when the architecture layer
    #: defines the reduction chain per architecture (v2 V2). Recorded in
    #: docs/v2_steering_platform_plan.md §5.
    motor_gear_ratio: float = 63.0

    def describe(self) -> dict[str, object]:
        """Flat summary for reports and the capability endpoint."""
        return {
            "enabled": self.enabled,
            "torsion_stiffness_nm_per_deg": self.column.torsion_stiffness_nm_per_deg,
            "sensor_range_nm": self.column.sensor_range_nm,
            "motor_peak_torque_nm": self.motor.peak_torque,
            "motor_continuous_torque_nm": self.motor.continuous_torque,
            "motor_no_load_speed_rpm": self.motor.no_load_speed_rpm,
            "rack_coulomb_friction_n": self.rack.coulomb_friction_n,
            "rack_reverse_efficiency": self.rack.reverse_efficiency,
            "architecture": self.architecture,
            "assist_map": self.assist_map,
            "motor_gear_ratio": self.motor_gear_ratio,
        }
