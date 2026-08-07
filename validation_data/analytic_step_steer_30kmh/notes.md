# analytic_step_steer_30kmh

This benchmark checks the kinematic ideal-Ackermann step-steer path against a closed-form analytic reference.

Reference construction:

- Default `VehicleParams()` geometry.
- Inner-front-wheel step angle = 1.508559°, `speed = 30 km/h`, `step_time = 1.0 s`.
  Declared as `unit: front_deg` (v0.100) so the driver-input feel layer is
  bypassed — the benchmark measures the vehicle path, not the feel mapping.
- Before the step: straight-line motion at constant speed.
- After the step: `kappa = tan(δ) / (L/2 + (tf/2)·tan(δ))`, `yaw_rate = vx * kappa`.
  This is `ideal_ackermann`'s own geometry — it steers both axles symmetrically,
  so the ICR sits on the lateral axis through the vehicle centre and the lever
  arm is `L/2`, not `L`. Using the bicycle relation `tan(δ)/L` here instead
  would understate the curvature for a given angle by ~49%.
- The angle is chosen so `kappa` equals the pre-v0.100 value
  `_max_curvature(VehicleParams()) · 0.05 = 0.0164534 /m` exactly, which is why
  `reference.csv` is unchanged from the normalised-input era.
- Continuous world trajectory after the step uses the same circular-arc formula as `analytic_steady_circle_30kmh`, offset by the pre-step straight distance.

Review boundary: this is an analytic geometry benchmark. It is useful for L3 reference-review evidence for the kinematic/ideal-Ackermann path, but it is not CarSim/CarMaker, bench, scaled-vehicle, or full-vehicle evidence.
