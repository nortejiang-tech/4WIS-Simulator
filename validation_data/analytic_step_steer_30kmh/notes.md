# analytic_step_steer_30kmh

This benchmark checks the kinematic ideal-Ackermann step-steer path against a closed-form analytic reference.

Reference construction:

- Default `VehicleParams()` geometry.
- `driver_steering = 0.05`, `speed = 30 km/h`, `step_time = 1.0 s`.
- Before the step: straight-line motion at constant speed.
- After the step: `kappa = _max_curvature(VehicleParams()) * driver_steering`, `yaw_rate = vx * kappa`.
- Continuous world trajectory after the step uses the same circular-arc formula as `analytic_steady_circle_30kmh`, offset by the pre-step straight distance.

Review boundary: this is an analytic geometry benchmark. It is useful for L3 reference-review evidence for the kinematic/ideal-Ackermann path, but it is not CarSim/CarMaker, bench, scaled-vehicle, or full-vehicle evidence.
