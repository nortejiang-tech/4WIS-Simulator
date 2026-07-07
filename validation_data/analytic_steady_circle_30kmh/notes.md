# analytic_steady_circle_30kmh

This benchmark checks the kinematic ideal-Ackermann steady-circle promise against a closed-form analytic reference.

Reference construction:

- Default `VehicleParams()` geometry.
- `driver_steering = 0.05`, `speed = 30 km/h`.
- `kappa = _max_curvature(VehicleParams()) * driver_steering`.
- `yaw_rate = vx * kappa`.
- Continuous world trajectory: `x = sin(yaw_rate * t) / kappa`, `y = (1 - cos(yaw_rate * t)) / kappa`.

Review boundary: this is an analytic geometry benchmark. It is useful for L3 reference-review evidence for the kinematic/ideal-Ackermann path, but it is not CarSim/CarMaker, bench, scaled-vehicle, or full-vehicle evidence.
