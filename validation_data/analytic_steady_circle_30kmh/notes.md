# analytic_steady_circle_30kmh

This benchmark checks the kinematic ideal-Ackermann steady-circle promise against a closed-form analytic reference.

Reference construction:

- Default `VehicleParams()` geometry.
- Inner-front-wheel angle = 1.508559° (`unit: front_deg`, v0.100 — bypasses the
  driver-input feel layer so the benchmark measures the vehicle, not the
  mapping), `speed = 30 km/h`.
- `kappa = tan(δ) / (L/2 + (tf/2)·tan(δ))` — `ideal_ackermann`'s symmetric-4WIS
  geometry, ICR on the lateral axis through the vehicle centre. The angle is
  chosen so this equals the pre-v0.100
  `_max_curvature(VehicleParams()) * 0.05 = 0.0164534 /m` exactly, leaving
  `reference.csv` unchanged.
- `yaw_rate = vx * kappa`.
- Continuous world trajectory: `x = sin(yaw_rate * t) / kappa`, `y = (1 - cos(yaw_rate * t)) / kappa`.

Review boundary: this is an analytic geometry benchmark. It is useful for L3 reference-review evidence for the kinematic/ideal-Ackermann path, but it is not CarSim/CarMaker, bench, scaled-vehicle, or full-vehicle evidence.
