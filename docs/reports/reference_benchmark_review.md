# Reference Benchmark Review Report

- Data root: `validation_data`
- Benchmarks checked: 2
- Passing benchmarks: 2/2
- Passing independent external/measured benchmarks: 0
- Evidence boundary: this report summarizes reproducibility checks only; it does not upgrade validation levels without human review.

> No passing independent external-tool, bench, scaled-vehicle, or full-vehicle benchmark is present.

## analytic_steady_circle_30kmh - PASS

- Source: analytic / closed-form ideal Ackermann steady circle / sim4wis-analytic-v1
- Checked metrics: 6
- Limitations: Analytic reference covers kinematic ideal-Ackermann geometry only.; It does not validate tyre forces, actuator lag, load transfer, suspension compliance, or measured vehicle behaviour.; It is L3 analytic-reference evidence, not external-tool or L4 measured evidence.

| Metric | Sim | Reference | Delta | Tolerance | Status |
|---|---:|---:|---:|---:|---|
| `yaw_rate_peak_dps` | 7.8559 | 7.8559 | 3.3412e-07 | 0.001 | PASS |
| `vy_peak_kmh` | 1.9984e-16 | 0 | 1.9984e-16 | 0.001 | PASS |
| `speed_error_rms_kmh` | 6.39488e-15 | 0 | 6.39488e-15 | 0.001 | PASS |
| `pose_y_peak_abs_m` | 32.9298 | 32.9298 | 1.11488e-06 | 0.05 | PASS |
| `trajectory_error_rms_m` | 6.7795e-07 | 0 | 6.7795e-07 | 0.02 | PASS |
| `trajectory_error_peak_m` | 1.49821e-06 | 0 | 1.49821e-06 | 0.03 | PASS |

### Reviewer Notes

```markdown
# analytic_steady_circle_30kmh

This benchmark checks the kinematic ideal-Ackermann steady-circle promise against a closed-form analytic reference.

Reference construction:

- Default `VehicleParams()` geometry.
- Inner-front-wheel angle = 1.508559° (`unit: front_deg`, v0.100 — bypasses the
```

## analytic_step_steer_30kmh - PASS

- Source: analytic / closed-form ideal Ackermann step steer / sim4wis-analytic-v1
- Checked metrics: 9
- Limitations: Analytic reference covers kinematic ideal-Ackermann step-steer geometry only.; It intentionally excludes steering actuator lag, tyre force build-up, load transfer, suspension compliance, and measured vehicle behaviour.; It is L3 analytic-reference evidence, not external-tool or L4 measured evidence.

| Metric | Sim | Reference | Delta | Tolerance | Status |
|---|---:|---:|---:|---:|---|
| `yaw_rate_peak_dps` | 7.8559 | 7.8559 | 3.3412e-07 | 0.001 | PASS |
| `yaw_gain_dps` | 5.20755 | 5.20755 | 2.74722e-11 | 0.001 | PASS |
| `yaw_rise_time_s` | 0 | 0 | 0 | 0.001 | PASS |
| `yaw_settling_time_s` | 0 | 0 | 0 | 0.001 | PASS |
| `vy_peak_kmh` | 1.9984e-16 | 0 | 1.9984e-16 | 0.001 | PASS |
| `speed_error_rms_kmh` | 6.39488e-15 | 0 | 6.39488e-15 | 0.001 | PASS |
| `pose_y_peak_abs_m` | 25.8072 | 25.8072 | 9.2717e-07 | 0.05 | PASS |
| `trajectory_error_rms_m` | 4.88147e-07 | 0 | 4.88147e-07 | 0.02 | PASS |
| `trajectory_error_peak_m` | 1.15555e-06 | 0 | 1.15555e-06 | 0.03 | PASS |

### Reviewer Notes

```markdown
# analytic_step_steer_30kmh

This benchmark checks the kinematic ideal-Ackermann step-steer path against a closed-form analytic reference.

Reference construction:

- Default `VehicleParams()` geometry.
- Inner-front-wheel step angle = 1.508559°, `speed = 30 km/h`, `step_time = 1.0 s`.
```
