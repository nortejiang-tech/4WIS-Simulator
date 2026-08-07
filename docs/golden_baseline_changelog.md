# Golden Baseline Change Log

Use this file when `docs/golden_experiments.json` changes intentionally.
Every entry must explain why the baseline moved. Do not update the JSON only.

## Template

```markdown
## YYYY-MM-DD - <short change title>

- Commit / branch:
- Changed baseline groups:
  - `step_steer_60kmh`:
  - `iso3888_dlc_60kmh`:
  - `sw_straight100_rl_stuck_value_baseline`:
  - `sw_straight100_rl_stuck_value_mitigated`:
  - `sw_curve60_fl_free_caster_baseline`:
- Reason:
  - [ ] Model correction
  - [ ] Parameter correction
  - [ ] Numerical integration / tolerance change
  - [ ] Experiment definition change
- Evidence:
  - `backend/.venv/bin/python scripts/check_golden_experiments.py --update`
  - `backend/.venv/bin/python scripts/check_golden_experiments.py`
  - Other focused tests:
- Review notes:
```

## 2026-08-07 - Correct the front_deg geometry; restore the safety baseline

Supersedes the 2026-08-06 entry below, which promoted values produced by a
mis-derived angle→curvature conversion.

- Commit / branch: v0.100.0 review fixes
- What was wrong: the feel layer converted its front-axle angle to curvature
  with the bicycle relation `κ = tan δ / L`. `ideal_ackermann` does not steer
  like a bicycle — it steers both axles symmetrically and places the ICR on the
  lateral axis through the vehicle *centre*, so the lever arm is `L/2` and the
  track enters as well:

      κ(δ) = tan δ / (L/2 + (tf/2)·tan δ)          [inverse of _max_curvature]

  Using the bicycle relation silently turned the vehicle into a front-steer car:
  at full lock κ fell 0.3291 → 0.2216, i.e. the minimum turning radius grew from
  3.04 m to 4.51 m (+48%) — while the v0.100.0 notes claimed it was unchanged.
  Interactively, every normalised steer input produced roughly half the intended
  curvature, which is what moved `sw_curve60` by 35–40%.

  δ is the *inner* wheel's angle, so the relation must also be odd-symmetric —
  "inner" swaps sides with the turn and the half-track term always widens the
  radius. The first cut of the fix let a negative `tan δ` into that term, which
  shrank it instead: a right turn came out tighter than the mirror-image left
  turn (κ 0.568 vs 0.301 at full lock) and the outer wheel saturated, so the
  four wheels stopped sharing one ICR (smoke test `瞬心一致性`, spread 10 m).
  Both halves are covered by tests now.
- Changed baseline groups (all deltas quoted against **v0.99.3**, since the
  2026-08-06 numbers are withdrawn):
  - `step_steer_60kmh`: amplitude re-derived with the correct geometry,
    `0.05 normalised → 1.508559°` (was 2.976°). Every dynamic KPI is now
    **identical to v0.99.3 to 5+ decimals**; only `yaw_gain_dps` changes, and
    only in *unit* — 313.57 "per normalised input" → 10.393 "per degree of
    inner-front-wheel angle". This is the byte-identical result the previous
    entry claimed but did not have.
  - `iso3888_dlc_60kmh`: `0.06 normalised → 1.814818°` (was 3.570°). Back to
    **within 0.05%** of v0.99.3 on every dynamic KPI (`yaw_rate_peak`
    18.5436 → 18.5478); `speed_error_rms_kmh` and `steer_energy_nms` move ~0.3%,
    which is the κ(δ) curve being mildly nonlinear where the normalised form was
    linear — the two agree at the plateau and differ marginally mid-transient.
    Commanded peak angle is identical in both (1.81482°), verified by replaying
    v0.99.3 side by side.
  - `sw_curve60_fl_free_caster_baseline`: **restored to v0.99.3** — every KPI
    within 0.03%. The study script was also migrated to `unit: front_deg` so
    future feel-layer tuning can no longer disturb it, the same decoupling the
    other two experiments got. Its `a_y ≈ 4.6 m/s²` scenario label is accurate
    again (measured 4.57), and every C-class in
    `single_wheel_failure_safety_analysis.html` is unchanged.
  - `sw_straight100_*`: unchanged (straight-line, no steer input).
- **Net effect: the migration is behaviour-neutral.** Every dynamic KPI across
  all five baselines is within 0.35% of v0.99.3, most at 0.000%. The only KPI
  that genuinely changes is `step_steer_60kmh.yaw_gain_dps`, and only in unit
  (313.57 per normalised input → 10.393 per degree of wheel angle).
- Reason:
  - [x] Model correction (angle → curvature geometry)
  - [ ] Parameter correction
  - [ ] Numerical integration / tolerance change
  - [x] Experiment definition change (normalised → front_deg amplitudes)
- Evidence:
  - `backend/.venv/bin/python scripts/check_golden_experiments.py --update`
  - `backend/.venv/bin/python scripts/check_golden_experiments.py`
  - `backend/.venv/bin/python scripts/check_reference_benchmarks.py --report docs/reports/reference_benchmark_review.md`
  - v0.99.3 side-by-side replay from the clean worktree at `710720d`.
- Review notes:
  - The conversion round-trips `_max_curvature` exactly (`κ → δ → κ` to 1e-16),
    which is the property the previous derivation lacked.
  - Both analytic benchmarks pass unchanged: `reference.csv` never had to move,
    because the corrected angles reproduce the original curvature exactly.

## 2026-08-06 - Steering feel layer + steer-unit decoupling (work-package B) [WITHDRAWN]

> Superseded by the 2026-08-07 entry above. The conversion used here
> (`κ = tan δ / L`) does not match `ideal_ackermann`'s geometry; the
> `sw_curve60` movement it records as "intended" was an artefact of that, and
> the "byte-identical" claim for step_steer/dlc was off by ~1e-4 because the
> amplitudes were truncated. Kept for the audit trail.


- Commit / branch: v0.100.0 work-package B
- Changed baseline groups:
  - `step_steer_60kmh`: switched to `unit: front_deg` (amplitude 0.05 →
    2.976°) and the strategy now bypasses the feel layer for it, so the
    vehicle response is byte-identical. `yaw_gain_dps` KPI changed *meaning*
    (was 313.57 "per normalised input", now 5.27 "per degree front angle").
  - `iso3888_dlc_60kmh`: same — `unit: front_deg` (0.06 → 3.570°), bypass,
    identical response.
  - `sw_curve60_fl_free_caster_baseline`: this curve-following test uses a
    **normalised** steer input (amplitude 0.05 at 60 km/h). It now passes
    through the new feel layer (variable gear ratio + μ-aware soft limit),
    so the effective front angle at 60 km/h is smaller than before → the
    path-tracking KPIs (`xtrack_react`, `dyaw_peak_dps`, `beta_peak_deg`)
    all moved. This is the intended behaviour change: the feel layer exists
    precisely so a normalised input no longer over-drives the front axle at
    speed.
  - `sw_straight100_*`: unchanged (straight-line, no steer input).
- Reason:
  - [ ] Model correction
  - [ ] Parameter correction
  - [ ] Numerical integration / tolerance change
  - [x] Experiment definition change
- Conversion (exact, reversible) for the front_deg experiments:
  - κ_max = 0.3291 /m (R_min = 3.04 m); `steering = (tan(δ)/L) / κ_max`
    gives `0.05 → 2.976°` and `0.06 → 3.570°`. The bypass delivers the raw
    radian front angle via `mode_params["steer_raw_rad"]`, so the feel layer
    never runs for these.
- Evidence:
  - `PYTHONPATH=backend/src python3 scripts/check_golden_experiments.py --update`
  - `PYTHONPATH=backend/src python3 scripts/check_golden_experiments.py`
- Review notes:
  - step_steer / dlc: vehicle trajectory and yaw-rate history unchanged;
    only amplitude-normalised KPIs changed units.
  - sw_curve60: behaviour change is the point of B1/B2 — a normalised steer
    at speed now commands a physically reasonable front angle, not a 9g
    lateral-accel demand. Re-baselined to the new (correct) response.

## 2026-07-07 - Add single-wheel-failure quick goldens

- Commit / branch: pending local commit
- Changed baseline groups:
  - `sw_straight100_rl_stuck_value_baseline`: added C3 fastest-regression sample for the ASIL-D source case.
  - `sw_straight100_rl_stuck_value_mitigated`: added C2 mitigated counterpart for the same case.
  - `sw_curve60_fl_free_caster_baseline`: added C2 front free-caster sample for the passive-safety behavior.
- Reason:
  - [ ] Model correction
  - [ ] Parameter correction
  - [ ] Numerical integration / tolerance change
  - [x] Experiment definition change
- Evidence:
  - `backend/.venv/bin/python scripts/check_golden_experiments.py --update`
  - `backend/.venv/bin/python scripts/check_golden_experiments.py`
- Review notes:
  - The new samples reuse `scripts/study_single_wheel_failure.py` scenario, fault, mitigation, metric, and C-class functions.
  - The gate does not require matplotlib because the research script now loads plotting dependencies only when building the full HTML report.
