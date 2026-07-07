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
