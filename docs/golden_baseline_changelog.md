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

## 2026-08-09 - 底盘调校：让默认车辆具备量产车的不足转向

- Commit / branch: 驾驶动态真实性迭代
- 触发：新增的 `scripts/driving_dynamics_review.py`（18 项标准客观指标对照
  该车型级别的公开典型区间）测出**不足转向梯度 K = 0.07 deg/g**，即中性转向。
  量产乘用车按设计都是 1–4 deg/g 的不足转向——极限时先推头（可读、可修正），
  而不是先甩尾。K≈0 还意味着横摆增益 r/δ = v/(L+K·v²) 随车速**线性增长不封顶**，
  高速会发贼。
- 根因：线性区的 K 由 `K = W_f/C_f − W_r/C_r` 决定。这台车 51/49 的轴荷配上
  四轮同一个 `tire_c_alpha` ⇒ K ≈ 0。**这不是 bug，是参数没调过。**
- 改动（两项都是新增能力 + 默认值调整）：
  - `tire_c_alpha_front_scale = 0.80` / `tire_c_alpha_rear_scale = 1.20`
    （新增）。前后侧偏刚度差，等效吸收进滑移角（与外倾推力同一手法，对线性和
    Pacejka 都精确）。实测 K = **1.49 deg/g**，与解析式 1.55 相差 0.06——那 0.06
    是载荷转移的二阶贡献。**这是调校选择，不是实测**：OEM 不公布轴侧偏刚度。
  - `roll_stiffness_front_frac = 0.60`（新增）。侧向载荷转移改按**侧倾刚度**分配
    而非静态轴荷；多体模型的防倾杆从「纯整车力矩」改为每角的力，这样它才真的转移
    载荷（此前它只阻侧倾、不影响操稳平衡，是项目遗留清单上的
    「防倾杆刚度分配横向载荷转移」）。注意：**这一项对线性区的 K 几乎没有影响**
    （c_α ∝ Fz^0.8 下现实转移量只让轴损失不到 1% 能力，K 只动 0.01 deg/g）——
    它影响的是极限行为，不是线性区平衡。设为 0 可回到旧行为。
- 基线影响：五组全动。方向一致——车更稳了：
  - `step_steer` / `iso3888`：横摆峰值 −16%，齿条力 −17%，响应更快（rise time −25%）。
    不足转向让同样的前轮角产生更小的横摆，符合预期。
  - `sw_curve60` baseline：xtrack −15%、dyaw_peak −20%、beta_peak −14%。刚度更高的
    后轴抵抗单轮故障引起的横摆更好。
  - `sw_straight100` mitigated：xtrack_react +91%（0.032→0.062 m，绝对值仍极小）、
    dpsi_2s +14%。缓解控制器的增益是按旧平衡整定的，新平衡下略有失配。
  - **安全研究的所有 C 等级不变**，报告与 metrics.json 已重算。
- Reason:
  - [ ] Model correction
  - [x] Parameter correction（底盘调校）
  - [x] Model correction（防倾杆载荷转移，此前该机制缺失）
  - [ ] Experiment definition change
- Evidence:
  - `backend/.venv/bin/python scripts/driving_dynamics_review.py` → 18/18
  - `backend/.venv/bin/python scripts/check_golden_experiments.py --update`
  - `backend/.venv/bin/python scripts/study_single_wheel_failure.py`（C 等级逐项比对）
- Review notes:
  - 关闭这两项（`*_scale = 1.0`、`roll_stiffness_front_frac = 0`）可完整回到
    2026-08-07 的基线，已验证 golden 逐位不变。
  - `sw_straight100` mitigated 的退化提示 `fault_reconfig` 的 `k_yaw` 是按中性车
    整定的，值得随平衡一起重调——**未做**，留作后续。

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
