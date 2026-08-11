# study 层 — 使用指南

> 面向：任何要用这个仿真器回答一个问题的人（以及替你干活的 Agent）　·　2026-08-12

设计取舍见 [`agent_interface_design.md`](agent_interface_design.md)。本文讲怎么用。

---

## 1. 一句话

`Experiment` 是"跑这个机动，给我通道数据"；**`StudySpec` 是"这是一个问题，
这是我要它在哪个网格上被回答，这些指标回答它，这条判据决定结论"**。

一次 study = 一份可入 repo、可 diff、可复跑的文件。**只存在于脚本里的研究，
是没人能复核的研究。**

---

## 2. 三条命令

```bash
sim4wis study run docs/examples/steady_yaw_vs_speed.yaml --dry-run   # 校验+估成本，不跑
sim4wis study run docs/examples/steady_yaw_vs_speed.yaml             # 跑
sim4wis study list                                                    # 看历史
sim4wis capabilities                                                  # 有哪些模型/指标/架构
```

退出码：`0` 判据全过 · `2` 判据有失败 · `1` spec 非法或出错。可以直接进 CI。

后端没起也能跑（自动转本地进程并说明）。起着后端时走 `/api/study/*` ——
和以后的 MCP server 同一条契约，不会出现"CLI 能做但 Agent 做不到"。

---

## 3. 写一份 spec

```yaml
study: steady_yaw_vs_speed
question: 1° 前轮阶跃下，稳态横摆角速度随车速怎么变？

model: simplified_dynamic        # 必填、无默认值 —— 见 §6

baseline:                        # 就是一个 Experiment，语法完全一致
  strategy: ideal_ackermann
  maneuver:
    steps:
      - {duration: 2.0, speed_kmh: 60, steer: {kind: constant, amplitude: 0.0}}
      - {duration: 14.0, speed_kmh: 60,
         steer: {kind: step, amplitude: 1.0, unit: front_deg, t_step: 0.5}}

sweep:
  speed_kmh:
    values: [30, 40, 50, 60, 80, 100]
    bind: maneuver.steps.1.speed_kmh      # ← 必填，见下

metrics:
  - yaw_gain_dps                          # 内置 KPI，直接取
  - name: beta_deg                        # 表达式指标
    expr: degrees(atan2(steady(vy), steady(vx)))

compare:
  group_by: speed_kmh
  against: {speed_kmh: 30}

criteria:
  - {metric: beta_deg, must: abs < 3.0, at: all}

report: {template: sweep}
```

### `bind` 为什么必填

研究变量名（`speed_kmh`）和 Experiment 的点路径
（`maneuver.steps.1.speed_kmh`）不是一套词汇。绑一次，后面表格、分组、判据全用
研究变量名。

**更重要的是它会被校验。** 批跑的写入器遇到不存在的键会**自动创建**，所以一个
拼错的点路径会安静地产出"一整格互相完全相同的 run"——而对比表看起来毫无异常。
这是这里能犯的最坏的错，所以在声明阶段就拒绝。

### `criteria` 是什么

**结论必须在跑之前被写成可判定的形式。** 跑完给 PASS/FAIL，而不是事后从一堆数里
叙述一个故事。

- 选择器匹配不到任何 cell → **FAIL**（空集上的"全部通过"是坏研究报告成功的方式）
- 失败会点名最差的那个 cell，不是只说"有失败"
- 报告里**只有判据被允许写成结论**

---

## 4. 指标两档

**内置**：直接读 run 已存的 KPI，与「分析」页口径永远一致。
`sim4wis capabilities` 可列全。

**表达式**：写在 spec 里，对 run 的通道求值。这一档是**Agent 可以不经审查就写**的，
所以刻意做弱：无语句、无属性访问、无 import、无 lambda、无推导式；
返回数组而不是标量会被拒绝（而不是被悄悄归约成某个作者没写的指标）。

可用：`steady()`（尾段均值）`peak()` `rms()` `mean()` `first()` `last()`
`degrees()` `radians()` `atan2()` `sqrt()` `clip()` `sign()` 三角函数，
以及所有通道名和 `t` / `pi` / `g`。

`steady()` 单独说一句：大家都在自己写"稳态值"，各写各的尾窗。统一在这里，
至少保证同一张表里两个指标是用同一种方式量出来的。

---

## 5. Token 预算是设计约束

一次 10 秒 run ≈ 667 采样 × 35 通道 ≈ **10 万 token**。所以：

- `run` 只回**汇总表**（约 1–2 k token）
- 轨迹要**显式**要，且强制降采样：`sim4wis study trace <run_id> --channels vx,ay --max-points 200`
- 报告只回**路径**

**Agent 永远不该在上下文里看到完整轨迹。** 要看波形就出图，要算东西就定义指标。

---

## 6. `model` 为什么没有默认值

运动学模型的 `ax = ay = 0` 是**故意的**（代码注释：宁可不报，也不给一个看着合理的假数）。
给 `model` 一个默认值，就是让人——或 Agent——在运动学上无意识地问动力学问题，
拿到 0，然后写进结论。

同理架构：问 C-EPS 后轮相位会被拒绝并指向 `eps_rws`。

---

## 7. 出处

每次 study 存 `studies/<id>/`，含 spec、结果表、报告，以及
版本 / git sha（脏树标 `-dirty`）/ **params_hash**（完全解析后的参数）/ 模型 / 策略 / dt。

params_hash 用的是**解析后**的参数，不是 spec 里的 `vehicle` 块——否则两次 study
看起来一样，其中一次却悄悄加载了不同的 profile。

---

## 8. 还没有的

- **求解轴 `solve_for`**（等 a_y 反解转角）：schema 里已声明，运行时拒绝。
  它比二分法难，因为 a_y 对转角**过峰后不单调**，天真的求根会收敛到下降支——
  那会让整张对比表看起来正常而每格比的都不是同一件事。
- **能力边界守卫的自动裁决**：`metrics[].requires` 已声明，尚未强制。
  现在问运动学要 `slip_alpha_peak_deg` 会得到"缺失"而不是"拒绝"。
- **MCP server**：契约（`/api/study/*`）已就位，server 未写。
