# Agent 研究接口设计 — study 层 + MCP/CLI 双门面

> 状态：**设计稿，待评审**　·　基线：v0.101.1　·　2026-08-10
>
> 目标：让 Agent 能按人的研究需求高效驱动这个仿真器，同时完整保留人自己用 GUI 的能力。
> 范围经确认为**全量**，含实时驾驶控制；形态为**共享 study 层 + MCP + CLI 两个薄门面**。
> 本文只定形状与语义，不含实现。

---

## 1. 先说结论

**缺的不是 API，是 API 之上的"研究层"，外加两条硬约束。**

`backend/src/sim4wis/experiment/` 已经是一套很干净的地基：`Experiment`（声明式）→ `SimSession`
（无头、确定性、快于实时）→ `compute_kpis` → `save_run` → `POST /api/batch`（带点路径 variants
的笛卡尔展开 + 作业轮询）。批跑走 `asyncio.to_thread`，**实时仿真回路不受影响**；`SimSession`
自己 new 模型，不碰全局单例。

所以有一个关键性质**今天就已经成立**：Agent 跑批量实验与人在 GUI 里开车互不干扰，且 Agent 产出的
run 会直接出现在人的「分析」页。人机共存的地基是对的，不需要重建。

真正缺的是三块，加两条约束：

| 缺口 | 证据 |
|---|---|
| **自定义指标** | `compute_kpis` 是固定指标集；decoupling study 需要稳态比例、相关法估相位、稳定判据、注入噪声测敏感度 |
| **跨 run 聚合** | 批跑只还 runs，"架构 × 控制律 × 车速"的对比、找符号翻转全是手写的 |
| **环内注入** | study harness 在控制律跑完后夹后轮指令（架构的后轮权限），`Experiment` 表达不了 |
| **约束 A：能力边界** | 运动学模型 `ax = ay = 0` 是故意的；Agent 会拿到 0 并写进结论 |
| **约束 B：token 预算** | 一次 10 s run ≈ 667 采样 × 35 通道 ≈ 10 万 token；一次 15 变体 sweep 直接爆上下文 |

**不要包装那七十来个 REST 端点。** 那是给 GUI 用的 CRUD，Agent 拿到只会先烧两万 token 学 API，
然后调错。

---

## 2. 设计目标与非目标

### 目标

1. **一次调用 = 一个研究问题**，而不是一个 HTTP 动作。
2. **答不了就拒绝**，绝不返回一个语义上无效的数（这是 Agent 做研究最容易产出的垃圾）。
3. **默认返回汇总**，原始数据靠显式索取且强制降采样，大产物一律回文件路径。
4. **人永远优先**。人碰 GUI 的那一刻，Agent 对实时仿真的控制立即失效，无需协商。
5. **每个结论可复现**：spec 摘要 + git sha + params 哈希 + 模型 + 判据，都随结果返回。
6. **现有三份研究能被重写成 spec**。这是验收标准，也是"这层是否真的够用"的唯一诚实检验。

### 非目标

- **不自动生成论文级报告**。现有两份研究的报告层是 2 200 行 bespoke 代码，跑在 126 行共享
  helper 之上。这个比例说明报告结构本身就是研究的一部分，不该被过度泛化。study 层提供
  标准汇总表 + 图元 + 默认模板；真要出 `steering_decoupling_value.html` 那种东西，仍然写
  专门的 `build_report.py`，只是数据来源换成 study 层。
- **不替代 `Experiment`**。StudySpec 里的单次运行**就是**一个 `Experiment`，复用现有 schema、
  存储、KPI、GUI 展示。
- **不做多租户/鉴权**。后端 `host="127.0.0.1"`，单机单人，本文所有"权限"讨论都是防误伤，
  不是防攻击。

---

## 3. 分层

```
   ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
   │  MCP server  │  │     CLI      │  │   前端 GUI    │  ← 三个门面，无业务逻辑
   │  (~9 tools)  │  │ sim4wis study│  │  （人在用）   │
   │  独立进程/stdio│  └──────┬───────┘  └──────┬───────┘
   └──────┬───────┘         │                 │
          └──────── HTTP ───┴────────┬────────┘
                             ┌───────▼──────┐
                             │  FastAPI      │  /api/study/*  /api/*
                             └───────┬──────┘
                 ┌──────────────────▼┐
                 │  study 层          │         ← 本设计的主体
                 │ spec → runs → metrics
                 │  → compare → report
                 │ + capability envelope
                 │ + realtime lease
                 └──────┬───────┘
        ┌───────────────┼────────────────┐
   ┌────▼─────┐   ┌─────▼──────┐  ┌──────▼──────┐
   │ Experiment│   │ Simulator  │  │ run store   │  ← 已存在，不动
   │ SimSession│   │ (realtime) │  │ runs/*/meta │
   │ batch     │   │ ScriptRunner│ │ + GUI 分析页 │
   └───────────┘   └────────────┘  └─────────────┘
```

study 层住在 `backend/src/sim4wis/study/`，是普通 Python 包，经 `/api/study/*` 出面。三个门面都只是
它的调用方，**没有任何逻辑住在门面里**——这是"多花一个门面几乎不要钱"的前提，也是决策 b（§13）
能成立的前提：MCP server 不 import `sim4wis`，只发 HTTP，因此可以和后端各自演进。

---

## 4. StudySpec

一次研究的完整声明。YAML 或 JSON，可入 repo、可 diff、可回归。

```yaml
study: rear_angle_authority
question: 后轮转角上限 2°→10°，横摆响应与稳态侧偏角怎么变？拐点在哪？

model: multibody                 # 显式声明 → 边界守卫据此裁决
baseline:                        # 就是一个 Experiment（复用现有 schema）
  vehicle: { profile: ls9 }
  strategy: rear_wheel_steer
  maneuver:
    steps:
      - duration: 8
        speed_kmh: 80
        steer: { kind: step, amplitude: 3, unit: front_deg }

sweep:                           # 笛卡尔积 → 现有 variants 点路径覆盖
  rear_limit_deg:
    values: [2, 4, 6, 8, 10]
    bind: vehicle.overrides.rear_steer_limit_deg
  speed_kmh:
    values: [60, 100, 140]
    bind: maneuver.steps.0.speed_kmh

metrics:                         # 内置 KPI + 自定义
  - yaw_gain_dps
  - yaw_overshoot_pct
  - yaw_rise_time_s
  - beta_steady_deg              # 自定义，见 §7

compare:
  group_by: rear_limit_deg
  against: { rear_limit_deg: 2 } # 相对基准列出 Δ 与 Δ%

criteria:                        # 判据 —— 让结论可证伪
  - { metric: yaw_overshoot_pct, must: "< 20", at: all }
  - { metric: beta_steady_deg,  must: "abs < 1.0", at: "speed_kmh == 140" }

report:
  template: sweep                # 默认模板；bespoke 报告另写
  out: docs/reports/rear_angle_authority.html
```

### 设计说明

- **`sweep` 的 `bind` 是必需的**，因为研究变量名（`rear_limit_deg`）和 `Experiment` 的点路径
  （`vehicle.overrides.rear_steer_limit_deg`）不是一回事。让 Agent 直接写点路径可读性太差，
  出了错也难查；显式 `bind` 一次，后面所有表格、分组、判据都用研究变量名。
- **`criteria` 是这份 spec 里最反 Agent 幻觉的字段**。它逼着"结论"在跑之前就被写成可判定的
  形式，跑完给 PASS/FAIL，而不是让 Agent 事后从一堆数里叙述一个故事。
- **`model` 必填、不给默认值**。默认值会让 Agent 无意识地在运动学上问动力学问题。

### 标定扫描 `solve_for`（决策 a）

有一类扫描的取值**跑之前不知道**，要先解出来。decoupling study 的核心比较就是这种：六个控制律
在四个速度上比，必须比在**等侧向加速度**下——按等转角比是不公平的，因为各律增益本就不同。
所以每个 (律, 速度) 格子都要先反解出"能产生 4 m/s² 的前轮转角是多少"。

这在 schema 里表达成一根**求解轴**：

```yaml
sweep:
  law:        { values: [yaw_fb, zero_beta, model_follow], bind: strategy.mode_params.law }
  speed_kmh:  { values: [40, 80, 120, 140], bind: maneuver.steps.0.speed_kmh }
  steer_deg:                                     # ← 值由求解决定，不由你给
    solve_for: { metric: ay_steady_ms2, target: 4.0, tol: 0.02 }
    bind:      maneuver.steps.0.steer.amplitude
    bracket:   [0.2, 8.0]                        # 只在上升支内搜索，见下
    max_iter:  12
    cache:     docs/reports/<study>/cal.json     # 复用既有做法
```

语义：对其它扫描轴的**每一个组合**，在 `bracket` 内对 `bind` 的参数做一维求根，直到 `metric`
落到 `target ± tol`。解出的值成为该格子的轴取值，并**逐格记进结果表**——结论依赖它，就必须
看得见。

#### 三个必须处理的坑

1. **a_y 对转角不单调。** 过了附着峰，转角再加 a_y 反而**下降**。所以"解 a_y = 4"在极限附近
   可能有两个解、也可能无解。求解器必须：只在**上升支**内搜索（用现有的
   `grip_util` / 过峰判据判断是否已越峰），若 `target` 超过该工况能达到的最大值，返回
   `unreachable` 并附上实际达到的峰值——**绝不能悄悄收敛到下降支上的那个解**，那会让整张
   对比表看起来正常而实际每个格子比的都不是同一件事。
2. **成本要报出来。** 一次求解 ≈ 8–12 次探针 run。6 律 × 4 速 = 24 次求解 ≈ 240 个额外 run。
   `--dry-run` 必须把这个数算出来告诉调用方，别让 Agent 一句话点着几百个 run 才发现。
3. **必须能缓存复用。** 标定只跟（params 哈希, model, strategy, 工况, target）有关，与被比较的
   量无关。现有研究已经在这么做了（`docs/reports/decoupling_study_data/cal.json`）。缓存键里
   **必须**含 params 哈希，否则改了轮胎参数还在用旧标定，是最隐蔽的一类错。

#### 可解性要声明

不是所有指标都能拿来求解——必须是每 run 一个标量，且在 `bracket` 内单调。指标注册表里因此
要有 `solvable: bool` 与建议 `bracket`；对不可解的指标写 `solve_for` 直接在 `--dry-run` 阶段拒绝。

这套机制不是给 a_y 特化的：解转角以达到目标横摆角速度、解车速以在固定转角下达到目标 a_y，
都是同一根轴。所以按通用的求解轴做，而不是加一个 `equal_ay` 开关。

---

## 5. 能力边界守卫（最重要的一节）

### 问题

`vehicle/kinematic.py` 里明写着：

> ax/ay therefore stay at zero here, consistent with `grip_valid` remaining False — this model
> reports what it can support and nothing more. Use simplified_dynamic or multibody for anything
> force-based.

一个傻包装会让 Agent 问运动学要侧向加速度、拿到 `0.0`、然后一本正经写进报告。同类的还有：
运动学的 `torque_steer` 是"Phase-1 静态占位"（`µ·Fz·scrub·sign(δ)`，不是真力矩链）、没有轮胎
所以没有侧偏角、没有姿态。

### 设计

**能力矩阵由模型自己声明，不写在文档里。** `VehicleModelInfo` 已经有 `id/label/layer/description`，
加一个字段：

```python
@dataclass(frozen=True)
class VehicleModelInfo:
    id: str
    label: str
    layer: ModelLayer
    description: str
    provides: frozenset[Capability]      # 新增
```

`Capability` 是一个受控枚举，指标注册表里每个指标声明自己 `requires` 哪些能力：

| Capability | kinematic | simplified_dynamic | multibody |
|---|---|---|---|
| `pose_kinematics`（轨迹 / ICR 偏差 / 横摆角速度） | ✅ | ✅ | ✅ |
| `body_accel`（a_x / a_y、g-g、摩擦圆利用率） | ❌ | ✅ | ✅ |
| `tyre_slip`（侧偏角、滑移率） | ❌ | ✅ | ✅ |
| `steering_effort`（齿条力、电机力矩需求） | ⚠️ 占位 | ✅ | ✅ |
| `attitude`（侧倾 / 俯仰 / 载荷转移） | ❌ | 部分 | ✅ |
| `drivetrain`（驱动形式、差速器、扭矩模式） | ❌ | ✅ | ✅ |

> 上表是**示意**。实现时必须逐模型对着代码核，并加一个测试：任何模型新增/删除能力，
> 若与它实际写入的 state 字段不符则测试失败。**文档会漂，测试不会。**

### 裁决语义

`run_study` 在**跑之前**做一次静态检查：spec 里每个 metric 的 `requires` ⊆ 声明模型的 `provides`。
不满足时**直接拒绝整份 spec**，返回：

```json
{
  "error": "capability_mismatch",
  "model": "kinematic",
  "unsupported": [
    {"metric": "grip_util_peak", "requires": "body_accel",
     "why": "运动学模型代数解算车身速度，没有自己的加速度；ax/ay 恒为 0 且 grip_valid=False",
     "use_instead": ["simplified_dynamic", "multibody"]}
  ]
}
```

三点都重要：**跑之前**（不浪费几分钟算出废数据）、**整份拒绝**（不部分成功让 Agent 拿一半
结果硬凑）、**带 `use_instead`**（让 Agent 能自己修复而不是放弃或编造）。

`⚠️ 占位` 这一档不拒绝但**必须在结果里带 `degraded` 标记**，且报告模板会把它渲染成显式告警。

---

## 6. 实时仿真的并发与让位

批跑是隔离的；**实时仿真是全局单例**，`POST /api/params|strategy|model|scene|faults` 都在改
人正在用的那台车。既然范围含实时驾驶控制，就必须有明确语义。

### 租约（lease）

```
acquire_realtime(owner="agent:研究后轮权限", ttl_s=300) -> {token, snapshot_id}
    · 快照当前 params / strategy / model / scene / faults
    · 拿到 token 后，所有实时改动调用必须带 token
release_realtime(token)
    · 恢复快照 —— 人回来时不会发现车重变成 2400 kg 还挂着故障注入
```

### 三条铁律

1. **人永远赢，且不需要协商。** 任何来自 GUI 的输入（WS driver 指令、任何 REST 改动）
   **立即吊销**租约。Agent 下一次调用得到 `lease_revoked`，附吊销时刻与原因。不排队、不重试、
   不询问 —— 人不该为了用自己的仿真器去跟 Agent 抢。
2. **TTL 必须有。** Agent 崩了不能把仿真器锁死。到期自动 release + 恢复快照。
3. **Agent 不通过 WS 开车。** 已经有 `ScriptRunner`：声明式、按仿真时间排序、带 ramp 和
   `wait_until`（t / distance / speed_below）谓词。Agent 要驾驶就提交一个 Script，而不是
   以 50 Hz 推 driver 指令 —— 后者既费 token 又不可复现。

### GUI 侧（新增工作）

- 顶栏出现「Agent 正在控制：<owner>」+ 一个**夺回**按钮。
- 租约被吊销时给 Agent 的错误信息里要能说清是谁在什么时候夺回的，这样 Agent 能在报告里
  如实写"该段实验被人工中断"，而不是把半截数据当完整结果。

### 未定 (open)

- 是否允许 Agent 在**持有租约**时改 `model`（切模型会 `_make_model` 重建，人正在开的车会瞬移）？
  倾向：允许但必须先 `reset`，且在 GUI 上明示。

---

## 7. 自定义指标与自定义控制律

### 现状

`user_python` 是 `plugins/strategies/user_strategy.py` 的 mtime 热重载，**没有沙箱**，
`compute()` 直接在后端进程里执行。出错时回落到全零转向并把错误挂到 `/api/user_python/status` ——
可靠性设计是有的，隔离没有。

由于后端只绑 `127.0.0.1`，而人本来就有这个权限，**安全姿态不变；变的是可靠性姿态**：Agent 能
以人做不到的频率写出会把仿真循环卡死的代码。

### 分两档

| 档 | 用途 | 执行面 | 约束 |
|---|---|---|---|
| **表达式指标** | 绝大多数派生指标（`beta_steady_deg = atan2(vy, vx)` 的稳态段均值） | 受限 eval：只暴露 numpy 子集 + 通道数组 | 默认档，无副作用，可安全给 Agent |
| **Python 插件** | 相关法估相位、注入噪声测敏感度、环内夹后轮指令 | 与 `user_python` 同等 | 仅允许在 `SimSession`（批跑，隔离、可超时终止）里用；进实时需显式开关 |

### 环内注入（`precompute` / `in_loop` 钩子）

decoupling study 需要"控制律跑完后夹后轮"。这是 `Experiment` 表达不了的那一类。提议在
`SimSession` 加一个可选钩子点，签名固定：

```python
def in_loop(cmd: ControlCommand, state: VehicleState, params: VehicleParams, t: float) -> None:
    """就地修改 cmd。在 strategy.compute 之后、apply_*_command 之前调用。"""
```

这一个钩子就能覆盖"架构后轮权限""作动器饱和""传感器噪声注入"三类需求，且不引入新概念。
代价是它是 Python 插件档 —— 必须走上表右列的约束。

### 多命名插件

现在 `user_python` 只认一个固定文件名，批跑里没法一个变体一个控制律。提议扩成
`plugins/strategies/<name>.py`，`Experiment.strategy` 可直接命名。这是个小改动，但**会影响
现有 GUI 的「Python 策略」面板语义**，需要一并想清楚。

---

## 8. Token 预算与返回形状

一次 10 s 机动 ≈ 667 采样 × ~35 通道 ≈ 2.3 万个浮点 ≈ **10 万 token**。一次 15 变体的 sweep
就是 150 万 token。这不是"注意一下"的问题，是设计约束。

| 调用 | 默认返回 | 量级 |
|---|---|---|
| `run_study` | spec 摘要 + N 行 × M 指标表 + 判据裁决 + run_ids + 产物路径 | 0.5–2 k token |
| `compare` | 分组聚合 + Δ/Δ% + 拐点/符号翻转标注 | < 1 k |
| `get_trace` | **必须显式给** channels 与 `max_points`（默认 200），返回降采样序列 | 按需 |
| `emit_report` | 只回文件路径 + 摘要 | < 200 |

原则：**Agent 永远不该在上下文里看到完整轨迹。** 需要看波形就出图（文件路径），需要算东西就
定义指标（在服务端算完只回标量）。

---

## 9. 出处与可复现

`save_run` 现在只记 `sim4wis_version`。Agent 研究要可信，run meta 至少还需要：

- `git_sha`（脏树时标 `-dirty`）
- `params_hash`（完全解析后的 `VehicleParams` 的稳定哈希 —— profile + overrides 展开后）
- `model_type` / `strategy` / `dt_sim`
- `study_id` + `spec_digest` + 该 run 对应的 sweep 坐标
- `capability_degraded`（若命中 §5 的 ⚠️ 档）

以及一条纪律，写进报告模板而不是靠自觉：**报告不得出现该次 study 没有测量的结论**。判据
（`criteria`）是唯一被允许升格为"结论"的东西。

---

## 10. 工具面

### MCP（9 个）

| 工具 | 作用 |
|---|---|
| `describe_capabilities` | 模型 × 能力矩阵、策略清单、内置指标及其 `requires`、工况模板。Agent 的第一站 |
| `run_study` | 提交 StudySpec，展开 → 批跑 → 指标 → 聚合。返回汇总表 + 判据裁决 |
| `get_study` | 轮询/取回既往 study 结果 |
| `compare` | 跨 run/跨 study 聚合对比 |
| `get_trace` | 显式降采样取通道 |
| `list_runs` | 检索既往 run（也能看到人在 GUI 里跑的） |
| `emit_report` | 用模板出 HTML，回路径 |
| `verify_against_golden` | 跑 golden 回归，确认这次改动没动到基线 |
| `realtime` | 子命令式：`acquire` / `release` / `drive`（提交 Script）/ `observe` |

`realtime` 刻意收成一个工具，因为它是**危险面**——集中一处便于加确认与审计。

### CLI

同一层的另一门面，命令面一一对应：

```
sim4wis study run    spec.yaml [--dry-run]     # --dry-run 只做边界守卫与展开，不跑
sim4wis study show   <study_id> [--metrics ...]
sim4wis study compare <a> <b> --group-by ...
sim4wis study trace  <run_id> --channels ay,delta.fl --max-points 200
sim4wis study report <study_id> --template sweep -o out.html
sim4wis capabilities [--model multibody]
sim4wis realtime acquire|release|drive|observe
```

`--dry-run` 值得单独指出：它让 Agent 能**零成本验证一份 spec 是否合法**（边界守卫 + sweep
展开 + 判据可解析），再决定要不要花几分钟真跑。

---

## 11. 分期

| 期 | 内容 | 验收 | 粗估 |
|---|---|---|---|
| **P0** | 本设计文档 | 评审通过 | — |
| **P1** ✅ | study 层核心 + `/api/study/*` + CLI：StudySpec、sweep 展开、指标注册表（内置 + 表达式档）、compare、criteria、默认 sweep 报告模板 | 见下方「P1 验收标准的更换」 | 已完成 |
| **P2** | 能力边界守卫 + 出处字段 + `--dry-run` | 模型能力矩阵有测试守着；对运动学问 `grip_util` 被拒绝且给出 `use_instead` | 0.5–1 天 |
| **P2.5** | 求解轴 `solve_for`（§4）：一维求根、上升支约束与 `unreachable`、标定缓存、`--dry-run` 报成本 | 目标 a_y 超过该工况极限时返回 `unreachable` 并给出实际峰值，**不**收敛到下降支；改车辆参数后缓存自动失效 | 0.5–1 天 |
| **P3** | MCP server（独立进程 / stdio，§13）：工具面、后端自动拉起与回收、版本协商 | 在 Claude Code 里端到端跑通一次真实研究；后端未启动时能自己拉起，退出时不误杀用户的后端 | 0.5–1 天 |
| **P3.5** | 随便携版分发（§13）：打包、`print-mcp-config`、说明书一节、发布门禁一条 | 解压一份新包，粘贴生成的配置，Agent 能直接跑通一次 study | 0.5 天 |
| **P4** | 实时租约 + Script 驱动 + GUI 让位提示 | 人在 Agent 持租约时点一下 GUI，租约立即吊销且状态被恢复 | 1 天 |
| **P5** | Python 插件档：自定义指标插件 + `in_loop` 钩子 + 多命名策略插件 | **用 spec 复刻 decoupling study 的六控制律阶梯** | 1–1.5 天 |

估时是粗估，且假设不返工。P1 的验收标准是刻意挑的：能复刻既有研究，这层才算真的够用；
复刻不了就说明抽象错了，越早发现越好。

P1 比初稿多了半天：决策 b 把 `/api/study/*` 从 P3 提到了 P1——MCP server 既然只发 HTTP，
服务端契约就得先有。好处是 CLI 和 MCP 从第一天起就共用同一条路，不会出现"CLI 能做但 MCP
做不到"的分叉。

P2.5 排在 P2 之后不是随意的：求解轴要靠过峰判据判断上升支，而那属于能力边界那一族的东西，
边界守卫先立起来，求解器才有可依据的裁决面。P5 依赖 P2.5——没有等 a_y 标定就复刻不了
decoupling study。

合计 **5–7.5 天**（初稿 4–6.5 天，a 与 c 各加约半天到一天）。

### P1 验收标准的更换

原定验收是「用 spec 复刻后轮转角范围研究的 sweep 部分」。**这在 P1 阶段做不到**，原因在
实现时才浮出来：**后轮转角权限根本不是车辆参数**。`VehicleParams` 只有全局
`steer_limit`（±35°/轮）；`rear_rack_travel_limit` 声明了，但在整个后端物理代码里一次都
没被用到（只出现在序列化与参数校验里）。现有研究因此要在控制律跑完之后手动夹后轮命令
——那是 `in_loop` 钩子，排在 P5。

替代验收（已通过）：**把 study 测出的稳态横摆角速度，与平台自带的闭式稳态解
`solve_steady_state_body` 对表**。这个参照更强，因为它与 study 实际走的时域路径不共享
任何代码，等于用一条独立通路验证了整条链路（展开 → 批跑 → 通道存储 → 表达式档的
`steady()` → 结果表）。线性区内两者吻合到 **0.13–1.10%**。

对表过程本身抓出一个缺陷，见下。

### 该由谁定：后轮权限要不要变成真参数

`in_loop` 钩子（P5）能让研究跑起来，但更根本的问题是：**后轮转向系统的角度上限本来就是
车辆属性**（它是一条作动器规格），现在它在模型里不存在。两条路：

- **(b) 加成真参数**（推荐）：`VehicleParams` 加 `rear_steer_limit`，在命令路径上生效，
  默认值取"不额外限制"因此是 no-op、golden 不受影响。之后后轮权限 sweep 就是一句
  `vehicle.overrides.rear_steer_limit_deg`，不需要钩子；顺带让车辆页那个一直没接线的
  `rear_rack_travel_limit` 有了归宿。
- **(a) 只靠钩子**：不动物理面，但每份研究各夹一遍，且这个"车辆属性"永远进不了 GUI。

---

## 12. 风险与未决

### 风险

1. **Agent 能以人做不到的速度产出看起来很像回事的结论。** 这是本设计最大的风险，
   §5（边界守卫）、`criteria`（可证伪判据）、§9（出处 + 报告纪律）三处都是针对它的。
   但没有任何机制能挡住"跑对了但解释错了"，所以**人对结论的评审不可省**。
2. **`in_loop` 钩子是个后门。** 它让 spec 不再是纯声明式的——一份带钩子的 spec 的行为要看
   Python 文件。缓解：钩子代码随 study 存档并计入 `spec_digest`。
3. **多命名策略插件会改动现有 GUI 语义**，需要和「Python 策略」面板一起想。
4. **实时租约要碰 `Simulator` 单例**，是全项目最热的一段状态。P4 单独成期就是为了不和前面
   的只读工作混在一起。

### 已定

- **a. sweep 支持求解轴 `solve_for`**，进 schema，不走钩子。详见 §4「标定扫描」。理由：等 a_y
  比较是这类研究的**基本手法**而不是个例，交给钩子等于每份研究各写一遍，还各自漏掉非单调性
  这个坑。
- **b. MCP server 走独立进程（stdio）。** 详见 §13。决定性理由是**可逆**：study 层不变，
  A 做完想再挂一个 `/mcp` 到 FastAPI 上很容易，反过来要拆。而且它对已经能跑的东西零风险。
- **c. 便携版 zip 带上 MCP server。** 详见 §13「随便携版分发」。

### 未决

- **d.** 后轮转角权限要不要变成真的 `VehicleParams` 字段？见 §11「该由谁定」。

### P1 实施中发现的缺陷

**解析路径漏了轴侧偏刚度分配。** `dynamic.py` 的时域模型按
`axle_cornering_scale`（默认车 0.80 前 / 1.20 后）缩放轮胎侧偏刚度，而
`quasi_static_wheel_loads` → `load_sensitive_cornering_stiffness` **不缩放**。于是所有走
准静态路径的消费方——`/api/model/demo/bicycle-gain` 的自行车增益演示、负载分析页——
解的是**另一台车**。

效应与转角幅值无关、随 v² 增长（它移动的是不足转向梯度 K，而横摆增益是 V/(L+K·V²)）：
稳态横摆角速度在 30 km/h 差 −5%，60 km/h 差 −17%。把分配补上后误差塌缩到 0.13–1.10%。

这与 decoupling study 记录在 `axle_cornering_stiffness()` 上的是**同一类缺陷的第二处实例**。
已由 `backend/tests/test_study_e2e.py::test_analytic_path_omits_the_axle_cornering_split`
钉住并写明"修好后请删掉这个测试"。修不修、什么时候修，待定。

---

## 13. MCP server 的进程拓扑（决策 b）

```
Claude Code / 其它 Agent 宿主
      │  stdio (JSON-RPC)
      ▼
sim4wis-mcp                     ← 独立进程，薄客户端，无业务逻辑
      │  HTTP 127.0.0.1:8010
      ▼
FastAPI（原样不动）
      ├── /api/*        GUI 与人用
      └── /api/study/*  study 层的 HTTP 面（新增，CLI 也走它）
```

### 这条边界带来的三个要求

1. **study 层必须有 HTTP 面。** MCP server 不 import `sim4wis`，只发 HTTP —— 否则它就得跟后端
   同版本、同解释器，"独立进程"的好处就没了。所以 `/api/study/*` 是 P1 的一部分，不是 P3 的。
   CLI 也走同一条路，于是三个门面（GUI / CLI / MCP）共用一个服务端契约。
2. **后端没起的时候要能自己拉起来。** stdio server 是 Agent 宿主的子进程，用户不会先去开后端。
   启动时探测 `GET /health`，没有就按便携版同样的方式拉起一个，并在自己退出时收掉——但**只收
   自己拉起来的那个**，绝不能杀掉用户正在用的后端。
3. **端口要可配。** `SIM4WIS_BACKEND_HTTP` 已经是前端 dev server 在用的约定，沿用它。
   默认 `http://127.0.0.1:8010`。

### 版本协商

MCP server 与后端可能不同版本（同事更新了包但没更新 MCP server）。启动时拉 `GET /api/version`，
和自己声明的契约版本比对，不匹配就在 `describe_capabilities` 的返回里带一条显式告警——不阻断，
但让 Agent 知道自己可能在用一个对不上的接口。

### 随便携版分发（决策 c）

同事拿到 zip 也能让自己的 Agent 用。便携包本来就内嵌 Python 3.12，所以增量只是几个 Python
文件 + 依赖。要处理的是这几件：

1. **依赖体积。** MCP Python SDK 要进 vendor 集。若体积不可接受，退路是**手写 JSON-RPC over
   stdio**——协议本身不复杂，但要自己维护 schema 序列化与生命周期，风险更高。**先按引 SDK 做，
   量出来再评估**，不要为了省几 MB 提前上手写。
2. **路径是每台机器不同的。** 用户解压到哪都行，配置里得写绝对路径。所以包里要带一个
   `print-mcp-config` 之类的小命令，直接打印一段可粘贴的 `.mcp.json`，路径已经填好。
   让人手抄路径是必错的。
3. **说明书要写。** iCloud 的 `4WIS_Simulator_便携版_说明.txt` 加一节，讲怎么把这段配置贴进
   Claude Code / 其它宿主，以及"Agent 会自己拉起后端，不用你先开"。
4. **发布门禁要覆盖。** `check_release_assets.py` 加一条：便携包内必须存在 MCP server 入口，
   且 `print-mcp-config` 能跑通。否则某次打包漏了没人会发现。

### 配置形态

对本仓库开发是一条 `.mcp.json` 记录；对便携版用户是上面第 2 条生成的那段。
