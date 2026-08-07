# 驾驶体验迭代方案（下一轮 · 提案版本 v0.100.0）

> 面向执行者：本文档假定你没有参与上一轮开发。所有涉及的文件路径、函数名、
> 现有数值都已核对过（基线 commit `710720d`，v0.99.3）。凡是"建议"的地方都给了
> 推荐值和理由，凡是"必须"的地方是会影响正确性或会破坏现有基线的。
>
> 本轮解决三件事：**制动系统缺失**、**转向手感（映射链路缺环）**、**缺少可驾驶的
> 试验场路网**。三者互相独立，可以并行，但第 2 项会动到验证基线，看 §6。

---

## 0. 已确认的决策

来自需求方的明确选择，不要再自行更改：

| 议题 | 决定 |
|---|---|
| 场景层次 | **平路即可，不做高程/坡度/横坡。** 要标准车道线、多车道、多种曲率弯角、足够长的直线。**只做与驾驶相关的部分，纯美化且增加渲染负载的一律不做**（不做天空盒、树木、护栏建模、贴图、动态交通） |
| 制动深度 | **独立制动通道 + 摩擦制动扭矩**（不做 ABS / 不做回收制动混合，本轮） |
| 倒车语义 | **纯刹车 + 独立倒车档**。S / 刹车踏板只减速，到 0 停住；倒车用单独档位 |
| 转向映射 | **真实传动比 + 随速软限幅** |

---

## 1. 现状诊断

### 1.1 制动：系统级缺失，不是参数问题

整条纵向链路目前是：

```
driver.throttle ∈ [-1,1]
      ↓  策略（如 ackermann.py:33）  v_cmd = v_max · throttle
      ↓  compute_commands()          wheel_speed_cmd[i] = v_wheel_i / r
      ↓  WheelSpeedServo (PI+FF)     motor_torque[i]
      ↓  semi_implicit_wheel_spin()  轮速 ODE
```

- `DriverInput`（[core/state.py:215](../backend/src/sim4wis/core/state.py)）只有
  `throttle / steering / handbrake / mode_params`，**没有 brake 通道**。
- 全后端搜索 `brake`，只有注释和变量名，**没有任何制动力学**。
- 脚本层的 `brake` 动作（[input/script.py:190](../backend/src/sim4wis/input/script.py)）字面就是
  `set_driver(throttle=-1.0)`。
- 因此按 S = 给电机负转速指令 = **反向驱动**。表现上"能减速"，但机理错误：
  它不受 μ·Fz 限制、不区分前后轴、不会抱死、在冰面上和干沥青上行为几乎一样、
  减速度上限由 `motor_torque_max` 决定而不是附着。
- `handbrake` 字段全链路（WS → Simulator → state → 前端类型）都在，但**没有任何模型
  读它**，是死字段。本轮把它接成驻车制动，或删掉。

### 1.2 转向：映射链路缺"转向传动比"和"随速调节"两环

`ideal_ackermann`（默认策略）把归一化输入线性映射到**曲率**：

```python
# controller/ideal_ackermann.py
kappa = self._kappa_max * float(driver.steering)
# _kappa_max = 1 / ((L/2)/tan(steer_limit) + tf/2)
```

用 LS9 默认参数（L=3.16, tf=1.565, steer_limit=35°）算出
`κ_max = 0.3291 /m`，即 **R_min = 3.04 m**。这个值是"停车场满舵"的几何极限，
但它**与车速无关**。后果：

| 车速 | 满舵指令 a_y = v²·κ_max | 达到 μ=0.85 附着极限所需 steering | 270° 盘对应转角 |
|---|---|---|---|
| 20 km/h | 10.2 m/s² (1.0 g) | 0.82 | 111° |
| 40 km/h | 40.6 m/s² (4.1 g) | 0.21 | 28° |
| 60 km/h | 91.4 m/s² (9.3 g) | **0.09** | **12°** |
| 80 km/h | 162.5 m/s² (16.6 g) | 0.05 | 7° |

复现脚本见 §7.2。结论：

- 60 km/h 时方向盘只要转 ±12°（270° 盘全行程的 4.5%）轮胎就打饱和，**再打没有任何
  额外响应**，车进入推头。用户感知到的"不够灵敏"实际是"可用行程塌缩到前 5%，
  之后是死区"。
- 同一输入下运动学模型给 315°/s 横摆、动力学只有 29°/s，**差 11 倍**。
  不是动力学模型迟钝，是运动学模型无视附着、掩盖了这个缺环。
- 线性胎在 `α_sat = μ·Fz/c_α = 2.89°` 侧偏角就饱和（c_α=120000 N/rad，Fz≈7112 N），
  饱和之后没有任何回正力矩变化，进一步加重"打了没用"的感觉。

**根因**：输入语义是"目标曲率的百分比"，而真车是"方向盘角 →(传动比)→ 前轮角"。
缺的是 (a) 方向盘总行程与传动比这两个物理量，(b) 随速调节。

### 1.3 场景：只有装饰几何，没有可驾驶路网

`environment/scenario.py` 的数据模型只有三种图元：

```python
Surface(points, color, kind)          # 填充多边形
Line(points, color, width, dash:bool) # 折线（dash 只有开/关，没有节奏）
Marker(type, x, y, heading, meta)     # 目前只有 traffic_light / start_finish
Scenario(..., spawn=(x, y, heading))  # 单一出生点
```

现有 4 个场景：`plaza`（100×100 m 广场 + 10 m 网格线）、`town`（小镇路网）、
`track_small`（0.5 km 环）、`track_shanghai`（1.6 km 近似）。

对照需求的差距：

| 需求 | 现状 |
|---|---|
| 标准车道线 | `_road()` 只画两条边缘线 + 一条虚线中心线，**没有多车道分道线**，虚线节奏不可控 |
| 多车道 | 无。所有道路都是单幅双向 |
| 多种曲率弯角 | `track_small` / `track_shanghai` 是手绘 Chaikin 平滑，**半径不可控、不可标注** |
| 足够长的直线 | 最长直线 < 200 m，不足以做加速/制动 |
| 出生点 | 每个场景只有 1 个，无法直接从长直线或定圆开始 |
| 工况与场景的关系 | `path` 工况永远铺在世界原点沿 +X，**与场景完全无关**，会和道路重叠 |

好消息：3D 渲染侧**几乎不需要改**。路面是 `Surface`、车道线是 ribbon，两者都已支持，
新增的是**数据生成**而不是渲染负载 —— 与"不增加渲染负载"的要求天然一致。

---

## 2. 工作包 A — 制动系统

### A1. 数据模型与协议

**`backend/src/sim4wis/core/state.py`**

```python
@dataclass
class DriverInput:
    throttle: float = 0.0   # 语义收窄为 [0, 1]，只表示驱动
    brake: float = 0.0      # 新增 [0, 1]，摩擦制动踏板行程
    gear: int = 1           # 新增 {-1: R, 0: N, 1: D}
    steering: float = 0.0
    handbrake: int = 0      # 接成驻车制动（见 A4）
    mode_params: dict = ...
```

`VehicleParams` 新增：

```python
brake_torque_max: float = 12000.0   # 整车最大摩擦制动力矩 [N·m]
brake_bias_front: float = 0.65      # 前轴分配比例
brake_tau: float = 0.08             # 液压/EMB 一阶响应 [s]
v_max_reverse: float = 5.0          # 倒车限速 [m/s]
```

`brake_torque_max` 取值依据：整车 1 g 制动需要 `m·g·r = 2900×9.81×0.395 ≈ 11 236 N·m`。
取 12000 保证干沥青上踩满能抱死（这是**要能演示的现象**，不是 bug）。

**`backend/src/sim4wis/api/ws.py`** `_apply_client_message`：`driver` 消息透传
`brake`、`gear`。**`core/simulator.py`** `set_driver()` 同步加参数并做区间钳制。

**向后兼容（必须做）**：`set_driver` 收到 `throttle < 0` 且**未显式给 brake** 时，
转换为 `brake = -throttle, throttle = 0`。理由：`scripts_lib/*.yaml` 和
`experiments/*.yaml` 里的 `brake` 动作就是 `throttle=-1`，这个转换正是它原本的语义。
同时更新 [input/script.py:190](../backend/src/sim4wis/input/script.py) 的 `brake` 动作
直接发 `brake=1.0`，[input/action_schema.py](../backend/src/sim4wis/input/action_schema.py)
的文档字符串跟着改。

### A2. 两层纵向控制（关键设计）

不要只在动力学模型里加制动 —— 运动学模型直接 `s.wheel_omega[:] = cmd.wheel_speed_cmd`
（[vehicle/kinematic.py:76](../backend/src/sim4wis/vehicle/kinematic.py)），没有力的概念，
不处理的话运动学下刹车会**瞬间停车**。分两层：

**第 1 层 · 纵向驾驶员模型（所有模型共用）**
新建 `backend/src/sim4wis/controller/longitudinal.py`：

```python
def speed_command(params, driver, v_actual, dt, state) -> float:
    """(throttle, brake, gear) → v_cmd，带加/减速率限幅。"""
```

- 驱动：`v_target = gear_sign · throttle · (v_max if gear>0 else v_max_reverse)`
- 制动：`v_target` 朝 0 收敛，减速率上限 `a_brake_max = brake · μ_eff · g`
  （μ_eff 取四轮平均，这样冰面上刹不住在运动学模型里也成立）
- N 档：只有滑行阻力
- 输出 `v_cmd` 供各策略替代原来的 `p.v_max * driver.throttle`

**改动面**：所有策略里的 `v_cmd = p.v_max * float(driver.throttle)` 一行替换为调用
第 1 层。涉及 `ackermann.py` / `ideal_ackermann.py` / `rear_wheel_steer.py` /
`crab.py` / `zero_radius.py` / `fault_reconfig.py` / `manual_*.py`（各文件搜
`driver.throttle`）。**建议把它收进 `controller/base.py` 的一个共享 helper**，
避免 8 处重复。

**第 2 层 · 摩擦制动扭矩（仅 dynamic / multibody）**

插入点非常明确：[vehicle/model_core.py:409 `semi_implicit_wheel_spin()`](../backend/src/sim4wis/vehicle/model_core.py)
的 `torque` 参数。目前只有 `t_motor`，改为 `t_motor + t_brake`：

```python
T_i = brake_cmd_filtered * brake_torque_max * bias_i / 2   # 单轮
t_brake_i = -sign(omega_i) * T_i
```
bias：前轮 `bias=brake_bias_front`，后轮 `1-brake_bias_front`。

三个必须处理的细节：

1. **零穿越钳制**。制动扭矩会把 ω 拉过零然后反向抖振。在
   `semi_implicit_wheel_spin` 里加：若 `w_trial * omega_i < 0` 且制动扭矩主导，
   则 `w = 0`，并把该轮当步的净扭矩削到刚好抵消 `r·Fx`（静摩擦锁定）。
   **验收**：整车从 60 km/h 全力制动到 0，末段 ω 无振荡、车速单调递减到 0 并保持 0。

2. **抱死是特性不是 bug**。当 `T_i > r·μ·Fz_i` 时轮子抱死，`κ → -1`，纵向力饱和，
   摩擦椭圆把侧向力吃光 → 抱死时转向失效。这正是需要能演示的现象（用户选了不做
   ABS）。要在 `WheelState` 增加 `locked: bool` 并推到前端 HUD 显示。

3. **伺服与制动打架（必须处理）**。制动让 ω 掉到 ω_cmd 以下，`WheelSpeedServo` 的 PI
   看到正误差就会加驱动扭矩去顶制动。必须在
   [vehicle/wheel_servo.py](../backend/src/sim4wis/vehicle/wheel_servo.py) 或调用处加门控：
   `brake > 0.02` 时伺服输出钳到 `min(t, 0)` 且**冻结积分器**。
   **验收**：制动过程中 `motor_torque` 不出现正值。

### A3. 倒车档

- 前端：`R` 键已被 reset 占用（[KeyboardInput.tsx:84](../frontend/src/components/KeyboardInput.tsx)）。
  **建议档位用 `Q`/`E` 或 `Shift+S`**，reset 保持 `R` 不动（很多文档和 e2e 依赖它）。
  这是个需要拍板的小项，见 §8。
- 手柄/方向盘：`gamepadConfig.ts` 增加一个 `gear` 按钮绑定（R 档拨片）。
- 只有 `|v| < 0.3 m/s` 时允许换档，否则忽略并给 toast 提示。
- HUD 显示当前档位。

### A4. 驻车制动

把死字段 `handbrake` 接成：`handbrake=1` 时后轴施加 `brake_torque_max·(1-bias)` 的
固定制动扭矩。空格键目前是"松开油门"，可改为驻车制动或另绑键（见 §8）。

### A5. 验收标准

| 项 | 判据 |
|---|---|
| 制动不再反推 | 干沥青 60 km/h 踩满刹车，车停住后**不倒车**，`vx` 恒为 0 |
| 附着受限 | 同样输入下，μ=0.85 制动距离约 20 m 量级；μ=0.2（冰面）制动距离应放大约 4 倍 |
| 前后配比 | 65/35 配比下，前轮先于后轮抱死 |
| 抱死转向失效 | 抱死状态下打满方向，横摆角速度显著低于未抱死时 |
| 运动学模型 | 运动学下刹车也是有限减速度，不是瞬间停车 |
| split-μ | 左右不同 μ 时制动产生可观测的横摆力矩 |

### A6. 测试

新建 `backend/tests/test_brake.py`，至少覆盖：
`test_brake_does_not_reverse_the_vehicle`、
`test_brake_distance_scales_with_mu`、
`test_front_axle_locks_first`、
`test_locked_wheel_loses_lateral_force`、
`test_servo_does_not_fight_brake`、
`test_kinematic_brake_is_rate_limited`、
`test_legacy_negative_throttle_maps_to_brake`。

---

## 3. 工作包 B — 转向映射

### B1. 三层映射

新建 `backend/src/sim4wis/controller/steering_feel.py`，所有策略统一调用：

```
第 1 层  方向盘角      θ_sw = steering · (steer_wheel_range / 2)
第 2 层  可变传动比    δ_raw = θ_sw / i(v)          i(v) 在 i_low…i_high 间随速插值
第 3 层  横向加速度软限幅  δ_eff = δ_raw · s(v, μ)   保证估计 a_y ≤ a_y_ref
```

`VehicleParams` 新增：

```python
steer_wheel_range: float = 540.0   # 方向盘锁到锁总转角 [deg]（Dolio R270 用户设 270）
steer_ratio_low: float = 4.0       # 低速传动比
steer_ratio_high: float = 14.0     # 高速传动比
steer_ratio_v_ref: float = 22.0    # 传动比过渡的参考车速 [m/s]（≈80 km/h）
steer_ay_ref_frac: float = 0.9     # 软限幅目标 a_y 相对 μ·g 的比例
```

**为什么必须要可变传动比**：270° 方向盘配真实的 15:1 传动比，满打只有 ±9° 前轮角，
绕桩和掉头根本开不了；而固定 4:1 在 100 km/h 又太贼。真车上的 VGR/AFS 就是这么解决的。
默认 `i_low=4.0` 配 270° 盘 → 满舵 33.75° ≈ `steer_limit`，低速手感与现在**完全一致**；
`i_high=14.0` 时 270° 盘满舵 = 9.6° 前轮角，80 km/h 下 a_y ≈ 4.4 m/s²，方向盘全行程可用。

插值建议用平滑函数而不是线性：`i(v) = i_low + (i_high - i_low) · v²/(v² + v_ref²)`。

**第 3 层软限幅**是保险，防止低 μ 路面上即使传动比合适也一打就失控：
`κ_allow = a_y_ref / max(v, v_min)²`，`a_y_ref = steer_ay_ref_frac · μ_avg · g`，
再把 `δ_eff` 限到该曲率对应的角度。低速时 `κ_allow` 远大于几何极限，不激活。

### B2. 与 4WIS 策略的衔接

`ideal_ackermann` 的语义是"曲率"，不是"前轮角"。转换：
`κ_cmd = tan(δ_eff) / L`（自行车模型），再走原来的 ICR 构造，
**保持 ideal_ackermann 的零侧滑 ICR 语义不变**。

`rear_wheel_steer` / `crab` / `zero_radius` 各自的输入语义不同，逐个确认
（`crab` 的 steering 是蟹行角，可能不该走这条链路 —— 执行时逐策略判断并在
代码注释里写明理由）。

### B3. ⚠️ 会打掉验证基线（重点）

`backend/src/sim4wis/experiment/session.py:137` 直接写 `driver.steering = ...`。
两个 golden 实验 `step_steer_60kmh` 和 `iso3888_dlc_60kmh` 都用
`steer.amplitude = 0.06` 在 60 km/h 跑（当前 κ=0.0197 /m、a_y≈5.5 m/s²，线性区内）。
改了映射，**这两个基线的 KPI 一定会变**，`scripts/check_golden_experiments.py` 会红。

**强烈建议同步做的解耦**（否则以后每次调手感都会打掉验证基线）：

> 把实验/激励的转向幅值从归一化 `[-1,1]` 迁移到**物理单位**——
> `steer.amplitude_deg`（前轮角，度）或 `steer.wheel_angle_deg`（方向盘角，度），
> 让 `session.py` 绕过驾驶员手感层、直接下发 `delta_cmd`。
> 验证实验测的是**车辆**，不该被**驾驶员输入映射**的调参影响。

迁移路径：
1. `experiment/schema.py` 的 steer 定义增加 `unit: "normalized" | "front_deg"`，
   老实验默认 `normalized` 保持不变。
2. 新的 golden 实验用 `front_deg`，把 `amplitude=0.06` 换算成等效前轮角
   （0.06 × κ_max = 0.0197 /m → δ = atan(κ·L) = 3.57°）后写成 `3.57`。
3. 重新 `scripts/promote_reference_benchmark.py` / 更新 `docs/golden_experiments.json`，
   在 `docs/golden_baseline_changelog.md` 记录这次基线迁移的原因和换算关系。

**如果不做这个解耦**，就必须接受每次改手感都要重新 promote 基线，且基线不再有物理含义。

### B4. 前端

- `ParamsPanel` / 车辆页暴露 `steer_wheel_range` / `steer_ratio_low` / `steer_ratio_high`。
- 方向盘用户要能一键设成 270（Dolio R270）。建议在手柄面板加"方向盘总转角"输入，
  写入 `steer_wheel_range`。
- HUD 增加方向盘角 θ_sw 与实际前轮角 δ 的读数，让"传动比在起作用"可见。

### B5. 验收标准

| 项 | 判据 |
|---|---|
| 低速不退化 | 20 km/h 满舵转弯半径仍 ≈ 3.0 m（与 v0.99.3 一致） |
| 高速可用行程 | 60/80 km/h 下，满舵对应的 a_y 落在 `0.9·μ·g ± 15%`，**不再是 9 g / 16 g** |
| 运动学↔动力学差距 | 同输入下两个模型的稳态横摆角速度差距从 11× 收敛到 2× 以内 |
| 物理量可读 | 状态帧里能读到 `steer_wheel_angle_deg` 和 `steer_ratio_current` |
| 低 μ | 冰面上满舵不会给出干沥青同样的曲率指令 |

### B6. 测试

新建 `backend/tests/test_steering_feel.py`：
`test_low_speed_full_lock_matches_geometric_limit`、
`test_high_speed_full_input_stays_within_grip`、
`test_ratio_interpolates_monotonically_with_speed`、
`test_soft_limit_scales_with_mu`、
`test_kinematic_and_dynamic_yaw_gain_converge`。

---

## 4. 工作包 C — 试验场路网

### C1. 数据模型扩展（最小必要）

**`environment/scenario.py`**

```python
@dataclass
class Line:
    points, color, width
    dash: bool = False                    # 保留，向后兼容
    dash_pattern: tuple[float,float] | None = None  # 新增 (实长, 空长) 单位 m

@dataclass
class Scenario:
    ...
    spawns: list[Spawn] = field(default_factory=list)   # 新增：多出生点
    anchors: dict[str, tuple[float,float,float]] = ...  # 新增：工况锚点 (x,y,heading)

@dataclass
class Spawn:
    name: str          # "长直线起点" / "环路起点" / "定圆入口" / "操控区"
    x: float; y: float; heading: float
```

`spawn` 单值字段保留（= `spawns[0]`），避免动 4 个现有场景。

前端跟随：`types/sim.ts` 的 `ScenarioLine` 加 `dash_pattern`；
`canvas2d/ScenarioLayer.tsx` 的 Konva `dash` 用 `dash_pattern × pxm`；
`Canvas3D.tsx` 的 `Scenario3D` 复用上一轮已经写好的
`canvas3d/courseFurniture.ts` 里的 `dashSegments()` —— **那个函数已经支持任意 on/off
长度，直接调用即可，不要重写**。

### C2. 新场景 `proving_ground`（试验场）

一个新的 builder 函数，加进 `_BUILDERS`。布局建议（总占地约 1200 × 800 m，平路）：

```
   ┌──────────────────────────────────────────────────┐
   │  ① 高速直线段  800 m × 3 车道（每 50 m 距离桩）    │
   │     ↓ 接                                          │
   │  ② 变曲率环道：R=150 → 100 → 60 → 40 → 25 m       │
   │     每个弯有入弯标识桩 + 半径标注锚点              │
   │  ③ 定圆场地 R=30 m（内外圈车道线）                 │
   │  ④ 低速操控区 200 × 60 m 空旷铺装（绕桩/移线用）   │
   └──────────────────────────────────────────────────┘
```

关键要求：

- **多车道**：主路 3 车道，单车道宽 **3.75 m**（国标高速）或 3.5 m（城市）。
  用参数控制，默认 3.75。
- **车道线（按 GB 5768.3）**：
  - 路缘边缘线：白色实线，宽 0.20 m
  - 车道分界线：白色虚线，高速节奏 **实 6 m / 空 9 m**，宽 0.15 m
  - 对向分隔：黄色双实线（两条 0.15 m 线，间距 0.15 m）
  - 减速提示线（可选）：进弯前的横向白色标线组
- **弯道半径必须是精确的圆弧**，不能再用 Chaikin 平滑手绘点。
  写一个 `_arc_road(center, radius, th0, th1, n_lanes, lane_width)` 生成器，
  半径是显式参数，这样"不同曲率"是可标注、可复现的。
- **距离桩**：直线段每 50 m 放一对 `Marker(type="distance", meta={"m": 350})`，
  3D 里画一块简单立牌（两个三角形 + 一个色块，零纹理），2D 里画一个刻度。
  这是驾驶相关信息，不是装饰。

**渲染负载约束（需求方明确要求）**：
- 不做天空盒 / 太阳 / 体积雾 / 树木 / 护栏 3D 模型 / 任何贴图
- 路面继续用扁平 `Surface`，车道线继续用 ribbon
- 新增几何全部是三角形数量可数的图元；`Marker` 立牌用 InstancedMesh 合批
- **验收**：`proving_ground` 场景下 3D drawcall 数不超过 `town` 场景的 1.5 倍，
  在 v0.99.2 修复过的软件渲染路径（`scripts/render_verify/launch.sh software`）
  下仍能跑到可用帧率

### C3. 工况铺到场景上（"路线"需求的核心）

现在 `path` 工况永远铺在原点沿 +X，和场景道路重叠。改为：

- `Scenario.anchors` 提供命名锚点，例如
  `{"straight_start": (x,y,0), "skidpad": (...), "handling": (...)}`
- `POST /api/path/template` 的 body 增加可选 `anchor: str`
- `plan_from_template` 生成后，把 `points / cones / marks` 整体做刚体变换到锚点位姿
  （加一个 `PathPlan.transform(x, y, heading)` 方法，注意 cones 和 marks 都要转）
- `TrajectoryPanel` 在有场景且场景有锚点时，显示一个锚点下拉框；
  各工况给默认锚点（绕桩/移线 → `handling`，定圆 → `skidpad`，直线 → `straight_start`）

### C4. 出生点选择

`ScenarioPanel` 在场景有多个 `spawns` 时显示按钮组，点击调用现有的
`POST /api/scenarios/{name}/load`（扩展一个 `?spawn=<name>` 参数），
复用已有的 `_spawn()` 传送逻辑（[api/routers/scenario.py](../backend/src/sim4wis/api/routers/scenario.py)）。

### C5. 验收标准

| 项 | 判据 |
|---|---|
| 车道几何 | 3 车道、车道宽误差 < 1 cm；虚线节奏实测 6/9 m |
| 弯道半径 | 每个弯的中线半径与标称值误差 < 0.5% |
| 直线长度 | 主直线 ≥ 800 m，可完成 0–100 km/h 加速 + 全力制动且不出界 |
| 出生点 | 至少 4 个，切换后车辆位姿正确、朝向沿车道 |
| 工况对齐 | 绕桩工况铺到 `handling` 锚点后，全部锥桶落在铺装面内 |
| 渲染 | 见 C2 的负载约束 |

### C6. 测试

`backend/tests/test_proving_ground.py`：
`test_lane_widths_and_count`、`test_corner_radii_match_nominal`、
`test_straight_length`、`test_all_spawns_are_on_pavement`、
`test_anchors_exist_and_courses_fit`、`test_dash_pattern_serialized`。

前端 e2e 增加：切换到 `proving_ground` → 选出生点 → 生成绕桩到 `handling` 锚点 →
断言 `当前路径` 文案与锥桶数。

---

## 5. 工作包 D — 撞桩与出界判定（可选，建议做最小版）

需求方没有明确要求，但"方便我进行测试和仿真驾驶"隐含需要。建议**只做最小版**：

- 车身多边形（`vehicleShape.bodyOutline` 已有，后端需要一份等价实现）与锥桶圆的碰撞
- 命中计数 + HUD 提示 + 该锥桶变色（不做物理撞飞）
- ISO 3888 通过/失败判定：撞桩数 = 0 且全程在段框内
- 在 `ScorePanel` 里显示

**不建议本轮做**：车辆与建筑/护栏的碰撞响应（需要完整碰撞求解，且和"只做驾驶相关"
的约束冲突）。

---

## 6. 执行顺序与依赖

```
A（制动）   ─┬─ 可与 B 并行，互不冲突
B（转向）   ─┘   但 B 会打掉 golden 基线，必须先做 §3.B3 的解耦
C（场景）   ─── 完全独立，可最先做（风险最低、见效最快）
D（判罚）   ─── 依赖 C 完成
```

建议节奏：

1. **先做 C**（独立、无基线风险、需求方最有感知）
2. **再做 A**（独立，有清晰验收）
3. **最后做 B**，且**必须先完成 B3 的实验单位解耦**，再改映射，再重新 promote 基线
4. D 视时间

每个工作包独立走一次 `scripts/pre_release_check.py`，不要攒到最后。

---

## 7. 附录

### 7.1 关键文件索引

| 关注点 | 文件 |
|---|---|
| 驾驶员输入定义 | `backend/src/sim4wis/core/state.py` (`DriverInput`, `VehicleParams`) |
| WS 协议 | `backend/src/sim4wis/api/ws.py` (`_apply_client_message`) |
| 输入落地 | `backend/src/sim4wis/core/simulator.py` (`set_driver`) |
| 策略共享逻辑 | `backend/src/sim4wis/controller/base.py` (`compute_commands`) |
| 曲率映射 | `backend/src/sim4wis/controller/ideal_ackermann.py` |
| 轮速伺服 | `backend/src/sim4wis/vehicle/wheel_servo.py` |
| **制动扭矩插入点** | `backend/src/sim4wis/vehicle/model_core.py` (`semi_implicit_wheel_spin`) |
| 动力学积分 | `backend/src/sim4wis/vehicle/dynamic.py` (`step`, `_derivatives`) |
| 运动学模型 | `backend/src/sim4wis/vehicle/kinematic.py` |
| 场景数据 | `backend/src/sim4wis/environment/scenario.py` |
| 工况生成 | `backend/src/sim4wis/controller/path.py` |
| 实验转向输入 | `backend/src/sim4wis/experiment/session.py:137` |
| 前端输入循环 | `frontend/src/components/KeyboardInput.tsx` |
| 手柄映射 | `frontend/src/input/gamepadConfig.ts` |
| 3D 视口 | `frontend/src/components/Canvas3D.tsx`、`canvas3d/` |
| 虚线切分（可复用） | `frontend/src/components/canvas3d/courseFurniture.ts` (`dashSegments`) |

### 7.2 转向诊断复现脚本

```python
# cd backend && python3 - <<'PY'
import sys; sys.path.insert(0,'src')
import numpy as np
from sim4wis.core.state import VehicleParams
p = VehicleParams()
kmax = 1.0/((p.wheelbase/2)/np.tan(p.steer_limit) + p.track_front/2)
for kmh in (20,40,60,80):
    v = kmh/3.6
    print(f"{kmh:3d} km/h  a_y_cmd={v*v*kmax:7.1f} m/s^2  "
          f"到极限只需 steering={min(8.34/(v*v*kmax),1.0):.3f}")
PY
```

### 7.3 已知会变红的检查

| 检查 | 原因 | 处理 |
|---|---|---|
| `check_golden_experiments.py` | B 改了转向映射 | 先做 B3 解耦，再重新 promote 并记录到 `docs/golden_baseline_changelog.md` |
| `test_v1_readiness.py` | browser smoke 用例数变化 | `check_v1_readiness.py --report docs/reports/v1_readiness.md` 重生成 |
| `check_release_assets.py` | 版本号变化后 `docs/user_manual.html` 过期 | `python3 scripts/build_manual.py`（需要先 `npm run build`；只有 conda 的 python3 装了 playwright+PIL） |
| `independent_reference` | **本来就是 GAP**，需要外部台架/实车数据 | 与本轮无关，不要试图"修好" |

### 7.4 环境坑（上一轮踩过）

- `sim4wis` 是 editable 安装、指向主仓库。**在 git worktree 里跑测试或起后端，必须
  `PYTHONPATH=<worktree>/backend/src`**，否则加载的是主仓库旧代码。
- e2e 占用 8010 端口；`build_manual.py` 占用 8016。
- Vite 在 macOS 上只监听 IPv6 回环，开发地址用 `localhost:5173` 不要用 `127.0.0.1`。

---

## 8. 待拍板（执行前需要需求方确认）

1. **档位与手刹的按键**。`R` 已经是 reset（e2e 和文档都依赖），倒车档建议绑 `Q`/`E`
   或 `Shift+S`；空格现在是"松开油门"，是否改为驻车制动？
2. **方向盘总转角默认值**。默认 540°（常见 PC 盘）还是 270°（你的 Dolio R270）？
   建议默认 540，在手柄面板里一键切 270。
3. **实验转向单位解耦**（§3.B3）做不做。不做的话每次调手感都要重新 promote 基线。
4. **工作包 D（撞桩计分）** 本轮做不做。
5. **场景是新增还是替换**。建议新增 `proving_ground`，`plaza` 保留（很多截图和
   e2e 依赖它）。
6. **版本号**：建议 `v0.100.0`（本轮是功能轮，不是 1.0）。
