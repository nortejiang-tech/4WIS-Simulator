# Phase 2 / Phase 3 实施计划

> 写于 2026-05-23，Phase 1 完成之后。
>
> 本文档目的：把后续开发拆成可独立交付的小步，每一步有明确的范围、依赖、验证标准。
> 任何人（包括未来的我）打开这个文件，应该能 *不再做决策* 直接按步推进。

---

## 1. 设计哲学（不要忘记的几条）

1. **Phase 1 的契约不破坏**。已经定型的：
   - `VehicleModel.step(dt, cmd, env) -> VehicleState` 接口
   - `ControllerStrategy.compute(driver, state) -> ControlCommand` 接口
   - 坐标系（X 前 / Y 左 / Z 上、车体原点 = 前后轴中点、车轮编号 FL/FR/RL/RR）
   - WebSocket 协议字段名
   - YAML 项目文件 6-section 结构
   见 [project-4wis-phase1-done](memory 中的同名条目).

2. **算法同事用 Simulink**。FMU/MATLAB Engine 接入是 Phase 2 的明星功能之一，比 3D 优先级高。

3. **运动学模型留着**，不要因为有了动力学就删。它跑得快、几何最干净，适合策略验证；动力学模型用于扰动响应、能耗、暂态分析。**模型类型作为项目级配置项切换**。

4. **配置先行**。任何新增的物理量、扰动、控制策略，都要先写 schema（pydantic）+ YAML 字段，再写实现。这样老姜按项目保存参数的需求始终满足。

5. **加 SuspensionParams.cg_height** 这种字段时，更新所有 3 个示例 YAML，并加到冒烟测试里检查。

---

## 2. 优先级与拆分

按"独立可交付 + 依赖最少 + 用户价值最高"排序：

| 步号 | 名称 | 阶段 | 依赖 | 用户能立刻看到的变化 |
|------|------|------|------|---------------------|
| 7 | Recording + CSV 导出 | 2a | 无 | 录制/导出按钮，离线分析能力 |
| 8 | 路面扰动（μ 类）+ 可视化 | 2a | 无 | 画布上能放冰面/对开路段 |
| 9 | YAML 动作序列脚本 | 2a | 无 | 自动跑标准工况（slalom 等） |
| 10 | SimplifiedDynamicModel | 2b | 无（独立基类） | 项目里能选"动力学模型" |
| 11 | 悬架几何 → 真实阻力矩 | 2b | 10 | 阻力矩曲线变成有意义的数据 |
| 12 | SpeedBump / Slope 扰动 | 2b | 10 | 减速带、斜坡的瞬态响应 |
| 13 | FMU 适配器 + 插件加载 | 2b | 无（但需 fmpy） | 算法同事的 Simulink 模型能跑 |
| 14 | MATLAB Engine 适配器 | 2b | 13 的插件机制 | 不导出 FMU 也能调试 |
| 15 | Phase 2 集成测试 + 文档 | 2b | 7-14 | smoke_test 增项 |
| 16 | 3D 可视化（Three.js） | 3 | 10、12 | 沉浸式 3D 视图 |
| 17 | 轨迹编辑器 + 标准路径模块 | 3 | 无 | 拖出标准工况 |
| 18 | USB 外设（WebHID） | 3 | 无 | 真方向盘 / 手柄输入 |
| 19 | 完整多体动力学（可选） | 3+ | 10 | 14+ DOF |

**Phase 2a = 步 7-9**：基础能力扩展，不动模型核心，风险低。
**Phase 2b = 步 10-15**：动力学和插件，需要扩展抽象。
**Phase 3 = 步 16-19**：可视化和外设。

---

## 3. Phase 2a 详细设计

### 步 7：Recording + CSV 导出

**目标**：仿真器内部环形缓冲所有 60 Hz 推送的状态；前端按钮开始/停止录制；通过 REST 下载 CSV。

**新增文件**：
```
backend/src/sim4wis/recorder/
├── buffer.py       Ring buffer + per-channel selection
├── csv_export.py   CSV 流式写入
└── replay.py       从 CSV 反向回放（可选，Phase 2a 不一定做完）
```

**修改文件**：
- `backend/src/sim4wis/core/simulator.py`：每次广播状态时同时写入 recorder
- `backend/src/sim4wis/api/rest.py`：增加 `/api/recording/start`、`/stop`、`/status`、`/export`
- `backend/src/sim4wis/project/schema.py`：`RecordingSection` 已存在，加 `buffer_seconds: float = 1800` 等字段
- `frontend/src/components/RecordingPanel.tsx`：新组件
- `frontend/src/api/ws.ts`：增加 `startRecording()`、`stopRecording()`、`exportCSV(channels)` 函数

**Recorder 数据结构**：
```python
@dataclass
class RecorderConfig:
    buffer_seconds: float = 1800.0   # 30 min default
    channels: list[str] = ...         # which fields to record
    rate_hz: float = 60.0             # write rate (≤ sim push rate)

class Recorder:
    def __init__(self, cfg: RecorderConfig): ...
    def write(self, state: VehicleState, cmd: ControlCommand) -> None: ...
    def is_recording(self) -> bool: ...
    def start(self) -> None: ...      # clear buffer + arm
    def stop(self) -> None: ...
    def to_csv(self, channels: list[str], from_t: float | None, to_t: float | None) -> Iterator[str]: ...
```

**Channels（字段名约定）**：扁平化、Snake_case、带单位后缀
```
t                          时间 (s)
pose_x, pose_y, pose_psi   位姿
vx, vy, yaw_rate           速度
delta_fl, delta_fr, delta_rl, delta_rr        实际转角 (rad)
deltacmd_fl, ..., deltacmd_rr                 指令转角
omega_fl, ..., omega_rr                       车轮转速 (rad/s)
fz_fl, ..., fz_rr                             垂直载荷 (N)
torque_steer_fl, ..., torque_steer_rr         转向阻力矩 (N·m)
icr_vehicle_body_x, icr_vehicle_body_y        实际 ICR 车体坐标
icr_target_body_x, icr_target_body_y          目标 ICR
strategy                                      字符串
```

**REST 端点**：
```
POST /api/recording/start         { channels?: string[] }
POST /api/recording/stop
GET  /api/recording/status        → { recording, buffer_s, samples, from_t, to_t }
GET  /api/recording/export.csv?from_t=&to_t=&channels=a,b,c
                                  → StreamingResponse text/csv
```

**前端 UI**：
- 大圆按钮 ● 录制 / ■ 停止
- 录制时间显示
- "导出 CSV" 下拉里选择 channels（多选）

**验证**：
1. 录制 30 秒 → 文件行数 ≈ 30 × 60 = 1800 ± 1
2. CSV 重新载入后能复现整车轨迹（用 pandas + matplotlib 画一遍）
3. 浏览器下载是 `.csv` MIME，文件名含时间戳

---

### 步 8：路面扰动（μ 类）

**目标**：用户能在画布上放一块 *冰面* 或 *对开路面* 区域；车辆驶入时左右两侧的 μ 不同。

**新增文件**：
```
backend/src/sim4wis/environment/
├── disturbance.py      Disturbance ABC + types
└── road.py             Scene 容器（disturbance 列表）

frontend/src/components/
├── DisturbancePanel.tsx   增删扰动
└── DisturbanceCanvas.tsx  画布上渲染扰动区域
```

**抽象**：
```python
class Disturbance(ABC):
    """Each disturbance is a region in world frame + a per-wheel modifier."""
    id: str
    @abstractmethod
    def query(self, wheel_world_pos: np.ndarray) -> WheelEnvLocal:
        """Return local env at this wheel — (mu_modifier, fz_offset, ground_z)."""

@dataclass
class WheelEnvLocal:
    mu_modifier: float = 1.0   # multiplied into base mu
    fz_offset: float = 0.0     # additive vertical force perturbation
    ground_z: float = 0.0      # for future SpeedBump / Slope
```

**具体类型（Phase 2a）**：
- `RectanglePatch(x, y, width, height, mu_modifier)`：矩形 μ 修正区域
- `SplitMu(x, y, width, length, mu_left, mu_right, heading=0)`：对开路面

**EnvironmentState 扩展**：
```python
@dataclass
class EnvironmentState:
    mu: float = 1.0
    surface_z: float = 0.0
    disturbances: list[Disturbance] = field(default_factory=list)
    
    def wheel_env(self, wheel_world_pos: np.ndarray) -> WheelEnvLocal:
        """Aggregate all disturbance effects at this wheel position."""
        local = WheelEnvLocal()
        for d in self.disturbances:
            l = d.query(wheel_world_pos)
            local.mu_modifier *= l.mu_modifier
            local.fz_offset += l.fz_offset
            local.ground_z += l.ground_z
        return local
```

**Project schema 扩展**：
```yaml
scene:
  surface: flat
  base_mu: 1.0
  disturbances:
    - type: split_mu
      x: 30.0
      y: 0.0
      width: 4.0
      length: 20.0
      mu_left: 0.3
      mu_right: 0.9
      heading: 0.0
    - type: ice_patch
      x: 10.0
      y: -2.0
      width: 5.0
      length: 5.0
      mu_modifier: 0.2
```

**前端可视化**：
- 在 Canvas2D 的 Grid 层之后、Trajectory 之前画扰动区域
- SplitMu 用半透明蓝（左）+ 半透明红（右）
- 冰面用淡蓝色矩形
- Drag-to-edit 可以 Phase 2.5 加，Phase 2a 只支持 YAML 配置

**WebSocket 协议**：状态消息里增加 `scene_snapshot` 字段（首次连接时推送一次完整快照，之后只在扰动变更时推送）

**验证**：
1. 加载含 SplitMu 的项目 → 画布上能看到对开区域
2. 驾驶车辆穿过对开区域 → 左右轮 μ 在曲线里有阶跃变化（导出 CSV 验证）
3. 仿真步性能不退化（扰动查询 < 0.1 ms / step）

---

### 步 9：YAML 动作序列脚本

**目标**：定义一种声明式 DSL，按时间表执行驾驶动作。算法验证、回放标准工况用。

**为什么选 YAML 而不是 Python**：
- 安全（不执行任意代码）
- 直接 round-trip 到项目文件
- 算法同事的复杂逻辑应该写在 Simulink 里通过 FMU 接入（步 13）

**Schema**：
```yaml
script:
  name: "double_lane_change"
  loop: false
  actions:
    - { t: 0.0,  action: set_strategy, name: ideal_ackermann }
    - { t: 0.0,  action: drive,        throttle: 0.6, steering: 0.0 }
    - { t: 3.0,  action: steer_ramp,   from: 0.0, to: +0.5, duration: 0.4 }
    - { t: 3.4,  action: steer_ramp,   from: +0.5, to: -0.5, duration: 0.8 }
    - { t: 4.2,  action: steer_ramp,   from: -0.5, to: 0.0, duration: 0.4 }
    - { t: 6.0,  action: brake,        deceleration: 3.0 }
    - { t: 8.0,  action: stop }
```

**动作类型（初版）**：
- `drive(throttle, steering)`
- `set_strategy(name)`
- `set_mode_params(...)`
- `steer_ramp(from, to, duration)`
- `throttle_ramp(from, to, duration)`
- `brake(deceleration)`
- `wait_until(distance | speed | t)`
- `reset()`
- `stop`

**新增文件**：
```
backend/src/sim4wis/input/
├── action_schema.py    pydantic schemas for each action type (union)
└── script.py           ScriptRunner: async coroutine that drives sim per action
```

**ScriptRunner**：
```python
class ScriptRunner:
    def __init__(self, sim: Simulator, script: Script): ...
    async def run(self) -> None:
        """Iterate actions in t-order, sleep to next action's t, execute."""
    def stop(self) -> None: ...
```

**REST 端点**：
```
POST /api/script/load     { yaml: "..." }    解析 + 校验
POST /api/script/start
POST /api/script/stop
GET  /api/script/status   → { running, current_action_idx, t_in_script }
```

**前端**：`ScriptPanel.tsx` —— textarea + load/start/stop 按钮 + 进度条。可以打包几个常用工况作为下拉模板：
- `slalom_3m.yaml`
- `double_lane_change.yaml`
- `j_turn.yaml`
- `crab_park.yaml`
- `spin_in_place.yaml`

放到 `<repo>/scripts_lib/`。

**验证**：
1. 跑一遍 DLC 工况，画出轨迹 → 应当是标准的"S"形（截图对比 ISO 3888-2 形状）
2. 仿真器接管期间，键盘输入被忽略；脚本结束后自动恢复键盘控制
3. 暂停/恢复正常

---

## 4. Phase 2b 详细设计（动力学 + 插件）

### 步 10：SimplifiedDynamicModel

**目标**：替代 KinematicModel 的可选物理模型。包含线性轮胎、垂直载荷转移、车轮转动惯量。

**物理状态**：在 VehicleState 已有字段基础上工作。每步集成：
- `body_dot = [ax_body, ay_body, ω̇]` 由所有 4 个轮胎力 + 重力分量决定
- `wheel_omega_dot[i] = (T_motor[i] + T_brake[i] - r·Fx_tire[i]) / Iw`
- `pose 积分`：standard

**新增文件**：
```
backend/src/sim4wis/vehicle/
├── tire.py             TireModel ABC + LinearTireModel
├── load_transfer.py    LongitudinalLoadTransfer + LateralLoadTransfer
├── dynamic.py          SimplifiedDynamicModel
└── wheel_servo.py      内部 PI 速度环（将 ω_wheel_cmd → T_motor）
```

**TireModel**：
```python
class TireModel(ABC):
    @abstractmethod
    def forces(
        self,
        alpha: float,        # slip angle [rad]
        kappa: float,        # longitudinal slip [-]
        fz: float,           # vertical load [N]
        mu: float,           # surface mu
    ) -> tuple[float, float, float]:
        """Return (Fx, Fy, Mz_self_aligning) in tire-aligned frame."""

@dataclass
class LinearTireModel(TireModel):
    c_alpha: float = 70_000   # cornering stiffness [N/rad]
    c_kappa: float = 100_000  # longitudinal stiffness [N]
    t_pneumatic: float = 0.03 # pneumatic trail [m]
    # forces = linear, saturated at μFz friction ellipse
```

**Slip 计算**：
- `vx_wheel` = velocity at wheel position in tire-aligned frame (along rolling direction)
- `vy_wheel` = velocity at wheel position in tire-aligned frame (lateral)
- `alpha = atan2(vy_wheel, vx_wheel)` (slip angle)
- `kappa = (r·ω_wheel - vx_wheel) / max(|vx_wheel|, vmin)` (longitudinal slip)
  - vmin ≈ 0.5 m/s for numerical stability

**载荷转移**：
```python
def vertical_loads(params, ax, ay) -> np.ndarray:
    """Return Fz per wheel given longitudinal/lateral accelerations."""
    m, g, h, L = params.mass, 9.81, params.cg_height, params.wheelbase
    tF, tR = params.track_front, params.track_rear
    a, b = params.cg_to_front, L - params.cg_to_front
    
    # Static
    Fz_f_static = m * g * b / L / 2
    Fz_r_static = m * g * a / L / 2
    
    # Longitudinal transfer
    dFz_long = m * ax * h / L / 2
    
    # Lateral transfer
    dFz_lat_f = m * ay * h * (b/L) / tF
    dFz_lat_r = m * ay * h * (a/L) / tR
    
    return np.array([
        Fz_f_static - dFz_long - dFz_lat_f,   # FL
        Fz_f_static - dFz_long + dFz_lat_f,   # FR
        Fz_r_static + dFz_long - dFz_lat_r,   # RL
        Fz_r_static + dFz_long + dFz_lat_r,   # RR
    ])
```

**积分**：RK4 fixed-step at `dt_sim = 5 ms`. 状态向量 13 维（pose 3 + body vel 3 + wheel ω × 4 + actuator δ × 4，如果带转向滞后）。Phase 2 内先不带转向作动器滞后（δ_actual = δ_cmd），Phase 2.5 加。

**Wheel speed servo**（在动力学模型内部）：
```python
class WheelSpeedPI:
    """Per-wheel PI controller: ω_cmd → motor torque."""
    def __init__(self, kp=200, ki=50, t_max=2000): ...
    def update(self, omega_actual, omega_cmd, dt) -> torque
```

**VehicleParams 新字段**：
```python
cg_height: float = 0.55         # CG 离地高度 [m]
wheel_inertia: float = 1.2      # 单轮转动惯量 [kg·m²]
motor_torque_max: float = 2000  # 单电机最大扭矩 [N·m]
tire: LinearTireModel = ...     # 整车统一胎，Phase 3 支持 per-axle
brake_torque_max: float = 4000  # 制动器最大扭矩 [N·m]
```

**Project schema 扩展**：
```yaml
vehicle:
  model: simplified_dynamic     # 或 kinematic（默认）
  # ... existing fields ...
  cg_height: 0.55
  wheel_inertia: 1.2
  motor_torque_max: 2000
  brake_torque_max: 4000
  tire:
    type: linear
    c_alpha: 70000
    c_kappa: 100000
    t_pneumatic: 0.03
```

**Simulator 修改**：
```python
def _make_model(self, params, model_type):
    if model_type == "kinematic":
        return KinematicModel(params)
    elif model_type == "simplified_dynamic":
        return SimplifiedDynamicModel(params)
    raise ValueError(model_type)
```

**Strategy 兼容性**：所有 Phase 1 策略都继续可用。它们输出 `wheel_speed_cmd[4]`，动力学模型用 wheel servo 跟踪。

**验证**：
1. 稳态圆周（throttle=0.3, steering=0.3, ideal_ackermann, 10s）：轨迹半径应当接近运动学解 ± 几个百分点
2. 阶跃响应（steering 从 0 跳到 0.5）：横摆角速度有一阶滞后曲线（dynamic 而非 kinematic 的瞬时跟随）
3. 高 μ 下能加速到 v_max；低 μ（0.3）下打滑、car spin
4. CSV 录制能看到 slip_angle 历程

**性能目标**：dt_sim=5ms 单步 < 0.5 ms（够 200 Hz 实时）

---

### 步 11：悬架几何 → 真实阻力矩

**目标**：当前 `torque_steer` 是占位；用真实主销几何 + 轮胎力计算。

**修改**：`SimplifiedDynamicModel.step` 末尾：
```python
def kingpin_torque(Fx, Fy, Mz, suspension, fz):
    """Steering torque around kingpin axis."""
    scrub = suspension.scrub_radius
    caster = suspension.caster_angle
    kpi = suspension.kingpin_inclination
    
    # Lateral force at contact patch → kingpin moment
    M_from_Fy = Fy * (scrub + ... caster effect on lateral arm)
    
    # Longitudinal force → kingpin moment (mostly via scrub_radius)
    M_from_Fx = Fx * scrub
    
    # Tire self-aligning torque
    M_from_Mz = Mz
    
    # Vertical-load-induced moment (KPI causes wheel to want to return to center)
    M_from_Fz_kpi = fz * kpi_steering_offset
    
    return M_from_Fy + M_from_Fx + M_from_Mz + M_from_Fz_kpi
```

引用：Reimpell 《The Automotive Chassis》§3.10；Pacejka 《Tire and Vehicle Dynamics》§9。

**验证**：
1. 直行时 τ_steer ≈ 0（除 KPI 居中力外）
2. 转弯时 τ_steer 与 v² 大致成正比
3. 低速急转 vs 高速缓转：低速急转 τ 显著大于高速缓转（这是真实车感）

---

### 步 12：SpeedBump / Slope 扰动

**SpeedBump**：在某条 X 段上，车轮经过时 Fz 有一个 ms 量级的脉冲（先压缩、后释放）。
- 简化模型：当 wheel 进入 bump 区域，给 Fz += k_bump · profile(x_wheel - x_bump_center)，其中 profile 是 raised cosine
- 持续时间正比于 bump 宽度 / 车速
- 影响：瞬态轮胎打滑、车身姿态扰动（PhaseE 2 不模拟 pitch）

**Slope**：在某矩形区域，重力的一部分分量作用为纵向力 -mg·sin(α)
- Force on body：在斜坡区，body frame Fx 加 -mg·sin(α)·cos(ψ - slope_heading)
- 每个轮的 Fz 也随之变化

**Disturbance 子类**：
- `SpeedBump(x, y, width, length, height, heading)`：position, dimensions, peak height [m]
- `Slope(x, y, width, length, angle_rad, heading)`：grade angle

**验证**：
1. 设置 5cm 高减速带，10 m/s 通过：CSV 中 Fz 有瞬态尖峰；车速略降
2. 设置 10% 斜坡上坡：稳态速度下降；轮胎力分布前后变化

---

### 步 13：FMU 适配器 + 插件加载

**目标**：算法同事在 Simulink 中开发控制策略，导出 FMU；放进插件目录就能作为可选策略。

**新增文件**：
```
backend/src/sim4wis/controller/plugins/
├── __init__.py
├── loader.py              扫描 <repo>/plugins/strategies/*.fmu
├── fmu_adapter.py         FMUControllerStrategy
└── matlab_adapter.py      MatlabEngineControllerStrategy (步 14)

plugins/
└── strategies/
    └── README.md          告诉用户怎么放 FMU
```

**依赖**：`fmpy >= 0.3`（pyproject.toml 加 optional `[fmu]` extra）

**Sidecar 配置文件**（`<fmu_name>.fmu.yaml`）：
```yaml
name: my_mpc_strategy
description: "MPC + 理想阿克曼前馈"
inputs:
  # FMU input variable name → expression evaluated from sim state
  vx:        "state.vx"
  vy:        "state.vy"
  yaw_rate:  "state.yaw_rate"
  throttle:  "driver.throttle"
  steering:  "driver.steering"
outputs:
  # ControlCommand field ← FMU output variable name
  delta_cmd[0]:       "delta_fl"
  delta_cmd[1]:       "delta_fr"
  delta_cmd[2]:       "delta_rl"
  delta_cmd[3]:       "delta_rr"
  wheel_speed_cmd[0]: "omega_fl"
  ...
step_size_ms: 10
```

**FMUControllerStrategy**：
```python
class FMUControllerStrategy(ControllerStrategy):
    def __init__(self, params, fmu_path, mapping):
        super().__init__(params)
        from fmpy import read_model_description, extract
        # ... initialize FMU instance ...
    def compute(self, driver, state) -> ControlCommand:
        # Evaluate mapping.inputs → set FMU vars
        # do_step(self.dt_strategy)
        # Read mapping.outputs → ControlCommand
```

**加载流程**：
1. 启动时扫描 `<repo>/plugins/strategies/*.fmu`
2. 每个 FMU 必须有同名 `.fmu.yaml`
3. 校验 + 注册到 strategy registry（名字 = sidecar.name）
4. 失败的 plugin 记录 warning，不阻塞启动

**REST 端点**：
```
GET  /api/plugins                   → { strategies: [...], loaded: int, failed: [{path, error}] }
POST /api/plugins/reload            重新扫描
```

**Phase 2 不要求 self-host FMU 编译工具链**，文档里告诉用户从 Simulink 用 Simulink Coder + FMI Kit 导出。

**测试**：
1. 单元测试：mock 一个简单的 FMU（c-code 编译的 Constant Output FMU）→ 验证适配器能加载、能 step、输出正确
2. 集成测试：跑用户提供的真实 FMU → 文档里给一个 demo FMU

---

### 步 14：MATLAB Engine 适配器

**目标**：无需导出 FMU，直接调 .slx 模型。

**依赖**：MATLAB R2020a+ 安装 + `matlab` Python 包（`python -m pip install matlabengine`）

**MatlabEngineControllerStrategy**：
```python
class MatlabEngineControllerStrategy(ControllerStrategy):
    def __init__(self, params, slx_path, mapping):
        import matlab.engine
        self.eng = matlab.engine.start_matlab()
        self.eng.load_system(slx_path)
        # ... configure inputs/outputs ...
    def compute(self, driver, state):
        # Set workspace vars
        # sim('model_name')
        # Read outputs
```

**性能注意**：start_matlab() 慢（5-15 秒），sim() 也较慢（10-100 ms 取决于模型）。
不能跑实时，仅用于离线/慢速验证。前端 UI 提示"MATLAB 模式速度受 MATLAB 启动限制"。

---

### 步 15：Phase 2 集成测试 + 文档

**Extend `scripts/smoke_test.py`** with new cases:
- 录制 → 导出 → 重新加载 → 数据等价
- 加载含 SplitMu 项目 → 不同位置查询返回不同 mu
- 加载 DLC 脚本 → 跑完成功 → 轨迹合理
- SimplifiedDynamicModel 圆周仿真闭合误差合理
- 悬架 KPI 不为 0 时直行 τ_steer 显著为 0

**新增文档**：
- `docs/tire_model.md` — 线性轮胎假设、参数选择、引用文献
- `docs/fmu_integration.md` — 从 Simulink 导出 FMU 步骤、sidecar 配置示例
- `docs/disturbances.md` — 扰动类型清单 + YAML 示例
- `docs/scripting.md` — 动作 DSL 参考

---

## 5. Phase 3 简要计划

### 步 16：3D 可视化 ✅ 已交付 (2026-05-24)

- 用 Three.js（@react-three/fiber + drei）**并列**于 Canvas2D，视口右上角 2D/3D 切换器，
  两视图共享同一个 zustand store（`Viewport.tsx`）。
- 坐标映射：sim 世界 (X 前 / Y 左 / Z 上) → three (x, z, -y)，保持右手系，
  偏航 ψ 即绕 three Y 轴旋转。
- 渲染：无限地面网格、车身+座舱+车头标记、四轮（cylinder，转角 + 自转可见）、
  扰动区域（减速带=黄色凸起盒、斜坡=倾斜薄板、对开/冰面=半透明地面色块）、
  参考路径 + 桩、轨迹拖尾、ICR 标记（红球=实际，蓝环=目标）。
- 相机：OrbitControls + 跟随模式（保留用户的轨道偏移），HUD 内可切换跟随/自由。
- 性能：per-frame 数据用 `useFrame` + `getState()` 命令式更新 mesh ref，React 树不在 60 Hz 重渲；
  three.js 用 `React.lazy` 代码分割（独立 ~824 KB chunk，仅打开 3D 时加载），低端机默认留在 2D。
- **实测**：浏览器内验证车身/转向/跟随/扰动/路径渲染均正常，无 console 错误。

### 步 17：轨迹编辑器 + pure-pursuit ✅ 已交付 (2026-05-24)

- 后端 `controller/path.py`：进程级 active `PathPlan`（points + cones + closed + version），
  Catmull-Rom 平滑 + 弧长重采样；标准模块生成器：直线/圆弧/slalom/DLC/八字/停车入位，
  其中 slalom/DLC/parking 自带地面**桩 (cones)** —— 同时解决"工况无地面标志、无法手动驾驶"的问题。
- 后端 `controller/follow_trajectory.py`：pure-pursuit，按 4WIS 理想阿克曼实现追踪曲率（稳态零滑移），
  速度由 throttle 控制；`mode_params.cruise_speed` 可让路径自动跑（demo）；开放路径末端速度 taper。
  已注册到 strategy registry（名 `follow_trajectory`）。
- REST：`GET /api/path[/templates]`、`POST /api/path/template|waypoints|clear`；
  state 消息新增 `path_version`，前端据此按需 REST 拉取（不在 60 Hz 流里塞路径点）。
- 前端 `TrajectoryPanel.tsx`：模板下拉 + 生成/清除、巡航速度 + "跟踪此路径"、手动绘制（点击画布放航点 → 完成）。
  路径 + 桩在 2D（Konva）与 3D 同步渲染；2D 画布点击放点用 Konva pointer 事件。
- **实测**：smoke_test 新增"轨迹跟踪"项通过（slalom 跟踪误差 < 2 m，5 种工况离线均收敛）；
  浏览器内 slalom 自动跑完、手动绘制 4 点生成平滑曲线均验证通过。

> **尚未做（步 16-17 范围内的次要项，可后续补）**：第一视角驾驶舱相机、悬架弹簧示意、
> 模块"拖拽"放置（当前是下拉生成）、轨迹编辑的航点拖动微调。

### 步 18：USB 外设

- WebHID API（Chrome/Edge 支持，Safari 不支持 → 这是 Web 标准的限制）
- 检测连接的设备（方向盘 / 手柄）
- 映射轴 → throttle / steering / brake
- 强制反馈：可选（少数高端方向盘支持）
- 后端 Python 端用 pyusb / pygame.joystick 作为 Safari 兜底（启动一个本地代理）

### 步 19：完整多体动力学 ✅ 已交付 (2026-05-25)

- **实现选择**：放弃 Pinocchio/PyDy（重依赖、装难、与现有纯 NumPy 风格不符），改为
  **手写 NumPy 多体模型** `vehicle/multibody.py`，定步长 RK4。
- **14 DOF**：sprung body 6（x,y,ψ,z,roll,pitch）+ 4 簧下垂向（每角 quarter-car）+ 4 轮转。
- 相比 SimplifiedDynamicModel 新增的保真度：
  - 真实**垂向动力学**（轮胎垂向刚度 + 悬架弹簧/阻尼，过坎跳动）；
  - **侧倾/俯仰**为真实 DOF → 载荷转移是**动态**（经 roll/pitch 模态滞后于 ay/ax），
    不再是 `load_transfer.vertical_loads()` 的准静态；
  - bump-steer 由**真实悬架行程**驱动（不再用路面高度近似）。
- 新增参数：`SuspensionParams.{spring_rate, damper_rate, anti_roll_rate}`、
  `VehicleParams.{unsprung_mass, tire_vertical_stiffness}`；roll/pitch 惯量由几何盒近似自动算。
- 注册为第三种模型 `multibody`（Simulator._make_model + /api/model + 前端模型选择器）；
  state 消息增 `attitude{z,roll,pitch}` 与每轮 `susp_defl`；Canvas3D 车身按 roll/pitch 倾斜、
  随 heave 起伏。demo：`projects/multibody_demo.yaml`。
- **验证**：smoke 新增"多体动力学"（静平衡 ΣFz=mg、左转侧倾+外侧加载、制动俯仰+前轴加载）通过；
  headless 复现 5 项物理行为符号/量级正确；**性能 ~111 µs/step = 45× 实时 @200Hz**（远好于计划估的 5-10× 代价）。

---

## 6. 接口兼容性矩阵

| 改动 | Phase 1 客户/策略需修改？ | 备注 |
|------|---------------------------|------|
| 加 `EnvironmentState.disturbances` | 否（默认空列表） | KinematicModel 忽略 |
| 加 `VehicleParams.cg_height` etc. | 否（默认值合理） | 仅 SimplifiedDynamicModel 使用 |
| 加 `VehicleParams.tire` | 否 | 同上 |
| 加 `vehicle.model: kinematic|simplified_dynamic` | 否（默认 kinematic） | Project file 字段 |
| WebSocket state 字段加 `slip_alpha[]`、`slip_kappa[]` | 否（前端可选读取） | 旧前端忽略未知字段 |
| Strategy registry 加 FMU 策略 | 否 | 动态发现，不影响内置 |
| Recorder 加入 Simulator | 否 | 默认 idle 状态 |

**唯一可能的破坏性变更**：如果发现 `ControlCommand.wheel_speed_cmd` 不够（需要 `wheel_torque_cmd`），就加新字段，**不**改原字段语义。所有现有策略输出 wheel_speed_cmd（dynamic model 用 servo 跟踪）；新策略可以选择直接输出 torque（dynamic model 直接施加）。

---

## 7. 测试策略

**单元层**：
- `tests/test_tire_model.py` — 线性轮胎 forces() 形态、力椭圆饱和
- `tests/test_load_transfer.py` — 总和等于 mg，转移方向正确
- `tests/test_dynamic_steady_state.py` — 稳态圆周与解析解一致
- `tests/test_disturbance.py` — query 点正确返回
- `tests/test_recorder.py` — round-trip CSV
- `tests/test_script_runner.py` — 时序正确
- `tests/test_fmu_adapter.py` — 用 mock FMU

**集成层** (`scripts/smoke_test.py` 扩展)：
- 每步完成后立即在 smoke_test 加一项验证
- 目标：Phase 2 结束时 smoke_test 通过 15+ 项

**性能基准** (`scripts/perf_bench.py` 新建)：
- 每个模型单步耗时
- 1000 步连跑总时间
- 防止 Phase 2 大幅退化

---

## 8. 风险登记

| 风险 | 影响 | 缓解 |
|------|------|------|
| 线性轮胎模型在大 slip 下不真实 | 用户看到不合理的极限工况 | 文档明确写"Phase 2 是线性轮胎"；Phase 2.5 上 Pacejka |
| 低速 (vx<1m/s) slip 数值不稳定 | 仿真发散 | velocity blending；动力学 ↔ 运动学 fallback |
| FMU 跨平台编译问题 | Mac 用户的 FMU 跑不动 Windows 导出的 | 文档 + 示例只提供源码，让用户在本机编译 |
| Recorder buffer 内存涨爆 | 30 min × 60 Hz × 大状态 → 数百 MB | 默认 channel 列表精简；超长录制持续写盘而不是全缓冲 |
| 前端 3D 性能差 | 低端机卡顿 | 提供 2D fallback；像素化降级；目标 30 fps 下限 |
| WebHID 在 Safari 不支持 | macOS 默认浏览器没法接方向盘 | 写明限制；Mac 用户用 Chrome 或后端代理 |

---

## 9. 待办状态（任务列表初始化）

启动后用 TaskCreate 加入：

1. 步 7 — Recording + CSV 导出
2. 步 8 — μ 类扰动 + 可视化
3. 步 9 — YAML 动作序列脚本
4. 步 10 — SimplifiedDynamicModel
5. 步 11 — 悬架几何 → 真实阻力矩
6. 步 12 — SpeedBump / Slope
7. 步 13 — FMU 适配器
8. 步 14 — MATLAB Engine 适配器
9. 步 15 — Phase 2 集成测试 + 文档

按 7→9 (Phase 2a) → 10→14 (Phase 2b) → 15 顺序推进。每步：先写测试或冒烟脚本 → 写实现 → 用 smoke_test 验证 → mark complete。

---

## 10. 元数据

- Phase 1 完成时间：2026-05-23
- Phase 2 计划写于：2026-05-23（Phase 1 完成同日）
- **Phase 2 完成时间：2026-05-24**（步 7-15 全部交付）
- 计划版本：v1.2
- 实际差异 vs 计划：基本一致；smoke_test 在 Phase 2 结束时为 24 项（计划目标 ≥15）
- **Phase 3 进度（2026-05-24）**：步 16（3D 可视化）、步 17（轨迹编辑器 + pure-pursuit）已交付。
  随后按集成测试评估（见 [phase3_review.md](phase3_review.md)）完成 P0+P1 改进：
  P0（项目 model 字段修正 / 减速带脉冲修正 / 键盘保持模式 / 每轮 μ 可视化）、
  P1（A/B 双跑对比 / bump-steer 近似 / 脚本↔桩关联）。smoke_test 增至 27 项（26 通过，
  唯一未过为既有 kingpin τ 单调，列入 P2）。
- **Phase 3 进度（2026-05-25）**：步 19（多体动力学 14 DOF）+ **P2（动力学标定）** 全部交付。
  P2 内容：kingpin τ 单调修复、转向作动器一阶滞后+速率限幅、轮胎侧偏刚度按 LS9 标定（c_alpha 70k→120k）。
  端口改 8010、摩擦系数绝对值化、LS9 标定等追加需求亦完成。**smoke 28/28 全通过**。
  **唯一剩余：步 18（WebHID 外设，需真实方向盘/手柄硬件）。**

## 11. 已知遗留 / Phase 3 优先项

1. ~~**轮速 PI 服务在稳态加速时存在前后轮 κ 不对称**~~（2026-05-25 复核）：尝试了 §11 建议的
   `T_ff = r·Fx` 前馈，但因 Fx 含 c_kappa·κ（依赖车轮自身转速）→ **正反馈致前轮打滑（κ 飙到 0.79）**，
   已否决。裸 PI 的残余不对称（加速瞬态 Δκ≈0.026）很小且稳定，判定可接受、不再处理。
2. **plugin reload 不清理 stale 条目** —— 重命名 / 删除 FMU 后旧名仍会保留在
   registry 中直到重启。修复需要在 `_BUILTIN` 中加 source 元数据。
3. **Slope 只产生纵向重力分量**，不考虑侧向坡（cross-slope）。轨迹编辑后可补。
4. **Pacejka Magic Formula**（取代 LinearTireModel）— 已在 Phase 3 计划。
5. ~~**kingpin τ_steer 不随转角单调**~~（2026-05-25 修复，P2-1）：根因是 KPI 居中项写成常量
   `Fz·sin(KPI)·scrub`（直行也非零、量级压过 Fy 项），改为 ∝ sin(δ)（直行≈0、随转角增长）+
   caster 机械拖距 `tire_radius·tan(caster)`。现单调（158→313→613 N·m），smoke 通过。

执行过程中如果发现计划不对，**就地更新本文档**，不要让现实和计划脱节。
