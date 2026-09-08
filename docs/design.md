# 4WIS Simulator — 坐标与设计约定

> 2026-09-08：坐标与积分约定已按当前实现校正。完整评审见 [ASTRA 科学模型评审](reports/astra_review_2026-09-08.md)。

## 1. 总体架构

参见 [README.md](../README.md)。

## 2. 坐标系约定

- 全局坐标系：固定右手系，Z 向上；X/Y 不随车辆转动。ψ=0 时与车体系重合。
- 车体坐标系：原点 O 位于前后轴中点，X 向前、Y 向左、Z 向上；与 SAE 的 Y 右/Z 下约定不同。
- 质心 G 位于 `(e, 0)`，`e=L/2−cg_to_front`。公开 pose、vx、vy、ICR 都以 O 为基准，横摆惯量在 G 定义。
- `M_G=M_O−e*Fy`；`u_dot=Fx/m+r*v+e*r²`；`v_dot=Fy/m−r*u−e*r_dot`。不可将 O 的速度代入质心形式而漏掉偏心项。
- 多体 heave 在 O，roll 沿 +X 为正（右侧下沉），pitch 沿 −Y 为正（抬头）；这是小角近似。
- 偏航角 ψ：车体 X 轴相对全局 X 轴绕 Z 轴的转角，逆时针为正
- 各车轮编号约定：
  - `FL` 左前（front-left）
  - `FR` 右前（front-right）
  - `RL` 左后（rear-left）
  - `RR` 右后（rear-right）

## 3. 仿真状态量（StateBus）

每个仿真周期向状态总线写入：

| 字段 | 含义 | 单位 |
|------|------|------|
| `t` | 仿真时刻 | s |
| `pose.x, pose.y, pose.psi` | 整车位姿 | m, m, rad |
| `velocity.vx, .vy, .yaw_rate` | 车体速度 + 横摆角速度 | m/s, m/s, rad/s |
| `wheels[i].delta` | 车轮 i 转角 | rad |
| `wheels[i].omega` | 车轮 i 转速 | rad/s |
| `wheels[i].fz` | 车轮 i 垂直载荷（Phase 2 启用） | N |
| `wheels[i].torque_steer` | 车轮 i 转向阻力矩 | N·m |
| `wheels[i].icr_local` | 车轮 i 对应的转向中心（车体系） | m |
| `vehicle_icr_local` | 整车瞬心（车体系） | m |
| `commands.delta_cmd[i]` | 控制器指令（vs 实际值） | rad |

## 4. 模型抽象

### VehicleModel（基类）
```python
class VehicleModel(ABC):
    @abstractmethod
    def step(self, dt: float, cmd: ControlCommand, env: EnvironmentState) -> VehicleState: ...
    @abstractmethod
    def reset(self, init: VehicleState) -> None: ...
```

产品主线保留两层：

- `KinematicModel`：积分速度→位姿，不考虑轮胎滑移/侧偏，不计载荷转移，适合几何和策略验证。
- `SimplifiedDynamicModel`：3-DOF 整车 + 轮胎力 + 垂直载荷转移 + 转向负载，是工程曲线和实时 KPI 的主物理模型。

`MultiBodyModel`（14 DOF：body 6 + 4 簧下垂向 + 4 轮转）保留为研究/回归模型，见
`vehicle/multibody.py` 和 `projects/multibody_demo.yaml`。它可手动选择，但后续新接口和页面不默认要求同步适配多体路径。

### ControllerStrategy（基类）
```python
class ControllerStrategy(ABC):
    @abstractmethod
    def compute(self, driver: DriverInput, state: VehicleState, params: VehicleParams) -> ControlCommand: ...
```

Phase 1 内置 5 种。Phase 2 加 FMU/MATLAB Engine 适配器，使外部 Simulink 策略也能作为 `ControllerStrategy` 实例注入。

## 5. 仿真循环

- 内部默认步长：5 ms（200 Hz）。RK4 用于车身/悬架子步，真实非线性后向欧拉用于轮速；分步耦合使整体收敛阶降为一阶。定量研究必须检查步长敏感性。
- 状态推送目标 60 Hz；当前实现整步取整，默认每 3 步推送，仿真时间频率约 66.7 Hz。
- 推送通道：WebSocket（JSON），通道名 `/ws/state`
- 实际实现见 `backend/src/sim4wis/core/simulator.py`

## 6. 项目文件 schema

见 [projects/default.yaml](../projects/default.yaml) 与 `backend/src/sim4wis/project/schema.py`（Step 5）。

## 7. 理想阿克曼策略数学推导（Phase 1 关键）

给定期望瞬心位置在车体坐标系下为 `(x_R, y_R)`（点 `R`）：

对每个车轮 `i`，其在车体系下的安装位置为 `(x_i, y_i)`，目标转角：
```
δ_i = wrap_to_[-π/2,π/2](atan2(x_i - x_R, y_R - y_i))
```
（注：转角 = 车轮 X 轴 与 车体 X 轴 的夹角，逆时针为正；车轮 Y 轴指向瞬心方向。）

进一步，整车的横摆角速度：
```
ψ̇ = v / r,   其中 r = √(x_R² + y_R²)（瞬心到车辆几何中心的距离）
```

每个车轮的线速度方向沿其纯滚动方向，速度大小：
```
v_i = ψ̇ · √((x_R - x_i)² + (y_R - y_i)²)
```

**几何性质**：当所有转角可达且四轮速度相容时，四轮法线共点于 R。运动学层的零侧滑约束不能作为真实轮胎零侧滑或零磨损的证明。
这是 4WIS 相对传统转向最显著的几何优势，Step 2 的策略实现会以此为准。

详细推导与可视化对比见 Step 2 提交的 PR / commit。

## 数值与可达性边界

- 运动学采用恒定车体速度的 SE(2) 精确位姿增量；不产生可用于载荷验证的轮胎力。
- 人工驾驶的转弯参考采用 μg 速度限制，保持同一 ICR；显式 `speed_target_ms` 实验命令不经此限制。
- 原地转向的默认指令上限为 30°/s，可在界面调整；不是实车能力承诺。默认 35° 轮角小于纯滚动需要的约 63.7°。
- 齿条力以虚功导数 `ds/dδ` 计算，现有 `geometry_efficiency` 字段仅作几何指标；不可达行程须显式报告。
- 路面采样、纵坡重力和垂向动力学属于低阶近似。横坡重力、三维接触法向和有限接地印迹尚未完整建模。
