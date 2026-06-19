# 4WIS Simulator — 设计文档（Phase 1）

> 状态：草稿。Step 2 起会随实现填充细节。

## 1. 总体架构

参见 [README.md](../README.md)。

## 2. 坐标系约定

- 全局坐标系：右手系，X 向前、Y 向左、Z 向上（汽车工程惯例）
- 车体坐标系：原点位于车辆几何中心（地面投影），X 向前、Y 向左、Z 向上
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

- 内部固定步长：5 ms（200 Hz）—— 给未来动力学扩展留余量
- 状态推送频率：60 Hz（每 ~3.3 内部步推一次）
- 推送通道：WebSocket（二进制 JSON），通道名 `/ws/state`
- 实际实现见 `core/loop.py`（Step 3）

## 6. 项目文件 schema

见 [projects/default.yaml](../projects/default.yaml) 与 `backend/src/sim4wis/project/schema.py`（Step 5）。

## 7. 理想阿克曼策略数学推导（Phase 1 关键）

给定期望瞬心位置在车体坐标系下为 `(x_R, y_R)`（点 `R`）：

对每个车轮 `i`，其在车体系下的安装位置为 `(x_i, y_i)`，目标转角：
```
δ_i = atan2(x_R - x_i, -(y_R - y_i))
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

**关键性质**：四个车轮的瞬心严格共点（即 `R`），轮胎无侧偏，零滑移、零附加磨损。
这是 4WIS 相对传统转向最显著的几何优势，Step 2 的策略实现会以此为准。

详细推导与可视化对比见 Step 2 提交的 PR / commit。
