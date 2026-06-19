# 4WIS 整车模型底座重构计划

日期：2026-06-19

本文档记录 4WIS Simulator 下一轮基础重构的工程边界。目标是把分散在仿真模型、负载特性页、控制器几何和实时面板里的物理/运动学公式收敛到一个可复用的底层模型库，同时避免把页面、仿真循环、控制器和 API 过度耦合在一起。

## 1. 背景

当前项目已经形成两条并行物理栈：

1. 时域仿真栈：`KinematicModel`、`SimplifiedDynamicModel`、`MultiBodyModel` 各自处理轮端速度、轮胎力、载荷和转向阻力。
2. 准静态分析栈：`vehicle/load_analysis.py` 为负载特性页单独实现 toe/camber/drive/aero/Pacejka/kingpin/linkage，并在 v0.7.5 加入稳态 bicycle coupling。

这带来三个问题：

- 同一物理概念有多处实现，后续修正容易漏一边。
- 负载页和时域模型的结果越来越难解释到同一套数学模型上。
- 多体模型、控制器几何、实时面板各自扩展，容易偏离主线。

## 2. 已确认的设计决策

- 架构选择：分层组件库，不做一个巨大的 `VehicleModel` 单体。
- 时域 fidelity：产品主线收敛为两层，`kinematic` + `dynamic`。`multibody` 暂作为研究/回归参考保留，不再作为所有新曲线和页面必须同时支持的主线。
- 参数策略：中度瘦身。工程核心参数直接展示；低敏感度或估算参数折叠到高级设置，不能为了照顾每个参数把模型变成难以解释的非线性堆叠。
- 新增说明页：静态讲解为主，配少量关键交互点。内容要能让产品经理理解，同时公式和工程边界足够让资深底盘工程师复核。

## 3. 低耦合原则

底层模型统一，不等于模块互相知道对方存在。后续代码按这些依赖方向组织：

```text
VehicleParams / lightweight inputs
        ↓
vehicle model core / component functions
        ↓
adapters used by dynamic model, load-analysis page, realtime diagnostics
        ↓
API routers / simulator loop / frontend views
```

禁止的方向：

- `vehicle/model_core.py` 不依赖 simulator、controller、API router、frontend schema。
- 控制器只输出 `ControlCommand`，不计算轮胎力、主销力矩或齿条力。
- API router 只做请求/响应适配，不放物理公式。
- 前端图表只消费后端行数据和说明文案，不复制物理计算。
- 负载页可以有页面级聚合逻辑，但不能再私有实现一套 toe/camber/aero/load-sensitive/slip 公式。

## 4. 底层组件分层

建议按由下到上的可复用组件组织：

| 层 | 职责 | 当前落点 |
| --- | --- | --- |
| 轮位几何 | 轮心位置、车体系/轮系速度变换、力旋转 | `vehicle/model_core.py` |
| 轴/整车载荷 | 静态 Fz、载荷转移、气动升力/下压力 | `load_transfer.py` + `model_core.py` |
| 轮胎输入 | slip angle、slip ratio、load-sensitive stiffness | `model_core.py` + `tire.py` |
| 轮胎力 | linear/Pacejka、combined slip、friction ellipse | 后续从 `load_analysis.py` 收敛 |
| 转向负载 | kingpin torque、linkage、rack/motor force | `kingpin.py` + `geometry.py` |
| 整车响应 | kinematic LSQ、3-DOF dynamic、稳态 bicycle | `kinematic.py`、`dynamic.py`、`model_core.py` |

`model_core.py` 现在是第一步聚合点。如果后续继续变大，应拆成 `vehicle/foundation/kinematics.py`、`loads.py`、`tire_inputs.py` 等小模块，而不是把所有物理塞进一个文件。

## 5. 当前已完成的步骤

### R0. 轮端运动学和准静态输入统一

已经抽出并接入第一批低层纯函数：

- `wheel_center_velocities_body`
- `body_to_wheel_frame`
- `wheel_slip_kinematics`
- `rotate_wheel_forces_to_body`
- `static_toe_offsets`
- `camber_per_wheel`
- `drive_force_per_wheel`
- `fz_with_aero_lift`
- `load_sensitive_cornering_stiffness`
- `steady_state_slip_angles`
- `low_speed_blend`
- `parking_turn_scale`

已接入：

- `SimplifiedDynamicModel` 使用 `wheel_slip_kinematics` 和 `rotate_wheel_forces_to_body`。
- `MultiBodyModel` 使用同一套 wheel slip / force rotation。
- `load_analysis.py` 使用同一套 toe/camber/drive/aero/load-sensitive/steady-state slip/parking blend。

验证：

- 后端测试：`157 passed`
- 负载页 + core 聚焦测试：`31 passed`
- smoke test：`32/32 通过`
- 前端 type-check：通过
- 高速线性区趋势：`tau_ideal` 斜率从 10 km/h 的约 `62.7 N m/deg` 增至 200 km/h 的约 `373.0 N m/deg`

### R1. 轮胎力微内核共享

已把负载页私有的 Pacejka pure-slip 和 friction ellipse 抽到 `vehicle/tire.py`：

- `pacejka_pure_slip_forces`
- `friction_ellipse_clip`
- `pacejka_combined_forces`

`PacejkaTireModel` 和 `load_analysis.py` 现在都使用同一套无状态函数。负载页仍然可以传入每个轮子的 `c_alpha_eff(Fz)`、等效 camber slip 和等效 drive slip，但不再复制 Pacejka/combined-slip 公式。

新增测试：

- 共享函数与 `PacejkaTireModel.forces()` 输出一致。
- pure-slip 可超过附着椭圆，combined 函数负责裁剪。
- 负载页现有 smooth shoulder 回归保持不变。

### R2. 建立整车输入/输出小数据结构

已为底层函数增加第一批薄数据结构：

- `WheelKinematics`
- `WheelAlignment`
- `WheelLoads`
- `WheelForceSet`

这些结构只承载数组和单位明确的数值，不带仿真循环状态，不保存 UI 字段。`load_analysis.py` 现在使用：

- `WheelAlignment` 传递 toe/camber。
- `WheelLoads` 传递 static Fz、aero-adjusted Fz、load-sensitive `c_alpha(Fz)`。
- `WheelForceSet` 传递 actual/ideal 轮胎力和回正力矩。

暂不把 `SteeringLoads` 放进 `model_core.py`，因为它会天然依赖 `kingpin.py` 和 `geometry.py`。后续如果需要，可在 `vehicle/steering_loads.py` 建立单向依赖，而不是让底层 core 反向依赖上层转向机构。

验收状态：

- 现有 `VehicleState` 未修改。
- WebSocket/API row 字段保持向后兼容。
- `test_model_core.py` 增加 bundle 与旧 helper 一致性测试。

### R3. 收敛主动模型为两层

已把模型目录收敛到显式层级：

- `vehicle/model_registry.py` 是后端单一模型目录。
- `kinematic` 与 `simplified_dynamic` 标记为 `primary`。
- `multibody` 标记为 `research`，仍可手动选择、旧项目仍可读取。

已接入：

- `Simulator._make_model()` 改为走 registry。
- `/api/model` 返回 `models[]`、`primary[]` 和当前 `layer`。
- 前端控制面板把主线模型和研究模型分组显示。
- `smoke_test.py` 要求普通扰动项目使用 `simplified_dynamic`，只允许 `multibody_demo.yaml` 作为研究例外。

验收：

- 项目 schema 中现有 `model` 字段继续可读。
- 默认项目仍可运行。
- 说明页明确“主线模型”和“研究模型”的区别。

### R4. 参数瘦身与分组

已建立共享参数分组元数据：

- `frontend/src/vehicle/parameterGroups.ts` 是前端唯一参数分组目录。
- `ParamsPanel` 与负载页 `LoadParamsEditor` 都使用同一套 `core` / `advanced` 分组。
- 核心参数包含整车质量/几何/惯量、轮胎刚度、转向执行器、主销/齿条传动。
- 高级参数折叠 Pacejka shape、气动、停车补偿、bump-steer、伺服、多体研究参数和齿条硬点。

本轮只做展示层瘦身，不删除字段：

- `/api/params` 字段名保持不变。
- 旧 YAML 兼容路径保持不变。
- 负载页仍发送完整 `params` 副本给后端，图表计算不迁移到前端。

验收：

- YAML 兼容旧字段。
- 默认 LS9 profile 不丢关键工程信息。
- 全局参数面板和负载页使用同一套参数命名。

### R5. 新增“数学模型”说明页

已新增顶层页面：

- `frontend/src/components/ModelTheoryPage.tsx`
- `App.tsx` 顶层 tab 增加“数学模型”

页面结构：

1. 车体坐标系和四轮编号。
2. 单轮：轮心速度、轮系变换、slip angle / slip ratio。
3. 单轴：垂向载荷、toe/camber、轮胎力。
4. 整车：kinematic LSQ、3-DOF dynamic、`multibody` 研究边界。
5. 稳态 bicycle coupling：解释高速线性区收缩和斜率增大。
6. 转向负载链路：轮胎力 -> kingpin torque -> tie rod/rack -> motor torque。
7. 参数边界：直接复用 R4 的核心/高级参数分组。
8. 工程边界：控制器/API/前端/负载页的低耦合规则。

验收：

- 浏览器打开“数学模型”tab：标题、8 个章节、bicycle coupling、转向负载链路均可见。
- 1440px 宽度无横向溢出。
- 切回负载页后核心/高级参数分组和图表仍渲染正常。

## 6. 下一步实施顺序

### R6. 后续建议

- 把数学模型页中的关键 SVG 图解进一步拆成可复用组件，供未来弹窗/说明复用。
- 在后端增加 `/api/model/foundation` 元数据接口，把模型 registry、参数分组、说明页锚点串起来。
- 继续审查控制器目录，筛出仍然私有实现 ICR/几何推导且可下沉到 `vehicle` 层的分支。

## 7. 不做清单

- 不把负载页直接改成 simulator 的一个运行模式。
- 不让控制器调用负载页或轮胎力内核。
- 不为了兼容 `MultiBodyModel` 把所有核心接口复杂化。
- 不在前端复制公式做“看起来一样”的曲线。
- 不一次性删除大量参数；先分组折叠，再用测试和实际使用判断能否退役。
