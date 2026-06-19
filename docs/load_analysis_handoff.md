# 转向负载分析页交接记录

日期：2026-06-17

这份文档记录今天新增和迭代的 `负载特性` 独立分析页，供后续 Claude 继续开发使用。

## 一句话目标

在现有 4WIS 仿真工作台之外，新增一个独立工程分析页面，用车辆底盘、轮胎、主销、转向硬点和齿条机构参数，计算单轮/四轮在不同车速和车轮转角下的转向阻力矩、齿条力、侧向力、几何效率和零输出自然转角。

## 入口和运行方式

前端顶栏新增了页面切换：

- `仿真工作台`：保持原有仿真 UI。
- `负载特性`：新增转向负载分析页。

本地验证时使用过的启动方式：

```bash
cd backend
PYTHONPATH=src uvicorn sim4wis.main:app --host 127.0.0.1 --port 8011

cd ../frontend
SIM4WIS_BACKEND_HTTP=http://127.0.0.1:8011 \
SIM4WIS_BACKEND_WS=ws://127.0.0.1:8011 \
npm run dev -- --host 127.0.0.1 --port 5173
```

注意：`frontend/vite.config.ts` 默认代理到 `http://127.0.0.1:8010`。如果后端跑在 8011，必须带上上面的环境变量，否则负载页会打到错误后端。

## 关键文件

前端：

- `frontend/src/App.tsx`
  - 新增顶层 `PageId = "sim" | "load"`。
  - 顶栏 `page-tabs` 中新增 `负载特性`。
  - `LoadAnalysisPage` 作为独立页面挂载，不塞进原有侧栏。
- `frontend/src/components/LoadAnalysisPage.tsx`
  - 负载分析页主体。
  - 车型 profile 选择/保存/应用。
  - 参数编辑表单。
  - 单轮阻力矩、齿条力、侧向力主图。
  - KPI、实时四轮负载、齿条-车轮几何、零输出自然转角图。
  - 反解齿条位移到车轮转角的机构示意逻辑也在这里。
- `frontend/src/charts/uplotFactory.ts`
  - uPlot 通用封装。
  - 已启用 cursor 竖线和数值读数。
  - 已关闭 hover 点：`cursor.points.show = false`，series points 也关闭。
- `frontend/src/styles.css`
  - 负载页布局、图表、KPI、机构 SVG 样式。
- `frontend/src/types/sim.ts`
  - WebSocket 轮端字段扩展：`rack_force`、`motor_torque_demand`、`linkage_*`、`tire_fx/fy`、`side_force_body_y`、`side_force_summary`。

后端：

- `backend/src/sim4wis/core/state.py`
  - 新增/扩展转向负载所需参数。
  - `SteeringGeometryParams`：前/后轴左轮 2D 硬点；右轮按 Y 镜像。
  - 轮胎/低速参数：`tire_width`、`contact_patch_radius`、`parking_scrub_coeff`、`low_speed_blend_ms`。
  - LS9 默认几何参数已写入。
- `backend/src/sim4wis/vehicle/geometry.py`
  - `steering_linkage_metrics(...)`：给定车轮转角，求拉杆/齿条几何、齿条位移、效率。
  - `wheel_rack_force_from_linkage(...)`：把 kingpin torque 转成 rack force 和 motor torque。
- `backend/src/sim4wis/vehicle/load_analysis.py`
  - `/api/load-analysis/sweep` 的核心计算。
  - 输出每个速度、请求转角、每个轮子的负载行。
  - 已做低速静态接地斑侧向力和停车转向阻力补偿。
- `backend/src/sim4wis/api/routers/load_analysis.py`
  - REST：`POST /api/load-analysis/sweep`。
- `backend/src/sim4wis/api/routers/vehicle_profiles.py`
  - REST：车型库 CRUD。
- `backend/src/sim4wis/project/vehicle_profiles.py`
  - 车型 profile YAML I/O。
  - 内置/默认 `LS9` profile。
- `vehicle_profiles/LS9.yaml`
  - 当前 LS9 profile 文件。
- `backend/src/sim4wis/project/schema.py`
  - 项目 YAML round-trip 已补齐齿条参数和 steering geometry。
- `backend/src/sim4wis/core/simulator.py`
  - WebSocket 状态扩展：实时 rack force、motor torque、linkage、tire force、side force summary。
- `backend/tests/test_load_analysis.py`
  - 负载页相关后端回归测试。

## REST API

### 车型 profile

路径：

- `GET /api/vehicle-profiles`
- `GET /api/vehicle-profiles/{name}`
- `POST /api/vehicle-profiles/{name}`
- `POST /api/vehicle-profiles/{name}/apply`
- `DELETE /api/vehicle-profiles/{name}`

语义：

- profile 只保存车辆/底盘/轮胎/转向几何，不保存场景、控制策略、扰动、录制配置。
- 内置名称：`LS9`；兼容 alias：`im_ls9`。
- 用户保存 profile 时写入 `vehicle_profiles/{name}.yaml` 或由 `SIM4WIS_VEHICLE_PROFILES_DIR` 指定目录。

### 负载 sweep

路径：

```http
POST /api/load-analysis/sweep
```

请求字段：

- `params`：可选车辆参数覆盖；为空时用当前仿真参数。
- `speeds`：m/s 数组，最多 80 个点。
- `angles`：rad 数组，最多 161 个点。
- `wheel_index`：0 FL、1 FR、2 RL、3 RR。
- `mode`：`single_wheel` 或已有策略名。
- `mu`：路面附着系数。

返回字段：

- `rows`：扁平数组，每个速度、请求角、每个轮一行。
- `summary`：峰值齿条力、峰值电机力矩、最低几何效率、最大附着利用率和警告。

单行 row 主要字段：

- `speed`：m/s。
- `requested_angle`：输入请求角 rad。
- `wheel_index`、`wheel_label`。
- `delta`：该轮实际转角 rad。
- `slip_alpha`。
- `fz`。
- `tire_fx`、`tire_fy`、`tire_mz`。
- `torque_steer`：转向阻力矩，单位 N m。
- `rack_force`：齿条力，单位 N。
- `motor_torque`：电机轴等效力矩，单位 N m。
- `side_force_body_y`：车身 Y 方向侧向力，单位 N。
- `arm_tie_angle`、`tie_rack_angle`。
- `geometry_efficiency`。
- `rack_travel`。
- `friction_utilization`。

## 前端页面结构

页面分区：

1. 顶部工具栏
   - 车型选择。
   - `载入`。
   - `应用到仿真`。
   - 保存新车型。
   - 导出全部 sweep CSV。

2. 左侧参数表
   - 所有尺寸显示和编辑为 mm。
   - 内部仍然按 SI 单位保存到后端。
   - 参数分组：
     - 底盘/轮胎。
     - 轮胎/低速补偿。
     - 主销/悬架。
     - 齿条/硬点。

3. 主控制栏
   - 车轮：FL/FR/RL/RR。
   - 模式：单轮强制转角、理想阿克曼、传统阿克曼、后轮转向、蟹行、零半径。
   - μ。
   - 最高车速、速度点。
   - 剖面车速：数值输入 + slider。
   - 最大转角、转角点。
   - 重新计算。

4. 置顶主图
   - 设定车速下：`转向阻力矩 tau - delta`。
   - 设定车速下：`齿条力 F_rack - delta`。
   - 设定车速下：`单轮侧向力 Fy_body - delta`。

5. 模型说明
   - 当前说明强调：
     - 低速静态接地斑补偿。
     - 行驶侧偏模型。
     - 零输出自然转角定义为 `F_rack = 0` 的车轮转角。
     - 当前对称模型下自然转角通常为 0 度。

6. KPI
   - 峰值齿条力。
   - 峰值电机力矩。
   - 最低几何效率。
   - 最大附着利用。
   - 零输出自然转角。
   - 0 度保持齿条力。

7. 实时四轮负载
   - 每轮显示：
     - delta。
     - steering torque。
     - rack force。
     - body-Y side force。
     - linkage efficiency。
   - 左侧合力、右侧合力、整车侧向合力。

8. 齿条-车轮几何
   - 机构图：屏幕上方为车辆前方。
   - 主销固定。
   - 转向节臂绕主销旋转。
   - 拉杆长度固定。
   - 内球头/齿条铰点沿齿条轴线移动。
   - slider 控制齿条位移。
   - 右侧小图显示 `rack travel -> wheel angle` 曲线。

9. 下方图表
   - 零输出自然转角 `delta_eq - 车速`。
   - 几何效率。
   - 转向节臂-拉杆夹角。
   - 拉杆-齿条夹角。
   - 四轮侧向力 @最高车速。
   - 左右轮侧向合力 @最高车速。

图表行为：

- 保留 cursor 竖线。
- 保留顶部数值读数。
- 删除/关闭曲线交点圆点显示。
- PNG 和 CSV 导出仍可用。

## 计算模型口径

### 单轮 sweep

`single_wheel` 模式下，只有选中的车轮被强制给定请求转角，其余轮为 0。

车辆速度被视为车身直行速度，车轮相对车身速度方向产生强制侧偏。

行驶侧偏：

- 轮端速度近似：
  - `vx_w = cos(delta) * speed`
  - `vy_w = -sin(delta) * speed`
- `alpha = atan2(vy_w, max(abs(vx_w), VMIN_SLIP))`
- 使用当前 tire model：linear 或 Pacejka。

### 低速和 0 车速补偿

之前的 bug：0 km/h 时侧偏力为 0，1 km/h 突然进入行驶侧偏，造成不连续。

现在的修正：

- 使用 `low_speed_blend = exp(-(abs(speed) / low_speed_blend_ms)^2)`。
- 行驶侧偏力乘以 `1 - low_speed_blend`。
- 增加静态接地斑侧向力：
  - `parking_side_force = parking_scrub_coeff * mu * fz * tanh(delta / 8deg) * low_speed_blend`
- 增加停车转向阻力矩：
  - `parking_torque = parking_scrub_coeff * mu * fz * contact_patch_radius * tanh(delta / 8deg) * low_speed_blend`

效果：

- 0 km/h 非零车轮转角会产生准静态侧向力和停车转向阻力。
- 0 到 1 km/h 平滑过渡。
- 例：LS9、FL、delta=0.2rad、mu=0.85：
  - 0 kph：约 `tau=868.5 N m`、`rack=6983 N`、`Fy=4309 N`。
  - 1 kph：约 `tau=851.1 N m`、`rack=6843 N`、`Fy=4367 N`。

### Kingpin torque 和齿条力

主销阻力矩：

- `kingpin_torque(...)` 使用 `fx/fy/mz/fz/suspension/delta/tire_radius/tire_t_pneumatic`。
- 之后加上低速停车转向阻力矩。

齿条力：

- `wheel_rack_force_from_linkage(...)` 调用 `steering_linkage_metrics(...)`。
- linkage 由硬点求：
  - 外球头随车轮转角绕主销旋转。
  - 内球头沿齿条轴线移动。
  - 保持零位拉杆长度。
  - 输出转向节臂-拉杆夹角、拉杆-齿条夹角、几何效率、齿条位移。
- `rack_force = torque / (arm_length * efficiency)`。
- `motor_torque = rack_force * pinion_radius / motor_gear_ratio`。

### 零输出自然转角

前端目前基于 sweep 数据计算，不是后端直接返回：

- 对每个车速，从选中车轮的 `rack_force - delta` 曲线找零交点。
- 优先使用 `F_rack = 0`。
- 找不到时退化使用 `torque_steer = 0`。
- 多个零点时选绝对值最小的转角。

当前模型的物理结论：

- 在左右对称、无 toe、无 camber thrust、无路拱、无横风、无转向系统偏置扭矩时，高车速不会让自然位置偏离 0 度。
- 高车速主要改变偏离 0 度后需要的保持力，而不是必然改变自然平衡点。
- 若希望出现“不同车速自然位置不同”，需要加入偏置源，例如：
  - static toe / toe compliance。
  - camber thrust。
  - road crown / bank。
  - crosswind。
  - steering column/rack residual torque。
  - left/right suspension asymmetry。
  - wheel alignment manufacturing offsets。

当前验证过：LS9 对称模型在 80 kph 下自然转角为 `0.0deg`，0 度保持齿条力为 `0 N`。

## LS9 默认参数

`vehicle_profiles/LS9.yaml` 和 `VehicleParams()` 默认值已使用 LS9 作为默认车型。

用户截图参数已写入：

- 主销间距/前等效轮距 `b = 1565.172 mm`。
- 轴距 `l = 3160 mm`。
- 齿条长度/内球心距相关 `a = 840 mm`，目前折算到前后对称硬点模型。
- 梯形臂长 `r = 146.451 mm`。
- 横拉杆长 `c = 352.052 mm`。
- 齿条轴线相对前轴偏距 `h = 176.324 mm`。
- 梯形底角 `theta = 85.361 deg`。

当前硬点：

- `front_outer_x = -0.145971235`
- `front_outer_y = -0.011844575`
- `front_inner_x = -0.176324`
- `front_inner_y = -0.362586`
- `front_rack_axis_deg = 90.0`
- `front_rack_travel_limit = 0.085`
- 后轴按前轴镜像/估算。

注意：

- 内部单位仍为 m/rad/SI。
- UI 显示为 mm/deg/kph。
- 未公开项目仍是工程估算：主销后倾、内倾、外倾、scrub、接地印迹半径、CG 高度、轮胎侧偏刚度等。

## WebSocket 实时字段

实时状态中每个 wheel 增加：

- `rack_force`
- `motor_torque_demand`
- `linkage_arm_tie_angle`
- `linkage_tie_rack_angle`
- `linkage_efficiency`
- `tire_fx`
- `tire_fy`
- `side_force_body_y`

同时增加：

- `side_force_summary.left`
- `side_force_summary.right`
- `side_force_summary.total`
- `side_force_summary.source`

运动学模型没有真实轮胎力时，侧向力来源会标记为估算。

## 已验证命令

最近一次验证通过：

```bash
cd frontend
npm run type-check
npm run build

cd ..
python3 scripts/smoke_test.py

cd backend
python3 -m pytest tests/ -q
```

历史验证结果：

- `backend/tests/test_load_analysis.py`：通过。
- 全量后端 pytest：`128 passed`。
- smoke：`32/32 passed`。
- 浏览器验证：
  - 负载页可渲染。
  - cursor hover 后没有点元素。
  - 新增 KPI 存在。
  - 齿条 slider 拖动时主销点位移 `0 px`，内球头沿齿条轴移动。

## 已知问题和模型边界

1. 零输出自然转角目前是前端从 sweep 曲线求零点，不是后端 summary 字段。
   - 如果后续要导出或做 API 稳定 contract，建议挪到后端。

2. 当前模型没有偏置源，所以自然转角随车速曲线通常是 0。
   - 这不是 UI bug，是模型对称性的结果。
   - 若用户想看到不同车速自然位置，需要新增物理参数。

3. 齿条位移 -> 车轮转角反解目前在前端 `LoadAnalysisPage.tsx` 内。
   - 它使用圆-圆交点：主销圆半径为转向节臂长，内球头圆半径为拉杆长。
   - 选取离上一解最近的连续分支。
   - 建议后续把反解迁到共享后端/工具函数，并补单元测试，避免前后端几何分叉。

4. 低速静态接地斑模型是工程近似。
   - `STATIC_TIRE_DEFLECTION_RAD = 8deg` 目前写死在 `load_analysis.py`。
   - 如果需要标定，应改成 VehicleParams 可编辑字段，并写入 profile/project schema。

5. `parking_scrub_coeff` 同时影响停车侧向力和停车阻力矩。
   - 后续可能需要拆成两个参数：
     - `parking_lateral_coeff`
     - `parking_torque_coeff`

6. 当前单轮强制转角的侧向力，是车身直行速度下的准稳态强制侧偏，不等同于完整车辆自由响应。
   - 适合做机构/电机负载 sizing。
   - 不应解释为闭环整车稳定后的自然转向状态。

7. 图表 cursor 点已经关闭。
   - 只保留竖线和顶部读数。
   - 不要再恢复自定义 overlay point，之前定位/语义容易引起误解。

## 推荐给 Claude 的下一步

优先级高：

1. 增加能让自然转角随车速偏移的偏置模型。
   - 建议最小实现：
     - `static_toe_front/rear` 或 per-wheel toe offset。
     - `camber_thrust_coeff`。
     - `steering_bias_torque` 或 `rack_bias_force`。
   - 同步修改：
     - `VehicleParams`
     - params API
     - project schema
     - vehicle profile YAML
     - LoadAnalysisPage 参数表
     - sweep 计算
     - tests

2. 把零输出自然转角计算移到后端。
   - 在 `/api/load-analysis/sweep` summary 或新增字段中返回：
     - per-speed `delta_eq`
     - `rack_force_at_zero`
     - `torque_at_zero`
     - 是否找到零交点。

3. 把齿条位移 -> 车轮转角反解做成后端公共函数。
   - 与 `steering_linkage_metrics(delta -> rack_travel)` 互为逆向验证。
   - 单测内容：
     - 0 mm -> 0 deg。
     - 正负行程单调。
     - 主销固定。
     - 拉杆长度守恒。
     - 左右轮镜像符号。
     - 不可达行程处理。

优先级中：

4. 增加自然转角图的 CSV 导出字段。
5. 图表加 hover 竖线读数时，长文本在窄屏可能仍偏挤，可继续优化。
6. LS9 后轴硬点目前是估算/镜像，后续有实测数据应覆盖。
7. 给 `LoadAnalysisPage` 拆组件：
   - `ProfileToolbar`
   - `LoadParamsEditor`
   - `LoadChartBox`
   - `RackSteerMechanism`
   - `LoadKpis`

## Claude 开始前建议先读这些文件

按顺序：

1. `docs/load_analysis_handoff.md`
2. `frontend/src/components/LoadAnalysisPage.tsx`
3. `backend/src/sim4wis/vehicle/load_analysis.py`
4. `backend/src/sim4wis/vehicle/geometry.py`
5. `backend/src/sim4wis/core/state.py`
6. `backend/tests/test_load_analysis.py`
7. `vehicle_profiles/LS9.yaml`

## 快速 sanity checklist

改完后至少跑：

```bash
cd frontend && npm run type-check && npm run build
cd ../backend && python3 -m pytest tests/test_load_analysis.py -q
cd .. && python3 scripts/smoke_test.py
```

如果动到 VehicleParams/profile/project schema，再跑全量：

```bash
cd backend && python3 -m pytest tests/ -q
```
