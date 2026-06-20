# Changelog

本项目版本约定：阶段即次版本（Phase 1 = 0.1，Phase 2 = 0.2，Phase 3 = 0.3，改进轮 = 0.4 起）。

## 0.8.1 — 2026-06-20（单轮分析口径开关 · body_coupling）

### 背景

v0.8.0 把单轮 sweep 升级到 bicycle 耦合（α = β + r·x/V − δ）后，工程师反馈
**少了一档**：bicycle 口径对"驾驶手感、δ_eq 工程预测"很对，但选作动器/电机
最差工况时，工程师想知道"无论车身怎么响应，单轮自己一定要扛住多少力"——
这就是 v0.7.x 之前的"单轮台架（α = −δ）"口径。
本版把两种口径做成开关同时提供。

### 新增 (Added)

- **`body_coupling` 参数**贯穿 `model_core.steady_state_slip_angles` → `load_analysis.sweep_load_analysis` / `sweep_sensitivity` → `/api/load-analysis/sweep` / `/api/load-analysis/sensitivity` / `/api/model/demo/kingpin-breakdown`，
  取值 `"vehicle"`（默认，v0.8.0 的 bicycle 耦合）或 `"isolated"`（单轮台架，α = −δ）。
  两种口径下轮自身物理（载荷敏感 c_α(F_z)、气动升力、驱动力 F_x、camber thrust、toe、parking）
  **全部照算**，只有车身响应是否生效的差别。
- **负载页"受力口径"分段开关**：顶栏新增 `[整车装载][单轮台架]` 切换，所有曲线/KPI/敏感度即刻按所选口径重算。
- **数学模型讲解页第 5 章和第 8 章**补充两种口径用途说明 + 主销力矩分解 demo 也加同一开关。

### 修复 (Fixed)

- 负载页 `chartSignal` 改用结果里的 `body_coupling` 而非 UI 状态，避免数据回来前 uPlot
  错把旧模式数据当新模式渲染（KPI 已更新但曲线仍展示旧口径的状态机问题）。
- `LoadSweepResponse` 类型补 `body_coupling` 字段，去掉前端 `as any`。

### 测试

- 后端新增 6 个 body_coupling 回归（isolated δ_eq(v) 几乎恒定、linear 斜率不随 v 增长、
  δ=0 处两口径残余力完全一致、δ=0 sanity、sweep 端点收 body_coupling、result 携带字段）。
- 全量 167 passed。

## 0.8.0 — 2026-06-20（统一底座 · bicycle 耦合 · 模型讲解页）

### 背景

v0.7.3 给负载页加完物理项后，做了一次完整的底盘动力学 review，发现两个根本性问题：
(a) **物理散落**——同一份物理（轮胎力、kingpin 力矩、载荷转移、bicycle 耦合）在
时域 3 个模型 + 准静态负载页 + 控制器里各写一遍，会漂；
(b) **单轮 sweep 用 α = −δ 把速度约掉了**，看不到高速线性区收缩、斜率随 v² 增长的关键 4WIS 行为。
本版同时解决：把底座统一到 `model_core`，把 α 升级为真实的 bicycle 耦合，并新建一个由浅入深的数学模型讲解页。

### 新增 (Added)

- **统一物理底座** `vehicle/model_core.py`：唯一共享数学层（轮心速度合成、轮坐标变换、
  滑移角/滑移率、稳态 bicycle 解 `solve_steady_state_body`、载荷敏感刚度
  `load_sensitive_cornering_stiffness`、气动升力、对齐、parking）。**不依赖** simulator/
  controller/API/前端，被 `load_analysis` / `dynamic` / `multibody` 共用。
- **统一轮胎力内核** `tire.pacejka_combined_forces`：唯一的 Pacejka pure-slip + friction
  ellipse 实现，准静态和时域共用。
- **kingpin 力矩四项分解** `kingpin.kingpin_torque_terms`：原 `kingpin_torque` 改为求和，
  保证单一来源；让讲解页能展示 Fy·拖距 / Fx·偏置 / Mz / KPI 各项随 δ 的贡献。
- **模型注册中心** `vehicle/model_registry.py`：把模型收敛为 primary 两层（运动学/动力学）
  + research 层（多体 14DOF 保留作回归参考）。新增 `/api/meta/model` 暴露注册信息。
- **稳态 bicycle 耦合（W1，关键物理升级）**：单轮 sweep 不再假设车身锁直行。
  对每个 (速度, δ) 解 2×2 bicycle 方程 (β, r) → 反推每轮真实 α。对 LS9 对称底盘：
  `α_FL ≈ -δ·(1/2 + mV²/(8 C_α L))`，斜率随 v² 增长，饱和发生点随车速收缩——
  这是 4WIS 工程师的常识，台架口径模型把它藏起来了。
- **轮胎载荷敏感刚度 (U2/M1)**：`c_α(F_z) = c_α0·(F_z/F_z,nom)^p`，p 默认 0.8。
  高速气动升力降 F_z → 线性区斜率也跟着软化，不是定值。
- **数学模型讲解页**（新顶栏 Tab「数学模型」）：
  - 单流逐步加深，10 章 + 附录，每章三段式：直觉（白话，给 PM/领导）→ 关键公式（KaTeX）→
    可展开的推导（教科书级，工程师可引用）；
  - 9 张教学 SVG（车-轴-轮层次、坐标系、滑移角、轮胎曲线、摩擦椭圆、载荷转移、
    主销侧/俯视、bicycle 模型、齿条机构、力链路）；
  - 3 个交互演示（拖滑块实时调后端真模型）：
    bicycle 滑移增益 vs 车速、轮胎 Fy-α vs Fz/μ、主销力矩四项分解 vs δ；
  - 内容数据抽到 `frontend/src/components/model/modelChapters.tsx`，
    SVG 在 `diagrams.tsx`，交互演示在 `demos.tsx`，主页面只做编排。
- **只读教学端点** `POST /api/model/demo/{bicycle-gain,tire-curve,kingpin-breakdown}`：
  讲解页交互直接复用真底座（`model_core` / `tire` / `kingpin` / `load_analysis`），
  不在前端重算公式。

### 修复 (Fixed / 物理修正)

- **U1 删除几何站不住的 caster 交叉项**：`kingpin_torque` 里 `−Fx · trail · sin(δ)` 是
  Fx ∥ trail 的叉乘=0，不是真物理项。已删除并加 Reimpell 等效力臂的注释说明。
- **U3 Pacejka 死代码**：v0.7.3 切到 Pacejka 后却把结果扔掉，主图仍是线性硬剪。
  现在改成等效滑移法 (`α_eq = α − C_γ γ F_z / c_α`、`κ_eq = F_x_drive / c_κ`)
  一次 Pacejka pure-slip + friction ellipse 成型，主图蓝线终于平滑饱和。
- **气动升力默认值** `aero_lift_coeff_front/rear` 提到 0.30/0.15，让车速依赖在 UI 上明显。

### 变更 (Changed / 工程边界与解耦)

- **控制器、API、前端不持有物理**：控制器只输出 δ/ω；API router 只做请求/响应适配；
  前端图表只消费后端结果。讲解页第 10 章把这些边界写明。
- **参数瘦身**（中度）：分核心 / 高级两组（`frontend/src/vehicle/parameterGroups.ts`），
  讲解页附录列两组对比。
- **CSS 扩展** `model-*` 系列：章节卡、KaTeX 容器、教学 SVG 配色、交互演示卡。

### 测试 (Tests)

- 后端测试 144 → 161 passed：U1 caster 删项、U2 载荷敏感斜率、U3 Pacejka 平滑肩部、
  W1 bicycle 增益随车速、W1 饱和点收缩、3 个 model_demo 端点 sanity + round-trip。
- smoke 32/32 passed。

### 内部 (Internal)

- 全部 squash 到 baseline commit；本版按 plan 拆为 2 个 commit 推 `main`：
  ①教学端点 + kingpin 分项 helper + 测试（后端）、②讲解页重做（前端）。
- 仓库：`nortejiang-tech/4WIS-Simulator`。

## 0.7.3 — 2026-06-18（负载特性页：物理补完 · δ_eq 敏感度 · 组件拆分）

### 背景

在 v0.7.2 引入的「负载特性」分析页里，原 sweep 模型在 δ_cmd=0 处恒等给出 `τ=0 / F_rack=0`，
即"直行位 = 电机零输出位"。用户反馈：传统底盘下这两个位置在工程上**不一定重合**，
不同车速下的"电机零输出转角"应该是会漂移的。本版补完了让 δ_eq(v) 真正能漂移的物理项，
并新增敏感度面板让工程师能定量看到每个旋钮的影响。

### 新增 (Added)

- **驱动力 / 气动 (A1)**：sweep 加 `Fx_total = Crr·m·g + ½ρ·Cd·A·v²`，按四轮均分注入
  主销 scrub_radius 力矩。新参数 `rolling_resistance_coeff`、`drag_coeff_cd`、
  `frontal_area`、`air_density`，LS9 默认 Crr=0.012、Cd=0.30、A=2.80 m²、ρ=1.225。
- **Camber thrust (A2)**：`Fy_camber = Cγ·γ_per_wheel·Fz` 加在 α=0 处。新参数
  `camber_thrust_coeff`（默认 1.0/rad）。左右镜像对称取消，但单轮非零。
- **静态 toe (A3)**：per-axle `static_toe_front` / `static_toe_rear`，前后独立暴露，
  per-wheel 镜像；LS9 默认前轴 0.1°、后轴 0°（4WIS 后轴常用零 toe）。
- **kingpin 力矩补项 (A4)**：`kingpin_torque` 新增 `Fx · r·tan(caster) · sin(δ)` 交叉项
  ——驱动力经主销后倾的二阶耦合。
- **后端 δ_eq summary (A5)**：`/api/load-analysis/sweep` 的 summary 新增
  `per_speed_equilibrium: [{speed, delta_eq, delta_eq_deg, rack_at_zero, torque_at_zero,
  source, found}]`。前端不再做客户端零点搜索。
- **δ_eq 敏感度面板 (E)**：固定剖面车速下，扫描某一个底盘参数（Camber thrust Cγ、
  前/后轴 toe、Crr、Cd、迎风面积、外倾 γ、主销偏置 scrub、主销后倾 ε）的范围，
  画出 `δ_eq 随该参数变化`曲线。每次参数表改动后自动重扫，summary 行给出 δ_eq 跨度和
  对应 rack@0 极差。新增 `POST /api/load-analysis/sensitivity` 端点。
- **齿条→δ 反解后端化 (B3)**：把前端的 `solveRackToWheel` 迁到后端
  `vehicle.geometry.wheel_angle_from_rack_travel()` 共享函数 +
  `POST /api/load-analysis/rack-solve` 端点。带单测：零位、单调、对称镜像、夹限。
- **静态接地斑系数拆分 (B2)**：`parking_scrub_coeff` 拆成 `parking_lateral_coeff` 和
  `parking_torque_coeff`，分别控制停车侧向力和停车阻力矩。旧字段自动 fallback 到新字段。
- **可调停车扭转饱和角 (B1)**：`STATIC_TIRE_DEFLECTION_RAD = 8°` 改为 VehicleParams
  `static_tire_deflection_deg` 字段。
- **重置 LS9 按钮 + 分组提示 (F)**：参数表加「重置 LS9」一键回出厂默认；新增的
  「对齐 / 偏置」和「驱动 / 气动」两个分组下加简短说明，告诉用户哪些项一起清零
  会让 δ_eq 回到 0°。
- **CSV 导出加 δ_eq 列**：导出文件名带时间戳；每行附带当前车速对应的 `delta_eq_deg`。

### 变更 (Changed)

- **`LoadAnalysisPage.tsx` 拆组件**：从 1060 行降到 ~290 行（仅编排），新建
  `components/load/` 目录：`types.ts`、`ChartBox`、`RackSteerMechanism`、`useLoadSweep`、
  `ProfileToolbar`、`LoadParamsEditor`、`LoadControls`、`LoadKpis`、`LoadLivePanel`、
  `SensitivityPanel`。
- **主图标题紧凑**：`τ (Nm) · FL · 30km/h` 之类的紧凑格式，省横向空间。
- **图表 chartSignal 加 paramsHash**：任何参数改动都触发图重建，避免编辑参数后老数据残留。
- **空 KPI 显示 `—`**：sweep 还没出结果时不会误导成 `0`。
- **`load_analysis` sweep 内部 tire model 缓存**：以前每对 (speed, angle) 重建一次
  `make_tire()`，现在按 sweep 整体缓存一次；大量参数表交互下肉眼可感的提速。

### 修复 (Fixed)

- 之前"对称模型下 δ_eq=0"实际是因为 sweep 把驱动力 / camber thrust / toe / 4WIS 耦合
  全部忽略，导致 `τ(δ=0) ≡ 0` 是模型恒等式，不是物理结论。现在每一项都进入计算口径，
  LS9 默认下 δ_eq 从 0° 漂到 ~0.16°，rack@0 在 92~142 N 间随车速变化——
  与"电机要持续输出扭矩维持直行位"的工程直觉一致。

### 测试 (Tests)

- `backend/tests/test_load_analysis.py` 新增 11 个用例：static toe → 非零 rack@0、
  camber thrust → 非零 rack@0、Fx_drive 随 v² 增长、per_speed_equilibrium 跨车速漂移、
  反解零位 / 往返一致 / 夹限、参数 codec 向后兼容、敏感度 API 单调响应、敏感度端点、
  rack-solve 端点。
- 全量后端 `128 → 139 passed`；smoke 32/32 passed。

## 0.7.2 — 2026-06-14（小镇路网场景 · 仿士瓦本格明德 + 3D 立体建筑）

### 变更 (Changed)
- **「城市路口」场景升级为「小镇 · 士瓦本格明德」路网**：原单一四向路口过于简单，替换为
  仿德国 Schwäbisch Gmünd 老城特征的拥挤小镇路网（场景 key `city` → `town`）——
  椭圆环路（旧城墙线）+ 密集内部街网（主轴市集街 / 南北平行街 / 5 条横街 + 2 条斜巷）形成
  不规则街区，中央集市广场（Marktplatz）+ 教堂广场（Münster / Johannis）+ 约纳河（Josefsbach）
  小河，沿街密集排布建筑（仅环路内侧，"城墙内"紧凑），主要路口设红绿灯。仅为特征近似，非测绘精度。

### 新增 (Added)
- **3D 立体建筑**：场景 `building` / `landmark` 面用 `ExtrudeGeometry` 拉伸为有高度的楼体
  （高度按footprint质心确定性伪随机 → 错落屋脊，教堂更高），投/受阴影；`water` 面（小河）下沉渲染。
  后端 `scenario.py` 新增开放折线偏移 `_offset_polyline`、街道带 `_road`、河流 `_water`、
  沿街建筑布放 `_buildings_along`（按点到其它路中心线距离自动避让，不压路面/广场）等几何工具。
- 2D 建筑/地标加描边以区分相邻房屋。

## 0.7.1 — 2026-06-14（后轮转向归并 + 三模式稳定性 + ICR 修复 + 标准场景路况）

### 变更 (Changed)
- **后轮转向归并为单一策略 `rear_wheel_steer`**：原 4 个独立 `rws_*` 策略 + 旧 `rear_steer`
  合并为一个「后轮转向」策略，控制策略下方出现**子模式下拉**（定比 / 车速调度 / 横摆反馈 /
  稳态+瞬态 / 模型跟踪）+ 增益说明，经 `mode_params.rws_mode` 切换；设计器 k(vx) 改为驱动它。

### 修复 (Fixed)
- **三个反馈模式（横摆反馈/稳态+瞬态/模型跟踪）在运动学模型上后轮剧烈摆动**：根因是运动学
  瞬时实现指令横摆 → `δr=g2·yaw` 与 `yaw=K·δr` 构成**无阻尼代数环**，环增益>1 时 ±限幅振荡
  （实测 ±33°）。修复：对反馈信号（yaw_rate、转角变化率）加一阶低通（τ=0.08s）+ 下调增益
  （g2 0.2→0.12、g_yaw 0.1→0.08），三档模型均稳定；新增运动学稳定性回归测试。
- **ICR 拖拽：瞬心在右侧时车轮方向/姿态错误**：`anglesFromICR` 用的 atan2 默认 CCW（瞬心在左），
  瞬心在右时取到反向垂线（≈±162°）被限幅到 ±60°。改为把角度折叠到轮线区间 (−90°,90°]，左右对称正确。

### 新增 (Added)
- **标准场景路况（2D + 3D，静态只渲染一次，不影响帧率）**：广场 / 城市路口（中国标准车道线、
  双黄线、虚线分道、停止线、斑马线、四向红绿灯）/ 小赛道 / 上海国际赛车场（近似）。
  后端 `environment/scenario.py`（surface/line/marker 三原语 + 中心线偏移生成赛道边线）、
  REST `GET /api/scenarios`、`GET /api/scenario`、`POST /api/scenarios/{name}/load`（加载即把车
  传送到场景起点）；前端 `ScenarioPanel`（场景页）+ 2D `ScenarioLayer`（Konva）+ 3D `Scenario3D`
  （ShapeGeometry 路面 / 条带标线 / 红绿灯，`useMemo` 一次构建）。

## 0.7.0 — 2026-06-13（后轮转向控制方法族 + 界面分组 Tab + 模块内嵌帮助 + 3D 修复）

### 新增 (Added)
- **后轮转向（RWS）控制方法族**（4 个新策略，LS9 标定默认增益，研究见
  `docs/rws_control_methods_research.md`）：
  - `rws_speed_schedule` ① 车速调度比例 k(vx)——低速反相、高速同相，曲线可在策略设计器编辑；
  - `rws_yaw_feedback` ③ 横摆角速度反馈 δr=g1·δf+g2·ψ̇（闭环）；
  - `rws_transient` ② 稳态(零侧偏调度)+瞬态(前轮角变化率)前馈，快打先反相；
  - `rws_model_following` ④ 零侧偏前馈 + 横摆参考跟踪反馈。
  共享 `rws_common.py`（零侧偏解析比 k(vx)、参考横摆、轴角→指令几何），pytest +8。
- **策略设计器新增 “k(vx) 调度” 模式**：可视化编辑后轮/前轮比随车速曲线，经
  `mode_params.k_curve` 实时下发给 `rws_speed_schedule` 策略；含当前车速标线、默认曲线一键恢复。
- **每模块内嵌帮助**：新增 `Panel` 外壳（标题 + 折叠 + ⓘ 帮助弹出）+ `ui/help.tsx` 帮助注册表，
  22 个面板全部内置使用说明。

### 变更 (Changed)
- **主界面侧栏重构为 5 个分组 Tab**（驾驶 / 设计 / 验证 / 场景 / 数据），面板保持挂载
  （`display` 切换，计时器/WS 订阅不中断），取代原先的纵向长列表。

### 修复 (Fixed)
- **3D 车轮自转方向反了**：前进时角速度应指向 −Z，`Canvas3D` 改 `rotation.z = -spin`。

### 调参 (Tuning)
- RWS 默认增益经 `scripts/tune_rws.py` 阶跃工况(30/70/120 km/h, simplified_dynamic)整定：
  `rws_yaw_feedback` g2 0.35→**0.20**、`rws_model_following` g_yaw 0.20→**0.10**、
  `rws_transient` c=0.12、① 默认曲线=解析零侧偏比。
  结果(120 km/h 阶跃)：前轮基准侧偏 −21.6°、恒定反相 −44°，四种 RWS 均 **侧偏≈0°**（vy 峰 50→0.4 km/h），全工况稳定。

## 0.6.0 — 2026-06-13（策略设计与验证套件 + 力链/故障/插件 + km/h·定速·测量）

> 说明：0.5.0 的力链/故障/插件批次曾在代码里 bump 但从未打包发布；本次与策略
> 设计验证套件合并为 0.6.0 一次发布。上一个真正发布的版本是 0.4.0。

### 新增 (Added)
- **策略设计器（所见即所得，无需写文件/代码）**：侧栏「策略设计器」两种模式——
  ① 曲线映射：四轮各一条分段线性曲线（X=转向输入 −1…+1，Y=轮角 ±60°，浅带标 ±35° 限位），
  点击加控制点/拖动调整/右键删除，内置 阿克曼·蟹行·后轮反向·零半径 预设；
  ② ICR 拖拽：在车辆俯视图上拖动瞬心，四轮转角按几何实时反解。
  结果经现有 `user_js` 策略实时驱动仿真（每帧 WebSocket 回传四轮角）。
- **开环激励测试**：标准化驾驶员输入序列——角阶跃 / 单频正弦 / 正弦扫频(chirp) /
  双移线(ISO-3888)，可设幅值/频率/时长/目标车速，定时器 60 Hz 驱动，油门定速保持。
- **策略评分（量化验证）**：从历史缓冲实时计算 7 项指标——瞬心偏差峰值/RMS、横摆角速度峰值、
  侧向速度峰值、转向能耗代理、齿条力峰值、侧偏角峰值，支持「捕获为 A/B」并排对比+自动高亮更优方。
- **分体齿条力链**：主销力矩 → 拉杆力 → 齿条轴向力 → 电机需求力矩
  （`τ_motor = τ_kingpin·cosβ/L_arm · r_p/(i·η)`），参数面板新增 5 个传动参数，
  新增「齿条力」「电机需求力矩」两张实时曲线，CSV 新增 8 通道。
- **故障注入框架（ISO 26262 功能安全验证）**：6 种故障——电机卡死(0)/卡死(指定角)/限幅、
  传感器偏置/噪声/丢帧；侧栏面板增删改+启停，REST `GET/POST/PATCH/DELETE /api/faults`；
  执行器故障改实际转角、传感器故障只改上报值（不动物理真值）。
- **Python 热重载策略插件**：编辑 `plugins/strategies/user_strategy.py` 保存后 ~1 s 自动加载，
  侧栏显示状态/错误，无需重启；`compute(driver, state) -> {'delta_cmd': [...]}`。
- **浏览器 JS 策略沙箱**：侧栏直接写 `compute(driver, state)`，实时编译并在浏览器执行、
  结果经 WebSocket 回传，4 个示例预设，localStorage 持久化（沙箱在浏览器侧，后端零 eval）。
- **轮胎侧偏角 α 可视化**：第 7 张实时曲线（仅动力学模型非零）。
- **A/B 对比扩展多通道**：除车速外可叠加 横摆角速度 / 瞬心偏差峰值 / 电机力矩合计。
- **定速巡航**：「驾驶输入」面板手动输入固定车速（km/h）并保持，覆盖 W/S，转向仍可手动/策略控制。
- **测量工具**：卷尺（2D 画布点两点测直线距离，画布标注）+ 轨迹自动尺寸（X/Y 跨度、路径总长、直线位移）。

### 变更 (Changed)
- **车速单位全面改为 km/h**：2D/3D HUD、控制/激励/轨迹/参数面板、车速曲线、A/B 车速通道、
  侧向速度评分均改 km/h（内部仍 m/s，仅显示/输入边界换算）。
- 集成测试：pytest 77 → 112（力链 18、故障注入+插件 17），新增前端单位/评分/测量逻辑。

### 修复 (Fixed)
- **开环激励油门误用比例控制器**：`0.35·(目标−vx)` 在 vx≈0 时饱和到 throttle=1，
  而模型中 `throttle·v_max = 目标速度`，于是实际命令到 v_max(≈200 km/h)，温和正弦也甩尾——
  一度误判为 simplified_dynamic「低速数值发散」。改为开环 `throttle = 目标速度/v_max`；
  复测 5 km/h 阶跃侧偏角 2.7°、侧向速度 0.4 km/h，模型本身稳定，后端未改。
- **评分/测量面板不刷新**：`history`/`trajectory` 缓冲为原地 mutate（数组引用不变），
  zustand 选择器不重渲染——改为 250 ms 定时器从 `getState()` 重算。

### 已知限制
- 步 18（WebHID/Gamepad 方向盘手柄）仍未实现（待硬件）。
- Windows 便携包仍为 Mac 交叉组装，未在真实 Windows 上验证。

## 0.4.0 — 2026-06-12（全面改进：需求补全 + 物理保真 + 工程质量 + UX）

### 新增 (Added)
- **每轮转向中心（需求补全）**：定义为整车实际瞬心在各轮垂线上的正交投影点 +
  带符号偏差（理想阿克曼下严格共点，偏差≡0；其它策略/滑移下量化该轮转向几何与
  整车运动的不一致）。后端 60Hz 推送、2D 画布按轮配色菱形标记+偏差线段（>0.3 m 变红）、
  HUD「ICR偏差」行、第四张实时曲线、recorder 新增 `icr_dev_*` 与 `icr_x/y_*` 通道。
- **扰动画布 GUI 编辑（需求补全）**：新面板选类型→「放置」→点击画布连续放置（Esc 退出）；
  点选区域→表单编辑/画布拖动；删除/清空。REST 新增
  `GET /api/scene`、`POST/PUT/DELETE /api/scene/disturbances`、`POST /api/scene/clear`。
- **车辆/悬架参数编辑面板（需求补全）**：几何/质量惯量/转向伺服/悬架几何/弹簧阻尼/轮胎
  分组编辑，应用即生效（重建模型），非法值显示后端 422 详情。
- **Pacejka 轮胎模型**：四参数简化 Magic Formula（B 由刚度反推保证小滑移区与 linear
  一致、μ 进 D 项与扰动机制零改动兼容、摩擦椭圆联合工况、滑移衰减气胎拖距）。
  `tire_model: linear | pacejka` 项目级/界面可切。回归：μ=0.9 稳态转向 30 s
  max|vy| 7.8 → 0.42 m/s。
- **pure-pursuit 曲率限速**：制动距离预瞄窗内扫描最大路径曲率，v ≤ √(a_y,max/|κ|)；
  Ld/ay_max/ax_brake 等全部可经 mode_params 配置。
- **图表导出**：四张实时曲线均可一键导出 PNG / CSV（当前 30 s 窗口）。
- 后端单元测试 23 → 77（tire/load_transfer/kingpin/wheel_servo/wheel_icr/
  follow_trajectory/params 校验/scene CRUD）；smoke 28 → 32 项。

### 变更 (Changed)
- **rest.py（472 行 36 端点）拆分为 `api/routers/` 按资源分文件**，URL 全部不变；
  参数合并逻辑由 13 个三元表达式收敛为 `ParamsUpdate.merged()`。
- **Canvas2D（586 行）按层拆分**：projection/colors/DisturbanceLayer/IcrLayer/Hud。
- **/api/params 全字段化 + 物理边界校验**（pydantic Field + cg_to_front<wheelbase 交叉检查）。
- 硬编码参数配置化：servo kp/ki、kinematic kingpin_mu、轮胎刚度、pure-pursuit Ld 系列。
- 前端共享基建：`fetchJSON`（8 s 超时+错误 detail 透传）、共享样式、uPlot 工厂。
- WebSocket：前端重连固定 1 s → 指数退避（1→15 s+抖动）；5 s 心跳 ping/pong 检测半开连接；
  后端一条畸形帧不再杀整条连接。
- 2D 缩放与 3D 相机位姿提升至全局状态——2D/3D 切换不再丢视角。
- 策略热切换时清零四轮速度伺服积分（消除切换扭矩跳变）。

### 修复 (Fixed)
- **`vehicle_icr_from_velocity` 符号错误**：实际瞬心（红点）此前镜像显示在车辆另一侧
  （正确公式 x_R=−vy/ω, y_R=+vx/ω）。smoke 仅验证了策略侧垂线交点，未覆盖该函数。
- **保存项目丢失 scene**：`ProjectFile.from_runtime` 此前不持久化扰动，
  「编辑扰动→保存项目→重载」闭环不成立。
- 三个过期 pytest（registry 5 策略、对踵轮对称断言、zero_radius 在 LS9 ±35° 限幅下不可达）。
- 全局 ErrorBoundary（视口/侧栏分隔，3D 崩溃不连坐）+ Toast 错误提示。

### 已知限制
- 步 18（WebHID/Gamepad 方向盘手柄）仍未实现（待硬件，本轮明确暂缓）。
- Windows 便携包仍为 Mac 交叉组装，未在真实 Windows 上验证。

## 0.3.0 — 2026-05-25（Phase 3 收尾发布）

Phase 3 主体（3D 可视化、轨迹编辑、多体动力学）+ 一轮集成测试改进（P0/P1/P2）
+ 用户追加需求 + 一批使用体验修复 + 便携一键启动打包。

### 新增 (Added)
- **3D 可视化（步 16）**：Three.js / react-three-fiber 视图，与 2D 并列、右上角切换；
  车身/座舱/可转向+自转车轮、地面网格、扰动区域、参考路径+桩、轨迹拖尾、ICR 标记、
  跟随相机；three.js 代码分割按需加载。
- **轨迹编辑器 + 纯追踪（步 17）**：标准工况生成器（直线/圆弧/绕桩/双移线/八字/停车），
  自带地面桩；`follow_trajectory` pure-pursuit 跟踪策略；画布点击放航点 + Catmull-Rom 平滑。
- **多体动力学模型（步 19）**：14 DOF（车身 6 + 悬架 4 + 车轮 4），每角 quarter-car，
  动态载荷转移、真实侧倾/俯仰、悬架行程驱动的 bump-steer。~45× 实时 @200Hz。
  前端可在 运动学 / 简化动力学 / 多体 间切换，3D 车身随侧倾/俯仰倾斜。
- **A/B 双跑对比**：保存两次行驶轨迹叠加 + 车速曲线对比。
- **路面摩擦设置**：下拉预设（水泥 0.85 / 干沥青 0.9 / 雨天 0.55 / 雪面 0.3 / 冰面 0.12）+ 手动微调；
  每轮有效 μ 在 HUD/画布可视化。
- **键盘保持车速模式** + **转向回正速度滑块**（0=保持不回正 ··· 1.5× 最快）。
- **转向作动器**：一阶滞后 + 角速率限幅（不再瞬时转向）。
- **深色 / 浅色主题**切换（含 2D/3D 画布配色），localStorage 持久化。
- **便携一键启动包**：内嵌 Python 3.12，双击 `start.command`/`start.bat` 即用，
  无需安装 Python/Node；参数（projects/scripts_lib/plugins）外置可改。

### 变更 (Changed)
- **默认车辆标定为智己 LS9**（轴距 3.16 m、整备质量 2900 kg、285/45R21、v_max 55.6 m/s 等；
  未公开项为工程估算）。
- **后端默认端口 8000 → 8010**（避开本机 LLM 推理服务）。
- **摩擦系数改为绝对值语义**：split-μ 的 `mu_left/mu_right`、ice_patch 的 `mu` 不再是 base_mu 乘子。
- **基础路面 μ 默认 1.0 → 0.85**（干水泥）。
- 轮胎侧偏刚度按 LS9 标定（c_alpha 70k→120k）。
- 生产部署改为**后端单进程托管前端静态**（无需 Vite/代理）。
- **2D 视图初始朝向**：前进方向由"向右"改为"向上"。
- **3D 车轮安装方向**修正（横向轴、沿前进滚动）。

### 修复 (Fixed)
- 含扰动的演示项目误用运动学模型（split-μ 演示不出 μ 效果）。
- 减速带 fz 脉冲过大（50 kN）致积分器发散/停车 → 量级修正 + clamp。
- kingpin 转向阻力矩 τ 不随转角单调（KPI 居中项错为常量，改为 ∝ sin δ）。

### 已知限制
- 步 18（WebHID 方向盘/手柄）未实现（需硬件）。
- Windows 便携包为 Mac 交叉组装，未在真实 Windows 上验证。
- 动力学为线性轮胎；极限工况不够真实（Pacejka 待后续）。

## 0.2.0 — 2026-05-24（Phase 2）
Recording/CSV、μ 扰动、YAML 动作脚本、简化动力学模型、悬架阻力矩、减速带/斜坡、
FMU/MATLAB 适配器、集成测试（smoke 24 项）。详见 docs/phase2_plan.md。

## 0.1.0 — 2026-05-23（Phase 1）
运动学 4WIS 模型、5 种控制策略（含理想阿克曼）、WebSocket 状态流、2D 可视化、
YAML 项目文件。详见 docs/design.md。
