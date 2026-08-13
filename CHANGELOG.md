# Changelog

本项目版本约定：阶段即次版本（Phase 1 = 0.1，Phase 2 = 0.2，Phase 3 = 0.3，改进轮 = 0.4 起）。

> 0.100.0 起每个版本另有一份详细变更说明在 `docs/v<版本>_changelog.md`，本文件只留摘要。

## 未发布 — v2 进行中（研究层 · 转向系统被控对象层）

### 新增 (Added)

- **MCP 服务器（S3）——服务发货**：`sim4wis-mcp` 独立 stdio 进程（薄 HTTP 客户端，
  不 import sim4wis，与后端各自演进）上线，8 个工具：describe_capabilities /
  run_study（dry_run 旗标；真跑阻塞轮询到出结果）/ get_study / list_studies /
  list_runs / get_trace（显式降采样）/ read_report（回本地路径，不塞 HTML）/
  verify_golden（子进程跑黄金门禁）。后端不在时自动拉起、退出时只收自己拉起的。
  验收流（新建 spec → dry-run → run → 读报告）已做成测试，含真实 stdio
  握手端到端。设计文档 §10 的 compare 并入 study 结果、realtime 属 P4 租约，
  均未发货（如实注明，不伪装）。依赖新增 `mcp>=2.0`，控制台入口 `sim4wis-mcp`。
- **统一实验体系（S1）**：黄金实验不再是平行体系——`step_steer_60kmh` 与
  `iso3888_dlc_60kmh` 两个 KPI 黄金移植为 procedures 模板（工况钉死 + criteria +
  报告 + 出处字段），`check_golden_experiments.py` 改为经 study runner 跑全部
  procedure 黄金（KPI 值逐位不变）。带 `library` 孪生文件的 procedure 会被
  **twin check** 校验与 GUI 实验库条目逐字段一致——两处工况漂移即红（实测篡改
  库文件立刻报错）。单轮失效 3 样本仍走研究脚本（其安全指标不在 study 指标
  注册表内，待 study 层扩展）。
- **摩擦的频域分离协议（C4 / D2）**：0.2 Hz weave 下迟滞环宽对齿条摩擦不单调，
  是因为车辆侧向动力学滞后贡献同量级正交分量。新增 `procedures/friction_slow_ramp.yaml`
  —— 0.02 Hz、50 km/h 慢斜坡：动力学分量消失后，环宽对齿条库仑摩擦**单调**
  （0/80/260/900 N → 0.29/0.96/1.12/1.71 N·m；死区 0.06→2.28→4.78°）。
  `test_the_loop_width_is_monotone_in_rack_friction_at_the_slow_ramp` 钉住该单调性；
  `steering_feel@2 → @3`：迟滞环宽与死区两条**从参考项升为判定项**，判定工况为
  `freq_hz == 0.02`（0.2 Hz 上仍只记录不判定，那里的数不承载摩擦含义）。
- **v2 黄金锚点**：`check_golden_experiments.py` 现在除了 KPI 黄金之外还跑
  `procedures/iso13674_oncentre.yaml`（100 km/h、0.2 Hz、前轮 0.4°、转向层开启、
  50 Hz 记录），把 ISO 13674 中心区 weave 的 10 个过程指标
  （力矩梯度 ×2、0.1 g 手力矩、迟滞环宽、死区、转向灵敏度、横摆相位滞后、
  实测幅值/频率）锁进 `docs/golden_experiments.json` —— 改动 `plant.py` /
  `oncentre.py` 的参数或算法即红（容差按实测篡改探针标定：扭杆 +10% 与
  拟合窗口 0.25→0.30 都能被抓到），耗时增量 ~4 s。
- **study 层**：`StudySpec` 声明式研究单元（网格 sweep → 指标 → 跨 run 对比 → 判据
  → 报告），`/api/study/*` 契约 + `sim4wis study` CLI + 出处字段（git sha / params_hash）。
  表达式指标档带 AST 白名单沙箱。见 `docs/study_guide.md`。
- **转向系统被控对象层**：转向柱/扭杆/扭矩传感器、助力曲线（可标定的表）、
  电机（峰值 + 反电动势包络 + 热降额）、齿条摩擦与柔度；线控前轴的角度跟踪与路感合成。
  **默认关闭**，关闭时两个时域模型逐位不变。见 `docs/steering_system_guide.md`。
- **八种转向架构**成为一等配置（C/P/DP/R-EPS · SBW · EPS+RWS · SBW+RWS · 4WIS），
  能力与失效模式按架构裁决；电机减速比归属架构而非车辆。
- **适用边界声明**进入产品界面与每份报告，而不是只写在文档里。

- **作动器选型包络**：四个最恶劣工况 → 峰值转矩/转速/RMS/热 + 判定；
  负载取自既有 `sweep_load_analysis`；驾驶员力矩到上限的工况被标记并拒绝给出余量。
- **目标层**：可版本化的需求集（`targets/`，带 `name@version`、归口与必填出处）
  + 符合性表，回答"这套转向配置达标了吗"。四种判定 **达标 / 边际 / 超标 / 未评估**，
  覆盖率是一等输出 —— 没测到永远不会读成通过。`sim4wis targets list|show|check`、
  `/api/targets/*`，study spec 可用 `targets:` 引用。见 `docs/targets_guide.md`。
  内置两套：`eps_actuator`（今天就能判）与 `steering_feel`（ISO 13674 口径，
  客观试验库落地前故意报"覆盖不全"并点名缺哪个测量）。

- **客观试验与指标库（v2 V3 前半）**：新的**过程指标**档 —— 既不是每次运行都算的
  KPI，也不是一行表达式，而是只对某类工况有定义、需要整条曲线才能提取的量。
  首个落地的是 ISO 13674-1 中心区 weave 分析（`study/oncentre.py`）：力矩梯度
  （角度域与 N·m/g 两种）、0.1 g 手力矩、迟滞环宽、角度死区、转向灵敏度、
  横摆相位滞后，外加实际达到的试验工况。工况不对就**拒绝**而不是给个看着合理的数：
  没有被控对象层、不是 weave、周期不够、或已经跑到 0.35 g 以上（那是操纵试验，
  不是中心区试验 —— 0.84 g 时拟合出的力矩梯度是**负的**）。
  模板在 `procedures/iso13674_oncentre.yaml`，`sim4wis study run` 直接可跑。
- **前轴通道进入记录**：`steer_hand_torque` / `steer_hand_angle` / `steer_torque_sensor`
  等 8 条。该架构没有的信号记 **NaN 而不是 0** —— 线控车没有扭杆，记 0 会被下游
  当成"传感器读数为零"。

### 修复 (Fixed)

- **渲染平滑性机器验证（H1）**：3D 车顶视角的 6.7 Hz 拍频修复此前只有人眼验证。
  新增 e2e：注入 rAF 时间戳 → 脚本驱动行车（实测车在动，防假绿）→ 帧间隔序列
  在 40–300 ms 带内自相关峰值 < 0.35（合成节拍数据下该统计量为 1.00，白抖动
  0.10，判别力已验证）。
- **顺带修掉 H1 揪出的真缺陷**：KeyboardInput 的 rAF 循环以 50 Hz 推送空闲
  驾驶指令，脚本命令活不过 20 ms——GUI 开着时脚本根本开不了车。现在脚本运行
  期间驾驶通道归脚本（WS driver 消息被忽略），停脚本即交还；
  `test_script_driver_arbitration.py` 钉住该契约。
- **峰值手力矩欠解析约 1/3（D5）——转向层多速率集成**：指令在 5 ms 外步内
  零阶保持时，~10 Hz 转向柱模态的共振峰落在两个采样点之间被错过（峰值
  1.864 → 2.812 N·m，5 ms → 0.5 ms，51% 且未收敛；RMS 只动 7%）。现在转向层
  以 0.5 ms 内步率运行（10×），指令在步内**线性插值**而非保持（车辆外环
  仍是 5 ms，输出在边界对齐回外环）：0.5 ms 内步峰值与 0.25 ms 参考差 < 2%
  （实测 0.00%）。钉住“欠解析”的断言已反转并删除。选型模块同样改为多速率
  驱动 plant：\`parking_hand_torque\` 10.331 → 10.791 N·m（+4.5%，旧值欠解析，
  新值方向“更真实”；判定不变，仍是 FAIL），其余选型项变化 < 0.5%。
  黄金实验 5/5（转向层关闭）不受影响；\`test_disabled_plant_changes_nothing\`
  位级契约保持。
- **v2 黄金锚点按 C2 重锁**（评审过的方向）：中心区 weave 的迟滞环宽
  −11.6%、角度死区 −5.9%、横摆相位滞后 −4.8%，其余 7 项 < 0.4%。方向评审：
  5 ms 指令阶梯在迟滞环里注入量化涂抹，插值消除后环宽、死区、滞后全部
  收窄/缩短——是“少涂抹”而不是新现象；0.5 ms 与 0.25 ms 内步的指标差
  ≤0.7%，已在收敛。
- **边际带（D3）重跑**：2.0→12.0 N·m 电机扫掠在多速率下仍然单调、无振荡
  （5.5 N·m 历史振荡 85 000 rpm → 现 7 583 rpm 且驾驶员限扭接管），守卫
  行为不变（欠尺寸 → 拒绝给出余量）；该条已知项按“收敛”更新。
- **解析路径漏轴侧偏刚度分配（D1）**：`quasi_static_wheel_loads` 返回的载荷敏感
  c_α 不带轴分配（0.80 前 / 1.20 后），解析路径一直在解“另一台车”——稳态横摆
  −5% @30 km/h、−17% @60 km/h（幅值无关、随 v² 增长）。现将本构律单源化为
  `effective_cornering_stiffness`（载荷敏感 × 轴分配）：时域模型以滑移角缩放
  达到同值（Fy = −c_α·(s·α) ≡ −(s·c_α)·α），准静态路径直接用缩放后的刚度，
  两条路径不再有平行实现。**负载分析页与自行车增益 demo 的显示值随之变化
  5–17%，这是修复本身的证据，不是回归**。钉住缺陷的
  `test_analytic_path_omits_the_axle_cornering_split` 与 `with_axle_split`
  开关已删除，代之以 `test_quasi_static_bundle_applies_the_axle_cornering_split`。
  控制器 `rws_common.axle_cornering_stiffness()` 的同源第一例仍由历史研究
  `docs/reports/steering_decoupling_value.html` 记录（该报告不重跑）。
- **`SteeringPlantState.to_channels()` 从来没有调用者**：手力矩每步都算、每步都扔，
  所以在补上记录之前，任何中心区指标都无从算起。
- **Experiment schema 静默吞掉拼错的字段**：`frequency: 0.2`（字段其实叫 `freq_hz`）
  校验通过、按默认 0.5 Hz 跑完、给出一个漂亮的结果 —— 回答的是没人问过的问题。
  为此我在一个不存在的 26° 相位滞后上白找了半天缺陷。现已 `extra="forbid"`。
- **助力控制阻尼在对抗操作本身**：阻尼项按小齿轮**绝对**角速度给，
  于是转得越快助力越少 —— 常规泊车速率下吃掉 413 N·m 助力中的 195 N·m。
  被阻尼的模态是扭杆模态，其速度是扭转速率，现按扭转速率给；稳态转向时该项归零。
- **停车时轮胎负载被当成回正弹簧**：`sweep_load_analysis` 是准静态的
  （"保持这个角度需要多少力"），按角度定号等于凭空造出一根 400 N·m 的回正弹簧 ——
  满锁停车松手，模型会把方向盘甩回去。站立轮胎是**擦地**，负载耗散而非回正。
  现按滚动速度阈值拆成回正份额与擦地份额，擦地份额走齿条库仑摩擦。
  （选型模块此前取的是**绝对值**，那更糟：无论车轮朝哪边，负载都往同一个方向推。）
- **选型输入型线角加速度无穷大**：三角波瞬时换向，指令角速度在每个顶点跳变 2R，
  经驾驶员握持阻尼即为无界力矩阶跃 —— 高速避让工况有 83% 的时间驾驶员被力矩上限
  钳住，即一次谁也做不出来的"操作"被精确测到了小数点后三位。工况现带角加速度，
  按航点走梯形速度型线。
  > 满锁停车前后对比：348 931 rpm → 2 471；手力矩由钳在 25 N·m → 10.3 N·m；
  > 齿条力 20.4 kN，与助力曲线标定所依据的负载一致。
- **信任边界随之改写**：原先画在助力饱和处，因为当时饱和意味着发散。
  2–12 N·m 的电机尺寸扫掠现已单调、全程无振荡，饱和只是变沉 ——
  而变沉正是选型不足的**答案**。现在作废一次运行的是**驾驶员**到上限：
  车轮没到指令角度，峰值描述的是一次没有发生的操作。
- **报告表格单元格里的标记被转义**：`html_table` 默认转义单元格内容（正确），
  markup 需显式 opt-out；study 报告里的 `<code>`/`<span>` 此前是以字面量渲染的。
- **助力曲线与平台自己的负载模型差 1.65×**：曲线按猜测的 10.4 kN 标定，
  而平台负载模型说满锁停车需 20.7 kN（413 N·m 小齿轮）。已按负载模型重标。
  选型模块首次运行即发现。
- **默认电机规格不足**：峰值 5.5→8.0 N·m；空载转速 2800→8000 rpm——
  转矩到空载点线性归零，2000 rpm 工作点在 2800 空载下只剩 28% 转矩。
- 助力回路缺阻尼导致发散（停车手力矩被助力从 96.5 抬到 174.3 N·m）。
  ζ 曾为 0.0036；阻尼增益现按局部 boost 调度以保持回路阻尼比。

### 已知（新增）

- **中心区迟滞环宽对齿条库仑摩擦不单调**：0 N 时 0.99 N·m，500 N 时 0.26，900 N 时 1.66。
  0.2 Hz 下车辆自身侧向动力学滞后贡献了同量级的正交分量，与摩擦项部分抵消。
  两种效应都真实存在，真车 weave 里也都有；不成立的是拿这个数写**摩擦**要求。
  故这两条在 `steering_feel@2` 里降为参考项，并由测试钉住。
- **回正性指标仍缺"松手"工况**：需要能在工况里表达"释放方向盘"，`steering_feel`
  的最后一条因此报"未评估"。

### 已知

- 转向被控对象层在**边际带**（作动器差一点够）仍会振荡；宽裕与明显不足两个区间都已正确。
  选型恰好工作在边际带，故守卫会标记并拒绝给出余量。
- 解析路径（`quasi_static_wheel_loads`）漏了轴侧偏刚度分配，与时域模型在
  30 km/h 差 5%、60 km/h 差 17%。已由测试钉住，待修。

## 0.101.1 — 2026-08-10（显示可读性 · 3D 运动重建 · 转向手感）

详见 [`docs/v0.101.1_changelog.md`](docs/v0.101.1_changelog.md)。

### 修复 (Fixed)

- **车速数字是纯黑的**：`Speedometer` 用了 `var(--fg)`，而该变量从未定义 —— CSS 变量
  替换失败静默退回 SVG 初始 fill（黑），深色 HUD 面板上等于隐形。改用 `--text` 并
  给所有主题色加字面回退。`--surface` / `--bg` 两个同样未定义的变量一并补齐。
- **浅色主题下整块 HUD 深底黑字**：`.canvas-hud` 面板底色写死深色、文字取主题变量。
  面板底色改为跟随主题。
- **3D 车顶视角闪烁 / 不流畅**：状态流实际是 66.7 Hz（`round(dt_push/dt_sim)=3` 步），
  与显示刷新拍频出 ~6.7 Hz 顿挫；车顶视角下整个世界绑在位姿上，于是满屏一顿一顿。
  新增 `view/renderPose.ts`，用车身速度把位姿航位推算到渲染时钟并抵消平滑滞后；
  与推送频率解耦，掉到 20 Hz 也连续。车轮转角同样重建。
- **3D 深度精度**：near/far 0.3/2000 → 0.4/1200，地平线盘改为跟随相机（顺带修掉
  开出原点 1400 m 后地平线消失的缺陷）。针对 Windows 16 位深度回退，Mac 无法复现。
- **转向初始动作不自然**：一阶滞后在按下瞬间速率最大（≈1500 °/s 方向盘）再指数衰减。
  改为限速率梯形曲线，第一帧轻 20 倍，全锁 ~0.52 s；回正仍是指数衰减。
- **轮胎 μ(Fz) 只作用于轮速与诊断、不进 RK4 力路径**：摩擦圆利用率曾报到 1.10。
  现在在每阶段的载荷上生效，四个消费方口径一致。

### 新增 (Added)

- 两份转向研究报告：`steering_decoupling_value.html`（解耦自由度价值 + 六种后轮控制律
  逐条讲解）、`rear_steer_angle_range_value.html`（后轮转角范围价值），含复现脚本
  `scripts/decoupling_study/`、`scripts/rear_angle_study/` 与 `kc_profiles/ls9_estimate.yaml`。

## 0.99.3 — 2026-08-06（3D 车顶视角 · 标准工况场地重制）

### 背景

为配合方向盘/手柄驾驶，需要一个"能看着桩开"的视角，以及真正按标准铺出来、
在这个视角下辨认得清的工况场地。原来的两点问题：3D 只有轨道相机（自由/跟随），
从斜上方俯视判不了车与锥桶的相对位置；工况锥桶是 `[x, y]` 裸坐标点，3D 里画成
0.6 m 的纯色小圆锥、2D 里画成 5 px 圆点，且几何是"简化对称"的示意版而非标准尺寸。

### 新增 (Added)

- **车顶相机（准第一人称）**：3D 视图相机三选一——`自由` / `跟随` / **`车顶`**。
  车顶机位刚性固定在车身上、朝前，看得到机头和两侧车身。可调**高度 / 后移 / 俯角 /
  视场角 / 平滑 / 姿态联动**，带 `车顶` `引擎盖` `追车` 三个预设，配置存本机；
  `C` 键循环切换。相机跟随**车身航向**而非速度方向（4WIS 蟹行时两者不同，驾驶员看到
  的是车头指向）；`平滑` 只作用于航向，位置永远焊死在车身上，相机不会被车甩开。
  另加地平线地盘，车顶视角下有远近参照。
- **工况参数面板**：`/api/path/templates` 新增 `specs`（每个工况的可调参数规格），
  轨迹面板据此自动渲染参数输入框；生成后回显后端**实际铺出的几何**（车道宽/桩距/
  横向偏移…），而不是只显示一个模板名。
- **新工况**：`单移线`、`双移线 · ISO 3888-1`、`定圆 (ISO 4138)`。

### 改进 (Changed)

- **锥桶数据模型**：`cones` 由 `[x, y]` 升级为 `{x, y, kind, color, height}`，并新增
  `marks`（地面标线）。3D 里锥桶是真的交通锥——锥体 + 白色反光环 + 底座 + 投影，
  绕桩用 1.5 m 红白标杆；同 (kind, color, height) 的桩走 InstancedMesh，一套 ISO 场地
  约 3 个 drawcall/组而不是 120 个。2D 按类型分别画锥/杆。
- **ISO 几何按标准铺**：`double_lane_change` 改为 **ISO 3888-2 麋鹿测试**（段长
  12/11/12 m，间隔 13.5/12.5 m，宽度 1.1b+0.25 / b+1 / 1.3b+0.25，侧道边到边偏移 1 m）；
  车道宽随**当前车辆车宽**、泊车位随**车长/车宽**自动计算（REST 层从仿真参数注入）。
  每段两侧按 ~3 m 间距连续布桩并画出白色矩形段框，进/出门绿/红、侧道桩黄色。
- 直线/圆弧/八字/侧方停车补上夹道锥桶与地面标线；参考路径线由 1 px GL line 改为
  0.12 m 地面色带（车顶视角下 1 px 线基本看不见）。

### 修复 (Fixed)

- **绕桩参考线从桩上碾过去**：原实现把路线的过零点放在桩位（`wp.append([xc, 0.0])`），
  即车辆中心线正好穿过每一根桩。改为在桩位达到侧向幅值、在两桩之间过中线——也就是
  真正的绕桩；幅值默认取 `车宽/2 + 1.2 m`。
- 3D 轨迹线抬到 0.06 m，不再被工况地面标线盖住。

## 0.99.2 — 2026-07-21（渲染兼容性修复：2D 崩溃闪烁 · 3D z-fighting/透明/阴影）

### 背景

0.99.1 车辆可视化重制在部分 Windows 机器上出现严重频闪、抖动、兼容性问题
（提交方的 Mac 走 ANGLE→Metal 24 位深度、硬件加速，观察不到）。定位到两个 v0.99.1
引入的渲染回归 + 一个弱 GPU 放大器：

1. **2D 默认视图崩溃→闪烁**：新 `VehicleLayer` 用了 Konva `shadowBlur`/渐变（旧 2D 视图
   0 处）。当 Konva stage 在布局瞬间为 0 尺寸时，带阴影形状会 blit 一块 0×0 buffer
   canvas → 抛 `drawImage ... width or height of 0`，被视图 ErrorBoundary 捕获并重建 →
   在慢机/高 DPI 缩放机器上表现为崩溃-重建循环频闪。对时序敏感，快 Mac 命中不到。
2. **3D z-fighting / 透明排序抖动**：新几何把座舱贴在车身上（`bodyHeight-0.02` 相交）、
   车身用 `transparent opacity=0.92`（几乎不透明却关了深度写入）。相机 `near:0.1/far:2000`
   在车辆位置(~12m)的深度分辨率 24 位≈0.09mm（Mac 干净）、16 位≈22mm（比 2cm 座舱缝还
   粗 → z-fighting）；软件/老 Windows GPU 回退到 16 位即闪。
3. **软件渲染性能悬崖**：改版把车辆网格/材质翻约 3–4 倍 + 投影阴影，VM/远程/无独显的
   Windows 机器回退到 SwiftShader 软件渲染 → 个位数帧率（抖动/卡顿），上下文丢失黑屏闪。

### 修复 (Fixed)

- **P0 · 2D**：`Canvas2D` 在测得 `W,H ≥ 4` 前不挂载 `<Stage>`（容器仍留 ref 供 ResizeObserver
  测量），彻底消除 0 尺寸 buffer 崩溃；`VehicleLayer` 的阴影形状加 `perfectDrawEnabled={false}`
  作为纵深防御 + 轻微提速。开发态 StrictMode 双挂载下原本必现的崩溃已消失。
- **P1 · 3D**：车身壳体改为**不透明**（0.92→实心，去掉 `transparent` 标志）——恢复深度写入，
  消除透明排序抖动，并让嵌入车身的座舱底部被正确遮挡；座舱下沉 8cm 嵌进实心壳体，座舱缝
  即使 16 位深度也不再 z-fight；相机 `near 0.1→0.3`（约 3× 深度精度，对近距观察无影响）；
  平行光加 `shadow-normalBias` + `shadow-mapSize 1024` 消除新曲面壳体的自阴影噪点。视觉观感
  与 0.99.1 一致（0.92 本就近乎不透明）。

### 验证

- 新增 `scripts/render_verify/`：在 Mac 上复现弱 GPU / Windows 渲染路径的验证工具——
  `launch.sh software` 强制 Chrome 走 SwiftShader 软件 WebGL（已验证复现 `[SOFTWARE FALLBACK]`），
  `webgl_probe.html` 读出 renderer/DEPTH_BITS/FPS + z-fight 自检，`README.md` 给 Mac(Tier 1)
  与真 Windows `chrome://gpu`(Tier 2) 两层路径。
- type-check / 生产构建通过；2D 崩溃在原可复现环境已消失，3D 车身渲染干净。
- **注**：16 位深度 z-fighting 与真实 Windows 驱动行为无法在 Mac 上 100% 复现；本补丁按机理
  修复，建议在报告问题的那台 Windows 机上回归确认。

## 0.99.1 — 2026-07-20（车辆可视化重制：2D/3D 共享车身轮廓 · 细节渲染）

### 背景

0.99.0 收敛 V1 候选门禁后，本轮回到前端表现层：2D 俯视图和 3D 视口此前各自
硬编码一套简化车身/车轮几何（矩形车身 + 矩形车轮），两个视口观感不一致，
细节也偏示意图。本轮把车身轮廓抽成单一数据源，两个视口共享，并把渲染精度
提升到能看清车灯、轮辋和轮胎打滑状态的程度。

### 新增 (Added)

- `frontend/src/components/vehicleShape.ts`：车身/座舱轮廓、车灯位置、轮胎/轮辋/
  轮毂比例和明暗主题调色板的唯一数据源，供 2D 和 3D 视口共同消费。
- `frontend/src/components/canvas2d/VehicleLayer.tsx`：从 `Canvas2D.tsx` 拆出的
  车身渲染层——渐变漆面车身、座舱玻璃、前后车灯（含大灯光锥）、轮辋+辐条+
  轮毂的车轮、滑移角警示光环（kinematic 模式 α≡0 时不可见）、限速裁剪的
  速度矢量箭头。车轮辐条随车轮积分转速旋转，缩放到底时自动降级为简化轮廓
  避免糊成一团。
- `Canvas3D.tsx`：车身壳体和座舱改为从 `vehicleShape.ts` 轮廓挤出（extrude）
  的实体几何，不再是长方体拼接；车轮加装轮辋圆盘、5 根辐条和轮毂盖；新增
  前大灯（emissive 球体）和尾灯（emissive 方块）。

### 变更 (Changed)

- `Canvas2D.tsx` 的车身/车轮内联绘制逻辑迁移到 `VehicleLayer.tsx`，主文件只负责
  传入 state/相机参数；车轮转速积分（用于辐条动画）从主组件的 `useFrame`-等价
  `useEffect` 中维护，和 `Canvas3D.tsx` 的积分方式保持一致。
- 2D/3D 车身尺寸统一改用 `vehicleShape.ts` 的 `bodyDimensions()`，短轴距标定
  （如 delivery_robot）比例不再失真。

### 验证

- `npm run type-check` 通过；生产构建、Playwright 生产冒烟和后端全量门禁见
  下方发布记录。

## 0.99.0 — 2026-07-08（V1 候选收敛：除独立外部/实测 reference 外全量门禁）

### 背景

本版本把 v1.0 前的工程、验证、报告和发布链路收敛到候选状态。真实外部工具、
台架、缩比车或实车 reference 数据暂时无法提供，因此不声称严格 v1.0；该缺口仍
由 `independent_reference` gate 明确阻塞。

### 新增 (Added)

- `scripts/check_v1_readiness.py` 汇总黄金实验、验证矩阵、reference benchmark、
  incoming intake、Playwright smoke、研究报告流水线、交付材料和实物验证接口状态；
  `docs/reports/v1_readiness.md` 留存当前机器可读结论。
- reference benchmark intake 闭环：`.incoming` 审核、promotion gate、原始 artifact
  SHA-256 校验、provenance 必填校验、metrics/artifacts 候选片段生成、CSV 归一化和
  `intake_checklist.json` 结构化待办清单。
- `scripts/pre_release_check.py` 接入 release asset 检查、reference report freshness、
  v1 readiness freshness、可选 incoming audit、可选 strict-v1 和 portable zip 检查。

### 变更 (Changed)

- 前端主入口、3D vendor、页面 CSS 和全局样式已按 v1 候选预算拆分，生产构建无
  chunk warning；Playwright workflow smoke 覆盖 50 个关键和异常路径场景。
- 单轮失效研究报告继续作为内部机制研究交付物，报告流水线、metrics JSON 和自包含
  HTML 保持可复现。

### 已知边界

- `backend/.venv/bin/python scripts/check_v1_readiness.py --strict-v1 --require-portable-zips`
  仍会失败，直到接入至少一个通过检查的 `external_tool`、`bench`、`scaled_vehicle`
  或 `full_vehicle` benchmark。

## 0.16.0 — 2026-07-03（车辆页几何工作室：三张参数驱动·可拖拽建模示意图）

### 背景

车辆页此前是纯数值表单，改了参数没有直观反馈。按需求把它升级为「几何工作室」：
三张随参数实时重建的 2D 示意图（整车 / 主销·车轮 / 前桥齿条硬点），且图上的
点可**拖拽反写参数**——把标定从"填数字"变成"看着几何调"。

### 新增 (Added)

- **共享几何数学** `vehicle/geometryModel.ts`：整车派生量（轴荷分配、
  阿克曼最小转弯半径、m·a·b 惯量参考）、主销派生量（机械拖距 R·tanε、总拖距）、
  转向连杆 4-bar 求解（linkageBase/solveRackToWheel/rackTravelProfile）、
  前桥阿克曼状态（内/外轮角、阿克曼误差、转弯半径、效率、奇异）、越界校验。
  纯函数、无 React——车辆页三图与理论页示意图共用这一份。
- **参数编辑 context** `VehicleParamsContext`：车辆页统一编辑缓冲，数值表与
  拖拽共写、一个「应用」提交；ParamsPanel 改为消费它。
- **三张示意图**（`components/vehicle/`，可拖拽建模）：
  - 整车俯视：轴距/轮距/质心/轴荷/转弯圆，拖质心·轮·前轴改参数 + 侧视质心高；
  - 主销·车轮：正视(内倾·外倾·scrub) + 侧视(后倾·机械/气胎拖距)，拖主销轴/轮顶；
  - 前桥齿条硬点：真实 4-bar 连杆求解，齿条位移滑杆驱动阿克曼，拖内/外球头改硬点。
  每图带实时派生量 readout + 越界/奇异红旗。
- **理论页示意图统一**：主销侧视/正视、齿条梯形机构改由 geometryModel 计算
  （标注真实派生值，消除硬编码数字，与车辆页同一数学源）。

### 修复 (Fixed)

- 前桥阿克曼建模：刚性齿条平移时右轮本地行程应取 −rt（轴向镜像），否则
  齿条变成"拉伸"、左右轮对称无阿克曼差。修后内/外轮角正确分化（满舵内 40.1°/
  外 33.7°、阿克曼误差 −4.8°）。
- 横摆惯量校验改用 m·a·b 经验估算（对 LS9 ≈7236，接近 7500）而非箱体估算
  （会对默认车误报）。

### 测试

- 前端 type-check + build 通过；后端 202 passed 无回归。浏览器端到端：三图
  参数驱动渲染、拖 CG→cg_to_front 联动数值表并触发「应用」栏、齿条滑杆驱动
  阿克曼、理论页派生值一致，零控制台错误。

## 0.15.0 — 2026-07-03（手柄映射机制：预设 + 分组直控 + 实时校准）

### 背景

手柄此前只做 (左摇杆→转向, 扳机→油门) 单一映射。用户需要"双摇杆独立控制"
且"既能左右独立又能前后独立、设置成不同模式"。据此把手柄输入重构为一套
**「轴→车轮分组」绑定机制**——分组本身就是可选模式，前后/左右/逐轮/蟹行
都是同一机制填不同绑定表。

### 新增 (Added)

- **手柄三种顶层模式**：
  - `assisted` 经典（默认，摇杆→油门/转向→当前策略，与键盘叠加）；
  - `direct` 直控（后端新增 `manual_wheel` 策略收 4 个归一化轮角），分组预设：
    **前后轴独立**（左摇杆X=前轴/右摇杆X=后轴）、**左右侧独立**、
    **逐轮直控**（左摇杆=左边两轮X前/Y后，右摇杆=右边两轮，四轮全独立）、
    **蟹行**（四轮同角）；
  - `holonomic` 全向（后端新增 `manual_body` 策略）：左摇杆=平移(前进+横移)、
    右摇杆X=自转——斜开+自转同时做。
- **实时校准/绑定面板**（`GamepadConfigPanel`）：每通道点「绑定」后拨动即
  自动识别物理轴（设备无关，方向盘/HOTAS 换轴序即用）；每轴死区/expo/反向；
  全局死区+转向灵敏度；实时轴/键监视条；配置存 localStorage。
- 选预设自动切到对应后端策略；从手柄模式切回「经典」自动恢复
  ideal_ackermann（避免策略滞留 manual_*）。
- 前端映射逻辑抽到 `input/gamepadConfig.ts`（类型/预设/轴读取/持久化），
  KeyboardInput 单写者循环按 config 分发。
- 测试 +6（`test_manual_strategies.py`：逐轮独立角、分组、全向蟹行/前进+自转、
  边界钳制）。

### 测试

- 全量 202 passed + smoke 全绿；前端 type-check + build 通过。浏览器端到端：
  前后轴独立 → FL/FR +17.5°、RL/RR −17.5°（分组生效）；全向 → 斜向平移
  同时自转；逐轮直控四通道轴0-3；预设自动切策略与回切均正确，无控制台错误。

## 0.14.0 — 2026-07-03（安全研究 v2：机构差异化失效模型 · 参数敏感性流水线）

### 背景

按实际机构设定重做单轮失效研究：前轮**无自锁**（逆效率 ~60%）→ 断电失效
呈**自由脚轮**；后轮**自锁**（正效率 30%/逆效率 0）→ 失效**锁死在当时位置**。
同时把可控性分析扩展到整车参数空间（车轮/车身/底盘 9 参数），并按
"用实际分析迭代仿真工具"的目标沉淀平台能力。

### 新增 (Added) — 平台能力（分析需求倒逼）

- **自由脚轮失效模型** `FaultSpec free_caster`：失效轮转向自由度变为机构
  动力学 J·δ̈ = −η_rev·τ_kingpin − c·δ̇ − τ_c·sgn(δ̇)（会话层半隐式积分，
  复用平台逐步实时计算的 Reimpell 主销力矩，经作动器环节生效；参数
  eta_rev/j_steer/c_damp/tau_coulomb 可配）。
- `fault_reconfig` 增加 **free 模式**：自由轮无寄生力可抵消 → 健康同轴轮
  增益补偿（front_gain≈2 恢复轴合力）+ 同款横摆 PI + 限幅减速。
- **参数×故障×工况敏感性流水线**：实验 vehicle.overrides（含 suspension
  嵌套）/ scene μ 逐点扫描，每个取值自动配同参数无故障参考 run。
- 测试 +2（自由轮自对准与摆振有界、free 模式增益分配），全量 196 passed。

### 研究结论 v2（报告已重写，docs/reports/）

- **两类失效物理本质不同**：前轮自由失效经 ~4 Hz 衰减摆振（经典 caster
  shimmy 瞬态，c=80 N·m·s/rad 下约 1 s 收敛）后自对准零侧偏力平衡，全部
  工况 ≤C2（直行仅剩 toe 失衡慢漂）；后轮自锁失效冻结为持续干扰，跑飞后
  锁死 @100 km/h 为 C3/ASIL D。缓解后全矩阵 C3 清零。
- **参数敏感性两条推翻直觉**：①第一敏感因子是轮胎侧偏刚度（硬胎干扰
  ∝c_α·δ_s，80→150 kN/rad 使缓解后偏差近翻倍）与横摆惯量（∝1/I_z，
  轻小车更危险）；路面 μ 近乎平坦——低附着把干扰力削顶、力平衡型缓解
  两侧同缩。②主销后倾对失效可控性近零敏感（锁死轮不进力路径；自由轮
  稳态由 α→0 结构性保证）——失效工况不构成 caster 设计约束；瞬态几何
  抓手是**小主销偏置 scrub**（5→50 mm 使 Δr 峰值 +31%）。
- **机构选型量化论据**：非自锁+高逆效率以正常供电能耗换失效温和性；
  后轮若改可反驱+常闭离合器（断电脱开→自由脚轮），ASIL D 危害可整体降级。
- 报告新增 §9「仿真工具迭代记录」：v0.10 实验底座 → v0.13 故障注入+容错
  策略 → v0.14 自由脚轮+敏感性流水线的演进链。

### 测试

- 全量 196 passed；报告 ~170 run 一键复现（主矩阵+检测延时+参数敏感性
  含逐点参考）。

## 0.13.0 — 2026-07-02（单轮失效功能安全研究：故障注入底座 · 容错策略 · 论文级报告）

### 背景

首个用平台完成的实战交付：单轮转向卡死（前/后轮）的 ISO 26262 可控性
定量分析 + 安全机制设计 + 自包含图文报告。路线图 C2「失效模式批量矩阵」
的能力底座随之落地。

### 新增 (Added)

- **实验级定时故障注入**：`FaultSpec`（stuck_zero / stuck_hold（锁存实际
  轮角）/ stuck_value / limited，按仿真时间触发）进 Experiment schema，
  SimSession 作用于指令级、经作动器动力学生效。
- **`fault_reconfig` 容错降级策略**（注册进 registry）：
  ①同轴镜像力抵消前馈——卡死轮偏差 e 由同轴伙伴轮反向 −e 精确对消
  ΣF_y 与 ΣM_z（同轴等臂）；②ESP 级横摆 PI 稳定（k_r=0.5、积分抗饱卷、
  ±8° 限幅、全部健康轮前+后−等臂分配）——镇定后轮高速卡死的自旋趋势
  （Δr 峰值 31.8→3.7°/s）；③带减速度限幅（3 m/s²）的降级限速——阶跃限速
  会让轮速伺服以摩擦极限制动、纵向力抢占饱和后轴的摩擦椭圆。
- **研究脚本** `scripts/study_single_wheel_failure.py`（一键复现）：
  3 工况×2 轮位×3 故障模式×2 缓解 + 5 档检测延时敏感性（22 个仿真 run）→
  横向偏差/TTLD/Δr/Δa_y/β 指标 → C1/C2/C3 分级 → 7 张图 →
  **自包含 HTML 论文级报告** `docs/reports/single_wheel_failure_safety_analysis.html`
  （摘要/相关项/HARA/判据/机制推导/结果/FTTI 分解/局限，图文内嵌 base64，
  指标 JSON 随附）。
- 测试 +4（`test_fault_reconfig.py`）：注入到位与锁存、镜像几何、
  缓解使 2 s 航向漂移至少减半。

### 关键工程发现（详见报告）

- 未缓解：高速直行单轮卡死 +4° 为 **C3**（TTLD 0.8–0.9 s < 反应时间）；
  后轮卡死呈自旋趋势（传统前转向车辆对后轴故障无执行器可补偿）。
- 缓解后：**全矩阵消灭 C3**；弯中 TTLD → ∞；τ_d ≤ 0.05 s 可达 C1 →
  给出 FTTI 分解建议（检测 ≤250 ms）。
- 设计权衡记录：纯运动学 ICR 投影重构被数据否决（蟹行漂移 + 过渡横摆
  22°/s 反超基线）——失效后控制目标应为"路径保持"而非"运动学一致"。

### 测试

- 全量 194 passed；研究矩阵与报告可由脚本完整复现。

## 0.12.0 — 2026-07-02（平台重构 Phase C-1：时域 c_α(Fz) 载荷敏感度）

### 背景

Phase C 物理深化（已批准优先级最高项）第一条：负载页一直在用的
c_α(Fz) = c_α0·(Fz/Fz_nom)^0.8 载荷敏感度进入时域轮胎模型。这是"横向载荷
转移降低轴侧向总容量"的物理根源（不足转向预算的来源）——此前时域里载荷
转移只在轮间搬力、线性区轴合力不变，操稳趋势失真。

### 新增 (Added)

- `LinearTireModel` / `PacejkaTireModel` 增加 `fz_nom`/`load_exp` 字段与
  `c_alpha_eff(Fz)`；`make_tire` 按新参数 **`tire_load_sensitivity_time_domain`
  （默认开）** 注入 Fz_nom = m·g/4 与现有 `tire_load_sensitivity_exp`。
  只缩放 c_α——c_κ 保持常数，轮速伺服整定不受影响。
- camber 等效侧偏吸收（`camber_thrust_alpha_offset`）同步改用载荷敏感 c_α
  （开关联动），保证等效力精确。
- 测试 +2：轮胎级单元（1.5×Fz_nom → 侧向力比 = 1.5^0.8；关闭时线性区
  刚度与载荷无关）；整车级（稳态弯中开/关横摆差异可测但 <15%，直行两者
  均无跑偏）。

### 测试

- 全量 190 passed + smoke 全绿。行为差异：带横向载荷转移的稳态弯中
  横摆增益略降（真实修正量级），直行/负载页不受影响。

## 0.11.1 — 2026-07-02（平台重构 Phase B-2：run 回放 · ⌘K 命令面板）

### 新增 (Added)

- **run 回放**（`ReplayPanel`，分析页「▶ 回放」）：所选 runs（≤6）以各自
  配色的**幽灵车**同屏重演——车身带指向鼻锥 + 四轮按记录的实际转角旋转，
  轨迹淡显作参照；播放/暂停、0.5–4× 倍速、时间轴拖动；车辆几何取自 run
  快照的 vehicle overrides（缺省 LS9）。回放时间在所有通道叠图上画**黄色
  竖直游标**（复用 uplotFactory verticalMarkers，不破坏缩放状态）——
  CarSim「动画与曲线共游标」的 Web 版。
- **⌘K / Ctrl+K 命令面板**（`CommandPalette`）：页面导航 ×7、策略切换
  ×N、模型切换 ×3、重置位姿、主题切换；子串过滤（中文标签+拉丁关键词）、
  ↑↓/Enter/Esc 全键盘操作。

### 测试

- 浏览器端到端：4 runs 开回放 → 拖到 t=8/9.5 s 幽灵车位置正确
  （40 km/h 落后 60 km/h）→ vx/横摆叠图黄游标同步；⌘K 过滤"试验"→
  Enter 跳页 → 面板关闭。前端 type-check + build 通过；后端 188 无回归。

### Phase B 剩余（B-3）

- dockview 自由布局（运行/分析页）、per-channel 单位体系、
  ExcitationPanel/ScorePanel 降级为后端 KPI 展示层。

## 0.11.0 — 2026-07-02（平台重构 Phase B-1：工作流导航壳 · 试验页 · 分析页）

### 背景

v1.0 平台重构第二期第一批：信息架构从"5 个侧栏 tab 堆 17 个面板"改为
CarMaker 式工作流分段，并给 Phase A 的实验底座配上完整的操作界面——
从此"定义实验 → 批量运行 → 结果对比"全程在浏览器里点击完成。

### 新增 (Added)

- **工作流导航 rail**（左侧七段）：运行（原工作台）/ 试验 / 分析 / 车辆 /
  场景 / 负载 / 原理。页面路由入 store（跨页跳转如"跑完→去分析"）。
- **试验页**（`ExperimentPage`）：实验库 CRUD（含 Phase A 种子实验）+
  机动分段编辑器（转向剖面×目标车速×斜坡，按剖面类型显示参数）+
  参考路径模板选择 + **运行矩阵**（策略多选 × 车速列表 → 变体展开）+
  批量进度条 + 完成即出 KPI 小表 + 一键跳分析页（预选这些 runs）。
  含幅值安全提示（归一化转向 0.05@60 km/h ≈ 4.6 m/s²）。
- **分析页**（`AnalysisPage`）：run 库浏览（最多 6 个调色板配色多选、删除）
  + **KPI 对比表**（8+ 指标 × runs，含阶跃响应组）+ **通道叠图**
  （uPlot 复用、通道选择器+预设 chips、PNG 导出、最长 run 供时间基）
  + **轨迹俯视叠图**（世界系等比例 canvas、10 m 网格、起点标记）。
- 面板按工作流归位：场景页 = 视图 + 场景路况/轨迹路径/扰动/故障注入
  （保留画布点击放置交互）；车辆页 = 参数 + 项目管理；工作台侧栏瘦身为
  驾驶/设计/验证/数据四段。
- `api/experiments.ts` 类型化客户端（schema 与后端 pydantic 一一对应）。

### 修复 (Fixed)

- KPI 对比表 flex 塌陷：flex 列子项设 `overflow-x:auto` 后自动最小高度
  归零、表格被压成 0 高（内容被下方图卡覆盖）→ `flex-shrink: 0`。

### 待办（Phase B 后续批次）

- dockview 自由布局（运行/分析页）、run 回放（artifact 驱动 2D/3D +
  时间轴 scrubber）、⌘K 命令面板、单位体系；ExcitationPanel/ScorePanel
  降级为后端 KPI 的展示层。

### 测试

- 前端 type-check + build 通过；浏览器端到端验证：载入种子 DLC 实验 →
  2 策略×2 车速矩阵 4 runs（2.9 s）→ KPI 小表 → 跳分析页 → KPI 对比表 +
  轨迹/车速/横摆叠图全部渲染，无控制台错误。后端 188 passed 无回归。

## 0.10.0 — 2026-07-02（平台重构 Phase A：实验底座 · 伺服前馈修正）

### 背景

v1.0 平台重构（docs/v1_platform_refactor_plan.md，已批准 A→B→C）第一期：
对标 CarSim/CarMaker 的核心范式——"实验是一等公民、每次运行留下结果资产"。
此前机动激励跑在前端 wall-clock 定时器上（不可复现/不可批量），KPI 算在浏览器
30 s 环形缓冲上（不落盘），求解器只有 1× 实时一种形态。

### 新增 (Added) — `sim4wis.experiment` 包

- **Experiment schema**（`experiment/schema.py`）：车辆(profile+overrides) +
  模型 + 策略(+mode_params) + 场景 + 参考路径(模板/waypoints) + **sim-time 机动**
  （分段：constant/step/ramp/sine/sweep/dlc 转向剖面 × 目标车速+斜坡）+ 记录率 +
  KPI 选择，YAML 存 `experiments/`。附两个示例：`iso3888_dlc_60kmh`、
  `step_steer_60kmh`。
- **SimSession 无头会话**（`experiment/session.py`）：与实时 Simulator 共享
  同一内核（模型+策略+场景+共享的 `core/derived.py` 派生量），去 wall-clock、
  全速执行、逐位可复现（确定性有测试兜底）；记录 46 通道（含滑移角/滑移率/
  驾驶员输入，比实时 Recorder 更全）。
- **KPI 后端化**（`experiment/kpi.py`）：ScorePanel 七项指标迁到后端并扩展——
  瞬心偏差峰值/RMS、横摆峰值、侧向速度峰值、转向能耗、齿条力峰值、侧偏峰值、
  车速跟踪 RMS + **阶跃响应组**（横摆增益、10–90% 上升时间、超调、5% 稳定时间）。
- **run 结果资产**（`experiment/store.py`）：`runs/<id>/meta.json`（实验快照+
  KPI+版本）+ `data.csv`；experiments YAML CRUD。
- **批量/变体矩阵**（`experiment/batch.py`）：dotted-path overrides 变体展开
  （如 `maneuver.steps.0.speed_kmh` / `strategy` / `vehicle.overrides.mass`），
  后台线程顺序执行不阻塞实时环，job 注册表可轮询/取消。
- **REST**（`/api/experiments`、`/api/batch`、`/api/runs`…）：定义 CRUD、
  批量启动/进度/取消、run 列表/通道查询(可抽取+抽稀)/删除。
- follow_trajectory 增加 `plan_override`：无头会话注入自己的路径，不再碰
  进程级单例 active plan。

### 修复 (Fixed)

- **轮速伺服斜坡跟踪结构性超调**：纯 PI 追速度斜坡需要持续误差喂 P 项
  （整车折算惯量 m·r²/4≈113 kg·m² ≫ 轮惯量），积分器在斜坡段合法充电、
  到速后把车速挂在 cmd+ki·I/kp（实测 60 指令稳在 64.6 km/h，只能靠风阻放电）。
  两处修正：①防饱卷从"先积后夹"改为**条件积分**（饱和且误差同向时冻结）；
  ②增加**指令加速度前馈** `ff_inertia·dω_cmd/dt`（滤波指令导数，非被否决的
  r·Fx 状态反馈，无反馈回路）。修后斜坡滞后 ~1.5 km/h、超调 0.4 km/h。
- smoke「动力学直线」窗口 3.0→3.5 s：旧窗口是靠积分饱卷的"超速冲刺"擦线
  通过的（摩擦限幅起步理想下限就要 2.4 s）。

### 测试

- 新增 `test_experiment_batch.py` 9 项：schema YAML 往返、转向剖面形状、
  会话确定性（两次运行逐位一致）、path override、阶跃 KPI、变体展开、
  DLC 批量验收（2 策略×2 车速落盘+回读+删除）、CSV NaN 往返、REST 全链路。
- 全量 188 passed + smoke 全绿。

## 0.9.0 — 2026-07-02（时域物理补齐 · 轮速积分稳定性 · 手柄输入）

### 背景

针对"整体完整性不够"的三视角（整车工程 / 底盘控制 / 软件产品）系统审查。
核心发现两类：①负载页 v0.7.3–0.8.x 补齐的物理（Crr/Cd·v²、气动升力、
camber thrust、静态 toe）从未进入时域模型——驾驶工作台与负载页口径割裂，
负载页会漂的 δ_eq 在工作台上永远看不到；②补入持续纵向力后暴露出一个
**0.8.2 就潜伏的数值失稳**：轮速自旋 ODE 的线性化模态 λ = c_κ·r²/(I_w·v)
在 v≤10 m/s 时 λ·h 超出 RK4 实轴稳定域（≈2.78），旧版只因平路巡航平衡点
恰好零力而未被激发；任何持续纵向力（坡道/阻力/加减速瞬态）都会触发 κ
持续振荡（实测 10 m/s 巡航 |κ| 达 0.13–0.76）。

### 修复 (Fixed)

- **轮速自旋积分数值失稳**（0.8.2 潜伏 bug）：轮速 DOF 从两个时域模型的
  RK4 状态向量中拆出，体动力学积分期间保持不变，随后用
  `model_core.semi_implicit_wheel_spin` 半隐式推进——线性滑移区后向欧拉
  （无条件稳定），摩擦饱和区（∂Fx/∂ω≈0，本身非刚性）退化为前向欧拉以免
  隐式分母错误压制打滑。实测 1–25 m/s 全速域稳态 |κ| ≤0.002。
- **multibody 缺坡道纵向重力**：simplified_dynamic 有 grade 项而 multibody
  没有——Slope 扰动在"高保真"模型下只抬轮不减速。补入与 simplified 相同的
  −m·g·sin(grade) 工程近似（grade 由前后轴 road_z 差反算）。
- **FastAPI 应用版本号硬编码 "0.3.0"**：改读 `sim4wis.__version__`。

### 新增 (Added)

- **时域模型物理补齐**（dynamic + multibody 同步，与负载页共用
  `model_core` 数学源）：
  - 空气阻力 ½ρ·Cd·A·v² + 滚动阻力 Crr·m·g（tanh 平滑符号），巡航时
    电机需持续输出驱动扭矩，稳态驱动力与负载页 A1 驱动力口径一致；
  - 每轴气动升力 ∝v²：dynamic 直接修正 Fz，multibody 作为簧上外力
    进垂向+俯仰动力学再经悬架传到轮荷；
  - camber thrust：等效侧偏角偏移吸收进轮胎模型（同负载页 H1 吸收法），
    直行时也加载主销力链；
  - 静态 toe：实际轮角 = 作动器角 + per-wheel toe 偏置（同负载页 A3）。
  - 效果：工作台直行时 δ、τ_steer、齿条力不再恒为零，δ_eq 的时域体现
    与负载页故事闭环。
- **手柄 / USB 方向盘输入**（需求文档第 4 条落地）：Web Gamepad API
  standard 映射，左摇杆 X=转向、RT/LT 扳机=前进/后退（无扳机设备退化为
  左摇杆 Y），死区 0.08，与键盘输入叠加合成（单写者循环）；控制面板显示
  连接状态 + 启用开关。
- **头部版本徽标**：前端从 `/api/version` 读取显示。
- **时域模型直接测试**（此前 168 个测试无一直接覆盖两个动力学模型）：
  新增 `test_time_domain_models.py` 11 项——巡航阻力扭矩、气动升力降 Fz、
  toe/camber 直行加载主销但不跑偏、清零对照、坡道驱动力增量（multibody
  坡道回归）、满舵数值有界。

### 测试

- 后端 179 passed（168 旧 + 11 新）+ smoke 全绿；前端 type-check + build 通过。
- 平路巡航一致性：两个时域模型稳态驱动力逐位一致且等于阻力理论值
  （25 m/s：711 N ≈ Crr 341 N + 风阻 372 N）。

## 0.8.2 — 2026-06-21（主销力矩气胎拖距修正 · 负载页/工作台 UX）

### 背景

依据姊妹项目《底盘与转向动力学讲义》参考《Steering Handbook》做的数学口径
复核（CHANGE_REPORT §5.5/§5.6），回头审查 4WIS 底座，发现 kingpin 力矩
**气胎拖距被重复结构化**：`Fy` 力臂里加了一份常数 `t_p`，求和时又加了
轮胎自回正力矩 `Mz = −Fy·t_p(α)`，两者符号相反互相抵消 → 气胎拖距对
转向阻力矩的贡献几乎为零（小角处最严重，正好是 δ_eq 所在区）。本版修正。
同时按试用反馈做一轮负载页/工作台的 UX 层级增强。

### 修复 (Fixed)

- **主销力矩·气胎拖距重复计算**（report §5.5）：`kingpin.py` 移除把 `t_p`
  并入 `Fy` 机械力臂的 `t_pneumatic_extra` 通道，`Fy` 力臂现在只含
  scrub + caster 机械拖距；气胎拖距**只**经 `Mz` 通道进入，且符号改为
  `−Mz` 让它与机械拖距**同向叠加**（之前抵消）。气胎拖距用的是 Pacejka
  随滑移衰减的 `t_p(α)`，比之前的常数力臂更真实。波及 `dynamic` /
  `multibody` / `load_analysis` / `model_demo` 四处调用点（均曾传入
  `t_pneumatic_extra=t_p`）。
  - 影响：转向阻力矩 τ、齿条力、电机扭矩、δ_eq 的小角段都会变化（更准）；
    `torque_steer` 仅作诊断/记录/δ_eq 检测，不进控制环，不影响伺服稳定性。

### 变更 (Changed)

- **负载页移除"峰值电机力矩"卡片**（按用户要求；电机扭矩可由峰值齿条力
  × r_p/i 直接换算，无需单列）。
- **负载页工具条三段分区**：模型选择 / 计算设置 / 输出结果，每段加标签，
  解决"参数表一锅端、操作目标不明确"。
- **负载页 KPI 卡片层级**：峰值齿条力、零输出自然转角作为主指标高亮，
  其余次级。
- **工作台首屏可读性**：新增可关闭「快速开始」四步引导卡（localStorage 记忆，
  表头 ? 重新唤出）；表头加「车速 / 策略」状态摘要；页签与侧栏当前项强调。
- **「数学模型」页改名「原理简介」**（顶栏页签）。
- **原理简介页第 5 章（主销力矩）同步修正**：公式由旧的
  `τ_KP = Fy·(s+t_m+t_p) + Fx·s + Mz + …`（气胎拖距重复计算）改为
  `τ_KP = Fy·(s+t_m) + Fx·s + Mz + …`，气胎拖距只经 `Mz ≈ −Fy·t_p(α)` 计一次，
  与修正后的底座 `kingpin.py` 及《Steering Handbook》Ch.6 一致；推导补"易错点"
  说明旧版抵消问题，并加 Reimpell scrub 工程等效/严格 3D 的口径注。
  主销俯视图与四项分解 demo 的标签同步（`Fy×(s+t_m)`、`t_p 只走 Mz`）。
- 第 3 章补摩擦椭圆口径注（一般式允许 μx≠μy，本模型取 μx=μy=μ）。

### 复核结论（未改动项）

- §5.2 阿克曼 `j/L`：底座用真实连杆几何（硬点）反解，不含 `cot=t/L` 简化式 → 无需改。
- §5.1 摩擦椭圆 μx≠μy、§5.4 λf/λr 横向载荷转移分配：属可选保真增强，非正确性 bug，本版不动。
- §5.7 4WIS 横摆≠独立外加 Mz：底座本就用逐轮力进稳态解，已符合。

### 测试

- 后端新增 2 个 kingpin 回归（`Mz` 以 `−Mz` 入和；气胎+机械拖距同向叠加、
  加入 `Mz` 必增大 |τ| 而非抵消）；全量 168 passed + smoke 全绿。
- 前端 type-check + build 通过；浏览器验证两页 UX 与快速开始关闭/唤出闭环。

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
