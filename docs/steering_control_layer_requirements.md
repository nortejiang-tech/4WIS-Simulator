# 转向角度跟随控制层 — 需求挖掘与方案调研（开发基准 v1.0）

- **日期**：2026-08-14
- **目标**：在线控转向（SBW）、后轮转向（RWS）与四轮独立转向（4WIS）上，评估与开发"转向角度跟随控制策略"——在"转角指令"与"车轮实际转角"之间插入一层可控的、可调参的、可替换的控制器
- **状态**：**已冻结为开发基准**。§4 的 7 个决策点已由需求方确认（见 §4），并补充两项：前馈三件套与控制库扩充
- **前置调研**：本文 §1 基于对现有链路代码的梳理；§2 基于公开文献调研（引用附文末）

---

## 1. 现状链路：指令到车轮转角的实际路径

```
DriverInput.steering ∈ [-1,1]（归一化）
  → ControllerStrategy.compute  →  ControlCommand.delta_cmd：4 元素数组，单位 rad，逐轮独立
  → VehicleModel.step
      无转向 plant（默认）:  四轮各自走 steer_actuator —— 一阶滞后（τ=60 ms）+ 速率限幅（8 rad/s）
      有转向 plant:          前轴 delta_cmd[0..1] 被平均成单一角度 → SteeringPlant（机械柱）或 ByWirePlant（线控）
                             后轴仍走 steer_actuator；前轴输出被同时赋给左右两轮
  → state.delta（四轮实际转角）
```

关键事实（详见代码）：

1. **指令层已经四轮独立**（`ControlCommand.delta_cmd` 是 (4,) 数组，`controller/base.py` 的 `compute_commands` 逐轮求解），但**执行层只有两个通道**：前轴一个 plant（左右被平均，`steering_link.py` 的 `0.5*(delta_cmd[0]+delta_cmd[1])`）、后轴一个参数共享的一阶执行器（`geometry.py` 的 `steer_actuator`）。
2. **线控前轴的"角度跟随"现在是开环趋近**：`ByWirePlant.step`（`steering/bywire.py`）是 `target = δ_act + (δ_cmd − δ_act)·α`（一阶带宽 10 Hz 限幅）+ 速率限幅 6 rad/s——**没有位置反馈闭环、没有积分/抗饱和、负载力不进入角度方程**。偏差只被监控（`steer_angle_deviation` 通道、`deviation_flag`），不参与控制。
3. **架构能力只声明不实现**：`architecture.py` 声明了 `angle_tracking / per_wheel_steer / rear_actuator / corner_actuator` 等能力，但除"前轴机械/线控二选一"外，**后轴、逐轮执行器没有任何对应对象**（`rear_axle`、`front_independent` 都是元数据）。
4. **采样结构**：外环 5 ms（`core/simulator.py` dt_sim=0.005），机械柱路径已有 0.5 ms 内环（`STEERING_INNER_DT` + 指令插值，`steering_link.plant_front_angle`）；**线控路径没有内环**，直接 5 ms 驱动。
5. **反馈信号现状**：线控有 `steer_road_wheel_angle`（轮角）与 `steer_angle_deviation`（指令−实际）；机械路径只有 pinion 角。**没有逐轮实际转角的传感器通道**（`state.delta` 是模型真相，不是测量信号）——控制层需要的"位置反馈"必须补齐，且应建模传感器量化/延迟/噪声。

**插入点结论**：控制层最自然的位置是 `delta_cmd[4]` 与执行器动力学之间——即改造 `steering_link.front_axle_step` / 新建 per-corner 执行器分支，使前轴、后轴、四轮全部经过"同一个角度跟随控制器接口"。这正好把"指令层已四轮独立、执行层只有两通道"的结构性缺口补上。

---

## 2. 控制方案调研（主流 / 非主流 / 调参）

### 2.1 主流方案

| 方案 | 结构 | 优势 | 劣势 | 文献定位 |
|---|---|---|---|---|
| **单环 PID/PI（位置环直驱）** | 位置误差 → P/PI → 执行器力矩（或速率指令） | 实现简单、整定直观、稳态精度好（有 I 项） | 带宽受限；抗负载扰动弱；增益与工况耦合，需要调度 | 基线；工业 SBW 量产常驻 |
| **级联双环 PID（位置环 + 速度/电流内环）** | 外环位置 PI → 内环速度（PI）→ 电流环（一阶近似或显式建模） | 电流波形平滑、稳态误差小、抗扰分层处理、符合真实 ECU 结构（EPS 电机驱动即此结构） | 响应偏慢；整定维度高（2 环 × 3 参数） | 清华 2024 对比研究：级联 PI 稳态误差最小、电流最平滑 |
| **LQR（+ 积分状态 / 状态反馈）** | 对执行器线性模型（角度、角速度、[误差积分]）做最优状态反馈，Q/R 权衡 | 上升快、抗扰好、增益解析可得（给定模型即给定增益）、可处理多状态 | 需要被控对象线性模型；模型失配时性能退化需鲁棒化；传感器噪声放大 | SBW 角跟踪对比研究的主流结论：LQR 响应更快、超调更低 |
| **前馈（速度前馈 + 动力学前馈）** | 反馈环外叠加：`τ_ff = J·α_ref + b·ω_ref + 负载估计`；速度前馈在位置环前加 `ω_ref` | 大幅降低反馈增益负担、缩小相位滞后；阶跃/斜坡类指令几乎消除跟踪误差 | 需要模型参数（J、b）与负载估计；估计错会变成干扰 | 所有高性能跟踪的结构性组件；与 LQR/PID 正交、可叠加 |

### 2.2 非主流 / 进阶方案

| 方案 | 思路 | 适用与代价 |
|---|---|---|
| **扰动观测器（DOB）** | 名义逆模型 + 低通滤波重构外部扰动（回正力矩、摩擦）并在控制输入前馈抵消 | 对抗负载扰动/参数摄动非常有效；结构简单；与 PID/LQR 组合使用是文献常见形态（KAIST 2024 论文即 DOB + 滑模用于 SBW 齿条位置控制） |
| **自抗扰（ADRC）** | 扩张状态观测器（ESO）把"未建模动力学 + 外部扰动"总包成一个扩张状态实时估计并补偿 | 模型依赖最低、抗扰最强；调参维度 3–4 个但物理含义不如 PID 直观；ESO 带宽受噪声限制（IEEE 文献：突变扰动下 ESO 相位滞后需改进） | 
| **滑模（SMC）** | 定义滑模面（误差及其导数），强鲁棒收敛 | 抗扰与鲁棒性极强、有限时间收敛；抖振需处理（边界层/高阶滑模）；工程落地多用于故障/极端工况 | 
| **MPC** | 在线优化带约束（角度/速率/力矩限幅）的预测控制 | 显式处理饱和与约束、可多目标；计算量大、实时性靠显式 MPC/短时域；转向执行器层面属于"杀鸡用牛刀"但在线控冗余/容错场景有文献 | 
| **鲁棒 H∞ / μ 综合** | 把参数摄动建模进设计 | 保证最坏情况性能；保守、增益阶数高；主要用于论证性对比（清华 2024：LQR/鲁棒/H∞ 响应快但误差大于级联 PI） | 

### 2.3 自动 / 半自动调参

| 方法 | 原理 | 本项目的适配性 |
|---|---|---|
| **模型解析 / 极点配置** | 已知 J、b、回路结构 → 直接解增益（LQR 属此类） | **首选**：仿真内被控对象参数完全已知，先解析后微调 |
| **继电器反馈辨识（Åström–Hägglund）** | 注入继电器自激振荡，测极限环频率/幅值 → 临界增益/周期 → ZN 类整定 | 成熟（Hang-Åström-Wang 2002 综述）；需在仿真内跑辨识流程，可做成"自动整定"工具；对位置类积分对象有已知局限需修正（文献：对伺服对象用带纯延时的改型） |
| **贝叶斯 / 网格 / 遗传优化（黑盒）** | 在仿真里批量跑工况、按指标集评分寻优 | 与现有 study 扫掠基础设施天然契合；成本是评估次数；适合"半自动"——给定参数先验 + 边界，输出帕累托面 |
| **增益调度** | 按车速/负载工况查表切换增益 | 必做：转向负载随车速强变化（停车 scrub vs 高速回正力矩），任何反馈方案的增益都应支持按 v 调度 |

### 2.4 调研结论（用于需求冻结）

1. **第一版控制库**：`open_loop`（现状行为，作基准）→ `pid_single` → `pid_cascade`（量产形态）→ `lqr` → 前馈组件（速度前馈 + 动力学前馈，作为可插拔叠加层）。**前馈与反馈正交**，所有反馈方案都可挂前馈。
2. **第二版（进阶）**：DOB、ADRC、滑模。它们是对抗"模型失配 + 强扰动"的答案，正好服务于"评估不同策略差异"的终极目标——差异恰恰在扰动抑制与鲁棒性上拉开。
3. **调参**：三层递进——解析增益（模型已知）→ 继电器辨识（半自动）→ 黑盒优化（复用 study 扫掠）。所有调参结果落盘、可复现、带参数空间余量（复用 C5 的 margins 方法论）。
4. **评估矩阵**：同一套工况 × 全部控制器，输出统一的阶跃/正弦/扰动指标表，直接复用 study 层的过程指标机制（oncentre 分析器的先例）。

---

## 3. 需求草案

### 3.0 层定位（边界）

- **控制层只管"跟"**：输入 = 目标角（可带前馈速度/加速度），输出 = 执行器力矩/速率指令，目标 = 实际角跟踪目标角。**"目标角怎么算出来"（策略/RWS 模式/轨迹规划）不是本层职责**。
- **手感合成（SBW 反馈作动器）不在本层**：现有 `ByWirePlant` 的路感合成回路保持独立，控制层只动角度通道。
- **默认行为不变契约**：控制层默认挂 `open_loop` 且参数与现 `ByWirePlant` 带宽/限幅一致，位级不变契约（`test_disabled_plant_changes_nothing`）与黄金实验全部保持。

### 3.1 功能需求（FR）

| 编号 | 需求 | 验收口径 |
|---|---|---|
| FR-1 | **控制器接口**：统一 `AngleTrackingController` 协议——`step(dt, target_angle, target_rate, feedback_angle, feedback_rate, load_torque, speed_kmh) → (actuator_torque_cmd | rate_cmd, diagnostics)`。所有控制器实现同一协议，可在运行中切换 | 新控制器零改动接入模型；切换无跳变（bumpless 要求） |
| FR-2 | **控制库 v1**：open_loop / pid_single（P/PI/PID）/ pid_cascade（位置环+速度环）/ lqr / 前馈组件（速度前馈、动力学前馈，可叠加到任何反馈） | 每方案有单元测试钉住增益↔响应的解析关系 |
| FR-3 | **控制库 v2（进阶）**：dob / adrc / smc | 与 v1 同接口；扰动抑制指标显著优于 v1 基准（量化为指标差距而非口头） |
| FR-4 | **执行器被控对象**：把"角度通道"从带宽+限幅升级为可控制对象——`J·θ̈ + b·θ̇ + F_c·sign(θ̇) + τ_load = τ_actuator`，τ_actuator 经电机包络（峰值/转速反电动势/热），负载来自现有 rack_force/回正力矩链。保留速率限幅作为可选约束 | 阶跃/正弦响应与手算二阶系统吻合（解析钉住测试） |
| FR-5 | **逐轮执行通道**：按架构生成执行器——4wis 四角各一、sbw 前轴一个、sbw_rws 前轴+后轴、eps_rws 机械前轴+后轴执行器。删除前轴左右平均与"后轮永远走 steer_actuator"分支 | 4wis 架构下四轮角度可独立跟随不同目标；`architecture.check_capabilities` 与执行器存在性一致 |
| FR-6 | **位置反馈传感器建模**：逐轮实际转角反馈通道，带可配置量化（如 0.01°）/延迟（1 个控制周期）/噪声（可关） | 噪声开启时不同控制器指标差异方向合理（如 LQR 噪声敏感度高于级联 PI） |
| FR-7 | **多速率**：控制层以 0.5 ms 内步运行（复用 `STEERING_INNER_DT` 插值骨架），目标角在 5 ms 内插值、负载保持，输出在 5 ms 边界对齐 | 0.5 ms 与 0.25 ms 参考的指标差 < 2%（沿用 D5 的收敛判据） |
| FR-8 | **增益调度**：增益/前馈按车速查表（线性插值） | 停车（scrub）与高速（回正）两档工况下同一控制器都满足指标 |
| FR-9 | **自动/半自动调参**：a) 解析（给定模型参数直接解增益）；b) 继电器辨识流程（仿真内注入、自动整定 PID）；c) 黑盒优化（参数空间扫掠 + 指标评分，复用 study 层）。全部可复现、落盘、带来源 | 一条命令产出"控制器 X 在工况集 Y 上的整定结果 + 指标表 + 参数空间余量" |
| FR-10 | **评估协议**：阶跃（上升/超调/稳态误差/调节时间）、正弦扫频（带宽、幅值衰减、相位滞后）、负载扰动（阶跃/正弦回正力矩下的误差恢复）、噪声敏感度。作为 study 过程指标族落地 | 生成控制器 × 工况对比表与 HTML 报告 |
| FR-11 | **Simulink 接口**：外部控制器接入的两种通道——(a) FMI 2.0 共仿真（Simulink 导出 FMU，sim4wis 作 co-simulation master，控制周期按内步调用）；(b) 代码生成（Embedded Coder 产物经 C ABI 包装）。接口契约：输入 [target_angle, target_rate, feedback_angle, feedback_rate, load_est, v]、输出 [actuator_cmd]、参数表、初始化/步进/复位三原语 | 用一个示例 FMU 跑通"Simulink 控制器在 sim4wis 里完成一次阶跃评估"；未发货前如实标注（仓库"失败不伪装"惯例） |
| FR-12 | **诊断与记录**：每控制器输出诊断（饱和标志、积分状态、控制量、跟踪误差），通道进入记录体系；`steer_angle_deviation` 语义升级为"控制层闭环后的残余偏差" | 分析页可直接对比各控制器的 `steer_angle_deviation` 曲线 |

### 3.2 非功能需求（NFR）

| 编号 | 需求 |
|---|---|
| NFR-1 | **可复现**：所有控制器确定性；同一 spec 两次运行逐位一致（研究层惯例） |
| NFR-2 | **性能**：控制层 0.5 ms × 4 角，纯 numpy/标量实现，单步微秒级；全工况评估耗时增量可控（复用 S5 并行） |
| NFR-3 | **失败不伪装**：控制器发散/饱和/参数越界 → 明确诊断通道，不静默降级 |
| NFR-4 | **测试契约**：每个控制器至少一条"解析钉住"测试（给定增益与线性对象，响应与解析解一致）；数值红线沿用位级不变契约 |
| NFR-5 | **文档**：`docs/steering_control_guide.md`——控制器目录、调参指南、评估协议、Simulink 接入说明 |

### 3.3 里程碑建议

| 里程碑 | 内容 | 验收 |
|---|---|---|
| M1 骨架 | FR-1/4/5/6/7 + open_loop 位级不变 + 阶跃评估协议 + 解析钉住测试 | 4wis 四轮独立通道就绪；默认行为与黄金全绿 |
| M2 控制库 v1 | FR-2（pid_single/cascade/lqr + 前馈）+ FR-8 调度 + FR-10 完整评估矩阵 | 首次"控制器 × 工况"对比报告：级联 PID 稳态误差最小、LQR 最快——与文献结论同向 |
| M3 调参工作台 | FR-9 三通道 + 参数空间余量（C5 方法论复用） | 一键整定 + 复现 + 余量报告 |
| M4 外部控制器 | FR-3（dob/adrc/smc 至少其一）+ FR-11 Simulink 接口 | FMU 示例闭环评估通过 |
| M5 4WIS 专项 | 四轮异目标跟随 + 单轮失效/卡死场景下控制层行为 | 故障场景指标对比（衔接 fault_reconfig） |

---

## 4. 需求方确认的决策（已冻结）

1. **执行器细度**：电流环不建模——控制层接口到**力矩指令**，执行器对象为"位置+速度"二阶（J·θ̈ + b·θ̇ + 库仑摩擦 + 负载 = 饱和力矩），电机包络沿用 `motor.py` 的思路（峰值/速率限幅）。
2. **逐轮执行通道改造（FR-5）在 M1 范围**：删除前轴左右平均、后轮建立执行器通道。
3. **Simulink 接口只做框架**：当前开发机无 Simulink，本层先落**接口契约 + 抽象层**（控制周期、输入/输出、参数表、初始化/步进/复位三原语），具体 FMU 加载/代码生成在生产环境再按实际情况实现，不预先过设计。
4. **不做手感**：反馈作动器/路感合成不在本层，保持现有实现。
5. **自动调参验收**：工况集加权指标不劣于人工解析整定的 **110%**，过程全自动可复现。
6. **评估工况集**：ISO 13674 中心区 + 停车 scrub + 高速避让三类，与现有 procedures 对齐；**工况模块后续有大改**——评估协议做成可插拔的工况族，不为当前三工况写死。
7. **前馈量来源**：控制层内部对目标角求导（带滤波）产生前馈量；接口预留显式 rate/accel 字段供策略层将来供给。
8. **补充——前馈模块三件套（积木化、可选、可配置）**：
   - **动力学/齿条力前馈**：`τ_ff_load = 齿条力（回正力矩/负载）的折算估计`，直接前馈抵消已知负载；
   - **速度前馈（请求速度前馈）**：`τ_ff_vel = b·ω_ref`（+ 可选的 `J·α_ref` 加速度项）；
   - **系统级摩擦补偿前馈**：`τ_ff_fric = F_c·sign(ω_ref)`（对已知库仑摩擦的补偿，可带速度死区/斜率形状）。
   三者都是独立"积木"，可单独或组合叠加到任意反馈控制器上，全部可在 spec 中配置开关与参数。
9. **补充——控制库扩充**：除 v1 的 open_loop / pid_single / pid_cascade / lqr 外，**MPC、滑模（SMC）、ADRC、H∞ 均列入正式范围**（与 DOB 一起作为进阶库），与 v1 同接口。

---

## 5. 参考文献（调研来源）

- [Comparison of Various Angle-Tracking Algorithms to Balance Performance and Noise for a Steering-by-Wire System, Int. J. Automotive Technology 25(3), 2024（清华；LQR/鲁棒/级联 PI 对比）](https://www.kci.go.kr/kciportal/ci/sereArticleSearch/ciSereArtiView.kci?sereArticleSearchBean.artiId=ART003083096)
- [Directional Control of Wheel Synchronization in Vehicle Steer-by-Wire System, Mekatronika 4(1), 2022（PID vs LQR 前轴同步）](https://umpir.ump.edu.my/id/eprint/35407/)
- [CN115771559B 双电机线控转向系统及其主动容错控制方法（LQR + 变速比 + 容错）](https://patents.google.com/patent/CN115771559B/en)
- [Hierarchical Hybrid Steering Control for Four-Wheel-Steering Vehicles Considering System Delays, IEEE TVT 74(2), 2025（前馈 + 自适应 LQR）](https://pure.bit.edu.cn/en/publications/hierarchical-hybrid-steering-control-for-four-wheel-steering-vehi/)
- [Robust Fault-Tolerant Control for Dual-Motor Steer-by-Wire with Delays and Motor Faults（鲁棒 LQR + 前馈 vs NMPC/SMC）](https://www.mdpi.com/search?q=lane-change+prediction)
- [Steering Angle Control of Distributed-Drive EVs Based on ADRC, Proc. IMechE Part D（ADRC vs PID/PD）](https://doi.org/10.1177/0954407020944288)
- [Adaptive ADRC for SbW under Communication Time Delays（ESO 相位滞后问题的改进）](https://repository.uobaghdad.edu.iq/articles/8RfDZZIBVTCNdQwCSa64)
- [Steering Feedback Torque & Rack Position Control of SbW, KAIST 2024（DOB + 滑模，HILS 验证）](https://koasas.kaist.ac.kr/handle/10203/331975)
- [Relay Feedback Auto-tuning of Process Controllers — A Tutorial Review（Åström–Hägglund 继电整定综述）](https://www.sciencedirect.com/science/article/abs/pii/S0959152401000257)
- 链路现状部分基于本仓库代码：`controller/base.py`、`vehicle/steering_link.py`、`vehicle/geometry.py`、`steering/bywire.py`、`steering/architecture.py`、`core/simulator.py`
