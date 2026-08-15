# 转向角度跟随控制层 — 使用指南

> 控制层的需求与设计决策见 [`steering_control_layer_requirements.md`](steering_control_layer_requirements.md)；
> 本文件是它的使用手册：控制器目录、调参指南、评估协议、Simulink 接入契约。

## 1. 层定位

策略层仍然决定"车轮该指向哪里"（`delta_cmd`，四轮独立）；控制层决定"作动器怎么跟过去"。启用方式（study spec 的 `vehicle.overrides`）：

```yaml
steering_system:
  enabled: true
  architecture: 4wis          # sbw / sbw_rws / eps_rws / 4wis
  angle_control:
    enabled: true
    controller: pid_cascade   # 控制器名，见 §2
    controller_kwargs: {…}    # 控制器构造参数（增益等）
    sensor_quant_rad: 0.00017
    sensor_delay_steps: 1
```

两条契约（测试钉住）：**关闭层位级不变**（`enabled: false` 时全部旧路径原样）；**open_loop 与旧线控更新逐位一致**——启用层本身零风险，open_loop 就是任何评估的天然基准。

## 2. 控制器目录

| 控制器 | 形态 | 特点与适用 |
|---|---|---|
| `open_loop` | 一阶带宽限幅 + 速率限幅（直接给角） | 旧线控行为逐位复现；**基准**，无反馈 |
| `pid_single` | 位置 P/PI/PID，力矩输出 | 实现与整定最直观；单环带宽受限 |
| `pid_cascade` | 位置 PI（外）→ 速度 PI（内） | **量产形态**；超调最小、电流/力矩最平滑 |
| `lqr` | 积分增广状态反馈（离散 Riccati） | 模型给定即增益给定；阶跃最快、稳态误差最小 |
| `dob` | PD 基底 + 扰动观测器（Q 滤波） | 负载阶跃恢复最好；**观测器本身就是负载补偿器——与 rack_force/friction 前馈块互斥**（构造期拒绝：同一扰动被补两次，实测轮子漂离指令 0.26-0.6 rad） |
| `adrc` | 线性自抗扰（ESO + 带宽参数化） | 模型依赖最低（只需 b0=1/J） |
| `smc` | 滑模（tanh 边界层） | 抗扰强、无抖振；稳态停留在边界层内 |
| `mpc` | 约束线性 MPC（Hildreth QP） | 唯一显式规划 `\|u\| ≤ u_max` 的成员 |
| `h_inf` | H∞ 状态反馈（game Riccati） | 唯一给出扰动衰减**界**（L2 增益 < γ）的成员 |
| `file_gain` | 外部控制器参考适配器（增益表） | Simulink/FMU 契约的占位实现（§5） |

前馈积木（可选、可配置，叠加到任何反馈控制器）：

```yaml
controller_kwargs:
  ff:
    blocks:
      - {type: rack_force, gain: 1.0, lowpass_hz: 0}   # 齿条力/动力学前馈
      - {type: velocity, damping: 4.0, inertia: 0.6}    # 速度+加速度前馈
      - {type: friction, friction_nm: 0.5}              # 摩擦补偿前馈
```

**注意**：齿条力本身已经滞后一步（5 ms 外环），在前馈上再加低通会把相位裕度推过悬崖——实测 15 Hz 低通使 LQR 超调 0.3%→3.8%、调节时间 1.28→2.5 s。默认不放低通；需要滤波时用高频截止并重新评估。

## 3. 真实工况验证

`scripts/validate_tracking_scenarios.py` 把全部 9 个控制器放进 5 类真实工况
（60 km/h 3° 阶跃 ≈0.5 g、ISO 3888 双移线 @60、中心区 weave @100、50 km/h
慢斜坡、原地全锁停车），逐角报告峰值/RMS 跟踪偏差并按架构自身的
angle_deviation 限（0.05 rad）判定。**当前全绿**（2026-08-15 轮）。

该轮验证抓到并修复的问题，都已成为设计约束：

1. **角执行器峰值 40 N·m 尺寸不足**：0.5 g 工况每角回正负载 ≈40 N·m 恰好
   压在极限上，轮子被吹离指令（实测漂到 −0.34 rad）。按仓库自己的选型模块
   定尺寸（全锁停车 ≈104 N·m/角）→ 默认 120 N·m。（该 104 的出处后来勘误：
   误把轮胎侧向力 tire_fy 当齿条力，静态基础实为 207 N·m；方向 6 收口后
   默认已改为 **260 N·m**——见 §3e。）
2. **SMC 滑动可达性被违反**：切换增益 12 N·m ≤ 100 km/h 直行回正负载
   （≈12 N·m），饱和后无回位力矩 → 增益 25 N·m。
3. **DOB 双重补偿**：齿条力前馈 + 观测器对同一负载补两次 → 互斥语义 +
   构造期拒绝。
4. **超包线发现（如实保留）**：5° @60 km/h ≈0.8 g 超出轮胎包线，深滑移区
   回正力矩反转可把执行器吹到限位——这是执行器选型维度的问题，由偏差通道
   如实标记，不属控制器对比。（"把 120 N·m 钉在限位"的推测已在方向 6 定量
   研究中证伪：120 N·m 从未被钉，被吹飞的是 40 N·m——见 §3e。）

## 3b. 扫频评估协议（FR-10 · 频域臂）

```bash
sim4wis study run procedures/tracking_sweep_response.yaml
```

60 km/h、0.2°（线性区）阶梯正弦 0.5→1→2→5→10 Hz。`trk_amp_ratio_*`、
`trk_phase_lag_2hz`、`trk_bw_hz`（−3 dB，对数插值；超出上界时只报下界并
拒绝给数）。首份：open_loop ≈10 Hz（设计参数即带宽）、LQR 更高、
PID/级联 3.8–4.1 Hz——带宽精度受频段间距限制（2→5 Hz 跨膝 ~9%）。

**进阶控制器默认值均为车辆闭环调参**（2026-08-15 第二轮）：dob/adrc/
mpc/h_inf 的阶跃 settle 0.055–0.13 s，与 v1 同档；mpc（15 ms 上升、
0 超调、55 ms settle）为当前全场最优。结构性约束：**dob 与 adrc 的
观测器就是负载补偿器**——与 rack_force/friction 前馈块互斥（构造期拒绝）。

## 3c. 负载扰动协议（FR-10 · 扰动臂）

```bash
sim4wis study run procedures/tracking_disturbance_response.yaml
```

60 km/h 直行后前轴 1.8°（≈0.31 g），车辆在角度稳定后驶入低 μ 冰面
（场景扰动 μ 0.85→0.35）——回正力阶跃即负载扰动（实测齿条力阶跃
1.1–2.1 kN）。**全员只用速度前馈**：负载补偿正是被比较的对象。
`trk_dist_*` 指标：扰动时刻（齿条力最大滑窗阶跃，中点计时）、峰值
跟踪偏差、回到 ±0.01 rad 的恢复时间（尾样本仍在带外 = 如实报"未恢复"）。

首份数据（2026-08-15）：峰值偏差全员 0.025–0.032 rad（因果系统挡不住
瞬时抛掷，彼此接近），**恢复时间分化**——adrc 0.045 s、smc 0.06 s
（观测器/滑移型）vs pid 0.14–0.145 s（积分器型，2–3 倍慢）。SMC 在
此工况暴露第二次可达性问题（k=25 < 冰面持续负载 ~40 N·m，漂出
0.129 rad），切换增益提至 60 N·m。

## 3d. 评估协议（FR-10 · 阶跃臂）

```bash
sim4wis study run procedures/tracking_step_response.yaml
```

4WIS + 层开启，前轴 1° 保持阶跃，200 Hz 记录。每个 sweep 单元是一个完整的
angle_control 配置（**9 控制器**：v1 三件套低通齿条力前馈、smc/mpc/h_inf 无低通、
dob/adrc 仅速度前馈）；`trk_*` 指标从 `delta_cmd/delta` 通道提取过程量（fl 为被
激励轮，rl 为串扰见证）：

- `trk_rise_s_fl` 上升时间（10→90%） · `trk_overshoot_pct_fl` 超调
- `trk_settle_s_fl` 调节时间（±2% 带） · `trk_ss_err_rad_fl` 稳态误差
- `trk_peak_dev_rad_fl` 峰值偏差（越过 90% 后，回路没消掉的偏差）
- `trk_ss_err_rad_rl / trk_peak_dev_rad_rl` 串扰见证（后轮应保持零）

首份对比（v0.102.0+，默认增益）：级联 PID 超调最小（1.3%），LQR 最快且稳态误差 4.7e-7 rad，open_loop 为无反馈基准。正弦扫频/负载扰动模板在工况模块扩展时另立（协议按工况族设计，不写死）。

## 3e. 超包线研究（方向 6）

`backend/.venv/bin/python scripts/devtools/over_envelope_study.py`：5°@60 km/h（≈0.8 g）
前轴阶跃 × 峰值扭矩扫掠（40–120 N·m，生产形控制器 pid_single+FF 与 open_loop）。
结论（逐轮齿条力 × 0.02 m pinion 口径）：

- 动态需求 ≈ **81 N·m**（峰值负载）；≥ **60 N·m** 即不被深滑移回正吹离——40 N·m 被吹飞
  0.698 rad 的原始发现已复现锚定，60 N·m 起 post-90% 偏差 0.026 rad、100+ N·m 后 ~0.008 rad。
- **静态口径勘误**：原"104 N·m（5.2 kN rack）"把轮胎侧向力 tire_fy 误标为齿条力；
  逐轮齿条链实为 ~**207 N·m/角**。
- **闭环互证结论（已落地）**：口径决策 = **pinion 等效轮域**（rack × pinion，
  coupling.py 既定设计）；kingpin 直驱口径下动态工况即 400–530 N·m、驻车 881 N·m——
  若要直驱模块需整体重设计，`size_corner_actuator()` 如实报告两个口径。
- **默认峰值 120 → 260 N·m**（选型口径修正后按 80% 使用率约定：207/0.8 = 258 → 260）。
  实测 120→260 的力矩限在全部评估工况从不绑定（5/5 逐位一致），发布默认增益无需重调；
  SMC 的 k=60 口径为"FF 补偿后的残余负载"，保持。动态模型泊车负载缺失（模型边界）
  意味着 260 只在准静态口径下被需求，仿真内不体现。

## 4. 调参工作台（FR-9）

Python API（`sim4wis.steering.tracking.tuning`）：

```python
tune("pid_single")            # 解析基线 → 黑盒精修 → 110% 验收门
relay_identify()              # Åström–Hägglund 继电辨识（带滞环）→ (Ku, Tu)
zn_pid(ku, tu)                # Ziegler–Nichols 表
step_cost("lqr", {...})       # 单点成本（ITAE 归一 + 峰值力矩惩罚）
```

三条通道：**解析**（模型给定增益即给定——极点配置/LQR）、**继电辨识**（半自动，对执行器对象做继电试验取 Ku/Tu）、**黑盒**（Nelder-Mead 精修，确定性）。验收：精修成本 ≤ 解析基线 × **110%**，不达标则如实报告解析增益，不"加冕"。全部可复现。

**发布默认值**由 `scripts/tune_vehicle_defaults.py` 在车辆闭环里调出（齿条力滞后一步 + 轮胎回正弹簧都在环内），该脚本是默认增益的出处；对新的执行器对象重新调参时，先在 plant 级用 `tune()`，再按此脚本在车辆闭环复调。

**工程化通道（方向 4）**：

- **多工况加权成本**：`step_cost(..., conditions=[{target, load_torque, t_end, weight}, ...])`
  以加权聚合替代单点阶跃成本（泊车负载 vs 高速负载一张成本表）；`tune()` 同名参数直达。
- **网格通道**：`grid_search("pid_single", points=4)` —— 轴空间均匀网格 + 同一条 110%
  验收门，与 Nelder-Mead 互为交叉验证（两通道应落在同一盆地）。
- **贝叶斯通道**：`bayesian_search("pid_single", max_evals=60)` —— 确定性 GP（固定
  超参）+ log-EI 获取函数、Halton 候选集，全程可复现，同一条 110% 验收门。
- **study 扫掠直连**：`tune_procedure("pid_single",
  "procedures/tracking_step_response.yaml")` —— 成本改由 study procedure（车辆环）给出，
  增益合入 spec 的 `controller_kwargs`（保留前馈栈）；即 `tune_vehicle_defaults.py`
  流程的通用化。
- **参数空间余量**：`tune_margins(controller, gains, ratio=1.5)` —— 逐增益二分"成本越过
  1.5× 名义值"的翻转点（C5 同款模式）；某个方向上界内不翻转 = 如实报 None（也是余量）。
- **入库与 CLI**：`save_record(result, path)` 落 JSON（含 git 出处、plant、conditions）；
  命令行 `sim4wis tune pid_single [--plant-json …] [--conditions-json …] [--grid N]
  [--bayesian] [--procedure spec.yaml] [--margins] [--out record.json] [--json]`。

## 4b. 柔度维度（方向 3：双质量传动 + 间隙）

被控对象支持双质量传动与间隙（`plant_transmission_stiffness_nms_per_rad` /
`plant_backlash_rad` / `plant_motor_inertia_fraction`，默认刚性、位级不变）。开启
柔度（k=1200 N·m/rad → ~17.8 Hz 共振）后：

- **默认增益全部失稳**（1° 阶跃超调 952/1322/1158%，pid_single/cascade/lqr）：轮侧
  速率反馈（PID 的 D 项、级联内环、LQR 的 ω 状态）从电机力矩侧激发双质量共振模态——
  非共置反馈的经典失稳，真实物理而非 bug。
- **鲁棒化通道**：速率通道低通（`deriv_tau_s`，柔度调参轴）+ 共振安全解析规则
  （`analytic_pid_compliant`：wn = ω_res/8、τ = 5/ω_res，网格钉住在稳定盆地内）。
  输出陷波（17.8 Hz）可行（12.9% @wn=15）但与 D 滤波相位相互作用、组合脆；电机侧
  速率阻尼（共置反馈教科书解，7.8% @wn=15）需电机侧传感器通道，留作下一阶段。
- **齿条力前馈别加低通**（教训 #6 的柔度版）：15 Hz 低通把柔度环的车辆级超调推到
  642%，去掉后 53%——柔度配置的前馈栈省略 `lowpass_hz`。

调参走工作台柔度通道（解析种子自动切换共振安全设计，`deriv_tau_s` 入轴，110% 验收
门不变）：

```python
tune("pid_single", plant_kwargs={"transmission_stiffness_nms_per_rad": 1200.0,
                                "peak_torque_nm": 120.0,
                                "motor_inertia_fraction": 0.2})
```

角级调参**不迁移到车辆环**（车辆横摆动态与柔度压低后的带宽同频）；两阶段配方与推荐
override 块见 `scripts/devtools/compliance_tuning.py`（角级表 → 车辆级 Nelder-Mead）。
车辆级正式评估：`sim4wis study run procedures/tracking_compliance_step.yaml`。

**间隙量化（方向 3 收尾 ③，柔度调参 pid_single + FF，车辆级 1° 阶跃）**：柔度环
容忍 ≤5 mrad 间隙（超调 0.1→3.0%，中心区极限环 0.0007 rad）；10 mrad 显著退化
（超调 35%、调节 2.0 s、中心区狩猎 0.012 rad）；20 mrad 越限（超调 192%、峰值
偏差 0.051 rad > 0.05 判据）。结论：精密齿轮箱（~2–5 mrad）可接受。**抗间隙结构研究**
（`scripts/devtools/backlash_structure_study.py`）：接合冲击 ∝ k·间隙（10 mrad →
12 N·m 阶跃打在 J_w 上），轮侧传感器在接合前看不见间隙——积分死区/条件积分
不动数字、kp 减半只把超调换成静差、kd 加大反而激发共振。**轮侧无解**；杠杆按
有效性排序：更紧的硬件间隙（精密齿轮箱）、电机通道过隙速率控制（需共置架构，
其自身权衡已实测）、更软 k（拿带宽换冲击）。均不落控制层死代码。

**电机侧传感器通道研究（共置反馈，实测负结果）**：`scripts/devtools/motor_channel_study.py`
对三种共置结构（电机速率阻尼 / 电机角位置反馈 / 共置级联）做网格钉住——**轮位置环
在任何结构下都跨传动弹簧闭环，外环带宽被反共振钉在 ~2–3 Hz**（k=1200 下），现有
轮侧设计（车辆级 wn≈20）已贴近天花板；共置结构最多 +10–20% 角级带宽，且间隙下更差
（电机角反馈根本看不见间隙：轮侧静差 14–129%）。结论：**电机通道不是本对象的根本解**，
真正的杠杆是更硬传动（抬高共振/反共振）或慢轮侧微调双环——均记录为硬件/设计项，
不落控制层死代码。

**柔度三臂齐备**：阶跃（`tracking_compliance_step.yaml`）之外，扫频
（`tracking_compliance_sweep.yaml`：带宽实测 1.1–5.2 Hz，0.5 Hz 幅值比
0.71–0.86——lqr 在 0.5 Hz 有车辆动态相互作用凹口；判据 0.65/0.7/相位 <60°）
与扰动（`tracking_compliance_disturbance.yaml`：峰值偏差 <0.05 全过；
cascade 纯 P 扰动后不回带、如实报"未恢复"——纯 P 外环的代价）。

**传动刚度 k 扫掠研究（`scripts/devtools/stiffness_sweep_study.py`，量化上述杠杆）**：
- 固定粘性阻尼（b=4 不随 k 缩放）下，稳定带宽天花板停滞在 ~2.5–3.1 Hz——共振随
  ω_res 升高越来越欠阻尼，规则设计（wn=ω_res/8、τ=5/ω_res）不可跨 k 迁移
  （7.1%→14→29→40→60→109% 超调）。
- 阻尼随 √k 缩放（真实皮带/齿轮的结构阻尼，ζ~0.01–0.05）时天花板随 √k 释放：
  2.5→3.1→4.4→6.3→12.6 Hz（k=1200→38400），直到采样极限 ω_res·h ≲ 0.2
  （2000 Hz 内环 → f_res ≲ 60 Hz）。
- 车辆级抽检（固定 b）：k=4800 规则增益 settle 0.30 s（vs k=1200 调参后 0.42 s）；
  更大 k 需连阻尼一起给。**传动选型结论：买带宽要连结构阻尼一起买，光加刚度不够。**

## 5. Simulink 接口（FR-11 · 框架）

当前交付**契约 + 参考适配器**，不包含真实 FMU 加载器（生产环境实现）：

- 端口契约（轮域）：in `target_angle / target_rate / feedback_angle / feedback_rate / load_torque / speed_ms`，out `actuator_torque`。
- 适配器三原语：`init_backend(workspace)` → 每控制周期一次 `_invoke(ports)` → `shutdown()`；控制周期由 `inner_rate_hz` 声明。
- 参数来源强制：`param_table` 逐参数带出处，无出处即拒绝运行（不伪装）。
- 参考实现 `file_gain`：JSON 增益表适配器，跑通完整契约；缺失即明确拒绝。
- 生产形态两通道：FMI 2.0 共仿真（sim4wis 作 master）或 Embedded Coder 产物经 C ABI 包装——两者都只替换 `_invoke`/`init_backend`，接口不再变化。

## 6. 已知边界（如实声明）

- 控制层只管"跟"：目标角怎么算（策略/RWS 模式）不是本层职责；手感合成不在本层。
- 执行器对象为位置+速度二阶（电流环不建模，按需求冻结）。
- 逐轮负载为 `rack_force × pinion_radius` 的一阶折算；更细的角执行器链路模型是 M2 的作动器规格工作。
- 传感器噪声默认关闭（可复现优先）；开启时各控制器噪声敏感度差异是评估维度之一。
- 柔度是评估维度不是发布配置：发布默认增益仍为刚性调参；柔度下轮侧反馈带宽须压到
  共振以下（车辆环有效上升 ~1 s），换执行器传动（k、惯量分配）必须重走柔度调参配方。
- 间隙对中心区手感的影响尚未研究（方向 3 剩余项）；电机侧传感器通道已实测证伪
  （§4b：外环带宽被反共振钉住，共置结构无实质收益且更怕间隙）——柔度环带宽的
  真实杠杆是更硬传动或慢微调双环，均为硬件/设计项。
- 转角执行器默认 260 N·m = 选型口径修正后的设计值（静态泊车基础 207 N·m ÷ 80%
  使用率）；kingpin 直驱口径（驻车 881 N·m）未采用，采用即需整体重设计（见 §3e）。
  动态模型泊车负载缺失（模型边界）意味着 260 在仿真内不会被需求。
