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
   定尺寸（全锁停车 ≈104 N·m/角）→ 默认 120 N·m。
2. **SMC 滑动可达性被违反**：切换增益 12 N·m ≤ 100 km/h 直行回正负载
   （≈12 N·m），饱和后无回位力矩 → 增益 25 N·m。
3. **DOB 双重补偿**：齿条力前馈 + 观测器对同一负载补两次 → 互斥语义 +
   构造期拒绝。
4. **超包线发现（如实保留）**：5° @60 km/h ≈0.8 g 超出轮胎包线，深滑移区
   回正力矩反转可把 120 N·m 执行器吹到限位——这是执行器选型维度的问题，
   由偏差通道如实标记，不属控制器对比。

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

4WIS + 层开启，前轴 1° 保持阶跃，200 Hz 记录。每个 sweep 单元换一个控制器；
`trk_*` 指标从 `delta_cmd/delta` 通道提取过程量（fl 为被激励轮，rl 为串扰见证）：

- `trk_rise_s_fl` 上升时间（10→90%） · `trk_overshoot_pct_fl` 超调
- `trk_settle_s_fl` 调节时间（±2% 带） · `trk_ss_err_rad_fl` 稳态误差
- `trk_peak_dev_rad_fl` 峰值偏差（越过 90% 后，回路没消掉的偏差）
- `trk_ss_err_rad_rl / trk_peak_dev_rad_rl` 串扰见证（后轮应保持零）

首份对比（v0.102.0+，默认增益）：级联 PID 超调最小（1.3%），LQR 最快且稳态误差 4.7e-7 rad，open_loop 为无反馈基准。正弦扫频/负载扰动模板在工况模块扩展时另立（协议按工况族设计，不写死）。

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
