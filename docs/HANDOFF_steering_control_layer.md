# 转向角度跟随控制层 — 阶段交接文档

- **交接日期**：2026-08-15
- **交接人**：本阶段开发（ZCode 会话）
- **接手对象**：后续开发 agent / 工程师
- **阶段范围**：转向控制层从需求冻结到 FR-10 三臂评估齐备 + 执行器柔度建模起步
- **仓库**：`4WIS_Simulator/4WIS Simulator`，分支 `main`

---

## 0. 一句话交接

转向角度跟随控制层（`backend/src/sim4wis/steering/tracking/`）已经是一套**带完整评估协议的产品级骨架**：9 种控制器、3 类前馈积木、三通道调参工作台、阶跃/扫频/扰动三臂评估、Simulink 接口契约、真实工况验证矩阵 45/45 全绿、全套 879+ 测试通过。接手者应**先读 §5 的"血泪教训"再动手**——那里每一条都对应一个已修复的真实缺陷。

---

## 1. 已交付清单（含提交）

| 提交 | 内容 |
|---|---|
| `e929491` + `900e570` | **M1 骨架**：控制器协议、力矩型角执行器（刚性）、角度传感器、按架构逐轮执行通道、`open_loop` 基准（与旧线控逐位一致）、4 条 `steer_corner_deviation_*` 通道 |
| `7b118f7` | **M2 控制库 v1**：`pid_single` / `pid_cascade` / `lqr`（积分增广离散 Riccati，Newton 法）+ 前馈三件套（rack_force/velocity/friction，积木化）+ 车速增益调度 + 阶跃评估（`trk_*` 指标族 + procedure）；默认增益由 `scripts/tune_vehicle_defaults.py` 在车辆闭环调出 |
| `9eeebae` | **M3 调参工作台**：解析（极点配置）→ 继电辨识（Åström–Hägglund 带滞环）→ 黑盒精修（Nelder-Mead），110% 验收门 |
| `fbbf16a` | **M4 进阶库**：`dob` / `adrc` / `smc` / `mpc` / `h_inf` + Simulink 接口框架（端口契约 + 三原语 + `file_gain` 参考适配器） |
| `73fae90` | 需求挖掘与调研文档（已冻结） |
| `389bb67` | **真实工况验证轮**：9 控制器 × 5 工况矩阵（`scripts/validate_tracking_scenarios.py`）全绿；修复执行器尺寸 40→120 N·m、SMC 可达性、DOB 双重补偿、两处性能 |
| `1660fa1` | **设计+测试报告**（`docs/reports/steering_control_layer_report.md`） |
| `23a694a` | **迭代轮 1+2**：进阶控制器车辆闭环调参（settle 全部进 v1 档，MPC 全场最优）、ADRC 双重补偿互斥、ESO 精确离散化、力矩限对齐、扫频评估（`trk_*` 频域族） |
| `ad55424` | **负载扰动臂**（FR-10 三臂齐备）：冰面 μ 阶跃工况、`trk_dist_*` 指标族；SMC 二次可达性修复（k=60） |
| *（本轮，待提交）* | **方向 3 执行器柔度起步**：双质量传动 + 间隙（默认刚性位级不变），解析钉住测试过，柔度闭环失稳作为发现记录 |

---

## 2. 代码地图

```
backend/src/sim4wis/steering/tracking/
├── __init__.py         # 导出 + 注册控制器模块（必须导入才注册）
├── controller.py       # AngleTrackingController 协议、TrackingOutput、open_loop、注册表、make_controller
├── controllers.py      # v1 库：pid_single / pid_cascade / lqr + GainSchedule + pole_place_pid + solve_dare(Newton+倍平方) + _expm_taylor
├── advanced.py         # dob / adrc / smc / mpc / h_inf + hildreth_qp + solve_game_care
├── feedforward.py      # 三件套积木 + FeedforwardStack + make_feedforward
├── plant.py            # CornerActuatorPlant：刚性单质量（默认）/ 双质量+间隙（方向3）
├── feedback.py         # AngleSensor：量化 + 管线延迟 + 种子噪声
├── coupling.py         # CornerTracker（控制器+执行器+传感器）、make_corner_trackers、corner_tracking_step
├── tuning.py           # 调参工作台（step_cost / relay_identify / zn_pid / tune）
└── external.py         # Simulink 接口契约 + file_gain 参考适配器

backend/src/sim4wis/study/
├── tracking_step.py        # 阶跃过程指标（trk_rise/overshoot/settle/ss_err/peak_dev）
├── tracking_sweep.py       # 扫频指标（幅值比/相位/带宽）
├── tracking_disturbance.py # 扰动指标（onset/peak/recover）
└── metrics.py              # 指标注册（ANALYSES / PROCEDURE 字典）

procedures/
├── tracking_step_response.yaml        # 阶跃评估（4 控制器 × 1°@30km/h）
├── tracking_sweep_response.yaml       # 扫频评估（0.5–10 Hz）
└── tracking_disturbance_response.yaml # 扰动评估（9 控制器 × 冰面 μ 阶跃）

scripts/
├── validate_tracking_scenarios.py  # 9×5 真实工况矩阵（**任何改动后必跑**）
├── tune_vehicle_defaults.py        # 车辆闭环调参（发布默认增益的出处）
└── devtools/step_table.py          # 9 控制器统一阶跃表
```

**关键接线**：`dynamic.py` / `multibody.py` 的 `__init__` 调 `make_corner_trackers(params)`（层默认关→None→legacy 路径位级不变）；`step()` 里 `corner_tracking_step` 推进逐轮执行器；`steering_link.STEERING_CHANNELS` 含 4 条逐轮偏差通道。`AngleControlParams`（`steering/params.py`）是层配置，经 `params_codec.py` 的嵌套块映射从 YAML overrides 注入。

---

## 3. 使用方式

```bash
# 跑一次阶跃评估（4 控制器对比）
sim4wis study run procedures/tracking_step_response.yaml
# 跑一次扰动评估（9 控制器对比）
sim4wis study run procedures/tracking_disturbance_response.yaml
# 9×5 真实工况矩阵（改任何控制器后必跑）
backend/.venv/bin/python scripts/validate_tracking_scenarios.py
# 车辆闭环调参（改执行器参数后重调默认增益）
backend/.venv/bin/python scripts/tune_vehicle_defaults.py
# 9 控制器统一阶跃表
backend/.venv/bin/python scripts/devtools/step_table.py
```

在 study spec 里启用控制层：

```yaml
vehicle:
  overrides:
    steering_system:
      enabled: true
      architecture: 4wis        # sbw / sbw_rws / eps_rws / 4wis
      angle_control:
        enabled: true
        controller: lqr         # open_loop/pid_single/pid_cascade/lqr/dob/adrc/smc/mpc/h_inf
        controller_kwargs: {…}  # 增益/结构参数；ff: {blocks: [...]} 挂前馈
        sensor_quant_rad: 0.00017
        sensor_delay_steps: 1
```

---

## 4. 当前验证状态（交接时点）

| 门禁 | 状态 |
|---|---|
| 后端全量（并行 `-n 24`） | **879 passed**（方向 3 增量提交后待复跑确认，预计 880+） |
| 黄金回归 | 6/6 |
| 阶跃 procedure | 判据 PASS（4/4） |
| 扫频 procedure | 判据 PASS（4/4） |
| 扰动 procedure | 判据 PASS（9/9） |
| 真实工况矩阵 | **45/45** |
| lint（仓库惯例：`backend/src` + `backend/tests` + 我新增脚本） | 干净（`scripts/` 下历史文件有存量 lint 问题，不在惯例内，勿批量 --fix） |

**默认行为契约（红线）**：`angle_control.enabled` 默认 False，开启后 legacy 路径位级不变；`open_loop` 与旧线控逐位一致。任何改动必须保持这两条 + 黄金 6/6。

---

## 5. 血泪教训（接手者必读——每条都是已修复缺陷的教训）

1. **DOB 与 ADRC 都是"观测器即负载补偿器"**：与 rack_force/friction 前馈块**互斥**（构造期拒绝）。观测器无法从指令力矩区分"抵消负载的力矩"与"负载本身"——叠前馈=双重补偿，轮子漂 0.26–0.6 rad。
2. **SMC 滑动可达性要真算**：|负载|<k 是硬条件。k=12 输给 100 km/h 直行回正（~12 N·m）；k=25 输给冰面持续转角负载（~40 N·m，无负载前馈时）。当前 k=60（覆盖 ~3 kN 齿条力）。**给 SMC 挂 rack_force 前馈前想清楚**（它不互斥，但可达性余量变了）。
3. **观测器/规划器的力矩限必须对齐执行器**：ESO 积分"以为施加的力矩"，未限幅指令 vs 已限幅对象=模型失配发散。adrc/dob/h_inf 的 `torque_limit`、mpc 的 `u_max` 默认都对齐 120 N·m。**改执行器峰值必须同步改这些**。
4. **ESO 要精确离散，不要显式欧拉**：ωo·h ≳ 0.2 就采样脆弱（大角度发散，小角度工况会掩盖）。当前是 ZOH 精确离散 + Ackermann 极点配置（z=exp(−ωo·h)）。**改采样率（`INNER_RATE_HZ`）必须重算离散矩阵**——`_a_d/_b_d/_l` 是构造期算的。
5. **LQR 的 Riccati 用 Newton 法**：跟踪对象含纯积分器（z=1 边缘特征值），朴素定点迭代收敛到反稳定解。Lyapunov 子问题用倍平方（对数收敛，否则 5.6 s/角）。
6. **齿条力前馈别加低通**：负载本身已滞后一步（5 ms 外环），再叠 15 Hz 低通把 LQR 超调 0.3%→3.8%、settle 1.28→2.5 s。要滤波用高频截止并重新评估。
7. **执行器峰值是设计参数不是占位符**：40 N·m 被 0.5 g 回正负载（≈40 N·m）打穿，轮子漂 −0.34 rad。120 N·m 是按选型模块算的（全锁停车 ≈104 N·m/角）。**想改回小执行器 = 重新做选型 + 全量重调**。
8. **MPC 的盒限是权重的一部分**：u_max 从 40→120 后旧权重解整体失效（超调 12.6%），必须按新盒限重调。QP 矩阵已在构造期预计算（改 horizon/权重后自动重建，无需手动）。
9. **扫频分析的带宽插值精度受频段间距限制**（2→5 Hz 跨膝 ~9%）；阶跃分析的峰值偏差从 90% 后起测（因果系统上升期偏差恒等于阶跃幅值）；扰动分析的恢复时间从**阶跃起点**测、尾样本在带外=如实报"未恢复"。
10. **`ruff --fix` 不要扫整个 `scripts/`**：历史脚本有一堆存量 lint 问题，曾误改 25 个文件（已回滚）。只 lint 自己动过的文件。

---

## 6. 未完成 / 待办（给接手者）

### 6.1 方向 3 收尾（进行中，本轮已提交代码+测试）
- **已就绪**：双质量传动 + 间隙被控对象（`plant.py`，默认刚性位级不变）、参数接线、解析钉住测试（共振频率 1.6% 内、间隙死区、刚性位级不变）。
- **已知发现（如实记录）**：柔度开启（k=1200 N·m/rad，~18 Hz 共振）后，**现有默认增益全部失稳**（pid_single 超调 1518%、lqr 908%、adrc 1015%）。这是真实物理（高带宽反馈+未建模共振=经典激发），不是 bug。
- **待做**：① 柔度下的调参/鲁棒化（低通反馈、陷波、或把柔度纳入调参成本）；② 柔度+间隙作为评估维度的正式 procedure；③ 间隙对中心区手感的影响研究。

### 6.2 方向 4：调参工作台工程化
- `tune()` 接 CLI、多工况加权成本（当前单点阶跃成本）、与 study 扫掠打通（网格/贝叶斯）、调参结果入库 + 参数空间余量（复用 C5 margins）。

### 6.3 方向 5：Simulink 落地（需生产环境）
- 接口已冻结（端口契约 + init/invoke/shutdown 三原语 + 参数出处强制）。生产环境实现 FMI 2.0 共仿真加载器或 Embedded Coder C ABI 包装，替换 `external.py` 的 `_invoke`/`init_backend`。当前 `file_gain` 是跑通契约的参考适配器。

### 6.4 方向 6：超包线定量研究
- 5°@60 km/h（≈0.8 g）超出轮胎包线，深滑移回正反转可把 120 N·m 执行器钉在限位。定量研究"多大峰值才不被吹到限位"，反哺选型模块。

### 6.5 其他候选
- 阶跃/扫频 procedure 扩展到 9 控制器（当前阶跃/扫频只 sweep 4 个 v1；扰动已 9 个）。
- 执行器规格模型与选型模块闭环互证（`steering/sizing.py`）。

---

## 7. 关键文件索引

| 文件 | 内容 |
|---|---|
| `docs/steering_control_layer_requirements.md` | 需求基准（已冻结，§4 决策记录） |
| `docs/steering_control_guide.md` | 使用手册（控制器目录/调参/三臂协议/Simulink 契约） |
| `docs/reports/steering_control_layer_report.md` | 设计+测试报告（含三轮实测数据表） |
| `docs/reports/dev_report_2026-08-15.md` | 当日开发报告（含收尾线 S4/S5/H2/H3） |
| `CHANGELOG.md` 未发布节 | 全部变更记录（含每轮修复的实测数字） |

---

## 8. 复现基线（接手时先跑一遍确认环境）

```bash
cd backend && .venv/bin/python -m pytest tests/ -q -n 24
cd .. && backend/.venv/bin/python scripts/check_golden_experiments.py
backend/.venv/bin/python scripts/validate_tracking_scenarios.py
backend/.venv/bin/python -m sim4wis.cli study run procedures/tracking_disturbance_response.yaml
```

四个全绿 = 环境与交接时点一致。有任何数字对不上，先看 CHANGELOG 未发布节和 §5 教训。
