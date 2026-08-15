# 转向角度跟随控制层 — 阶段交接文档 v2（主线收尾）

- **交接日期**：2026-08-16 凌晨（主线收尾，待推送 GitHub）
- **交接人**：本阶段开发（连续会话）
- **接手对象**：后续开发 agent / 工程师
- **阶段范围**：自 v1 交接后，方向 3（柔度）收尾、方向 4（调参工程化）、方向 6
  （超包线）落地、执行器规格/选型闭环互证、共置反馈/刚度/抗间隙三项研究、
  评估三臂扩展至 9 控制器与柔度配置
- **仓库**：`4WIS_Simulator/4WIS Simulator`，分支 `main`（本阶段提交已推送 GitHub origin）

---

## 0. 一句话交接

转向角度跟随控制层已从"产品级骨架"推进到**主线收尾**：柔度维度三臂齐备且带
车辆级调参配方，调参工作台五通道（解析/继电/黑盒/网格/贝叶斯）+ 车辆环直连 +
参数余量 + 记录入库 + CLI，执行器规格与选型模块闭环互证（口径决策 + 峰值
120→260 N·m + 全限位对齐），超包线定量研究闭环。**三项研究（共置反馈 / 刚度
k 扫掠 / 抗间隙）均为诚实的实证结果**——其中两项是负结果，同样入档。当前
门禁：913 测试、黄金 6/6、矩阵 45/45、刚性三臂 9/9/9、柔度三臂 3/3/3。

---

## 1. 已交付清单（v2 阶段提交，倒序）

| 提交 | 内容 |
|---|---|
| `646f9d3` | 柔度扫频/扰动臂（`tracking_compliance_sweep/disturbance.yaml`，方向 3 三臂齐备） |
| `1aaf759` | 超包线工况矩阵（3/5/8°×60/80 @260 N·m） |
| `f7e59c7` | 抗间隙结构研究（轮侧手段证伪 + 杠杆清单） |
| `6d9b23c` | 传动刚度 k 扫掠研究（带宽要连结构阻尼一起买） |
| `2e39b42` | 电机侧传感器通道研究（共置反馈，实测负结果）+ 阶跃/扫频 procedure 扩到 9 控制器 |
| `3376624` | 方向 4 收口：贝叶斯通道 `bayesian_search` + study 扫掠直连 `tune_procedure` |
| `883ec68` | 执行器规格/选型闭环互证：pinion 等效口径决策 + 峰值 120→260 + 全限位对齐 |
| `bc24283` | 方向 4/6 启动：多工况成本/网格/余量/记录/CLI + 超包线峰值扫掠 |
| `c60fe1b` | 方向 3 收尾：柔度调参通道 + 车辆级复调 + 柔度阶跃臂 |
| `7cefc78` | 间隙量化（≤5 mrad 可接受 / 10 退化 / 20 越限） |
| `143d8c7` | 方向 3 起步：双质量传动 + 间隙被控对象 + v1 交接文档 |

（v1 阶段提交见 v1 交接文档原表，不在本表重复。）

---

## 2. 代码地图（v2 更新点）

```
backend/src/sim4wis/steering/
├── sizing.py            # size_corner_actuator()：三口径（rack 链/kingpin 直驱/tire_fy 旧口径）+ 80% 选型推荐
├── params.py            # plant_peak_torque_nm = 260（选型基础 207 ÷ 0.8）；传动三参数
└── tracking/
    ├── tuning.py        # 五通道：tune / relay / grid_search / bayesian_search / tune_procedure
    │                    # + step_cost(conditions=…)、tune_margins()、save_record()、共振安全解析种子
    ├── plant.py         # 双质量+间隙（默认刚性位级不变）；峰值默认 260
    ├── advanced.py      # dob/adrc/h_inf torque_limit、mpc u_max 默认 260；SMC k=60（残余口径）
    └── coupling.py      # pinion 等效轮域负载口径（rack × pinion，既定设计）

procedures/
├── tracking_step/sweep/disturbance_response.yaml   # 刚性三臂，9 控制器（dict 单元 sweep）
└── tracking_compliance_step/sweep/disturbance.yaml # 柔度三臂，3 个柔度调参配置

scripts/devtools/
├── compliance_tuning.py        # 柔度两阶段配方（角级表 → 车辆级多起点 NM）——柔度增益的出处
├── over_envelope_study.py      # 超包线峰值扫掠 + 工况矩阵（方向 6）
├── motor_channel_study.py      # 共置反馈三结构网格（负结果）
├── stiffness_sweep_study.py    # k 扫掠四问 + 采样极限（传动选型结论）
├── backlash_structure_study.py # 抗间隙轮侧手段证伪 + 杠杆清单
└── step_table.py               # 9 控制器统一阶跃表
```

---

## 3. 使用方式（收尾时点）

```bash
# 刚性三臂（各 9 控制器）
sim4wis study run procedures/tracking_step_response.yaml
sim4wis study run procedures/tracking_sweep_response.yaml
sim4wis study run procedures/tracking_disturbance_response.yaml
# 柔度三臂（3 个柔度调参配置）
sim4wis study run procedures/tracking_compliance_step.yaml
sim4wis study run procedures/tracking_compliance_sweep.yaml
sim4wis study run procedures/tracking_compliance_disturbance.yaml
# 真实工况矩阵（任何改动后必跑）
backend/.venv/bin/python scripts/validate_tracking_scenarios.py
# 调参工作台 CLI
sim4wis tune pid_single [--plant-json …] [--conditions-json …] [--grid N]
                         [--bayesian] [--procedure spec.yaml] [--margins]
                         [--out record.json] [--json]
# 研究脚本（确定性复现）
backend/.venv/bin/python scripts/devtools/{compliance_tuning,over_envelope_study,motor_channel_study,stiffness_sweep_study,backlash_structure_study}.py
```

---

## 4. 当前验证状态（收尾时点）

| 门禁 | 状态 |
|---|---|
| 后端全量（并行 -n 24） | **913 passed** |
| 黄金回归 | 6/6（位级契约不变） |
| 真实工况矩阵 | 45/45 |
| 阶跃 procedure | 9/9（扩展后全库） |
| 扫频 procedure | 9/9（带宽超上界的如实报"只知道下界"） |
| 扰动 procedure | 9/9 |
| 柔度阶跃/扫频/扰动 | 3/3、3/3、3/3 |
| lint | 干净（`backend/src` + `backend/tests` + devtools 脚本） |

**红线（不变）**：`angle_control.enabled` 默认 False 位级不变；`open_loop` 与旧线控
逐位一致；发布默认增益 = 刚性调参（柔度配置是评估维度，opt-in）。

---

## 5. 血泪教训（v1 的 10 条全部有效，v2 新增 6 条）

v2 新增（每条都对应一个实测数字）：

11. **外环带宽由传动反共振钉死**：轮位置环在任何结构下都跨传动弹簧闭环，
    共置速率反馈救不了（电机通道研究，~2–3 Hz 天花板），k 加倍带宽也只按
    √k 走——而且**光加刚度不够，共振随 ω_res 升高欠阻尼**，必须连结构阻尼
    （∝√k）一起给（k 扫掠研究）。
12. **接合冲击 ∝ k·间隙**：间隙冲击是扭矩阶跃（10 mrad → 12 N·m 打在 J_w 上），
    轮侧传感器接合前看不见——积分死区/降 kp/加 kd 全部无效或更糟。**轮侧无解**，
    要抗间隙就买精密齿轮箱（2–5 mrad）。
13. **选型数字的出处必须点名到量**："104 N·m（5.2 kN rack）"实为 tire_fy 误标，
    逐轮齿条链是 207 N·m——口径决策 + 出处审计比数字本身值钱。
14. **限位是否绑定要实测，不要推**：120→260 的峰值改动在全部评估工况从未绑定
    （5/5 逐位一致）→ 默认增益不用重调；反过来 40→120 时代是绑定的（12.6% 超调）。
15. **GP 通道的两个坑**：单观测种子在尖锐成本地形上无局部梯度（40 次评估零改善，
    改用 NM 起始单纯形种子）；成本跨数量级（不稳定区），观测必须逐轮标准化。
16. **车辆成本面多峰**：确定性多起点（角级种子 + 解析种子 + warm-start）比
    单起点可靠——两种子实测局部最优 0.43 vs 1.50。

---

## 6. 研究结论速查（负结果同样是交付）

| 研究 | 结论 |
|---|---|
| 电机侧传感器通道（共置反馈） | **负结果**：外环带宽被反共振钉死，共置结构 +10–20% 且更怕间隙；不落死代码 |
| 传动刚度 k 扫掠 | 固定 b 天花板 ~3 Hz；√k 阻尼下按 √k 释放至 12.6 Hz@k=38400；采样极限 ω_res·h≲0.2。**带宽要连结构阻尼一起买** |
| 抗间隙结构 | **负结果（轮侧）**：接合冲击 ∝ k·间隙，轮侧无解；杠杆 = 精密齿轮箱 / 电机过隙控制 / 更软 k |
| 超包线（方向 6） | 动态需求 81 N·m、≥60 N·m 不被吹离；"钉住 120"证伪（是 40 被吹飞）；默认 260 覆盖全地貌 |
| 规格/选型闭环 | pinion 等效口径成立；静态基础 207 N·m；峰值 260 N·m（80% 使用率）；kingpin 直驱 881 N·m 未采用 |

---

## 7. 关键文件索引

| 文件 | 内容 |
|---|---|
| `docs/HANDOFF_steering_control_layer.md` | 本交接文档（v2） |
| `docs/steering_control_layer_requirements.md` | 需求基准（已冻结，§4 决策记录） |
| `docs/steering_control_guide.md` | 使用手册（§3 三臂/§3e 超包线/§4 工作台五通道/§4b 柔度与三项研究/§6 边界） |
| `docs/reports/steering_control_layer_report.md` | 设计+测试报告（§7 方向 3 增补 / §8 收口增补含全部研究结论） |
| `docs/reports/dev_report_2026-08-15.md` | 当日开发报告（含晚场方向 3/4/6 全记录） |
| `CHANGELOG.md` 未发布节 | 全部变更记录（含每轮修复与研究的实测数字） |

---

## 8. 剩余 / 待办

1. **方向 5：Simulink 落地（用户明确搁置）**——接口已冻结，生产环境实现 FMI 2.0
   加载器或 Embedded Coder C ABI，替换 `external.py` 的 `_invoke`/init_backend。
2. **结构阻尼模型细化（研究衍生）**：被控对象的 b 目前是固定粘性值；把真实传动
   的结构阻尼项（c = 2ζ√(k·J)，ζ~0.01–0.05）建入 plant，k 扫掠结论即可变成
   可仿真的选型曲线。
3. **抗间隙的电机通道方案（若重启电机通道）**：过隙速率控制需要共置架构 + 慢轮侧
   微调双环——其权衡已在研究中量化，开工前先读两个研究脚本。
4. **新需求线**：需求挖掘子代理线程已在等用户回答 Q0（新想法是什么），与主线无关。
5. `docs/steering_control_guide.md` §4b 与报告 §7/§8 是柔度/研究结论的权威出处，
   改任何相关代码先对这两处。

---

## 9. 复现基线（接手时先跑）

```bash
cd backend && .venv/bin/python -m pytest tests/ -q -n 24
cd .. && backend/.venv/bin/python scripts/check_golden_experiments.py
backend/.venv/bin/python scripts/validate_tracking_scenarios.py
sim4wis study run procedures/tracking_step_response.yaml     # 9/9
sim4wis study run procedures/tracking_sweep_response.yaml    # 9/9
sim4wis study run procedures/tracking_compliance_step.yaml   # 3/3
```

全部绿 = 环境与交接时点一致。
