# 4WIS Simulator 可信度矩阵

本文记录当前仿真平台各能力的验证等级、证据位置和已知边界。它不是宣传材料；用途是帮助后续开发判断哪些结论可以直接复用，哪些结论只能作为内部研究假设。

## 等级定义

| 等级 | 含义 |
|---|---|
| L0 概念 | 有设计意图或界面入口，但缺少自动化验证。 |
| L1 单元 | 有单元测试覆盖核心函数或局部不变量。 |
| L2 集成 | 通过后端/前端集成测试、smoke 或端到端人工验证。 |
| L3 参考对照 | 与理论解、工程公式、基准模型或独立工具做过定量对照。 |
| L4 实测对照 | 与实车、台架或缩比验证平台数据做过定量对照。 |

## 当前能力状态

| 能力 | 当前等级 | 证据 | 主要边界 |
|---|---:|---|---|
| 运动学 4WIS 策略 | L3 | `backend/tests/test_geometry.py`, `backend/tests/test_strategies.py`, `scripts/smoke_test.py` 的瞬心一致性和圆周闭合检查，`validation_data/analytic_steady_circle_30kmh/` 解析稳态圆周对照，`validation_data/analytic_step_steer_30kmh/` 解析阶跃转向对照 | 主要验证几何一致性、解析稳态圆周和无执行器滞后的阶跃转向几何响应，不代表执行器、轮胎瞬态或实车精度。 |
| 简化动力学模型 | L2 | `backend/tests/test_time_domain_models.py`, `backend/tests/test_load_transfer.py`, smoke 动力学稳定性检查 | 已覆盖阻力、升力、toe、camber、坡道和载荷转移，但仍缺外部车辆模型对照。 |
| 多体动力学模型 | L2 | smoke 静平衡、侧倾、俯仰、坡度回归；`backend/tests/test_time_domain_models.py` | 14 DOF 为工程近似模型，尚未与 CarSim/CarMaker 或台架数据对齐。 |
| 轮胎与载荷敏感度 | L3 | `backend/tests/test_tire.py`, `backend/tests/test_time_domain_models.py`, `CHANGELOG.md` v0.12 记录 | `c_alpha(Fz)` 指数模型来自工程假设；缺同款轮胎实测曲线。 |
| 主销/齿条负载分析 | L3 | `backend/tests/test_kingpin.py`, `backend/tests/test_rack_force.py`, `backend/tests/test_load_analysis.py`, `docs/load_analysis_handoff.md` | LS9 参数和机构效率仍是标定快照；负载页不等价于完整台架校准。 |
| 实验批跑与 KPI | L2 | `backend/tests/test_experiment_batch.py`, `frontend/src/components/ExperimentPage.tsx`, `frontend/src/components/AnalysisPage.tsx` | KPI 足够用于内部比较；跨版本稳定性由黄金实验回归补充。 |
| 黄金实验回归 | L2 | `scripts/check_golden_experiments.py`, `docs/golden_experiments.json` | 当前覆盖 step steer、ISO 3888 DLC 和 3 个单轮失效快速样本；仍缺外部基准对照。 |
| run 回放与分析页 | L2 | `frontend/tests/e2e/workflow-smoke.spec.ts`, `frontend/src/components/ReplayPanel.tsx`, `frontend/src/charts/uplotFactory.ts` | 已有浏览器 smoke 覆盖实验到分析页链路、通道增删/加图、hover cursor、drag-to-zoom/双击复位、PNG 导出、工作区与关键 workflow 页截图 attachment、回放时间轴 scrub、负载页图表绘制/原理说明弹窗、场景页标准路径生成/跟踪/清除，以及分析页 run 数据读取失败、试验页 batch 启动失败、车辆页参数应用被拒绝异常态；仍需扩展更多深层页面状态和更少见后端失败分支截图。 |
| 单轮失效安全研究 | L3 | `scripts/study_single_wheel_failure.py`, `docs/reports/single_wheel_failure_safety_analysis.html`, `backend/tests/test_fault_reconfig.py` | 可支撑内部机制研究，不应直接作为实车 ISO 26262 认证证据。 |
| 手柄映射与直控模式 | L2 | `backend/tests/test_manual_strategies.py`, `frontend/src/input/gamepadConfig.ts`, `frontend/tests/e2e/workflow-smoke.spec.ts`, 手册 GIF | 配置 UI 和直控策略有自动化覆盖；浏览器 Gamepad API 和设备轴序仍依赖真实硬件回归。 |
| 车辆几何工作室 | L2 | `frontend/src/vehicle/geometryModel.ts`, `frontend/src/components/vehicle/*`, `frontend/tests/e2e/workflow-smoke.spec.ts`, `CHANGELOG.md` v0.16 记录 | 已有浏览器 smoke 覆盖 SVG 拖拽点写入参数编辑缓冲；前端几何数学与后端部分共享概念但不是同一语言实现，仍需跨端一致性测试。 |
| 便携包发布 | L2 | `scripts/build_portable.py`, `dist_portable/`, GitHub Release assets | 依赖 python-build-standalone 和目标平台 wheel 可用性；发布前必须跑 `scripts/pre_release_check.py`。 |
| 参考/外部/实测对照接入 | L3 | `docs/reference_benchmark_protocol.md`, `validation_data/README.md`, `validation_data/analytic_steady_circle_30kmh/`, `validation_data/analytic_step_steer_30kmh/`, `scripts/check_reference_benchmarks.py`, `backend/tests/test_reference_benchmarks.py` | 已有两个解析 benchmark，`--require-data` 可通过；尚无真实外部工具或实测数据，因此不能提升到外部工具对照或 L4 实测等级。 |

## 当前最高风险

1. 外部对照不足：已有稳态圆周和阶跃转向两个解析 benchmark，但多数结论仍缺 CarSim/CarMaker、公开基准或实测数据。
2. 浏览器端自动化仍偏 smoke：关键 workflow 页截图、车辆几何拖拽、分析页截图/图表增删/hover cursor/drag-to-zoom/PNG 导出、负载页图表绘制/原理说明弹窗、场景页标准路径生成/跟踪/清除、run 数据读取失败异常态、试验页 batch 启动失败异常态、车辆页参数应用被拒绝异常态、回放时间轴、命令面板导航和手柄配置编辑已有覆盖，但更多深层页面状态和更少见后端失败分支仍主要靠人工端到端验证。
3. 安全研究边界需要持续显式化：报告结论应始终标注参数假设、机构假设和不可外推范围。
4. 发布资产一致性要机械化：版本号、lockfile、构建产物、手册和 release asset 需要同一套检查清单约束。

## 下一步提高建议

1. 继续做外部基准对照：在已有解析稳态圆周和阶跃转向 benchmark 之外，至少再接入 ISO 3888 双移线或 CarSim/CarMaker 导出结果，形成更完整的 L3 证据包。
2. 扩展 Playwright：继续覆盖更深页面状态和异常态截图，减少人工 UI 回归成本。
3. 接入真实 reference 数据：checker 已能汇总误差表和 reviewer notes；下一步是放入真实外部/实测 benchmark，并由人工审查误差解释。
4. 抽象研究报告流水线：后续新增故障研究脚本应复用 `scripts/reporting.py` 的文档外壳、章节、callout、表格和图片组件，避免复制整页模板。
5. 接入实物验证数据：缩比 4WIS K&C 平台或台架数据进入后，将关键模型从 L3 推进到 L4。
