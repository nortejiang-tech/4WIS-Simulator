# 4WIS Simulator v1.0 收敛路线

本文把 v0.16.0 之后的工作从“继续加功能”收敛为“把工程研究平台做可信、可复现、可交付”。v1.0 不以 UI 数量或版本号为目标，而以验证闭环为目标。

## v1.0 完成定义

v1.0 只有在以下门槛都满足时才应发布：

1. **实验可信**：核心实验有黄金 KPI 回归，跨版本变化必须显式更新基线并解释原因。
2. **模型边界清楚**：每个核心模型能力在 `docs/validation_matrix.md` 中有等级、证据和边界。
3. **分析可复现**：安全研究脚本从同一套实验/报告工具生成数据、图表、HTML 和 sidecar JSON。
4. **前端可回归**：运行页、试验页、分析页、车辆几何工作室至少有 smoke 级浏览器自动化覆盖。
5. **发布机械化**：`scripts/pre_release_check.py` 通过；版本号、lockfile、测试、smoke、黄金实验、前端构建同一门禁。
6. **交付材料齐全**：README、用户手册、quickstart、便携包、关键研究报告和 GitHub Release assets 同步。

## 优先级路线

### P0: 可信回归底座

- 已完成：`docs/validation_matrix.md` 建立 L0-L4 可信度口径。
- 已完成：`scripts/check_golden_experiments.py` + `docs/golden_experiments.json` 覆盖 step steer 和 ISO 3888 DLC。
- 已完成：把单轮失效关键场景压缩成 3 个快速黄金样本，覆盖后轮跑飞锁死未缓解 C3、同工况缓解后 C2、前轮自由脚轮弯中 C2，避免完整 170-run 报告成为唯一回归证据。
- 已完成：`docs/golden_baseline_changelog.md` 建立黄金实验基线更新模板，要求说明指标变化是模型修正、参数修正、数值漂移还是实验定义变化。

### P1: 外部对照

- 已完成：建立 `docs/reference_benchmark_protocol.md` 与 `validation_data/` 数据接入结构，明确外部/实测证据不能用内部测试替代。
- 已完成：`scripts/check_reference_benchmarks.py` 可校验 benchmark 目录、manifest、CSV 最小通道、实验 YAML，并在有数据时运行 Sim4WIS 对比指标；默认空目录只提示外部证据缺失，`--require-data` 可强制失败。
- 选 2-3 个公开或可导出的参考工况：稳态圆周、阶跃转向、双移线。
- 对每个工况保存参考来源、车辆参数映射、目标指标和误差解释。
- 优先对照 `yaw_rate_peak_dps`、稳态横摆增益、速度跟踪 RMS、侧偏角峰值、轨迹横向偏差。
- 目标是把运动学/轮胎/简化动力学关键能力从内部 L2/L3 推进到更强的 L3 参考证据。

### P2: 浏览器自动化

- 已完成：引入 Playwright workflow smoke，覆盖生产 app shell 中运行页、试验页、分析页、车辆页、场景页、负载页、原理页的关键渲染，并纳入 `scripts/pre_release_check.py`。
- 已完成：Playwright 现在会从试验页启动默认种子实验，等待 batch 完成，并通过“去分析页”链路验证已有 run 的 KPI 对比和通道叠图渲染。
- 第二批场景：拖拽车辆几何控制点、触发实验批跑、回放时间轴、命令面板导航。
- 通过截图/DOM 断言减少手册截图和人工端到端验证的遗漏。

### P3: 研究报告流水线

- 已完成：`scripts/reporting.py` 抽出图片 base64 和 JSON 写入工具。
- 已完成：`scripts/reporting.py` 扩展为小型 report kit，集中提供 HTML 属性转义、表格单元格、HTML table 与内嵌 PNG figure；单轮失效报告的指标表和图片嵌入已改为复用这些 helper。
- 下一步：让新的故障研究脚本复用同一套报告骨架，避免每个研究复制一份 HTML 模板。

### P4: 前端结构与性能

- 已完成：对试验页、分析页、车辆几何工作室、负载页、模型原理页做动态 import；主 JS chunk 从约 1018 kB 降到约 609 kB。
- 已完成：运行页非默认侧栏组改为首次打开时懒加载、之后保持挂载；实时曲线/uPlot、设计、验证、数据、场景编辑面板不再进入默认首屏主包。主 JS chunk 进一步降到约 502 kB。
- 已完成：Vite chunk warning 门槛设为 650 kB；允许默认主包在 650 kB 以下，已知 `Canvas3D`/Three.js opt-in chunk 仍保持 warning 可见。
- 把全局样式按页面/组件逐步收敛，避免新增页面改动影响现有工具面板。

### P5: 实物验证接入

- 已完成：为缩比 4WIS K&C 平台或台架数据预留 `validation_data/` 结构。
- 已完成：明确数据格式：工况定义、车辆/机构参数、传感器通道、采样率、滤波口径。
- 已完成：建立仿真 vs 实测/参考对照脚本；真实台架或缩比车数据进入后，再把对应能力从 L3 推进到 L4。

## 非目标

- v1.0 前不追求大规模 UI 重做。
- v1.0 前不把安全研究报告包装成认证证据。
- v1.0 前不引入会改变模型结果的依赖升级，除非是明确的缺陷修复或安全修复。
- v1.0 前不把完整便携打包上传流程自动化到无人值守发布；GitHub Release 仍需人工确认资产。

## 每次合并前的检查清单

```bash
python scripts/pre_release_check.py
```

涉及模型或实验结果的改动还必须运行：

```bash
backend/.venv/bin/python scripts/check_golden_experiments.py
```

如果黄金基线需要更新：

```bash
backend/.venv/bin/python scripts/check_golden_experiments.py --update
git diff docs/golden_experiments.json
```

更新基线时，提交说明必须解释指标变化原因。
