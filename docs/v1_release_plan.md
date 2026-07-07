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
- 下一步：把单轮失效关键场景压缩成 2-3 个快速黄金样本，避免完整 170-run 报告成为唯一回归证据。
- 下一步：给黄金实验基线更新建立 CHANGELOG 模板，说明指标变化是模型修正、参数修正还是数值漂移。

### P1: 外部对照

- 选 2-3 个公开或可导出的参考工况：稳态圆周、阶跃转向、双移线。
- 对每个工况保存参考来源、车辆参数映射、目标指标和误差解释。
- 优先对照 `yaw_rate_peak_dps`、稳态横摆增益、速度跟踪 RMS、侧偏角峰值、轨迹横向偏差。
- 目标是把运动学/轮胎/简化动力学关键能力从内部 L2/L3 推进到更强的 L3 参考证据。

### P2: 浏览器自动化

- 引入 Playwright，先覆盖页面是否可打开和关键控件是否渲染。
- 第一批场景：运行页、试验页加载种子实验、分析页打开已有 run、车辆页三张几何图、负载页、原理页。
- 第二批场景：拖拽车辆几何控制点、触发实验批跑、回放时间轴、命令面板导航。
- 通过截图/DOM 断言减少手册截图和人工端到端验证的遗漏。

### P3: 研究报告流水线

- 已完成：`scripts/reporting.py` 抽出图片 base64 和 JSON 写入工具。
- 下一步：把 HTML 章节拼接、图表索引、指标表格渲染继续抽成小型 report kit。
- 下一步：让新的故障研究脚本复用同一套报告骨架，避免每个研究复制一份 HTML 模板。

### P4: 前端结构与性能

- 对 `Canvas3D`、分析页、车辆几何工作室、模型原理页做动态 import。
- 给 Vite chunk warning 建立门槛：允许已知 3D 包偏大，但主 bundle 不应持续增长。
- 把全局样式按页面/组件逐步收敛，避免新增页面改动影响现有工具面板。

### P5: 实物验证接入

- 为缩比 4WIS K&C 平台或台架数据预留 `validation_data/` 结构。
- 明确数据格式：工况定义、车辆/机构参数、传感器通道、采样率、滤波口径。
- 建立仿真 vs 实测对照脚本后，再把对应能力从 L3 推进到 L4。

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
