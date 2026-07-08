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
- 已完成：reference checker 可用 `--report` 生成 reviewer-facing Markdown，汇总来源、limitations、warnings/failures、指标误差表和 `notes.md` 摘要；报告明确不自动提升可信度等级。
- 已完成：reference checker 新增 `--require-independent-source`，可在审查时强制要求至少一个通过检查的 `external_tool`、`bench`、`scaled_vehicle` 或 `full_vehicle` benchmark，避免解析参考被误当成独立外部/实测来源。
- 已完成：接入解析参考 benchmark `validation_data/analytic_steady_circle_30kmh/`，覆盖 kinematic ideal-Ackermann 稳态圆周，对照横摆率、侧向速度、速度误差、横向位移和轨迹误差 6 个指标。
- 已完成：接入第二个解析参考 benchmark `validation_data/analytic_step_steer_30kmh/`，覆盖 kinematic ideal-Ackermann 30 km/h 阶跃转向，对照横摆峰值、横摆增益、上升/稳定时间、侧向速度、速度误差、横向位移和轨迹误差 9 个指标。
- 已完成：生成并纳入 `docs/reports/reference_benchmark_review.md`，作为当前 reviewer-facing 审查件；报告明确当前通过项都是解析参考，独立外部/实测 benchmark 数量仍为 0。
- 继续补齐公开或可导出的参考工况：ISO 3888 双移线，以及至少一个外部工具或实测来源。
- 对每个新增工况保存参考来源、车辆参数映射、目标指标和误差解释。
- 优先对照 `yaw_rate_peak_dps`、稳态横摆增益、速度跟踪 RMS、侧偏角峰值、轨迹横向偏差。
- 目标是把运动学/轮胎/简化动力学关键能力从内部 L2/L3 推进到更强的 L3 参考证据；L4 仍必须等待台架、缩比车或实车数据。

### P2: 浏览器自动化

- 已完成：引入 Playwright workflow smoke，覆盖生产 app shell 中运行页、试验页、分析页、车辆页、场景页、负载页、原理页的关键渲染，并纳入 `scripts/pre_release_check.py`。
- 已完成：Playwright 现在会从试验页启动默认种子实验，等待 batch 完成，并通过“去分析页”链路验证已有 run 的 KPI 对比、通道叠图渲染、通道 chip 增删、下拉加图、hover cursor、PNG 导出和分析工作区截图 attachment。
- 已完成：Playwright 现在覆盖车辆几何 SVG 拖拽点写入共享参数编辑缓冲、分析页回放时间轴 scrub、命令面板键盘导航、手柄配置编辑。
- 已完成：Playwright 覆盖分析页回放 meta 读取失败 fallback，断言回放卡显示默认 LS9 尺寸提示且 canvas/时间轴仍可用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖分析页 uPlot 通道叠图的 drag-to-zoom 与双击复位，断言 X 轴可见窗口按交互收缩/恢复。
- 已完成：Playwright 为运行、试验、空分析、车辆、场景、负载、原理页生成 workflow screenshot attachment，并对截图非空做基本断言。
- 已完成：Playwright 覆盖分析页 run 列表读取失败异常态，断言错误不会伪装成空 run 库、刷新按钮仍可用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖试验页实验库读取失败异常态，断言错误不会伪装成空实验库、刷新/新建按钮仍可用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖试验页机动模板读取失败异常态，断言参考路径模板错误可见、参考路径选择器仍可用、运行矩阵不被阻塞，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖分析页 run 数据读取失败异常态，断言错误 toast、保留 KPI 表可读，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖试验页 batch 启动失败异常态，断言错误 toast、运行矩阵仍可操作，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖运行页模型切换和路面 μ 设置失败异常态，断言错误 toast、控制面板保持可操作，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖运行页 Python 策略状态读取失败异常态，断言设计面板显示“状态未知”与后端错误，不再伪装成文件不存在，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖车辆页参数应用被拒绝异常态，断言错误 toast、未应用编辑仍保留，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖车辆页项目列表读取失败异常态，断言错误停留在项目面板、加载按钮禁用、车辆几何图仍可见，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖车辆页项目 YAML 加载失败异常态，断言错误停留在项目面板、加载按钮恢复可操作、车辆几何图仍可见，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖负载页图表 canvas 绘制和图表原理说明弹窗，生成深层页面状态 screenshot attachment。
- 已完成：Playwright 覆盖负载页扫图计算失败异常态，断言错误 toast、计算控件恢复可操作，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖负载页车型库读取失败异常态，断言错误 toast、页面主体不空白且参数/计算控件仍可用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖负载页当前参数读取失败异常态，断言错误 toast、底盘参数区保持可理解空态且重新计算禁用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖原理页交互 demo 后端计算失败异常态，断言错误留在对应 demo 区域、页面主体不空白，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖场景页标准路径生成、跟踪策略切换和路径清除，生成场景路径 workflow screenshot attachment。
- 已完成：Playwright 覆盖 App 顶层 path_version 触发的参考路径刷新失败异常态，断言错误 toast 可见且场景路径工具仍可用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖 App 顶层 scenario_version 触发的场景几何刷新失败异常态，断言错误 toast 可见且场景面板工具仍可用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖场景页路径模板读取失败异常态，断言错误停留在轨迹/路径面板、生成按钮禁用、刷新模板和手动绘制仍可用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖场景页场景列表读取失败异常态，断言错误停留在场景路况面板、刷新按钮仍可用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖场景页路面扰动编辑的新建、选中编辑、参数应用和清空，生成扰动编辑 workflow screenshot attachment。
- 已完成：Playwright 覆盖场景页故障注入面板的故障添加、停用/启用和清空，生成故障注入 workflow screenshot attachment。
- 已完成：Playwright 覆盖场景页故障列表读取失败异常态，断言错误不会伪装成空故障配置、刷新故障和添加入口仍可用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖运行页动作脚本库的脚本载入、启动、参考路径/桩布局铺设和停止，生成脚本库 workflow screenshot attachment。
- 已完成：Playwright 覆盖运行页动作脚本库读取失败异常态，断言错误不会伪装成空脚本库、刷新库按钮仍可用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖运行页动作脚本状态读取失败异常态，断言错误停留在脚本面板、脚本编辑和刷新库仍可用，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖动作脚本解析失败异常态，断言错误保留在脚本面板且原 YAML 仍可编辑，并生成异常态 screenshot attachment。
- 已完成：Playwright 覆盖运行页数据录制面板的开始录制、采样数增长、停止录制和 CSV 导出，生成录制 workflow screenshot attachment。
- 已完成：Playwright 覆盖运行页数据录制 CSV 导出失败异常态，断言错误停留在录制面板、导出按钮恢复可用，并生成异常态 screenshot attachment。
- 下一批场景：更多深层页面状态和更少见后端失败分支截图。
- 通过截图/DOM 断言减少手册截图和人工端到端验证的遗漏。

### P3: 研究报告流水线

- 已完成：`scripts/reporting.py` 抽出图片 base64 和 JSON 写入工具。
- 已完成：`scripts/reporting.py` 扩展为小型 report kit，集中提供 HTML 属性转义、表格单元格、HTML table 与内嵌 PNG figure；单轮失效报告的指标表和图片嵌入已改为复用这些 helper。
- 已完成：`scripts/reporting.py` 提供 `ReportDocument` 自包含 HTML 报告外壳；单轮失效报告已复用该骨架，重跑脚本后已确认生成报告内容无实质变化。
- 已完成：`scripts/reporting.py` 提供章节、摘要/提示框和元信息 helper；单轮失效报告的摘要、关键结论、工具迭代记录和结论章节已迁移到这些复用组件，重跑脚本后确认指标/图片无变化。
- 下一步：把后续新增的故障研究脚本接到同一套 report kit，继续减少整页模板复制。

### P4: 前端结构与性能

- 已完成：对试验页、分析页、车辆几何工作室、负载页、模型原理页做动态 import；主 JS chunk 从约 1018 kB 降到约 609 kB。
- 已完成：运行页非默认侧栏组改为首次打开时懒加载、之后保持挂载；实时曲线/uPlot、设计、验证、数据、场景编辑面板不再进入默认首屏主包。主 JS chunk 进一步降到约 502 kB。
- 已完成：`Canvas3D`/Three.js 依赖拆成懒加载 3D vendor chunks；构建期预算插件把默认入口 chunk 限在 500 kB 内、`vendor-three-core` 限在 700 kB 内。当前构建默认 `index` chunk 354.92 kB，最大 3D vendor chunk `vendor-three-core` 666.67 kB，生产构建无 chunk warning。
- 已完成：车辆几何工作室 `vg-*` 样式从全局 `styles.css` 拆到页面私有 CSS，随懒加载车辆页生成 `VehicleGeometryStudio` CSS chunk；全局 `index` CSS 从 47.98 kB 降到 44.06 kB。
- 已完成：负载页样式从全局 `styles.css` 拆到 `LoadAnalysisPage` 页面 CSS、`ChartBox`/解释弹窗/指标信息组件 CSS，以及模型页复用的 body-coupling 控件 CSS；全局 `index.css` 降到 27.92 kB，新增 `LoadAnalysisPage` CSS 10.26 kB 和共享 load 图表/控制 CSS 35.36 kB。
- 已完成：模型原理页 `model-*` 样式从全局 `styles.css` 拆到 `ModelTheoryPage` 私有 CSS；当前全局 `index.css` 降到 21.56 kB，新增 `ModelTheoryPage` CSS 6.41 kB。
- 已完成：试验/分析/车辆几何工作流共用的 `wf-*`、`veh-*`、`replay-*` 样式从全局 `styles.css` 拆到共享 `WorkflowPage` CSS；当前全局 `index.css` 降到 16.23 kB，新增 `WorkflowPage` CSS 5.42 kB。
- 已完成：命令面板 `cmdk-*`、手柄配置 `gp-*`、Panel 壳层 `panel-*` 和快速开始卡片 `quickstart-*` 样式从全局 `styles.css` 拆到组件私有 CSS；这些组件仍在入口依赖链，当前入口 `index.css` 为 16.23 kB / gzip 3.48 kB。
- 已完成：App 壳层、顶部栏、工作流 rail、运行页侧栏和页面加载态样式从全局 `styles.css` 拆到 `App.css`；当前入口 `index.css` 为 15.68 kB / gzip 3.40 kB。
- 已完成：视口切换、Canvas 容器/加载态和 2D/3D 共用 HUD 样式从全局 `styles.css` 拆到 `Viewport.css` 与共享 `CanvasHud.css`；当前入口 `index.css` 为 15.68 kB / gzip 3.41 kB。
- 已完成：Panel 内容共享控件（图表块、策略按钮、读数、按钮行、参数列表等）从全局 `styles.css` 拆到 `PanelContent.css`；全局 `styles.css` 仅保留主题 token、reset 和 `mono`/`small` 工具类，当前入口 `index.css` 为 15.68 kB / gzip 3.41 kB。
- 下一步：后续新增样式默认放在页面/组件私有 CSS；若继续压缩全局面，只评估 `mono`/`small` 这类工具类是否需要替换为局部样式或设计 token。

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
