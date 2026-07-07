# 4WIS Simulator

四轮独立转向（4-Wheel Independent Steering）工程研究平台。当前版本是 `v0.16.0`，默认车辆标定为智己 LS9，已经形成从实时驾驶、参数建模、实验批跑、KPI 分析、run 回放、安全研究报告到便携版发布的闭环。

项目仍定位为内部预研和工程分析工具，不是经过实车标定或认证的安全结论工具。模型可信度和边界见 [docs/validation_matrix.md](docs/validation_matrix.md)。

## 当前能力

- 实时仿真：运动学、简化动力学、多体动力学三档模型，支持 2D/3D 视图、路面扰动、坡道、减速带、split-μ、bump-steer。
- 控制策略：阿克曼、理想阿克曼、后轮转向、蟹行、零半径、轨迹跟踪、故障重构、手动逐轮和全向车身控制。
- 实验系统：YAML 实验定义、无头批跑、策略/车速/参数变体矩阵、run 落盘、KPI 后端化。
- 分析系统：run 库、KPI 对比、通道叠图、轨迹叠图、幽灵车回放、命令面板。
- 车辆建模：车辆参数页、项目保存加载、三张可拖拽几何工作室图（整车、主销/车轮、前桥齿条硬点）。
- 输入系统：键盘、YAML 动作脚本、Web Gamepad API，支持前后轴/左右侧/逐轮/蟹行/全向等手柄映射预设和实时校准。
- 安全研究：单轮转向失效注入、自由脚轮/锁死机构差异、容错重构策略、参数敏感性流水线、自包含 HTML 报告。
- 发布交付：macOS / Windows 便携包、图文用户手册、GitHub Release 资产。

## 目录结构

```text
4WIS Simulator/
├── backend/              Python 仿真后端（FastAPI + WebSocket）
├── frontend/             React + TypeScript + Vite 前端
├── docs/                 设计、手册、可信度矩阵、研究报告
├── experiments/          可复现实验定义
├── projects/             车辆/场景项目文件
├── runs/                 实验运行结果资产
├── scripts/              启动、smoke、手册、发布检查、便携打包脚本
├── scripts_lib/          YAML 动作脚本库
└── vehicle_profiles/     车辆标定快照
```

## 快速开始

### 一键启动

```bash
python scripts/start.py
```

启动后浏览器打开 `http://127.0.0.1:8010/`。

### 开发模式

后端：

```bash
cd backend
python -m pip install -e ".[dev]"
uvicorn sim4wis.main:app --reload --port 8010
```

前端：

```bash
cd frontend
npm install
npm run dev
```

浏览器打开 `http://127.0.0.1:5173/`。如果前端需要代理到非默认后端端口，设置 `SIM4WIS_BACKEND_HTTP` 和 `SIM4WIS_BACKEND_WS`。

## 主要工作流

### 实时驾驶与场景验证

1. 打开「运行」页，选择模型和策略。
2. 在「场景」页配置路面、轨迹、扰动和故障。
3. 用键盘、手柄或 YAML 脚本输入。
4. 观察 2D/3D 视图、瞬心、轮胎状态、曲线和评分面板。

### 实验批跑与分析

1. 在「试验」页载入或编辑 `experiments/*.yaml`。
2. 设置策略、车速或车辆参数变体矩阵。
3. 批量运行后跳转「分析」页。
4. 对比 KPI、通道曲线、轨迹和幽灵车回放。

### 车辆几何建模

1. 在「车辆」页打开几何工作室。
2. 拖拽整车、主销/车轮、前桥齿条硬点图上的控制点。
3. 检查派生量、红旗提示、转弯圆和阿克曼误差。
4. 点击「应用」后把编辑缓冲提交到仿真参数。

### 安全研究复现

```bash
backend/.venv/bin/python scripts/study_single_wheel_failure.py
```

输出位于 `docs/reports/`，包括 `single_wheel_failure_safety_analysis.html` 和指标 JSON。该报告用于内部机制研究，不应直接作为实车 ISO 26262 认证证据。
报告脚本依赖 backend dev 环境中的 `matplotlib`；首次复现前请确保已安装 `backend[dev]`。

## 验证

发布前建议跑完整检查：

```bash
python scripts/pre_release_check.py
```

该脚本会检查版本一致性、前端 lockfile、后端 pytest、smoke、黄金实验 KPI 回归、外部参考数据结构、前端 type-check、生产构建和 Playwright 浏览器 smoke。

常用单项命令：

```bash
backend/.venv/bin/python -m pytest tests/        # 后端全量测试
backend/.venv/bin/python scripts/smoke_test.py   # 从仓库根目录运行 smoke
backend/.venv/bin/python scripts/check_golden_experiments.py
backend/.venv/bin/python scripts/check_reference_benchmarks.py
backend/.venv/bin/python scripts/check_reference_benchmarks.py --report docs/reports/reference_benchmark_review.md
backend/.venv/bin/python scripts/check_reference_benchmarks.py --require-independent-source  # 需要真实外部/实测来源时使用
cd frontend && npm run type-check
cd frontend && npm run build
cd frontend && npm run e2e       # 先生产构建，再跑 Playwright，避免 stale dist
cd frontend && npm run e2e:prod  # 仅在已构建 dist 后直接跑 Playwright
```

当前 `v0.16.0` 验证基线：

- 后端：`210 passed`
- smoke：`32/32 通过`
- 黄金实验：`step_steer_60kmh`、`iso3888_dlc_60kmh` 和 3 个单轮失效快速样本 KPI 回归通过
- 外部参考：`analytic_steady_circle_30kmh` 和 `analytic_step_steer_30kmh` 两个解析 benchmark 通过；`docs/reports/reference_benchmark_review.md` 已生成当前 reviewer-facing 审查材料；`--require-independent-source` 仍会失败，直到接入 CarSim/CarMaker、公开基准或实测数据
- 前端：type-check 通过
- 浏览器 smoke：Playwright Chromium `20 passed`，覆盖工作流渲染、试验到分析页交接、分析页通道切换/加图/hover cursor/drag-to-zoom/PNG 导出/截图证据、车辆几何拖拽、车辆页项目列表/加载失败异常态、数据录制开始/停止/CSV 导出/导出失败异常态、分析回放时间轴、场景路径/扰动/故障 workflow、脚本库/解析失败、命令面板导航、手柄配置编辑
- 前端生产构建通过且无 Vite chunk warning；默认 `index` chunk 354.67 kB，低于 500 kB 入口预算；懒加载 3D vendor 最大 chunk `vendor-three-core` 666.67 kB，低于 700 kB 3D core 预算；入口 `index.css` 为 15.68 kB / gzip 3.41 kB，车辆几何、工作流页面、负载页、模型页和共享 load 图表/控制样式拆为独立 CSS chunks，App 壳层、视口/HUD、命令面板、手柄配置、Panel 壳层、Panel 内容控件和快速开始卡片样式已收敛到组件私有 CSS

首次运行 Playwright 前需要安装浏览器运行时：

```bash
cd frontend
npx playwright install chromium
```

## 便携版打包

先构建前端：

```bash
cd frontend
npm run build
```

再生成便携包：

```bash
python scripts/build_portable.py --targets macos-arm64 windows-x64
```

已有 runtime/vendor 时可用快速同步：

```bash
python scripts/build_portable.py --targets macos-arm64 windows-x64 --update
```

输出在 `dist_portable/`。发布到 GitHub Release 时优先使用 ASCII 文件名。

## 版本与发布约定

版本号至少需要同步：

- `backend/src/sim4wis/__init__.py`
- `backend/pyproject.toml`
- `frontend/package.json`
- `frontend/package-lock.json`

推荐发布顺序：

1. 更新版本号和 CHANGELOG。
2. 运行 `python scripts/pre_release_check.py`。
3. 构建便携包。
4. 生成或刷新 `docs/user_manual.html`。
5. 上传 macOS/Windows 便携包、quickstart、用户手册和关键研究报告到 GitHub Release。

## 重要文档

- [CHANGELOG.md](CHANGELOG.md) - 版本演进和每次发布的真实功能边界。
- [docs/user_manual.html](docs/user_manual.html) - 图文用户手册，含手柄 GIF、实验页、分析页、几何工作室。
- [docs/validation_matrix.md](docs/validation_matrix.md) - 当前能力可信度、证据和边界。
- [docs/v1_release_plan.md](docs/v1_release_plan.md) - v1.0 收敛路线和完成定义。
- [docs/reference_benchmark_protocol.md](docs/reference_benchmark_protocol.md) - 外部工具/实测数据对照协议。
- [docs/reports/reference_benchmark_review.md](docs/reports/reference_benchmark_review.md) - 当前参考 benchmark 审查报告，明确解析参考通过但独立外部/实测来源为 0。
- [docs/golden_baseline_changelog.md](docs/golden_baseline_changelog.md) - 黄金实验基线更新说明模板和变更记录。
- [docs/v1_platform_refactor_plan.md](docs/v1_platform_refactor_plan.md) - 平台化重构路线。
- [docs/load_analysis_handoff.md](docs/load_analysis_handoff.md) - 转向负载分析页面和 API 交接说明。
- [docs/reports/single_wheel_failure_safety_analysis.html](docs/reports/single_wheel_failure_safety_analysis.html) - 单轮失效功能安全研究报告。
- [docs/vehicle_model_refactor_plan.md](docs/vehicle_model_refactor_plan.md) - 整车模型重构主线。
- [docs/tire_model.md](docs/tire_model.md) - 轮胎模型说明。
- [docs/fmu_integration.md](docs/fmu_integration.md) - FMU 集成说明。
- [docs/scripting.md](docs/scripting.md) - YAML 动作脚本说明。
