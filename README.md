# 4WIS Simulator

四轮独立转向（4-Wheel Independent Steering）工程研究平台。当前版本是 `v0.101.1`，默认车辆标定为智己 LS9，已经形成从实时驾驶、参数建模、实验批跑、KPI 分析、run 回放、安全研究报告到便携版发布的闭环。

项目仍定位为内部预研和工程分析工具，不是经过实车标定或认证的安全结论工具。模型可信度和边界见 [docs/validation_matrix.md](docs/validation_matrix.md)。

## 当前能力

- 实时仿真：运动学、简化动力学、多体动力学三档模型，支持 2D/3D 视图、路面扰动、坡道、减速带、split-μ、bump-steer。
- 3D 视角：自由 / 跟随 / **车顶**三挡相机（`C` 键循环）。车顶机位固定在车身上朝前，机位几何可调，是配方向盘/手柄跑工况的驾驶视角。
- 标准工况场地：直线、圆弧、绕桩、单移线、ISO 3888-1/-2 双移线、定圆(ISO 4138)、八字、侧方停车，按标准几何铺出锥桶/标杆和地面标线，车道宽和车位尺寸随当前车辆自适应。
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

不想装任何东西：从 [Releases](https://github.com/nortejiang-tech/4WIS-Simulator/releases)
下载对应平台的便携包解压，双击 `start.command`（macOS）或 `start.bat`（Windows）。
包里自带 Python 运行时和已构建的前端，无需安装 Python / Node，浏览器会自动打开
`http://127.0.0.1:8010/`。macOS 首次会被 Gatekeeper 拦截 → 右键点 `start.command`
→ 打开 → 再点"打开"。

有源码的话：

```bash
python scripts/start.py
```

首次运行会自动装依赖（`pip install -e backend` + `npm install`），然后起后端
（`:8010`）和 Vite 开发服务器（`:5173`），并打开 `http://localhost:5173/`。
加 `--build` 则改为构建前端并由后端单口托管，浏览器打开 `http://127.0.0.1:8010/`。

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

浏览器打开 `http://localhost:5173/`（Vite 在 macOS 上只监听 IPv6 回环，写死 `127.0.0.1`
会连不上）。如果前端需要代理到非默认后端端口，设置 `SIM4WIS_BACKEND_HTTP` 和 `SIM4WIS_BACKEND_WS`。

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

该脚本会检查版本一致性、前端 lockfile、交付材料清单、后端 pytest、smoke、黄金实验 KPI 回归、外部参考数据结构、参考 benchmark 审查报告一致性、v1 readiness 报告一致性、前端 type-check、生产构建和 Playwright 浏览器 smoke。

常用单项命令：

```bash
backend/.venv/bin/python -m pytest tests/        # 后端全量测试
backend/.venv/bin/python scripts/smoke_test.py   # 从仓库根目录运行 smoke
backend/.venv/bin/python scripts/check_golden_experiments.py
backend/.venv/bin/python scripts/check_reference_benchmarks.py
backend/.venv/bin/python scripts/check_reference_benchmarks.py --report docs/reports/reference_benchmark_review.md
backend/.venv/bin/python scripts/check_reference_benchmarks.py --check-report docs/reports/reference_benchmark_review.md
backend/.venv/bin/python scripts/check_reference_benchmarks.py --require-independent-source  # 需要真实外部/实测来源时使用
backend/.venv/bin/python scripts/scaffold_reference_benchmark.py carmaker_iso3888_dlc_60kmh --source-type external_tool --source-name CarMaker --source-version 14.0 --template iso3888_dlc_60kmh
backend/.venv/bin/python scripts/normalize_reference_csv.py --input raw_export.csv --output validation_data/.incoming/carmaker_iso3888_dlc_60kmh/reference.csv --map t=Time_ms --unit t=ms --map vx=Vx_kmh --unit vx=km/h --map vy=Vy_kmh --unit vy=km/h --map yaw_rate=YawRate_deg_s --unit yaw_rate=deg/s --map pose_x=X_mm --unit pose_x=mm --map pose_y=Y_mm --unit pose_y=mm --map driver_steering=Steer_deg --unit driver_steering=deg --crop-start-s 1.5 --crop-end-s 9.5 --zero-time
backend/.venv/bin/python scripts/suggest_reference_metrics.py validation_data/.incoming/carmaker_iso3888_dlc_60kmh
backend/.venv/bin/python scripts/suggest_reference_artifacts.py validation_data/.incoming/carmaker_iso3888_dlc_60kmh raw_source_export.csv --role "raw CarMaker CSV export before normalization"
backend/.venv/bin/python scripts/promote_reference_benchmark.py carmaker_iso3888_dlc_60kmh --dry-run  # incoming 数据补齐后先检查
backend/.venv/bin/python scripts/check_release_assets.py
backend/.venv/bin/python scripts/check_release_assets.py --require-portable-zips  # 发布收尾时使用
backend/.venv/bin/python scripts/check_v1_readiness.py --report docs/reports/v1_readiness.md
backend/.venv/bin/python scripts/check_v1_readiness.py --strict-v1 --require-portable-zips  # 声称 v1.0 前使用
cd frontend && npm run type-check
cd frontend && npm run build
cd frontend && npm run e2e       # 先生产构建，再跑 Playwright，避免 stale dist
cd frontend && npm run e2e:prod  # 仅在已构建 dist 后直接跑 Playwright
```

当前 `v0.101.1` 验证基线：

- 后端：`245 passed`
- smoke：`32/32 通过`
- 黄金实验：`step_steer_60kmh`、`iso3888_dlc_60kmh` 和 3 个单轮失效快速样本 KPI 回归通过
- 外部参考：`analytic_steady_circle_30kmh` 和 `analytic_step_steer_30kmh` 两个解析 benchmark 通过；`docs/reports/reference_benchmark_review.md` 已生成当前 reviewer-facing 审查材料，pre-release 会用 `--check-report` 防止该报告与 benchmark 数据漂移；`--require-independent-source` 仍会失败，直到接入 CarSim/CarMaker、公开基准或实测数据
- 独立 reference 接入：`scripts/scaffold_reference_benchmark.py` 默认把 CarSim/CarMaker、台架、缩比车或实车 benchmark 模板生成到 `validation_data/.incoming/`，不会被正式 checker 误当成证据；`scripts/normalize_reference_csv.py` 可把外部导出的原始 CSV 列和单位归一到标准 `reference.csv`，并支持可复现时间窗裁剪与时间归零；`scripts/suggest_reference_metrics.py` 可生成待审 `manifest.metrics` 候选片段，但会保留容差/理由 TODO，不会生成来源结论或让 benchmark 自动具备证据资格；`scripts/suggest_reference_artifacts.py` 可生成带 `path`、`role` 与 `sha256` 的 `manifest.source_artifacts` 候选片段，方便把原始/导出/测量/报告文件登记清晰化；独立来源还必须填写 `manifest.provenance` 并在 `manifest.source_artifacts` 登记原始/导出/测量/报告文件的 SHA-256，checker 会拒绝缺失 provenance、缺失 artifact、checksum 不匹配或用生成件冒充原始证据；补齐真实数据、metrics、notes 并清理占位符后，用 `scripts/promote_reference_benchmark.py` 校验并移入正式 `validation_data/<benchmark_id>/`
- 交付材料：`scripts/check_release_assets.py` 默认检查 README、`docs/user_manual.html`、30 个手册截图/GIF、关键研究报告和打包脚本；发布收尾可加 `--require-portable-zips` 检查当前版本 macOS/Windows 便携 zip
- v1 readiness：`docs/reports/v1_readiness.md` 当前结论为 `NOT READY`，唯一 strict blocker 是缺少通过检查的 `external_tool`、`bench`、`scaled_vehicle` 或 `full_vehicle` benchmark；pre-release 会检查该报告是否与当前证据一致
- 前端：type-check 通过
- 浏览器 smoke：Playwright Chromium `50 passed`，覆盖工作流渲染、试验到分析页交接、分析页通道切换/加图/hover cursor/drag-to-zoom/PNG 导出/截图证据、分析页 run 列表/数据读取/删除失败异常态、试验页实验库读取/保存/删除失败和机动模板读取失败异常态、原理页 demo 计算失败异常态、车辆几何拖拽、车辆页项目列表/加载/保存失败异常态、运行页模型/路面控制失败异常态、Python 策略状态读取和手动重载失败异常态、数据录制开始/停止/CSV 导出/状态读取失败/开始失败/停止失败/导出失败异常态、分析回放时间轴、回放 meta 读取失败 fallback、场景路径/扰动/故障 workflow、路径/场景版本刷新失败异常态、场景列表/加载/清除失败、路径模板读取失败、扰动更新/删除/清空失败和 2D 画布放置/拖动失败异常态、故障列表读取失败和添加/切换/删除/清空失败异常态、脚本库读取/解析/状态读取/启动/停止失败异常态、负载页车型库/参数读取失败、车型载入/保存/应用失败和敏感度扫描失败异常态、命令面板导航/模型切换失败异常态、手柄配置编辑
- 前端生产构建通过且无 Vite chunk warning；默认 `index` chunk 355.05 kB，低于 500 kB 入口预算；懒加载 3D vendor 最大 chunk `vendor-three-core` 666.67 kB，低于 700 kB 3D core 预算；入口 `index.css` 为 15.81 kB / gzip 3.47 kB，车辆几何、工作流页面、负载页、模型页和共享 load 图表/控制样式拆为独立 CSS chunks，App 壳层、视口/HUD、命令面板、手柄配置、Panel 壳层、Panel 内容控件和快速开始卡片样式已收敛到组件私有 CSS；全局 `styles.css` 仅保留主题 token 与 reset

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
