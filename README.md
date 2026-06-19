# 4WIS Simulator

四轮独立转向（4-Wheel Independent Steering）仿真工具。当前版本以 `docs/vehicle_model_refactor_plan.md` 中的整车底座重构为主线，面向四轮独立转向预研项目的运动学、动力学和协同控制策略研究。

跨平台运行（Windows / macOS / Linux），前后端分离架构。默认车辆标定为智己 LS9。

## 功能一览

- 三档模型：运动学 / 简化动力学 / 多体动力学（14 DOF，含侧倾、俯仰、悬架、bump-steer）
- 轮胎模型：线性 + 摩擦圆 / Pacejka（简化 Magic Formula）
- 控制策略：传统/理想阿克曼、后轮转向、蟹行、零半径、轨迹跟踪，可插 FMU/MATLAB
- 2D + 3D 可视化：可切换且保持视角；3D 车身随侧倾/俯仰倾斜；深色/浅色主题
- 每轮转向中心：整车瞬心在各轮垂线上的投影点 + 偏差实时显示与曲线记录
- 轨迹编辑：标准工况一键生成，或画布点击放航点
- 路面与扰动：基础 μ 预设 + 微调、对开路面、冰面、减速带、斜坡；GUI 放置/拖动/编辑扰动
- 参数编辑：车辆几何、质量惯量、悬架几何、弹簧阻尼、轮胎、伺服全部界面可调，带物理边界校验，随项目保存
- 录制 / 对比：CSV 导出、A/B 双跑轨迹 + 曲线对比
- 输入：键盘、YAML 动作脚本
- 便携一键启动包：内嵌 Python，双击即用

## 架构概览

```text
4WIS Simulator/
├── backend/        Python 仿真后端（FastAPI + WebSocket）
├── frontend/       Web 前端（React + TypeScript + Vite）
├── scripts/        启动脚本和工具
├── projects/       用户项目文件（YAML）
└── docs/           设计文档
```

### 技术栈

| 层 | 技术 |
|----|------|
| 前端 | React 18 · TypeScript · Vite · Konva（2D）· Three.js / R3F（3D）· uPlot（曲线）· Zustand |
| 后端 | Python 3.10+ · FastAPI · WebSocket · NumPy · pydantic v2 · PyYAML |
| 通信 | WebSocket（实时状态推送）· REST（项目/参数管理） |

### 仿真核心扩展点

- `VehicleModel`：抽象基类，已实现 `KinematicModel`、`SimplifiedDynamicModel`、`MultiBodyModel`
- `ControllerStrategy`：抽象基类，内置阿克曼、理想阿克曼、后轮转向、蟹行、零半径、轨迹跟踪
- 控制策略对外接口：Python 插件目录、FMU、MATLAB Engine、外部 socket

## 快速开始

### 前置依赖

- Python 3.10+
- Node.js 18+

### 一键启动

```bash
python scripts/start.py
```

### 手动启动

后端：

```bash
cd backend
pip install -e ".[dev]"
uvicorn sim4wis.main:app --reload --port 8010
```

前端：

```bash
cd frontend
npm install
npm run dev
```

浏览器打开 `http://localhost:5173`

### 便携版

```bash
python scripts/build_portable.py --targets macos-arm64 windows-x64
```

生成的便携包位于 `dist_portable/`，解压后双击 `start.command`（Mac）或 `start.bat`（Windows）即可运行。

## 当前进度

### Phase 1 - 已完成

- 工程骨架
- 仿真核心与基础控制策略
- WebSocket 状态推送
- 2D 可视化
- 键盘输入 + 实时曲线 + 参数面板
- YAML 项目保存/加载

### Phase 2 - 已完成

- 数据录制 + CSV 导出
- μ 类路面扰动 + 可视化
- YAML 动作序列脚本
- SimplifiedDynamicModel
- 悬架几何 → 真实转向阻力矩
- SpeedBump / Slope 扰动
- Simulink FMU 适配器
- MATLAB Engine 适配器
- 集成测试与完整文档

### Phase 3 - 已完成

- 3D 可视化
- 轨迹编辑器 + pure-pursuit 跟踪策略
- 多体动力学模型（14 DOF）
- 集成测试改进、LS9 标定、便携打包

## 验证

```bash
python3 scripts/smoke_test.py
cd backend && python3 -m pytest tests/
```

详细说明见：

- [docs/user_guide.md](docs/user_guide.md)
- [docs/design.md](docs/design.md)
- [docs/vehicle_model_refactor_plan.md](docs/vehicle_model_refactor_plan.md)
- [docs/tire_model.md](docs/tire_model.md)
- [docs/fmu_integration.md](docs/fmu_integration.md)
- [docs/disturbances.md](docs/disturbances.md)
- [docs/scripting.md](docs/scripting.md)
- [docs/phase3_review.md](docs/phase3_review.md)
- [CHANGELOG.md](CHANGELOG.md)
