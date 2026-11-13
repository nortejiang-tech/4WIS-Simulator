# 原生仿真内核预览版

这是 v0.105.0 的独立 C++17 计算内核分支，应用版本为 v0.106.0。
界面、MCP、故障编排、项目与结果格式继续兼容。独立预览包默认自动使用 C++。

## 安装与运行

便携 ZIP 无需开发工具。Mac 解压后运行 `start.command`，Windows 运行 `start.bat`。
Agent 仍使用 `agent_mcp.command` / `agent_mcp.bat`。先启动应用服务，再连接 MCP。
`GET /api/agent/capabilities` 的 `engine.native_available` 应为 true；仿真后的
Agent 观察或实验结果 `meta.engine.body_core` 应为 `cpp17`。
运动学模型不使用动力学 RK4；自定义 Python 物理扩展可保留 Python 路径。

从源码构建：需要 Python 3.10+ 和 C++17 编译器。Mac 使用 Xcode Command Line Tools；
Windows 使用 Visual Studio C++ Build Tools。执行 `python -m pip install -e './backend[dev]'`。
不需要额外数值引擎或 Rust 工具链。二进制须匹配 Python 小版本、操作系统和 CPU 架构。
只运行参照实现时，构建前设置 `SIM4WIS_BUILD_NATIVE=0`。

## 选择与回退

`SIM4WIS_ENGINE=auto` 默认优先已安装原生库；没有库时明确记录 Python。
`SIM4WIS_ENGINE=native` 要求原生库可用，缺失时报错。
`SIM4WIS_ENGINE=python` 强制原实现，不修改项目文件。
Mac 可在终端执行 `SIM4WIS_ENGINE=python ./start.command`；Windows 在同一终端先执行
`set SIM4WIS_ENGINE=python`，再运行 `start.bat`。删除环境变量或设为 `auto` 可恢复。
运行中的数值失败不会通过静默切换内核重试掩盖。

## 复现验收

`python -m pytest backend/tests/test_native_core.py -q` 验证数值迁移；
`python -m pytest backend/tests -n 4 -q` 执行完整后端回归。
`python scripts/benchmark_native_core.py --repeat 5 --duration 14 --output benchmark.json`
交替测量完整实验（包含记录），输出各次耗时、通道差异与精确故障时间线。
测量期间不应并行运行其他仿真负载。

便携包使用 Python 3.12：先在目标平台用 Python 3.12 构建 wheel，
`python -m pip wheel ./backend --no-deps -w dist_portable/wheels`，再向
`scripts/build_portable.py --targets <target> --native-wheel <wheel路径>` 提供对应 wheel。
构建器核对版本、ABI、CPU、源代码指纹，避免将 Mac 库误装入 Windows 包。
`scripts/check_portable_package.py --targets <target> --native-smoke --require-native-core --json`
从解压后的嵌入 Python 检查真实启动、C++ 仿真和 MCP。

两种内核的一致性是软件迁移证据。它不能代替轮胎/机构标定、台架/实车验证，也不改变
原模型整体一阶分步积分、独立轮端故障及观测型传感器故障的适用边界。
