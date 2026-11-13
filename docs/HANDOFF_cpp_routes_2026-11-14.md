# 两条 C++ 候选路线交接与对比

日期：2026-11-14。用户要求分别保留两个独立分支，明天自行比较后决定路线。主分支与稳定 v0.105.0 不合并、不替换。两个路线共用逐字节相同的 backend/native/core.hpp，外层 5 ms 与内部最大 1.25 ms 步长相同。

|路线|分支|发布入口|默认端口|启动文件|
|---|---|---|---|---|
|A：C++ 仿真内核|[codex/native-simulation-core](https://github.com/nortejiang-tech/4WIS-Simulator/tree/codex/native-simulation-core)|[v0.106.0 preview](https://github.com/nortejiang-tech/4WIS-Simulator/releases/tag/native-core-v0.106.0-preview.1)|8010|Mac start.command；Windows start.bat|
|B：完整 C++ 后端|[codex/cpp-full-backend](https://github.com/nortejiang-tech/4WIS-Simulator/tree/codex/cpp-full-backend)|[v0.107.0 preview](https://github.com/nortejiang-tech/4WIS-Simulator/releases/tag/cpp-full-v0.107.0-preview.1)|8020|Mac Start.command；Windows Start.bat|

发布包均为 macOS arm64 / Windows x64 的独立 ZIP。已有开发 Python/Node 不是运行前提。B 内置功能不需要 Python；随包 Python 仅用于外部用户插件。两者均保留相同 React 浏览器界面，B 不代表原生 C++ 控件界面的重写。没有 Apple Developer ID 公证或 Windows Authenticode 签名验收；实际目标机的系统启动确认与 GUI 验收仍需本机进行。B C++ 可执行文件部署目标为 macOS 13.3，实际只在 macOS 27.2.1 验收，不能把编译下限当成所有系统版本通过。

## 建议的同机比较方法

1. 将 A、B 解压到两个独立目录。停止占用 8010/8020 的旧服务，分别启动两包，确认浏览器地址和引擎元数据对应当前目录。
2. 导入同一车辆配置和实验定义，用相同步长、记录频率、随机种子、控制器与故障时间线对比。先健康参考，再双前轮同时卡滞，最后 FL 卡滞 + RR 自由转向 + 观测偏差/噪声/掉线的错时组合。
3. 重点比较日常操作：3D 视图、实时控制、暂停/恢复、CSV/JSON 导出、载荷与选型、研究批跑、MCP 初始化和工具调用，以及你实际使用的用户插件。
4. 在同一台电脑上交替执行两路线各五次，比较耗时中位数和结果通道；不要把两个服务同时重负载运行的耗时当成单服务性能。
5. MCP 启动入口 A 为 agent_mcp.command / agent_mcp.bat，B 为 MCP.command / MCP.bat；--print-config 输出对应配置。两条 Agent 调用都覆盖多点同时/错时、实际事件时间线与回读。

完整样例、原始耗时和图表见 [三路线分析报告](https://github.com/nortejiang-tech/4WIS-Simulator/blob/codex/cpp-full-backend/docs/reports/cpp_v0.107.0/comparison.md)。Release 另附可离线打开的 HTML 与 evidence.zip。A 的内核对照在同分支 docs/reports/native_v0.106.0/analysis.html。

## 已测差别及维护代价

同一台 Apple M5 Pro / 64 GiB，8 个 14 s 工况各预热后交替 5 次：A 相对 Python 为 2.08–3.17×，B 为 40.82–50.75×，B 相对 A 为 14.90–20.69×。B 的计时还包括新进程启动、KPI 与完整 JSON 写出/读取；这些是完整实验耗时，不是单函数或实时频率提升。全部记录通道对照通过，x/y 最大分量差约 3.91e-13 m，是软件一致性，不是车辆精度。

A 改动面小，保留 Python 服务、工具和维护路径；B 将循环、控制、分析、服务及 MCP 全部迁入 C++，批量吞吐收益更大，但跨语言维护和编译调试成本也更大。用户自定义 FMU/MATLAB 模型、引擎和许可仍是外部前提，兼容工作进程不是安全沙箱。

## 验收与已知边界

A：Mac / Windows 后端均 1122 项通过，54 项前端单测与 58 项浏览器流程通过；最终包 Mac 43 / Windows 49 项通过。Windows 最后补充了 MSVCP140.dll 本地部署及实际加载来源断言。B：Mac / Windows C++ 差分与接口均 238 项通过；54 项前端单测与 58 项浏览器流程通过；两平台包级各 10 项通过。两条路线完整验收记录、构建哈希和已修复失败均在各自 acceptance.md。

A 早先一轮 Windows 3D 周期性帧时序检查 FAIL，最终相同前端通过；异常原因未单独定位，目标机长期流畅性仍为 PENDING。模型是工程近似：总体分区积分、研究型多体、传感器故障只改变观测、选型单侧负载对称镜像。台架/实车与真实参数关联未做。物理检查发现并共同修复了选型四轮载荷混入、执行器坐标和跨 Python 版本时长预算问题，没有放宽原有物理/黄金判据。

## 回退和后续决策

停止当前候选并从另一个独立目录启动即可切换。A 还保留 SIM4WIS_ENGINE=python 的参考引擎。没有用户数据格式迁移，没有执行用户数据回滚。待用户选定后再决定主线合并、正式版本和后续物理标定；本次不替用户做路线选择。

EVO-Coder 账本：两路线各 0 调用；本地生成 0 tokens、可核实云端节省 0；主模型 tokens 不可获得。数值、ABI、运行时和发布由 Planner 负责，原始 Markdown/JSON 报告随版本提交。
