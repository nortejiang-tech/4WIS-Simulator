# C++ 正式路线与维护交接

日期：2026-11-14。用户完成对比并选定 B。**main 与唯一正式发布线收敛到 v0.107.0 全 C++ 后端。** A 保留作每轮维护升级到全 C++ 的中间移植状态，不再独立发布。两条路线共用逐字节相同的 backend/native/core.hpp，外层 5 ms 与内部最大 1.25 ms 步长相同。决策见 [ADR](adr/ADR-2026-11-14-cpp-canonical-release.md)。

|路线|分支|发布入口|默认端口|启动文件|
|---|---|---|---|---|
|A：C++ 内核迁移桥（不单独发布）|[codex/native-simulation-core](https://github.com/nortejiang-tech/4WIS-Simulator/tree/codex/native-simulation-core)|[历史迁移包](https://github.com/nortejiang-tech/4WIS-Simulator/releases/tag/native-core-v0.106.0-preview.1)|8010|仅用于过渡验证|
|B：唯一正式全 C++ 后端|[main / codex/cpp-full-backend](https://github.com/nortejiang-tech/4WIS-Simulator/tree/main)|[v0.107.0 正式 Release](https://github.com/nortejiang-tech/4WIS-Simulator/releases/tag/v0.107.0)|8020|Mac Start.command；Windows Start.bat|

发布包均为 macOS arm64 / Windows x64 的独立 ZIP。已有开发 Python/Node 不是运行前提。B 内置功能不需要 Python；随包 Python 仅用于外部用户插件。两者均保留相同 React 浏览器界面，B 不代表原生 C++ 控件界面的重写。没有 Apple Developer ID 公证或 Windows Authenticode 签名验收；实际目标机的系统启动确认与 GUI 验收仍需本机进行。B C++ 可执行文件部署目标为 macOS 13.3，实际只在 macOS 27.2.1 验收，不能把编译下限当成所有系统版本通过。

## 已完成的路线对照

同机测试已包含 8 个 14 s 工况各预热一次并交替运行五次、健康/同时故障/错时故障分析、逐通道数值核对、接口与 MCP、多平台解压包检查。用户已根据该证据选择 B；历史测量口径和边界继续保存在报告中。

完整样例、原始耗时和图表见 [三路线分析报告](https://github.com/nortejiang-tech/4WIS-Simulator/blob/main/docs/reports/cpp_v0.107.0/comparison.md)。正式 Release 附可离线打开的 HTML 与 evidence.zip。A 的历史内核对照在 [A 分支](https://github.com/nortejiang-tech/4WIS-Simulator/tree/codex/native-simulation-core)。

## 已测差别及维护代价

同一台 Apple M5 Pro / 64 GiB，8 个 14 s 工况各预热后交替 5 次：A 相对 Python 为 2.08–3.17×，B 为 40.82–50.75×，B 相对 A 为 14.90–20.69×。B 的计时还包括新进程启动、KPI 与完整 JSON 写出/读取；这些是完整实验耗时，不是单函数或实时频率提升。全部记录通道对照通过，x/y 最大分量差约 3.91e-13 m，是软件一致性，不是车辆精度。

A 改动面小，保留 Python 服务、工具和维护路径；B 将循环、控制、分析、服务及 MCP 全部迁入 C++，批量吞吐收益更大，但跨语言维护和编译调试成本也更大。用户自定义 FMU/MATLAB 模型、引擎和许可仍是外部前提，兼容工作进程不是安全沙箱。

## 验收与已知边界

A：Mac / Windows 后端均 1122 项通过，54 项前端单测与 58 项浏览器流程通过；最终包 Mac 43 / Windows 49 项通过。Windows 最后补充了 MSVCP140.dll 本地部署及实际加载来源断言。B：Mac / Windows C++ 差分与接口均 238 项通过；54 项前端单测与 58 项浏览器流程通过；两平台包级各 10 项通过。两条路线完整验收记录、构建哈希和已修复失败均在各自 acceptance.md。

A 早先一轮 Windows 3D 周期性帧时序检查 FAIL，最终相同前端通过；异常原因未单独定位，目标机长期流畅性仍为 PENDING。模型是工程近似：总体分区积分、研究型多体、传感器故障只改变观测、选型单侧负载对称镜像。台架/实车与真实参数关联未做。物理检查发现并共同修复了选型四轮载荷混入、执行器坐标和跨 Python 版本时长预算问题，没有放宽原有物理/黄金判据。

## 后续维护路径

先在 A 桥接路线完成需要保留 Python 宿主的过渡实现和功能验证，再将相同变更移植到 B 并通过 C++ 差分、接口/MCP、双平台包与物理验收。只有 B 通过才发布新正式版本；不得从 A 单独打包成用户产品。A 的历史 preview 资产保留作追溯，不删除。遇到正式版回归，保留 v0.105.0 安装包作为上一个稳定回退点；不要强推重置 main。

没有用户数据格式迁移，没有执行用户数据回滚。实车、台架和用户目标机边界仍按验收记录执行。

EVO-Coder 账本：两路线各 0 调用；本地生成 0 tokens、可核实云端节省 0；主模型 tokens 不可获得。数值、ABI、运行时和发布由 Planner 负责，原始 Markdown/JSON 报告随版本提交。
