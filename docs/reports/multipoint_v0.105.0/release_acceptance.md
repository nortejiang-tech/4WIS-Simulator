# v0.105.0 发布验收记录

基线为 v0.104.0 / d810968e0e3f0ce407b37e5fc8b92b09f6e661c3；日期 2026-11-13。
本文件记录本轮实际结果，不将合成仿真当成实车相关性。

## 当前证据

- 故障、Agent、物理审计与转向层聚焦回归：147 passed。
- 真实 MCP stdio 端到端：8 passed（最终完整发布门禁会再次覆盖）。
- 故障 GUI 生产浏览器流程：4 passed。使用已安装 Chromium；测试绑定隔离本机端口，单 worker 驱动共享实时仿真。
- 两模型、8 个14秒样例，12 组步长敏感性：全部满足报告中本例数值门槛。
- 中心区四步长比较：数值与残余敏感性详见 `numerical_review.md`；黄金迁移保持原容差与分类。
- 全量发布门禁：**PASS**。后端 1086 passed（2 个上游弃用警告，289.12 s）；32/32 smoke；6/6 黄金；2 个解析参考 / 15 指标；前端类型检查、54 单测、生产构建、58 浏览器 E2E 全通过。
- 全量收集后补充数值失败暂停保护测试；最终故障/物理集 99 passed，新增 1 项。没有重复计数为额外 99 个新用例。
- Mac 原生便携包：**PASS，41 项**，含解压后的嵌入式 Python/后端/前端、全部后端源文件哈希、真实 MCP 双模型同时/错时故障和导出。首次发现测试子进程未继承包内 PYTHONPATH，修复测试启动环境后通过。
- Windows 原生 CI：**PASS**，199 项故障/物理/Agent 测试（37.93 s）与 45 项便携包检查通过；嵌入式后端启动、Web UI、真实 MCP 双模型事件及导出均已执行。候选源码 `6ee61cea688c3c20271b2852462dbcba63cd34c1`，证据见 [Windows run 43453353932](https://github.com/nortejiang-tech/4WIS-Simulator/actions/runs/43453353932) 和 `windows_native_checks.json`。
- 本记录加入源码后，正式标签再次触发 Windows 原生构建。最终标签提交、正式构建 run 与双平台资产 SHA-256 以 [v0.105.0 Release](https://github.com/nortejiang-tech/4WIS-Simulator/releases/tag/v0.105.0) 的 `release-verification.json` / `SHA256SUMS` 为准；上传和回读结果不提前写成通过。
- 原始完整门禁输出经本地路径脱敏保存在 `local-validation.txt`；结构化汇总为 `local_validation.json`；Mac 包证据为 `mac_portable_checks.json`。

## 范围与回滚

UI、REST、MCP 的同时/错时/恢复、互斥配置、错误提示和用户参数保真均纳入验收。MCP 包内测试使用真实子进程，分别在两种受力模型中执行非网格同时/错时事件并导出观测偏差。

未运行台架、实车、CAN 或独立商业模型对标；这些为 PENDING。原版本标签与便携包保留；未执行回滚。用户原有未跟踪目录保留，不纳入版本提交。

EVO-Coder 调用与 Planner 返工见 `../evo-coder/release-v0.105.0.md` 及 JSON；Token 用量与云端节省无账本数据，未估造。
