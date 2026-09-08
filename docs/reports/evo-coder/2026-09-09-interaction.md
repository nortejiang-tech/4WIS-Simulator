# EVO-Coder 调用报告：第二轮交互接口与界面重构

本任务实际调用 **1 次**。监督器返回 PASS / CODER_DONE_PENDING_REVIEW；独立审阅结论为**部分接受，由 ASTRA 补改后集成**。不能把结构化执行通过当作首次质量验收通过。

## 贡献与路由

隔离实现无业务状态的三模式导航栏 InteractionModeBar.tsx/css。 采用导航栏结构、模式回调、aria-pressed、连接/停止控件与基础样式。

Agent 生命周期/步进幂等、共享输入仲裁、脚本时钟、遥测/导出、MCP、其余工作台与集成由 ASTRA 完成。高耦合状态控制和数据边界直接由 Planner 实施；未调用 Spark/Luna fallback。

基线：隔离目录 /tmp/4wis-interaction-coder，仅有已审查编译配置及已安装依赖链接。主项目基线为上一轮所有未提交物理修正；本轮快照 /tmp/4wis-interaction-review/baseline-manifest.json。

## 可复查调用记录

| 字段 | 记录 |
|---|---|
| run_id | `3fbe957f-326c-4d3f-a016-e75867cda06a` |
| 开始时间 | 2026-09-08T23:28:39.847000+08:00 |
| 模型 / provider | `qwen3-coder-next-q5-k-m` / `evo-coder` |
| 工作目录 | `/private/tmp/4wis-interaction-coder` |
| temperature / top_p | 0.0 / 1 |
| 单次输出上限 | 4096 tokens |
| 墙钟耗时 | 49.862 s |
| 回合 / 工具调用 | 7 / 10 |
| verify 尝试 / 最终通过 / 声明数 | 2 / 1 / 1 |
| 最大上下文 | 5493 tokens（不是总消耗） |
| 终止 | stop_reason=toolUse，structured_terminal=True |
| 监督器故障 / 越界 | None / 0 |
| 提示词 SHA-256 | `5a71c0af14bc57fe972340937b1a3543117c82a7ada2f0ce70331ee8ef8fca81` |

精确写入白名单：`["src/InteractionModeBar.tsx", "src/InteractionModeBar.css"]`。实际变更：`["src/InteractionModeBar.css", "src/InteractionModeBar.tsx"]`。

声明的验证：

- `./node_modules/.bin/tsc --noEmit`

原始提示词由本任务工具调用记录恢复，字节哈希与账本一致，见 [2026-09-09-interaction.prompt.txt](2026-09-09-interaction.prompt.txt)。结构化账本与独立评审字段见 [2026-09-09-interaction.json](2026-09-09-interaction.json)。账本来源为 `/Users/nortepro/.local/state/evo-coder/runs.jsonl` 第 95 行。

## 独立验收与返工

- props 虽声明 sourceLabel，实际未解构或显示；Planner 补入控制来源读数。
- 遗漏提示词明确要求的 CSS import；Planner 补入。
- 集成后 Planner 调整换行/间距，并修正宿主 grid 的窄屏裁切和脚本视图区高度。宿主布局问题不归因于 Coder。

最终声明的 tsc 检查通过；Planner 独立再次编译并审阅语义后认定需补改。补改集成后 57 项完整浏览器回归、54 项前端单元、类型与生产构建通过。后端/MCP 的整体验收见本轮 STAGE，不能计作 Coder 自己完成的验证。

Coder 返回值保留原样；Planner 质量分支记为 LOCAL_POSTWRITE_FAILURE，保留差异并由 Planner 接管，没有用另一模型自动覆盖。本轮 verify 执行两次、最终一项声明验证通过；账本没有首次失败的详细诊断，不推断具体编译错误。

## 接受率定义

| 指标 | 分子 / 分母 | 本任务结果 |
|---|---|---|
| 监督器通过率 | 结构化 PASS 调用 / 全部调用 | 1 / 1 = 100% |
| 首次独立验收直接接受率 | 无需补改直接接受 / 已独立审阅调用 | 0 / 1 = 0% |
| 有用产出采纳率 | 至少部分采用 / 已独立审阅调用 | 1 / 1 = 100% |
| 部分接受 / 拒绝 / 待审 | 调用数 | 1 / 0 / 0 |

这三个指标回答不同问题，不能合并称为“接受率 100%”。这是一个窄 UI 子任务的单样本，不能外推模型整体能力，也不代表全任务代码贡献率。没有记录总输入/输出 tokens、首 token 延迟、吞吐和 Planner 返工耗时；这些字段标为 unavailable，不估造。

## 对下一轮调用方案的启示

先补强可验收条件：编译之外，检查必须显示的 props、CSS 接入、坐标/尺寸物理语义和实际页面布局。两次样本都表现为“编译通过仍遗漏明确需求”，现有证据不足以归因到 temperature、模型容量或上下文上限。建议积累同类受控任务，再比较提示词/验收方案；本次没有改生产 wrapper、模型参数、路由或预算。

配置指纹：

- extension: `e5f3ffbfcd3ad230ea950fe65e649c798c1059a61c5d7a256cd2f478557753e3`
- models: `b45d46ad9d485255d974733d3e458207729626b045ac2f26d25d3c91fb5e165b`
- policy: `03ba61a71edc5ea35402a52ea66340c0d91a5c9e93ccfc09d66886bdf68c6218`
- supervisor: `3f7cf4908814222f3f40ce599f3b98af5b3068119e385ab1841809b05e81283e`
