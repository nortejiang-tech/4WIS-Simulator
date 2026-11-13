# v0.105.0 EVO-Coder 调用报告

本轮调用 1 次：多点故障 UI 窄阶段，EVO-X2 `halogen-qwen3.8-flash-next`，唯一入口 `evo-coder`。动力学、架构、MCP 契约、风险判断和最终验收由 Planner 完成。详细字段及哈希见同名 JSON。

- run_id：`0d41e632-0373-424c-a1ae-3a2b5a09750a`；基线 `d810968e0e3f0ce407b37e5fc8b92b09f6e661c3`。
- 允许/可归因于模型的修改：`frontend/src/components/FaultPanel.tsx`。同一工作树按文件限域，未建立独立 checkout。
- 耗时 132.691 s；12 回合、14 工具调用；2 次验证尝试、1 次声明通过；停止原因 `toolUse`，结构化输出有效。
- 可核实设置：temperature=0、top_p=1、max_output_tokens=4096；记录最大上下文 17957。thinking=false 来自执行入口约定，不冒充账本实测。
- 监督器结果 FAIL，分类 `LOCAL_POSTWRITE_FAILURE` / `WRITE_SCOPE_VIOLATION`。监督器还看到了 Planner 同时写入的 `experiment/artifacts.py` 和 `test_multipoint_faults.py`；工具记录确认这些为 Planner 写入，不能据此说 Coder 越界。根因是本轮编排未隔离并行写入；没有调整 wrapper 或绕过信任门。
- Planner 结论：**部分接受并补改**。补回草稿数值/自由转向参数，增加有限值/范围校验，修正初始错误状态、标签与显示，经过实际浏览器验收。
- 监督器通过率 **0/1**；首次独立验收通过率 **0/1**；有用产出采纳率 **1/1**。这些是阶段计数，不是代码贡献比例。
- 生成 Token、输入 Token、总 Token、节省云端 Token：**unavailable / 未测量**。现有无敏感账本未提供用量；17957 是上下文观测上限，不能当本轮生成量或节省量。

本轮评价：能产出可用界面骨架，但参数保真与错误处理需要 Planner 返工。一次样本不足以评价长期能力；下次将编码阶段工作树与 Planner 写入隔离，并把用户输入往返保真作为明确验收条件。物理底座继续由 Planner 独立推导和测试。
